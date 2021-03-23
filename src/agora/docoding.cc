/**
 * @file docoding.cc
 * @brief Implmentation file for the Docoding class.  Includes the DoEncode and
 * DoDecode classes.
 */
#include "docoding.h"

#include "concurrent_queue_wrapper.h"
#include "encoder.h"
#include "scrambler.h"
#include "udp_server.h"
#include "utils_ldpc.h"

static constexpr bool kPrintEncodedData = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

/// The size of the kernel-level socket receive buffer,
/// used for receiving responses from the remote LDPC worker.
static constexpr size_t kRxBufSize = 4 * 1024 * 1024;
/// The factor used to determine when a remote LDPC worker is considered to be
/// falling behind and is in an "unhealthy" state.
static constexpr size_t thresholdPendingRequestsFactor = 10;

DoEncode::DoEncode(Config* in_config, int in_tid,
                   Table<int8_t>& in_raw_data_buffer,
                   Table<int8_t>& in_encoded_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      raw_data_buffer_(in_raw_data_buffer),
      encoded_buffer_(in_encoded_buffer),
      scrambler_(std::make_unique<AgoraScrambler::Scrambler>()) {
  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kEncode, in_tid);
  parity_buffer_ = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      LdpcEncodingParityBufSize(cfg_->LdpcConfig().BaseGraph(),
                                cfg_->LdpcConfig().ExpansionFactor())));
  assert(parity_buffer_ != nullptr);
  encoded_buffer_temp_ = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      LdpcEncodingEncodedBufSize(cfg_->LdpcConfig().BaseGraph(),
                                 cfg_->LdpcConfig().ExpansionFactor())));

  scrambler_buffer_ =
      new int8_t[cfg_->NumBytesPerCb() +
                 kLdpcHelperFunctionInputBufferSizePaddingBytes]();
  assert(encoded_buffer_temp_ != nullptr);
}

DoEncode::~DoEncode() {
  std::free(parity_buffer_);
  std::free(encoded_buffer_temp_);
  delete[] scrambler_buffer_;
}

EventData DoEncode::Launch(size_t tag) {
  const LDPCconfig& ldpc_config = cfg_->LdpcConfig();
  size_t frame_id = gen_tag_t(tag).frame_id_;
  size_t symbol_id = gen_tag_t(tag).symbol_id_;
  size_t cb_id = gen_tag_t(tag).cb_id_;
  size_t cur_cb_id = cb_id % cfg_->LdpcConfig().NumBlocksInSymbol();
  size_t ue_id = cb_id / cfg_->LdpcConfig().NumBlocksInSymbol();
  if (kDebugPrintInTask) {
    std::printf(
        "In doEncode thread %d: frame: %zu, symbol: %zu, code block %zu, "
        "ue_id: %zu\n",
        tid_, frame_id, symbol_id, cur_cb_id, ue_id);
  }

  size_t start_tsc = GetTime::WorkerRdtsc();
  size_t symbol_idx_dl = cfg_->Frame().GetDLSymbolIdx(symbol_id);
  int8_t* ldpc_input = nullptr;

  if (this->cfg_->ScrambleEnabled()) {
    std::memcpy(
        scrambler_buffer_,
        cfg_->GetInfoBits(raw_data_buffer_, symbol_idx_dl, ue_id, cur_cb_id),
        cfg_->NumBytesPerCb());
    scrambler_->Scramble(scrambler_buffer_, cfg_->NumBytesPerCb());
    ldpc_input = scrambler_buffer_;
  } else {
    ldpc_input =
        cfg_->GetInfoBits(raw_data_buffer_, symbol_idx_dl, ue_id, cur_cb_id);
  }

  LdpcEncodeHelper(ldpc_config.BaseGraph(), ldpc_config.ExpansionFactor(),
                   ldpc_config.NumRows(), encoded_buffer_temp_, parity_buffer_,
                   ldpc_input);
  int8_t* final_output_ptr = cfg_->GetEncodedBuf(
      encoded_buffer_, frame_id, symbol_idx_dl, ue_id, cur_cb_id);
  AdaptBitsForMod(reinterpret_cast<uint8_t*>(encoded_buffer_temp_),
                  reinterpret_cast<uint8_t*>(final_output_ptr),
                  BitsToBytes(ldpc_config.NumCbCodewLen()),
                  cfg_->ModOrderBits());

  if (kPrintEncodedData == true) {
    std::printf("Encoded data\n");
    int num_mod = cfg_->LdpcConfig().NumCbCodewLen() / cfg_->ModOrderBits();
    for (int i = 0; i < num_mod; i++) {
      std::printf("%u ", *(final_output_ptr + i));
    }
    std::printf("\n");
  }

  size_t duration = GetTime::WorkerRdtsc() - start_tsc;
  duration_stat_->task_duration_[0] += duration;
  duration_stat_->task_count_++;
  if (GetTime::CyclesToUs(duration, cfg_->FreqGhz()) > 500) {
    std::printf("Thread %d Encode takes %.2f\n", tid_,
                GetTime::CyclesToUs(duration, cfg_->FreqGhz()));
  }
  return EventData(EventType::kEncode, tag);
}

DoDecode::DoDecode(
    Config* in_config, int in_tid,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
    PhyStats* in_phy_stats, Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      demod_buffers_(demod_buffers),
      decoded_buffers_(decoded_buffers),
      phy_stats_(in_phy_stats),
      scrambler_(std::make_unique<AgoraScrambler::Scrambler>()) {
  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kDecode, in_tid);
  resp_var_nodes_ = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, 1024 * 1024 * sizeof(int16_t)));
}

DoDecode::~DoDecode() {
  std::free(resp_var_nodes_);
  // NOTE: currently, the remote_ldpc_stub_ pointer may be shared with other
  // DoDecode instances. Therefore, we cannot free it here, but rather
  // we can only free it safely from within the worker thread.
}

EventData DoDecode::Launch(size_t tag) {
  const LDPCconfig& ldpc_config = cfg_->LdpcConfig();
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t symbol_idx_ul = cfg_->Frame().GetULSymbolIdx(symbol_id);
  const size_t cb_id = gen_tag_t(tag).cb_id_;
  const size_t symbol_offset =
      cfg_->GetTotalDataSymbolIdxUl(frame_id, symbol_idx_ul);
  const size_t cur_cb_id = (cb_id % cfg_->LdpcConfig().NumBlocksInSymbol());
  const size_t ue_id = (cb_id / cfg_->LdpcConfig().NumBlocksInSymbol());
  const size_t frame_slot = (frame_id % kFrameWnd);
  if (kDebugPrintInTask == true) {
    std::printf(
        "In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
        "%zu, ue: %zu offset %zu\n",
        tid_, frame_id, symbol_id, cur_cb_id, ue_id, symbol_offset);
  }

  // `llr_buffer_ptr` is the input buffer into the decoder.
  int8_t* llr_buffer_ptr =
      demod_buffers_[frame_slot][symbol_idx_ul][ue_id] +
      (cfg_->ModOrderBits() * (cfg_->LdpcConfig().NumCbCodewLen() * cur_cb_id));

  if (kPrintLLRData) {
    printf("LLR data, symbol_offset: %zu\n", symbol_offset);
    for (size_t i = 0; i < cfg_->LdpcConfig().NumCbCodewLen(); i++) {
      printf("%d ", *(llr_buffer_ptr + i));
    }
    printf("\n");
  }

  // `decoded_buffer_ptr` is the output buffer to which the decoded data
  // will be written.
  uint8_t* decoded_buffer_ptr =
      decoded_buffers_[frame_slot][symbol_idx_ul][ue_id] +
      (cur_cb_id * Roundup<64>(cfg_->NumBytesPerCb()));

  size_t start_tsc = GetTime::WorkerRdtsc();

  if (kUseRemote) {
    auto* decode_context =
        new DecodeContext(decoded_buffer_ptr, this, tid_, tag);
    size_t data_bytes = LdpcMaxNumEncodedBits(
        cfg_->LdpcConfig().BaseGraph(), cfg_->LdpcConfig().ExpansionFactor());
    size_t msg_len = DecodeMsg::SizeWithoutData() + data_bytes;
    uint16_t dest_port = cfg_->RemoteLdpcBasePort() + tid_;

#if defined(USE_DPDK)
    uint16_t src_port = 31850 - 1;  /// TODO: FIXME: use proper src port
    rte_mbuf* tx_mbuf = DpdkTransport::alloc_udp(
        remote_ldpc_stub_->mbuf_pool, remote_ldpc_stub_->local_mac_addr,
        remote_ldpc_stub_->remote_mac_addr, remote_ldpc_stub_->local_ip_addr,
        remote_ldpc_stub_->remote_ip_addr, src_port, dest_port, msg_len);
    // `msg_buf` points to where our payload starts in the DPDK `tx_mbuf`
    uint8_t* msg_buf = (rte_pktmbuf_mtod(tx_mbuf, uint8_t*) + kPayloadOffset);
#else
    // TODO: pre-allocate a pool of request buffers to avoid this
    //       temporary allocation on the critical data path.
    uint8_t* msg_buf = new uint8_t[msg_len];
#endif  // USE_DPDK

    auto* request = reinterpret_cast<DecodeMsg*>(msg_buf);
    request->context = decode_context;
    request->msg_id = remote_ldpc_stub_->num_requests_issued_;
    memcpy(request->data, reinterpret_cast<uint8_t*>(llr_buffer_ptr),
           data_bytes);

    printf("Docoding: Issuing request %zu, context: %p\n",
           remote_ldpc_stub_->num_requests_issued_, request->context);
    remote_ldpc_stub_->num_requests_issued_++;

#if defined(USE_DPDK)
    rt_assert(rte_eth_tx_burst(0, tid, &tx_mbuf, 1) == 1,
              "rte_eth_tx_burst() failed");
#else
    remote_ldpc_stub_->udp_client_.Send(cfg_->RemoteLdpcIpAddr(), dest_port,
                                        msg_buf, msg_len);
    delete[] msg_buf;  // TODO: remove this once we use request buf pools
#endif  // USE_DPDK

  } else {
    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {};
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {};

    // Decoder setup
    int16_t num_filler_bits = 0;
    int16_t num_channel_llrs = ldpc_config.NumCbCodewLen();

    ldpc_decoder_5gnr_request.numChannelLlrs = num_channel_llrs;
    ldpc_decoder_5gnr_request.numFillerBits = num_filler_bits;
    ldpc_decoder_5gnr_request.maxIterations = ldpc_config.MaxDecoderIter();
    ldpc_decoder_5gnr_request.enableEarlyTermination =
        ldpc_config.EarlyTermination();
    ldpc_decoder_5gnr_request.Zc = ldpc_config.ExpansionFactor();
    ldpc_decoder_5gnr_request.baseGraph = ldpc_config.BaseGraph();
    ldpc_decoder_5gnr_request.nRows = ldpc_config.NumRows();

    int num_msg_bits = ldpc_config.NumCbLen() - num_filler_bits;
    ldpc_decoder_5gnr_response.numMsgBits = num_msg_bits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes_;

    // int8_t* llr_buffer_ptr =
    //  int8_t* llr_buffer_ptr =
    //  demod_buffers_[frame_slot][symbol_idx_ul][ue_id]
    //      + (cfg->mod_order_bits * (LDPC_config.cbCodewLen * cur_cb_id));
    //  uint8_t* decoded_buffer_ptr
    //      = decoded_buffers_[frame_slot][symbol_idx_ul][ue_id]
    //      + (cur_cb_id * roundup<64>(cfg->num_bytes_per_cb));
    ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

    size_t start_tsc1 = GetTime::WorkerRdtsc();
    duration_stat_->task_duration_[1] += start_tsc1 - start_tsc;

    bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                            &ldpc_decoder_5gnr_response);

    if (cfg_->ScrambleEnabled()) {
      scrambler_->Descramble(decoded_buffer_ptr, cfg_->NumBytesPerCb());
    }

    size_t start_tsc2 = GetTime::WorkerRdtsc();
    duration_stat_->task_duration_[2] += start_tsc2 - start_tsc1;

    if (kPrintLLRData) {
      std::printf("LLR data, symbol_offset: %zu\n", symbol_offset);
      for (size_t i = 0; i < ldpc_config.NumCbCodewLen(); i++) {
        std::printf("%d ", *(llr_buffer_ptr + i));
      }
      std::printf("\n");
    }

    if (kPrintDecodedData) {
      std::printf("Decoded data\n");
      for (size_t i = 0; i < (ldpc_config.NumCbLen() >> 3); i++) {
        std::printf("%u ", *(decoded_buffer_ptr + i));
      }
      std::printf("\n");
    }

    if ((kEnableMac == false) && (kPrintPhyStats == true) &&
        (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols())) {
      phy_stats_->UpdateDecodedBits(ue_id, symbol_offset,
                                    cfg_->NumBytesPerCb() * 8);
      phy_stats_->IncrementDecodedBlocks(ue_id, symbol_offset);
      size_t block_error(0);
      for (size_t i = 0; i < cfg_->NumBytesPerCb(); i++) {
        uint8_t rx_byte = decoded_buffer_ptr[i];
        auto tx_byte = static_cast<uint8_t>(cfg_->GetInfoBits(
            cfg_->UlBits(), symbol_idx_ul, ue_id, cur_cb_id)[i]);
        phy_stats_->UpdateBitErrors(ue_id, symbol_offset, tx_byte, rx_byte);
        if (rx_byte != tx_byte) {
          block_error++;
        }
      }
      phy_stats_->UpdateBlockErrors(ue_id, symbol_offset, block_error);
    }

    size_t duration = GetTime::WorkerRdtsc() - start_tsc;
    duration_stat_->task_duration_[0] += duration;
    duration_stat_->task_count_++;
    if (GetTime::CyclesToUs(duration, cfg_->FreqGhz()) > 500) {
      std::printf("Thread %d Decode takes %.2f\n", tid_,
                  GetTime::CyclesToUs(duration, cfg_->FreqGhz()));
    }

    // When using remote LDPC, we ship the decoding task asynchronously to a
    // remote server and return immediately
    if (kUseRemote) {
      return EventData(EventType::kPendingToRemote, tag);
    } else {
      return EventData(EventType::kDecode, tag);
    }
  }

  /// This function should be run on its own thread to endlessly receive
  /// decode responses from remote LDPC workers and trigger the proper
  /// completion events expected by the rest of Agora. There should only be one
  /// thread instance of this function for the entire Agora process.
  void DecodeResponseLoop(Config * cfg) {
    UDPServer udp_server(cfg->remote_ldpc_completion_port, kRxBufSize);
    size_t decoded_bits =
        ldpc_encoding_input_buf_size(cfg->LDPC_config.Bg, cfg->LDPC_config.Zc);
    // The number of bytes that we receive at one time from the LDPC worker
    // is defined by the `DecodeMsg` struct.
    size_t rx_buf_len = DecodeMsg::size_without_data() + decoded_bits;
    uint8_t* rx_buf = new uint8_t[rx_buf_len];
    DecodeMsg* msg = reinterpret_cast<DecodeMsg*>(rx_buf);

    while (cfg->running) {
      ssize_t bytes_rcvd = udp_server.recv_nonblocking(rx_buf, rx_buf_len);
      if (bytes_rcvd == 0) {
        // no data received
        continue;
      } else if (bytes_rcvd == -1) {
        // There was a socket receive error
        cfg->running = false;
        break;
      }

      rt_assert(bytes_rcvd == (ssize_t)rx_buf_len,
                "Rcvd wrong decode response len");
      DecodeContext* context = msg->context;
      DoDecode* compute_decoding = context->doer;
      RtAssert(context->tid == compute_decoding->tid, "DoDecode tid mismatch");

      // Copy the decoded buffer received from the remote LDPC worker
      // into the appropriate location in the decode doer's decoded buffer.
      std::memcpy(context->output_ptr, msg->data, decoded_bits);

      EventData resp_event;
      resp_event.num_tags = 1;
      resp_event.tags[0] = context->tag;
      resp_event.event_type = EventType::kDecode;

      try_enqueue_fallback(&compute_decoding->complete_task_queue,
                           compute_decoding->worker_producer_token, resp_event);

      // TODO: here, return msg buffers to the pool

      // `rx_buf` will be reused to receive the next message,
      // so we only delete the DecodeContext that was allocated
      // by the DoDecode doer when the request was sent.
      delete context;

      printf("Docoding: Received response %zu\n", msg->msg_id);
      compute_decoding->remote_ldpc_stub_->num_responses_received++;

      // Optional: here we can check if we have received far fewer responses
      // than requests issued, and then issue a health warning.
      if ((compute_decoding->remote_ldpc_stub_->num_requests_issued -
           compute_decoding->remote_ldpc_stub_->num_responses_received) >
          (thresholdPendingRequestsFactor * cfg->UE_ANT_NUM)) {
        MLPD_WARN(
            "Some remote LDPC requests were lost or got no response. "
            "Requests: %zu, Responses: %zu, batch size: %zu\n",
            compute_decoding->remote_ldpc_stub_->num_requests_issued,
            compute_decoding->remote_ldpc_stub_->num_responses_received,
            cfg->UE_ANT_NUM);
      }
    }

    std::printf("Exiting decode_response_loop()\n");
    delete rx_buf;
  }
