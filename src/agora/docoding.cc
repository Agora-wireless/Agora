/**
 * @file docoding.cc
 * @brief Implmentation file for the Docoding class.  Includes the DoEncode and
 * DoDecode classes.
 */
#include "docoding.h"

#include "concurrent_queue_wrapper.h"
#include "encoder.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "scrambler.h"

static constexpr bool kPrintEncodedData = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

DoEncode::DoEncode(Config* in_config, int in_tid,
                   Table<int8_t>& in_raw_data_buffer,
                   Table<int8_t>& in_encoded_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      raw_data_buffer_(in_raw_data_buffer),
      encoded_buffer_(in_encoded_buffer) {
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
    Scrambler::WlanScramble(scrambler_buffer_, cfg_->NumBytesPerCb());
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
      phy_stats_(in_phy_stats) {
  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kDecode, in_tid);
  resp_var_nodes_ = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, 1024 * 1024 * sizeof(int16_t)));
}

DoDecode::~DoDecode() { std::free(resp_var_nodes_); }

EventData DoDecode::Launch(size_t tag) {
  const LDPCconfig& ldpc_config = cfg_->LdpcConfig();
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_idx_ul = gen_tag_t(tag).symbol_id_;
  const size_t cb_id = gen_tag_t(tag).cb_id_;
  const size_t symbol_offset =
      cfg_->GetTotalDataSymbolIdxUl(frame_id, symbol_idx_ul);
  const size_t cur_cb_id = (cb_id % cfg_->LdpcConfig().NumBlocksInSymbol());
  const size_t ue_id = (cb_id / cfg_->LdpcConfig().NumBlocksInSymbol());
  const size_t frame_slot = (frame_id % kFrameWnd);
  if (kDebugPrintInTask == true) {
    std::printf(
        "In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
        "%zu, ue: %zu\n",
        tid_, frame_id, symbol_idx_ul, cur_cb_id, ue_id);
  }

  size_t start_tsc = GetTime::WorkerRdtsc();

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

  int8_t* llr_buffer_ptr =
      demod_buffers_[frame_slot][symbol_idx_ul][ue_id] +
      (cfg_->ModOrderBits() * (ldpc_config.NumCbCodewLen() * cur_cb_id));

  uint8_t* decoded_buffer_ptr =
      decoded_buffers_[frame_slot][symbol_idx_ul][ue_id] +
      (cur_cb_id * Roundup<64>(cfg_->NumBytesPerCb()));

  ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
  ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

  size_t start_tsc1 = GetTime::WorkerRdtsc();
  duration_stat_->task_duration_[1] += start_tsc1 - start_tsc;

  bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                          &ldpc_decoder_5gnr_response);

  if (cfg_->ScrambleEnabled()) {
    Scrambler::WlanDescramble(decoded_buffer_ptr, cfg_->NumBytesPerCb());
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

  return EventData(EventType::kDecode, tag);
}
