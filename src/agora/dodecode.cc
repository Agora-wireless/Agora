/**
 * @file dodecode.cc
 * @brief Implmentation file for the DoDecode class. Currently, just supports
 * basestation
 */
#include "dodecode.h"

#include "concurrent_queue_wrapper.h"
#include "phy_ldpc_decoder_5gnr.h"

static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

static constexpr size_t kVarNodesSize = 1024 * 1024 * sizeof(int16_t);

DoDecode::DoDecode(
    Config* in_config, int in_tid,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffers,
    MacScheduler* mac_sched, PhyStats* in_phy_stats, Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      demod_buffers_(demod_buffers),
      decoded_buffers_(decoded_buffers),
      mac_sched_(mac_sched),
      phy_stats_(in_phy_stats),
      scrambler_(std::make_unique<AgoraScrambler::Scrambler>()) {
  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kDecode, in_tid);
  resp_var_nodes_ = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, kVarNodesSize));
}

DoDecode::~DoDecode() { std::free(resp_var_nodes_); }

EventData DoDecode::Launch(size_t tag) {
  const LDPCconfig& ldpc_config =
      mac_sched_->Params().LdpcConfig(Direction::kUplink);
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  size_t symbol_id, cb_id, cur_cb_id, ue_id, stream_id;
  size_t data_symbol_idx_ul = 0;
  size_t symbol_offset = 0;
  if (cfg_->SlotScheduling() == false) {
    symbol_id = gen_tag_t(tag).symbol_id_;
    cb_id = gen_tag_t(tag).cb_id_;
    cur_cb_id = (cb_id % ldpc_config.NumBlocksInSymbol());
    stream_id = (cb_id / ldpc_config.NumBlocksInSymbol());
    ue_id = mac_sched_->ScheduledUeIndex(frame_id, 0, stream_id);

    if (symbol_id <
        cfg_->Frame().GetULSymbol(cfg_->Frame().ClientUlPilotSymbols())) {
      return {EventType::kDecode, tag};
    }
    /*RtAssert(symbol_id >=
               cfg_->Frame().GetULSymbol(cfg_->Frame().ClientUlPilotSymbols()),
           "Not a UL data symbol!");*/

    data_symbol_idx_ul = cfg_->Frame().GetULSymbolIdx(symbol_id) -
                         cfg_->Frame().ClientUlPilotSymbols();
    symbol_offset = cfg_->GetTotalDataSymbolIdxUl(frame_id, data_symbol_idx_ul);
    if (kDebugPrintInTask == true) {
      std::printf(
          "In doDecode thread %d: frame: %zu, symbol: %zu, cur cb: "
          "%zu, cb: %zu, ue: %zu symbol offset %zu\n",
          tid_, frame_id, symbol_id, cur_cb_id, cb_id, ue_id, symbol_offset);
    }
  } else {
    cb_id = gen_tag_t(tag).symbol_id_;
    cur_cb_id = cb_id;
    stream_id = gen_tag_t(tag).ue_id_;
    ue_id = mac_sched_->ScheduledUeIndex(frame_id, cb_id, stream_id);
    if (kDebugPrintInTask == true) {
      std::printf(
          "In doDecode thread %d: frame: %zu, cur cb: "
          "%zu, ue: %zu\n",
          tid_, frame_id, cur_cb_id, ue_id);
    }
  }

  const size_t frame_slot = (frame_id % kFrameWnd);
  const size_t num_bytes_per_cb =
      mac_sched_->Params().NumBytesPerCb(Direction::kUplink);

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

  size_t block_id = cfg_->SlotScheduling() ? cur_cb_id : data_symbol_idx_ul;
  size_t demod_offset = (cfg_->SlotScheduling() == true
                             ? 0
                             : ldpc_config.NumCbCodewLen() * cur_cb_id) *
                        mac_sched_->Params().ModOrderBits(Direction::kUplink);
  int8_t* llr_buffer_ptr =
      demod_buffers_[frame_slot][block_id][stream_id] + demod_offset;

  size_t decode_block = cfg_->SlotScheduling() ? 0 : data_symbol_idx_ul;
  uint8_t* decoded_buffer_ptr =
      (uint8_t*)decoded_buffers_[frame_slot][decode_block][ue_id] +
      (cur_cb_id *
       Roundup<64>(num_bytes_per_cb));  // alignment is necessary here

  ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
  ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

  size_t start_tsc1 = GetTime::WorkerRdtsc();
  duration_stat_->task_duration_[1] += start_tsc1 - start_tsc;

  bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                          &ldpc_decoder_5gnr_response);

  if (cfg_->ScrambleEnabled()) {
    scrambler_->Descramble(decoded_buffer_ptr, num_bytes_per_cb);
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
    std::stringstream dataprint;
    dataprint << std::setfill('0') << std::hex;
    for (size_t i = 0; i < num_bytes_per_cb; i++) {
      dataprint << " " << std::setw(2) << (int)(*(decoded_buffer_ptr + i));
    }
    std::printf("Decoded data ue %zu cb %zu, \nData: %s\n", ue_id, cb_id,
                dataprint.str().c_str());
  }

  size_t duration = GetTime::WorkerRdtsc() - start_tsc;
  duration_stat_->task_duration_[0] += duration;
  duration_stat_->task_count_++;
  if (GetTime::CyclesToUs(duration, cfg_->FreqGhz()) > 500) {
    std::printf("Thread %d Decode takes %.2f\n", tid_,
                GetTime::CyclesToUs(duration, cfg_->FreqGhz()));
  }

  return {cfg_->SlotScheduling() ? EventType::kDecodeRb : EventType::kDecode,
          tag};
}
