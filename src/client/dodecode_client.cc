/**
 * @file dodecode_client.cc
 * @brief Implmentation file for the DoDecodeClient class.
 */
#include "dodecode_client.h"

#include "concurrent_queue_wrapper.h"
#include "gettime.h"
#include "logger.h"
#include "message.h"
#include "phy_ldpc_decoder_5gnr.h"

static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

static constexpr size_t kVarNodesSize = 1024 * 1024 * sizeof(int16_t);

DoDecodeClient::DoDecodeClient(
    Config* in_config, int in_tid,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffers,
    PhyStats* in_phy_stats, Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      demod_buffers_(demod_buffers),
      decoded_buffers_(decoded_buffers),
      phy_stats_(in_phy_stats),
      scrambler_(std::make_unique<AgoraScrambler::Scrambler>()) {
  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kDecode, in_tid);
  resp_var_nodes_ = static_cast<int16_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, kVarNodesSize));
}

DoDecodeClient::~DoDecodeClient() { std::free(resp_var_nodes_); }

EventData DoDecodeClient::Launch(size_t tag) {
  const LDPCconfig& ldpc_config = cfg_->LdpcConfig(Direction::kDownlink);
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t cb_id = gen_tag_t(tag).cb_id_;

  const size_t symbol_idx_dl = cfg_->Frame().GetDLSymbolIdx(symbol_id);
  const size_t symbol_offset =
      cfg_->GetTotalDataSymbolIdxDl(frame_id, symbol_idx_dl);
  const size_t cur_cb_id = (cb_id % ldpc_config.NumBlocksInSymbol());
  const size_t ue_id = (cb_id / ldpc_config.NumBlocksInSymbol());
  const size_t frame_slot = (frame_id % kFrameWnd);

  if (kDebugPrintInTask == true) {
    AGORA_LOG_INFO(
        "In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
        "%zu, ue: %zu offset %zu\n",
        tid_, frame_id, symbol_id, cur_cb_id, ue_id, symbol_offset);
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

  int8_t* llr_buffer_ptr = demod_buffers_[frame_slot][symbol_idx_dl][ue_id] +
                           (cfg_->ModOrderBits(Direction::kDownlink) *
                            (ldpc_config.NumCbCodewLen() * cur_cb_id));

  uint8_t* decoded_buffer_ptr =
      (uint8_t*)decoded_buffers_[frame_slot][symbol_idx_dl][ue_id] +
      (cur_cb_id * Roundup<64>(cfg_->NumBytesPerCb(Direction::kDownlink)));

  ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
  ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

  size_t start_tsc1 = GetTime::WorkerRdtsc();
  duration_stat_->task_duration_[1] += start_tsc1 - start_tsc;

  bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request,
                          &ldpc_decoder_5gnr_response);

  if (cfg_->ScrambleEnabled()) {
    scrambler_->Descramble(decoded_buffer_ptr,
                           cfg_->NumBytesPerCb(Direction::kDownlink));
  }

  size_t start_tsc2 = GetTime::WorkerRdtsc();
  duration_stat_->task_duration_[2] += start_tsc2 - start_tsc1;

  if (kPrintLLRData) {
    AGORA_LOG_INFO("LLR data, symbol_offset: %zu\n", symbol_offset);
    for (size_t i = 0; i < ldpc_config.NumCbCodewLen(); i++) {
      AGORA_LOG_INFO("%d ", *(llr_buffer_ptr + i));
    }
    AGORA_LOG_INFO("\n");
  }

  if (kPrintDecodedData) {
    AGORA_LOG_INFO("Decoded data\n");
    for (size_t i = 0; i < (ldpc_config.NumCbLen() >> 3); i++) {
      AGORA_LOG_INFO("%u ", *(decoded_buffer_ptr + i));
    }
    AGORA_LOG_INFO("\n");
  }

  if ((kEnableMac == false) && (kPrintPhyStats == true) &&
      (symbol_idx_dl >= cfg_->Frame().ClientDlPilotSymbols())) {
    phy_stats_->UpdateDecodedBits(
        ue_id, symbol_offset, frame_slot,
        cfg_->NumBytesPerCb(Direction::kDownlink) * 8);
    phy_stats_->IncrementDecodedBlocks(ue_id, symbol_offset, frame_slot);
    size_t block_error(0);
    for (size_t i = 0; i < cfg_->NumBytesPerCb(Direction::kDownlink); i++) {
      uint8_t rx_byte = decoded_buffer_ptr[i];
      auto tx_byte = static_cast<uint8_t>(
          cfg_->GetInfoBits(cfg_->DlBits(), Direction::kDownlink, symbol_idx_dl,
                            kDebugDownlink ? 0 : ue_id, cur_cb_id)[i]);
      phy_stats_->UpdateBitErrors(ue_id, symbol_offset, frame_slot, tx_byte,
                                  rx_byte);
      if (rx_byte != tx_byte) {
        block_error++;
      }
    }
    phy_stats_->UpdateBlockErrors(ue_id, symbol_offset, frame_slot,
                                  block_error);
  }

  size_t duration = GetTime::WorkerRdtsc() - start_tsc;
  duration_stat_->task_duration_[0] += duration;
  duration_stat_->task_count_++;
  if (GetTime::CyclesToUs(duration, cfg_->FreqGhz()) > 500) {
    AGORA_LOG_INFO("Thread %d Decode takes %.2f\n", tid_,
                   GetTime::CyclesToUs(duration, cfg_->FreqGhz()));
  }

  return EventData(EventType::kDecode, tag);
}
