/**
 * @file doencode.cc
 * @brief Implmentation file for the DoEncode class.  Currently, just supports
 * basestation
 */

#include "doencode.h"

#include "concurrent_queue_wrapper.h"
#include "encoder.h"
#include "phy_ldpc_decoder_5gnr.h"

static constexpr bool kPrintEncodedData = false;

DoEncode::DoEncode(Config* in_config, int in_tid,
                   Table<int8_t>& in_mac_data_buffer,
                   Table<int8_t>& in_encoded_buffer, size_t in_mac_frame_wnd,
                   size_t in_mac_bytes_perframe, Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      mac_frame_wnd_(in_mac_frame_wnd),
      mac_bytes_perframe_(in_mac_bytes_perframe),
      mac_data_buffer_(in_mac_data_buffer),
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
  assert(encoded_buffer_temp_ != nullptr);

  scrambler_buffer_ = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      cfg_->NumBytesPerCb() + kLdpcHelperFunctionInputBufferSizePaddingBytes));

  assert(scrambler_buffer_ != nullptr);
}

DoEncode::~DoEncode() {
  std::free(parity_buffer_);
  std::free(encoded_buffer_temp_);
  std::free(scrambler_buffer_);
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

  size_t symbol_idx;
  if (cfg_->IsUe() == false) {
    symbol_idx = cfg_->Frame().GetDLSymbolIdx(symbol_id);
  } else {
    symbol_idx = cfg_->Frame().GetULSymbolIdx(symbol_id);
  }

  int8_t* mac_output = nullptr;
  if (kEnableMac == true) {
    size_t symbol_idx_data;
    if (cfg_->IsUe() == false) {
      if (symbol_idx >= cfg_->Frame().ClientDlPilotSymbols()) {
        symbol_idx_data = symbol_idx - cfg_->Frame().ClientDlPilotSymbols();
      } else {
        return EventData(EventType::kEncode, tag);
      }
    } else {
      if (symbol_idx >= cfg_->Frame().ClientUlPilotSymbols()) {
        symbol_idx_data = symbol_idx - cfg_->Frame().ClientUlPilotSymbols();
      } else {
        return EventData(EventType::kEncode, tag);
      }
    }
    mac_output = cfg_->GetMacBits(mac_data_buffer_, frame_id, symbol_idx_data,
                                  ue_id, cur_cb_id);
  } else {
    mac_output =
        cfg_->GetInfoBits(mac_data_buffer_, symbol_idx, ue_id, cur_cb_id);
  }
  std::memcpy(scrambler_buffer_, mac_output, cfg_->NumBytesPerCb());

  if (this->cfg_->ScrambleEnabled()) {
    scrambler_->Scramble(scrambler_buffer_, cfg_->NumBytesPerCb());
  }
  int8_t* ldpc_input = scrambler_buffer_;

  LdpcEncodeHelper(ldpc_config.BaseGraph(), ldpc_config.ExpansionFactor(),
                   ldpc_config.NumRows(), encoded_buffer_temp_, parity_buffer_,
                   ldpc_input);
  int8_t* final_output_ptr = cfg_->GetEncodedBuf(encoded_buffer_, frame_id,
                                                 symbol_idx, ue_id, cur_cb_id);
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
