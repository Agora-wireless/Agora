/**
 * @file doencode_client.cc
 * @brief Implmentation file for the DoEncodeClient class.  Currently, just
 * supports client
 */

#include "doencode_client.h"

#include "concurrent_queue_wrapper.h"
#include "encoder.h"
#include "phy_ldpc_decoder_5gnr.h"

static constexpr bool kPrintEncodedData = false;

DoEncodeClient::DoEncodeClient(Config* in_config, size_t in_tid,
                               Table<int8_t>& in_raw_data_buffer,
                               Table<int8_t>& out_encoded_buffer,
                               Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      raw_data_buffer_(in_raw_data_buffer),
      encoded_buffer_(out_encoded_buffer),
      scrambler_(std::make_unique<AgoraScrambler::Scrambler>()) {
  // raw_data_buffer_ = UlBits() or ul_bits_buffer_ (mac enabled)

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

DoEncodeClient::~DoEncodeClient() {
  std::free(parity_buffer_);
  std::free(encoded_buffer_temp_);
  std::free(scrambler_buffer_);
}

EventData DoEncodeClient::Launch(size_t tag) {
  const LDPCconfig& ldpc_config = cfg_->LdpcConfig();
  size_t frame_id = gen_tag_t(tag).frame_id_;
  size_t symbol_id = gen_tag_t(tag).symbol_id_;
  size_t cb_id = gen_tag_t(tag).cb_id_;
  size_t cur_cb_id = cb_id % cfg_->LdpcConfig().NumBlocksInSymbol();
  size_t ue_id = cb_id / cfg_->LdpcConfig().NumBlocksInSymbol();

  size_t start_tsc = GetTime::WorkerRdtsc();
  // size_t symbol_idx_dl = cfg_->Frame().GetDLSymbolIdx(symbol_id); *****
  size_t symbol_idx_ul = cfg_->Frame().GetULSymbolIdx(symbol_id);

  if (kDebugPrintInTask == false) {
    std::printf(
        "In DoEncodeClient thread [%d]: frame %zu, symbol %zu, code block "
        "%zu, ue_id %zu, data symbol index %zu\n",
        tid_, frame_id, symbol_id, cur_cb_id, ue_id, symbol_idx_ul);
  }

  int8_t* ldpc_input = nullptr;

  if (this->cfg_->ScrambleEnabled()) {
    std::memcpy(
        scrambler_buffer_,
        cfg_->GetInfoBits(raw_data_buffer_, symbol_idx_ul, ue_id, cur_cb_id),
        cfg_->NumBytesPerCb());
    scrambler_->Scramble(scrambler_buffer_, cfg_->NumBytesPerCb());
    ldpc_input = scrambler_buffer_;
  } else {
    ldpc_input =
        cfg_->GetInfoBits(raw_data_buffer_, symbol_idx_ul, ue_id, cur_cb_id);
  }

  LdpcEncodeHelper(ldpc_config.BaseGraph(), ldpc_config.ExpansionFactor(),
                   ldpc_config.NumRows(), encoded_buffer_temp_, parity_buffer_,
                   ldpc_input);

  int8_t* final_output_ptr = cfg_->GetEncodedBuf(
      encoded_buffer_, frame_id, symbol_idx_ul, ue_id, cur_cb_id);

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
