/**
 * @file doencode.cc
 * @brief Implmentation file for the DoEncode class.  Currently, just supports
 * basestation
 */

#include "doencode.h"

#include "concurrent_queue_wrapper.h"
#include "encoder.h"
#include "logger.h"
#include "phy_ldpc_decoder_5gnr.h"

static constexpr bool kPrintEncodedData = false;
static constexpr bool kPrintRawMacData = false;

DoEncode::DoEncode(Config* in_config, int in_tid, Direction dir,
                   Table<int8_t>& in_raw_data_buffer, size_t in_buffer_rollover,
                   Table<int8_t>& in_mod_bits_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      dir_(dir),
      raw_data_buffer_(in_raw_data_buffer),
      raw_buffer_rollover_(in_buffer_rollover),
      mod_bits_buffer_(in_mod_bits_buffer),
      scrambler_(std::make_unique<AgoraScrambler::Scrambler>()) {
  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kEncode, in_tid);
  parity_buffer_ = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      LdpcEncodingParityBufSize(cfg_->LdpcConfig(dir).BaseGraph(),
                                cfg_->LdpcConfig(dir).ExpansionFactor())));
  assert(parity_buffer_ != nullptr);
  encoded_buffer_temp_ = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      LdpcEncodingEncodedBufSize(cfg_->LdpcConfig(dir).BaseGraph(),
                                 cfg_->LdpcConfig(dir).ExpansionFactor())));
  assert(encoded_buffer_temp_ != nullptr);

  const size_t scrambler_buffer_bytes =
      cfg_->NumBytesPerCb(dir) + cfg_->NumPaddingBytesPerCb(dir);

  scrambler_buffer_ = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, scrambler_buffer_bytes));
  std::memset(scrambler_buffer_, 0u, scrambler_buffer_bytes);

  assert(scrambler_buffer_ != nullptr);
}

DoEncode::~DoEncode() {
  std::free(parity_buffer_);
  std::free(encoded_buffer_temp_);
  std::free(scrambler_buffer_);
}

EventData DoEncode::Launch(size_t tag) {
  const LDPCconfig& ldpc_config = cfg_->LdpcConfig(dir_);
  size_t frame_id = gen_tag_t(tag).frame_id_;
  size_t symbol_id = gen_tag_t(tag).symbol_id_;
  size_t cb_id = gen_tag_t(tag).cb_id_;
  size_t cur_cb_id = cb_id % ldpc_config.NumBlocksInSymbol();
  size_t ue_id = cb_id / ldpc_config.NumBlocksInSymbol();

  size_t start_tsc = GetTime::WorkerRdtsc();

  size_t symbol_idx;
  size_t symbol_idx_data;
  if (dir_ == Direction::kDownlink) {
    symbol_idx = cfg_->Frame().GetDLSymbolIdx(symbol_id);
    assert(symbol_idx >= cfg_->Frame().ClientDlPilotSymbols());
    symbol_idx_data = symbol_idx - cfg_->Frame().ClientDlPilotSymbols();
  } else {
    symbol_idx = cfg_->Frame().GetULSymbolIdx(symbol_id);
    assert(symbol_idx >= cfg_->Frame().ClientUlPilotSymbols());
    symbol_idx_data = symbol_idx - cfg_->Frame().ClientUlPilotSymbols();
  }

  if (kDebugPrintInTask) {
    std::printf(
        "In doEncode thread %d: frame: %zu, symbol: %zu:%zu:%zu, code block "
        "%zu, ue_id: %zu\n",
        tid_, frame_id, symbol_id, symbol_idx, symbol_idx_data, cur_cb_id,
        ue_id);
  }

  int8_t* tx_data_ptr = nullptr;
  ///\todo Remove the IsUe condition and make GetMacBits and GetInfoBits
  /// universal with raw_buffer_rollover_ the parameter.
  if (kEnableMac) {
    // All cb's per symbol are included in 1 mac packet
    tx_data_ptr = cfg_->GetMacBits(raw_data_buffer_, dir_,
                                   (frame_id % raw_buffer_rollover_),
                                   symbol_idx_data, ue_id, cur_cb_id);

    if (kPrintRawMacData) {
      auto* pkt = reinterpret_cast<MacPacketPacked*>(tx_data_ptr);
      std::printf(
          "In doEncode [%d] mac packet frame: %d, symbol: %zu:%d, ue_id: %d, "
          "data length %d, crc %d size %zu:%zu\n",
          tid_, pkt->Frame(), symbol_idx_data, pkt->Symbol(), pkt->Ue(),
          pkt->PayloadLength(), pkt->Crc(), cfg_->MacPacketLength(dir_),
          cfg_->NumBytesPerCb(dir_));
      std::printf("Data: ");
      for (size_t i = 0; i < cfg_->MacPayloadMaxLength(dir_); i++) {
        std::printf(" %02x", (uint8_t)(pkt->Data()[i]));
      }
      std::printf("\n");
    }
  } else {
    tx_data_ptr =
        cfg_->GetInfoBits(raw_data_buffer_, dir_, symbol_idx, ue_id, cur_cb_id);
  }

  int8_t* ldpc_input = tx_data_ptr;
  const size_t num_bytes_per_cb = cfg_->NumBytesPerCb(dir_);
  const size_t num_padding_bytes_per_cb = cfg_->NumPaddingBytesPerCb(dir_);

  if (this->cfg_->ScrambleEnabled()) {
    scrambler_->Scramble(scrambler_buffer_, ldpc_input, num_bytes_per_cb);
    ldpc_input = scrambler_buffer_;
  }
  if (num_padding_bytes_per_cb > 0) {
    std::memset(&ldpc_input[num_bytes_per_cb], 0u, num_padding_bytes_per_cb);
  }

  if (kDebugTxData) {
    std::stringstream dataprint;
    dataprint << std::setfill('0') << std::hex;
    for (size_t i = 0; i < num_bytes_per_cb; i++) {
      dataprint << " " << std::setw(2)
                << std::to_integer<int>(
                       reinterpret_cast<std::byte*>(ldpc_input)[i]);
    }
    AGORA_LOG_INFO("ldpc input (%zu %zu %zu): %s\n", frame_id, symbol_idx,
                   ue_id, dataprint.str().c_str());
  }

  LdpcEncodeHelper(ldpc_config.BaseGraph(), ldpc_config.ExpansionFactor(),
                   ldpc_config.NumRows(), encoded_buffer_temp_, parity_buffer_,
                   ldpc_input);
  if (kDebugTxData) {
    std::stringstream dataprint;
    dataprint << std::setfill('0') << std::hex;
    for (size_t i = 0; i < BitsToBytes(ldpc_config.NumCbCodewLen()); i++) {
      dataprint << " " << std::setw(2)
                << std::to_integer<int>(
                       reinterpret_cast<std::byte*>(encoded_buffer_temp_)[i]);
    }
    AGORA_LOG_INFO("ldpc output (%zu %zu %zu): %s\n", frame_id, symbol_idx,
                   ue_id, dataprint.str().c_str());
  }
  int8_t* mod_buffer_ptr = cfg_->GetModBitsBuf(mod_bits_buffer_, dir_, frame_id,
                                               symbol_idx, ue_id, cur_cb_id);

  if (kPrintRawMacData && dir_ == Direction::kUplink) {
    std::printf("Encoded data - placed at location (%zu %zu %zu) %zu\n",
                frame_id, symbol_idx, ue_id,
                reinterpret_cast<intptr_t>(mod_buffer_ptr));
  }
  AdaptBitsForMod(reinterpret_cast<uint8_t*>(encoded_buffer_temp_),
                  reinterpret_cast<uint8_t*>(mod_buffer_ptr),
                  BitsToBytes(ldpc_config.NumCbCodewLen()),
                  cfg_->ModOrderBits(dir_));

  if (kPrintEncodedData) {
    std::printf("Encoded data\n");
    const size_t num_mod = cfg_->SubcarrierPerCodeBlock(dir_);
    for (size_t i = 0; i < num_mod; i++) {
      std::printf("%u ", *(mod_buffer_ptr + i));
    }
    std::printf("\n");
  }

  const size_t duration = GetTime::WorkerRdtsc() - start_tsc;
  duration_stat_->task_duration_[0] += duration;
  duration_stat_->task_count_++;
  if (GetTime::CyclesToUs(duration, cfg_->FreqGhz()) > 500) {
    std::printf("Thread %d Encode takes %.2f\n", tid_,
                GetTime::CyclesToUs(duration, cfg_->FreqGhz()));
  }
  return EventData(EventType::kEncode, tag);
}
