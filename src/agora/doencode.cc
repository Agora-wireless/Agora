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
                   Table<int8_t>& in_mod_bits_buffer, MacScheduler* mac_sched,
                   Stats* in_stats_manager)
    : Doer(in_config, in_tid),
      dir_(dir),
      raw_data_buffer_(in_raw_data_buffer),
      raw_buffer_rollover_(in_buffer_rollover),
      mod_bits_buffer_(in_mod_bits_buffer),
      mac_sched_(mac_sched),
      scrambler_(std::make_unique<AgoraScrambler::Scrambler>()) {
  const auto bg = cfg_->LdpcConfig(dir).BaseGraph();
  const auto zc = cfg_->LdpcConfig(dir).ExpansionFactor();

  duration_stat_ = in_stats_manager->GetDurationStat(DoerType::kEncode, in_tid);
  parity_buffer_ = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, LdpcEncodingParityBufSize(bg, zc)));
  assert(parity_buffer_ != nullptr);
  encoded_buffer_temp_ = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, LdpcEncodingEncodedBufSize(bg, zc)));
  assert(encoded_buffer_temp_ != nullptr);

  scrambler_buffer_bytes_ =
      cfg_->NumBytesPerCb(dir) + cfg_->NumPaddingBytesPerCb(dir);

  scrambler_buffer_ = static_cast<int8_t*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64, scrambler_buffer_bytes_));
  std::memset(scrambler_buffer_, 0u, scrambler_buffer_bytes_);

  assert(scrambler_buffer_ != nullptr);
}

DoEncode::~DoEncode() {
  std::free(parity_buffer_);
  std::free(encoded_buffer_temp_);
  std::free(scrambler_buffer_);
}

EventData DoEncode::Launch(size_t tag) {
  const LDPCconfig& ldpc_config = cfg_->LdpcConfig(dir_);
  const size_t frame_id = gen_tag_t(tag).frame_id_;
  const size_t symbol_id = gen_tag_t(tag).symbol_id_;
  const size_t cb_id = gen_tag_t(tag).cb_id_;
  const size_t cur_cb_id = cb_id % ldpc_config.NumBlocksInSymbol();
  const size_t sched_ue_id = cb_id / ldpc_config.NumBlocksInSymbol();

  size_t start_tsc = GetTime::WorkerRdtsc();

  size_t symbol_idx;
  size_t data_symbol_idx;
  size_t ue_id;
  if (dir_ == Direction::kDownlink) {
    symbol_idx = cfg_->Frame().GetDLSymbolIdx(symbol_id);
    assert(symbol_idx >= cfg_->Frame().ClientDlPilotSymbols());
    data_symbol_idx = symbol_idx - cfg_->Frame().ClientDlPilotSymbols();
    ue_id = mac_sched_->ScheduledUeIndex(frame_id, 0u, sched_ue_id);
  } else {
    symbol_idx = cfg_->Frame().GetULSymbolIdx(symbol_id);
    assert(symbol_idx >= cfg_->Frame().ClientUlPilotSymbols());
    data_symbol_idx = symbol_idx - cfg_->Frame().ClientUlPilotSymbols();
    ue_id = sched_ue_id;
  }

  if (kDebugPrintInTask) {
    std::printf(
        "In doEncode thread %d: frame: %zu, symbol: %zu:%zu:%zu, code block "
        "%zu, ue_id: %zu\n",
        tid_, frame_id, symbol_id, symbol_idx, data_symbol_idx, cur_cb_id,
        ue_id);
  }

  int8_t* tx_data_ptr = nullptr;
  ///\todo Make GetMacBits and GetInfoBits
  /// universal with raw_buffer_rollover_ the parameter.
  if (kEnableMac) {
    // All cb's per symbol are included in 1 mac packet
    tx_data_ptr = cfg_->GetMacBits(raw_data_buffer_, dir_,
                                   (frame_id % raw_buffer_rollover_),
                                   data_symbol_idx, ue_id, cur_cb_id);

    if (kPrintRawMacData) {
      auto* pkt = reinterpret_cast<MacPacketPacked*>(tx_data_ptr);
      std::printf(
          "In doEncode [%d] mac packet frame: %d, symbol: %zu:%d, ue_id: %d, "
          "data length %d, crc %d size %zu:%zu\n",
          tid_, pkt->Frame(), data_symbol_idx, pkt->Symbol(), pkt->Ue(),
          pkt->PayloadLength(), pkt->Crc(), cfg_->MacPacketLength(dir_),
          cfg_->NumBytesPerCb(dir_));
      std::printf("Data: ");
      for (size_t i = 0; i < cfg_->MacPayloadMaxLength(dir_); i++) {
        std::printf(" %02x", (uint8_t)(pkt->Data()[i]));
      }
      std::printf("\n");
    }
  } else {
    tx_data_ptr = cfg_->GetInfoBits(raw_data_buffer_, dir_, data_symbol_idx,
                                    ue_id, cur_cb_id);
  }

  int8_t* ldpc_input = tx_data_ptr;
  const size_t num_bytes_per_cb = cfg_->NumBytesPerCb(dir_);

  if (this->cfg_->ScrambleEnabled()) {
    scrambler_->Scramble(scrambler_buffer_, ldpc_input, num_bytes_per_cb);
    ldpc_input = scrambler_buffer_;
  }
  if (scrambler_buffer_bytes_ > num_bytes_per_cb) {
    std::memset(&ldpc_input[num_bytes_per_cb], 0u,
                scrambler_buffer_bytes_ - num_bytes_per_cb);
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
  int8_t* mod_buffer_ptr =
      cfg_->GetModBitsBuf(mod_bits_buffer_, dir_, frame_id, data_symbol_idx,
                          sched_ue_id, cur_cb_id);

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
