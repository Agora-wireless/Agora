/**
 * @file txrx_worker_usrp.cc
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with usrp hardware
 */

#include "txrx_worker_usrp.h"

#include <cassert>

#include "gettime.h"
#include "logger.h"
#include "message.h"

TxRxWorkerUsrp::TxRxWorkerUsrp(
    size_t core_offset, size_t tid, size_t radio_hi, size_t radio_lo,
    Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    std::mutex& sync_mutex, std::condition_variable& sync_cond,
    std::atomic<bool>& can_proceed, RadioConfig& radio_config)
    : TxRxWorker(core_offset, tid, radio_hi, radio_lo, config->NumChannels(),
                 config, rx_frame_start, event_notify_q, tx_pending_q,
                 tx_producer, notify_producer, rx_memory, tx_memory, sync_mutex,
                 sync_cond, can_proceed),
      radio_config_(radio_config) {}

TxRxWorkerUsrp::~TxRxWorkerUsrp() = default;

//Main Thread Execution loop
void TxRxWorkerUsrp::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  AGORA_LOG_INFO("TxRxWorkerUsrp[%zu] has %zu:%zu total radios %zu\n", tid_,
                 interface_offset_, (interface_offset_ + num_interfaces_) - 1,
                 num_interfaces_);

  // prepare BS beacon in host buffer
  std::vector<void*> beaconbuff(2);
  void* zeros =
      std::calloc(Configuration()->SampsPerSymbol(), sizeof(int16_t) * 2);

  beaconbuff.at(0) = Configuration()->BeaconCi16().data();
  beaconbuff.at(1) = zeros;

  std::vector<std::complex<int16_t>> samp_buffer0(
      Configuration()->SampsPerSymbol() * 14, 0);
  std::vector<std::complex<int16_t>> samp_buffer1(
      Configuration()->SampsPerSymbol() * 14, 0);
  std::vector<std::vector<std::complex<int16_t>>*> samp_buffer(2);
  samp_buffer.at(0) = &samp_buffer0;
  if (true) {
    samp_buffer.at(1) = &samp_buffer1;
  }

  rx_time_bs_ = 0;
  tx_time_bs_ = 0;

  Radio::RxFlags rx_flags;
  std::cout << "Sync BS host and FGPA timestamp..." << std::endl;
  radio_config_.RadioRx(0, samp_buffer, Configuration()->SampsPerSymbol(),
                        rx_flags, rx_time_bs_);
  // Schedule the first beacon in the future
  tx_time_bs_ = rx_time_bs_ + Configuration()->SampsPerSymbol() *
                                  Configuration()->Frame().NumTotalSyms() * 40;
  radio_config_.RadioTx(0, beaconbuff.data(), Radio::TxFlags::kEndTransmit,
                        tx_time_bs_);
  long long bs_init_rx_offset = tx_time_bs_ - rx_time_bs_;
  for (int it = 0;
       it < std::floor(bs_init_rx_offset / Configuration()->SampsPerSymbol());
       it++) {
    radio_config_.RadioRx(0, samp_buffer, Configuration()->SampsPerSymbol(),
                          rx_flags, rx_time_bs_);
  }

  std::cout << std::endl;
  std::cout << "Init BS sync done..." << std::endl;
  std::cout << "Start BS main recv loop..." << std::endl;

  size_t global_frame_id = 0;
  size_t global_symbol_id = 0;

  int prev_frame_id = -1;
  size_t local_interface = 0;

  running_ = true;
  WaitSync();

  while (Configuration()->Running() == true) {
    // transmit data
    // if (-1 != dequeue_send_usrp(tid))
    //   continue;
    // receive data
    std::vector<Packet*> rx_pkts =
        RecvEnqueue(local_interface, global_frame_id, global_symbol_id);

    // Schedule beacon in the future
    if (global_symbol_id == 0) {
      tx_time_bs_ = rx_time_bs_ + Configuration()->SampsPerSymbol() *
                                      Configuration()->Frame().NumTotalSyms() *
                                      20;
      int tx_ret = radio_config_.RadioTx(
          0, beaconbuff.data(), Radio::TxFlags::kEndTransmit, tx_time_bs_);
      if (tx_ret != (int)Configuration()->SampsPerSymbol()) {
        std::cerr << "BAD Transmit(" << tx_ret << "/"
                  << Configuration()->SampsPerSymbol() << ") at Time "
                  << tx_time_bs_ << ", frame count " << global_frame_id
                  << std::endl;
      }
    }

    if (++local_interface == num_interfaces_) {
      local_interface = 0;
    }

    // Update global frame_id and symbol_id
    global_symbol_id++;
    if (global_symbol_id == Configuration()->Frame().NumTotalSyms()) {
      global_symbol_id = 0;
      global_frame_id++;
    }

    if (rx_pkts.empty()) {
      continue;
    }
    if (kIsWorkerTimingEnabled) {
      int frame_id = rx_pkts.front()->frame_id_;
      if (frame_id > prev_frame_id) {
        rx_frame_start_[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
        prev_frame_id = frame_id;
      }
    }
  }

  std::free(zeros);
  zeros = nullptr;
  running_ = false;
}

//RX data
std::vector<Packet*> TxRxWorkerUsrp::RecvEnqueue(size_t radio_id,
                                                 size_t frame_id,
                                                 size_t symbol_id) {
  std::vector<Packet*> rx_packets;
  std::vector<RxPacket*> memory_tracking;

  size_t n_channels = Configuration()->NumChannels();
  std::vector<void*> samp(n_channels);
  for (size_t ch = 0; ch < n_channels; ++ch) {
    RxPacket& rx = GetRxPacket();
    memory_tracking.push_back(&rx);
    samp.at(ch) = rx.RawPacket();
  }

  bool dummy_read = false;
  if ((Configuration()->IsPilot(frame_id, symbol_id) == false) &&
      (Configuration()->IsUplink(frame_id, symbol_id) == false)) {
    dummy_read = true;
  }

  Radio::RxFlags rx_flags;
  const int tmp_ret = radio_config_.RadioRx(
      radio_id, samp, Configuration()->SampsPerSymbol(), rx_flags, rx_time_bs_);

  if ((tmp_ret > 0) && (dummy_read == false)) {
    const size_t ant_id = radio_id * n_channels;
    if (Configuration()->IsPilot(frame_id, symbol_id) ||
        Configuration()->IsUplink(frame_id, symbol_id)) {
      for (size_t ch = 0; ch < n_channels; ++ch) {
        RxPacket& rx = *memory_tracking.at(ch);
        memory_tracking.at(ch) = nullptr;
        new (rx.RawPacket()) Packet(frame_id, symbol_id, 0, ant_id + ch);
        rx_packets.push_back(rx.RawPacket());
        // Push kPacketRX event into the queue
        const EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);
        NotifyComplete(rx_message);
      }
    }
    return rx_packets;
  }
  //Free memory from most recent allocated to latest
  for (size_t idx = (memory_tracking.size() - 1); idx > 0; idx--) {
    auto* memory_location = memory_tracking.at(idx);
    if (memory_location != nullptr) {
      ReturnRxPacket(*memory_location);
    }
  }
  return rx_packets;
}

//Tx data
int TxRxWorkerUsrp::DequeueSend() {
  auto events = GetPendingTxEvents(1);
  if (events.empty()) {
    return -1;
  } else if (events.size() > 1) {
    throw std::runtime_error("DequeueSend returned too many events");
    return -1;
  }
  EventData event = events.at(0);
  assert(event.event_type_ == EventType::kPacketTX);

  const size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

  const size_t offset =
      (Configuration()->GetTotalDataSymbolIdx(frame_id, symbol_id) *
       Configuration()->BsAntNum()) +
      ant_id;

  symbol_id += Configuration()->UeAntNum();
  frame_id += TX_FRAME_DELTA;

  const size_t n_channels = Configuration()->NumChannels();
  const size_t radio_id = ant_id / n_channels;
  const size_t ch = ant_id % n_channels;

  std::vector<const void*> txbuffs(n_channels, nullptr);

  if (kDebugDownlink == true) {
    const size_t cell_id = Configuration()->CellId().at(radio_id);
    const std::vector<std::complex<int16_t>> zeros(
        Configuration()->SampsPerSymbol(), std::complex<int16_t>(0, 0));
    size_t dl_symbol_idx = Configuration()->Frame().GetDLSymbolIdx(symbol_id);
    if (ant_id != Configuration()->RefAnt(cell_id)) {
      txbuffs.at(ch) = zeros.data();
    } else if (dl_symbol_idx <
               Configuration()->Frame().ClientDlPilotSymbols()) {
      txbuffs.at(ch) =
          reinterpret_cast<void*>(Configuration()->UeSpecificPilotT()[0]);
    } else {
      txbuffs.at(ch) = reinterpret_cast<void*>(
          Configuration()
              ->DlIqT()[dl_symbol_idx -
                        Configuration()->Frame().ClientDlPilotSymbols()]);
    }
  } else {
    auto* pkt = GetTxPacket(frame_id, symbol_id, ant_id + ch);
    txbuffs.at(ch) = reinterpret_cast<void*>(pkt->data_);
  }

  size_t last = Configuration()->Frame().GetDLSymbolLast();
  Radio::TxFlags flags = (symbol_id != last) ? Radio::TxFlags::kTxFlagNone
                                             : Radio::TxFlags::kEndTransmit;
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radio_config_.RadioTx(radio_id, txbuffs.data(), flags, frame_time);

  if (kDebugPrintInTask == true) {
    std::printf(
        "TxRxWorkerUsrp[%zu]: Transmitted frame %zu, symbol %zu, "
        "ant %zu, offset: %zu\n",
        tid_, frame_id, symbol_id, ant_id, offset);
  }

  const auto complete_event = EventData(EventType::kPacketTX, event.tags_[0]);
  NotifyComplete(complete_event);
  return event.tags_[0];
}

int TxRxWorkerUsrp::DequeueSend(int frame_id, int symbol_id) {
  auto events = GetPendingTxEvents(1);
  if (events.empty()) {
    return -1;
  } else if (events.size() > 1) {
    throw std::runtime_error("DequeueSend returned too many events");
    return -1;
  }
  EventData event = events.at(0);
  std::cout << "DDD" << std::endl;
  assert(event.event_type_ == EventType::kPacketTX);

  const size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  // size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
  // size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

  size_t offset = (Configuration()->GetTotalDataSymbolIdx(frame_id, symbol_id) *
                   Configuration()->BsAntNum()) +
                  ant_id;

  // symbol_id += Configuration()->UeAntNum();
  // frame_id += TX_FRAME_DELTA;

  std::cout << "ant_id: " << ant_id << ", frame_id: " << frame_id
            << ", symbol_id: " << symbol_id;
  std::cout << "offset: " << offset << std::endl;

  const size_t n_channels = Configuration()->NumChannels();
  const size_t radio_id = ant_id / n_channels;
  const size_t ch = ant_id % n_channels;

  std::vector<const void*> txbuffs(n_channels, nullptr);

  if (kDebugDownlink) {
    const size_t cell_id = Configuration()->CellId().at(radio_id);
    std::vector<std::complex<int16_t>> zeros(Configuration()->SampsPerSymbol());
    size_t dl_symbol_idx = Configuration()->Frame().GetDLSymbolIdx(symbol_id);
    if (ant_id != Configuration()->RefAnt(cell_id)) {
      txbuffs.at(ch) = zeros.data();
    } else if (dl_symbol_idx <
               Configuration()->Frame().ClientDlPilotSymbols()) {
      txbuffs.at(ch) =
          reinterpret_cast<void*>(Configuration()->UeSpecificPilotT()[0]);
    } else {
      txbuffs.at(ch) = reinterpret_cast<void*>(
          Configuration()
              ->DlIqT()[dl_symbol_idx -
                        Configuration()->Frame().ClientDlPilotSymbols()]);
    }
  } else {
    auto* pkt = GetTxPacket(frame_id, symbol_id, ant_id + ch);
    txbuffs.at(ch) = reinterpret_cast<void*>(pkt->data_);
  }

  const size_t last = Configuration()->Frame().GetDLSymbolLast();
  Radio::TxFlags flags = (symbol_id != static_cast<int>(last))
                             ? Radio::TxFlags::kTxFlagNone
                             : Radio::TxFlags::kEndTransmit;
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radio_config_.RadioTx(radio_id, txbuffs.data(), flags, frame_time);

  if (kDebugPrintInTask) {
    std::printf(
        "TxRxWorkerUsrp[%zu]: Transmitted frame %d, symbol %d, "
        "ant %zu, offset: %zu\n",
        tid_, frame_id, symbol_id, ant_id, offset);
  }

  const auto complete_event = EventData(EventType::kPacketTX, event.tags_[0]);
  NotifyComplete(complete_event);
  return event.tags_[0];
}