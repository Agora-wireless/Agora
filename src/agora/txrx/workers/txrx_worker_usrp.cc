/**
 * @file txrx_worker_usrp.cc
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real hardware
 */

#include "txrx_worker_usrp.h"

#include <cassert>

#include "logger.h"

static constexpr bool kDebugDownlink = false;

TxRxWorkerUsrp::TxRxWorkerUsrp(
    size_t core_offset, size_t tid, size_t radio_hi, size_t radio_lo,
    Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    RadioConfig* const radio_config)
    : TxRxWorker(core_offset, tid, radio_hi, radio_lo, config, rx_frame_start,
                 event_notify_q, tx_pending_q, tx_producer, notify_producer,
                 rx_memory, tx_memory),
      radio_config_(radio_config) {}

TxRxWorkerUsrp::~TxRxWorkerUsrp() {}

//Main Thread Execution loop
void TxRxWorkerUsrp::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);
  size_t rx_slot = 0;

  MLPD_INFO("TxRxWorkerUsrp[%zu] has %zu:%zu total radios %zu\n", tid_,
            interface_offset_, (interface_offset_ + num_interfaces_) - 1,
            num_interfaces_);

  // prepare BS beacon in host buffer
  std::vector<void*> beaconbuff(2);
  std::vector<void*> zeros(2);
  zeros[0] = calloc(Configuration()->SampsPerSymbol(), sizeof(int16_t) * 2);
  zeros[1] = calloc(Configuration()->SampsPerSymbol(), sizeof(int16_t) * 2);
  beaconbuff[0] = Configuration()->BeaconCi16().data();
  beaconbuff[1] = zeros[0];

  std::vector<std::complex<int16_t>> samp_buffer0(
      Configuration()->SampsPerSymbol() * 14, 0);
  std::vector<std::complex<int16_t>> samp_buffer1(
      Configuration()->SampsPerSymbol() * 14, 0);
  std::vector<void*> samp_buffer(2);
  samp_buffer[0] = samp_buffer0.data();
  if (true) {
    samp_buffer[1] = samp_buffer1.data();
  }

  rx_time_bs_ = 0;
  tx_time_bs_ = 0;

  std::cout << "Sync BS host and FGPA timestamp..." << std::endl;
  radio_config_->RadioRx(0, samp_buffer.data(), rx_time_bs_);
  // Schedule the first beacon in the future
  tx_time_bs_ = rx_time_bs_ + Configuration()->SampsPerSymbol() *
                                  Configuration()->Frame().NumTotalSyms() * 40;
  radio_config_->RadioTx(0, beaconbuff.data(), 2, tx_time_bs_);
  long long bs_init_rx_offset = tx_time_bs_ - rx_time_bs_;
  for (int it = 0;
       it < std::floor(bs_init_rx_offset / Configuration()->SampsPerSymbol());
       it++) {
    radio_config_->RadioRx(0, samp_buffer.data(), rx_time_bs_);
  }

  std::cout << std::endl;
  std::cout << "Init BS sync done..." << std::endl;
  std::cout << "Start BS main recv loop..." << std::endl;

  size_t global_frame_id = 0;
  size_t global_symbol_id = 0;

  int prev_frame_id = -1;
  size_t local_interface = 0;

  running_ = true;
  started_ = true;
  while (Configuration()->Running() == true) {
    // transmit data
    // if (-1 != dequeue_send_usrp(tid))
    //   continue;
    // receive data
    Packet* pkt = RecvEnqueue(local_interface, rx_slot, global_frame_id,
                              global_symbol_id);

    // Schedule beacon in the future
    if (global_symbol_id == 0) {
      tx_time_bs_ = rx_time_bs_ + Configuration()->SampsPerSymbol() *
                                      Configuration()->Frame().NumTotalSyms() *
                                      20;
      int tx_ret = radio_config_->RadioTx(0, beaconbuff.data(), 2, tx_time_bs_);
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

    if (pkt == nullptr) {
      continue;
    }
    rx_slot = (rx_slot + Configuration()->NumChannels()) % rx_memory_.size();

    if (kIsWorkerTimingEnabled) {
      int frame_id = pkt->frame_id_;
      if (frame_id > prev_frame_id) {
        rx_frame_start_[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
        prev_frame_id = frame_id;
      }
    }
  }
  running_ = false;
}

//RX data
Packet* TxRxWorkerUsrp::RecvEnqueue(size_t radio_id, size_t rx_slot,
                                    size_t frame_id, size_t symbol_id) {
  // init samp_buffer for dummy read
  std::vector<std::complex<int16_t>> samp_buffer0(
      Configuration()->SampsPerSymbol() *
          Configuration()->Frame().NumTotalSyms(),
      0);
  std::vector<std::complex<int16_t>> samp_buffer1(
      Configuration()->SampsPerSymbol() *
          Configuration()->Frame().NumTotalSyms(),
      0);
  std::vector<void*> samp_buffer(2);
  samp_buffer[0] = samp_buffer0.data();
  if (Configuration()->NumChannels() == 2) {
    samp_buffer[1] = samp_buffer1.data();
  }

  size_t n_channels = Configuration()->NumChannels();
  std::vector<void*> samp(n_channels);
  for (size_t ch = 0; ch < n_channels; ++ch) {
    RxPacket& rx = rx_memory_.at(rx_slot + ch);
    // if rx_buffer is full, exit
    if (rx.Empty() == false) {
      std::printf("Receive thread %zu rx_buffer full, offset: %zu\n", tid_,
                  rx_slot);
      Configuration()->Running(false);
      break;
    }
    samp.at(ch) = rx.RawPacket();
  }

  int tmp_ret;
  if (Configuration()->IsPilot(frame_id, symbol_id) ||
      Configuration()->IsUplink(frame_id, symbol_id)) {
    tmp_ret = radio_config_->RadioRx(radio_id, samp.data(), rx_time_bs_);
  } else {
    tmp_ret = radio_config_->RadioRx(radio_id, samp_buffer.data(), rx_time_bs_);
  }

  if ((Configuration()->Running() == false) || tmp_ret <= 0 ||
      (!Configuration()->IsPilot(frame_id, symbol_id) &&
       !Configuration()->IsUplink(frame_id, symbol_id))) {
    return nullptr;
  }

  size_t ant_id = radio_id * n_channels;
  if (Configuration()->IsPilot(frame_id, symbol_id) ||
      Configuration()->IsUplink(frame_id, symbol_id)) {
    for (size_t ch = 0; ch < n_channels; ++ch) {
      RxPacket& rx = rx_memory_.at(rx_slot + ch);
      new (rx.RawPacket()) Packet(frame_id, symbol_id, 0, ant_id + ch);
      rx.Use();
      // Push kPacketRX event into the queue
      EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);

      if (event_notify_q_->enqueue(notify_producer_token_, rx_message) ==
          false) {
        std::printf("socket message enqueue failed\n");
        throw std::runtime_error("PacketTxRx: socket message enqueue failed");
      }
    }
  }
  return rx_memory_.at(rx_slot).RawPacket();
}

//Tx data
int TxRxWorkerUsrp::DequeueSend() {
  EventData event;
  if (tx_pending_q_->try_dequeue_from_producer(tx_producer_token_, event) ==
      0) {
    return -1;
  }

  std::printf("tx queue length: %zu\n", tx_pending_q_->size_approx());
  assert(event.event_type_ == EventType::kPacketTX);

  size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
  size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;

  size_t offset = (Configuration()->GetTotalDataSymbolIdx(frame_id, symbol_id) *
                   Configuration()->BsAntNum()) +
                  ant_id;

  symbol_id += Configuration()->UeAntNum();
  frame_id += TX_FRAME_DELTA;

  void* txbuf[2];
  int n_channels = Configuration()->NumChannels();
  int ch = ant_id % n_channels;

  if (kDebugDownlink == true) {
    std::vector<std::complex<int16_t>> zeros(Configuration()->SampsPerSymbol());
    size_t dl_symbol_idx = Configuration()->Frame().GetDLSymbolIdx(symbol_id);
    if (ant_id != Configuration()->RefAnt()) {
      txbuf[ch] = zeros.data();
    } else if (dl_symbol_idx <
               Configuration()->Frame().ClientDlPilotSymbols()) {
      txbuf[ch] =
          reinterpret_cast<void*>(Configuration()->UeSpecificPilotT()[0]);
    } else {
      txbuf[ch] = reinterpret_cast<void*>(
          Configuration()
              ->DlIqT()[dl_symbol_idx -
                        Configuration()->Frame().ClientDlPilotSymbols()]);
    }
  } else {
    auto* pkt = reinterpret_cast<struct Packet*>(
        &tx_memory_[offset * Configuration()->PacketLength()]);
    txbuf[ch] = reinterpret_cast<void*>(pkt->data_);
  }

  size_t last = Configuration()->Frame().GetDLSymbolLast();
  int flags = (symbol_id != last) ? 1   // HAS_TIME
                                  : 2;  // HAS_TIME & END_BURST, fixme
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radio_config_->RadioTx(ant_id / n_channels, txbuf, flags, frame_time);

  if (kDebugPrintInTask == true) {
    std::printf(
        "TxRxWorkerUsrp[%zu]: Transmitted frame %zu, symbol %zu, "
        "ant %zu, offset: %zu, msg_queue_length: %zu\n",
        tid_, frame_id, symbol_id, ant_id, offset,
        event_notify_q_->size_approx());
  }

  RtAssert(
      event_notify_q_->enqueue(notify_producer_token_,
                               EventData(EventType::kPacketTX, event.tags_[0])),
      "Socket message enqueue failed\n");
  return event.tags_[0];
}

int TxRxWorkerUsrp::DequeueSend(int frame_id, int symbol_id) {
  EventData event;
  if (tx_pending_q_->try_dequeue_from_producer(tx_producer_token_, event) ==
      0) {
    return -1;
  }
  std::cout << "DDD" << std::endl;

  std::printf("tx queue length: %zu\n", tx_pending_q_->size_approx());
  assert(event.event_type_ == EventType::kPacketTX);

  size_t ant_id = gen_tag_t(event.tags_[0]).ant_id_;
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

  void* txbuf[2];
  const size_t n_channels = Configuration()->NumChannels();
  const size_t ch = ant_id % n_channels;

  if (kDebugDownlink) {
    std::vector<std::complex<int16_t>> zeros(Configuration()->SampsPerSymbol());
    size_t dl_symbol_idx = Configuration()->Frame().GetDLSymbolIdx(symbol_id);
    if (ant_id != Configuration()->RefAnt()) {
      txbuf[ch] = zeros.data();
    } else if (dl_symbol_idx <
               Configuration()->Frame().ClientDlPilotSymbols()) {
      txbuf[ch] =
          reinterpret_cast<void*>(Configuration()->UeSpecificPilotT()[0]);
    } else {
      txbuf[ch] = reinterpret_cast<void*>(
          Configuration()
              ->DlIqT()[dl_symbol_idx -
                        Configuration()->Frame().ClientDlPilotSymbols()]);
    }
  } else {
    auto* pkt = reinterpret_cast<struct Packet*>(
        &tx_memory_[offset * Configuration()->PacketLength()]);
    txbuf[ch] = reinterpret_cast<void*>(pkt->data_);
  }

  const size_t last = Configuration()->Frame().GetDLSymbolLast();
  int flags = (symbol_id != static_cast<int>(last))
                  ? 1   // HAS_TIME
                  : 2;  // HAS_TIME & END_BURST, fixme
  long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
  radio_config_->RadioTx(ant_id / n_channels, txbuf, flags, frame_time);

  if (kDebugPrintInTask) {
    std::printf(
        "TxRxWorkerUsrp[%zu]: Transmitted frame %d, symbol %d, "
        "ant %zu, offset: %zu, msg_queue_length: %zu\n",
        tid_, frame_id, symbol_id, ant_id, offset,
        event_notify_q_->size_approx());
  }

  RtAssert(
      event_notify_q_->enqueue(notify_producer_token_,
                               EventData(EventType::kPacketTX, event.tags_[0])),
      "Socket message enqueue failed\n");
  return event.tags_[0];
}