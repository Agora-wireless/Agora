/**
 * @file txrx_worker_argos.cc
 * @brief Implementation of PacketTxRx datapath functions for communicating
 * with real Argos hardware
 */

#include "txrx_worker_argos.h"

#include <cassert>

#include "logger.h"

static constexpr bool kDebugDownlink = false;

TxRxWorkerArgos::TxRxWorkerArgos(
    size_t core_offset, size_t tid, size_t interface_count,
    size_t interface_offset, Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    RadioConfig* const radio_config)
    : TxRxWorker(core_offset, tid, interface_count, interface_offset, config,
                 rx_frame_start, event_notify_q, tx_pending_q, tx_producer,
                 notify_producer, rx_memory, tx_memory),
      radio_config_(radio_config) {}

TxRxWorkerArgos::~TxRxWorkerArgos() {}

//Main Thread Execution loop
void TxRxWorkerArgos::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  MLPD_INFO("TxRxWorkerArgos[%zu] has %zu:%zu total radios %zu\n", tid_,
            interface_offset_, (interface_offset_ + num_interfaces_) - 1,
            num_interfaces_);

  size_t rx_slot = 0;
  ssize_t prev_frame_id = -1;
  size_t local_interface = 0;
  running_ = true;
  started_ = true;

  if (num_interfaces_ == 0) {
    MLPD_WARN("LoopTxRxArgos[%zu] has no interfaces, exiting\n", tid_);
    running_ = false;
    return;
  }

  while (Configuration()->Running() == true) {
    if (0 == DequeueSend()) {
      // receive data
      auto pkts = RecvEnqueue(local_interface, rx_slot);
      if (pkts.size() > 0) {
        rx_slot = (rx_slot + pkts.size()) % rx_memory_.size();
        RtAssert(pkts.size() == channels_per_interface_,
                 "Received data but it was the wrong dimension");

        if (kIsWorkerTimingEnabled) {
          const auto frame_id = pkts.front()->frame_id_;
          if (frame_id > prev_frame_id) {
            rx_frame_start_[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
            prev_frame_id = frame_id;
          }
        }
        local_interface++;
        if (local_interface == num_interfaces_) {
          local_interface = 0;
        }
      }  // if pkts.size() != 0
    }    // if DequeueSend() == -1
  }      // while Configuration()->Running() == true
  running_ = false;
}

//RX data
std::vector<Packet*> TxRxWorkerArgos::RecvEnqueue(size_t interface_id,
                                                  size_t rx_slot) {
  const size_t global_interface_id = interface_id + interface_offset_;
  const size_t packet_length = Configuration()->DlPacketLength();

  size_t ant_id = global_interface_id * channels_per_interface_;
  std::vector<size_t> ant_ids(channels_per_interface_);
  std::vector<void*> samp(channels_per_interface_);

  //Check for available memory
  for (size_t ch = 0; ch < channels_per_interface_; ++ch) {
    RxPacket& rx = rx_memory_.at(rx_slot + ch);

    // if rx_buffer is full, exit
    if (rx.Empty() == false) {
      MLPD_ERROR("TxRxWorkerArgos[%zu] rx_buffer full, rx slot: %zu\n", tid_,
                 rx_slot);
      Configuration()->Running(false);
      break;
    }
    ant_ids.at(ch) = ant_id + ch;
    samp.at(ch) = rx.RawPacket()->data_;
  }

  long long frame_time;
  int rx_status =
      radio_config_->RadioRx(global_interface_id, samp.data(), frame_time);

  if (rx_status <= 0) {
    if (rx_status < 0) {
      MLPD_ERROR("RX status = %d is less than 0\n", rx_status);
    }
    std::vector<Packet*> empty_pkt;
    return empty_pkt;
  } else if (rx_status != Configuration()->SampsPerSymbol()) {
    MLPD_ERROR("RX status = %d is not the expected value\n", rx_status);
  }  /// This check is redundant

  size_t frame_id = (size_t)(frame_time >> 32);
  size_t symbol_id = (size_t)((frame_time >> 16) & 0xFFFF);
  std::vector<size_t> symbol_ids(channels_per_interface_, symbol_id);

  /// \TODO: What if ref_ant is set to the second channel?
  if ((Configuration()->Frame().IsRecCalEnabled() == true) &&
      (Configuration()->IsCalDlPilot(frame_id, symbol_id) == true) &&
      (global_interface_id == Configuration()->RefRadio()) &&
      (Configuration()->AntPerGroup() > 1)) {
    if (Configuration()->AntPerGroup() > channels_per_interface_) {
      symbol_ids.resize(Configuration()->AntPerGroup());
      ant_ids.resize(Configuration()->AntPerGroup());
    }
    for (size_t s = 1; s < Configuration()->AntPerGroup(); s++) {
      RxPacket& rx = rx_memory_.at(rx_slot + s);

      std::vector<void*> tmp_samp(channels_per_interface_);
      std::vector<char> dummy_buff(packet_length);
      tmp_samp.at(0) = rx.RawPacket()->data_;
      tmp_samp.at(1) = dummy_buff.data();
      if ((Configuration()->Running() == false) ||
          radio_config_->RadioRx(global_interface_id, tmp_samp.data(),
                                 frame_time) <= 0) {
        std::vector<Packet*> empty_pkt;
        return empty_pkt;
      }
      symbol_ids.at(s) = Configuration()->Frame().GetDLCalSymbol(s);
      ant_ids.at(s) = ant_id;
    }
  }

  std::vector<Packet*> pkt;
  for (size_t ch = 0; ch < symbol_ids.size(); ++ch) {
    RxPacket& rx = rx_memory_.at(rx_slot + ch);
    pkt.push_back(rx.RawPacket());
    new (rx.RawPacket())
        Packet(frame_id, symbol_ids.at(ch), 0 /* cell_id */, ant_ids.at(ch));

    rx.Use();
    // Push kPacketRX event into the queue.
    EventData rx_message(EventType::kPacketRX, rx_tag_t(rx).tag_);

    if (event_notify_q_->enqueue(notify_producer_token_, rx_message) == false) {
      std::printf("TxRxWorkerArgos[%zu]: socket message enqueue failed\n",
                  tid_);
      throw std::runtime_error(
          "TxRxWorkerArgos: socket message enqueue failed");
    }
  }
  return pkt;
}

//Tx data
size_t TxRxWorkerArgos::DequeueSend() {
  const size_t channels_per_interface = Configuration()->NumChannels();
  const size_t max_dequeue_items = num_interfaces_ * channels_per_interface;
  std::vector<EventData> events(max_dequeue_items);

  //Single producer ordering in q is preserved
  const size_t dequeued_items = tx_pending_q_->try_dequeue_bulk_from_producer(
      tx_producer_token_, events.data(), max_dequeue_items);

  for (size_t item = 0; item < dequeued_items; item++) {
    EventData& current_event = events.at(item);

    // std::printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(current_event.event_type_ == EventType::kPacketTX);

    size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    const size_t symbol_id = gen_tag_t(current_event.tags_[0u]).symbol_id_;
    const size_t ant_id = gen_tag_t(current_event.tags_[0u]).ant_id_;
    const size_t radio_id = ant_id / channels_per_interface;

    RtAssert((radio_id >= interface_offset_) &&
             (radio_id <= (interface_offset_ + num_interfaces_)));

    //See if this is the last antenna on the radio.  Assume that we receive the last one
    // last (and all the others came before).  No explicit tracking
    const bool last_antenna =
        ((ant_id % channels_per_interface) + 1) == (channels_per_interface);

    std::printf("TxRxWorkerArgos[%zu]: tx antenna %zu radio %zu is last %d\n",
                tid_, ant_id, radio_id, last_antenna);

    const size_t dl_symbol_idx =
        Configuration()->Frame().GetDLSymbolIdx(symbol_id);
    const size_t offset =
        (Configuration()->GetTotalDataSymbolIdxDl(frame_id, dl_symbol_idx) *
         Configuration()->BsAntNum()) +
        ant_id;

    if (last_antenna) {
      // Transmit downlink calibration (array to ref) pilot
      std::vector<void*> caltxbuf(Configuration()->NumChannels());
      if ((Configuration()->Frame().IsRecCalEnabled() == true) &&
          (symbol_id == Configuration()->Frame().GetDLSymbol(0)) &&
          (radio_id != Configuration()->RefRadio())) {
        std::vector<std::complex<int16_t>> zeros(
            Configuration()->SampsPerSymbol(), std::complex<int16_t>(0, 0));
        for (size_t s = 0; s < Configuration()->RadioPerGroup(); s++) {
          if (radio_id != Configuration()->RefRadio()) {
            bool calib_turn =
                (frame_id % Configuration()->RadioGroupNum() ==
                     radio_id / Configuration()->RadioPerGroup() &&
                 s == radio_id % Configuration()->RadioPerGroup());
            for (size_t ch = 0; ch < channels_per_interface; ch++) {
              caltxbuf.at(ch) = calib_turn ? Configuration()->PilotCi16().data()
                                           : zeros.data();
              if (channels_per_interface > 1) {
                caltxbuf.at(1 - ch) = zeros.data();
              }
              long long frame_time =
                  ((long long)(frame_id + TX_FRAME_DELTA) << 32) |
                  (Configuration()->Frame().GetDLCalSymbol(
                       s * channels_per_interface + ch)
                   << 16);
              radio_config_->RadioTx(radio_id, caltxbuf.data(), 1, frame_time);
            }
          } else {
            caltxbuf.at(0) = Configuration()->PilotCi16().data();
            if (channels_per_interface > 1) {
              caltxbuf.at(1) = zeros.data();
            }
            long long frame_time =
                ((long long)(frame_id + TX_FRAME_DELTA) << 32) |
                (Configuration()->Frame().GetULCalSymbol(0) << 16);
            radio_config_->RadioTx(radio_id, caltxbuf.data(), 1, frame_time);
          }
        }
      }

      std::vector<void*> txbuf(channels_per_interface);
      if (kDebugDownlink == true) {
        std::vector<std::complex<int16_t>> zeros(
            Configuration()->SampsPerSymbol());
        for (size_t ch = 0; ch < channels_per_interface; ch++) {
          if (ant_id != 0) {
            txbuf.at(ch) = zeros.data();
          } else if (dl_symbol_idx <
                     Configuration()->Frame().ClientDlPilotSymbols()) {
            txbuf.at(ch) =
                reinterpret_cast<void*>(Configuration()->UeSpecificPilotT()[0]);
          } else {
            txbuf.at(ch) = reinterpret_cast<void*>(
                Configuration()->DlIqT()[dl_symbol_idx]);
          }
          if (channels_per_interface > 1) {
            txbuf.at(1 - ch) = zeros.data();
          }
        }
      } else {
        for (size_t ch = 0; ch < channels_per_interface; ch++) {
          auto* pkt = reinterpret_cast<Packet*>(
              &tx_memory_[((offset + ch) * Configuration()->DlPacketLength())]);
          txbuf.at(ch) = reinterpret_cast<void*>(pkt->data_);
        }
      }

      const size_t last = Configuration()->Frame().GetDLSymbolLast();
      const int flags = (symbol_id != last) ? 1   // HAS_TIME
                                            : 2;  // HAS_TIME & END_BURST, fixme
      frame_id += TX_FRAME_DELTA;
      long long frame_time = ((long long)frame_id << 32) | (symbol_id << 16);
      radio_config_->RadioTx(radio_id, txbuf.data(), flags, frame_time);
    }

    if (kDebugPrintInTask == true) {
      std::printf(
          "TxRxWorkerArgos[%zu]: Transmitted frame %zu, symbol %zu, "
          "ant %zu, offset: %zu, msg_queue_length: %zu\n",
          tid_, frame_id, symbol_id, ant_id, offset,
          event_notify_q_->size_approx());
    }

    RtAssert(event_notify_q_->enqueue(
                 notify_producer_token_,
                 EventData(EventType::kPacketTX, current_event.tags_[0])),
             "Socket message enqueue failed\n");
  }
  return dequeued_items;
}