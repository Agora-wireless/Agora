/**
 * @file txrx_worker.cc
 * @brief Implementation of PacketTxRx initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "txrx_worker_sim.h"

#include <cassert>

#include "logger.h"

static constexpr bool kEnableSlowStart = true;
static constexpr bool kEnableSlowSending = false;
static constexpr bool kDebugPrintBeacon = false;

static constexpr size_t kSlowStartMulStage1 = 32;
static constexpr size_t kSlowStartMulStage2 = 8;

static constexpr size_t kSocketRxBufferSize = (1024 * 1024 * 64 * 8) - 1;

TxRxWorkerSim::TxRxWorkerSim(
    size_t core_offset, size_t tid, size_t interface_count,
    size_t interface_offset, Config* const config, size_t* rx_frame_start,
    moodycamel::ConcurrentQueue<EventData>* event_notify_q,
    moodycamel::ConcurrentQueue<EventData>* tx_pending_q,
    moodycamel::ProducerToken& tx_producer,
    moodycamel::ProducerToken& notify_producer,
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory)
    : TxRxWorker(core_offset, tid, interface_count, interface_offset, config,
                 rx_frame_start, event_notify_q, tx_pending_q, tx_producer,
                 notify_producer, rx_memory, tx_memory) {
  for (size_t interface = 0; interface < num_interfaces_; ++interface) {
    const uint16_t local_port_id =
        config->BsServerPort() + interface + interface_offset_;

    udp_servers_.emplace_back(
        std::make_unique<UDPServer>(local_port_id, kSocketRxBufferSize));
    udp_clients_.emplace_back(std::make_unique<UDPClient>());
    MLPD_FRAME(
        "TXRX thread [%zu]: set up UDP socket server listening to local port "
        "%d\n",
        tid_, local_port_id);
  }
  beacon_buffer_.resize(config->PacketLength());
}

TxRxWorkerSim::~TxRxWorkerSim() {}

//Main Thread Execution loop
void TxRxWorkerSim::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  const double rdtsc_freq = GetTime::MeasureRdtscFreq();
  const size_t frame_tsc_delta =
      Configuration()->GetFrameDurationSec() * 1e9f * rdtsc_freq;
  const size_t two_hundred_ms_ticks = (0.2f /* 200 ms */ * 1e9f * rdtsc_freq);

  // Slow start variables (Start with no less than 200 ms)
  const size_t slow_start_tsc1 =
      std::max(kSlowStartMulStage1 * frame_tsc_delta, two_hundred_ms_ticks);

  const size_t slow_start_thresh1 = kFrameWnd;
  const size_t slow_start_tsc2 = kSlowStartMulStage2 * frame_tsc_delta;
  const size_t slow_start_thresh2 = kFrameWnd * 4;
  size_t delay_tsc = frame_tsc_delta;

  if (kEnableSlowStart) {
    delay_tsc = slow_start_tsc1;
  }

  size_t prev_frame_id = SIZE_MAX;
  size_t rx_slot = 0;
  size_t tx_frame_start = GetTime::Rdtsc();
  size_t tx_frame_id = 0;
  size_t send_time = delay_tsc + tx_frame_start;
  size_t current_interface = 0;

  running_ = true;
  started_ = true;
  // Send Beacons for the first time to kick off sim
  // SendBeacon(tid, tx_frame_id++);
  while (Configuration()->Running() == true) {
    size_t rdtsc_now = GetTime::Rdtsc();

    if (rdtsc_now > send_time) {
      SendBeacon(tx_frame_id++);

      if (kEnableSlowStart) {
        if (tx_frame_id == slow_start_thresh1) {
          delay_tsc = slow_start_tsc2;
        } else if (tx_frame_id == slow_start_thresh2) {
          delay_tsc = frame_tsc_delta;
          if (kEnableSlowSending) {
            // Temp for historic reasons
            delay_tsc = frame_tsc_delta * 4;
          }
        }
      }
      tx_frame_start = send_time;
      send_time += delay_tsc;
    }

    const size_t send_result = DequeueSend();
    if (0 == send_result) {
      // receive data
      // Need to get NumChannels data here
      Packet* pkt = RecvEnqueue(rx_memory_.at(rx_slot), current_interface);
      if (pkt != nullptr) {
        rx_slot = (rx_slot + 1) % rx_memory_.size();

        if (kIsWorkerTimingEnabled) {
          uint32_t frame_id = pkt->frame_id_;
          if (frame_id != prev_frame_id) {
            rx_frame_start_[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
            prev_frame_id = frame_id;
          }
        }

        if (++current_interface == num_interfaces_) {
          current_interface = 0;
        }
      }
    }  // end if -1 == send_result
  }    // end while
  running_ = false;
}

void TxRxWorkerSim::SendBeacon(size_t frame_id) {
  const double time_now = GetTime::GetTimeUs() / 1000.0f;

  // Send a beacon packet in the downlink to trigger user pilot
  auto* pkt = reinterpret_cast<Packet*>(beacon_buffer_.data());

  if (kDebugPrintBeacon) {
    std::printf("TXRX [%zu]: Sending beacon for frame %zu tx delta %f ms\n",
                tid_, frame_id, time_now - beacon_send_time_);
  }
  beacon_send_time_ = time_now;

  for (size_t beacon_sym = 0u;
       beacon_sym < Configuration()->Frame().NumBeaconSyms(); beacon_sym++) {
    for (size_t interface = 0u; interface < num_interfaces_; interface++) {
      const size_t global_interface_id = interface + interface_offset_;
      for (size_t channel = 0u; channel < channels_per_interface_; channel++) {
        const size_t ant_id =
            ((global_interface_id * channels_per_interface_) + channel);
        new (pkt) Packet(frame_id,
                         Configuration()->Frame().GetBeaconSymbol(beacon_sym),
                         0 /* cell_id */, ant_id);

        udp_clients_.at(interface)->Send(
            Configuration()->BsRruAddr(),
            Configuration()->BsRruPort() + global_interface_id,
            beacon_buffer_.data(), beacon_buffer_.size());
      }
    }
  }
}

Packet* TxRxWorkerSim::RecvEnqueue(RxPacket& rx_placement,
                                   size_t interface_id) {
  const size_t packet_length = Configuration()->PacketLength();

  // if rx_buffer is full, exit
  if (rx_placement.Empty() == false) {
    MLPD_ERROR("PacketTxRx [%zu]: rx_buffer full\n", tid_);
    Configuration()->Running(false);
    return (nullptr);
  }
  Packet* pkt = rx_placement.RawPacket();

  ssize_t rx_bytes = udp_servers_.at(interface_id)
                         ->Recv(reinterpret_cast<uint8_t*>(pkt), packet_length);
  if (0 > rx_bytes) {
    MLPD_ERROR("RecvEnqueue: Udp Recv failed with error\n");
    throw std::runtime_error("PacketTxRx: recv failed");
  } else if (rx_bytes == 0) {
    pkt = nullptr;
  } else if (static_cast<size_t>(rx_bytes) == packet_length) {
    if (kDebugPrintInTask) {
      std::printf("In TXRX thread %zu: Received frame %d, symbol %d, ant %d\n",
                  tid_, pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
    }
    if (kDebugMulticell) {
      std::printf(
          "Before packet combining: receiving data stream from the antenna "
          "%d in cell %d,\n",
          pkt->ant_id_, pkt->cell_id_);
    }
    pkt->ant_id_ += pkt->cell_id_ * ant_per_cell_;
    if (kDebugMulticell) {
      std::printf(
          "After packet combining: the combined antenna ID is %d, it comes "
          "from the cell %d\n",
          pkt->ant_id_, pkt->cell_id_);
    }

    // Push kPacketRX event into the queue.
    rx_placement.Use();
    EventData rx_message(EventType::kPacketRX, rx_tag_t(rx_placement).tag_);
    if (event_notify_q_->enqueue(notify_producer_token_, rx_message) == false) {
      MLPD_ERROR("socket message enqueue failed\n");
      throw std::runtime_error("PacketTxRx: socket message enqueue failed");
    }
  } else {
    MLPD_ERROR("RecvEnqueue: Udp Recv failed to receive all expected bytes");
    throw std::runtime_error(
        "PacketTxRx::RecvEnqueue: Udp Recv failed to receive all expected "
        "bytes");
  }
  return pkt;
}

//Function of the TxRx thread
size_t TxRxWorkerSim::DequeueSend() {
  const size_t channels_per_interface = Configuration()->NumChannels();
  const size_t max_dequeue_items = num_interfaces_ * channels_per_interface;
  std::vector<EventData> events(max_dequeue_items);

  //Single producer ordering in q is preserved
  const size_t dequeued_items = tx_pending_q_->try_dequeue_bulk_from_producer(
      tx_producer_token_, events.data(), max_dequeue_items);

  for (size_t item = 0; item < dequeued_items; item++) {
    EventData& current_event = events.at(item);

    // std::printf("tx queue length: %d\n", tx_pending_q_->size_approx());
    assert(current_event.event_type_ == EventType::kPacketTX);

    const size_t ant_id = gen_tag_t(current_event.tags_[0u]).ant_id_;
    const size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    const size_t symbol_id = gen_tag_t(current_event.tags_[0u]).symbol_id_;
    const size_t interface_id = ant_id / channels_per_interface_;

    assert((interface_id >= interface_offset_) &&
           (interface_id <= (num_interfaces_ + interface_offset_)));

    const size_t data_symbol_idx_dl =
        Configuration()->Frame().GetDLSymbolIdx(symbol_id);
    const size_t offset = (Configuration()->GetTotalDataSymbolIdxDl(
                               frame_id, data_symbol_idx_dl) *
                           Configuration()->BsAntNum()) +
                          ant_id;

    if (kDebugPrintInTask) {
      std::printf(
          "PacketTxRx:DequeueSend [%zu]: Transmitted frame %zu, symbol %zu, "
          "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
          tid_, frame_id, symbol_id, ant_id,
          gen_tag_t(current_event.tags_[0]).tag_, offset,
          event_notify_q_->size_approx());
    }

    auto* pkt = reinterpret_cast<Packet*>(
        &tx_memory_[offset * Configuration()->DlPacketLength()]);
    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

    const size_t local_interface_id = interface_id - interface_offset_;
    // Send data (one OFDM symbol)
    udp_clients_.at(local_interface_id)
        ->Send(Configuration()->BsRruAddr(),
               Configuration()->BsRruPort() + interface_id,
               reinterpret_cast<uint8_t*>(pkt),
               Configuration()->DlPacketLength());

    //Send tx completion event
    RtAssert(event_notify_q_->enqueue(
                 notify_producer_token_,
                 EventData(EventType::kPacketTX, current_event.tags_[0])),
             "Socket message enqueue failed\n");
  }
  return dequeued_items;
}