/**
 * @file txrx_worker_sim.cc
 * @brief Implementation of PacketTxRx initialization functions, and datapath
 * functions for communicating with simulators.
 */

#include "txrx_worker_sim.h"

#include <cassert>

#include "gettime.h"
#include "logger.h"
#include "message.h"

static constexpr bool kEnableSlowStart = true;
static constexpr bool kDebugPrintBeacon = false;

static constexpr size_t kSlowStartThresh1 = kFrameWnd;
static constexpr size_t kSlowStartThresh2 = (kFrameWnd * 4);
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
    std::vector<RxPacket>& rx_memory, std::byte* const tx_memory,
    std::mutex& sync_mutex, std::condition_variable& sync_cond,
    std::atomic<bool>& can_proceed)
    : TxRxWorker(core_offset, tid, interface_count, interface_offset,
                 config->NumChannels(), config, rx_frame_start, event_notify_q,
                 tx_pending_q, tx_producer, notify_producer, rx_memory,
                 tx_memory, sync_mutex, sync_cond, can_proceed) {
  for (size_t interface = 0; interface < num_interfaces_; ++interface) {
    const uint16_t local_port_id =
        config->BsServerPort() + interface + interface_offset_;
    const uint16_t rem_port_id =
        config->BsRruPort() + interface + interface_offset_;

    udp_comm_.emplace_back(std::make_unique<UDPComm>(
        config->BsServerAddr(), local_port_id, kSocketRxBufferSize, 0));
    udp_comm_.back()->Connect(config->BsRruAddr(), rem_port_id);

    AGORA_LOG_FRAME(
        "TxRxWorkerSim[%zu]: set up UDP socket server listening to %s:%d "
        " sending to %s:%d\n",
        tid_, config->BsServerAddr().c_str(), local_port_id,
        config->BsRruAddr().c_str(), rem_port_id);
  }
  beacon_buffer_.resize(config->PacketLength());
}

TxRxWorkerSim::~TxRxWorkerSim() = default;

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

  const size_t slow_start_tsc2 = kSlowStartMulStage2 * frame_tsc_delta;
  size_t delay_tsc = frame_tsc_delta;

  if (kEnableSlowStart) {
    delay_tsc = slow_start_tsc1;
  }

  size_t prev_frame_id = SIZE_MAX;
  size_t tx_frame_id = 0;
  size_t thread_local_interface = 0;
  running_ = true;
  WaitSync();

  size_t tx_frame_start = GetTime::Rdtsc();
  size_t send_time = delay_tsc + tx_frame_start;

  // Send Beacons for the first time to kick off sim
  // SendBeacon(tid, tx_frame_id++);
  while (Configuration()->Running() == true) {
    const size_t rdtsc_now = GetTime::Rdtsc();

    if (rdtsc_now > send_time) {
      AGORA_LOG_SYMBOL(
          "TxRxWorkerSim[%zu]: sending beacon for frame %zu at time %zu\n",
          tid_, tx_frame_id, rdtsc_now);
      SendBeacon(tx_frame_id++);

      if (kEnableSlowStart) {
        if (tx_frame_id == kSlowStartThresh1) {
          delay_tsc = slow_start_tsc2;
          AGORA_LOG_TRACE(
              "TxRxWorkerSim[%zu]: increasing beacon rate at frame %zu time "
              "%zu\n",
              tid_, kSlowStartThresh1, rdtsc_now);
        } else if (tx_frame_id == kSlowStartThresh2) {
          delay_tsc = frame_tsc_delta;
          AGORA_LOG_TRACE(
              "TxRxWorkerSim[%zu]: increasing beacon rate to full speed at "
              "frame %zu time %zu\n",
              tid_, kSlowStartThresh2, rdtsc_now);
        }
      }
      tx_frame_start = send_time;
      send_time += delay_tsc;
    }

    const size_t send_result = DequeueSend();
    if (0 == send_result) {
      // receive data
      // Need to get NumChannels data here
      const auto rx_packets = RecvEnqueue(thread_local_interface);
      for (const auto& packet : rx_packets) {
        if (kIsWorkerTimingEnabled) {
          const uint32_t frame_id = packet->frame_id_;
          if (frame_id != prev_frame_id) {
            rx_frame_start_[frame_id % kNumStatsFrames] = GetTime::Rdtsc();
            prev_frame_id = frame_id;
          }
        }
      }

      thread_local_interface++;
      if (thread_local_interface == num_interfaces_) {
        thread_local_interface = 0;
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
    std::printf(
        "TxRxWorkerSim [%zu]: Sending beacon for frame %zu tx delta %f ms\n",
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

        udp_comm_.at(interface)->Send(beacon_buffer_.data(),
                                      beacon_buffer_.size());
      }
    }
  }
}

std::vector<Packet*> TxRxWorkerSim::RecvEnqueue(size_t interface_id) {
  std::vector<Packet*> rx_packets;
  const size_t packet_length = Configuration()->PacketLength();

  RxPacket& rx_placement = GetRxPacket();
  Packet* pkt = rx_placement.RawPacket();

  ssize_t rx_bytes =
      udp_comm_.at(interface_id)
          ->Recv(reinterpret_cast<std::byte*>(pkt), packet_length);
  if (rx_bytes == static_cast<ssize_t>(packet_length)) {
    if (kDebugPrintInTask) {
      std::printf("TxRxWorkerSim[%zu]: Received frame %d, symbol %d, ant %d\n",
                  tid_, pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
    }
    if (kDebugMulticell) {
      std::printf(
          "Before packet combining: receiving data stream from the antenna "
          "%d in cell %d,\n",
          pkt->ant_id_, pkt->cell_id_);
    }
    pkt->ant_id_ += pkt->cell_id_ *
                    (Configuration()->BsAntNum() / Configuration()->NumCells());
    if (kDebugMulticell) {
      std::printf(
          "After packet combining: the combined antenna ID is %d, it comes "
          "from the cell %d\n",
          pkt->ant_id_, pkt->cell_id_);
    }
    // Push kPacketRX event into the queue.
    EventData rx_message(EventType::kPacketRX, rx_tag_t(rx_placement).tag_);
    NotifyComplete(rx_message);
    rx_packets.push_back(pkt);
  } else if (0 > rx_bytes) {
    AGORA_LOG_ERROR("RecvEnqueue: Udp Recv failed with error\n");
    throw std::runtime_error("TxRxWorkerSim: recv failed");
  } else if (0 != rx_bytes) {
    AGORA_LOG_ERROR(
        "RecvEnqueue: Udp Recv failed to receive all expected bytes");
    throw std::runtime_error(
        "PacketTxRx::RecvEnqueue: Udp Recv failed to receive all expected "
        "bytes");
  } else {
    // Received 0 bytes.  Time to recycle the buffer
    ReturnRxPacket(rx_placement);
  }
  return rx_packets;
}

//Function of the TxRx thread
size_t TxRxWorkerSim::DequeueSend() {
  auto tx_events = GetPendingTxEvents();

  //Process each pending tx event
  for (const EventData& current_event : tx_events) {
    // std::printf("tx queue length: %d\n", tx_pending_q_->size_approx());
    assert(current_event.event_type_ == EventType::kPacketTX);

    const size_t frame_id = gen_tag_t(current_event.tags_[0u]).frame_id_;
    const size_t symbol_id = gen_tag_t(current_event.tags_[0u]).symbol_id_;
    const size_t ant_id = gen_tag_t(current_event.tags_[0u]).ant_id_;
    const size_t interface_id = ant_id / channels_per_interface_;

    assert((interface_id >= interface_offset_) &&
           (interface_id <= (num_interfaces_ + interface_offset_)));

    auto* pkt = GetTxPacket(frame_id, symbol_id, ant_id);
    new (pkt) Packet(frame_id, symbol_id,
                     Configuration()->CellId().at(interface_id), ant_id);

    if (kDebugPrintInTask) {
      std::printf(
          "TxRxWorkerSim[%zu]::DequeueSend() Transmitted frame %zu, symbol "
          "%zu, ant %zu, tag %zu\n",
          tid_, frame_id, symbol_id, ant_id,
          gen_tag_t(current_event.tags_[0]).tag_);
    }

    if (kDebugDownlink == true) {
      const size_t data_symbol_idx_dl =
          Configuration()->Frame().GetDLSymbolIdx(symbol_id);

      if (ant_id != 0) {
        std::memset(pkt->data_, 0,
                    Configuration()->SampsPerSymbol() * sizeof(int16_t) * 2);
      } else if (data_symbol_idx_dl <
                 Configuration()->Frame().ClientDlPilotSymbols()) {
        std::memcpy(pkt->data_, Configuration()->UeSpecificPilotT()[0],
                    Configuration()->SampsPerSymbol() * sizeof(int16_t) * 2);
      } else {
        std::memcpy(pkt->data_, Configuration()->DlIqT()[data_symbol_idx_dl],
                    Configuration()->SampsPerSymbol() * sizeof(int16_t) * 2);
      }
    }

    const size_t local_interface_idx = interface_id - interface_offset_;
    // Send data (one OFDM symbol)
    udp_comm_.at(local_interface_idx)
        ->Send(reinterpret_cast<std::byte*>(pkt),
               Configuration()->DlPacketLength());

    const auto complete_event =
        EventData(EventType::kPacketTX, current_event.tags_[0]);
    NotifyComplete(complete_event);
  }
  return tx_events.size();
}