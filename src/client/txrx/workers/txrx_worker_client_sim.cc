/**
 * @file txrx_worker_client_sim.cc
 * @brief Implementation of PacketTxRx initialization functions, and datapath
 * functions for communicating with simulators in the user equptiment.
 */

#include "txrx_worker_client_sim.h"

#include <cassert>

#include "logger.h"

static constexpr bool kEnableSlowStart = true;
static constexpr size_t kSocketRxBufferSize = (1024 * 1024 * 64 * 8) - 1;

TxRxWorkerClientSim::TxRxWorkerClientSim(
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
                 config->NumUeChannels(), config, rx_frame_start,
                 event_notify_q, tx_pending_q, tx_producer, notify_producer,
                 rx_memory, tx_memory, sync_mutex, sync_cond, can_proceed) {
  for (size_t interface = 0; interface < num_interfaces_; interface++) {
    const uint16_t local_port_id =
        config->UeServerPort() + interface + interface_offset_;

    udp_servers_.emplace_back(
        std::make_unique<UDPServer>(local_port_id, kSocketRxBufferSize));
    udp_clients_.emplace_back(std::make_unique<UDPClient>());
    MLPD_FRAME(
        "TxRxWorkerClientSim[%zu]: set up UDP socket server listening "
        "to local port %d\n",
        tid_, local_port_id);
  }

  tx_pkt_zeros_ = std::vector<char*>(config->PacketLength(), 0);
  tx_pkt_pilot_ = std::vector<char*>(config->PacketLength(), 0);
  std::memcpy(&tx_pkt_pilot_.at(Packet::kOffsetOfData),
              config->PilotCi16().data(),
              config->PacketLength() - Packet::kOffsetOfData);
}

TxRxWorkerClientSim::~TxRxWorkerClientSim() = default;

//Main Thread Execution loop
void TxRxWorkerClientSim::DoTxRx() {
  PinToCoreWithOffset(ThreadType::kWorkerTXRX, core_offset_, tid_);

  size_t prev_frame_id = SIZE_MAX;
  size_t thread_local_interface = 0;
  running_ = true;
  WaitSync();

  // Send Beacons for the first time to kick off sim
  while (Configuration()->Running() == true) {
    const size_t send_result = DequeueSend();
    if (0 == send_result) {
      // receive data
      // Need to get NumChannels data here
      const auto rx_packets = RecvEnqueue(thread_local_interface);
      for (auto& packet : rx_packets) {
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

//Combine with BS code, getting the entire packet / symbol...
std::vector<Packet*> TxRxWorkerClientSim::RecvEnqueue(size_t interface_id) {
  std::vector<Packet*> rx_packets;
  const size_t packet_length = Configuration()->PacketLength();

  RxPacket& rx_placement = GetRxPacket();
  Packet* pkt = rx_placement.RawPacket();

  ssize_t rx_bytes = udp_servers_.at(interface_id)
                         ->Recv(reinterpret_cast<uint8_t*>(pkt), packet_length);
  if (rx_bytes == static_cast<ssize_t>(packet_length)) {
    if (kDebugPrintInTask) {
      std::printf(
          "TxRxWorkerClientSim[%zu]: Received frame %d, symbol %d, ant %d\n",
          tid_, pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
    }
    // Push kPacketRX event into the queue.
    EventData rx_message(EventType::kPacketRX, rx_tag_t(rx_placement).tag_);
    NotifyComplete(rx_message);
    rx_packets.push_back(pkt);
  } else if (0 > rx_bytes) {
    MLPD_ERROR("RecvEnqueue: Udp Recv failed with error\n");
    throw std::runtime_error("TxRxWorkerClientSim: recv failed");
  } else if (0 != rx_bytes) {
    MLPD_ERROR("RecvEnqueue: Udp Recv failed to receive all expected bytes");
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
size_t TxRxWorkerClientSim::DequeueSend() {
  auto tx_events = GetPendingTxEvents();

  //Process each pending tx event
  for (const EventData& current_event : tx_events) {
    RtAssert((current_event.event_type_ == EventType::kPacketTX) ||
                 (current_event.event_type_ == EventType::kPacketPilotTX),
             "RadioTxRx: Wrong Event Type in TX Queue!");

    //Handle Pilot symbols first, then send the client data.
    const size_t frame_id = gen_tag_t(current_event.tags_[0]).frame_id_;
    const size_t ue_id = gen_tag_t(current_event.tags_[0]).ue_id_;

    assert((ue_id >= interface_offset_) &&
           (ue_id <= (num_interfaces_ + interface_offset_)));

    const size_t local_interface = ue_id - interface_offset_;

    for (size_t channel = 0; channel < channels_per_interface_; channel++) {
      const size_t ant_id = (ue_id * channels_per_interface_) + channel;
      // Transmit pilot symbols on each UE channel
      for (size_t symbol_idx = 0;
           symbol_idx < Configuration()->Frame().NumPilotSyms(); symbol_idx++) {
        if (kDebugPrintInTask) {
          std::printf(
              "TxRxWorkerClientSim[%zu]: Transmitted pilot in frame %zu, "
              "symbol %zu, ant %zu\n",
              tid_, frame_id,
              Configuration()->Frame().GetPilotSymbol(symbol_idx), ant_id);
        }
        Packet* tx_packet = nullptr;
        if (symbol_idx == ant_id) {
          tx_packet = reinterpret_cast<Packet*>(tx_pkt_pilot_.data());
        } else {
          tx_packet = reinterpret_cast<Packet*>(tx_pkt_zeros_.data());
        }

        //Fill out the frame / symbol / cell / ant
        new (tx_packet) Packet(
            frame_id, Configuration()->Frame().GetPilotSymbol(symbol_idx),
            0 /* cell_id */, ant_id);

        udp_clients_.at(local_interface)
            ->Send(Configuration()->BsRruAddr(),
                   Configuration()->UeRruPort() + ant_id,
                   reinterpret_cast<uint8_t*>(tx_packet),
                   Configuration()->PacketLength());
      }

      if (current_event.event_type_ == EventType::kPacketTX) {
        for (size_t symbol_id = 0;
             symbol_id < Configuration()->Frame().NumULSyms(); symbol_id++) {
          if (kDebugPrintInTask) {
            std::printf(
                "TxRxWorkerClientSim[%zu]: Transmitted frame %zu, data symbol "
                "%zu, ant %zu, tag %zu\n",
                tid_, frame_id, Configuration()->Frame().GetULSymbol(symbol_id),
                ant_id, gen_tag_t(current_event.tags_[0]).tag_);
          }

          //auto* pkt = (Packet*)(tx_buffer_ + offset * c->PacketLength());
          auto* tx_pkt = GetUlTxPacket(frame_id, symbol_id, ant_id);
          new (tx_pkt)
              Packet(frame_id, Configuration()->Frame().GetULSymbol(symbol_id),
                     0 /* cell_id */, ant_id);

          // Send data (one OFDM symbol)
          udp_clients_.at(local_interface)
              ->Send(Configuration()->BsRruAddr(),
                     Configuration()->UeRruPort() + ant_id,
                     reinterpret_cast<uint8_t*>(tx_pkt),
                     Configuration()->PacketLength());
        }
      }  // event.event_type_ == EventType::kPacketTX
    }    // foreach channel

    EventData complete_event;
    if (current_event.event_type_ == EventType::kPacketPilotTX) {
      complete_event =
          EventData(EventType::kPacketPilotTX, current_event.tags_[0]);
    } else if (current_event.event_type_ == EventType::kPacketTX) {
      complete_event = EventData(EventType::kPacketTX, current_event.tags_[0]);
    } else {
      RtAssert(false, "Invalid type of event");
    }
    NotifyComplete(complete_event);
  }
  return tx_events.size();
}