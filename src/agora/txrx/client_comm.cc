/**
 * @file client_comm.cc
 * @brief Implementation file for the WiredControlChannel class.
 */
#include "client_comm.h"

#include "gettime.h"
#include "logger.h"

// ticks every ~1s
WiredControlChannel::WiredControlChannel(
    Config* cfg, size_t rx_core_offset, size_t tx_core_offset,
    std::string local_addr, size_t local_port, std::string remote_addr,
    size_t remote_port, moodycamel::ConcurrentQueue<EventData>* rx_queue,
    moodycamel::ConcurrentQueue<EventData>* tx_queue,
    moodycamel::ProducerToken* rx_producer,
    moodycamel::ProducerToken* tx_producer, char* rx_socket_buf,
    size_t rx_pkt_num, const std::string& log_filename)
    : cfg_(cfg),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      tsc_delta_((cfg_->GetFrameDurationSec() * 1e9) / freq_ghz_),
      rx_core_offset_(rx_core_offset),
      tx_core_offset_(tx_core_offset),
      pkt_tracker_(0),
      rx_queue_(rx_queue),
      tx_queue_(tx_queue),
      rx_producer_(rx_producer),
      tx_producer_(tx_producer),
      rx_socket_buf_(rx_socket_buf),
      max_pkt_cnt_(rx_pkt_num) {
  // Set up log file
  if (log_filename.empty() == false) {
    log_filename_ = log_filename;  // Use a non-default log filename
  } else {
    log_filename_ = kDefaultLogFilename;
  }
  log_file_ = std::fopen(log_filename_.c_str(), "w");
  RtAssert(log_file_ != nullptr, "Failed to open DYNAMIC CORE log file");

  AGORA_LOG_INFO("WiredControlChannel: Frame duration %.2f ms, tsc_delta %zu\n",
                 cfg_->GetFrameDurationSec() * 1000, tsc_delta_);

  const size_t udp_pkt_len = cfg_->PacketLength();
  udp_pkt_buf_.resize(udp_pkt_len);
  for (size_t buffer = 0; buffer < max_pkt_cnt_; buffer++) {
    auto* pkt_loc = reinterpret_cast<Packet*>(
        &rx_socket_buf_[buffer * cfg_->PacketLength()]);
    rx_packets_.emplace_back(pkt_loc);
  }

  AGORA_LOG_INFO(
      "WiredControlChannel: Setting up UDP server for Wired Control Channel "
      "data at port %zu\n",
      local_port);
  udp_comm_ = std::make_unique<UDPComm>(local_addr, local_port,
                                        udp_pkt_len * kMaxUEs, 0);
  udp_comm_->Connect(remote_addr, remote_port);
}

WiredControlChannel::~WiredControlChannel() {
  std::fclose(log_file_);
  //delete tx_producer_;
  AGORA_LOG_INFO("WiredControlChannel: Thread destroyed\n");
}

RxPacket& WiredControlChannel::GetRxPacket() {
  RxPacket& new_packet = rx_packets_.at(pkt_tracker_);
  AGORA_LOG_TRACE(
      "WiredControlChannel: Getting new rx packet at location %ld\n",
      reinterpret_cast<intptr_t>(&new_packet));

  // if rx_buffer is full, exit
  if (new_packet.Empty() == false) {
    AGORA_LOG_ERROR("WiredControlChannel: rx buffer full, memory overrun\n");
    throw std::runtime_error("rx buffer full, memory overrun");
  }
  // Mark the packet as used
  new_packet.Use();

  pkt_tracker_ = (pkt_tracker_ + 1);
  //Round robbin
  if (pkt_tracker_ == max_pkt_cnt_) {
    pkt_tracker_ = 0;
  }
  return new_packet;
}

void WiredControlChannel::ReturnRxPacket(RxPacket& unused_packet) {
  size_t new_index;
  //Decrement the rx_memory_idx
  if (pkt_tracker_ == 0) {
    new_index = max_pkt_cnt_ - 1;
  } else {
    new_index = pkt_tracker_ - 1;
  }
  const RxPacket& returned_packet = rx_packets_.at(new_index);
  //Make sure we are returning the correct packet, used for extra error checking
  if (&returned_packet != &unused_packet) {
    AGORA_LOG_WARN(
        "WiredControlChannel: returned memory that wasn't used last at address "
        "%ld\n",
        reinterpret_cast<intptr_t>(&unused_packet));
  } else {
    //If they are the same, reuse the old position
    pkt_tracker_ = new_index;
  }
  // if the returned packet is free, something is wrong
  if (unused_packet.Empty()) {
    AGORA_LOG_ERROR(
        "WiredControlChannel: rx buffer returned free memory at address %ld\n",
        reinterpret_cast<intptr_t>(&unused_packet));
    throw std::runtime_error(
        "WiredControlChannel: rx buffer returned free memory");
  }
  // Mark the packet as free
  unused_packet.Free();
}

bool WiredControlChannel::SendEventToLocalPhy(RxPacket& rx_packet) {
  // create event from pkt
  EventData msg(EventType::kPacketRX, rx_tag_t(rx_packet).tag_);
  auto enqueue_status = rx_queue_->enqueue(*rx_producer_, msg);
  if (enqueue_status == false) {
    throw std::runtime_error(
        "WiredControlChannel: Failed to enqueue control packet");
  }
  return enqueue_status;
}

bool WiredControlChannel::ReceivePacketFromRemotePhy() {
  RxPacket& rx_packet = GetRxPacket();
  Packet* pkt = rx_packet.RawPacket();

  ssize_t ret =
      udp_comm_->Recv(reinterpret_cast<std::byte*>(pkt), cfg_->PacketLength());
  if (ret == 0) {
    AGORA_LOG_TRACE("WiredControlChannel: No data received\n");
    ReturnRxPacket(rx_packet);
    return false;
  } else if (ret < 0) {
    AGORA_LOG_TRACE("WiredControlChannel: Error in reception %zu\n", ret);
    cfg_->Running(false);
    return false;
  } else if (ret < cfg_->PacketLength()) {
    AGORA_LOG_ERROR(
        "ReceivePacketFromRemotePhy: Udp Recv failed to receive all expected "
        "bytes");
    cfg_->Running(false);
    return false;
  }
  return SendEventToLocalPhy(rx_packet);
}

bool WiredControlChannel::ReceiveEventFromLocalPhy() {
  EventData event;
  if (tx_queue_->try_dequeue_from_producer(*tx_producer_, event) == false) {
    return false;
  }

  if (event.event_type_ == EventType::kPacketToRemote) {
    RxPacket* rx = rx_tag_t(event.tags_[0u]).rx_packet_;
    Packet* pkt = rx->RawPacket();
    AGORA_LOG_INFO(
        "WiredControlChannel: Transmitting wired control data for Frame "
        "%zu, Symbol %zu, Ant %zu\n",
        pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
    SendPacketToRemotePhy(event);
    return true;
  }
  AGORA_LOG_ERROR("Wired Ctrl Dequeue: not a kPacketToRemote event!\n");
  return false;
}

void WiredControlChannel::SendPacketToRemotePhy(EventData event) {
  RtAssert(event.event_type_ == EventType::kPacketToRemote,
           "Unexpected Event! Only kPacketToRemote is expected.");
  RxPacket* rxpkt = rx_tag_t(event.tags_[0]).rx_packet_;
  Packet* pkt = rxpkt->RawPacket();
  udp_comm_->Send(reinterpret_cast<std::byte*>(pkt), cfg_->PacketLength());
  AGORA_LOG_INFO(
      "WiredControlChannel: Transmitting wired control data for Frame "
      "%zu, Symbol %zu, Ant %zu\n",
      pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
  rx_tag_t(event.tags_[0]).rx_packet_->Free();
}

void WiredControlChannel::RunTxEventLoop() {
  AGORA_LOG_INFO(
      "WiredControlChannel: Running thread event loop, logging to file "
      "%s\n",
      log_filename_.c_str());

  PinToCoreWithOffset(ThreadType::kWorkerWCC, tx_core_offset_,
                      0 /* thread ID */, false, true);
  // keep the frequency of function call
  size_t last_frame_tx_tsc = 0;
  size_t frame = 0;
  while (cfg_->Running() == true) {
    size_t packet_count = 0;
    if ((GetTime::Rdtsc() - last_frame_tx_tsc) > tsc_delta_) {
      while (cfg_->Running() == true &&
             //((GetTime::Rdtsc() - last_frame_tx_tsc) < tsc_delta_) &&
             packet_count < cfg_->Frame().NumDLCalSyms() * cfg_->UeAntNum()) {
        if (ReceiveEventFromLocalPhy()) packet_count++;
      }
      last_frame_tx_tsc = GetTime::Rdtsc();
    }
  }
}

void WiredControlChannel::RunRxEventLoop() {
  AGORA_LOG_INFO(
      "WiredControlChannel: Running thread event loop, logging to file "
      "%s\n",
      log_filename_.c_str());

  PinToCoreWithOffset(ThreadType::kWorkerWCC, rx_core_offset_,
                      0 /* thread ID */, false, true);
  // keep the frequency of function call
  size_t last_frame_rx_tsc = 0;

  while (cfg_->Running() == true) {
    if ((GetTime::Rdtsc() - last_frame_rx_tsc) > tsc_delta_) {
      size_t packet_count = 0;
      while (cfg_->Running() == true &&
             packet_count < cfg_->Frame().NumDLCalSyms() * cfg_->UeAntNum()) {
        if (ReceivePacketFromRemotePhy()) packet_count++;
      }
      last_frame_rx_tsc = GetTime::Rdtsc();
    }
  }
}
