/**
 * @file client_comm.cc
 * @brief Implementation file for the WiredControlChannel class.
 */
#include "client_comm.h"

#include "gettime.h"
#include "logger.h"

// ticks every ~1s
WiredControlChannel::WiredControlChannel(
    Config* cfg, size_t tx_core_offset, size_t rx_core_offset,
    std::string local_addr, size_t local_port, std::string remote_addr,
    size_t remote_port, moodycamel::ConcurrentQueue<EventData>* rx_queue,
    moodycamel::ConcurrentQueue<EventData>* tx_queue,
    const std::string& log_filename)
    : cfg_(cfg),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      tsc_delta_((cfg_->GetFrameDurationSec() * 1e9) / freq_ghz_ * 1000),
      tx_core_offset_(tx_core_offset),
      rx_core_offset_(rx_core_offset),
      pkt_tracker_(0),
      rx_queue_(rx_queue),
      tx_queue_(tx_queue) {
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

  rx_socket_buf_.Calloc(kFrameWnd, cfg_->PacketLength(),
                        Agora_memory::Alignment_t::kAlign64);

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
  rx_socket_buf_.Free();
  AGORA_LOG_INFO("WiredControlChannel: Thread destroyed\n");
}

/*
 * RP -> ReceiveUdpPacketsFromRp() -> SendEventToAgora(payload) -> Agora
 */
void WiredControlChannel::SendEventToLocalPhy(std::byte* data) {
  RxPacket rxpkt(reinterpret_cast<Packet*>(data));
  auto* pkt = rxpkt.RawPacket();
  rxpkt.Use();

  // create event from pkt
  EventData msg(EventType::kPacketFromRemote, rx_tag_t(rxpkt).tag_);
  AGORA_LOG_INFO(
      "WiredControlChannel: Received wired control data for "
      "Frame %zu, Symbol %zu, Ant %zu\n",
      pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
  RtAssert(rx_queue_->enqueue(msg),
           "WiredControlChannel: Failed to enqueue control packet");
}

void WiredControlChannel::ReceivePacketFromRemotePhy() {
  std::byte* pkt_udp = GetNextPacketBuff();
  std::memset(pkt_udp, 0, cfg_->PacketLength());
  ssize_t ret = udp_comm_->Recv(pkt_udp, cfg_->PacketLength());
  if (ret == 0) {
    AGORA_LOG_TRACE("WiredControlChannel: No data received\n");
    return;
  } else if (ret < 0) {
    AGORA_LOG_TRACE("WiredControlChannel: Error in reception %zu\n", ret);
    cfg_->Running(false);
    return;
  }
  SendEventToLocalPhy(pkt_udp);
}

/*
 * Agora -> ReceiveEventFromAgora() -> SendUdpPacketsToRp(event) -> RP
 */
void WiredControlChannel::ReceiveEventFromLocalPhy() {
  EventData event;
  if (tx_queue_->try_dequeue(event) == false) {
    return;
  }

  if (event.event_type_ == EventType::kPacketToRemote) {
    RxPacket* rx = rx_tag_t(event.tags_[0u]).rx_packet_;
    Packet* pkt = rx->RawPacket();
    AGORA_LOG_INFO(
        "WiredControlChannel: Transmitting wired control data for Frame "
        "%zu, Symbol %zu, Ant %zu\n",
        pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
    SendPacketToRemotePhy(event);
  }
}

void WiredControlChannel::SendPacketToRemotePhy(EventData event) {
  assert(event.event_type_ == EventType::kPacketToRemote);
  /*const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
  const size_t ue_id = gen_tag_t(event.tags_[0]).ant_id_;*/
  RxPacket* rxpkt = rx_tag_t(event.tags_[0]).rx_packet_;
  Packet* pkt = rxpkt->RawPacket();
  udp_comm_->Send(reinterpret_cast<std::byte*>(pkt), cfg_->PacketLength());
  rxpkt->Free();
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

  while (cfg_->Running() == true) {
    ReceiveEventFromLocalPhy();
    /*if ((GetTime::Rdtsc() - last_frame_tx_tsc) > tsc_delta_) {
      ReceiveEventFromLocalPhy();
      last_frame_tx_tsc = GetTime::Rdtsc();
    }*/
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
    ReceivePacketFromRemotePhy();
    /*if ((GetTime::Rdtsc() - last_frame_rx_tsc) > tsc_delta_) {
      ReceivePacketFromRemotePhy();
      last_frame_rx_tsc = GetTime::Rdtsc();
    }*/
  }
}
