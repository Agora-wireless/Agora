/**
 * @file client_comm.cc
 * @brief Implementation file for the UserWiredChannel class.
 */
#include "client_comm.h"

#include "gettime.h"
#include "logger.h"

// ticks every ~1s
UserWiredChannel::UserWiredChannel(
    Config* cfg, size_t core_offset,
    moodycamel::ConcurrentQueue<EventData>* rx_queue,
    moodycamel::ConcurrentQueue<EventData>* tx_queue,
    const std::string& log_filename)
    : cfg_(cfg),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      tsc_delta_((cfg_->GetFrameDurationSec() * 1e9) / freq_ghz_ * 1000),
      core_offset_(core_offset),
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

  AGORA_LOG_INFO("UserWiredChannel: Frame duration %.2f ms, tsc_delta %zu\n",
                 cfg_->GetFrameDurationSec() * 1000, tsc_delta_);

  // RP sends control message of size defined by RPControlMsg
  //sizeof(RPControlMsg);
  const size_t udp_pkt_len = cfg_->PacketLength();
  udp_pkt_buf_.resize(kFrameWnd);
  //  udp_pkt_buf_.at(i).resize(udp_pkt_len);  //+ kUdpRxBufferPadding);
  pkt_tracker_ = 0;

  size_t udp_server_port = cfg_->CwcRxPort();
  AGORA_LOG_INFO(
      "UserWiredChannel: Setting up UDP server for UserWiredLink data at port "
      "%zu\n",
      udp_server_port);
  udp_comm_ = std::make_unique<UDPComm>(cfg_->BsServerAddr(), udp_server_port,
                                        udp_pkt_len * kMaxUEs, 0);
}

UserWiredChannel::~UserWiredChannel() {
  std::fclose(log_file_);
  AGORA_LOG_INFO("UserWiredChannel: Thread destroyed\n");
}

/*
 * RP -> ReceiveUdpPacketsFromRp() -> SendEventToAgora(payload) -> Agora
 */
void UserWiredChannel::SendEventToAgora(RxPacket* rxpkt) {
  Packet* pkt = rxpkt->RawPacket();

  // create event from pkt
  EventData msg(EventType::kPacketFromClient, rx_tag_t(rxpkt).tag_);
  AGORA_LOG_INFO(
      "UserWiredChannel: Sending core update data to Agora of "
      "Frame %zu, Symbol %zu, Ant %zu\n",
      pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
  RtAssert(tx_queue_->enqueue(msg),
           "UserWiredChannel: Failed to enqueue control packet");
}

RxPacket* UserWiredChannel::ReceiveUdpPacketsFromClient() {
  Packet* pkt = udp_pkt_buf_.at(pkt_tracker_)->RawPacket();
  ssize_t ret =
      udp_comm_->Recv(reinterpret_cast<std::byte*>(pkt), cfg_->PacketLength());
  if (ret == 0) {
    AGORA_LOG_TRACE("UserWiredChannel: No data received\n");
    return NULL;
  } else if (ret < 0) {
    AGORA_LOG_TRACE("UserWiredChannel: Error in reception %zu\n", ret);
    cfg_->Running(false);
    return NULL;
  } else { /* Got some data */
    //SendEventToAgora((char*)&udp_pkt_buf_[0]);
    RxPacket* rxpkt = udp_pkt_buf_.at(pkt_tracker_++);
    pkt_tracker_ %= kFrameWnd;
    return rxpkt;
  }
}

/*
 * Agora -> ReceiveEventFromAgora() -> SendUdpPacketsToRp(event) -> RP
 */
void UserWiredChannel::ReceiveEventFromAgora() {
  EventData event;
  if (rx_queue_->try_dequeue(event) == false) {
    return;
  }

  if (event.event_type_ == EventType::kPacketToClient) {
    AGORA_LOG_INFO(
        "UserWiredChannel: Received status from Agora of latency %zu, "
        "core_num %zu\n",
        event.tags_[0], event.tags_[1]);
    SendUdpPacketsToClient(event);
  }
}

void UserWiredChannel::SendUdpPacketsToClient(EventData event) {
  // Create traffic data packet
  /*RpStatusMsg msg;
  msg.latency_ = event.tags_[0];
  msg.core_num_ = event.tags_[1];
  udp_comm_->Send(cfg_->UeServerAddr(), cfg_->CwcTxPort(), (std::byte*)&msg,
                  sizeof(CwcStatusMsg));*/
}

void UserWiredChannel::RunEventLoop() {
  AGORA_LOG_INFO(
      "UserWiredChannel: Running thread event loop, logging to file "
      "%s\n",
      log_filename_.c_str());

  PinToCoreWithOffset(ThreadType::kWorkerCWC, core_offset_, 0 /* thread ID */);
  // keep the frequency of function call
  size_t last_frame_tx_tsc = 0;

  while (cfg_->Running() == true) {
    if ((GetTime::Rdtsc() - last_frame_tx_tsc) > tsc_delta_) {
      ReceiveEventFromAgora();
      ReceiveUdpPacketsFromClient();
      last_frame_tx_tsc = GetTime::Rdtsc();
    }
  }
}
