/**
 * @file rp_thread.cc
 * @brief Implementation file for the ResourceProvisionerThread class.
 */
#include "resource_provisioner_thread.h"

#include "logger.h"

// ticks every ~1s
ResourceProvisionerThread::ResourceProvisionerThread(
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

  AGORA_LOG_INFO(
      "ResourceProvisionerThread: Frame duration %.2f ms, tsc_delta %zu\n",
      cfg_->GetFrameDurationSec() * 1000, tsc_delta_);

  // RP sends control message of size defined by RPControlMsg
  //sizeof(RPControlMsg);
  const size_t udp_pkt_len = 32;
  udp_pkt_buf_.resize(udp_pkt_len + kUdpRxBufferPadding);

  size_t udp_server_port = cfg_->RpRxPort();
  AGORA_LOG_INFO(
      "ResourceProvisionerThread: Setting up UDP server for RP data at port "
      "%zu\n",
      udp_server_port);
  udp_comm_ = std::make_unique<UDPComm>(cfg_->UeServerAddr(), udp_server_port,
                                        udp_pkt_len * kMaxUEs, 0);
}

ResourceProvisionerThread::~ResourceProvisionerThread() {
  std::fclose(log_file_);
  AGORA_LOG_INFO("ResourceProvisionerThread: Thread destroyed\n");
}

void ResourceProvisionerThread::RequestEventFromAgora() {
  // create event from pkt
  EventData msg(EventType::kPacketToRp, 0);
  RtAssert(tx_queue_->enqueue(msg),
           "ResourceProvisionerThread: Failed to enqueue control packet");
}

/*
 * RP -> ReceiveUdpPacketsFromRp() -> SendEventToAgora(payload) -> Agora
 */
void ResourceProvisionerThread::SendEventToAgora(const char* payload) {
  const auto* pkt = reinterpret_cast<const RPControlMsg*>(&payload[0]);

  if (pkt->add_core_ == 0 && pkt->remove_core_ == 0) {
    // request traffic data
    AGORA_LOG_INFO("ResourceProvisionerThread: Requesting status from Agora\n");
    RequestEventFromAgora();
  } else {
    // create event from pkt
    EventData msg(EventType::kPacketFromRp, pkt->add_core_, pkt->remove_core_);
    AGORA_LOG_INFO(
        "ResourceProvisionerThread: Sending core update data to Agora of "
        "add_cores %zu, remove_cores %zu\n",
        pkt->add_core_, pkt->remove_core_);
    RtAssert(tx_queue_->enqueue(msg),
             "ResourceProvisionerThread: Failed to enqueue control packet");
  }
}

void ResourceProvisionerThread::ReceiveUdpPacketsFromRp() {
  ssize_t ret = udp_comm_->Recv(&udp_pkt_buf_.at(0), udp_pkt_buf_.size());
  if (ret == 0) {
    AGORA_LOG_TRACE("ResourceProvisionerThread: No data received\n");
    return;
  } else if (ret < 0) {
    AGORA_LOG_TRACE("ResourceProvisionerThread: Error in reception %zu\n", ret);
    cfg_->Running(false);
    return;
  } else { /* Got some data */
    SendEventToAgora((char*)&udp_pkt_buf_[0]);
    return;
  }
}

/*
 * Agora -> ReceiveEventFromAgora() -> SendUdpPacketsToRp(event) -> RP
 */
void ResourceProvisionerThread::ReceiveEventFromAgora() {
  EventData event;
  if (rx_queue_->try_dequeue(event) == false) {
    return;
  }

  if (event.event_type_ == EventType::kPacketToRp) {
    AGORA_LOG_INFO(
        "ResourceProvisionerThread: Received status from Agora of latency %zu, "
        "core_num %zu\n",
        event.tags_[0], event.tags_[1]);
    SendUdpPacketsToRp(event);
  }
}

void ResourceProvisionerThread::SendUdpPacketsToRp(EventData event) {
  // Create traffic data packet
  RPStatusMsg msg;
  msg.latency_ = event.tags_[0];
  msg.core_num_ = event.tags_[1];
  udp_comm_->Send(cfg_->RpRemoteHostName(), cfg_->RpTxPort(), (std::byte*)&msg,
                  sizeof(RPStatusMsg));
}

void ResourceProvisionerThread::RunEventLoop() {
  AGORA_LOG_INFO(
      "ResourceProvisionerThread: Running thread event loop, logging to file "
      "%s\n",
      log_filename_.c_str());

  PinToCoreWithOffset(ThreadType::kWorkerRpTXRX, core_offset_,
                      0 /* thread ID */);
  // keep the frequency of function call
  size_t last_frame_tx_tsc = 0;

  while (cfg_->Running() == true) {
    if ((GetTime::Rdtsc() - last_frame_tx_tsc) > tsc_delta_) {
      ReceiveEventFromAgora();
      ReceiveUdpPacketsFromRp();
      last_frame_tx_tsc = GetTime::Rdtsc();
    }
  }
}