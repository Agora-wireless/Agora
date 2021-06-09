/**
 * @file mac_thread.cc
 * @brief Implementation file for the MacThreadClient class.
 */
#include "mac_thread_client.h"

#include "logger.h"
#include "utils_ldpc.h"

MacThreadClient::MacThreadClient(
    Config* cfg, size_t core_offset,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffer,
    Table<int8_t>* ul_bits_buffer, Table<int8_t>* ul_bits_buffer_status,
    moodycamel::ConcurrentQueue<EventData>* rx_queue,
    moodycamel::ConcurrentQueue<EventData>* tx_queue,
    const std::string& log_filename)
    : cfg_(cfg),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      tsc_delta_((cfg_->GetFrameDurationSec() * 1e9) / freq_ghz_),
      core_offset_(core_offset),
      decoded_buffer_(decoded_buffer),
      rx_queue_(rx_queue),
      tx_queue_(tx_queue) {
  // Set up MAC log file
  if (log_filename.empty() == false) {
    log_filename_ = log_filename;  // Use a non-default log filename
  } else {
    std::string mode_string = "_client";
    log_filename_ = kDefaultLogFilename + mode_string;
  }
  log_file_ = std::fopen(log_filename_.c_str(), "w");
  RtAssert(log_file_ != nullptr, "Failed to open MAC log file");

  std::printf("MAC thread: Frame duration %.2f ms, tsc_delta %zu\n",
              cfg_->GetFrameDurationSec() * 1000, tsc_delta_);

  // Set up buffers
  client_.ul_bits_buffer_ = ul_bits_buffer;
  client_.ul_bits_buffer_status_ = ul_bits_buffer_status;

  server_.n_filled_in_frame_.fill(0);
  for (auto& v : server_.frame_data_) {
    v.resize(cfg_->DlMacDataBytesNumPerframe());
  }

  client_.ul_bits_buffer_id_.fill(0);

  const size_t udp_pkt_len = cfg_->UlMacDataBytesNumPerframe();
  udp_pkt_buf_.resize(udp_pkt_len);

  // TODO: See if it makes more sense to split up the UE's by port here for
  // client mode.
  size_t udp_server_port = cfg_->UeMacRxPort();
  MLPD_TRACE("MacThreadClient: setting up udp server at port %zu\n",
             udp_server_port);
  udp_server_ = std::make_unique<UDPServer>(
      udp_server_port, udp_pkt_len * kMaxUEs * kMaxPktsPerUE);

  const size_t udp_control_len = sizeof(RBIndicator);
  udp_control_buf_.resize(udp_control_len);

  MLPD_TRACE("MacThreadClient: setting up udp server at port %zu\n",
             kMacBaseClientPort);
  udp_control_channel_ = std::make_unique<UDPServer>(
      kMacBaseClientPort, udp_control_len * kMaxUEs * kMaxPktsPerUE);

  udp_client_ = std::make_unique<UDPClient>();
  crc_obj_ = std::make_unique<DoCRC>();
}

MacThreadClient::~MacThreadClient() {
  std::fclose(log_file_);
  MLPD_INFO("MAC thread destroyed\n");
}

void MacThreadClient::ProcessRxFromPhy() {
  EventData event;
  if (rx_queue_->try_dequeue(event) == false) {
    return;
  }

  if (event.event_type_ == EventType::kPacketToMac) {
    MLPD_TRACE("MAC thread event kPacketToMac\n");
    ProcessCodeblocksFromPhy(event);
  } else if (event.event_type_ == EventType::kSNRReport) {
    MLPD_TRACE("MAC thread event kSNRReport\n");
    ProcessSnrReportFromPhy(event);
  }
}

void MacThreadClient::ProcessSnrReportFromPhy(EventData event) {
  const size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
  if (server_.snr_[ue_id].size() == kSNRWindowSize) {
    server_.snr_[ue_id].pop();
  }

  float snr;
  std::memcpy(&snr, &event.tags_[1], sizeof(float));
  server_.snr_[ue_id].push(snr);
}

void MacThreadClient::SendRanConfigUpdate(EventData /*event*/) {
  RanConfig rc;
  rc.n_antennas_ = 0;  // TODO [arjun]: What's the correct value here?
  rc.mod_order_bits_ = CommsLib::kQaM16;
  rc.frame_id_ = scheduler_next_frame_id_;
  // TODO: change n_antennas to a desired value
  // cfg_->BsAntNum() is added to fix compiler warning
  rc.n_antennas_ = cfg_->BsAntNum();

  EventData msg(EventType::kRANUpdate);
  msg.num_tags_ = 3;
  msg.tags_[0] = rc.n_antennas_;
  msg.tags_[1] = rc.mod_order_bits_;
  msg.tags_[2] = rc.frame_id_;
  RtAssert(tx_queue_->enqueue(msg),
           "MAC thread: failed to send RAN update to Agora");

  scheduler_next_frame_id_++;
}

void MacThreadClient::ProcessCodeblocksFromPhy(EventData event) {
  assert(event.event_type_ == EventType::kPacketToMac);

  const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
  const size_t symbol_idx_dl = this->cfg_->Frame().GetDLSymbolIdx(symbol_id);
  const size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
  const int8_t* ul_data_ptr =
      decoded_buffer_[frame_id % kFrameWnd][symbol_idx_dl][ue_id];

  std::stringstream ss;  // Debug-only

  // Only non-pilot uplink symbols have application data.
  if (symbol_idx_dl >= cfg_->Frame().ClientDlPilotSymbols()) {
    auto* pkt = (struct MacPacket*)ul_data_ptr;

    // We send data to app irrespective of CRC condition
    // TODO: enable ARQ and ensure reliable data goes to app
    const size_t frame_data_offset =
        (symbol_idx_dl - cfg_->Frame().ClientDlPilotSymbols()) *
        cfg_->MacPayloadLength();
    std::memcpy(&server_.frame_data_[ue_id][frame_data_offset], pkt->data_,
                cfg_->MacPayloadLength());
    server_.n_filled_in_frame_[ue_id] += cfg_->MacPayloadLength();

    // Check CRC
    auto crc = static_cast<uint16_t>(
        crc_obj_->CalculateCrc24((unsigned char*)pkt->data_,
                                 cfg_->MacPayloadLength()) &
        0xFFFF);
    if (crc == pkt->crc_) {
      // Print information about the received symbol
      if (kLogMacPackets) {
        std::fprintf(
            log_file_,
            "MAC thread received frame %zu, downlink symbol index %zu, "
            "size %zu, copied to frame data offset %zu\n",
            frame_id, symbol_idx_dl, cfg_->MacPayloadLength(),
            frame_data_offset);

        ss << "Header Info:\n"
           << "FRAME_ID: " << pkt->frame_id_
           << "\nSYMBOL_ID: " << pkt->symbol_id_ << "\nUE_ID: " << pkt->ue_id_
           << "\nDATLEN: " << pkt->datalen_ << "\nPAYLOAD:\n";
        for (size_t i = 0; i < cfg_->MacPayloadLength(); i++) {
          ss << std::to_string(pkt->data_[i]) << " ";
        }
        std::fprintf(log_file_, "%s\n", ss.str().c_str());
        ss.str("");
      }
    } else {
      std::printf("Bad Packet: CRC Check Failed! \n");
    }
  }

  // When the frame is full, send it to the application
  if (server_.n_filled_in_frame_[ue_id] == cfg_->DlMacDataBytesNumPerframe()) {
    server_.n_filled_in_frame_[ue_id] = 0;

    udp_client_->Send(kMacRemoteHostname, cfg_->UeMacTxPort() + ue_id,
                      &server_.frame_data_[ue_id][0],
                      cfg_->DlMacDataBytesNumPerframe());
    std::fprintf(log_file_,
                 "MAC thread: Sent data for frame %zu, ue %zu, size %zu\n",
                 frame_id, ue_id, cfg_->DlMacDataBytesNumPerframe());
    for (size_t i = 0; i < cfg_->DlMacDataBytesNumPerframe(); i++) {
      ss << std::to_string(server_.frame_data_[ue_id][i]) << " ";
    }
    std::fprintf(log_file_, "%s\n", ss.str().c_str());
    ss.str("");
  }

  RtAssert(
      tx_queue_->enqueue(EventData(EventType::kPacketToMac, event.tags_[0])),
      "Socket message enqueue failed\n");
}

void MacThreadClient::SendControlInformation() {
  // send RAN control information UE
  RBIndicator ri;
  ri.ue_id_ = next_radio_id_;
  ri.mod_order_bits_ = CommsLib::kQaM16;
  udp_client_->Send(cfg_->UeServerAddr(), kMacBaseClientPort + ri.ue_id_,
                    (uint8_t*)&ri, sizeof(RBIndicator));

  // update RAN config within Agora
  SendRanConfigUpdate(EventData(EventType::kRANUpdate));
}

void MacThreadClient::ProcessControlInformation() {
  std::memset(&udp_control_buf_[0], 0, udp_control_buf_.size());
  ssize_t ret =
      udp_control_channel_->Recv(&udp_control_buf_[0], udp_control_buf_.size());
  if (ret == 0) {
    return;  // No data received
  } else if (ret == -1) {
    // There was an error in receiving
    cfg_->Running(false);
    return;
  }

  RtAssert(static_cast<size_t>(ret) == sizeof(RBIndicator));

  const auto* ri = reinterpret_cast<RBIndicator*>(&udp_control_buf_[0]);
  ProcessUdpPacketsFromApps(*ri);
}

void MacThreadClient::ProcessUdpPacketsFromApps(RBIndicator ri) {
  if (0 == cfg_->UlMacDataBytesNumPerframe()) return;

  std::memset(&udp_pkt_buf_[0], 0, udp_pkt_buf_.size());

  size_t rx_bytes = 0;
  size_t rx_request_size = udp_pkt_buf_.size();
  uint8_t* rx_location = &udp_pkt_buf_[0];
  for (size_t rx_tries = 0; rx_tries < cfg_->UlMacPacketsPerframe();
       rx_tries++) {
    ssize_t ret = udp_server_->Recv(rx_location, rx_request_size);
    if (ret == 0) {
      // printf("No data received\n");
      return;  // No data received
    } else if (ret < 0) {
      // There was an error in receiving
      MLPD_ERROR("MacThreadClient: Error in reception %zu\n", ret);
      cfg_->Running(false);
      return;
    } else {
      MLPD_FRAME("MacThreadClient: Received %zu : %zu bytes\n", ret,
                 rx_request_size);
      rx_bytes += ret;
      if (rx_bytes == udp_pkt_buf_.size()) {
        break;
      }
      rx_request_size -= ret;
      rx_location += ret;
    }
  }
  if (rx_bytes != cfg_->UlMacDataBytesNumPerframe()) {
    MLPD_ERROR("MacThreadClient: Received %zu : %zu bytes\n", rx_bytes,
               cfg_->UlMacDataBytesNumPerframe());
  } else {
    MLPD_INFO("MacThreadClient: Received Mac Frame Data\n");
  }
  RtAssert(
      rx_bytes == cfg_->UlMacDataBytesNumPerframe(),
      "MacThreadClient:ProcessUdpPacketsFromApps incorrect number of bytes "
      "received.");

  const auto* pkt = reinterpret_cast<MacPacket*>(&udp_pkt_buf_[0]);
  ProcessUdpPacketsFromAppsClient((char*)pkt, ri);
}

void MacThreadClient::ProcessUdpPacketsFromAppsClient(const char* payload,
                                                      RBIndicator ri) {
  // We've received bits for the uplink. The received MAC packet does not
  // specify a radio ID, send to radios in round-robin order
  size_t& radio_buf_id = client_.ul_bits_buffer_id_[next_radio_id_];

  if ((*client_.ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] == 1) {
    std::fprintf(stderr,
                 "MAC thread: UDP RX buffer full, buffer ID: %zu. Dropping "
                 "packet.\n",
                 radio_buf_id);
    return;
  }

  if (kLogMacPackets) {
    std::stringstream ss;
    std::fprintf(
        log_file_,
        "MAC thread: Received data from app for frame %zu, ue %zu, size "
        "%zu:\n",
        next_frame_id_, next_radio_id_, cfg_->UlMacDataBytesNumPerframe());

    for (size_t i = 0; i < cfg_->UlMacDataBytesNumPerframe(); i++) {
      ss << std::to_string((uint8_t)(payload[i])) << " ";
    }
    std::fprintf(log_file_, "%s\n", ss.str().c_str());
  }

  for (size_t pkt_id = 0; pkt_id < cfg_->UlMacPacketsPerframe(); pkt_id++) {
    size_t pkt_offset = radio_buf_id * cfg_->UlMacBytesNumPerframe() +
                        pkt_id * cfg_->MacPacketLength();
    auto* pkt = reinterpret_cast<MacPacket*>(
        &(*client_.ul_bits_buffer_)[next_radio_id_][pkt_offset]);

    pkt->frame_id_ = next_frame_id_;
    pkt->symbol_id_ = pkt_id;
    pkt->ue_id_ = next_radio_id_;
    pkt->datalen_ = cfg_->MacPayloadLength();
    pkt->rsvd_[0] = static_cast<uint16_t>(fast_rand_.NextU32() >> 16);
    pkt->rsvd_[1] = static_cast<uint16_t>(fast_rand_.NextU32() >> 16);
    pkt->rsvd_[2] = static_cast<uint16_t>(fast_rand_.NextU32() >> 16);
    pkt->crc_ = 0;
    pkt->rb_indicator_ = ri;

    std::memcpy(pkt->data_, payload + pkt_id * cfg_->MacPayloadLength(),
                cfg_->MacPayloadLength());
    // Insert CRC
    pkt->crc_ = (uint16_t)(crc_obj_->CalculateCrc24((unsigned char*)pkt->data_,
                                                    cfg_->MacPayloadLength()) &
                           0xFFFF);

    if (kLogMacPackets) {
      std::stringstream ss;
      std::printf(
          "MAC thread created packet frame %zu, pkt %zu, size %zu, copied to "
          "location %zu\n",
          next_frame_id_, pkt_id, cfg_->MacPayloadLength(), (size_t)pkt);

      ss << "Header Info:\n"
         << "FRAME_ID: " << pkt->frame_id_ << "\nSYMBOL_ID: " << pkt->symbol_id_
         << "\nUE_ID: " << pkt->ue_id_ << "\nDATLEN: " << pkt->datalen_
         << "\nPAYLOAD:\n";
      for (size_t i = 0; i < cfg_->MacPayloadLength(); i++) {
        ss << std::to_string(pkt->data_[i]) << " ";
      }
      std::fprintf(log_file_, "%s\n", ss.str().c_str());
      std::printf("%s\n", ss.str().c_str());
      ss.str("");
    }
  }

  (*client_.ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] = 1;
  EventData msg(EventType::kPacketFromMac,
                rx_tag_t(next_radio_id_, radio_buf_id).tag_);
  RtAssert(tx_queue_->enqueue(msg),
           "MAC thread: Failed to enqueue uplink packet");

  radio_buf_id = (radio_buf_id + 1) % kFrameWnd;
  next_radio_id_ = (next_radio_id_ + 1) % cfg_->UeAntNum();
  if (next_radio_id_ == 0) {
    next_frame_id_++;
  }
}

void MacThreadClient::RunEventLoop() {
  MLPD_INFO("Running MAC thread event loop, logging to file %s\n",
            log_filename_.c_str());
  PinToCoreWithOffset(ThreadType::kWorkerMacTXRX, core_offset_,
                      0 /* thread ID */);

  while (cfg_->Running() == true) {
    ProcessRxFromPhy();

    ProcessControlInformation();

    if (next_frame_id_ == cfg_->FramesToTest()) {
      MLPD_WARN(
          "MAC thread stopping. Next frame ID = %zu, configured "
          "frames to test = %zu\n",
          next_frame_id_, cfg_->FramesToTest());
      break;
    }
  }
}
