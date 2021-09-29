/**
 * @file mac_thread.cc
 * @brief Implementation file for the MacThreadClient class.
 */
#include "mac_thread_client.h"

#include "logger.h"
#include "utils_ldpc.h"

static constexpr size_t kUdpRxBufferPadding = 2048u;

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
    log_filename_ = kDefaultLogFilename;
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
  udp_pkt_buf_.resize(udp_pkt_len + kUdpRxBufferPadding);

  // TODO: See if it makes more sense to split up the UE's by port here for
  // client mode.
  size_t udp_server_port = cfg_->UeMacRxPort();
  MLPD_INFO("MacThreadClient: setting up udp server for mac data at port %zu\n",
            udp_server_port);
  udp_server_ = std::make_unique<UDPServer>(
      udp_server_port, udp_pkt_len * kMaxUEs * kMaxPktsPerUE);

  const size_t udp_control_len = sizeof(RBIndicator);
  udp_control_buf_.resize(udp_control_len);

  MLPD_INFO(
      "MacThreadClient: setting up udp server for mac control channel at port "
      "%zu\n",
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

  const int8_t* dl_data_ptr =
      decoded_buffer_[(frame_id % kFrameWnd)][symbol_idx_dl][ue_id];

  std::stringstream ss;  // Debug-only

  // Only non-pilot downlink symbols have application data.
  if (symbol_idx_dl >= cfg_->Frame().ClientDlPilotSymbols()) {
    auto* pkt = reinterpret_cast<struct MacPacket const*>(dl_data_ptr);

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
            "User MAC thread received frame %zu, downlink symbol index %zu, "
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

  //Processes the packets of an entire frame
  const size_t packets_required = cfg_->UlMacPacketsPerframe();

  size_t packets_received = 0;
  size_t current_packet_bytes = 0;
  size_t current_packet_start_index = 0;

  size_t total_bytes_received = 0;

  const size_t max_recv_attempts = (packets_required * 10u);
  size_t rx_attempts;
  for (rx_attempts = 0u; rx_attempts < max_recv_attempts; rx_attempts++) {
    ssize_t ret = udp_server_->Recv(&udp_pkt_buf_.at(total_bytes_received),
                                    udp_pkt_buf_.size() - total_bytes_received);
    if (ret == 0) {
      MLPD_TRACE("MacThreadClient: No data received with %zu pending\n",
                 total_bytes_received);
      if (total_bytes_received == 0) {
        return;  // No data received
      } else {
        MLPD_INFO(
            "MacThreadClient: No data received but there was data in "
            "buffer pending %zu : try %zu out of %zu\n",
            total_bytes_received, rx_attempts, max_recv_attempts);
      }
    } else if (ret < 0) {
      // There was an error in receiving
      MLPD_ERROR("MacThreadClient: Error in reception %zu\n", ret);
      cfg_->Running(false);
      return;
    } else { /* Got some data */
      total_bytes_received += ret;
      current_packet_bytes += ret;

      //std::printf(
      //    "Received %zu bytes packet number %zu packet size %zu total %zu\n",
      //    ret, packets_received, total_bytes_received, current_packet_bytes);

      //While we have packets remaining and a header to process
      const size_t header_size = MacPacket::kOffsetOfData;
      while ((packets_received < packets_required) &&
             (current_packet_bytes >= header_size)) {
        // See if we have enough data and process the MacPacket header
        auto* rx_mac_packet_header = reinterpret_cast<const MacPacketPacked*>(
            &udp_pkt_buf_.at(current_packet_start_index));

        const size_t current_packet_size =
            header_size + rx_mac_packet_header->datalen_;

        //std::printf("Packet number %zu @ %zu packet size %d:%zu total %zu\n",
        //            packets_received, current_packet_start_index,
        //            rx_mac_packet_header->datalen_, current_packet_size,
        //            current_packet_bytes);

        if (current_packet_bytes >= current_packet_size) {
          current_packet_bytes = current_packet_bytes - current_packet_size;
          current_packet_start_index =
              current_packet_start_index + current_packet_size;
          packets_received++;
        } else {
          // Don't have the entire packet, keep trying
          break;
        }
      }
      MLPD_FRAME(
          "MacThreadClient: Received %zu : %zu bytes in packet %zu : %zu\n",
          ret, total_bytes_received, packets_received, packets_required);
    }

    //Check for completion
    if (packets_received == packets_required) {
      break;
    }
  }  // end rx attempts

  if (packets_received != packets_required) {
    MLPD_ERROR(
        "MacThreadClient: Received %zu : %zu packets with %zu total bytes in "
        "%zu attempts\n",
        packets_received, packets_required, total_bytes_received, rx_attempts);
  } else {
    MLPD_FRAME("MacThreadClient: Received Mac Frame Data\n");
  }
  RtAssert(
      packets_received == packets_required,
      "MacThreadClient:ProcessUdpPacketsFromApps incorrect data received!");

  //Currently this is a packet list of mac packets
  ProcessUdpPacketsFromAppsClient((char*)&udp_pkt_buf_[0], ri);
}

void MacThreadClient::ProcessUdpPacketsFromAppsClient(const char* payload,
                                                      RBIndicator ri) {
  // Data integrity check
  size_t pkt_offset = 0;
  size_t ue_id = 0;
  size_t symbol_id = 0;
  for (size_t packet = 0u; packet < cfg_->UlMacPacketsPerframe(); packet++) {
    auto* pkt = reinterpret_cast<const MacPacketPacked*>(&payload[pkt_offset]);

    //std::printf("Frame %d, Packet %zu, symbol %d, user %d\n", pkt->frame_id_,
    //            packet, pkt->symbol_id_, pkt->ue_id_);
    if (packet == 0) {
      ue_id = pkt->ue_id_;
    } else {
      if (ue_id != pkt->ue_id_) {
        MLPD_ERROR(
            "Received pkt %zu data with unexpected UE id %zu, expected %d\n",
            packet, ue_id, pkt->ue_id_);
      }
      if ((symbol_id + 1) != pkt->symbol_id_) {
        MLPD_ERROR("Received out of order symbol id %d, expected %zu\n",
                   pkt->symbol_id_, symbol_id + 1);
      }
    }
    symbol_id = pkt->symbol_id_;
    pkt_offset += MacPacket::kOffsetOfData + pkt->datalen_;
  }

  if (next_radio_id_ != ue_id) {
    MLPD_ERROR("Error - radio id %zu, expected %zu\n", ue_id, next_radio_id_);
  }
  //End data integrity check

  next_radio_id_ = ue_id;

  // We've received bits for the uplink. The received MAC packet does not
  // specify a radio ID, send to radios in round-robin order
  size_t& radio_buf_id = client_.ul_bits_buffer_id_[next_radio_id_];

  if ((*client_.ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] == 1) {
    std::fprintf(stderr,
                 "MAC thread: UDP RX buffer full, buffer ID: %zu. Dropping "
                 "rx frame data\n",
                 radio_buf_id);
    return;
  }

  if (kLogMacPackets) {
    std::stringstream ss;
    std::fprintf(log_file_,
                 "User MAC thread: Received data from app for frame %zu, ue "
                 "%zu size %zu\n",
                 next_tx_frame_id_, next_radio_id_, pkt_offset);

    for (size_t i = 0; i < pkt_offset; i++) {
      ss << std::to_string((uint8_t)(payload[i])) << " ";
    }
    std::fprintf(log_file_, "%s\n", ss.str().c_str());
  }

  size_t src_pkt_offset = 0;
  //Copy from the packet rx buffer into ul_bits memory (unpacked)
  for (size_t pkt_id = 0; pkt_id < cfg_->UlMacPacketsPerframe(); pkt_id++) {
    auto* src_packet =
        reinterpret_cast<const MacPacketPacked*>(&payload[src_pkt_offset]);
    //next_radio_id_ = src_packet->ue_id;

    // could use pkt_id vs src_packet->symbol_id_ but might reorder packets
    const size_t dest_pkt_offset =
        radio_buf_id * cfg_->UlMacBytesNumPerframe() +
        ((cfg_->Frame().GetULSymbolIdx(src_packet->symbol_id_) -
          cfg_->Frame().ClientUlPilotSymbols()) *
         cfg_->MacPacketLength());

    auto* pkt = reinterpret_cast<MacPacket*>(
        &(*client_.ul_bits_buffer_)[next_radio_id_][dest_pkt_offset]);

    pkt->frame_id_ = next_tx_frame_id_;
    pkt->symbol_id_ = src_packet->symbol_id_;
    pkt->ue_id_ = src_packet->ue_id_;
    pkt->datalen_ = src_packet->datalen_;
    pkt->rsvd_[0u] = static_cast<uint16_t>(fast_rand_.NextU32() >> 16);
    pkt->rsvd_[1u] = static_cast<uint16_t>(fast_rand_.NextU32() >> 16);
    pkt->rsvd_[2u] = static_cast<uint16_t>(fast_rand_.NextU32() >> 16);
#if ENABLE_RB_IND
    pkt->rb_indicator_ = ri;
#endif

    std::memcpy(pkt->data_, src_packet->data_, pkt->datalen_);
    // Insert CRC
    pkt->crc_ = (uint16_t)(crc_obj_->CalculateCrc24((unsigned char*)pkt->data_,
                                                    pkt->datalen_) &
                           0xFFFF);

    if (kLogMacPackets) {
      std::stringstream ss;
      std::printf(
          "User MAC thread created packet frame %zu, pkt %zu, size %zu, "
          "copied to location %zu dest offset %zu\n",
          next_tx_frame_id_, pkt_id, cfg_->MacPayloadLength(), (size_t)pkt,
          dest_pkt_offset);

      ss << "Header Info:\n"
         << "FRAME_ID: " << pkt->frame_id_ << "\nSYMBOL_ID: " << pkt->symbol_id_
         << "\nUE_ID: " << pkt->ue_id_ << "\nDATLEN: " << pkt->datalen_
         << "\nPAYLOAD:\n";
      for (size_t i = 0; i < pkt->datalen_; i++) {
        ss << std::to_string(pkt->data_[i]) << " ";
      }
      std::fprintf(log_file_, "%s\n", ss.str().c_str());
      std::printf("%s\n", ss.str().c_str());
      ss.str("");
    }
    src_pkt_offset += pkt->datalen_ + MacPacket::kOffsetOfData;
  }  // end all packets

  (*client_.ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] = 1;
  EventData msg(EventType::kPacketFromMac,
                rx_mac_tag_t(next_radio_id_, radio_buf_id).tag_);

  MLPD_TRACE("Tx mac information to %zu %zu\n", next_radio_id_, radio_buf_id);
  RtAssert(tx_queue_->enqueue(msg),
           "MAC thread: Failed to enqueue uplink packet");

  radio_buf_id = (radio_buf_id + 1) % kFrameWnd;
  //Might be unnecessary now.
  next_radio_id_ = (next_radio_id_ + 1) % cfg_->UeAntNum();
  if (next_radio_id_ == 0) {
    next_tx_frame_id_++;
  }
}

void MacThreadClient::RunEventLoop() {
  MLPD_INFO("Running MAC thread event loop, logging to file %s\n",
            log_filename_.c_str());
  PinToCoreWithOffset(ThreadType::kWorkerMacTXRX, core_offset_,
                      0 /* thread ID */);

  while (cfg_->Running() == true) {
    ProcessRxFromPhy();

    // No need to process incomming packets if we are finished
    if (next_tx_frame_id_ != cfg_->FramesToTest()) {
      ProcessControlInformation();
    }
  }
}
