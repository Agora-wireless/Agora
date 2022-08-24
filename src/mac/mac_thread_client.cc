/**
 * @file mac_thread.cc
 * @brief Implementation file for the MacThreadClient class.
 */
#include "mac_thread_client.h"

#include "gettime.h"
#include "logger.h"
#include "message.h"
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

  AGORA_LOG_INFO("MacThreadClient: Frame duration %.2f ms, tsc_delta %zu\n",
                 cfg_->GetFrameDurationSec() * 1000, tsc_delta_);

  // Set up buffers
  client_.ul_bits_buffer_id_.fill(0);
  client_.ul_bits_buffer_ = ul_bits_buffer;
  client_.ul_bits_buffer_status_ = ul_bits_buffer_status;

  server_.n_filled_in_frame_.fill(0);
  for (size_t ue_ant = 0; ue_ant < cfg_->UeAntTotal(); ue_ant++) {
    server_.data_size_.emplace_back(
        std::vector<size_t>(cfg->Frame().NumDlDataSyms()));
  }

  // The frame data will hold the data comming from the Phy (Received)
  for (auto& v : server_.frame_data_) {
    v.resize(cfg_->MacDataBytesNumPerframe(Direction::kDownlink));
  }

  const size_t udp_pkt_len = cfg_->MacDataBytesNumPerframe(Direction::kUplink);
  udp_pkt_buf_.resize(udp_pkt_len + kUdpRxBufferPadding);

  // TODO: See if it makes more sense to split up the UE's by port here for
  // client mode.
  const size_t udp_server_port = cfg_->UeMacRxPort();
  AGORA_LOG_INFO(
      "MacThreadClient: setting up udp server for mac data at port %zu\n",
      udp_server_port);
  udp_comm_ =
      std::make_unique<UDPComm>(cfg_->UeServerAddr(), udp_server_port,
                                udp_pkt_len * kMaxUEs * kMaxPktsPerUE, 0);

  const size_t udp_control_len = sizeof(RBIndicator);
  udp_control_buf_.resize(udp_control_len);

  AGORA_LOG_INFO(
      "MacThreadClient: setting up udp server for mac control channel at port "
      "%zu\n",
      kMacBaseClientPort);
  udp_control_channel_ =
      std::make_unique<UDPServer>(cfg_->UeServerAddr(), kMacBaseClientPort,
                                  udp_control_len * kMaxUEs * kMaxPktsPerUE);
  crc_obj_ = std::make_unique<DoCRC>();
}

MacThreadClient::~MacThreadClient() {
  std::fclose(log_file_);
  AGORA_LOG_INFO("MacThreadClient: MAC thread destroyed\n");
}

void MacThreadClient::ProcessRxFromPhy() {
  EventData event;
  if (rx_queue_->try_dequeue(event) == false) {
    return;
  }

  if (event.event_type_ == EventType::kPacketToMac) {
    AGORA_LOG_TRACE("MacThreadClient: MAC thread event kPacketToMac\n");
    ProcessCodeblocksFromPhy(event);
  } else if (event.event_type_ == EventType::kSNRReport) {
    AGORA_LOG_TRACE("MacThreadClient: MAC thread event kSNRReport\n");
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

void MacThreadClient::ProcessCodeblocksFromPhy(EventData event) {
  assert(event.event_type_ == EventType::kPacketToMac);

  const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
  const size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;

  // Helper variables (changes with bs / user)
  const size_t num_pilot_symbols = cfg_->Frame().ClientDlPilotSymbols();
  const size_t symbol_array_index = cfg_->Frame().GetDLSymbolIdx(symbol_id);
  const size_t data_symbol_index_start =
      cfg_->Frame().GetDLSymbol(num_pilot_symbols);
  const size_t data_symbol_index_end = cfg_->Frame().GetDLSymbolLast();
  const size_t mac_data_bytes_per_frame =
      cfg_->MacDataBytesNumPerframe(Direction::kDownlink);
  const size_t num_mac_packets_per_frame =
      cfg_->MacPacketsPerframe(Direction::kDownlink);
  const size_t dest_packet_size =
      cfg_->MacPayloadMaxLength(Direction::kDownlink);

  const int8_t* src_data =
      decoded_buffer_[(frame_id % kFrameWnd)][symbol_array_index][ue_id];

  std::stringstream ss;  // Debug-only

  // Only non-pilot data symbols have application data.
  if (symbol_array_index >= num_pilot_symbols) {
    // The decoded symbol knows nothing about the padding / storage of the data
    const auto* pkt = reinterpret_cast<const MacPacketPacked*>(src_data);
    // Destination only contains "payload"

    // TODO: enable ARQ and ensure reliable data goes to app
    const size_t frame_data_offset =
        (symbol_array_index - num_pilot_symbols) * dest_packet_size;

    // Who's junk is better? No reason to copy currupted data
    server_.n_filled_in_frame_.at(ue_id) += dest_packet_size;

    ss << "MacThreadClient: Received frame " << pkt->Frame() << ":" << frame_id
       << " symbol " << pkt->Symbol() << ":" << symbol_id << " user "
       << pkt->Ue() << ":" << ue_id << " length " << pkt->PayloadLength() << ":"
       << cfg_->MacPayloadMaxLength(Direction::kDownlink) << " crc "
       << pkt->Crc() << " copied to offset " << frame_data_offset << std::endl;

    if (kLogMacPackets) {
      ss << "Header Info:" << std::endl
         << "FRAME_ID: " << pkt->Frame() << std::endl
         << "SYMBOL_ID: " << pkt->Symbol() << std::endl
         << "UE_ID: " << pkt->Ue() << std::endl
         << "DATLEN: " << pkt->PayloadLength() << std::endl
         << "PAYLOAD:" << std::endl;
      for (size_t i = 0; i < cfg_->MacPayloadMaxLength(Direction::kDownlink);
           i++) {
        ss << std::to_string(pkt->Data()[i]) << " ";
      }
      ss << std::endl;
    }

    bool data_valid = false;
    // Data validity check
    if ((static_cast<size_t>(pkt->PayloadLength()) <= dest_packet_size) &&
        ((pkt->Symbol() >= data_symbol_index_start) &&
         (pkt->Symbol() <= data_symbol_index_end)) &&
        (pkt->Ue() <= cfg_->UeAntNum())) {
      auto crc = static_cast<uint16_t>(
          crc_obj_->CalculateCrc24(pkt->Data(), pkt->PayloadLength()) & 0xFFFF);

      data_valid = (crc == pkt->Crc());
    }

    if (data_valid) {
      AGORA_LOG_SYMBOL("%s", ss.str().c_str());
      AGORA_LOG_TRACE(
          "Looking at index ue %zu:%zu, offset %zu:%zu, length %d\nFrame Data "
          "size: %zu:%zu  %zu:%zu\n",
          ue_id, server_.frame_data_.size(), frame_data_offset,
          server_.frame_data_.at(ue_id).size(), pkt->PayloadLength(), ue_id,
          server_.data_size_.size(), symbol_array_index - num_pilot_symbols,
          server_.data_size_.at(ue_id).size());
      /// Spot to be optimized #1
      std::memcpy(&server_.frame_data_.at(ue_id).at(frame_data_offset),
                  pkt->Data(), pkt->PayloadLength());

      server_.data_size_.at(ue_id).at(symbol_array_index - num_pilot_symbols) =
          pkt->PayloadLength();

    } else {
      ss << "  *****Failed Data integrity check - invalid parameters"
         << std::endl;

      AGORA_LOG_ERROR("%s", ss.str().c_str());
      // Set the default to 0 valid data bytes
      server_.data_size_.at(ue_id).at(symbol_array_index - num_pilot_symbols) =
          0;
    }
    std::fprintf(log_file_, "%s", ss.str().c_str());
    ss.str("");
  }

  // When the frame is full, send it to the application
  if (server_.n_filled_in_frame_.at(ue_id) == mac_data_bytes_per_frame) {
    server_.n_filled_in_frame_.at(ue_id) = 0;
    /// Spot to be optimized #2 -- left shift data over to remove padding
    bool shifted = false;
    size_t src_offset = 0;
    size_t dest_offset = 0;
    for (size_t packet = 0; packet < num_mac_packets_per_frame; packet++) {
      const size_t rx_packet_size = server_.data_size_.at(ue_id).at(packet);
      if (rx_packet_size < dest_packet_size || (shifted == true)) {
        shifted = true;
        if (rx_packet_size > 0) {
          std::memmove(&server_.frame_data_.at(ue_id).at(dest_offset),
                       &server_.frame_data_.at(ue_id).at(src_offset),
                       rx_packet_size);
        }
      }
      dest_offset += rx_packet_size;
      src_offset += cfg_->MacPayloadMaxLength(Direction::kDownlink);
    }

    if (dest_offset > 0) {
      udp_comm_->Send(kMacRemoteHostname, cfg_->UeMacTxPort() + ue_id,
                      &server_.frame_data_.at(ue_id).at(0), dest_offset);
    }

    ss << "MacThreadClient: Sent data for frame " << frame_id << ", ue "
       << ue_id << ", size " << dest_offset << ":" << mac_data_bytes_per_frame
       << std::endl;

    if (kLogMacPackets) {
      std::fprintf(stdout, "%s", ss.str().c_str());
    }

    for (size_t i = 0u; i < dest_offset; i++) {
      ss << static_cast<uint8_t>(server_.frame_data_.at(ue_id).at(i)) << " ";
    }
    std::fprintf(log_file_, "%s", ss.str().c_str());
    ss.str("");
  }

  RtAssert(
      tx_queue_->enqueue(EventData(EventType::kPacketToMac, event.tags_[0])),
      "Socket message enqueue failed\n");
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
  const size_t max_data_bytes_per_frame =
      cfg_->MacDataBytesNumPerframe(Direction::kUplink);
  const size_t num_mac_packets_per_frame =
      cfg_->MacPacketsPerframe(Direction::kUplink);

  if (0 == max_data_bytes_per_frame) {
    return;
  }

  // Processes the packets of an entire frame (remove variable later)
  const size_t packets_required = num_mac_packets_per_frame;

  size_t packets_received = 0;
  size_t current_packet_bytes = 0;
  size_t current_packet_start_index = 0;

  size_t total_bytes_received = 0;

  const size_t max_recv_attempts = (packets_required * 10u);
  size_t rx_attempts;
  for (rx_attempts = 0u; rx_attempts < max_recv_attempts; rx_attempts++) {
    ssize_t ret = udp_comm_->Recv(&udp_pkt_buf_.at(total_bytes_received),
                                  (udp_pkt_buf_.size() - total_bytes_received));
    if (ret == 0) {
      AGORA_LOG_TRACE("MacThreadClient: No data received with %zu pending\n",
                      total_bytes_received);
      if (total_bytes_received == 0) {
        return;  // No data received
      } else {
        AGORA_LOG_INFO(
            "MacThreadClient: No data received but there was data in "
            "buffer pending %zu : try %zu out of %zu\n",
            total_bytes_received, rx_attempts, max_recv_attempts);
      }
    } else if (ret < 0) {
      // There was an error in receiving
      AGORA_LOG_ERROR("MacThreadClient: Error in reception %zu\n", ret);
      cfg_->Running(false);
      return;
    } else { /* Got some data */
      total_bytes_received += ret;
      current_packet_bytes += ret;

      // std::printf(
      //    "Received %zu bytes packet number %zu packet size %zu total %zu\n",
      //    ret, packets_received, total_bytes_received, current_packet_bytes);

      // While we have packets remaining and a header to process
      const size_t header_size = sizeof(MacPacketHeaderPacked);
      while ((packets_received < packets_required) &&
             (current_packet_bytes >= header_size)) {
        // See if we have enough data and process the MacPacket header
        const auto* rx_mac_packet_header =
            reinterpret_cast<const MacPacketPacked*>(
                &udp_pkt_buf_.at(current_packet_start_index));

        const size_t current_packet_size =
            header_size + rx_mac_packet_header->PayloadLength();

        // std::printf("Packet number %zu @ %zu packet size %d:%zu total %zu\n",
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
      AGORA_LOG_FRAME(
          "MacThreadClient: Received %zu : %zu bytes in packet %zu : %zu\n",
          ret, total_bytes_received, packets_received, packets_required);
    }

    // Check for completion
    if (packets_received == packets_required) {
      break;
    }
  }  // end rx attempts

  if (packets_received != packets_required) {
    AGORA_LOG_ERROR(
        "MacThreadClient: Received %zu : %zu packets with %zu total bytes in "
        "%zu attempts\n",
        packets_received, packets_required, total_bytes_received, rx_attempts);
  } else {
    AGORA_LOG_FRAME("MacThreadClient: Received Mac Frame Data\n");
  }
  RtAssert(
      packets_received == packets_required,
      "MacThreadClient: ProcessUdpPacketsFromApps incorrect data received!");

  // Currently this is a packet list of mac packets
  ProcessUdpPacketsFromAppsClient((char*)&udp_pkt_buf_[0], ri);
}

void MacThreadClient::ProcessUdpPacketsFromAppsClient(const char* payload,
                                                      RBIndicator /*ri*/) {
  const size_t num_mac_packets_per_frame =
      cfg_->MacPacketsPerframe(Direction::kUplink);
  const size_t num_pilot_symbols = cfg_->Frame().ClientUlPilotSymbols();
  // Data integrity check
  size_t pkt_offset = 0;
  size_t ue_id = 0;
  size_t symbol_id = 0;
  size_t frame_id = 0;
  for (size_t packet = 0u; packet < num_mac_packets_per_frame; packet++) {
    const auto* pkt =
        reinterpret_cast<const MacPacketPacked*>(&payload[pkt_offset]);

    // std::printf("Frame %d, Packet %zu, symbol %d, user %d\n", pkt->Frame(),
    //            packet, pkt->Symbol(), pkt->Ue());
    if (packet == 0) {
      ue_id = pkt->Ue();
      frame_id = pkt->Frame();
    } else {
      if (ue_id != pkt->Ue()) {
        AGORA_LOG_ERROR(
            "Received pkt %zu data with unexpected UE id %zu, expected %d\n",
            packet, ue_id, pkt->Ue());
      }
      if ((symbol_id + 1) != pkt->Symbol()) {
        AGORA_LOG_ERROR("Received out of order symbol id %d, expected %zu\n",
                        pkt->Symbol(), symbol_id + 1);
      }

      if (frame_id != pkt->Frame()) {
        AGORA_LOG_ERROR(
            "Received pkt %zu data with unexpected frame id %zu, expected %d\n",
            packet, frame_id, pkt->Frame());
      }
    }
    symbol_id = pkt->Symbol();
    pkt_offset += MacPacketPacked::kHeaderSize + pkt->PayloadLength();
  }

  if (next_radio_id_ != ue_id) {
    AGORA_LOG_ERROR("Error - radio id %zu, expected %zu\n", ue_id,
                    next_radio_id_);
  }
  // End data integrity check

  next_radio_id_ = ue_id;

  // We've received bits for the uplink.
  size_t& radio_buf_id = client_.ul_bits_buffer_id_[next_radio_id_];

  if ((*client_.ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] == 1) {
    std::fprintf(
        stderr,
        "MacThreadClient: UDP RX buffer full, buffer ID: %zu. Dropping "
        "rx frame data\n",
        radio_buf_id);
    return;
  }

  if (kLogMacPackets) {
    std::stringstream ss;
    std::fprintf(log_file_,
                 "MacThreadClient: Received data from app for frame %zu, ue "
                 "%zu size %zu\n",
                 next_tx_frame_id_, next_radio_id_, pkt_offset);

    for (size_t i = 0; i < pkt_offset; i++) {
      ss << std::to_string((uint8_t)(payload[i])) << " ";
    }
    std::fprintf(log_file_, "%s\n", ss.str().c_str());
  }

  size_t src_pkt_offset = 0;
  // Copy from the packet rx buffer into ul_bits memory
  for (size_t pkt_id = 0; pkt_id < num_mac_packets_per_frame; pkt_id++) {
    const auto* src_packet =
        reinterpret_cast<const MacPacketPacked*>(&payload[src_pkt_offset]);
    const size_t symbol_idx =
        cfg_->Frame().GetULSymbolIdx(src_packet->Symbol());
    // next_radio_id_ = src_packet->ue_id;

    // could use pkt_id vs src_packet->symbol_id_ but might reorder packets
    const size_t dest_pkt_offset = ((radio_buf_id * num_mac_packets_per_frame) +
                                    (symbol_idx - num_pilot_symbols)) *
                                   cfg_->MacPacketLength(Direction::kUplink);

    auto* pkt = reinterpret_cast<MacPacketPacked*>(
        &(*client_.ul_bits_buffer_)[next_radio_id_][dest_pkt_offset]);

    pkt->Set(next_tx_frame_id_, src_packet->Symbol(), src_packet->Ue(),
             src_packet->PayloadLength());

#if ENABLE_RB_IND
    pkt->rb_indicator_ = ri;
#endif

    pkt->LoadData(src_packet->Data());
    // Insert CRC
    pkt->Crc((uint16_t)(
        crc_obj_->CalculateCrc24(pkt->Data(), pkt->PayloadLength()) & 0xFFFF));

    if (kLogMacPackets) {
      std::stringstream ss;

      ss << "MacThreadClient: created packet frame " << next_tx_frame_id_
         << ", pkt " << pkt_id << ", size "
         << cfg_->MacPayloadMaxLength(Direction::kUplink) << " radio buff id "
         << radio_buf_id << ", loc " << (size_t)pkt << " dest offset "
         << dest_pkt_offset << std::endl;

      ss << "Header Info:" << std::endl
         << "FRAME_ID: " << pkt->Frame() << std::endl
         << "SYMBOL_ID: " << pkt->Symbol() << std::endl
         << "UE_ID: " << pkt->Ue() << std::endl
         << "DATLEN: " << pkt->PayloadLength() << std::endl
         << "PAYLOAD:" << std::endl;
      for (size_t i = 0; i < pkt->PayloadLength(); i++) {
        ss << std::to_string(pkt->Data()[i]) << " ";
      }
      ss << std::endl;
      std::fprintf(stdout, "%s", ss.str().c_str());
      std::fprintf(log_file_, "%s", ss.str().c_str());
      ss.str("");
    }
    src_pkt_offset += pkt->PayloadLength() + MacPacketPacked::kHeaderSize;
  }  // end all packets

  (*client_.ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] = 1;
  EventData msg(EventType::kPacketFromMac,
                rx_mac_tag_t(next_radio_id_, radio_buf_id).tag_);

  AGORA_LOG_FRAME("MacThreadClient: Tx mac information to %zu %zu\n",
                  next_radio_id_, radio_buf_id);
  RtAssert(tx_queue_->enqueue(msg),
           "MacThreadClient: Failed to enqueue uplink packet");

  radio_buf_id = (radio_buf_id + 1) % kFrameWnd;
  // Might be unnecessary now.
  next_radio_id_ = (next_radio_id_ + 1) % cfg_->UeAntNum();
  if (next_radio_id_ == 0) {
    next_tx_frame_id_++;
  }
}

void MacThreadClient::RunEventLoop() {
  AGORA_LOG_INFO(
      "MacThreadClient: Running MAC thread event loop, logging to file %s\n",
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
