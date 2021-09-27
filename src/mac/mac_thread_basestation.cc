/**
 * @file mac_thread.cc
 * @brief Implementation file for the MacThreadBaseStation class.
 */
#include "mac_thread_basestation.h"

#include "logger.h"
#include "utils_ldpc.h"

static constexpr size_t kUdpRxBufferPadding = 2048u;

MacThreadBaseStation::MacThreadBaseStation(
    Config* cfg, size_t core_offset,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffer,
    Table<int8_t>* dl_bits_buffer, Table<int8_t>* dl_bits_buffer_status,
    moodycamel::ConcurrentQueue<EventData>* rx_queue,
    moodycamel::ConcurrentQueue<EventData>* tx_queue,
    const std::string& log_filename)
    : cfg_(cfg),
      freq_ghz_(GetTime::MeasureRdtscFreq()),
      tsc_delta_((cfg_->GetFrameDurationSec() * 1e9) / freq_ghz_),
      core_offset_(core_offset),
      decoded_buffer_(decoded_buffer),
      dl_bits_buffer_(dl_bits_buffer),
      dl_bits_buffer_status_(dl_bits_buffer_status),
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

  MLPD_INFO("MacThreadBaseStation: Frame duration %.2f ms, tsc_delta %zu\n",
            cfg_->GetFrameDurationSec() * 1000, tsc_delta_);

  // Set up buffers
  dl_bits_buffer_id_.fill(0);

  server_.n_filled_in_frame_.fill(0);
  for (size_t ue_ant = 0; ue_ant < cfg_->UeAntTotal(); ue_ant++) {
    server_.data_size_.emplace_back(
        std::vector<size_t>(cfg->Frame().NumUlDataSyms()));
  }

  for (auto& v : server_.frame_data_) {
    v.resize(cfg_->UlMacDataBytesNumPerframe());
  }

  const size_t udp_pkt_len = cfg_->DlMacDataBytesNumPerframe();
  udp_pkt_buf_.resize(udp_pkt_len + kUdpRxBufferPadding);

  // TODO: See if it makes more sense to split up the UE's by port here for
  // client mode.
  size_t udp_server_port = cfg_->BsMacRxPort();
  MLPD_INFO("MacThreadBaseStation: setting up udp server at port %zu\n",
            udp_server_port);
  udp_server_ = std::make_unique<UDPServer>(
      udp_server_port, udp_pkt_len * kMaxUEs * kMaxPktsPerUE);

  const size_t udp_control_len = sizeof(RBIndicator);
  udp_control_buf_.resize(udp_control_len);

  udp_client_ = std::make_unique<UDPClient>();
  crc_obj_ = std::make_unique<DoCRC>();
}

MacThreadBaseStation::~MacThreadBaseStation() {
  std::fclose(log_file_);
  MLPD_INFO("MAC thread destroyed\n");
}

void MacThreadBaseStation::ProcessRxFromPhy() {
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

void MacThreadBaseStation::ProcessSnrReportFromPhy(EventData event) {
  const size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
  if (server_.snr_[ue_id].size() == kSNRWindowSize) {
    server_.snr_[ue_id].pop();
  }

  float snr;
  std::memcpy(&snr, &event.tags_[1], sizeof(float));
  server_.snr_[ue_id].push(snr);
}

void MacThreadBaseStation::SendRanConfigUpdate(EventData /*event*/) {
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

void MacThreadBaseStation::ProcessCodeblocksFromPhy(EventData event) {
  assert(event.event_type_ == EventType::kPacketToMac);

  const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  const size_t symbol_id = gen_tag_t(event.tags_[0]).symbol_id_;
  const size_t symbol_idx_ul = this->cfg_->Frame().GetULSymbolIdx(symbol_id);
  const size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
  const int8_t* ul_data_ptr =
      decoded_buffer_[frame_id % kFrameWnd][symbol_idx_ul][ue_id];

  std::stringstream ss;  // Debug-only

  // Only non-pilot uplink symbols have application data.
  if (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols()) {
    auto* pkt = reinterpret_cast<const MacPacket*>(ul_data_ptr);
    const size_t dest_packet_size = cfg_->MacPayloadLength();

    // We send data to app irrespective of CRC condition
    // TODO: enable ARQ and ensure reliable data goes to app
    const size_t frame_data_offset =
        (symbol_idx_ul - cfg_->Frame().ClientUlPilotSymbols()) *
        dest_packet_size;

    //Who's junk is better? No reason to copy currupted data
    server_.n_filled_in_frame_[ue_id] += dest_packet_size;

    bool data_valid = false;
    //Data validity check
    if ((static_cast<size_t>(pkt->datalen_) <= dest_packet_size) &&
        (pkt->symbol_id_ >=
         cfg_->Frame().GetULSymbol(cfg_->Frame().ClientUlPilotSymbols())) &&
        (pkt->symbol_id_ <= cfg_->Frame().GetULSymbolLast()) &&
        (pkt->ue_id_ <= cfg_->UeAntNum())) {
      auto crc = static_cast<uint16_t>(
          crc_obj_->CalculateCrc24((unsigned char*)pkt->data_, pkt->datalen_) &
          0xFFFF);

      data_valid = (crc == pkt->crc_);
    }

    if (data_valid) {
      MLPD_FRAME(
          "Base Station MAC thread received frame %zu, uplink "
          "symbol index %zu, size %zu, copied to frame data offset %zu\n",
          frame_id, symbol_idx_ul, cfg_->MacPayloadLength(), frame_data_offset);
      /// Spot to be optimized #1
      std::memcpy(&server_.frame_data_[ue_id][frame_data_offset], pkt->data_,
                  pkt->datalen_);

      server_.data_size_.at(ue_id).at(
          this->cfg_->Frame().GetULSymbolIdx(pkt->symbol_id_) -
          cfg_->Frame().ClientUlPilotSymbols()) = pkt->datalen_;
      // Print information about the received symbol
      if (kLogMacPackets) {
        std::fprintf(
            log_file_,
            "Base Station MAC thread received frame %zu, uplink "
            "symbol index %zu, size %zu, copied to frame data offset %zu\n",
            frame_id, symbol_idx_ul, cfg_->MacPayloadLength(),
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
      MLPD_ERROR(
          "Failed Data integrity check - invalid parameters frame %d:%zu "
          "symbol %d:%zu user %d:%zu length %d crc %d\n",
          pkt->frame_id_, frame_id, pkt->symbol_id_, symbol_id, pkt->ue_id_,
          ue_id, pkt->datalen_, pkt->crc_);
      //Set the default to 0 valid data bytes
      server_.data_size_.at(ue_id).at(
          this->cfg_->Frame().GetULSymbolIdx(symbol_id) -
          cfg_->Frame().ClientUlPilotSymbols()) = 0;
    }
  }

  // When the frame is full, send it to the application
  if (server_.n_filled_in_frame_[ue_id] == cfg_->UlMacDataBytesNumPerframe()) {
    server_.n_filled_in_frame_[ue_id] = 0;
    ///Spot to be optimized #2 -- left shift data over to remove padding
    bool shifted = false;
    size_t src_offset = 0;
    size_t dest_offset = 0;
    for (size_t packet = 0; packet < cfg_->UlMacPacketsPerframe(); packet++) {
      const size_t rx_packet_size = server_.data_size_.at(ue_id).at(packet);
      if (rx_packet_size < cfg_->MacPayloadLength() || (shifted == true)) {
        shifted = true;
        if (rx_packet_size > 0) {
          std::memmove(&server_.frame_data_[ue_id][dest_offset],
                       &server_.frame_data_[ue_id][src_offset], rx_packet_size);
        }
        dest_offset += rx_packet_size;
        src_offset += cfg_->MacPayloadLength();
      }
    }

    if (dest_offset > 0) {
      udp_client_->Send(kMacRemoteHostname, cfg_->BsMacTxPort() + ue_id,
                        &server_.frame_data_[ue_id][0], dest_offset);
    }

    std::fprintf(
        stdout, "MAC thread: Sent data for frame %zu, ue %zu, size %zu:%zu\n",
        frame_id, ue_id, dest_offset, cfg_->UlMacDataBytesNumPerframe());

    std::fprintf(log_file_,
                 "MAC thread: Sent data for frame %zu, ue %zu, size %zu:%zu\n",
                 frame_id, ue_id, dest_offset,
                 cfg_->UlMacDataBytesNumPerframe());
    for (size_t i = 0u; i < dest_offset; i++) {
      ss << std::to_string(server_.frame_data_[ue_id][i]) << " ";
    }
    std::fprintf(log_file_, "%s\n", ss.str().c_str());
    ss.str("");
  }

  RtAssert(
      tx_queue_->enqueue(EventData(EventType::kPacketToMac, event.tags_[0])),
      "Socket message enqueue failed\n");
}

void MacThreadBaseStation::SendControlInformation() {
  // send RAN control information UE
  RBIndicator ri;
  ri.ue_id_ = next_radio_id_;
  ri.mod_order_bits_ = CommsLib::kQaM16;
  udp_client_->Send(cfg_->UeServerAddr(), kMacBaseClientPort + ri.ue_id_,
                    (uint8_t*)&ri, sizeof(RBIndicator));

  // update RAN config within Agora
  SendRanConfigUpdate(EventData(EventType::kRANUpdate));
}

void MacThreadBaseStation::ProcessUdpPacketsFromApps() {
  if (0 == cfg_->DlMacDataBytesNumPerframe()) return;

  size_t rx_bytes = 0;
  size_t rx_bytes_required = cfg_->DlMacDataBytesNumPerframe();
  const size_t max_recv_attempts = (cfg_->DlMacPacketsPerframe() * 10u);
  size_t rx_attempts;
  for (rx_attempts = 0u; rx_attempts < max_recv_attempts; rx_attempts++) {
    ssize_t ret = udp_server_->Recv(&udp_pkt_buf_.at(rx_bytes),
                                    udp_pkt_buf_.size() - rx_bytes);
    if (ret == 0) {
      MLPD_FRAME("MacThreadBaseStation: No data received\n");
      if (rx_bytes == 0) {
        return;  // No data received
      } else {
        MLPD_FRAME(
            "MacThreadBaseStation: No data received but there was data in "
            "buffer pending %zu : try %zu out of %zu\n",
            rx_bytes, rx_attempts, max_recv_attempts);
      }
    } else if (ret < 0) {
      // There was an error in receiving
      MLPD_ERROR("MacThreadBaseStation: Error in reception %zu\n", ret);
      cfg_->Running(false);
      return;
    } else {
      rx_bytes += ret;
      MLPD_TRACE("MacThreadBaseStation: Received %zu : %zu bytes\n", ret,
                 rx_bytes_required);
      if (rx_bytes >= rx_bytes_required) {
        MLPD_FRAME("MacThreadBaseStation rx full frame data %zu\n", rx_bytes);
        break;
      }
    }
  }

  if (rx_bytes != cfg_->DlMacDataBytesNumPerframe()) {
    MLPD_ERROR(
        "MacThreadBaseStation: Received %zu : %zu bytes in %zu attempts\n",
        rx_bytes, cfg_->DlMacDataBytesNumPerframe(), rx_attempts);
  } else {
    MLPD_FRAME("MacThreadBaseStation: Received Mac Frame Data\n");
  }
  RtAssert(rx_bytes == cfg_->DlMacDataBytesNumPerframe(),
           "MacThreadBaseStation:ProcessUdpPacketsFromApps incorrect number of "
           "bytes received.");

  const char* payload = reinterpret_cast<char*>(&udp_pkt_buf_[0]);

  // We've received bits for the uplink. The received MAC packet does not
  // specify a radio ID, send to radios in round-robin order
  size_t& radio_buf_id = dl_bits_buffer_id_[next_radio_id_];

  if ((*dl_bits_buffer_status_)[next_radio_id_][radio_buf_id] == 1) {
    std::fprintf(stderr,
                 "MAC thread: UDP RX buffer full, buffer ID: %zu. Dropping "
                 "packet.\n",
                 radio_buf_id);
    return;
  }

  RBIndicator ri;
  ri.ue_id_ = next_radio_id_;
  ri.mod_order_bits_ = CommsLib::kQaM16;
  for (size_t pkt_id = 0; pkt_id < cfg_->DlMacPacketsPerframe(); pkt_id++) {
    size_t pkt_offset = radio_buf_id * cfg_->DlMacBytesNumPerframe() +
                        pkt_id * cfg_->MacPacketLength();
    auto* pkt = reinterpret_cast<MacPacket*>(
        &(*dl_bits_buffer_)[next_radio_id_][pkt_offset]);

    pkt->frame_id_ = next_tx_frame_id_;
    pkt->symbol_id_ = pkt_id;
    pkt->ue_id_ = next_radio_id_;
    pkt->datalen_ = cfg_->MacPayloadLength();
    pkt->rsvd_[0] = static_cast<uint16_t>(fast_rand_.NextU32() >> 16);
    pkt->rsvd_[1] = static_cast<uint16_t>(fast_rand_.NextU32() >> 16);
    pkt->rsvd_[2] = static_cast<uint16_t>(fast_rand_.NextU32() >> 16);
    pkt->rb_indicator_ = ri;

    std::memcpy(pkt->data_, payload + pkt_id * cfg_->MacPayloadLength(),
                pkt->datalen_);
    // Insert CRC
    pkt->crc_ = (uint16_t)(crc_obj_->CalculateCrc24((unsigned char*)pkt->data_,
                                                    pkt->datalen_) &
                           0xFFFF);

    if (kLogMacPackets) {
      std::stringstream ss;
      std::printf(
          "Base Station MAC thread created packet frame %zu, pkt %zu, size "
          "%zu, copied to location %zu\n",
          next_tx_frame_id_, pkt_id, cfg_->MacPayloadLength(), (size_t)pkt);

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

  (*dl_bits_buffer_status_)[next_radio_id_][radio_buf_id] = 1;
  EventData msg(EventType::kPacketFromMac,
                rx_mac_tag_t(next_radio_id_, next_tx_frame_id_).tag_);
  RtAssert(tx_queue_->enqueue(msg),
           "MAC thread: Failed to enqueue uplink packet");

  radio_buf_id = (radio_buf_id + 1) % kFrameWnd;
  next_radio_id_ = (next_radio_id_ + 1) % cfg_->UeAntNum();
  if (next_radio_id_ == 0) {
    next_tx_frame_id_++;
  }
}

void MacThreadBaseStation::RunEventLoop() {
  MLPD_INFO("Running MAC thread event loop, logging to file %s\n",
            log_filename_.c_str());
  PinToCoreWithOffset(ThreadType::kWorkerMacTXRX, core_offset_,
                      0 /* thread ID */);

  size_t last_frame_tx_tsc = 0;

  while (cfg_->Running() == true) {
    ProcessRxFromPhy();

    if ((GetTime::Rdtsc() - last_frame_tx_tsc) > tsc_delta_) {
      SendControlInformation();
      last_frame_tx_tsc = GetTime::Rdtsc();
    }

    // No need to process incomming packets if we are finished
    if (next_tx_frame_id_ != cfg_->FramesToTest()) {
      ProcessUdpPacketsFromApps();
    }
  }
}
