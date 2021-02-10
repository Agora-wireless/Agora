/**
 * @file mac_thread.cc
 * @brief Implementation file for the MacThread class.
 */
#include "mac_thread.h"

#include "logger.h"
#include "utils_ldpc.h"

MacThread::MacThread(
    Mode mode, Config* cfg, size_t core_offset,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffer,
    Table<uint8_t>* ul_bits_buffer, Table<uint8_t>* ul_bits_buffer_status,
    Table<uint8_t>* dl_bits_buffer, Table<uint8_t>* dl_bits_buffer_status,
    moodycamel::ConcurrentQueue<EventData>* rx_queue,
    moodycamel::ConcurrentQueue<EventData>* tx_queue,
    const std::string& log_filename)
    : mode_(mode),
      cfg_(cfg),
      freq_ghz_(MeasureRdtscFreq()),
      tsc_delta_((cfg_->GetFrameDurationSec() * 1e9) / freq_ghz_),
      core_offset_(core_offset),
      decoded_buffer_(decoded_buffer),
      dl_bits_buffer_(dl_bits_buffer),
      dl_bits_buffer_status_(dl_bits_buffer_status),
      rx_queue_(rx_queue),
      tx_queue_(tx_queue) {
  // Set up MAC log file
  if (!log_filename.empty()) {
    log_filename_ = log_filename;  // Use a non-default log filename
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
    v.resize(cfg_->MacDataBytesNumPerframe());
  }

  client_.ul_bits_buffer_id_.fill(0);

  const size_t udp_pkt_len = cfg_->MacDataBytesNumPerframe();
  udp_pkt_buf_.resize(udp_pkt_len);
  udp_server_ =
      new UDPServer(kLocalPort, udp_pkt_len * kMaxUEs * kMaxPktsPerUE);

  const size_t udp_control_len = sizeof(RBIndicator);
  udp_control_buf_.resize(udp_control_len);
  udp_control_channel_ =
      new UDPServer(kBaseClientPort, udp_control_len * kMaxUEs * kMaxPktsPerUE);

  udp_client_ = new UDPClient();
  crc_obj_ = new DoCRC();
}

MacThread::~MacThread() {
  std::fclose(log_file_);
  MLPD_INFO("MAC thread destroyed\n");
}

void MacThread::ProcessRxFromMaster() {
  EventData event;
  if (!rx_queue_->try_dequeue(event)) {
    return;
  }

  if (event.event_type_ == EventType::kPacketToMac) {
    ProcessCodeblocksFromMaster(event);
  } else if (event.event_type_ == EventType::kSNRReport) {
    ProcessSnrReportFromMaster(event);
  }
}

void MacThread::ProcessSnrReportFromMaster(EventData event) {
  const size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
  if (server_.snr_[ue_id].size() == kSNRWindowSize) {
    server_.snr_[ue_id].pop();
  }

  float snr;
  std::memcpy(&snr, &event.tags_[1], sizeof(float));
  server_.snr_[ue_id].push(snr);
}

void MacThread::SendRanConfigUpdate(EventData /*event*/) {
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

void MacThread::ProcessCodeblocksFromMaster(EventData event) {
  assert(event.event_type_ == EventType::kPacketToMac);

  const size_t frame_id = gen_tag_t(event.tags_[0]).frame_id_;
  const size_t symbol_idx_ul = gen_tag_t(event.tags_[0]).symbol_id_;
  const size_t ue_id = gen_tag_t(event.tags_[0]).ue_id_;
  const uint8_t* ul_data_ptr =
      decoded_buffer_[frame_id % kFrameWnd][symbol_idx_ul][ue_id];

  std::stringstream ss;  // Debug-only

  // Only non-pilot uplink symbols have application data.
  if (symbol_idx_ul >= cfg_->Frame().ClientUlPilotSymbols()) {
    auto* pkt = (struct MacPacket*)ul_data_ptr;

    // We send data to app irrespective of CRC condition
    // TODO: enable ARQ and ensure reliable data goes to app
    const size_t frame_data_offset =
        (symbol_idx_ul - cfg_->Frame().ClientUlPilotSymbols()) *
        cfg_->MacPayloadLength();
    std::memcpy(&server_.frame_data_[ue_id][frame_data_offset], pkt->data_,
                cfg_->MacPayloadLength());
    server_.n_filled_in_frame_[ue_id] += cfg_->MacPayloadLength();

    // Check CRC
    uint16_t crc =
        (uint16_t)(crc_obj_->CalculateCrc24((unsigned char*)pkt->data_,
                                            cfg_->MacPayloadLength()) &
                   0xFFFF);
    if (crc == pkt->crc_) {
      // Print information about the received symbol
      if (kLogMacPackets) {
        std::fprintf(log_file_,
                     "MAC thread received frame %zu, uplink symbol index %zu, "
                     "size %zu, copied to frame data offset %zu\n",
                     frame_id, symbol_idx_ul, cfg_->MacPayloadLength(),
                     frame_data_offset);

        ss << "Header Info:\n"
           << "FRAME_ID: " << pkt->frame_id_
           << "\nSYMBOL_ID: " << pkt->symbol_id_ << "\nUE_ID: " << pkt->ue_id_
           << "\nDATLEN: " << pkt->datalen_ << "\nPAYLOAD:\n";
        for (size_t i = 0; i < cfg_->MacPayloadLength(); i++) {
          ss << std::to_string(ul_data_ptr[i]) << " ";
        }
        std::fprintf(log_file_, "%s\n", ss.str().c_str());
        ss.str("");
      }
    } else {
      std::printf("Bad Packet: CRC Check Failed! \n");
    }
  }

  // When the frame is full, send it to the application
  if (server_.n_filled_in_frame_[ue_id] == cfg_->MacDataBytesNumPerframe()) {
    server_.n_filled_in_frame_[ue_id] = 0;

    udp_client_->Send(k_remote_hostname_, kBaseRemotePort + ue_id,
                      &server_.frame_data_[ue_id][0],
                      cfg_->MacDataBytesNumPerframe());
    std::fprintf(log_file_,
                 "MAC thread: Sent data for frame %zu, ue %zu, size %zu\n",
                 frame_id, ue_id, cfg_->MacDataBytesNumPerframe());
    for (size_t i = 0; i < cfg_->MacDataBytesNumPerframe(); i++) {
      ss << std::to_string(server_.frame_data_[ue_id][i]) << " ";
    }
    std::fprintf(log_file_, "%s\n", ss.str().c_str());
    ss.str("");
  }

  RtAssert(
      tx_queue_->enqueue(EventData(EventType::kPacketToMac, event.tags_[0])),
      "Socket message enqueue failed\n");
}

void MacThread::SendControlInformation() {
  // send RAN control information UE
  RBIndicator ri;
  ri.ue_id_ = next_radio_id_;
  ri.mod_order_bits_ = CommsLib::kQaM16;
  udp_client_->Send(cfg_->UeServerAddr(), kBaseClientPort + ri.ue_id_,
                    (uint8_t*)&ri, sizeof(RBIndicator));

  // update RAN config within Agora
  SendRanConfigUpdate(EventData(EventType::kRANUpdate));
}

void MacThread::ProcessControlInformation() {
  std::memset(&udp_control_buf_[0], 0, udp_control_buf_.size());
  ssize_t ret = udp_control_channel_->RecvNonblocking(&udp_control_buf_[0],
                                                      udp_control_buf_.size());
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

void MacThread::ProcessUdpPacketsFromApps(RBIndicator ri) {
  std::memset(&udp_pkt_buf_[0], 0, udp_pkt_buf_.size());
  ssize_t ret =
      udp_server_->RecvNonblocking(&udp_pkt_buf_[0], udp_pkt_buf_.size());
  if (ret == 0) {
    return;  // No data received
  } else if (ret == -1) {
    // There was an error in receiving
    cfg_->Running(false);
    return;
  }
  RtAssert(static_cast<size_t>(ret) == cfg_->MacDataBytesNumPerframe());

  const auto* pkt = reinterpret_cast<MacPacket*>(&udp_pkt_buf_[0]);
  mode_ == Mode::kServer ? ProcessUdpPacketsFromAppsServer(pkt, ri)
                         : ProcessUdpPacketsFromAppsClient((char*)pkt, ri);
}

void MacThread::ProcessUdpPacketsFromAppsServer(const MacPacket* pkt,
                                                RBIndicator /*ri*/) {
  // We've received bits for the downlink
  const size_t total_symbol_idx =
      cfg_->GetTotalDataSymbolIdxDl(pkt->frame_id_, pkt->symbol_id_);
  const size_t rx_offset =
      total_symbol_idx * cfg_->LdpcConfig().NumBlocksInSymbol();

  if ((*dl_bits_buffer_status_)[pkt->ue_id_][rx_offset] == 1) {
    MLPD_ERROR("MAC thread: dl_bits_buffer full, offset %zu. Exiting.\n",
               rx_offset);
    cfg_->Running(false);
    return;
  }

  for (size_t i = 0; i < cfg_->LdpcConfig().NumBlocksInSymbol(); i++) {
    (*dl_bits_buffer_status_)[pkt->ue_id_][rx_offset + i] = 1;
  }
  std::memcpy(
      &(*dl_bits_buffer_)[total_symbol_idx][pkt->ue_id_ * cfg_->OfdmDataNum()],
      pkt->data_, udp_pkt_buf_.size());

  EventData msg(
      EventType::kPacketFromMac,
      gen_tag_t::FrmSymUe(pkt->frame_id_, pkt->symbol_id_, pkt->ue_id_).tag_);
  RtAssert(tx_queue_->enqueue(msg),
           "MAC thread: Failed to enqueue downlink packet");
}

void MacThread::ProcessUdpPacketsFromAppsClient(const char* payload,
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
        next_frame_id_, next_radio_id_, cfg_->MacDataBytesNumPerframe());

    for (size_t i = 0; i < cfg_->MacDataBytesNumPerframe(); i++) {
      ss << std::to_string((uint8_t)(payload[i])) << " ";
    }
    std::fprintf(log_file_, "%s\n", ss.str().c_str());
  }

  for (size_t pkt_id = 0; pkt_id < cfg_->MacPacketsPerframe(); pkt_id++) {
    size_t data_offset = radio_buf_id * cfg_->MacBytesNumPerframe() +
                         pkt_id * cfg_->MacPacketLength();
    auto* pkt =
        (MacPacket*)(&(*client_.ul_bits_buffer_)[next_radio_id_][data_offset]);
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

void MacThread::RunEventLoop() {
  MLPD_INFO("Running MAC thread event loop, logging to file %s\n",
            log_filename_.c_str());
  PinToCoreWithOffset(ThreadType::kWorkerMacTXRX, core_offset_,
                      0 /* thread ID */);

  while (cfg_->Running()) {
    ProcessRxFromMaster();

    if (mode_ == Mode::kServer) {
      if (Rdtsc() - last_frame_tx_tsc_ > tsc_delta_) {
        SendControlInformation();
        last_frame_tx_tsc_ = Rdtsc();
      }
    } else {
      ProcessControlInformation();
    }

    if (next_frame_id_ == cfg_->FramesToTest()) {
      MLPD_WARN(
          "MAC thread stopping. Next frame ID = %zu, configured "
          "frames to test = %zu\n",
          next_frame_id_, cfg_->FramesToTest());
      break;
    }
  }
}

// TODO: Integrate process_codeblocks_from_master_client() for downlink at the
// client, based on this excerpt from txrx_mac.cpp

/*
int PacketTXRX::dequeue_send(int tid)
{
    auto& c = config_;
    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;

    // std::printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(event.event_type == EventType::kPacketTX);

    size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    size_t data_symbol_idx = gen_tag_t(event.tags[0]).symbol_id;

    size_t offset = (c->GetTotalDataSymbolIdx(frame_id, data_symbol_idx)
                        * c->BsAntNum())
        + ant_id;

    if (kDebugPrintInTask) {
        std::printf("In TX thread %d: Transmitted frame %zu, symbol %zu, "
               "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, data_symbol_idx, ant_id,
            gen_tag_t(event.tags[0])._tag, offset,
            message_queue_->size_approx());
    }

    size_t socket_symbol_offset = offset
        % (kFrameWnd * c->Frame().NumDataSyms()
              * c->BsAntNum());
    char* cur_buffer_ptr = tx_buffer_ + socket_symbol_offset *
c->PacketLength(); auto* pkt = (Packet*)cur_buffer_ptr; new (pkt)
Packet(frame_id, data_symbol_idx, 0, ant_id);

    // Send data (one OFDM symbol)
    ssize_t ret = sendto(socket_[ant_id % config_->SocketThreadNum()],
        cur_buffer_ptr, c->PacketLength(), 0, (struct
sockaddr*)&servaddr_[tid], sizeof(servaddr_[tid])); rt_assert(ret > 0, "sendto()
failed");

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}
*/
