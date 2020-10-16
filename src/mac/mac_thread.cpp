#include "mac_thread.hpp"
#include "logger.h"
#include "utils_ldpc.hpp"

MacThread::MacThread(Mode mode, Config* cfg, size_t core_offset,
    PtrGrid<kFrameWnd, kMaxUEs, uint8_t>& decoded_buffer,
    Table<uint8_t>* ul_bits_buffer, Table<uint8_t>* ul_bits_buffer_status,
    Table<uint8_t>* dl_bits_buffer, Table<uint8_t>* dl_bits_buffer_status,
    moodycamel::ConcurrentQueue<Event_data>* rx_queue,
    moodycamel::ConcurrentQueue<Event_data>* tx_queue, std::string log_filename)
    : mode_(mode)
    , cfg_(cfg)
    , freq_ghz_(measure_rdtsc_freq())
    , tsc_delta_((cfg_->get_frame_duration_sec() * 1e9) / freq_ghz_)
    , core_offset_(core_offset)
    , decoded_buffer_(decoded_buffer)
    , dl_bits_buffer_(dl_bits_buffer)
    , dl_bits_buffer_status_(dl_bits_buffer_status)
    , rx_queue_(rx_queue)
    , tx_queue_(tx_queue)
{
    // Set up MAC log file
    if (log_filename != "") {
        log_filename_ = log_filename; // Use a non-default log filename
    }
    log_file_ = fopen(log_filename_.c_str(), "w");
    rt_assert(log_file_ != nullptr, "Failed to open MAC log file");

    printf("MAC thread: Frame duration %.2f ms, tsc_delta %zu\n",
        cfg_->get_frame_duration_sec() * 1000, tsc_delta_);

    // Set up buffers
    client_.ul_bits_buffer_ = ul_bits_buffer;
    client_.ul_bits_buffer_status_ = ul_bits_buffer_status;

    server_.n_filled_in_frame_.fill(0);
    for (auto& v : server_.frame_data_)
        v.resize(cfg_->mac_data_bytes_num_perframe);

    client_.ul_bits_buffer_id_.fill(0);

    const size_t udp_pkt_len = cfg_->mac_data_bytes_num_perframe;
    udp_pkt_buf_.resize(udp_pkt_len);
    udp_server
        = new UDPServer(kLocalPort, udp_pkt_len * kMaxUEs * kMaxPktsPerUE);

    const size_t udp_control_len = sizeof(RBIndicator);
    udp_control_buf_.resize(udp_control_len);
    udp_control_channel = new UDPServer(
        kBaseClientPort, udp_control_len * kMaxUEs * kMaxPktsPerUE);

    udp_client = new UDPClient();
    crc_obj = new DoCRC();
}

MacThread::~MacThread()
{
    fclose(log_file_);
    MLPD_INFO("MAC thread destroyed\n");
}

void MacThread::process_rx_from_master()
{
    Event_data event;
    if (!rx_queue_->try_dequeue(event))
        return;

    if (event.event_type == EventType::kPacketToMac)
        process_codeblocks_from_master(event);
    else if (event.event_type == EventType::kSNRReport)
        process_snr_report_from_master(event);
}

void MacThread::process_snr_report_from_master(Event_data event)
{
    const size_t ue_id = gen_tag_t(event.tags[0]).ue_id;
    if (server_.snr_[ue_id].size() == kSNRWindowSize) {
        server_.snr_[ue_id].pop();
    }

    float snr;
    memcpy(&snr, &event.tags[1], sizeof(float));
    server_.snr_[ue_id].push(snr);
}

void MacThread::send_ran_config_update(Event_data event)
{
    RanConfig rc;
    rc.n_antennas = 0; // TODO [arjun]: What's the correct value here?
    rc.mod_order_bits = CommsLib::QAM16;
    rc.frame_id = scheduler_next_frame_id_;
    // TODO: change n_antennas to a desired value
    // cfg_->BS_ANT_NUM is added to fix compiler warning
    rc.n_antennas = cfg_->BS_ANT_NUM;

    Event_data msg(EventType::kRANUpdate);
    msg.num_tags = 3;
    msg.tags[0] = rc.n_antennas;
    msg.tags[1] = rc.mod_order_bits;
    msg.tags[2] = rc.frame_id;
    rt_assert(tx_queue_->enqueue(msg),
        "MAC thread: failed to send RAN update to Agora");

    scheduler_next_frame_id_++;
}

void MacThread::process_codeblocks_from_master(Event_data event)
{
    assert(event.event_type == EventType::kPacketToMac);

    const size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    const size_t cb_id = gen_tag_t(event.tags[0]).cb_id;
    const size_t ue_id = gen_tag_t(event.tags[0]).cb_ue_id;
    const uint8_t* ul_data_ptr = decoded_buffer_[frame_id % kFrameWnd][ue_id]
        + cb_id * roundup<64>(cfg_->num_bytes_per_cb);

    std::stringstream ss; // Debug-only

    auto* pkt = (struct MacPacket*)ul_data_ptr;

    // We send data to app irrespective of CRC condition
    // TODO: enable ARQ and ensure reliable data goes to app
    const size_t frame_data_offset = cb_id * cfg_->mac_payload_length;
    memcpy(&server_.frame_data_[ue_id][frame_data_offset], pkt->data,
        cfg_->mac_payload_length);
    server_.n_filled_in_frame_[ue_id] += cfg_->mac_payload_length;

    // Check CRC
    uint16_t crc
        = (uint16_t)(crc_obj->calculate_crc24(
                         (unsigned char*)pkt->data, cfg_->mac_payload_length)
            & 0xFFFF);
    if (crc == pkt->crc) {

        // Print information about the received symbol
        if (kLogMacPackets) {
            fprintf(log_file_,
                "MAC thread received frame %zu, ue %zu, cb %zu, "
                "size %zu, copied to frame data offset %zu\n",
                frame_id, ue_id, cb_id, cfg_->mac_payload_length,
                frame_data_offset);

            ss << "Header Info:\n"
               << "FRAME_ID: " << pkt->frame_id << "\nUE_ID: " << pkt->ue_id
               << "\nCB_ID: " << pkt->cb_id << "\nDATLEN: " << pkt->datalen
               << "\nPAYLOAD:\n";
            for (size_t i = 0; i < cfg_->mac_payload_length; i++) {
                ss << std::to_string(ul_data_ptr[i]) << " ";
            }
            fprintf(log_file_, "%s\n", ss.str().c_str());
            ss.str("");
        }
    } else {
        printf("Bad Packet: CRC Check Failed! \n");
    }

    // When the frame is full, send it to the application
    if (server_.n_filled_in_frame_[ue_id]
        == cfg_->mac_data_bytes_num_perframe) {
        server_.n_filled_in_frame_[ue_id] = 0;

        udp_client->send(kRemoteHostname, kBaseRemotePort + ue_id,
            &server_.frame_data_[ue_id][0], cfg_->mac_data_bytes_num_perframe);
        fprintf(log_file_,
            "MAC thread: Sent data for frame %zu, ue %zu, size %zu\n", frame_id,
            ue_id, cfg_->mac_data_bytes_num_perframe);
        for (size_t i = 0; i < cfg_->mac_data_bytes_num_perframe; i++) {
            ss << std::to_string(server_.frame_data_[ue_id][i]) << " ";
        }
        fprintf(log_file_, "%s\n", ss.str().c_str());
        ss.str("");
    }

    rt_assert(
        tx_queue_->enqueue(Event_data(EventType::kPacketToMac, event.tags[0])),
        "Socket message enqueue failed\n");
}

void MacThread::send_control_information()
{
    // send RAN control information UE
    RBIndicator ri;
    ri.ue_id = next_radio_id_;
    ri.mod_order_bits = CommsLib::QAM16;
    udp_client->send(cfg_->ue_server_addr, kBaseClientPort + ri.ue_id,
        (uint8_t*)&ri, sizeof(RBIndicator));

    // update RAN config within Agora
    send_ran_config_update(Event_data(EventType::kRANUpdate));
}

void MacThread::process_control_information()
{
    memset(&udp_control_buf_[0], 0, udp_control_buf_.size());
    ssize_t ret = udp_control_channel->recv_nonblocking(
        &udp_control_buf_[0], udp_control_buf_.size());
    if (ret == 0) {
        return; // No data received
    } else if (ret == -1) {
        // There was an error in receiving
        cfg_->running = false;
        return;
    }

    rt_assert(static_cast<size_t>(ret) == sizeof(RBIndicator));

    const auto* ri = reinterpret_cast<RBIndicator*>(&udp_control_buf_[0]);
    process_udp_packets_from_apps(*ri);
}

void MacThread::process_udp_packets_from_apps(RBIndicator ri)
{
    memset(&udp_pkt_buf_[0], 0, udp_pkt_buf_.size());
    ssize_t ret
        = udp_server->recv_nonblocking(&udp_pkt_buf_[0], udp_pkt_buf_.size());
    if (ret == 0) {
        return; // No data received
    } else if (ret == -1) {
        // There was an error in receiving
        cfg_->running = false;
        return;
    }
    rt_assert(static_cast<size_t>(ret) == cfg_->mac_data_bytes_num_perframe);

    const auto* pkt = reinterpret_cast<MacPacket*>(&udp_pkt_buf_[0]);
    mode_ == Mode::kServer
        ? process_udp_packets_from_apps_server(pkt, ri)
        : process_udp_packets_from_apps_client((char*)pkt, ri);
}

void MacThread::process_udp_packets_from_apps_server(
    const MacPacket* pkt, RBIndicator ri)
{
    // We've received bits for the downlink
    const size_t rx_offset = pkt->frame_id * cfg_->UE_ANT_NUM + pkt->ue_id;

    if ((*dl_bits_buffer_status_)[rx_offset][pkt->cb_id] == 1) {
        MLPD_ERROR("MAC thread: dl_bits_buffer full, offset %zu. Exiting.\n",
            rx_offset);
        cfg_->running = false;
        return;
    }

    (*dl_bits_buffer_status_)[rx_offset][pkt->cb_id] = 1;
    memcpy(&(*dl_bits_buffer_)[rx_offset][pkt->cb_id
               * roundup<64>(cfg_->num_bytes_per_cb)],
        pkt->data, udp_pkt_buf_.size());

    Event_data msg(EventType::kPacketFromMac,
        gen_tag_t::frm_sym_cb(pkt->frame_id, pkt->ue_id, pkt->cb_id)._tag);
    rt_assert(tx_queue_->enqueue(msg),
        "MAC thread: Failed to enqueue downlink packet");
}

void MacThread::process_udp_packets_from_apps_client(
    const char* payload, RBIndicator ri)
{
    // We've received bits for the uplink. The received MAC packet does not
    // specify a radio ID, send to radios in round-robin order
    size_t& radio_buf_id = client_.ul_bits_buffer_id_[next_radio_id_];

    if ((*client_.ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] == 1) {
        fprintf(stderr,
            "MAC thread: UDP RX buffer full, buffer ID: %zu. Dropping "
            "packet.\n",
            radio_buf_id);
        return;
    }

    if (kLogMacPackets) {
        std::stringstream ss;
        fprintf(log_file_,
            "MAC thread: Received data from app for frame %zu, ue %zu, size "
            "%zu:\n",
            next_frame_id_, next_radio_id_, cfg_->mac_data_bytes_num_perframe);

        for (size_t i = 0; i < cfg_->mac_data_bytes_num_perframe; i++) {
            ss << std::to_string((uint8_t)(payload[i])) << " ";
        }
        fprintf(log_file_, "%s\n", ss.str().c_str());
    }

    for (size_t pkt_id = 0; pkt_id < cfg_->mac_packets_perframe; pkt_id++) {
        size_t data_offset = radio_buf_id * cfg_->mac_bytes_num_perframe
            + pkt_id * cfg_->mac_packet_length;
        auto* pkt = (MacPacket*)(&(
            *client_.ul_bits_buffer_)[next_radio_id_][data_offset]);
        pkt->frame_id = next_frame_id_;
        pkt->ue_id = next_radio_id_;
        pkt->cb_id = pkt_id; // One code block per packet
        pkt->datalen = cfg_->mac_payload_length;
        pkt->rsvd[0] = static_cast<uint16_t>(fast_rand_.next_u32() >> 16);
        pkt->rsvd[1] = static_cast<uint16_t>(fast_rand_.next_u32() >> 16);
        pkt->rsvd[2] = static_cast<uint16_t>(fast_rand_.next_u32() >> 16);
        pkt->crc = 0;
        pkt->rb_indicator = ri;

        memcpy(pkt->data, payload + pkt_id * cfg_->mac_payload_length,
            cfg_->mac_payload_length);
        // Insert CRC
        pkt->crc
            = (uint16_t)(crc_obj->calculate_crc24((unsigned char*)pkt->data,
                             cfg_->mac_payload_length)
                & 0xFFFF);
    }

    (*client_.ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] = 1;
    Event_data msg(
        EventType::kPacketFromMac, rx_tag_t(next_radio_id_, radio_buf_id)._tag);
    rt_assert(
        tx_queue_->enqueue(msg), "MAC thread: Failed to enqueue uplink packet");

    radio_buf_id = (radio_buf_id + 1) % kFrameWnd;
    next_radio_id_ = (next_radio_id_ + 1) % cfg_->UE_ANT_NUM;
    if (next_radio_id_ == 0)
        next_frame_id_++;
}

void MacThread::run_event_loop()
{
    MLPD_INFO("Running MAC thread event loop, logging to file %s\n",
        log_filename_.c_str());
    pin_to_core_with_offset(
        ThreadType::kWorkerMacTXRX, core_offset_, 0 /* thread ID */);

    while (cfg_->running) {
        process_rx_from_master();

        if (mode_ == Mode::kServer) {
            if (rdtsc() - last_frame_tx_tsc_ > tsc_delta_) {
                send_control_information();
                last_frame_tx_tsc_ = rdtsc();
            }
        } else {
            process_control_information();
        }

        if (next_frame_id_ == cfg_->frames_to_test) {
            MLPD_WARN("MAC thread stopping. Next frame ID = %zu, configured "
                      "frames to test = %zu\n",
                next_frame_id_, cfg_->frames_to_test);
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

    // printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(event.event_type == EventType::kPacketTX);

    size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    size_t data_symbol_idx = gen_tag_t(event.tags[0]).symbol_id;

    size_t offset = (c->get_total_data_symbol_idx(frame_id, data_symbol_idx)
                        * c->BS_ANT_NUM)
        + ant_id;

    if (kDebugPrintInTask) {
        printf("In TX thread %d: Transmitted frame %zu, symbol %zu, "
               "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, data_symbol_idx, ant_id,
            gen_tag_t(event.tags[0])._tag, offset,
            message_queue_->size_approx());
    }

    size_t socket_symbol_offset = offset
        % (kFrameWnd * c->data_symbol_num_perframe
              * c->BS_ANT_NUM);
    char* cur_buffer_ptr = tx_buffer_ + socket_symbol_offset * c->packet_length;
    auto* pkt = (Packet*)cur_buffer_ptr;
    new (pkt) Packet(frame_id, data_symbol_idx, 0, ant_id);

    // Send data (one OFDM symbol)
    ssize_t ret = sendto(socket_[ant_id % config_->socket_thread_num],
        cur_buffer_ptr, c->packet_length, 0, (struct sockaddr*)&servaddr_[tid],
        sizeof(servaddr_[tid]));
    rt_assert(ret > 0, "sendto() failed");

    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return event.tags[0];
}
*/
