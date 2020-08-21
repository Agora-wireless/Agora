#include "mac_thread.hpp"
#include "logger.h"
#include "utils_ldpc.hpp"

MacThread::MacThread(Mode mode, Config* cfg, size_t core_offset,
    Table<uint8_t>* ul_bits_buffer, Table<uint8_t>* ul_bits_buffer_status,
    Table<uint8_t>* dl_bits_buffer, Table<uint8_t>* dl_bits_buffer_status,
    moodycamel::ConcurrentQueue<Event_data>* rx_queue,
    moodycamel::ConcurrentQueue<Event_data>* tx_queue, std::string log_filename)
    : mode_(mode)
    , cfg_(cfg)
    , freq_ghz_(measure_rdtsc_freq())
    , tsc_delta_((cfg_->get_frame_duration_sec() * 1e9) / freq_ghz_)
    , core_offset_(core_offset)
    , ul_bits_buffer_(ul_bits_buffer)
    , ul_bits_buffer_status_(ul_bits_buffer_status)
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
    server_.n_filled_in_frame_.fill(0);
    for (auto& v : server_.frame_data_)
        v.resize(cfg_->mac_data_bytes_num_perframe);

    client_.ul_bits_buffer_id_.fill(0);

    const size_t udp_pkt_len = cfg_->mac_data_bytes_num_perframe;
    udp_pkt_buf_.resize(udp_pkt_len);

    const size_t udp_control_len = sizeof(ControlPacket);
    udp_control_buf_.resize(sizeof(udp_control_len));

    udp_server
        = new UDPServer(kLocalPort, udp_pkt_len * kMaxUEs * kMaxPktsPerUE);
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
    assert(event.event_type == EventType::kSNRReport);

    const size_t ue_id = gen_tag_t(event.tags[0]).ue_id;
    if (server_.snr_[ue_id].size() == kSNRWindowSize) {
        server_.snr_[ue_id].pop();
    }

    server_.snr_[ue_id].push(*reinterpret_cast<float*>(&event.tags[1]));
}

void MacThread::send_ran_config_update(Event_data event, RanConfig rc)
{
    assert(event.event_type == EventType::kRANUpdate);
}

void MacThread::process_codeblocks_from_master(Event_data event)
{
    assert(event.event_type == EventType::kPacketToMac);

    const size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    const size_t symbol_idx_ul = gen_tag_t(event.tags[0]).symbol_id;
    const size_t ue_id = gen_tag_t(event.tags[0]).ue_id;

    const size_t data_offset = (bits_to_bytes(cfg_->LDPC_config.cbLen)
        * cfg_->LDPC_config.nblocksInSymbol * ue_id);
    const uint8_t* ul_data_ptr
        = &(*ul_bits_buffer_)[cfg_->get_total_data_symbol_idx_ul(
            frame_id, symbol_idx_ul)][data_offset];

    std::stringstream ss; // Debug-only

    // Only non-pilot uplink symbols have application data.
    if (symbol_idx_ul >= cfg_->UL_PILOT_SYMS) {
        auto* pkt = (struct MacPacket*)ul_data_ptr;

        // we send data to app irrespective of CRC condition
        // TODO: enable ARQ and ensure reliable data goes to app
        const size_t frame_data__offset
            = (symbol_idx_ul - cfg_->UL_PILOT_SYMS) * cfg_->mac_payload_length;
        memcpy(&server_.frame_data_[ue_id][frame_data__offset], pkt->data,
            cfg_->mac_payload_length);
        server_.n_filled_in_frame_[ue_id] += cfg_->mac_payload_length;

        // Check CRC
        uint16_t crc
            = (uint16_t)(crc_obj->calculateCRC24((unsigned char*)pkt->data,
                             cfg_->mac_payload_length)
                & 0xFFFF);
        if (crc == pkt->crc) {

            // Print information about the received symbol
            if (kDebugBSReceiver) {
                fprintf(log_file_,
                    "MAC thread received frame %zu, uplink symbol index %zu, "
                    "size %zu, copied to frame data offset %zu\n",
                    frame_id, symbol_idx_ul, cfg_->mac_payload_length,
                    frame_data__offset);

                ss << "Header Info:\n"
                   << "FRAME_ID: " << pkt->frame_id
                   << "\nSYMBOL_ID: " << pkt->symbol_id
                   << "\nUE_ID: " << pkt->ue_id << "\nDATLEN: " << pkt->datalen
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

void MacThread::handle_control_information()
{
    mode_ == Mode::kServer ? send_control_information()
                           : process_control_information();
}

void MacThread::send_control_information()
{
    ControlPacket ci;
    ci.ue_id = next_radio_id_;
    ci.mod_type = CommsLib::QAM64;
    udp_client->send(kClientHostname, kBaseClientPort + ci.ue_id, (uint8_t*)&ci,
        sizeof(ControlPacket));
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

    rt_assert(static_cast<size_t>(ret) == sizeof(ControlPacket));

    const auto* ci = reinterpret_cast<ControlPacket*>(&udp_control_buf_[0]);
    Event_data msg(EventType::kControlPacket);
    msg.num_tags = 2;
    msg.tags[0] = ci->ue_id;
    msg.tags[1] = ci->mod_type;
    printf("MAC thread: received control packet for ue %zu, mod %zu", ci->ue_id,
        ci->mod_type);
    rt_assert(tx_queue_->enqueue(msg),
        "MAC thread: fail to send control packet to PHY");
}

void MacThread::process_udp_packets_from_apps()
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
    mode_ == Mode::kServer ? process_udp_packets_from_apps_server(pkt)
                           : process_udp_packets_from_apps_client((char*)pkt);
}

void MacThread::process_udp_packets_from_apps_server(const MacPacket* pkt)
{
    // We've received bits for the downlink
    const size_t total_symbol_idx
        = cfg_->get_total_data_symbol_idx_dl(pkt->frame_id, pkt->symbol_id);
    const size_t rx_offset
        = total_symbol_idx * cfg_->LDPC_config.nblocksInSymbol;

    if ((*dl_bits_buffer_status_)[pkt->ue_id][rx_offset] == 1) {
        MLPD_ERROR("MAC thread: dl_bits_buffer full, offset %zu. Exiting.\n",
            rx_offset);
        cfg_->running = false;
        return;
    }

    for (size_t i = 0; i < cfg_->LDPC_config.nblocksInSymbol; i++)
        (*dl_bits_buffer_status_)[pkt->ue_id][rx_offset + i] = 1;
    memcpy(
        &(*dl_bits_buffer_)[total_symbol_idx][pkt->ue_id * cfg_->OFDM_DATA_NUM],
        pkt->data, udp_pkt_buf_.size());

    Event_data msg(EventType::kPacketFromMac,
        gen_tag_t::frm_sym_ue(pkt->frame_id, pkt->symbol_id, pkt->ue_id)._tag);
    rt_assert(tx_queue_->enqueue(msg),
        "MAC thread: Failed to enqueue downlink packet");
}

void MacThread::process_udp_packets_from_apps_client(const char* payload)
{
    // We've received bits for the uplink. The received MAC packet does not
    // specify a radio ID, send to radios in round-robin order
    size_t& radio_buf_id = client_.ul_bits_buffer_id_[next_radio_id_];

    if ((*ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] == 1) {
        fprintf(stderr,
            "MAC thread: UDP RX buffer full, buffer ID: %zu. Dropping "
            "packet.\n",
            radio_buf_id);
        return;
    }

    if (kDebugBSReceiver) {
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
        MacPacket* pkt
            = (MacPacket*)(&(*ul_bits_buffer_)[next_radio_id_][data_offset]);
        pkt->frame_id = next_frame_id_;
        pkt->symbol_id = pkt_id;
        pkt->ue_id = next_radio_id_;
        pkt->datalen = cfg_->mac_payload_length;
        pkt->rsvd[0] = static_cast<uint16_t>(fast_rand_.next_u32() >> 16);
        pkt->rsvd[1] = static_cast<uint16_t>(fast_rand_.next_u32() >> 16);
        pkt->rsvd[2] = static_cast<uint16_t>(fast_rand_.next_u32() >> 16);
        pkt->crc = 0;

        memcpy(pkt->data, payload + pkt_id * cfg_->mac_payload_length,
            cfg_->mac_payload_length);
        // Insert CRC
        pkt->crc = (uint16_t)(crc_obj->calculateCRC24((unsigned char*)pkt->data,
                                  cfg_->mac_payload_length)
            & 0xFFFF);
    }

    (*ul_bits_buffer_status_)[next_radio_id_][radio_buf_id] = 1;
    Event_data msg(
        EventType::kPacketFromMac, rx_tag_t(next_radio_id_, radio_buf_id)._tag);
    rt_assert(
        tx_queue_->enqueue(msg), "MAC thread: Failed to enqueue uplink packet");

    radio_buf_id = (radio_buf_id + 1) % TASK_BUFFER_FRAME_NUM;
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
        if (rdtsc() - last_mac_pkt_rx_tsc_ > tsc_delta_) {
            process_udp_packets_from_apps();
            handle_control_information();
            last_mac_pkt_rx_tsc_ = rdtsc();
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
        % (SOCKET_BUFFER_FRAME_NUM * c->data_symbol_num_perframe
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
