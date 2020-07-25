#include "mac_thread.hpp"
#include "logger.h"
#include "utils_ldpc.hpp"

MacThread::MacThread(Mode mode, Config* cfg, size_t core_offset,
    Table<uint8_t>* ul_bits_buffer, Table<uint8_t>* ul_bits_buffer_status,
    Table<int8_t>* dl_bits_buffer, Table<int>* dl_bits_buffer_status,
    moodycamel::ConcurrentQueue<Event_data>* rx_queue,
    moodycamel::ConcurrentQueue<Event_data>* tx_queue, std::string log_filename)
    : mode_(mode)
    , cfg_(cfg)
    , core_offset_(core_offset)
    , ul_bits_buffer_(ul_bits_buffer)
    , ul_bits_buffer_status_(ul_bits_buffer_status)
    , dl_bits_buffer_(dl_bits_buffer)
    , dl_bits_buffer_status_(dl_bits_buffer_status)
    , rx_queue_(rx_queue)
{
    // Set up MAC log file
    if (log_filename != "") {
        log_filename_ = log_filename; // Use a non-default log filename
    }
    log_file_ = fopen(log_filename_.c_str(), "w");
    rt_assert(log_file_ != nullptr, "Failed to open MAC log file");

    // Set up buffers
    num_filled_bytes_in_frame_.fill(0);
    for (auto& v : frame_data_)
        v.resize(cfg_->mac_data_bytes_num_perframe);

    const size_t udp_pkt_len = MacPacket::kOffsetOfData
        + bits_to_bytes(cfg_->LDPC_config.cbLen)
            * cfg_->LDPC_config.nblocksInSymbol;
    udp_pkt_buf_.resize(udp_pkt_len);

    udp_server = UDPServer(kLocalPort, udp_pkt_len * kMaxUEs * kMaxPktsPerUE);
}

MacThread::~MacThread()
{
    fclose(log_file_);
    MLPD_INFO("MAC thread destroyed\n");
}

void MacThread::process_codeblocks_from_master()
{
    Event_data event;
    if (!rx_queue_->try_dequeue(event))
        return;
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
        const size_t frame_data__offset = (symbol_idx_ul - cfg_->UL_PILOT_SYMS)
            * cfg_->data_bytes_num_persymbol;
        memcpy(&frame_data_[ue_id][frame_data__offset], ul_data_ptr,
            cfg_->data_bytes_num_persymbol);
        num_filled_bytes_in_frame_[ue_id] += cfg_->data_bytes_num_persymbol;

        // Print information about the received symbol
        if (kDebugBSReceiver) {
            fprintf(log_file_,
                "MAC thread received frame %zu, uplink symbol index %zu, "
                "size %zu, copied to frame data offset %zu\n",
                frame_id, symbol_idx_ul, cfg_->data_bytes_num_perframe,
                frame_data__offset);

            for (size_t i = 0; i < cfg_->data_bytes_num_persymbol; i++) {
                ss << std::to_string(ul_data_ptr[i]) << " ";
            }
            fprintf(log_file_, "%s\n", ss.str().c_str());
            ss.str("");
        }
    }

    // When the frame is full, send it to the application
    if (num_filled_bytes_in_frame_[ue_id] == cfg_->data_bytes_num_perframe) {
        num_filled_bytes_in_frame_[ue_id] = 0;

        udp_client.send(kRemoteHostname, kBaseRemotePort + ue_id,
            &frame_data_[ue_id][0], cfg_->mac_data_bytes_num_perframe);

        fprintf(log_file_,
            "MAC thread: Sent data for frame %zu, ue %zu, size %zu\n", frame_id,
            ue_id, cfg_->mac_data_bytes_num_perframe);
        for (size_t i = 0; i < cfg_->mac_data_bytes_num_perframe; i++) {
            ss << std::to_string(frame_data_[ue_id][i]) << " ";
        }
        fprintf(log_file_, "%s\n", ss.str().c_str());
        ss.str("");
    }

    rt_assert(
        tx_queue_->enqueue(Event_data(EventType::kPacketToMac, event.tags[0])),
        "Socket message enqueue failed\n");
}

void MacThread::process_udp_packets_from_apps()
{
    ssize_t ret
        = udp_server.recv_nonblocking(&udp_pkt_buf_[0], udp_pkt_buf_.size());
    if (ret == 0) {
        return; // No data received
    } else if (ret == -1) {
        // There was an error in receiving
        cfg_->running = false;
        return;
    }

    const auto* pkt = reinterpret_cast<MacPacket*>(&udp_pkt_buf_[0]);
    const size_t total_symbol_idx
        = cfg_->get_total_data_symbol_idx_dl(pkt->frame_id, pkt->symbol_id);
    const size_t rx_offset
        = total_symbol_idx * cfg_->LDPC_config.nblocksInSymbol;

    if ((*dl_bits_buffer_status_)[pkt->ue_id][rx_offset] == 1) {
        MLPD_ERROR("MAC thread: dl_bits_buffer full, offset %zu\n", rx_offset);
        cfg_->running = false;
        return;
    } else {
        for (size_t i = 0; i < cfg_->LDPC_config.nblocksInSymbol; i++)
            (*dl_bits_buffer_status_)[pkt->ue_id][rx_offset + i] = 1;
        memcpy(&(*dl_bits_buffer_)[total_symbol_idx]
                                  [pkt->ue_id * cfg_->OFDM_DATA_NUM],
            pkt->data, udp_pkt_buf_.size());
    }

    Event_data msg(EventType::kPacketFromMac,
        gen_tag_t::frm_sym_ue(pkt->frame_id, pkt->symbol_id, pkt->ue_id)._tag);
    rt_assert(tx_queue_->enqueue(msg),
        "MAC thread: Failed to enqueue downlink packet");
}

void MacThread::run_event_loop()
{
    MLPD_INFO("Running MAC thread event loop, logging to file %s\n",
        log_filename_.c_str());
    pin_to_core_with_offset(
        ThreadType::kWorkerMacTXRX, core_offset_, 0 /* thread ID */);

    while (cfg_->running) {
        process_codeblocks_from_master();
        process_udp_packets_from_apps();
    }
}
