#include "mac_thread.hpp"
#include "utils_ldpc.hpp"

MacThread::MacThread(Config* cfg, size_t core_offset,
    Table<int8_t>* dl_bits_buffer, Table<int>* dl_bits_buffer_status,
    Table<uint8_t>* ul_bits_buffer,
    moodycamel::ConcurrentQueue<Event_data>* rx_queue,
    moodycamel::ConcurrentQueue<Event_data>* tx_queue,
    moodycamel::ProducerToken* rx_ptok, moodycamel::ProducerToken* tx_ptok)
    : cfg_(cfg)
    , core_offset_(core_offset)
    , dl_bits_buffer_(dl_bits_buffer)
    , dl_bits_buffer_status_(dl_bits_buffer_status)
    , ul_bits_buffer_(ul_bits_buffer)
    , rx_queue_(rx_queue)
    , rx_ptok_(rx_ptok)
    , tx_ptok_(tx_ptok)
{
}

MacThread::~MacThread() {}

void MacThread::run_event_loop()
{
    pin_to_core_with_offset(
        ThreadType::kWorkerMacTXRX, core_offset_, 0 /* thread ID */);

    std::vector<uint8_t> mac_pkt_buffer(
        Packet::kOffsetOfData + cfg_->OFDM_DATA_NUM);
    std::vector<uint8_t> frame_data(cfg_->mac_data_bytes_num_perframe);

    // Set up sockets to push MAC data to applications
    int app_sockets[cfg_->UE_NUM];
    sockaddr_in app_sockaddr[cfg_->UE_NUM];
    for (size_t i = 0; i < cfg_->UE_NUM; i++) {
        size_t local_port = 8090 + i;
        size_t remote_port = 8080 + i;
        std::string remote_addr = "127.0.0.1";
        size_t sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
        app_sockets[i] = setup_socket_ipv4(local_port, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(
            &app_sockaddr[i], remote_port, remote_addr.c_str());
        printf("MAC thread: Sending data to app at %s:%zu from port %zu\n",
            remote_addr.c_str(), remote_port, local_port);
    }

    const size_t cbLenBytes = (cfg_->LDPC_config.cbLen + 7) >> 3;
    const size_t ul_data_syms
        = cfg_->ul_data_symbol_num_perframe - cfg_->UL_PILOT_SYMS;
    std::stringstream ss;
    std::stringstream ss1[ul_data_syms];

    size_t num_filled_bytes_in_frame = 0;
    while (cfg_->running) {
        Event_data event;
        if (!rx_queue_->try_dequeue_from_producer(*rx_ptok_, event))
            continue;
        assert(event.event_type == EventType::kPacketToMac);

        const size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
        const size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
        const size_t ue_id = gen_tag_t(event.tags[0]).ue_id;
        const size_t data_offset = kUseLDPC
            ? (cbLenBytes * cfg_->LDPC_config.nblocksInSymbol * ue_id)
            : (cfg_->OFDM_DATA_NUM * ue_id);
        const size_t total_symbol_idx
            = cfg_->get_total_data_symbol_idx_ul(frame_id, symbol_id);

        // Copy over the packet
        const uint8_t* ul_data_ptr
            = &(*ul_bits_buffer_)[total_symbol_idx][data_offset];
        auto* pkt = reinterpret_cast<MacPacket*>(&mac_pkt_buffer[0]);
        pkt->frame_id = frame_id;
        pkt->symbol_id = symbol_id;
        pkt->cell_id = 0;
        pkt->ue_id = ue_id;
        if (!kUseLDPC) {
            adapt_bits_from_mod((int8_t*)ul_data_ptr, (int8_t*)pkt->data,
                cfg_->OFDM_DATA_NUM, cfg_->mod_type);
        } else {
            memcpy(pkt->data, ul_data_ptr, cfg_->data_bytes_num_persymbol);
        }

        // Print information about the received packet
        if (kDebugBSReceiver) {
            if (pkt->symbol_id == cfg_->UL_PILOT_SYMS) {
                ss << "MAC thread received frame " << pkt->frame_id
                   << " symbol " << pkt->symbol_id << ", ue " << pkt->ue_id
                   << ", size " << cfg_->data_bytes_num_perframe << std::endl;
            }
            if (pkt->symbol_id >= cfg_->UL_PILOT_SYMS) {
                for (size_t i = 0; i < cfg_->data_bytes_num_persymbol; i++) {
                    ss1[pkt->symbol_id - cfg_->UL_PILOT_SYMS]
                        << std::to_string(pkt->data[i]) << " ";
                }
                if (pkt->symbol_id == cfg_->ul_data_symbol_num_perframe - 1) {
                    for (size_t j = 0; j < ul_data_syms; j++) {
                        ss << ss1[j].str();
                        ss1[j].str(std::string());
                    }
                    ss << std::endl;
                    std::cout << ss.str();
                    ss.str(std::string());
                }
            }
        }

        // Copy packet to the frame data buffer. It might take multiple symbols
        // to fill up the frame. If the frame is full, send the frame to the
        // application.
        assert(pkt->symbol_id == 2 || pkt->symbol_id == 3); // TODO: Why?
        memcpy(&frame_data[0]
                + (pkt->symbol_id == 2 ? 0 : cfg_->data_bytes_num_persymbol),
            (char*)pkt->data, cfg_->data_bytes_num_persymbol);
        num_filled_bytes_in_frame += cfg_->data_bytes_num_persymbol;

        if (num_filled_bytes_in_frame == cfg_->data_bytes_num_perframe) {
            num_filled_bytes_in_frame = 0;

            int ret = sendto(app_sockets[pkt->ue_id], &frame_data[0],
                cfg_->mac_data_bytes_num_perframe, 0,
                (struct sockaddr*)&app_sockaddr[pkt->ue_id],
                sizeof(app_sockaddr[pkt->ue_id]));
            if (ret == -1) {
                fprintf(stderr, "MAC thread: sendto() failed with error %s\n",
                    strerror(errno));
                exit(-1);
            }

            printf("Sent data for frame %u, ue %u, size %zu\n", pkt->frame_id,
                pkt->ue_id, cfg_->mac_data_bytes_num_perframe);
            for (size_t i = 0; i < cfg_->mac_data_bytes_num_perframe; i++) {
                printf("%u ", frame_data[i]);
            }
            printf("\n");
        }

        rt_assert(tx_queue_->enqueue(*tx_ptok_,
                      Event_data(EventType::kPacketToMac, event.tags[0])),
            "Socket message enqueue failed\n");
    }
}
