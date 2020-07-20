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
    , ul_bits_buffer_(ul_bits_buffer)
    , dl_bits_buffer_(dl_bits_buffer)
    , dl_bits_buffer_status_(dl_bits_buffer_status)
    , rx_queue_(rx_queue)
    , rx_ptok_(rx_ptok)
    , tx_ptok_(tx_ptok)
{
    // Set up sockets to push MAC data to applications
    app_sockets_.resize(cfg_->UE_NUM);
    app_sockaddr_.resize(cfg_->UE_NUM);

    // Set up sockets to push MAC data to applications
    int app_sockets_[cfg_->UE_NUM];
    sockaddr_in app_sockaddr_[cfg_->UE_NUM];
    for (size_t i = 0; i < cfg_->UE_NUM; i++) {
        size_t local_port = 8090 + i;
        size_t remote_port = 8080 + i;
        std::string remote_addr = "127.0.0.1";
        size_t sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
        app_sockets_[i] = setup_socket_ipv4(local_port, true, sock_buf_size);
        setup_sockaddr_remote_ipv4(
            &app_sockaddr_[i], remote_port, remote_addr.c_str());
        printf("MAC thread: Sending data to app at %s:%zu from port %zu\n",
            remote_addr.c_str(), remote_port, local_port);
    }
}

MacThread::~MacThread() {}

void MacThread::run_event_loop()
{
    pin_to_core_with_offset(
        ThreadType::kWorkerMacTXRX, core_offset_, 0 /* thread ID */);

    std::vector<uint8_t> staging_buf(cfg_->OFDM_DATA_NUM);
    std::vector<uint8_t> frame_data(cfg_->mac_data_bytes_num_perframe);

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
        const size_t symbol_idx_ul = gen_tag_t(event.tags[0]).symbol_id;
        const size_t ue_id = gen_tag_t(event.tags[0]).ue_id;

        const size_t data_offset = kUseLDPC
            ? (cbLenBytes * cfg_->LDPC_config.nblocksInSymbol * ue_id)
            : (cfg_->OFDM_DATA_NUM * ue_id);
        const uint8_t* ul_data_ptr
            = &(*ul_bits_buffer_)[cfg_->get_total_data_symbol_idx_ul(
                frame_id, symbol_idx_ul)][data_offset];

        if (!kUseLDPC) {
            adapt_bits_from_mod((int8_t*)ul_data_ptr, (int8_t*)&staging_buf[0],
                cfg_->OFDM_DATA_NUM, cfg_->mod_type);
        } else {
            memcpy(
                &staging_buf[0], ul_data_ptr, cfg_->data_bytes_num_persymbol);
        }

        // Print information about the received packet
        if (kDebugBSReceiver) {
            if (symbol_idx_ul == cfg_->UL_PILOT_SYMS) {
                ss << "MAC thread received frame " << frame_id
                   << ", uplink symbol index " << symbol_idx_ul << ", ue "
                   << ue_id << ", size " << cfg_->data_bytes_num_perframe
                   << std::endl;
            }
            if (symbol_idx_ul >= cfg_->UL_PILOT_SYMS) {
                for (size_t i = 0; i < cfg_->data_bytes_num_persymbol; i++) {
                    ss1[symbol_idx_ul - cfg_->UL_PILOT_SYMS]
                        << std::to_string(staging_buf[i]) << " ";
                }
                if (symbol_idx_ul == cfg_->ul_data_symbol_num_perframe - 1) {
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

        // Only non-pilot uplink symbols have application data.
        if (symbol_idx_ul >= cfg_->UL_PILOT_SYMS) {
            const size_t frame_data_offset
                = (symbol_idx_ul - cfg_->UL_PILOT_SYMS)
                * cfg_->data_bytes_num_persymbol;
            memcpy(&frame_data[frame_data_offset], &staging_buf,
                cfg_->data_bytes_num_persymbol);
            num_filled_bytes_in_frame += cfg_->data_bytes_num_persymbol;
        }

        // When the frame is full, send it to the application.
        if (num_filled_bytes_in_frame == cfg_->data_bytes_num_perframe) {
            num_filled_bytes_in_frame = 0;

            int ret = sendto(app_sockets_[ue_id], &frame_data[0],
                cfg_->mac_data_bytes_num_perframe, 0,
                (struct sockaddr*)&app_sockaddr_[ue_id], sizeof(sockaddr_in));
            if (ret == -1) {
                fprintf(stderr, "MAC thread: sendto() failed with error %s\n",
                    strerror(errno));
                exit(-1);
            }

            printf("MAC thread: Sent data for frame %zu, ue %zu, size %zu\n",
                frame_id, ue_id, cfg_->mac_data_bytes_num_perframe);
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
