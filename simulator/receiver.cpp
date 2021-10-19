#include "receiver.hpp"
#include "datatype_conversion.h"

Receiver::Receiver(Config* cfg, size_t rx_thread_num, size_t core_offset, void* mbuf_pool)
    : rx_thread_num_(rx_thread_num)
    , core_id_(core_offset)
    , cfg(cfg)
{
#ifdef USE_DPDK
    uint16_t portid = 0; // For now, hard-code to port zero
    this->mbuf_pool = reinterpret_cast<rte_mempool*>(mbuf_pool);
    
    // Parse IP addresses and MAC addresses
    int ret = inet_pton(AF_INET, cfg->bs_rru_addr.c_str(), &bs_rru_addr);
    rt_assert(ret == 1, "Invalid sender IP address");
    ret = inet_pton(AF_INET, cfg->bs_server_addr.c_str(), &bs_server_addr);
    rt_assert(ret == 1, "Invalid server IP address");

    bs_server_addr_list.resize(cfg->bs_server_addr_list.size());
    for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
        ret = inet_pton(AF_INET, cfg->bs_server_addr_list[i].c_str(), &bs_server_addr_list[i]);
    }

    server_mac_addr_list.resize(cfg->bs_server_addr_list.size());
    for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
        ether_addr* parsed_mac = ether_aton(cfg->bs_server_mac_list[i].c_str());
        rt_assert(parsed_mac != NULL, "Invalid server mac address");
        memcpy(&server_mac_addr_list[i], parsed_mac, sizeof(ether_addr));
    }

    ret = rte_eth_macaddr_get(portid, &receiver_mac_addr);
    rt_assert(ret == 0, "Cannot get MAC address of the port");
    printf("Number of DPDK cores: %d\n", rte_lcore_count());

    for (size_t i = 0; i < rx_thread_num_; i++) {
        uint16_t src_port = rte_cpu_to_be_16(cfg->bs_server_port + i);
        uint16_t dst_port = rte_cpu_to_be_16(cfg->bs_rru_port + i);

        for (size_t j = 0; j < cfg->bs_server_addr_list.size(); j ++) {
            uint32_t src_ip = bs_server_addr_list[j];
            uint32_t dst_ip = bs_rru_addr;
            printf("Adding steering rule for src IP %s, dest IP %s, src port: %zu, "
                "dst port: %zu, queue: %zu\n",
                cfg->bs_server_addr_list[j].c_str(), cfg->bs_rru_addr.c_str(),
                cfg->bs_server_port + i, cfg->bs_rru_port + i, i);
            DpdkTransport::install_flow_rule(
                portid, i, src_ip, dst_ip, src_port, dst_port);
        }
    }
#endif
    socket_buffer_.calloc(cfg->BS_ANT_NUM, cfg->packet_length * kFrameWnd * cfg->symbol_num_perframe, 64);
    socket_buffer_status_.calloc(cfg->BS_ANT_NUM, kFrameWnd * cfg->symbol_num_perframe, 64);

    dl_ue_data_buffer_.calloc(cfg->BS_ANT_NUM, cfg->OFDM_CA_NUM * sizeof(short) * 2 * kFrameWnd * cfg->symbol_num_perframe, 64);

    completion_num_ = 0;

    for (size_t i = 0; i < kFrameWnd; i ++) {
        frame_status_[i] = 0;
    }
}

Receiver::Receiver(Config* cfg, size_t rx_thread_num, size_t core_offset,
    moodycamel::ConcurrentQueue<EventData>* in_queue_message,
    moodycamel::ProducerToken** in_rx_ptoks)
    : Receiver(cfg, rx_thread_num, core_offset)
{
    message_queue_ = in_queue_message;
    rx_ptoks_ = in_rx_ptoks;
}

Receiver::~Receiver() { delete cfg; }

// std::vector<pthread_t> Receiver::startRecv(Table<char>& in_buffer,
//     Table<int>& in_buffer_status, size_t in_buffer_frame_num,
//     size_t in_buffer_length, Table<double>& in_frame_start)
void Receiver::startRecv(Table<char>& in_buffer,
    Table<int>& in_buffer_status, size_t in_buffer_frame_num,
    size_t in_buffer_length, Table<double>& in_frame_start)
{
    buffer_frame_num_ = in_buffer_frame_num;
    buffer_length_ = in_buffer_length;
    buffer_ = &in_buffer;
    buffer_status_ = &in_buffer_status;
    frame_start_ = &in_frame_start;

    printf("start Recv thread\n");
    // std::vector<pthread_t> created_threads;

    // for (size_t i = 0; i < rx_thread_num_; i++) {
    //     pthread_t recv_thread_;
    //     auto context = new EventHandlerContext<Receiver>;
    //     context->obj_ptr = this;
    //     context->id = i;
    //     if (pthread_create(&recv_thread_, NULL,
    //             pthread_fun_wrapper<Receiver, &Receiver::loopRecv>, context)
    //         != 0) {
    //         perror("Socket recv thread create failed");
    //         exit(0);
    //     }
    //     created_threads.push_back(recv_thread_);
    // }
    // return created_threads;

    for (size_t i = 0; i < rx_thread_num_; i ++) {
        receiver_threads_[i] = std::thread(&Receiver::loopRecv, this, i);
    }
}

void Receiver::join_thread() {
    for (size_t i = 0; i < rx_thread_num_; i ++) {
        receiver_threads_[i].join();
    }
}

void* Receiver::loopRecv(int tid)
{
    printf("New receiver thread!\n");
    size_t core_offset = core_id_;
    pin_to_core_with_offset(ThreadType::kWorkerRX, core_offset, tid);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    struct sockaddr_in remote_addr;
    int socket_local
        = setup_socket_ipv4(cfg->bs_rru_port + tid, true, sock_buf_size);
    setup_sockaddr_remote_ipv4(
        &remote_addr, cfg->bs_server_port + tid, cfg->bs_server_addr.c_str());

    /* use token to speed up */
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];

    char* buffer_ptr = (*buffer_)[tid];
    int* buffer_status_ptr = (*buffer_status_)[tid];
    long long buffer_length = buffer_length_;
    int buffer_frame_num = buffer_frame_num_;
    double* frame_start = (*frame_start_)[tid];

    DFTI_DESCRIPTOR_HANDLE mkl_handle;
    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->OFDM_CA_NUM);
    (void)DftiCommitDescriptor(mkl_handle);

    auto ifft_inout = reinterpret_cast<complex_float*>(
        memalign(64, cfg->OFDM_CA_NUM * sizeof(complex_float)));

    char* cur_buffer_ptr = buffer_ptr;
    int* cur_buffer_status_ptr = buffer_status_ptr;
    // loop recv
    socklen_t addrlen = sizeof(remote_addr);
    size_t offset = 0;
    int prev_frame_id = -1;
    while (true) {
        /* if buffer is full, exit */
        if (cur_buffer_status_ptr[0] == 1) {
            printf("Receive thread %d buffer full, offset: %zu\n", tid, offset);
            exit(0);
        }

#ifdef USE_DPDK
        rte_mbuf* rx_bufs[kRxBatchSize];
        uint16_t nb_rx = rte_eth_rx_burst(0, tid, rx_bufs, kRxBatchSize);
        if (unlikely(nb_rx == 0))
            continue;

        for (size_t i = 0; i < nb_rx; i++) {
            rte_mbuf* dpdk_pkt = rx_bufs[i];
            auto* eth_hdr = rte_pktmbuf_mtod(dpdk_pkt, rte_ether_hdr*);
            auto* ip_hdr = reinterpret_cast<rte_ipv4_hdr*>(
                reinterpret_cast<uint8_t*>(eth_hdr) + sizeof(rte_ether_hdr));
            uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);

            if (eth_type != RTE_ETHER_TYPE_IPV4
                or ip_hdr->next_proto_id != IPPROTO_UDP) {
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }

            bool found = false;
            for (size_t j = 0; j < bs_server_addr_list.size(); j ++) {
                if (ip_hdr->src_addr == bs_server_addr_list[j]) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                fprintf(stderr, "DPDK: Source addr does not match\n");
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }
            if (ip_hdr->dst_addr != bs_rru_addr) {
                fprintf(stderr, "DPDK: Destination addr does not match\n");
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }

            auto* pkt = reinterpret_cast<Packet*>(reinterpret_cast<char*>(eth_hdr) + kPayloadOffset);
            if (pkt->pkt_type_ == Packet::PktType::kIQFromServer) {
                if (pkt->frame_id_ >= cur_frame_ + kFrameWnd) {
                    printf("Error! Socket buffer overflow!\n");
                    exit(1);
                }
                size_t frame_slot = pkt->frame_id_ % kFrameWnd;
                size_t symbol_id = pkt->symbol_id_;
                size_t ant_id = pkt->ant_id_;
                size_t server_id = pkt->server_id_;
                size_t symbol_offset = frame_slot * cfg->symbol_num_perframe + symbol_id;
                size_t table_offset = symbol_offset * cfg->packet_length 
                    + sizeof(float) * (cfg->OFDM_DATA_START + server_id * cfg->get_num_sc_per_server());
                DpdkTransport::fastMemcpy(&socket_buffer_[ant_id][table_offset], pkt->data_, sizeof(float) * cfg->get_num_sc_per_server());
                socket_buffer_status_[ant_id][symbol_offset] ++;
                if (socket_buffer_status_[ant_id][symbol_offset] == cfg->bs_server_addr_list.size()) {
                    char* src = &socket_buffer_[ant_id][symbol_offset * cfg->packet_length];
                    run_ifft(reinterpret_cast<short*>(src), ifft_inout, mkl_handle);
                    socket_buffer_status_[ant_id][symbol_offset] = 0;
                    // TODO: OUTPUT TO PROCESS
                    size_t buffer_offset = symbol_offset * (sizeof(short) * 2 * cfg->OFDM_CA_NUM);
                    DpdkTransport::fastMemcpy(&dl_ue_data_buffer_[ant_id][buffer_offset], src, sizeof(short) * 2 * cfg->OFDM_CA_NUM);
                }
                frame_mutex_.lock();
                frame_status_[frame_slot] ++;
                if (frame_status_[frame_slot] == cfg->dl_data_symbol_num_perframe * cfg->bs_server_addr_list.size() * cfg->BS_ANT_NUM) {
                    frame_status_[frame_slot] = 0;
                    cur_frame_ ++;
                }
                frame_mutex_.unlock();
            } else {
                printf("Received unknown packet type in receiver thread\n");
                exit(1);
            }
            rte_pktmbuf_free(rx_bufs[i]);
        }

        if (cur_frame_ == cfg->frames_to_test) {
            break;
        }
#else
        int recvlen = -1;
        // if ((recvlen = recv(socket_local, (char*)cur_buffer_ptr,
        // packet_length, 0))<0) {
        if ((recvlen = recvfrom(socket_local, (char*)cur_buffer_ptr,
                 cfg->packet_length, 0, (struct sockaddr*)&remote_addr,
                 &addrlen))
            < 0) {
            perror("recv failed");
            exit(0);
        }

        // Read information from received packet
        auto* pkt = (struct Packet*)cur_buffer_ptr;
        int frame_id = pkt->frame_id_;

        if (kDebugSenderReceiver) {
            printf("RX thread %d received frame %d symbol %d, ant %d\n ", tid,
                frame_id, pkt->symbol_id_, pkt->ant_id_);
        }

        if (kIsWorkerTimingEnabled) {
            if (frame_id > prev_frame_id) {
                frame_start[frame_id] = get_time();
                prev_frame_id = frame_id;
            }
        }
        /* get the position in buffer */
        offset = cur_buffer_status_ptr - buffer_status_ptr;
        cur_buffer_status_ptr[0] = 1;
        cur_buffer_status_ptr
            = buffer_status_ptr + (offset + 1) % buffer_frame_num;
        cur_buffer_ptr = buffer_ptr
            + (cur_buffer_ptr - buffer_ptr + cfg->packet_length)
                % buffer_length;
#endif
    }
    printf("Receiver received all packets!\n");
    save_tx_data_to_file(0);
    completion_num_ ++;

    return NULL;
}

void Receiver::save_tx_data_to_file(int frame_id)
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tx_data.bin";
    printf("Saving TX data to %s\n", filename.c_str());
    FILE* fp = fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        size_t total_data_symbol_id
            = cfg->get_total_data_symbol_idx_dl(frame_id, i);

        for (size_t ant_id = 0; ant_id < cfg->BS_ANT_NUM; ant_id++) {
            short* socket_ptr = reinterpret_cast<short*>(&dl_ue_data_buffer_[ant_id][total_data_symbol_id * sizeof(short) * 2 * cfg->OFDM_CA_NUM]);
            // if (ant_id == 0) {
            //     printf("Symbol offset: %u\n", total_data_symbol_id);
            //     for (size_t j = 0; j < cfg->OFDM_CA_NUM; j ++) {
            //         printf("(%u %u) ", socket_ptr[j * 2], socket_ptr[j * 2 + 1]);
            //     }
            //     printf("\n");
            // }
            fwrite(socket_ptr, cfg->sampsPerSymbol * 2, sizeof(short), fp);
        }
    }
    fclose(fp);
}

void Receiver::run_ifft(short* src, complex_float* ifft_inout,
    DFTI_DESCRIPTOR_HANDLE mkl_handle) const
{
    simd_convert_float16_to_float32(reinterpret_cast<float*>(ifft_inout), 
        reinterpret_cast<const float*>(src), cfg->OFDM_CA_NUM * 2);

    // Begin Debug
    // printf("Recovered precoded data:\n");
    // for (size_t i = 0; i < cfg->OFDM_CA_NUM; i ++) {
    //     printf("(%u %lf %lf) ", i, ifft_inout[i].re, ifft_inout[i].im);
    // }
    // printf("\n");
    // End Debug
    
    DftiComputeBackward(mkl_handle, ifft_inout);

    arma::cx_fmat mat_data((arma::cx_float*)ifft_inout, 1, cfg->OFDM_CA_NUM, false);
    float post_scale = cfg->OFDM_CA_NUM;
    mat_data /= post_scale;

    simd_convert_float_to_short(reinterpret_cast<float*>(ifft_inout), 
        src, cfg->OFDM_CA_NUM * 2);

    // Begin Debug
    // printf("IFFT data:\n");
    // for (size_t i = 0; i < cfg->OFDM_CA_NUM; i ++) {
    //     printf("(%u %d %d) ", i, src[i * 2], src[i * 2 + 1]);
    // }
    // printf("\n");
    // End Debug
}