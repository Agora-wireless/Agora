#include "txrx_bigstation.hpp"
#include "Symbols.hpp"
#include <datatype_conversion.h>
#include <netinet/ether.h>

static constexpr bool kDebugDPDK = false;

size_t rte_eth_tx_burst_loop(int port, int queue, struct rte_mbuf** mbuf, int num) {
    size_t sum = 0;
    size_t count = 0;
    const size_t kMaxTxLoop = 1000;
    while (sum < num) {
        size_t cur_num = rte_eth_tx_burst(port, queue, mbuf + sum, num - sum);
        sum += cur_num;
        if (count >= kMaxTxLoop) {
            break;
        }
    }
    return sum;
}

BigStationTXRX::BigStationTXRX(Config* cfg, size_t in_core_offset,
    Table<char>& time_iq_buffer,
    Table<char>& freq_iq_buffer_to_send,
    Table<char>& freq_iq_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, uint8_t>& post_zf_buffer_to_send,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& post_zf_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& post_demul_buffer_to_send,
    Table<int8_t>& post_demul_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& post_decode_buffer,
    Table<int8_t>& dl_encoded_buffer_to_send,
    Table<int8_t>& dl_encoded_buffer,
    Table<complex_float>& dl_precoded_buffer_to_send,
    Table<complex_int16_t>& dl_precoded_buffer,
    BigStationState* bigstation_state)
    : cfg_(cfg)
    , core_offset_(in_core_offset)
    , rx_thread_num_(cfg->rx_thread_num)
    , tx_thread_num_(cfg->tx_thread_num)
    , time_iq_buffer_(time_iq_buffer)
    , freq_iq_buffer_to_send_(freq_iq_buffer_to_send)
    , freq_iq_buffer_(freq_iq_buffer)
    , post_zf_buffer_to_send_(post_zf_buffer_to_send)
    , post_zf_buffer_(post_zf_buffer)
    , post_demul_buffer_to_send_(post_demul_buffer_to_send)
    , post_demul_buffer_(post_demul_buffer)
    , post_decode_buffer_(post_decode_buffer)
    , dl_encoded_buffer_to_send_(dl_encoded_buffer_to_send)
    , dl_encoded_buffer_(dl_encoded_buffer)
    , dl_precoded_buffer_to_send_(dl_precoded_buffer_to_send)
    , dl_precoded_buffer_(dl_precoded_buffer)
    , bigstation_state_(bigstation_state)
{
    DpdkTransport::dpdk_init(core_offset_, rx_thread_num_ + tx_thread_num_, cfg_->pci_addr);
    for (size_t i = 0; i < std::max(rx_thread_num_, tx_thread_num_); i ++) {
        mbuf_pool_[i] = DpdkTransport::create_mempool(i);
    }

    const uint16_t port_id = 0; // The DPDK port ID
    if (DpdkTransport::nic_init(port_id, mbuf_pool_, rx_thread_num_, tx_thread_num_) != 0)
        rte_exit(EXIT_FAILURE, "Cannot init port %u\n", port_id);

    bs_rru_addrs_.resize(cfg->bs_rru_addr_list.size());
    for (size_t i = 0; i < cfg->bs_rru_addr_list.size(); i ++) {
        int ret = inet_pton(AF_INET, cfg->bs_rru_addr_list[i].c_str(), &bs_rru_addrs_[i]);
        rt_assert(ret == 1, "Invalid sender IP address");
    }

    bs_rru_mac_addrs_.resize(cfg->bs_rru_mac_list.size());
    for (size_t i = 0; i < cfg->bs_rru_mac_list.size(); i ++) {
        ether_addr* parsed_mac = ether_aton(cfg->bs_rru_mac_list[i].c_str());
        rt_assert(parsed_mac != NULL, "Invalid rru mac address");
        memcpy(&bs_rru_mac_addrs_[i], parsed_mac, sizeof(ether_addr));
    }

    bs_server_addrs_.resize(cfg->bs_server_addr_list.size());
    bs_server_mac_addrs_.resize(cfg->bs_server_addr_list.size());

    int ret = inet_pton(AF_INET, cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(), &bs_server_addrs_[cfg->bs_server_addr_idx]);
    rt_assert(ret == 1, "Invalid sender IP address");

    ether_addr* parsed_mac = ether_aton(cfg->bs_server_mac_list[cfg->bs_server_addr_idx].c_str());
    rt_assert(parsed_mac != NULL, "Invalid server mac address");
    memcpy(&bs_server_mac_addrs_[cfg->bs_server_addr_idx], parsed_mac, sizeof(ether_addr));

    for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
        uint16_t src_port = rte_cpu_to_be_16(cfg->bs_rru_port + i);
        uint16_t dst_port = rte_cpu_to_be_16(cfg->bs_server_port + i);
        for (size_t j = 0; j < cfg->bs_rru_addr_list.size(); j ++) {
            printf("Adding steering rule for src IP %s(%x), dest IP %s(%x), src port: %u, "
                "dst port: %u, queue: %zu\n",
                cfg->bs_rru_addr_list[j].c_str(), bs_rru_addrs_[j], cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(), bs_server_addrs_[cfg->bs_server_addr_idx],
                rte_cpu_to_be_16(src_port), rte_cpu_to_be_16(dst_port), i % rx_thread_num_);
            DpdkTransport::install_flow_rule(
                port_id, i % rx_thread_num_, bs_rru_addrs_[j], bs_server_addrs_[cfg->bs_server_addr_idx], src_port, dst_port);
        }
    }  

    for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
        if (i == cfg->bs_server_addr_idx) {
            continue;
        }
        int ret = inet_pton(AF_INET, cfg->bs_server_addr_list[i].c_str(), &bs_server_addrs_[i]);
        rt_assert(ret == 1, "Invalid sender IP address");
        ether_addr* parsed_mac = ether_aton(cfg->bs_server_mac_list[i].c_str());
        rt_assert(parsed_mac != NULL, "Invalid server mac address");
        memcpy(&bs_server_mac_addrs_[i], parsed_mac, sizeof(ether_addr));
        for (size_t j = 0; j < cfg->BS_ANT_NUM; j ++) {
            uint16_t src_port = rte_cpu_to_be_16(cfg->demod_tx_port + j);
            uint16_t dst_port = rte_cpu_to_be_16(cfg->demod_rx_port + j);
            printf("Adding steering rule for src IP %s, dest IP %s, src port: %zu, "
                "dst port: %zu, queue: %zu\n",
                cfg->bs_server_addr_list[i].c_str(), cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(),
                cfg->demod_tx_port + j, cfg->demod_rx_port + j, j % rx_thread_num_);
            DpdkTransport::install_flow_rule(
                port_id, j % rx_thread_num_, bs_server_addrs_[i], bs_server_addrs_[cfg->bs_server_addr_idx], src_port, dst_port);
        }
    } 

    printf("Number of DPDK cores: %d\n", rte_lcore_count());
}

BigStationTXRX::~BigStationTXRX() { 
    for (size_t i = 0; i < std::max(rx_thread_num_, tx_thread_num_); i ++) {
        rte_mempool_free(mbuf_pool_[i]); 
    }
}

bool BigStationTXRX::StartTXRX()
{
    unsigned int lcore_id;
    size_t worker_id = 0;
    // Launch specific task to cores
    RTE_LCORE_FOREACH_WORKER(lcore_id)
    {
        // launch communication and task thread onto specific core
        if (worker_id < rx_thread_num_) {
            auto context = new EventHandlerContext<BigStationTXRX>;
            context->obj_ptr = this;
            context->id = worker_id;
            printf("Launch BigStation RX thread %zu on core %u\n", worker_id, lcore_id);
            rte_eal_remote_launch((lcore_function_t*)
                pthread_fun_wrapper<BigStationTXRX, &BigStationTXRX::loop_tx_rx>,
                context, lcore_id);
        } else {
            auto context = new EventHandlerContext<BigStationTXRX>;
            context->obj_ptr = this;
            context->id = worker_id - rx_thread_num_;
            printf("Launch BigStation TX thread %zu on core %u\n", worker_id, lcore_id);
            if (cfg_->downlink_mode) {
                rte_eal_remote_launch((lcore_function_t*)
                    pthread_fun_wrapper<BigStationTXRX, &BigStationTXRX::tx_thread_dl>,
                    context, lcore_id);
            } else {
                rte_eal_remote_launch((lcore_function_t*)
                    pthread_fun_wrapper<BigStationTXRX, &BigStationTXRX::tx_thread_ul>,
                    context, lcore_id);
            }
        }
        worker_id ++;
    }

    return true;
}

void* BigStationTXRX::loop_tx_rx(int tid)
{
    size_t recv_pkts = 0;

    while (cfg_->running) {
        int res = recv_relocate(tid);
        if (res == 0)
            continue;

        recv_pkts += res;
    }
    printf("Thread %u receive %lu packets\n", tid, recv_pkts);
    return 0;
}

int BigStationTXRX::recv_relocate(int tid)
{
    rte_mbuf* rx_bufs[kRxBatchSize];
    uint16_t nb_rx = rte_eth_rx_burst(0, tid, rx_bufs, kRxBatchSize);
    if (unlikely(nb_rx == 0))
        return 0;

    size_t valid_pkts = 0;

    for (size_t i = 0; i < nb_rx; i++) {
        rte_mbuf* dpdk_pkt = rx_bufs[i];
        auto* eth_hdr = rte_pktmbuf_mtod(dpdk_pkt, rte_ether_hdr*);
        auto* ip_hdr = reinterpret_cast<rte_ipv4_hdr*>(
            reinterpret_cast<uint8_t*>(eth_hdr) + sizeof(rte_ether_hdr));
        uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);
        if (kDebugDPDK) {
            auto* udp_h = reinterpret_cast<rte_udp_hdr*>(
                reinterpret_cast<uint8_t*>(ip_hdr) + sizeof(rte_ipv4_hdr));
            DpdkTransport::print_pkt(ip_hdr->src_addr, ip_hdr->dst_addr,
                udp_h->src_port, udp_h->dst_port, dpdk_pkt->data_len, tid);
            printf("pkt_len: %d, nb_segs: %d, Header type: %d, IPV4: %d\n",
                dpdk_pkt->pkt_len, dpdk_pkt->nb_segs, eth_type,
                RTE_ETHER_TYPE_IPV4);
            printf("UDP: %d, %d\n", ip_hdr->next_proto_id, IPPROTO_UDP);
        }

        if (unlikely(eth_type != RTE_ETHER_TYPE_IPV4
            or ip_hdr->next_proto_id != IPPROTO_UDP)) {
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }

        if (unlikely(ip_hdr->dst_addr != bs_server_addrs_[cfg_->bs_server_addr_idx])) {
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }

        auto* pkt = reinterpret_cast<Packet*>(reinterpret_cast<uint8_t*>(eth_hdr) + kPayloadOffset);

        if (pkt->pkt_type_ == Packet::PktType::kFreqIQ) {
            char* dst_ptr = freq_iq_buffer_[pkt->ant_id_] + (pkt->frame_id_ % kFrameWnd) * cfg_->symbol_num_perframe
                * cfg_->packet_length + pkt->symbol_id_ * cfg_->packet_length + Packet::kOffsetOfData
                + (cfg_->OFDM_DATA_START + pkt->sc_id_) * 2 * sizeof(unsigned short);
            memcpy(dst_ptr, pkt->data_, pkt->sc_len_ * 2 * sizeof(unsigned short));
            if (pkt->symbol_id_ < cfg_->pilot_symbol_num_perframe) {
                // if (pkt->frame_id_ >= 1 && pkt->frame_id_ <= 25) {
                //     printf("[Frame %zu] Recv ant %zu pilot packets from server %zu sc %zu\n", 
                //         pkt->frame_id_, pkt->ant_id_, pkt->server_id_, pkt->sc_id_);
                // }
                if (!bigstation_state_->receive_pilot_pkt(pkt->frame_id_, pkt->ant_id_)) {
                    cfg_->error = true;
                    cfg_->running = false;
                }
            } else {
                if (!bigstation_state_->receive_ul_data_pkt(pkt->frame_id_, pkt->symbol_id_ - cfg_->pilot_symbol_num_perframe, pkt->ant_id_)) {
                    cfg_->error = true;
                    cfg_->running = false;
                }
            }
        } else if (pkt->pkt_type_ == Packet::PktType::kDemod) {
            int8_t* dst_ptr = post_demul_buffer_[(pkt->frame_id_%kFrameWnd)*cfg_->ul_data_symbol_num_perframe+pkt->symbol_id_] +
                cfg_->OFDM_DATA_NUM * kMaxModType * pkt->ue_id_ + pkt->sc_id_ * cfg_->mod_order_bits;
            memcpy(dst_ptr, pkt->data_, pkt->sc_len_ * cfg_->mod_order_bits);
            if (!bigstation_state_->receive_demod_pkt(pkt->frame_id_, pkt->symbol_id_, pkt->ue_id_, pkt->sc_len_)) {
                cfg_->error = true;
                cfg_->running = false;
            }
        } else if (pkt->pkt_type_ == Packet::PktType::kTimeIQ) {
            uint8_t* iq_ptr = (uint8_t*)time_iq_buffer_[pkt->ant_id_] + (pkt->frame_id_ % kFrameWnd) * cfg_->symbol_num_perframe
                * cfg_->packet_length + pkt->symbol_id_ * cfg_->packet_length + Packet::kOffsetOfData
                + pkt->sc_id_ * 2 * sizeof(unsigned short);
            memcpy(iq_ptr, (uint8_t*)pkt + Packet::kOffsetOfData, pkt->sc_len_ * 2 * sizeof(unsigned short));
            if (!bigstation_state_->receive_time_iq_pkt(pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_)) {
                cfg_->error = true;
                cfg_->running = false;
            }
            valid_pkts ++;
        } else if (pkt->pkt_type_ == Packet::PktType::kPostZF) {
            uint8_t* dst_ptr = ((uint8_t*)post_zf_buffer_[pkt->frame_id_%kFrameWnd][pkt->sc_id_]) + pkt->data_off_;
            memcpy(dst_ptr, pkt->data_, pkt->data_len_);
            if (!bigstation_state_->receive_zf_pkt(pkt->frame_id_, pkt->sc_id_)) {
                cfg_->error = true;
                cfg_->running = false;
            }
        } else if (pkt->pkt_type_ == Packet::PktType::kEncode) {
            uint8_t* dst_ptr = (uint8_t*)cfg_->get_encoded_buf(dl_encoded_buffer_, pkt->frame_id_, pkt->symbol_id_, pkt->ue_id_)
                + pkt->sc_id_ * cfg_->mod_order_bits;
            memcpy(dst_ptr, pkt->data_, pkt->sc_len_ * cfg_->mod_order_bits);
            if (!bigstation_state_->receive_encode_pkt(pkt->frame_id_, pkt->symbol_id_, pkt->ue_id_)) {
                cfg_->error = true;
                cfg_->running = false;
            }
        } else if (pkt->pkt_type_ == Packet::PktType::kPrecode) {
            size_t ant_offset = ((pkt->frame_id_ % kFrameWnd) * cfg_->dl_data_symbol_num_perframe + pkt->symbol_id_)
                * cfg_->BS_ANT_NUM + pkt->ant_id_;
            uint8_t* dst_ptr = (uint8_t*)(&dl_precoded_buffer_[ant_offset][cfg_->OFDM_DATA_START + pkt->sc_id_]);
            memcpy(dst_ptr, pkt->data_, pkt->sc_len_ * sizeof(complex_int16_t));
            // static size_t last_precode_frame = 200;
            // static size_t last_precode_symbol = 0;
            // if (pkt->frame_id_ > 200) {
            //     if (pkt->frame_id_ > last_precode_frame) {
            //         printf("Receive precode packet for frame %zu symbol %zu\n", pkt->frame_id_, pkt->symbol_id_);
            //         last_precode_frame = pkt->frame_id_;
            //         last_precode_symbol = pkt->symbol_id_;
            //     } else if (pkt->frame_id_ == last_precode_frame && pkt->symbol_id_ > last_precode_symbol) {
            //         printf("Receive precode packet for frame %zu symbol %zu\n", pkt->frame_id_, pkt->symbol_id_);
            //         last_precode_frame = pkt->frame_id_;
            //         last_precode_symbol = pkt->symbol_id_;
            //     }
            // }
            // printf("Receive precode packet frame %zu symbol %zu ant %zu\n", 
            //     pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_);
            if (!bigstation_state_->receive_precode_pkt(pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_, pkt->sc_len_)) {
                cfg_->error = true;
                cfg_->running = false;
            }
        } else {
            // printf("Received unknown packet from rru\n");
            // exit(1);
        }

        rte_pktmbuf_free(rx_bufs[i]);
    }
    return valid_pkts;
}

void* BigStationTXRX::tx_thread_ul(int tid)
{
    size_t freq_ghz = measure_rdtsc_freq();

    size_t start_tsc = 0;
    size_t work_tsc_duration = 0;
    size_t send_tsc_duration = 0;
    size_t loop_count = 0;
    size_t work_count = 0;
    
    size_t work_start_tsc, send_start_tsc, send_end_tsc;
    size_t worked;

    size_t ant_start = cfg_->ant_start + tid * (double)(cfg_->get_num_ant_to_process()) / cfg_->tx_thread_num;
    size_t ant_end = cfg_->ant_start + (tid + 1) * (double)(cfg_->get_num_ant_to_process()) / cfg_->tx_thread_num;
    size_t freq_iq_pkt_frame = 0;
    size_t freq_iq_pkt_symbol = 0;
    size_t freq_iq_pkt_ant = ant_start;

    size_t zf_worker_start = cfg_->zf_thread_offset + tid * (double)(cfg_->num_zf_workers[cfg_->bs_server_addr_idx]) / cfg_->tx_thread_num;
    size_t zf_worker_end = cfg_->zf_thread_offset + (tid + 1) * (double)(cfg_->num_zf_workers[cfg_->bs_server_addr_idx]) / cfg_->tx_thread_num;
    size_t zf_pkt_frame = 0;

    size_t ue_start = tid * (double)cfg_->UE_NUM / cfg_->tx_thread_num;
    size_t ue_end = (tid + 1) * (double)cfg_->UE_NUM / cfg_->tx_thread_num;
    size_t demul_pkt_frame = 0;
    size_t demul_pkt_symbol_ul = 0;
    size_t demul_pkt_ue = ue_start;

    while (cfg_->running) {     
        if (likely(start_tsc > 0)) {
            loop_count ++;
        }

        worked = 0;

        if (bigstation_state_->prepared_all_freq_iq_pkts(freq_iq_pkt_frame, freq_iq_pkt_symbol)) {
            if (unlikely(start_tsc == 0)) {
                start_tsc = rdtsc();
            }
            work_start_tsc = rdtsc();
            send_start_tsc = work_start_tsc;
            worked = 1;
            if (freq_iq_pkt_symbol < cfg_->pilot_symbol_num_perframe) {
                uint8_t* ant_ptr = (uint8_t*)freq_iq_buffer_to_send_[freq_iq_pkt_ant] + 
                    (freq_iq_pkt_frame % kFrameWnd) * cfg_->symbol_num_perframe
                    * cfg_->packet_length + freq_iq_pkt_symbol * cfg_->packet_length;
                size_t sc_offset = simple_hash(freq_iq_pkt_frame) % cfg_->UE_NUM;

                struct rte_mbuf* tx_bufs[kMaxTxBufSize] __attribute__((aligned(64)));
                size_t total_buf_id = 0;
                for (size_t i = 0; i < cfg_->total_zf_workers; i ++) {
                    size_t sc_start = cfg_->OFDM_DATA_NUM * 1.0 * i / cfg_->total_zf_workers;
                    size_t sc_end = cfg_->OFDM_DATA_NUM * 1.0 * (i + 1) / cfg_->total_zf_workers;
                    int first_possible_sc = sc_start - sc_start % cfg_->UE_NUM + sc_offset;
                    int last_possible_sc = sc_end - sc_end % cfg_->UE_NUM + sc_offset;
                    if (first_possible_sc < (int)sc_start) {
                        first_possible_sc += cfg_->UE_NUM;
                    }
                    if (last_possible_sc >= (int)sc_end) {
                        last_possible_sc -= cfg_->UE_NUM;
                    }
                    size_t server_id = cfg_->zf_server_mapping[sc_start];
                    size_t sc_len = 0;
                    if (last_possible_sc >= first_possible_sc) {
                        sc_len = last_possible_sc - first_possible_sc + cfg_->UE_NUM;
                    }
                    uint8_t* src_ptr = ant_ptr + (cfg_->OFDM_DATA_START + first_possible_sc - 
                        first_possible_sc % cfg_->UE_NUM) * 2 * sizeof(unsigned short);
                    
                    // if (freq_iq_pkt_frame >= 1 && freq_iq_pkt_frame <= 4) {
                    //     printf("[Frame %zu] Send ant %zu pilot packets to server %zu sc %zu\n", 
                    //         freq_iq_pkt_frame, freq_iq_pkt_ant, server_id, first_possible_sc - first_possible_sc % cfg_->UE_NUM);
                    // }
                    if (server_id == cfg_->bs_server_addr_idx) {
                        char* dst_ptr = freq_iq_buffer_[freq_iq_pkt_ant] + (freq_iq_pkt_frame % kFrameWnd) * cfg_->symbol_num_perframe
                            * cfg_->packet_length + freq_iq_pkt_symbol * cfg_->packet_length + Packet::kOffsetOfData
                            + (cfg_->OFDM_DATA_START + first_possible_sc - first_possible_sc % cfg_->UE_NUM) * 2 * sizeof(unsigned short);
                        memcpy(dst_ptr, src_ptr, sc_len * 2 * sizeof(unsigned short));
                        if (!bigstation_state_->receive_pilot_pkt(freq_iq_pkt_frame, freq_iq_pkt_ant)) {
                            cfg_->error = true;
                            cfg_->running = false;
                        }
                    } else {
                        tx_bufs[total_buf_id] = DpdkTransport::alloc_udp(mbuf_pool_[tid], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                            bs_server_mac_addrs_[server_id],
                            bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                            cfg_->demod_tx_port + freq_iq_pkt_ant, cfg_->demod_rx_port + freq_iq_pkt_ant, 
                            Packet::kOffsetOfData + sc_len * 2 * sizeof(short));
                        struct rte_ether_hdr* eth_hdr
                            = rte_pktmbuf_mtod(tx_bufs[total_buf_id], struct rte_ether_hdr*);
                        char* payload = (char*)eth_hdr + kPayloadOffset;
                        auto* pkt = reinterpret_cast<Packet*>(payload);
                        pkt->pkt_type_ = Packet::PktType::kFreqIQ;
                        pkt->frame_id_ = freq_iq_pkt_frame;
                        pkt->symbol_id_ = freq_iq_pkt_symbol;
                        pkt->ant_id_ = freq_iq_pkt_ant;
                        pkt->server_id_ = cfg_->bs_server_addr_idx;
                        pkt->sc_id_ = first_possible_sc - first_possible_sc % cfg_->UE_NUM;
                        pkt->sc_len_ = sc_len;
                        memcpy(pkt->data_, src_ptr, sc_len * 2 * sizeof(short));
                        total_buf_id ++;
                    }
                }
                // Send data (one OFDM symbol)
                for (size_t buf_id = 0; buf_id < total_buf_id; buf_id += kTxBatchSize) {
                    size_t batch_size = std::min(kTxBatchSize, total_buf_id - buf_id);
                    size_t nb_tx_new = rte_eth_tx_burst_loop(0, tid, tx_bufs + buf_id, batch_size);
                    if (unlikely(nb_tx_new != batch_size)) {
                        printf("rte_eth_tx_burst() failed\n");
                        exit(0);
                    }
                }
            } else {
                uint8_t* ant_ptr = (uint8_t*)freq_iq_buffer_to_send_[freq_iq_pkt_ant] + (freq_iq_pkt_frame % kFrameWnd) * 
                    cfg_->symbol_num_perframe * cfg_->packet_length + freq_iq_pkt_symbol * cfg_->packet_length;
                size_t demul_start = 0;
                size_t demul_end = 0;
                size_t cur_demul_worker_num = 0;
                struct rte_mbuf* tx_bufs[kMaxTxBufSize] __attribute__((aligned(64)));
                size_t total_buf_id = 0;
                for (size_t server_id = 0; server_id < cfg_->bs_server_addr_list.size(); server_id ++) {
                    demul_start = cfg_->OFDM_DATA_NUM * cur_demul_worker_num / cfg_->total_demul_workers;
                    cur_demul_worker_num += cfg_->num_demul_workers[server_id];
                    demul_end = cfg_->OFDM_DATA_NUM * cur_demul_worker_num / cfg_->total_demul_workers;
                    uint8_t* src_ptr = ant_ptr + (cfg_->OFDM_DATA_START + demul_start) * 2 * sizeof(unsigned short);
                    
                    if (server_id == cfg_->bs_server_addr_idx) {
                        char* dst_ptr = freq_iq_buffer_[freq_iq_pkt_ant] + (freq_iq_pkt_frame % kFrameWnd) * cfg_->symbol_num_perframe
                            * cfg_->packet_length + freq_iq_pkt_symbol * cfg_->packet_length + Packet::kOffsetOfData
                            + (cfg_->OFDM_DATA_START + demul_start) * 2 * sizeof(unsigned short);
                        memcpy(dst_ptr, src_ptr, (demul_end - demul_start) * 2 * sizeof(unsigned short));
                        if (!bigstation_state_->receive_ul_data_pkt(freq_iq_pkt_frame, freq_iq_pkt_symbol - cfg_->pilot_symbol_num_perframe, freq_iq_pkt_ant)) {
                            cfg_->error = true;
                            cfg_->running = false;
                        }
                    } else {
                        tx_bufs[total_buf_id] = DpdkTransport::alloc_udp(mbuf_pool_[tid], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                            bs_server_mac_addrs_[server_id],
                            bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                            cfg_->demod_tx_port + freq_iq_pkt_ant, cfg_->demod_rx_port + freq_iq_pkt_ant, 
                            Packet::kOffsetOfData + (demul_end - demul_start) * 2 * sizeof(short));
                        struct rte_ether_hdr* eth_hdr
                            = rte_pktmbuf_mtod(tx_bufs[total_buf_id], struct rte_ether_hdr*);
                        char* payload = (char*)eth_hdr + kPayloadOffset;
                        auto* pkt = reinterpret_cast<Packet*>(payload);
                        pkt->pkt_type_ = Packet::PktType::kFreqIQ;
                        pkt->frame_id_ = freq_iq_pkt_frame;
                        pkt->symbol_id_ = freq_iq_pkt_symbol;
                        pkt->ant_id_ = freq_iq_pkt_ant;
                        pkt->server_id_ = cfg_->bs_server_addr_idx;
                        pkt->sc_id_ = demul_start;
                        pkt->sc_len_ = demul_end - demul_start;
                        memcpy(pkt->data_, src_ptr, pkt->sc_len_ * 2 * sizeof(short));
                        total_buf_id ++;
                    }
                }

                // Send data (one OFDM symbol)
                for (size_t buf_id = 0; buf_id < total_buf_id; buf_id += kTxBatchSize) {
                    size_t batch_size = std::min(kTxBatchSize, total_buf_id - buf_id);
                    size_t nb_tx_new = rte_eth_tx_burst_loop(0, tid, tx_bufs + buf_id, batch_size);
                    if (unlikely(nb_tx_new != batch_size)) {
                        printf("rte_eth_tx_burst() failed\n");
                        exit(0);
                    }
                }
            }

            freq_iq_pkt_ant ++;
            if (freq_iq_pkt_ant == ant_end) {
                freq_iq_pkt_ant = ant_start;
                freq_iq_pkt_symbol ++;
                if (freq_iq_pkt_symbol == cfg_->symbol_num_perframe) {
                    freq_iq_pkt_symbol = 0;
                    freq_iq_pkt_frame ++;
                }
            }

            send_end_tsc = rdtsc();
            send_tsc_duration += send_end_tsc - send_start_tsc;
            work_tsc_duration += send_end_tsc - work_start_tsc;
        }

        if (bigstation_state_->prepared_all_zf_pkts(zf_pkt_frame)) {
            if (unlikely(start_tsc == 0)) {
                start_tsc = rdtsc();
            }
            work_start_tsc = rdtsc();
            send_start_tsc = work_start_tsc;
            worked = 1;

            size_t sc_offset = simple_hash(zf_pkt_frame) % cfg_->UE_NUM;
            struct rte_mbuf* tx_bufs[kMaxTxBufSize] __attribute__((aligned(64)));
            size_t total_buf_id = 0;
            for (size_t worker_id = zf_worker_start; worker_id < zf_worker_end; worker_id ++) {
                size_t sc_start = cfg_->OFDM_DATA_NUM * 1.0 * worker_id / cfg_->total_zf_workers;
                size_t sc_end = cfg_->OFDM_DATA_NUM * 1.0 * (worker_id + 1) / cfg_->total_zf_workers;
                int first_possible_sc = sc_start - sc_start % cfg_->UE_NUM + sc_offset;
                int last_possible_sc = sc_end - sc_end % cfg_->UE_NUM + sc_offset;
                if (first_possible_sc < (int)sc_start) {
                    first_possible_sc += cfg_->UE_NUM;
                }
                if (last_possible_sc >= (int)sc_end) {
                    last_possible_sc -= cfg_->UE_NUM;
                }
                if (last_possible_sc >= first_possible_sc) {
                    for (int sc_id = first_possible_sc; sc_id <= last_possible_sc; sc_id += cfg_->UE_NUM) {
                        size_t server_start = cfg_->demul_server_mapping[sc_id - sc_id % cfg_->UE_NUM];
                        size_t server_end = cfg_->demul_server_mapping[sc_id - sc_id % cfg_->UE_NUM + cfg_->UE_NUM - 1];
                        for (size_t server_id = server_start; server_id <= server_end; server_id ++) {
                            for (size_t byte_off = 0; byte_off < cfg_->BS_ANT_NUM * cfg_->UE_NUM * 2 * sizeof(float); byte_off += cfg_->post_zf_step_size) {                    
                                uint8_t* src_ptr = post_zf_buffer_to_send_[zf_pkt_frame%kFrameWnd][sc_id - sc_id % cfg_->UE_NUM] + byte_off;
                                size_t data_len = std::min(cfg_->BS_ANT_NUM * cfg_->UE_NUM * 2 * sizeof(float) - byte_off, (size_t)cfg_->post_zf_step_size);
                                if (server_id == cfg_->bs_server_addr_idx) {
                                    uint8_t* dst_ptr = ((uint8_t*)post_zf_buffer_[zf_pkt_frame%kFrameWnd][sc_id - sc_id % cfg_->UE_NUM]) + byte_off;
                                    memcpy(dst_ptr, src_ptr, data_len);
                                    if (!bigstation_state_->receive_zf_pkt(zf_pkt_frame, sc_id - sc_id % cfg_->UE_NUM)) {
                                        cfg_->error = true;
                                        cfg_->running = false;
                                    }
                                } else {
                                    tx_bufs[total_buf_id] = DpdkTransport::alloc_udp(mbuf_pool_[tid], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                                        bs_server_mac_addrs_[server_id],
                                        bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                                        cfg_->demod_tx_port + (sc_id % cfg_->UE_NUM), cfg_->demod_rx_port + (sc_id % cfg_->UE_NUM), 
                                        Packet::kOffsetOfData + data_len);
                                    struct rte_ether_hdr* eth_hdr
                                        = rte_pktmbuf_mtod(tx_bufs[total_buf_id], struct rte_ether_hdr*);
                                    char* payload = (char*)eth_hdr + kPayloadOffset;
                                    auto* pkt = reinterpret_cast<Packet*>(payload);
                                    pkt->pkt_type_ = Packet::PktType::kPostZF;
                                    pkt->frame_id_ = zf_pkt_frame;
                                    pkt->server_id_ = cfg_->bs_server_addr_idx;
                                    pkt->sc_id_ = sc_id - sc_id % cfg_->UE_NUM;
                                    pkt->data_off_ = byte_off;
                                    pkt->data_len_ = data_len;
                                    memcpy(pkt->data_, src_ptr, data_len);
                                    total_buf_id ++;
                                }
                            }
                        }
                    }
                }
            }

            // Send data (one OFDM symbol)
            for (size_t buf_id = 0; buf_id < total_buf_id; buf_id += kTxBatchSize) {
                size_t batch_size = std::min(kTxBatchSize, total_buf_id - buf_id);
                size_t nb_tx_new = rte_eth_tx_burst_loop(0, tid, tx_bufs + buf_id, batch_size);
                if (unlikely(nb_tx_new != batch_size)) {
                    printf("rte_eth_tx_burst() failed\n");
                    exit(0);
                }
            }
            zf_pkt_frame ++;

            send_end_tsc = rdtsc();
            send_tsc_duration += send_end_tsc - send_start_tsc;
            work_tsc_duration += send_end_tsc - work_start_tsc;
        }

        if (bigstation_state_->prepared_all_demod_pkts(demul_pkt_frame, demul_pkt_symbol_ul)) {
            if (unlikely(start_tsc == 0)) {
                start_tsc = rdtsc();
            }
            work_start_tsc = rdtsc();
            send_start_tsc = work_start_tsc;
            worked = 1;

            uint8_t* src_ptr = (uint8_t*)post_demul_buffer_to_send_[demul_pkt_frame%kFrameWnd][demul_pkt_symbol_ul][demul_pkt_ue] + cfg_->demul_start * cfg_->mod_order_bits;
            struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
            size_t server_id = cfg_->ue_server_mapping[demul_pkt_ue];

            if (server_id == cfg_->bs_server_addr_idx) { 
                int8_t* dst_ptr = post_demul_buffer_[(demul_pkt_frame%kFrameWnd)*cfg_->ul_data_symbol_num_perframe+demul_pkt_symbol_ul] +
                    cfg_->OFDM_DATA_NUM * kMaxModType * demul_pkt_ue + cfg_->demul_start * cfg_->mod_order_bits;
                memcpy(dst_ptr, src_ptr, cfg_->get_num_demul_sc_to_process() * cfg_->mod_order_bits);
                if (!bigstation_state_->receive_demod_pkt(demul_pkt_frame, demul_pkt_symbol_ul, demul_pkt_ue, cfg_->get_num_demul_sc_to_process())) {
                    cfg_->error = true;
                    cfg_->running = false;
                }
            } else {
                tx_bufs[0] = DpdkTransport::alloc_udp(mbuf_pool_[tid], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                    bs_server_mac_addrs_[server_id],
                    bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                    cfg_->demod_tx_port + demul_pkt_ue, cfg_->demod_rx_port + demul_pkt_ue, 
                    Packet::kOffsetOfData + cfg_->get_num_demul_sc_to_process() * cfg_->mod_order_bits);
                struct rte_ether_hdr* eth_hdr
                    = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
                char* payload = (char*)eth_hdr + kPayloadOffset;
                auto* pkt = reinterpret_cast<Packet*>(payload);
                pkt->pkt_type_ = Packet::PktType::kDemod;
                pkt->frame_id_ = demul_pkt_frame;
                pkt->symbol_id_ = demul_pkt_symbol_ul;
                pkt->ue_id_ = demul_pkt_ue;
                pkt->server_id_ = cfg_->bs_server_addr_idx;
                pkt->sc_id_ = cfg_->demul_start;
                pkt->sc_len_ = cfg_->get_num_demul_sc_to_process();
                memcpy(pkt->data_, src_ptr, cfg_->get_num_demul_sc_to_process() * cfg_->mod_order_bits);

                // Send data (one OFDM symbol)
                size_t nb_tx_new = rte_eth_tx_burst_loop(0, tid, tx_bufs, 1);
                if (unlikely(nb_tx_new != 1)) {
                    printf("rte_eth_tx_burst() failed\n");
                    exit(0);
                }
            }

            demul_pkt_ue ++;
            if (demul_pkt_ue == ue_end) {
                demul_pkt_ue = ue_start;
                demul_pkt_symbol_ul ++;
                if (demul_pkt_symbol_ul == cfg_->ul_data_symbol_num_perframe) {
                    demul_pkt_symbol_ul = 0;
                    demul_pkt_frame ++;
                }
            }

            send_end_tsc = rdtsc();
            send_tsc_duration += send_end_tsc - send_start_tsc;
            work_tsc_duration += send_end_tsc - work_start_tsc;
        }
        work_count += worked;
    }

    if (cfg_->error) {
        printf("TX error traceback: fft (frame %zu symbol %zu ant %zu), "
            "zf (frame %zu), demul (frame %zu symbol %zu ue %zu)\n", 
            freq_iq_pkt_frame, freq_iq_pkt_symbol, freq_iq_pkt_ant,
            zf_pkt_frame,
            demul_pkt_frame, demul_pkt_symbol_ul, demul_pkt_ue);
    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("TX Thread duration stats: total time used %.2lfms, "
        "send %.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), "
        "working proportions (%zu/%zu: %.2lf%%)\n",
        cycles_to_ms(whole_duration, freq_ghz),
        cycles_to_ms(send_tsc_duration, freq_ghz), send_tsc_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, work_count * 100.0f / loop_count);

    return nullptr;
}

void* BigStationTXRX::tx_thread_dl(int tid)
{
    size_t ant_start = cfg_->ant_start + tid * (double)(cfg_->get_num_ant_to_process()) / cfg_->tx_thread_num;
    size_t ant_end = cfg_->ant_start + (tid + 1) * (double)(cfg_->get_num_ant_to_process()) / cfg_->tx_thread_num;
    size_t freq_iq_pkt_frame = 0;
    size_t freq_iq_pkt_symbol = 0;
    size_t freq_iq_pkt_ant = ant_start;

    size_t zf_worker_start = cfg_->zf_thread_offset + tid * (double)(cfg_->num_zf_workers[cfg_->bs_server_addr_idx]) / cfg_->tx_thread_num;
    size_t zf_worker_end = cfg_->zf_thread_offset + (tid + 1) * (double)(cfg_->num_zf_workers[cfg_->bs_server_addr_idx]) / cfg_->tx_thread_num;
    size_t zf_pkt_frame = 0;

    size_t ue_start = cfg_->ue_start + tid * (double)(cfg_->ue_end - cfg_->ue_start) / cfg_->tx_thread_num;
    size_t ue_end = cfg_->ue_start + (tid + 1) * (double)(cfg_->ue_end - cfg_->ue_start) / cfg_->tx_thread_num;
    printf("tx thread ue %zu %zu\n", ue_start, ue_end);
    size_t encode_pkt_frame = 0;
    size_t encode_pkt_symbol_dl = 0;
    size_t encode_pkt_ue = ue_start;

    size_t precode_pkt_ant_start = tid * cfg_->BS_ANT_NUM / cfg_->tx_thread_num;
    size_t precode_pkt_ant_end = (tid + 1) * cfg_->BS_ANT_NUM / cfg_->tx_thread_num;
    size_t precode_pkt_frame = 0;
    size_t precode_pkt_symbol_dl = 0;
    // size_t precode_pkt_ant = 0;

    while (cfg_->running) {     
        if (bigstation_state_->prepared_all_freq_iq_pkts(freq_iq_pkt_frame, freq_iq_pkt_symbol)) {
            uint8_t* ant_ptr = (uint8_t*)freq_iq_buffer_to_send_[freq_iq_pkt_ant] + 
                (freq_iq_pkt_frame % kFrameWnd) * cfg_->symbol_num_perframe
                * cfg_->packet_length + freq_iq_pkt_symbol * cfg_->packet_length;
            size_t sc_offset = simple_hash(freq_iq_pkt_frame) % cfg_->UE_NUM;

            struct rte_mbuf* tx_bufs[kMaxTxBufSize] __attribute__((aligned(64)));
            size_t total_buf_id = 0;
            for (size_t i = 0; i < cfg_->total_zf_workers; i ++) {
                size_t sc_start = cfg_->OFDM_DATA_NUM * 1.0 * i / cfg_->total_zf_workers;
                size_t sc_end = cfg_->OFDM_DATA_NUM * 1.0 * (i + 1) / cfg_->total_zf_workers;
                int first_possible_sc = sc_start - sc_start % cfg_->UE_NUM + sc_offset;
                int last_possible_sc = sc_end - sc_end % cfg_->UE_NUM + sc_offset;
                if (first_possible_sc < (int)sc_start) {
                    first_possible_sc += cfg_->UE_NUM;
                }
                if (last_possible_sc >= (int)sc_end) {
                    last_possible_sc -= cfg_->UE_NUM;
                }
                size_t server_id = cfg_->zf_server_mapping[sc_start];
                size_t sc_len = 0;
                if (last_possible_sc >= first_possible_sc) {
                    sc_len = last_possible_sc - first_possible_sc + cfg_->UE_NUM;
                }
                uint8_t* src_ptr = ant_ptr + (cfg_->OFDM_DATA_START + first_possible_sc - 
                    first_possible_sc % cfg_->UE_NUM) * 2 * sizeof(unsigned short);
                
                if (server_id == cfg_->bs_server_addr_idx) {
                    char* dst_ptr = freq_iq_buffer_[freq_iq_pkt_ant] + (freq_iq_pkt_frame % kFrameWnd) * cfg_->symbol_num_perframe
                        * cfg_->packet_length + freq_iq_pkt_symbol * cfg_->packet_length + Packet::kOffsetOfData
                        + (cfg_->OFDM_DATA_START + first_possible_sc - first_possible_sc % cfg_->UE_NUM) * 2 * sizeof(unsigned short);
                    memcpy(dst_ptr, src_ptr, sc_len * 2 * sizeof(unsigned short));
                    if (!bigstation_state_->receive_pilot_pkt(freq_iq_pkt_frame, freq_iq_pkt_ant)) {
                        cfg_->error = true;
                        cfg_->running = false;
                    }
                } else {
                    tx_bufs[total_buf_id] = DpdkTransport::alloc_udp(mbuf_pool_[tid], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                        bs_server_mac_addrs_[server_id],
                        bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                        cfg_->demod_tx_port + freq_iq_pkt_ant, cfg_->demod_rx_port + freq_iq_pkt_ant, 
                        Packet::kOffsetOfData + sc_len * 2 * sizeof(short));
                    struct rte_ether_hdr* eth_hdr
                        = rte_pktmbuf_mtod(tx_bufs[total_buf_id], struct rte_ether_hdr*);
                    char* payload = (char*)eth_hdr + kPayloadOffset;
                    auto* pkt = reinterpret_cast<Packet*>(payload);
                    pkt->pkt_type_ = Packet::PktType::kFreqIQ;
                    pkt->frame_id_ = freq_iq_pkt_frame;
                    pkt->symbol_id_ = freq_iq_pkt_symbol;
                    pkt->ant_id_ = freq_iq_pkt_ant;
                    pkt->server_id_ = cfg_->bs_server_addr_idx;
                    pkt->sc_id_ = first_possible_sc - first_possible_sc % cfg_->UE_NUM;
                    pkt->sc_len_ = sc_len;
                    memcpy(pkt->data_, src_ptr, sc_len * 2 * sizeof(short));
                    total_buf_id ++;
                }
            }
            // Send data (one OFDM symbol)
            for (size_t buf_id = 0; buf_id < total_buf_id; buf_id += kTxBatchSize) {
                size_t batch_size = std::min(kTxBatchSize, total_buf_id - buf_id);
                size_t nb_tx_new = rte_eth_tx_burst_loop(0, tid, tx_bufs + buf_id, batch_size);
                if (unlikely(nb_tx_new != batch_size)) {
                    printf("rte_eth_tx_burst() failed\n");
                    exit(0);
                }
            }

            freq_iq_pkt_ant ++;
            if (freq_iq_pkt_ant == ant_end) {
                freq_iq_pkt_ant = ant_start;
                freq_iq_pkt_symbol ++;
                if (freq_iq_pkt_symbol == cfg_->pilot_symbol_num_perframe) {
                    freq_iq_pkt_symbol = 0;
                    freq_iq_pkt_frame ++;
                }
            }
        }

        if (bigstation_state_->prepared_all_zf_pkts(zf_pkt_frame)) {
            size_t sc_offset = simple_hash(zf_pkt_frame) % cfg_->UE_NUM;
            struct rte_mbuf* tx_bufs[kMaxTxBufSize] __attribute__((aligned(64)));
            size_t total_buf_id = 0;
            for (size_t worker_id = zf_worker_start; worker_id < zf_worker_end; worker_id ++) {
                size_t sc_start = cfg_->OFDM_DATA_NUM * 1.0 * worker_id / cfg_->total_zf_workers;
                size_t sc_end = cfg_->OFDM_DATA_NUM * 1.0 * (worker_id + 1) / cfg_->total_zf_workers;
                int first_possible_sc = sc_start - sc_start % cfg_->UE_NUM + sc_offset;
                int last_possible_sc = sc_end - sc_end % cfg_->UE_NUM + sc_offset;
                if (first_possible_sc < (int)sc_start) {
                    first_possible_sc += cfg_->UE_NUM;
                }
                if (last_possible_sc >= (int)sc_end) {
                    last_possible_sc -= cfg_->UE_NUM;
                }
                if (last_possible_sc >= first_possible_sc) {
                    for (int sc_id = first_possible_sc; sc_id <= last_possible_sc; sc_id += cfg_->UE_NUM) {
                        size_t server_start = cfg_->precode_server_mapping[sc_id - sc_id % cfg_->UE_NUM];
                        size_t server_end = cfg_->precode_server_mapping[sc_id - sc_id % cfg_->UE_NUM + cfg_->UE_NUM - 1];
                        for (size_t server_id = server_start; server_id <= server_end; server_id ++) {
                            for (size_t byte_off = 0; byte_off < cfg_->BS_ANT_NUM * cfg_->UE_NUM * 2 * sizeof(float); byte_off += cfg_->post_zf_step_size) {                    
                                uint8_t* src_ptr = post_zf_buffer_to_send_[zf_pkt_frame%kFrameWnd][sc_id - sc_id % cfg_->UE_NUM] + byte_off;
                                size_t data_len = std::min(cfg_->BS_ANT_NUM * cfg_->UE_NUM * 2 * sizeof(float) - byte_off, (size_t)cfg_->post_zf_step_size);
                                if (server_id == cfg_->bs_server_addr_idx) {
                                    uint8_t* dst_ptr = ((uint8_t*)post_zf_buffer_[zf_pkt_frame%kFrameWnd][sc_id - sc_id % cfg_->UE_NUM]) + byte_off;
                                    memcpy(dst_ptr, src_ptr, data_len);
                                    if (!bigstation_state_->receive_zf_pkt(zf_pkt_frame, sc_id - sc_id % cfg_->UE_NUM)) {
                                        cfg_->error = true;
                                        cfg_->running = false;
                                    }
                                } else {
                                    tx_bufs[total_buf_id] = DpdkTransport::alloc_udp(mbuf_pool_[tid], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                                        bs_server_mac_addrs_[server_id],
                                        bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                                        cfg_->demod_tx_port + (sc_id % cfg_->UE_NUM), cfg_->demod_rx_port + (sc_id % cfg_->UE_NUM), 
                                        Packet::kOffsetOfData + data_len);
                                    struct rte_ether_hdr* eth_hdr
                                        = rte_pktmbuf_mtod(tx_bufs[total_buf_id], struct rte_ether_hdr*);
                                    char* payload = (char*)eth_hdr + kPayloadOffset;
                                    auto* pkt = reinterpret_cast<Packet*>(payload);
                                    pkt->pkt_type_ = Packet::PktType::kPostZF;
                                    pkt->frame_id_ = zf_pkt_frame;
                                    pkt->server_id_ = cfg_->bs_server_addr_idx;
                                    pkt->sc_id_ = sc_id - sc_id % cfg_->UE_NUM;
                                    pkt->data_off_ = byte_off;
                                    pkt->data_len_ = data_len;
                                    memcpy(pkt->data_, src_ptr, data_len);
                                    total_buf_id ++;
                                }
                            }
                        }
                    }
                }
            }

            // Send data (one OFDM symbol)
            for (size_t buf_id = 0; buf_id < total_buf_id; buf_id += kTxBatchSize) {
                size_t batch_size = std::min(kTxBatchSize, total_buf_id - buf_id);
                size_t nb_tx_new = rte_eth_tx_burst_loop(0, tid, tx_bufs + buf_id, batch_size);
                if (unlikely(nb_tx_new != batch_size)) {
                    printf("rte_eth_tx_burst() failed\n");
                    exit(0);
                }
            }
            zf_pkt_frame ++;
        }

        if (bigstation_state_->prepared_all_encode_pkts(encode_pkt_frame, encode_pkt_symbol_dl)) {
            // uint8_t* src_ptr = (uint8_t*)post_demul_buffer_to_send_[demul_pkt_frame%kFrameWnd][demul_pkt_symbol_ul][demul_pkt_ue] + cfg_->demul_start * cfg_->mod_order_bits;
            uint8_t* ue_ptr = (uint8_t*)cfg_->get_encoded_buf(dl_encoded_buffer_to_send_, encode_pkt_frame, 
                encode_pkt_symbol_dl, encode_pkt_ue);
            struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));

            size_t precode_start = 0;
            size_t precode_end = 0;
            size_t cur_precode_worker_num = 0;
            size_t total_buf_id = 0;
            for (size_t server_id = 0; server_id < cfg_->bs_server_addr_list.size(); server_id ++) {
                precode_start = cfg_->OFDM_DATA_NUM * cur_precode_worker_num / cfg_->total_precode_workers;
                cur_precode_worker_num += cfg_->num_precode_workers[server_id];
                precode_end = cfg_->OFDM_DATA_NUM * cur_precode_worker_num / cfg_->total_precode_workers;
                uint8_t* src_ptr = ue_ptr + precode_start * cfg_->mod_order_bits;
                if (server_id == cfg_->bs_server_addr_idx) {
                    uint8_t* dst_ptr = (uint8_t*)cfg_->get_encoded_buf(dl_encoded_buffer_, encode_pkt_frame,
                        encode_pkt_symbol_dl, encode_pkt_ue) + precode_start * cfg_->mod_order_bits;
                    memcpy(dst_ptr, src_ptr, (precode_end - precode_start) * cfg_->mod_order_bits);
                    // if (encode_pkt_frame == 0 && encode_pkt_symbol_dl)
                    if (!bigstation_state_->receive_encode_pkt(encode_pkt_frame, encode_pkt_symbol_dl, encode_pkt_ue)) {
                        cfg_->error = true;
                        cfg_->running = false;
                    }
                } else {
                    tx_bufs[total_buf_id] = DpdkTransport::alloc_udp(mbuf_pool_[tid], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                        bs_server_mac_addrs_[server_id],
                        bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                        cfg_->demod_tx_port + encode_pkt_ue, cfg_->demod_rx_port + encode_pkt_ue, 
                        Packet::kOffsetOfData + (precode_end - precode_start) * cfg_->mod_order_bits);
                    struct rte_ether_hdr* eth_hdr
                        = rte_pktmbuf_mtod(tx_bufs[total_buf_id], struct rte_ether_hdr*);
                    char* payload = (char*)eth_hdr + kPayloadOffset;
                    auto* pkt = reinterpret_cast<Packet*>(payload);
                    pkt->pkt_type_ = Packet::PktType::kEncode;
                    pkt->frame_id_ = encode_pkt_frame;
                    pkt->symbol_id_ = encode_pkt_symbol_dl;
                    pkt->ue_id_ = encode_pkt_ue;
                    pkt->server_id_ = cfg_->bs_server_addr_idx;
                    pkt->sc_id_ = precode_start;
                    pkt->sc_len_ = precode_end - precode_start;
                    memcpy(pkt->data_, src_ptr, (precode_end - precode_start) * cfg_->mod_order_bits);
                    total_buf_id ++;
                }
            }
            // Send data (one OFDM symbol)
            size_t nb_tx_new = rte_eth_tx_burst_loop(0, tid, tx_bufs, total_buf_id);
            if (unlikely(nb_tx_new != total_buf_id)) {
                printf("rte_eth_tx_burst() failed\n");
                exit(0);
            }

            encode_pkt_ue ++;
            if (encode_pkt_ue == ue_end) {
                encode_pkt_ue = ue_start;
                encode_pkt_symbol_dl ++;
                if (encode_pkt_symbol_dl == cfg_->dl_data_symbol_num_perframe) {
                    encode_pkt_symbol_dl = 0;
                    encode_pkt_frame ++;
                }
            }
        }

        if (bigstation_state_->prepared_all_precode_pkt(precode_pkt_frame, precode_pkt_symbol_dl)) {
            size_t total_buf_id = 0;
            struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
            for (size_t precode_pkt_ant = precode_pkt_ant_start; precode_pkt_ant < precode_pkt_ant_end; precode_pkt_ant ++) {
                size_t ant_offset = ((precode_pkt_frame % kFrameWnd) * cfg_->dl_data_symbol_num_perframe + precode_pkt_symbol_dl)
                    * cfg_->BS_ANT_NUM + precode_pkt_ant;
                uint8_t* src_ptr = (uint8_t*)(&dl_precoded_buffer_to_send_[ant_offset][cfg_->precode_start]);
                size_t server_id = cfg_->ant_server_mapping[precode_pkt_ant];
                // if (precode_pkt_frame == 200 && precode_pkt_symbol_dl == 0) {
                //     printf("Send precode packet %zu %zu %zu %zu %zu to server %zu\n", precode_pkt_frame, precode_pkt_symbol_dl, precode_pkt_ant,
                //         cfg_->precode_start, cfg_->precode_end, server_id);
                // }
                if (server_id == cfg_->bs_server_addr_idx) {
                    unsigned short* dst_ptr = (unsigned short*)(&dl_precoded_buffer_[ant_offset][cfg_->OFDM_DATA_START + cfg_->precode_start]);
                    // memcpy(dst_ptr, src_ptr, (cfg_->precode_end - cfg_->precode_start) * sizeof(complex_float));
                    for (size_t i = 0; i < cfg_->precode_end - cfg_->precode_start; i ++) {
                        single_convert_float32_to_float16(dst_ptr + i * 2, (unsigned int*)(src_ptr + i * 2 * sizeof(float)));
                        single_convert_float32_to_float16(dst_ptr + i * 2 + 1, (unsigned int*)(src_ptr + (i * 2 + 1) * sizeof(float)));
                    }

                    if (!bigstation_state_->receive_precode_pkt(precode_pkt_frame, precode_pkt_symbol_dl, precode_pkt_ant, 
                        (cfg_->precode_end - cfg_->precode_start))) {
                        cfg_->error = true;
                        cfg_->running = false;
                    }
                } else {
                    tx_bufs[total_buf_id] = DpdkTransport::alloc_udp(mbuf_pool_[tid], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                        bs_server_mac_addrs_[server_id],
                        bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                        cfg_->demod_tx_port + precode_pkt_ant, cfg_->demod_rx_port + precode_pkt_ant, 
                        Packet::kOffsetOfData + (cfg_->precode_end - cfg_->precode_start) * sizeof(short) * 2);
                    struct rte_ether_hdr* eth_hdr
                        = rte_pktmbuf_mtod(tx_bufs[total_buf_id], struct rte_ether_hdr*);
                    char* payload = (char*)eth_hdr + kPayloadOffset;
                    auto* pkt = reinterpret_cast<Packet*>(payload);
                    pkt->pkt_type_ = Packet::PktType::kPrecode;
                    pkt->frame_id_ = precode_pkt_frame;
                    pkt->symbol_id_ = precode_pkt_symbol_dl;
                    pkt->ant_id_ = precode_pkt_ant;
                    pkt->server_id_ = cfg_->bs_server_addr_idx;
                    pkt->sc_id_ = cfg_->precode_start;
                    pkt->sc_len_ = cfg_->precode_end - cfg_->precode_start;
                    // memcpy(pkt->data_, src_ptr, (cfg_->precode_end - cfg_->precode_start) * sizeof(complex_float));
                    for (size_t i = 0; i < cfg_->precode_end - cfg_->precode_start; i ++) {
                        single_convert_float32_to_float16((unsigned short*)pkt->data_ + i * 2, (unsigned int*)(src_ptr + i * 2 * sizeof(float)));
                        single_convert_float32_to_float16((unsigned short*)pkt->data_ + i * 2 + 1, (unsigned int*)(src_ptr + (i * 2 + 1) * sizeof(float)));
                    }
                    // static size_t last_precode_frame = 200;
                    // static size_t last_precode_symbol = 0;
                    // if (precode_pkt_frame > 200) {
                    //     if (precode_pkt_frame > last_precode_frame) {
                    //         printf("Send precode packet for frame %zu symbol %zu\n", precode_pkt_frame, precode_pkt_symbol_dl);
                    //         last_precode_frame = precode_pkt_frame;
                    //         last_precode_symbol = precode_pkt_symbol_dl;
                    //     } else if (precode_pkt_frame == last_precode_frame && precode_pkt_symbol_dl > last_precode_symbol) {
                    //         printf("Send precode packet for frame %zu symbol %zu\n", precode_pkt_frame, precode_pkt_symbol_dl);
                    //         last_precode_frame = precode_pkt_frame;
                    //         last_precode_symbol = precode_pkt_symbol_dl;
                    //     }
                    // }

                    // printf("Send precode packet to %zu, frame %zu symbol %zu ant %zu\n", server_id, 
                    //     precode_pkt_frame, precode_pkt_symbol_dl, precode_pkt_ant);
                    size_t nb_tx_new = rte_eth_tx_burst_loop(0, tid, tx_bufs + total_buf_id, 1);
                    if (unlikely(nb_tx_new != 1)) {
                        printf("rte_eth_tx_burst() failed\n");
                        exit(0);
                    }
                    total_buf_id ++;
                }
            }
            // Send data (one OFDM symbol)
            // for (size_t buf_id = 0; buf_id < total_buf_id; buf_id += kTxBatchSize) {
            //     size_t batch_size = std::min(kTxBatchSize, total_buf_id - buf_id);
            //     size_t nb_tx_new = rte_eth_tx_burst_loop(0, tid, tx_bufs + buf_id, batch_size);
            //     if (unlikely(nb_tx_new != batch_size)) {
            //         printf("rte_eth_tx_burst() failed\n");
            //         exit(0);
            //     }
            // }
            // precode_pkt_ant ++;
            // if (precode_pkt_ant == cfg_->BS_ANT_NUM) {
                // precode_pkt_ant = 0;
                precode_pkt_symbol_dl ++;
                if (precode_pkt_symbol_dl == cfg_->dl_data_symbol_num_perframe) {
                    precode_pkt_symbol_dl = 0;
                    precode_pkt_frame ++;
                }
            // }
        }
    }

    if (cfg_->error) {
        printf("TX error traceback: fft (frame %zu symbol %zu ant %zu), "
            "zf (frame %zu), encode (frame %zu symbol %zu ue %zu), "
            "precode (frame %zu symbol %zu prep %d %zu %zu)\n", 
            freq_iq_pkt_frame, freq_iq_pkt_symbol, freq_iq_pkt_ant,
            zf_pkt_frame,
            encode_pkt_frame, encode_pkt_symbol_dl, encode_pkt_ue,
            precode_pkt_frame, precode_pkt_symbol_dl, 
            bigstation_state_->prepared_all_precode_pkt(precode_pkt_frame, precode_pkt_symbol_dl),
            bigstation_state_->num_precode_pkts_prepared_[precode_pkt_frame % kFrameWnd][precode_pkt_symbol_dl].load(),
            bigstation_state_->num_precode_pkts_prepared_per_symbol_);
    }

    return nullptr;
}

void BigStationTXRX::notify_sender()
{
    uint16_t src_port = 2222;
    uint16_t dst_port = 3333;
    const size_t magic = 0xea10bafe;
    rte_mbuf* mbuf = DpdkTransport::alloc_udp(mbuf_pool_[0], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
        bs_rru_mac_addrs_[0], bs_server_addrs_[cfg_->bs_server_addr_idx], bs_rru_addrs_[0], 
        src_port, dst_port, 4*sizeof(size_t));
    struct rte_ether_hdr* eth_hdr
        = rte_pktmbuf_mtod(mbuf, struct rte_ether_hdr*);
    char* payload = (char*)eth_hdr + kPayloadOffset;
    *((size_t*)payload) = magic;
    *((size_t*)payload + 1) = cfg_->bs_server_addr_idx;
    size_t nb_tx = rte_eth_tx_burst(0, 0, &mbuf, 1);
    if (unlikely(nb_tx != 1)) {
        printf("rte_eth_tx_burst() failed\n");
        exit(0);
    }
}