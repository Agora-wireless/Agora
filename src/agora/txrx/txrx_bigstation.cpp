#include "txrx_bigstation.hpp"
#include "Symbols.hpp"
#include <datatype_conversion.h>
#include <netinet/ether.h>

static constexpr bool kDebugDPDK = false;

BigStationTXRX::BigStationTXRX(Config* cfg, size_t in_core_offset,
    Table<char>& time_iq_buffer,
    Table<char>& freq_iq_buffer_to_send,
    Table<char>& freq_iq_buffer,
    PtrGrid<kFrameWnd, kMaxDataSCs, uint8_t>& post_zf_buffer_to_send,
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& post_zf_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& post_demul_buffer_to_send,
    Table<int8_t>& post_demul_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& post_decode_buffer,
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
            // if (pkt->symbol_id < cfg_->pilot_symbol_num_perframe) {
            //     char* dst_ptr = pilot_buffer_[pkt->frame_id][pkt->ant_id] + pkt->sc_id_ * 2 * sizeof(unsigned short);
            //     memcpy(dst_ptr, pkt->data, pkt->sc_len_ * 2 * sizeof(unsigned short));
            //     if (!bigstation_state_->receive_pilot_pkt(pkt->frame_id, pkt->ant_id)) {
            //         cfg_->error = true;
            //         cfg_->running = false;
            //     }
            // } else {
            //     char* dst_ptr = data_buffer_[pkt->frame_id][pkt->symbol_id - cfg_->pilot_symbol_num_perframe][pkt->ant_id] 
            //         + pkt->sc_id_ * 2 * sizeof(unsigned short); 
            //     memcpy(dst_ptr, pkt->data, pkt->sc_len_ * 2 * sizeof(unsigned short));
            //     if (!bigstation_state_->receive_ul_data_pkt(pkt->frame_id, pkt->symbol_id - cfg_->pilot_symbol_num_perframe, pkt->ant_id)) {
            //         cfg_->error = true;
            //         cfg_->running = false;
            //     }
            // }
            char* dst_ptr = freq_iq_buffer_[pkt->ant_id_] + (pkt->frame_id_ % kFrameWnd) * cfg_->symbol_num_perframe
                * cfg_->packet_length + pkt->symbol_id_ * cfg_->packet_length + Packet::kOffsetOfData
                + pkt->sc_id_ * 2 * sizeof(unsigned short);
            memcpy(dst_ptr, pkt->data_, pkt->sc_len_ * 2 * sizeof(unsigned short));
            if (pkt->symbol_id_ < cfg_->pilot_symbol_num_perframe) {
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
            // uint8_t* dst_ptr = post_demul_buffer_[pkt->frame_id_][pkt->symbol_id_][pkt->ue_id_] + pkt->sc_id_ * cfg_->mod_order_bits;
            int8_t* dst_ptr = post_demul_buffer_[(pkt->frame_id_%kFrameWnd)*cfg_->ul_data_symbol_num_perframe+pkt->symbol_id_] +
                cfg_->OFDM_DATA_NUM * kMaxModType * pkt->ue_id_ + pkt->sc_id_ * cfg_->mod_order_bits;
            memcpy(dst_ptr, pkt->data_, pkt->sc_len_ * cfg_->mod_order_bits);
            if (!bigstation_state_->receive_demod_pkt(pkt->frame_id_, pkt->symbol_id_, pkt->ue_id_, pkt->sc_len_)) {
                cfg_->error = true;
                cfg_->running = false;
            }
        } else if (pkt->pkt_type_ == Packet::PktType::kTimeIQ) {
            // uint8_t* iq_ptr = time_iq_buffer_[pkt->frame_id_][pkt->symbol_id_][pkt->ant_id_] + pkt->sc_id_ * 2 * sizeof(unsigned short);
            uint8_t* iq_ptr = (uint8_t*)time_iq_buffer_[pkt->ant_id_] + (pkt->frame_id_ % kFrameWnd) * cfg_->symbol_num_perframe
                * cfg_->packet_length + pkt->symbol_id_ * cfg_->packet_length + Packet::kOffsetOfData
                + pkt->sc_id_ * 2 * sizeof(unsigned short);
            memcpy(iq_ptr, (uint8_t*)pkt + Packet::kOffsetOfData, pkt->sc_len_ * 2 * sizeof(unsigned short));
            if (!bigstation_state_->receive_time_iq_pkt(pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_)) {
                cfg_->error = true;
                cfg_->running = false;
            }
        } else if (pkt->pkt_type_ == Packet::PktType::kPostZF) {
            uint8_t* dst_ptr = ((uint8_t*)post_zf_buffer_[pkt->frame_id_][pkt->sc_id_]) + pkt->data_off_;
            memcpy(dst_ptr, pkt->data_, pkt->data_len_);
            if (!bigstation_state_->receive_zf_pkt(pkt->frame_id_, pkt->sc_id_)) {
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

void* BigStationTXRX::tx_thread(int tid)
{
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
        if (bigstation_state_->prepared_all_freq_iq_pkts(freq_iq_pkt_frame, freq_iq_pkt_symbol)) {
            if (freq_iq_pkt_symbol < cfg_->pilot_symbol_num_perframe) {
                // uint8_t* ant_ptr = freq_iq_buffer_to_send_[freq_iq_pkt_frame][freq_iq_pkt_symbol][freq_iq_pkt_ant];
                uint8_t* ant_ptr = (uint8_t*)freq_iq_buffer_to_send_[freq_iq_pkt_ant] + 
                    (freq_iq_pkt_frame % kFrameWnd) * cfg_->symbol_num_perframe
                    * cfg_->packet_length + freq_iq_pkt_symbol * cfg_->packet_length;
                size_t sc_offset = simple_hash(freq_iq_pkt_frame) % cfg_->UE_NUM;

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
                    if (last_possible_sc >= first_possible_sc + 1) {
                        sc_len = (last_possible_sc - first_possible_sc + 1) * cfg_->UE_NUM;
                    }
                    uint8_t* src_ptr = ant_ptr + (cfg_->OFDM_DATA_START + first_possible_sc) * 2 * sizeof(unsigned short);
                    struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
                    tx_bufs[0] = DpdkTransport::alloc_udp(mbuf_pool_[0], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                        bs_server_mac_addrs_[server_id],
                        bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                        cfg_->demod_tx_port + freq_iq_pkt_ant, cfg_->demod_rx_port + freq_iq_pkt_ant, 
                        Packet::kOffsetOfData + sc_len * 2 * sizeof(short));
                    struct rte_ether_hdr* eth_hdr
                        = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
                    char* payload = (char*)eth_hdr + kPayloadOffset;
                    auto* pkt = reinterpret_cast<Packet*>(payload);
                    pkt->pkt_type_ = Packet::PktType::kFreqIQ;
                    pkt->frame_id_ = freq_iq_pkt_frame;
                    pkt->symbol_id_ = freq_iq_pkt_symbol;
                    pkt->ant_id_ = freq_iq_pkt_ant;
                    pkt->server_id_ = cfg_->bs_server_addr_idx;
                    pkt->sc_id_ = first_possible_sc;
                    pkt->sc_len_ = sc_len;
                    memcpy(pkt->data_, src_ptr, sc_len * 2 * sizeof(short));

                    // Send data (one OFDM symbol)
                    size_t nb_tx_new = rte_eth_tx_burst(0, 0, tx_bufs, 1);
                    if (unlikely(nb_tx_new != 1)) {
                        printf("rte_eth_tx_burst() failed\n");
                        exit(0);
                    }
                }
            } else {
                // uint8_t* ant_ptr = freq_iq_buffer_to_send_[freq_iq_pkt_frame][freq_iq_pkt_symbol][freq_iq_pkt_ant]; 
                uint8_t* ant_ptr = (uint8_t*)freq_iq_buffer_to_send_[freq_iq_pkt_ant] + (freq_iq_pkt_frame % kFrameWnd) * 
                    cfg_->symbol_num_perframe * cfg_->packet_length + freq_iq_pkt_symbol * cfg_->packet_length;
                size_t demul_start = 0;
                size_t demul_end = 0;
                size_t cur_demul_worker_num = 0;
                for (size_t server_id = 0; server_id < cfg_->bs_server_addr_list.size(); server_id ++) {
                    demul_start = cfg_->OFDM_DATA_NUM * cur_demul_worker_num / cfg_->total_demul_workers;
                    cur_demul_worker_num += cfg_->num_demul_workers[server_id];
                    demul_end = cfg_->OFDM_DATA_NUM * cur_demul_worker_num / cfg_->total_demul_workers;
                    uint8_t* src_ptr = ant_ptr + (cfg_->OFDM_DATA_START + demul_start) * 2 * sizeof(unsigned short);
                    struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
                    tx_bufs[0] = DpdkTransport::alloc_udp(mbuf_pool_[0], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                        bs_server_mac_addrs_[server_id],
                        bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                        cfg_->demod_tx_port + freq_iq_pkt_ant, cfg_->demod_rx_port + freq_iq_pkt_ant, 
                        Packet::kOffsetOfData + (demul_end - demul_start) * 2 * sizeof(short));
                    struct rte_ether_hdr* eth_hdr
                        = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
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

                    // Send data (one OFDM symbol)
                    size_t nb_tx_new = rte_eth_tx_burst(0, 0, tx_bufs, 1);
                    if (unlikely(nb_tx_new != 1)) {
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
        }

        if (bigstation_state_->prepared_all_zf_pkts(zf_pkt_frame)) {
            size_t sc_offset = simple_hash(zf_pkt_frame) % cfg_->UE_NUM;
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
                        for (size_t server_id = server_start; server_id < server_end; server_id ++) {
                            for (size_t byte_off = 0; byte_off < cfg_->BS_ANT_NUM * cfg_->UE_NUM * 2 * sizeof(float); byte_off += cfg_->post_zf_step_size) {                    
                                uint8_t* src_ptr = post_zf_buffer_to_send_[zf_pkt_frame][sc_id - sc_id % cfg_->UE_NUM] + byte_off;
                                size_t data_len = std::min(cfg_->BS_ANT_NUM * cfg_->UE_NUM * 2 * sizeof(float) - byte_off, (size_t)cfg_->post_zf_step_size);
                                struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
                                tx_bufs[0] = DpdkTransport::alloc_udp(mbuf_pool_[0], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                                    bs_server_mac_addrs_[server_id],
                                    bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[server_id], 
                                    cfg_->demod_tx_port + (sc_id % cfg_->UE_NUM), cfg_->demod_rx_port + (sc_id % cfg_->UE_NUM), 
                                    Packet::kOffsetOfData + data_len);
                                struct rte_ether_hdr* eth_hdr
                                    = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
                                char* payload = (char*)eth_hdr + kPayloadOffset;
                                auto* pkt = reinterpret_cast<Packet*>(payload);
                                pkt->pkt_type_ = Packet::PktType::kPostZF;
                                pkt->frame_id_ = zf_pkt_frame;
                                pkt->server_id_ = cfg_->bs_server_addr_idx;
                                pkt->sc_id_ = sc_id - sc_id % cfg_->UE_NUM;
                                pkt->data_off_ = byte_off;
                                pkt->data_len_ = data_len;
                                memcpy(pkt->data_, src_ptr, data_len);

                                // Send data (one OFDM symbol)
                                size_t nb_tx_new = rte_eth_tx_burst(0, 0, tx_bufs, 1);
                                if (unlikely(nb_tx_new != 1)) {
                                    printf("rte_eth_tx_burst() failed\n");
                                    exit(0);
                                }
                            }
                        }
                    }
                }
            }
            zf_pkt_frame ++;
        }

        if (bigstation_state_->prepared_all_demod_pkts(demul_pkt_frame, demul_pkt_symbol_ul)) {
            uint8_t* src_ptr = (uint8_t*)post_demul_buffer_to_send_[demul_pkt_frame][demul_pkt_symbol_ul][demul_pkt_ue] + cfg_->demul_start * cfg_->mod_order_bits;
            struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
            size_t server_id = cfg_->ue_server_mapping[demul_pkt_ue];
            tx_bufs[0] = DpdkTransport::alloc_udp(mbuf_pool_[0], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
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
            size_t nb_tx_new = rte_eth_tx_burst(0, 0, tx_bufs, 1);
            if (unlikely(nb_tx_new != 1)) {
                printf("rte_eth_tx_burst() failed\n");
                exit(0);
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
        }
    }

    return nullptr;
}