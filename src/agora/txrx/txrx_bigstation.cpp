#include "txrx.hpp"
#include "Symbols.hpp"
#include <datatype_conversion.h>
#include <netinet/ether.h>

BigStationTXRX::BigStationTXRX(Config* cfg, size_t in_core_offset,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxAntennas, uint8_t>& time_iq_buffer,
    BigStationState* bigstation_state)
    : cfg_(cfg)
    , core_offset_(in_core_offset)
    , rx_thread_num_(cfg->rx_thread_num)
    , tx_thread_num_(cfg->tx_thread_num)
    , bigstation_state_(bigstation_state)
    , time_iq_buffer_(time_iq_buffer)
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
        for (size_t j = 0; j < cfg->symbol_num_perframe; j ++) {
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
            char* iq_ptr = cfg_->get_freq_domain_iq_buffer(freq_domain_iq_buffer_, pkt->ant_id_, pkt->frame_id_, pkt->symbol_id_);

            size_t sc_offset = Packet::kOffsetOfData + 2 * sizeof(unsigned short)
                    * (cfg_->OFDM_DATA_START + cfg_->subcarrier_start);
            DpdkTransport::fastMemcpy(iq_ptr, pkt, Packet::kOffsetOfData);
            memcpy(iq_ptr + sc_offset,
                (uint8_t*)pkt + Packet::kOffsetOfData,
                cfg_->get_num_sc_to_process() * 2 * sizeof(unsigned short));

            valid_pkts ++;
            size_t cur_cycle = rdtsc();
            if (unlikely(last_packet_cycle_[tid] == 0) && pkt->frame_id_ > 2000) {
                last_packet_cycle_[tid] = cur_cycle;
            }

            if (likely(last_packet_cycle_[tid] > 0) && unlikely(max_inter_packet_gap_[tid] < cur_cycle - last_packet_cycle_[tid])) {
                max_inter_packet_gap_[tid] = cur_cycle - last_packet_cycle_[tid];
                max_inter_packet_gap_frame_[tid] = pkt->frame_id_;
            }

            if (likely(last_packet_cycle_[tid] > 0)) {
                last_packet_cycle_[tid] = cur_cycle;
            }

            // get the position in rx_buffer
            cur_cycle = rdtsc();
            if (!shared_state_->receive_freq_iq_pkt(pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_)) {
                cfg_->error = true;
                cfg_->running = false;
            }
            size_t record_cycle = rdtsc() - cur_cycle;
            if (record_cycle > max_packet_record_time_[tid]) {
                max_packet_record_time_[tid] = record_cycle;
                max_packet_record_time_frame_[tid] = pkt->frame_id_;
            }

        } else if (pkt->pkt_type_ == Packet::PktType::kDemod) {
            const size_t symbol_idx_ul = pkt->symbol_id_;
            const size_t sc_id = cfg_->subcarrier_num_start[pkt->server_id_];

            int8_t* demod_ptr
                = cfg_->get_demod_buf_to_decode(demod_buffer_to_decode_,
                    pkt->frame_id_, symbol_idx_ul, pkt->ue_id_, sc_id);
            memcpy(demod_ptr, pkt->data_,
                cfg_->subcarrier_num_list[pkt->server_id_] * cfg_->mod_order_bits);
            if (!shared_state_->receive_demod_pkt(pkt->ue_id_, pkt->frame_id_, symbol_idx_ul)) {
                cfg_->error = true;
                cfg_->running = false;
            }
        } else if (pkt->pkt_type_ == Packet::PktType::kTimeIQ) {
            uint8_t* iq_ptr = time_iq_buffer_[pkt->frame_id_][pkt->symbol_id_][pkt->ant_id_];
            memcpy(iq_ptr, (uint8_t*)pkt + Packet::kOffsetOfData, cfg_->OFDM_CA_NUM * 2 * sizeof(unsigned short));
            if (!bigstation_state_->receive_time_iq_pkt(pkt->frame_id_, pkt->symbol_id_, pkt->ant_id_)) {
                cfg_->error = true;
                cfg_->running = false;
            }
        } else if (pkt->pkt_type_ == Packet::PktType::kEncode) {
            const size_t symbol_idx_dl = pkt->symbol_id_;

            int8_t* encode_ptr
                = cfg_->get_encoded_buf(encoded_buffer_to_precode_,
                    pkt->frame_id_, symbol_idx_dl, pkt->ue_id_);
            memcpy(encode_ptr, pkt->data_,
                cfg_->get_num_sc_to_process() * cfg_->mod_order_bits);
            if (!shared_state_->receive_encoded_pkt(pkt->frame_id_, symbol_idx_dl, pkt->ue_id_)) {
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

    while (cfg_->running) {     

    }
}