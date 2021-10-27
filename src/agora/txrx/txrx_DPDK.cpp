/**
 * @file txrx_DPDK.cpp
 * @brief Implementation of PacketTXRX datapath functions for communicating 
 * with DPDK
 */

#include "txrx.hpp"
#include "Symbols.hpp"
#include <datatype_conversion.h>
#include <netinet/ether.h>

static constexpr bool kDebugDPDK = false;

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset, 
    Table<char>& time_domain_iq_buffer,
    Table<char>& freq_domain_iq_buffer_to_send,
    Table<char>& freq_domain_iq_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_to_send,
    Table<int8_t>& demod_buffer_to_decode,
    SharedState* shared_state)
    : cfg_(cfg)
    , core_offset_(core_offset)
    , rx_thread_num_(cfg->rx_thread_num)
    , fft_tx_thread_num_(cfg->use_time_domain_iq ? cfg->fft_tx_thread_num : 0)
    , time_domain_iq_buffer_(time_domain_iq_buffer)
    , freq_domain_iq_buffer_to_send_(freq_domain_iq_buffer_to_send)
    , freq_domain_iq_buffer_(freq_domain_iq_buffer)
    , demod_buffer_to_send_(demod_buffer_to_send)
    , demod_buffer_to_decode_(demod_buffer_to_decode)
    , shared_state_(shared_state)
{
    DpdkTransport::dpdk_init(core_offset_, rx_thread_num_ + kNumDemodTxThread + fft_tx_thread_num_, cfg_->pci_addr);
    for (size_t i = 0; i < std::max(rx_thread_num_, fft_tx_thread_num_ + kNumDemodTxThread); i ++) {
        mbuf_pool_[i] = DpdkTransport::create_mempool(i);
    }
    demod_symbol_ul_to_send_ = 0;

    const uint16_t port_id = 0; // The DPDK port ID
    if (DpdkTransport::nic_init(port_id, mbuf_pool_, rx_thread_num_, kNumDemodTxThread + fft_tx_thread_num_) != 0)
        rte_exit(EXIT_FAILURE, "Cannot init port %u\n", port_id);

    int ret = inet_pton(AF_INET, cfg->bs_rru_addr.c_str(), &bs_rru_addr_);
    rt_assert(ret == 1, "Invalid sender IP address");

    bs_server_addrs_.resize(cfg->bs_server_addr_list.size());
    bs_server_mac_addrs_.resize(cfg->bs_server_addr_list.size());

    ret = inet_pton(AF_INET, cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(), &bs_server_addrs_[cfg->bs_server_addr_idx]);
    rt_assert(ret == 1, "Invalid sender IP address");

    ether_addr* parsed_mac = ether_aton(cfg->bs_server_mac_list[cfg->bs_server_addr_idx].c_str());
    rt_assert(parsed_mac != NULL, "Invalid server mac address");
    memcpy(&bs_server_mac_addrs_[cfg->bs_server_addr_idx], parsed_mac, sizeof(ether_addr));

    parsed_mac = ether_aton(cfg->bs_rru_mac_addr.c_str());
    rt_assert(parsed_mac != NULL, "Invalid rru mac address");
    memcpy(&bs_rru_mac_addr_, parsed_mac, sizeof(ether_addr));

    for (size_t i = 0; i < cfg->BS_ANT_NUM; i++) {
        uint16_t src_port = rte_cpu_to_be_16(cfg->bs_rru_port + i);
        uint16_t dst_port = rte_cpu_to_be_16(cfg->bs_server_port + i);

        printf("Adding steering rule for src IP %s(%x), dest IP %s(%x), src port: %u, "
               "dst port: %u, queue: %zu\n",
            cfg->bs_rru_addr.c_str(), bs_rru_addr_, cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(), bs_server_addrs_[cfg->bs_server_addr_idx],
            rte_cpu_to_be_16(src_port), rte_cpu_to_be_16(dst_port), i % rx_thread_num_);
        DpdkTransport::install_flow_rule(
            port_id, i % rx_thread_num_, bs_rru_addr_, bs_server_addrs_[cfg->bs_server_addr_idx], src_port, dst_port);
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

    if (cfg_->use_time_domain_iq) {
        for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
            if (i == cfg->bs_server_addr_idx) {
                continue;
            }
            for (size_t j = 0; j < cfg->BS_ANT_NUM; j++) {
                uint16_t src_port = rte_cpu_to_be_16(cfg->fft_tx_port + j);
                uint16_t dst_port = rte_cpu_to_be_16(cfg->fft_rx_port + j);

                printf("Adding steering rule for src IP %s(%x), dest IP %s(%x), src port: %u, "
                    "dst port: %u, queue: %zu\n",
                    cfg->bs_server_addr_list[i].c_str(), bs_server_addrs_[i], cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(), bs_server_addrs_[cfg->bs_server_addr_idx],
                    rte_cpu_to_be_16(src_port), rte_cpu_to_be_16(dst_port), j);
                DpdkTransport::install_flow_rule(
                    port_id, j % rx_thread_num_, bs_server_addrs_[i], bs_server_addrs_[cfg->bs_server_addr_idx], src_port, dst_port);
            }
        }
    }

    printf("Number of DPDK cores: %d\n", rte_lcore_count());
}

PacketTXRX::~PacketTXRX() { 
    for (size_t i = 0; i < rx_thread_num_ + kNumDemodTxThread; i ++) {
        rte_mempool_free(mbuf_pool_[i]); 
    }
}

bool PacketTXRX::StartTXRX()
{
    unsigned int lcore_id;
    size_t worker_id = 0;
    // Launch specific task to cores
    RTE_LCORE_FOREACH_SLAVE(lcore_id)
    {
        // launch communication and task thread onto specific core
        if (worker_id < rx_thread_num_) {
            auto context = new EventHandlerContext<PacketTXRX>;
            context->obj_ptr = this;
            context->id = worker_id;
            printf("Launch TXRX thread %zu on core %u\n", worker_id, lcore_id);
            rte_eal_remote_launch(
                (lcore_function_t*)
                    pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loop_tx_rx>,
                context, lcore_id);
        } else if (worker_id == rx_thread_num_) {
            auto context = new EventHandlerContext<PacketTXRX>;
            context->obj_ptr = this;
            context->id = worker_id;
            printf("Launch demod tx thread on core %u\n", lcore_id);
            rte_eal_remote_launch((lcore_function_t*)
                    pthread_fun_wrapper<PacketTXRX, &PacketTXRX::demod_tx_thread>,
                context, lcore_id);
        } else if (cfg_->use_time_domain_iq && worker_id > rx_thread_num_) {
            auto context = new EventHandlerContext<PacketTXRX>;
            context->obj_ptr = this;
            context->id = worker_id;
            printf("Launch fft tx thread on core %u\n", lcore_id);
            rte_eal_remote_launch((lcore_function_t*)
                    pthread_fun_wrapper<PacketTXRX, &PacketTXRX::fft_tx_thread>,
                context, lcore_id);
        }
        worker_id ++;
    }

    return true;
}

void* PacketTXRX::fft_tx_thread(int tid)
{
    size_t freq_ghz = measure_rdtsc_freq();

    printf("FFT TX thread %d\n", tid);

    size_t start_tsc = 0;
    size_t work_tsc_duration = 0;
    size_t send_tsc_duration = 0;
    size_t loop_count = 0;
    size_t work_count = 0;
    
    size_t work_start_tsc, send_start_tsc, send_end_tsc;
    size_t worked;

    size_t fft_frame_to_send = 0;
    size_t fft_symbol_to_send = tid - rx_thread_num_ - kNumDemodTxThread;

    while (cfg_->running) {

        if (likely(start_tsc > 0)) {
            loop_count ++;
        }

        worked = 0;

        // 1. Try to send demodulated data to decoders
        if (shared_state_->is_fft_tx_ready(
                fft_frame_to_send, fft_symbol_to_send)) {

            if (unlikely(start_tsc == 0)) {
                start_tsc = rdtsc();
            }
            
            work_start_tsc = rdtsc();
            send_start_tsc = work_start_tsc;
            worked = 1;

            for (size_t ant_id = cfg_->ant_start; ant_id < cfg_->ant_end; ant_id++) {
                char* ant_ptr = cfg_->get_freq_domain_iq_buffer(freq_domain_iq_buffer_to_send_, ant_id, fft_frame_to_send, fft_symbol_to_send);

                for (size_t target_server_idx = 0; target_server_idx < cfg_->bs_server_addr_list.size(); target_server_idx ++) {
                    if (target_server_idx == cfg_->bs_server_addr_idx) {
                        char* target_ant_ptr
                            = cfg_->get_freq_domain_iq_buffer(freq_domain_iq_buffer_, ant_id, fft_frame_to_send, fft_symbol_to_send);

                        size_t sc_offset = Packet::kOffsetOfData + 2 * sizeof(unsigned short)
                                * (cfg_->OFDM_DATA_START + cfg_->subcarrier_start);
                        memcpy(target_ant_ptr + sc_offset, ant_ptr + 2 * sizeof(unsigned short)
                                * (cfg_->OFDM_DATA_START + cfg_->subcarrier_start),
                            cfg_->get_num_sc_to_process() * 2 * sizeof(unsigned short));
                        if (!shared_state_->receive_freq_iq_pkt(fft_frame_to_send, fft_symbol_to_send)) {
                            cfg_->running = false;
                        }
                    } else {
                        struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
                        tx_bufs[0] = DpdkTransport::alloc_udp(mbuf_pool_[tid - rx_thread_num_], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], 
                            bs_server_mac_addrs_[target_server_idx],
                            bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[target_server_idx], cfg_->fft_tx_port + ant_id, 
                            cfg_->fft_rx_port + ant_id, 
                            Packet::kOffsetOfData + cfg_->get_num_sc_to_process() * 2 * sizeof(unsigned short));
                        struct rte_ether_hdr* eth_hdr
                            = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);

                        char* payload = (char*)eth_hdr + kPayloadOffset;
                        auto* pkt = reinterpret_cast<Packet*>(payload);
                        pkt->pkt_type_ = Packet::PktType::kFreqIQ;
                        pkt->frame_id_ = fft_frame_to_send;
                        pkt->symbol_id_ = fft_symbol_to_send;
                        pkt->ant_id_ = ant_id;
                        pkt->server_id_ = cfg_->bs_server_addr_idx;
                        memcpy(pkt->data_, ant_ptr + 2 * sizeof(unsigned short) * (cfg_->OFDM_DATA_START + cfg_->subcarrier_num_start[target_server_idx]),
                            cfg_->subcarrier_num_list[target_server_idx] * 2 * sizeof(unsigned short));

                        // Send data (one OFDM symbol)
                        size_t nb_tx_new = rte_eth_tx_burst(0, tid - rx_thread_num_, tx_bufs, 1);
                        if (unlikely(nb_tx_new != 1)) {
                            printf("rte_eth_tx_burst() failed\n");
                            exit(0);
                        }
                    }
                }
            }
            fft_symbol_to_send += fft_tx_thread_num_;
            if (fft_symbol_to_send > cfg_->symbol_num_perframe) {
                fft_symbol_to_send = tid - rx_thread_num_ - kNumDemodTxThread;
                fft_frame_to_send ++;
            }

            send_end_tsc = rdtsc();
            send_tsc_duration += send_end_tsc - send_start_tsc;
            work_tsc_duration += send_end_tsc - work_start_tsc;
        }

        work_count += worked;
    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("FFT TX Thread duration stats: total time used %.2lfms, "
        "send %.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), "
        "working proportions (%zu/%zu: %.2lf%%)\n",
        cycles_to_ms(whole_duration, freq_ghz),
        cycles_to_ms(send_tsc_duration, freq_ghz), send_tsc_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, work_count * 100.0f / loop_count);

    return 0;
}

void* PacketTXRX::demod_tx_thread(int tid)
{
    size_t freq_ghz = measure_rdtsc_freq();

    printf("Demodulation TX thread\n");

    size_t start_tsc = 0;
    size_t work_tsc_duration = 0;
    size_t send_tsc_duration = 0;
    size_t loop_count = 0;
    size_t work_count = 0;
    
    size_t work_start_tsc, send_start_tsc, send_end_tsc;
    size_t worked;

    while (cfg_->running) {

        if (likely(start_tsc > 0)) {
            loop_count ++;
        }

        worked = 0;

        // 1. Try to send demodulated data to decoders
        if (shared_state_->is_demod_tx_ready(
                demod_frame_to_send_, demod_symbol_ul_to_send_)) {

            if (unlikely(start_tsc == 0)) {
                start_tsc = rdtsc();
            }
            
            work_start_tsc = rdtsc();
            send_start_tsc = work_start_tsc;
            worked = 1;

            for (size_t ue_id = 0; ue_id < cfg_->UE_NUM; ue_id++) {
                int8_t* demod_ptr = cfg_->get_demod_buf_to_send(demod_buffer_to_send_, demod_frame_to_send_, demod_symbol_ul_to_send_, ue_id);

                size_t target_server_idx = cfg_->get_server_idx_by_ue(ue_id);
                if (target_server_idx == cfg_->bs_server_addr_idx) {
                    int8_t* target_demod_ptr
                        = cfg_->get_demod_buf_to_decode(demod_buffer_to_decode_,
                            demod_frame_to_send_, demod_symbol_ul_to_send_, ue_id, cfg_->subcarrier_start);
                    memcpy(target_demod_ptr, demod_ptr, cfg_->get_num_sc_to_process() * cfg_->mod_order_bits);
                    if (!shared_state_->receive_demod_pkt(ue_id, demod_frame_to_send_, demod_symbol_ul_to_send_)) {
                        cfg_->running = false;
                    }
                } else {
                    struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
                    tx_bufs[0] = DpdkTransport::alloc_udp(mbuf_pool_[0], bs_server_mac_addrs_[cfg_->bs_server_addr_idx], bs_server_mac_addrs_[cfg_->get_server_idx_by_ue(ue_id)],
                        bs_server_addrs_[cfg_->bs_server_addr_idx], bs_server_addrs_[cfg_->get_server_idx_by_ue(ue_id)], cfg_->demod_tx_port + demod_symbol_ul_to_send_, cfg_->demod_rx_port + demod_symbol_ul_to_send_, 
                        Packet::kOffsetOfData + cfg_->get_num_sc_to_process() * cfg_->mod_order_bits);
                    struct rte_ether_hdr* eth_hdr
                        = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);

                    char* payload = (char*)eth_hdr + kPayloadOffset;
                    auto* pkt = reinterpret_cast<Packet*>(payload);
                    pkt->pkt_type_ = Packet::PktType::kDemod;
                    pkt->frame_id_ = demod_frame_to_send_;
                    pkt->symbol_id_ = demod_symbol_ul_to_send_;
                    pkt->ue_id_ = ue_id;
                    pkt->server_id_ = cfg_->bs_server_addr_idx;
                    memcpy(pkt->data_, demod_ptr,
                        cfg_->get_num_sc_to_process() * cfg_->mod_order_bits);

                    // Send data (one OFDM symbol)
                    size_t nb_tx_new = rte_eth_tx_burst(0, 0, tx_bufs, 1);
                    if (unlikely(nb_tx_new != 1)) {
                        printf("rte_eth_tx_burst() failed\n");
                        exit(0);
                    }
                }
            }
            demod_symbol_ul_to_send_++;
            if (demod_symbol_ul_to_send_
                == cfg_->ul_data_symbol_num_perframe) {
                demod_symbol_ul_to_send_ = 0;
                demod_frame_to_send_++;
            }

            send_end_tsc = rdtsc();
            send_tsc_duration += send_end_tsc - send_start_tsc;
            work_tsc_duration += send_end_tsc - work_start_tsc;
        }

        work_count += worked;
    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("Demod TX Thread duration stats: total time used %.2lfms, "
        "send %.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), "
        "working proportions (%zu/%zu: %.2lf%%)\n",
        cycles_to_ms(whole_duration, freq_ghz),
        cycles_to_ms(send_tsc_duration, freq_ghz), send_tsc_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, work_count * 100.0f / loop_count);

    return 0;
}

void* PacketTXRX::loop_tx_rx(int tid)
{
    int radio_lo = tid * cfg_->nRadios / rx_thread_num_;
    int radio_hi = (tid + 1) * cfg_->nRadios / rx_thread_num_;
    int radio_id = radio_lo;

    frame_to_send_[tid] = 0;

    size_t recv_pkts = 0;

    while (cfg_->running) {
        int res = recv_relocate(tid);
        if (res == 0)
            continue;

        recv_pkts += res;

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
        
    }
    printf("Thread %u receive %lu packets, max gap is %lu, frame %zu, max record %lu, frame %lu\n", tid, 
        recv_pkts, max_inter_packet_gap_[tid], max_inter_packet_gap_frame_[tid],
        max_packet_record_time_[tid], max_packet_record_time_frame_[tid]);
    return 0;
}

int PacketTXRX::recv_relocate(int tid)
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
            if (!shared_state_->receive_freq_iq_pkt(pkt->frame_id_, pkt->symbol_id_)) {
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
                cfg_->running = false;
            }
        } else if (pkt->pkt_type_ == Packet::PktType::kTimeIQ) {
            char* iq_ptr = cfg_->get_freq_domain_iq_buffer(time_domain_iq_buffer_, pkt->ant_id_, pkt->frame_id_, pkt->symbol_id_);
            memcpy(iq_ptr, (uint8_t*)pkt + Packet::kOffsetOfData, cfg_->OFDM_CA_NUM * 2 * sizeof(unsigned short));
            if (!shared_state_->receive_time_iq_pkt(pkt->frame_id_, pkt->symbol_id_)) {
                cfg_->running = false;
            }
        } else {
            printf("Received unknown packet from rru\n");
            exit(1);
        }

        rte_pktmbuf_free(rx_bufs[i]);
    }
    return valid_pkts;
}

#if 0
void* PacketTXRX::encode_thread(int tid)
{
    std::vector<uint8_t> recv_buf(cfg->packet_length);

    printf("Encoding TX/RX thread\n");
    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    while (cfg->running) {
        // 1. Try to send encoded data to dosubcarriers
        if (encode_status_->ready_to_precode(
                encode_ue_to_send_, encode_frame_to_send_, encode_symbol_dl_to_send_)) {
            int8_t* ptr = cfg->get_encoded_buf(encoded_buffer_, encode_frame_to_send_, 
                encode_symbol_dl_to_send_, encode_ue_to_send_, 0);
            // printf("Start to send encoded data frame %u symbol %u ue %u\n", encode_frame_to_send_, encode_symbol_dl_to_send_, encode_ue_to_send_);
            
            for (size_t server_idx = 0; server_idx < cfg->bs_server_addr_list.size(); server_idx ++) {
                // int8_t* src_ptr = ptr + cfg->get_num_sc_per_server() * server_idx;
                int8_t* src_ptr = ptr + cfg->subcarrier_num_start[server_idx];
                if (server_idx == cfg->bs_server_addr_idx) {
                    int8_t* dst_ptr = cfg->get_encoded_buf(encoded_buffer_to_precode_, encode_frame_to_send_,
                        encode_symbol_dl_to_send_, encode_ue_to_send_, 0) + cfg->subcarrier_start;
                    // memcpy(dst_ptr, src_ptr, cfg->get_num_sc_per_server());
                    memcpy(dst_ptr, src_ptr, cfg->get_num_sc_to_process());
                    precode_status_->receive_encoded_data(encode_frame_to_send_, encode_symbol_dl_to_send_);
                } else {
                    struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
                    tx_bufs[0] = DpdkTransport::alloc_udp(mbuf_pool_[0], bs_server_mac_addrs_[cfg->bs_server_addr_idx], bs_server_mac_addrs_[server_idx],
                        bs_server_addrs_[cfg->bs_server_addr_idx], bs_server_addrs_[server_idx], cfg->encode_tx_port, cfg->encode_rx_port, cfg->packet_length);
                    struct rte_ether_hdr* eth_hdr
                        = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);

                    char* payload = (char*)eth_hdr + kPayloadOffset;
                    auto* pkt = reinterpret_cast<Packet*>(payload);
                    pkt->pkt_type_ = Packet::PktType::kEncode;
                    pkt->frame_id_ = encode_frame_to_send_;
                    pkt->symbol_id_ = encode_symbol_dl_to_send_;
                    pkt->ue_id_ = encode_ue_to_send_;
                    pkt->server_id_ = cfg->bs_server_addr_idx;
                    // DpdkTransport::fastMemcpy(pkt->data, src_ptr,
                    //     cfg->get_num_sc_per_server());
                    // memcpy(pkt->data, src_ptr, cfg->get_num_sc_per_server());
                    memcpy(pkt->data_, src_ptr, cfg->get_num_sc_to_process());

                    // printf("Send encoded data frame %u symbol %u ue %u to server %u\n", pkt->frame_id, pkt->symbol_id, pkt->ue_id, server_idx);
                    // Send data (one OFDM symbol)
                    size_t nb_tx_new = rte_eth_tx_burst(0, tid - 1, tx_bufs, 1);
                    if (unlikely(nb_tx_new != 1)) {
                        printf("rte_eth_tx_burst() failed\n");
                        exit(0);
                    }
                }
            }
    
            encode_ue_to_send_ ++;
            if (encode_ue_to_send_ == cfg->ue_end) {
                encode_ue_to_send_ = cfg->ue_start;
                encode_symbol_dl_to_send_ ++;
                if (encode_symbol_dl_to_send_ == cfg->dl_data_symbol_num_perframe) {
                    encode_symbol_dl_to_send_ = 0;
                    encode_frame_to_send_ ++;
                }
            }
        }

        // 2. Try to receive demodulated data for decoding
        rte_mbuf* rx_bufs[kRxBatchSize];
        uint16_t nb_rx = rte_eth_rx_burst(0, tid, rx_bufs, kRxBatchSize);
        if (unlikely(nb_rx == 0))
            continue;

        for (size_t i = 0; i < nb_rx; i++) {
            rte_mbuf* dpdk_pkt = rx_bufs[i];
            auto* eth_hdr = rte_pktmbuf_mtod(dpdk_pkt, rte_ether_hdr*);
            auto* ip_hdr = reinterpret_cast<rte_ipv4_hdr*>(
                reinterpret_cast<uint8_t*>(eth_hdr) + sizeof(rte_ether_hdr));
            auto* udp_hdr = reinterpret_cast<rte_udp_hdr*>(reinterpret_cast<uint8_t*>(eth_hdr) + sizeof(rte_ether_hdr) + sizeof(rte_ipv4_hdr));
            uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);

            if (eth_type != RTE_ETHER_TYPE_IPV4
                or ip_hdr->next_proto_id != IPPROTO_UDP) {
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }

            bool found = false;
            for (size_t j = 0; j < bs_server_addrs_.size(); j ++) {
                if (ip_hdr->src_addr == bs_server_addrs_[j]) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                // for (size_t j = 0; j < bs_server_addrs_.size(); j ++) {
                //     fprintf(stderr, "%x ", bs_server_addrs_[j]);
                // }
                // fprintf(stderr, "\n");
                // fprintf(stderr, "DPDK relocate(%u): Source addr does not match (%x:%u->%x:%u)\n", tid, ip_hdr->src_addr, rte_be_to_cpu_16(udp_hdr->src_port), ip_hdr->dst_addr, rte_be_to_cpu_16(udp_hdr->dst_port));
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }
            if (ip_hdr->dst_addr != bs_server_addrs_[cfg->bs_server_addr_idx]) {
                fprintf(stderr, "DPDK: Destination addr does not match (%x %x)\n", ip_hdr->dst_addr, bs_server_addrs_[cfg->bs_server_addr_idx]);
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }

            auto* pkt = reinterpret_cast<Packet*>((char*)(eth_hdr) + kPayloadOffset);
            if (pkt->pkt_type == Packet::PktType::kEncode) {
                // printf("TXRX receive encoded data frame %u symbol %u ue %u from server %u\n", pkt->frame_id, pkt->symbol_id, pkt->ue_id, pkt->server_id);
                const size_t symbol_idx_dl = pkt->symbol_id;
                const size_t ue_id = pkt->ue_id;

                int8_t* dst_ptr
                    = cfg->get_encoded_buf(encoded_buffer_to_precode_,
                        pkt->frame_id, symbol_idx_dl, pkt->ue_id, 0) + cfg->subcarrier_start;
                // DpdkTransport::fastMemcpy(dst_ptr, pkt->data,
                //     cfg->get_num_sc_per_server());
                // memcpy(dst_ptr, pkt->data, cfg->get_num_sc_per_server());
                memcpy(dst_ptr, pkt->data, cfg->get_num_sc_to_process());
                // Begin Debug
                // if (ue_id == 2) {
                //     complex_float tf = mod_single_uint8(((uint8_t*)pkt->data)[1], cfg->mod_table);
                //     printf("Received mod data: (%lf %lf)\n", tf.re, tf.im);
                // }
                // End Debug
                precode_status_->receive_encoded_data(pkt->frame_id, symbol_idx_dl);
            } else {
                printf("Received unknown packet type in encode TX/RX thread\n");
                exit(1);
            }

            rte_pktmbuf_free(rx_bufs[i]);
        }
    }
    return 0;
}

// TODO: check correctness of this funcion
int PacketTXRX::dequeue_send(int tid, size_t symbol_dl_to_send, size_t ant_to_send)
{
    if (shared_state_->cur_frame_ > frame_to_send_[tid]) {

        struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
        tx_bufs[0] = rte_pktmbuf_alloc(mbuf_pool_[tid]);
        struct rte_ether_hdr* eth_hdr
            = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
        eth_hdr->ether_type = rte_be_to_cpu_16(RTE_ETHER_TYPE_IPV4);
        memcpy(eth_hdr->s_addr.addr_bytes, bs_server_mac_addrs_[cfg->bs_server_addr_idx].addr_bytes,
            RTE_ETHER_ADDR_LEN);
        memcpy(eth_hdr->d_addr.addr_bytes, bs_rru_mac_addr_.addr_bytes,
            RTE_ETHER_ADDR_LEN);

        struct rte_ipv4_hdr* ip_h
            = (struct rte_ipv4_hdr*)((char*)eth_hdr + sizeof(struct rte_ether_hdr));
        ip_h->src_addr = bs_server_addrs_[cfg->bs_server_addr_idx];
        ip_h->dst_addr = bs_rru_addr_;
        ip_h->next_proto_id = IPPROTO_UDP;

        struct rte_udp_hdr* udp_h
            = (struct rte_udp_hdr*)((char*)ip_h + sizeof(struct rte_ipv4_hdr));
        udp_h->src_port = rte_cpu_to_be_16(cfg->bs_server_port);
        udp_h->dst_port = rte_cpu_to_be_16(cfg->bs_rru_port);

        tx_bufs[0]->pkt_len = cfg->packet_length + kPayloadOffset;
        tx_bufs[0]->data_len = cfg->packet_length + kPayloadOffset;

        char* payload = (char*)eth_hdr + kPayloadOffset;
        auto* pkt = reinterpret_cast<Packet*>(payload);
        pkt->pkt_type_ = Packet::PktType::kIQFromServer;
        pkt->frame_id_ = frame_to_send_[tid];
        pkt->symbol_id_ = symbol_dl_to_send;
        pkt->ant_id_ = ant_to_send;
        pkt->server_id_ = cfg->bs_server_addr_idx;
        size_t data_symbol_idx_dl = cfg->get_total_data_symbol_idx_dl(pkt->frame_id_, pkt->symbol_id_);
        size_t offset = data_symbol_idx_dl * cfg->BS_ANT_NUM + ant_to_send;

        simd_convert_float32_to_float16(reinterpret_cast<float*>(pkt->data_), 
            reinterpret_cast<float*>(&dl_ifft_buffer_[offset][0]), 
            cfg->get_num_sc_to_process() * 2);

        // Send data (one OFDM symbol)
        size_t nb_tx_new = rte_eth_tx_burst(0, tid, tx_bufs, 1);
        if (unlikely(nb_tx_new != 1)) {
            printf("rte_eth_tx_burst() failed\n");
            exit(0);
        }

        

        // dl_symbol_to_send_ ++;
        // if (dl_symbol_to_send_ == cfg->dl_data_symbol_num_perframe) {
        //     dl_symbol_to_send_ = 0;
        //     dl_frame_to_send_ ++;
        // }

        return 1;
    }
    
    return 0;
}
#endif