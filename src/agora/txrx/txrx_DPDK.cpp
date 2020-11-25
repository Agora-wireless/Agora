/**
 * @file txrx_DPDK.cpp
 * @brief Implementation of PacketTXRX datapath functions for communicating 
 * with DPDK
 */

#include "txrx.hpp"
#include <netinet/ether.h>

static constexpr bool kDebugDPDK = false;

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset, RxStatus* rx_status,
    DemulStatus* demul_status, DecodeStatus* decode_status)
    : cfg(cfg)
    , core_offset(core_offset)
    , socket_thread_num(cfg->socket_thread_num)
    , rx_status_(rx_status)
    , demul_status_(demul_status)
    , decode_status_(decode_status)
{
    DpdkTransport::dpdk_init(core_offset - 1, socket_thread_num + 1);
    mbuf_pool = DpdkTransport::create_mempool();
    demod_symbol_to_send_ = cfg->pilot_symbol_num_perframe;
    encode_ue_to_send_ = cfg->ue_start;

    const uint16_t port_id = 0; // The DPDK port ID
    if (DpdkTransport::nic_init(port_id, mbuf_pool, socket_thread_num + 1) != 0)
        rte_exit(EXIT_FAILURE, "Cannot init port %u\n", port_id);

    int ret = inet_pton(AF_INET, cfg->bs_rru_addr.c_str(), &bs_rru_addr);
    rt_assert(ret == 1, "Invalid sender IP address");

    bs_server_addrs_.reserve(cfg->bs_server_addr_list.size());
    bs_server_mac_addrs_.reserve(cfg->bs_server_addr_list.size());

    ret = inet_pton(AF_INET, cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(), &bs_server_addrs_[cfg->bs_server_addr_idx]);
    
    ether_addr* parsed_mac = ether_aton(cfg->bs_server_mac_list[cfg->bs_server_addr_idx].c_str());
    rt_assert(parsed_mac != NULL, "Invalid server mac address");
    memcpy(&bs_server_mac_addrs_[cfg->bs_server_addr_idx], parsed_mac, sizeof(ether_addr));

    rt_assert(ret == 1, "Invalid sender IP address");

    // for (size_t i = 0; i < socket_thread_num; i++) {
    //     uint16_t src_port = rte_cpu_to_be_16(cfg->bs_rru_port + i);
    //     uint16_t dst_port = rte_cpu_to_be_16(cfg->bs_server_port + i);

    //     printf("Adding steering rule for src IP %s, dest IP %s, src port: %zu, "
    //            "dst port: %zu, queue: %zu\n",
    //         cfg->bs_rru_addr.c_str(), cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(),
    //         cfg->bs_rru_port + i, cfg->bs_server_port + i, i);
    //     DpdkTransport::install_flow_rule(
    //         port_id, i, bs_rru_addr, bs_server_addrs_[cfg->bs_server_addr_idx], src_port, dst_port);
    // }

    for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
        if (i == cfg->bs_server_addr_idx) {
            continue;
        }
        int ret = inet_pton(AF_INET, cfg->bs_server_addr_list[i].c_str(), &bs_server_addrs_[i]);
        rt_assert(ret == 1, "Invalid sender IP address");
        ether_addr* parsed_mac = ether_aton(cfg->bs_server_mac_list[i].c_str());
        rt_assert(parsed_mac != NULL, "Invalid server mac address");
        memcpy(&bs_server_mac_addrs_[i], parsed_mac, sizeof(ether_addr));
        uint16_t src_port = rte_cpu_to_be_16(cfg->demod_tx_port);
        uint16_t dst_port = rte_cpu_to_be_16(cfg->demod_rx_port);
        // printf("Adding steering rule for src IP %s, dest IP %s, src port: %zu, "
        //        "dst port: %zu, queue: %zu\n",
        //     cfg->bs_server_addr_list[i].c_str(), cfg->bs_server_addr_list[cfg->bs_server_addr_idx].c_str(),
        //     cfg->demod_tx_port, cfg->demod_rx_port, socket_thread_num);
        // DpdkTransport::install_flow_rule(
        //     port_id, socket_thread_num, bs_server_addrs_[i], bs_server_addrs_[cfg->bs_server_addr_idx], src_port, dst_port);
    }

    printf("Number of DPDK cores: %d\n", rte_lcore_count());
}

PacketTXRX::~PacketTXRX() { rte_mempool_free(mbuf_pool); }

bool PacketTXRX::startTXRX(Table<char>& buffer,
    size_t packet_num_in_buffer, Table<size_t>& frame_start, char* tx_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>* demod_buffers,
    Table<int8_t>* demod_soft_buffer_to_decode)
{
    buffer_ = &buffer;
    frame_start_ = &frame_start;

    packet_num_in_buffer_ = packet_num_in_buffer;
    tx_buffer_ = tx_buffer;

    demod_buffers_ = demod_buffers;
    demod_soft_buffer_to_decode_ = demod_soft_buffer_to_decode;

    if (kUseArgos) {
        if (!radioconfig_->radioStart()) {
            fprintf(stderr, "Failed to start radio\n");
            return false;
        }
    }

    unsigned int lcore_id;
    size_t worker_id = 0;
    // Launch specific task to cores
    RTE_LCORE_FOREACH_SLAVE(lcore_id)
    {
        // launch communication and task thread onto specific core
        if (worker_id < socket_thread_num) {
            auto context = new EventHandlerContext<PacketTXRX>;
            context->obj_ptr = this;
            context->id = worker_id;
            rte_eal_remote_launch(
                (lcore_function_t*)
                    pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loop_tx_rx>,
                context, lcore_id);
        } else if (worker_id == socket_thread_num) {
            auto context = new EventHandlerContext<PacketTXRX>;
            context->obj_ptr = this;
            context->id = worker_id;
            rte_eal_remote_launch((lcore_function_t*)
                    pthread_fun_wrapper<PacketTXRX, &PacketTXRX::demod_thread>,
                context, lcore_id);
        }
        worker_id++;
    }

    if (kUseArgos)
        radioconfig_->go();

    return true;
}

void* PacketTXRX::demod_thread(int tid) 
{
    std::vector<uint8_t> recv_buf(cfg->packet_length);

    printf("Demodulation TX/RX thread\n");
    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;

    while (cfg->running) {
        // 1. Try to send demodulated data to decoders
        if (demul_status_->ready_to_decode(
                demod_frame_to_send_, demod_symbol_to_send_)) {
            for (size_t ue_id = 0; ue_id < cfg->UE_NUM; ue_id++) {
                int8_t* demod_ptr = (*demod_buffers_)[demod_frame_to_send_
                    % kFrameWnd][demod_symbol_to_send_
                    - cfg->pilot_symbol_num_perframe][ue_id];

                size_t target_server_idx = cfg->get_server_idx_by_ue(ue_id);
                if (target_server_idx == cfg->bs_server_addr_idx) {
                    int8_t* target_demod_ptr
                        = cfg->get_demod_buf_to_decode(*demod_soft_buffer_to_decode_,
                            demod_frame_to_send_, demod_symbol_to_send_, ue_id, cfg->bs_server_addr_idx * cfg->get_num_sc_per_server());
                    memcpy(target_demod_ptr, demod_ptr, cfg->get_num_sc_per_server() * cfg->mod_order_bits);
                    decode_status_->receive_demod_data(
                        ue_id, demod_frame_to_send_, demod_symbol_to_send_ - cfg->pilot_symbol_num_perframe);
                } else {
                    struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
                    tx_bufs[0] = rte_pktmbuf_alloc(mbuf_pool);
                    struct rte_ether_hdr* eth_hdr
                        = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
                    eth_hdr->ether_type = rte_be_to_cpu_16(RTE_ETHER_TYPE_IPV4);
                    memcpy(eth_hdr->s_addr.addr_bytes, bs_server_mac_addrs_[cfg->bs_server_addr_idx].addr_bytes,
                        RTE_ETHER_ADDR_LEN);
                    memcpy(eth_hdr->d_addr.addr_bytes, bs_server_mac_addrs_[cfg->get_server_idx_by_ue(ue_id)].addr_bytes,
                        RTE_ETHER_ADDR_LEN);

                    struct rte_ipv4_hdr* ip_h
                        = (struct rte_ipv4_hdr*)((char*)eth_hdr + sizeof(struct rte_ether_hdr));
                    ip_h->src_addr = bs_server_addrs_[cfg->bs_server_addr_idx];
                    ip_h->dst_addr = bs_server_addrs_[cfg->get_server_idx_by_ue(ue_id)];
                    ip_h->next_proto_id = IPPROTO_UDP;

                    struct rte_udp_hdr* udp_h
                        = (struct rte_udp_hdr*)((char*)ip_h + sizeof(struct rte_ipv4_hdr));
                    udp_h->src_port = rte_cpu_to_be_16(cfg->demod_tx_port);
                    udp_h->dst_port = rte_cpu_to_be_16(cfg->demod_rx_port);

                    tx_bufs[0]->pkt_len = cfg->packet_length + kPayloadOffset;
                    tx_bufs[0]->data_len = cfg->packet_length + kPayloadOffset;

                    char* payload = (char*)eth_hdr + kPayloadOffset;
                    auto* pkt = reinterpret_cast<Packet*>(payload);
                    pkt->pkt_type = Packet::PktType::kDemod;
                    pkt->frame_id = demod_frame_to_send_;
                    pkt->symbol_id = demod_symbol_to_send_;
                    pkt->ue_id = ue_id;
                    pkt->server_id = cfg->bs_server_addr_idx;
                    DpdkTransport::fastMemcpy(pkt->data, demod_ptr,
                        cfg->get_num_sc_per_server() * cfg->mod_order_bits);

                    // Send data (one OFDM symbol)
                    size_t nb_tx_new = rte_eth_tx_burst(0, tid - 1, tx_bufs, 1);
                    if (unlikely(nb_tx_new != 1)) {
                        printf("rte_eth_tx_burst() failed\n");
                        exit(0);
                    }
                }
            }
            demod_symbol_to_send_++;
            if (demod_symbol_to_send_
                == cfg->pilot_symbol_num_perframe
                    + cfg->ul_data_symbol_num_perframe) {
                demod_symbol_to_send_ = cfg->pilot_symbol_num_perframe;
                demod_frame_to_send_++;
            }
        }

        // 2. Try to receive demodulated data for decoding
        rte_mbuf* rx_bufs[kRxBatchSize];
        uint16_t nb_rx = rte_eth_rx_burst(0, tid, rx_bufs, kRxBatchSize);
        if (unlikely(nb_rx == 0))
            continue;

        for (size_t i = 0; i < nb_rx; i++) {
            printf("Received packet!\n");
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
            for (size_t j = 0; j < bs_server_addrs_.size(); j ++) {
                if (ip_hdr->src_addr == bs_server_addrs_[j]) {
                    found = true;
                    break;
                }
            }
            if (!found) {
                fprintf(stderr, "DPDK: Source addr does not match\n");
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }
            if (ip_hdr->dst_addr != bs_server_addrs_[cfg->bs_server_addr_idx]) {
                fprintf(stderr, "DPDK: Destination addr does not match (%x %x)\n", ip_hdr->dst_addr, bs_server_addrs_[cfg->bs_server_addr_idx]);
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }

            auto* pkt = reinterpret_cast<Packet*>(eth_hdr) + kPayloadOffset;
            if (pkt->pkt_type == Packet::PktType::kDemod) {
                const size_t symbol_idx_ul
                    = pkt->symbol_id - cfg->pilot_symbol_num_perframe;
                const size_t sc_id = pkt->server_id * cfg->get_num_sc_per_server();

                int8_t* demod_ptr
                    = cfg->get_demod_buf_to_decode(*demod_soft_buffer_to_decode_,
                        pkt->frame_id, symbol_idx_ul, pkt->ue_id, sc_id);
                DpdkTransport::fastMemcpy(demod_ptr, pkt->data,
                    cfg->get_num_sc_per_server() * cfg->mod_order_bits);
                decode_status_->receive_demod_data(
                    pkt->ue_id, pkt->frame_id, symbol_idx_ul);
            } else {
                printf("Received unknown packet type in demod TX/RX thread\n");
                exit(1);
            }

            rte_pktmbuf_free(rx_bufs[i]);
        }
    }

    return 0;
}

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
            
            for (size_t server_idx = 0; server_idx < cfg->bs_server_addr_list.size(); server_idx ++) {
                int8_t* src_ptr = ptr + cfg->get_num_sc_per_server() * server_idx;
                if (server_idx == cfg->bs_server_addr_idx) {
                    int8_t* dst_ptr = cfg->get_encoded_buf(encoded_buffer_to_precode_, encode_frame_to_send_,
                        encode_symbol_dl_to_send_, encode_ue_to_send_, 0) + cfg->get_num_sc_per_server() * server_idx;
                    memcpy(dst_ptr, src_ptr, cfg->get_num_sc_per_server());
                    precode_status_->receive_encoded_data(encode_ue_to_send_, encode_frame_to_send_, encode_symbol_dl_to_send_);
                } else {
                    struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
                    tx_bufs[0] = rte_pktmbuf_alloc(mbuf_pool);
                    struct rte_ether_hdr* eth_hdr
                        = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
                    eth_hdr->ether_type = rte_be_to_cpu_16(RTE_ETHER_TYPE_IPV4);
                    memcpy(eth_hdr->s_addr.addr_bytes, bs_server_mac_addrs_[cfg->bs_server_addr_idx].addr_bytes,
                        RTE_ETHER_ADDR_LEN);
                    memcpy(eth_hdr->d_addr.addr_bytes, bs_server_mac_addrs_[server_idx].addr_bytes,
                        RTE_ETHER_ADDR_LEN);

                    struct rte_ipv4_hdr* ip_h
                        = (struct rte_ipv4_hdr*)((char*)eth_hdr + sizeof(struct rte_ether_hdr));
                    ip_h->src_addr = bs_server_addrs_[cfg->bs_server_addr_idx];
                    ip_h->dst_addr = bs_server_addrs_[server_idx];
                    ip_h->next_proto_id = IPPROTO_UDP;

                    struct rte_udp_hdr* udp_h
                        = (struct rte_udp_hdr*)((char*)ip_h + sizeof(struct rte_ipv4_hdr));
                    udp_h->src_port = rte_cpu_to_be_16(cfg->encode_tx_port);
                    udp_h->dst_port = rte_cpu_to_be_16(cfg->enxode_rx_port);

                    tx_bufs[0]->pkt_len = cfg->packet_length + kPayloadOffset;
                    tx_bufs[0]->data_len = cfg->packet_length + kPayloadOffset;

                    char* payload = (char*)eth_hdr + kPayloadOffset;
                    auto* pkt = reinterpret_cast<Packet*>(payload);
                    pkt->pkt_type = Packet::PktType::kEncode;
                    pkt->frame_id = encode_frame_to_send_;
                    pkt->symbol_id = encode_symbol_to_send_;
                    pkt->ue_id = encode_ue_to_send_;
                    pkt->server_id = cfg->bs_server_addr_idx;
                    DpdkTransport::fastMemcpy(pkt->data, src_ptr,
                        cfg->get_num_sc_per_server());

                    // Send data (one OFDM symbol)
                    size_t nb_tx_new = rte_eth_tx_burst(0, tid - 1, tx_bufs, 1);
                    if (unlikely(nb_tx_new != 1)) {
                        printf("rte_eth_tx_burst() failed\n");
                        exit(0);
                    }
                }
            }
    
            encode_ue_to_send_ ++;
            if (encode_ue_to_send_ == cfg->get_num_ues_to_process()) {
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
                fprintf(stderr, "DPDK: Source addr does not match\n");
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }
            if (ip_hdr->dst_addr != bs_server_addrs_[cfg->bs_server_addr_idx]) {
                fprintf(stderr, "DPDK: Destination addr does not match (%x %x)\n", ip_hdr->dst_addr, bs_server_addrs_[cfg->bs_server_addr_idx]);
                rte_pktmbuf_free(rx_bufs[i]);
                continue;
            }

            auto* pkt = reinterpret_cast<Packet*>(eth_hdr) + kPayloadOffset;
            if (pkt->pkt_type == Packet::PktType::kEncode) {
                const size_t symbol_idx_dl = pkt->symbol_id;
                const size_t ue_id = pkt->ue_id;

                int8_t* dst_ptr
                    = cfg->get_encoded_buf(encoded_buffer_to_precode_,
                        pkt->frame_id, symbol_idx_dl, pkt->ue_id, 0) + cfg->bs_server_addr_idx * cfg->get_num_sc_per_server();
                DpdkTransport::fastMemcpy(dst_ptr, pkt->data,
                    cfg->get_num_sc_per_server());
                precode_status_->receive_encoded_data(
                    pkt->ue_id, pkt->frame_id, symbol_idx_dl);
            } else {
                printf("Received unknown packet type in demod TX/RX thread\n");
                exit(1);
            }

            rte_pktmbuf_free(rx_bufs[i]);
        }
    }
    return 0;
}

void PacketTXRX::send_beacon(int tid, size_t frame_id)
{
    // TODO: implement beacon transmission for DPDK mode
    _unused(tid);
    _unused(frame_id);
}

void* PacketTXRX::loop_tx_rx(int tid)
{
    int radio_lo = tid * cfg->nRadios / socket_thread_num;
    int radio_hi = (tid + 1) * cfg->nRadios / socket_thread_num;
    int radio_id = radio_lo;

    while (cfg->running) {
        // Receive data
        // int res = recv_relocate(tid);
        int res = recv(tid);
        if (res == 0)
            continue;

        if (++radio_id == radio_hi)
            radio_id = radio_lo;
        
    }
    return 0;
}

int PacketTXRX::recv_relocate(int tid)
{
    rte_mbuf* rx_bufs[kRxBatchSize];
    uint16_t nb_rx = rte_eth_rx_burst(0, tid, rx_bufs, kRxBatchSize);
    if (unlikely(nb_rx == 0))
        return 0;

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

        if (eth_type != RTE_ETHER_TYPE_IPV4
            or ip_hdr->next_proto_id != IPPROTO_UDP) {
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }

        if (ip_hdr->src_addr != bs_rru_addr) {
            fprintf(stderr, "DPDK relocate: Source addr does not match (%x->%x)\n", ip_hdr->src_addr, ip_hdr->dst_addr);
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }
        if (ip_hdr->dst_addr != bs_server_addrs_[cfg->bs_server_addr_idx]) {
            fprintf(stderr, "DPDK relocate: Destination addr does not match (%x %x)\n", ip_hdr->dst_addr, bs_server_addrs_[cfg->bs_server_addr_idx]);
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }

        auto* pkt = reinterpret_cast<Packet*>(reinterpret_cast<uint8_t*>(eth_hdr) + kPayloadOffset);
        if (pkt->pkt_type == Packet::PktType::kIQFromRRU) {
            char* rx_buffer = (*buffer_)[pkt->ant_id];
            const size_t rx_offset_ = (pkt->frame_id % SOCKET_BUFFER_FRAME_NUM)
                    * cfg->symbol_num_perframe
                + pkt->symbol_id;

            size_t sc_offset = Packet::kOffsetOfData
                + 2 * sizeof(unsigned short)
                    * (cfg->OFDM_DATA_START
                        + cfg->bs_server_addr_idx * cfg->get_num_sc_per_server());
            DpdkTransport::fastMemcpy(
                &rx_buffer[rx_offset_ * cfg->packet_length], pkt, Packet::kOffsetOfData);
            DpdkTransport::fastMemcpy(
                &rx_buffer[rx_offset_ * cfg->packet_length + sc_offset],
                (uint8_t*)pkt + Packet::kOffsetOfData,
                cfg->get_num_sc_per_server() * 2 * sizeof(unsigned short));

            // get the position in rx_buffer
            if (!rx_status_->add_new_packet(pkt)) {
                cfg->running = false;
            }
        } else {
            printf("Received unknown packet from rru\n");
            exit(1);
        }

        rte_pktmbuf_free(rx_bufs[i]);

        // if (kIsWorkerTimingEnabled) {
        //     if (prev_frame_id == SIZE_MAX or pkt->frame_id > prev_frame_id) {
        //         (*frame_start_)[tid][pkt->frame_id % kNumStatsFrames] = rdtsc();
        //         prev_frame_id = pkt->frame_id;
        //     }
        // }
    }
    return nb_rx;
}

int PacketTXRX::recv(int tid)
{
    rte_mbuf* rx_bufs[kRxBatchSize];
    uint16_t nb_rx = rte_eth_rx_burst(0, tid, rx_bufs, kRxBatchSize);
    if (unlikely(nb_rx == 0))
        return 0;

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

        if (eth_type != RTE_ETHER_TYPE_IPV4
            or ip_hdr->next_proto_id != IPPROTO_UDP) {
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }

        if (ip_hdr->dst_addr != bs_server_addrs_[cfg->bs_server_addr_idx]) {
            char src_mac[32], dst_mac[32];
            rte_ether_format_addr(src_mac, 32, &eth_hdr->s_addr);
            rte_ether_format_addr(dst_mac, 32, &eth_hdr->d_addr);
            fprintf(stderr, "DPDK relocate: Destination addr does not match (%x %x) (%s %s)\n", 
                ip_hdr->dst_addr, bs_server_addrs_[cfg->bs_server_addr_idx], src_mac, dst_mac);
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }

        auto* pkt = reinterpret_cast<Packet*>(reinterpret_cast<uint8_t*>(eth_hdr) + kPayloadOffset);
        if (pkt->pkt_type == Packet::PktType::kIQFromRRU) {
            char* rx_buffer = (*buffer_)[pkt->ant_id];
            const size_t rx_offset_ = (pkt->frame_id % SOCKET_BUFFER_FRAME_NUM)
                    * cfg->symbol_num_perframe
                + pkt->symbol_id;

            size_t sc_offset = Packet::kOffsetOfData
                + 2 * sizeof(unsigned short)
                    * (cfg->OFDM_DATA_START
                        + cfg->bs_server_addr_idx * cfg->get_num_sc_per_server());
            DpdkTransport::fastMemcpy(
                &rx_buffer[rx_offset_ * cfg->packet_length], pkt, Packet::kOffsetOfData);
            DpdkTransport::fastMemcpy(
                &rx_buffer[rx_offset_ * cfg->packet_length + sc_offset],
                (uint8_t*)pkt + Packet::kOffsetOfData,
                cfg->get_num_sc_per_server() * 2 * sizeof(unsigned short));

            // get the position in rx_buffer
            if (!rx_status_->add_new_packet(pkt)) {
                cfg->running = false;
            }
        } else if (pkt->pkt_type == Packet::PktType::kDemod) {
            const size_t symbol_idx_ul
                = pkt->symbol_id - cfg->pilot_symbol_num_perframe;
            const size_t sc_id = pkt->server_id * cfg->get_num_sc_per_server();

            int8_t* demod_ptr
                = cfg->get_demod_buf_to_decode(*demod_soft_buffer_to_decode_,
                    pkt->frame_id, symbol_idx_ul, pkt->ue_id, sc_id);
            DpdkTransport::fastMemcpy(demod_ptr, pkt->data,
                cfg->get_num_sc_per_server() * cfg->mod_order_bits);
            decode_status_->receive_demod_data(
                pkt->ue_id, pkt->frame_id, symbol_idx_ul);
        } else {
            printf("Received unknown packet from rru\n");
            exit(1);
        }

        rte_pktmbuf_free(rx_bufs[i]);

        // if (kIsWorkerTimingEnabled) {
        //     if (prev_frame_id == SIZE_MAX or pkt->frame_id > prev_frame_id) {
        //         (*frame_start_)[tid][pkt->frame_id % kNumStatsFrames] = rdtsc();
        //         prev_frame_id = pkt->frame_id;
        //     }
        // }
    }
    return nb_rx;
}

// TODO: check correctness of this funcion
// int PacketTXRX::dequeue_send(int tid)
// {
//     auto& c = cfg;
//     Event_data event;
//     if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
//         return -1;

//     // printf("tx queue length: %d\n", task_queue_->size_approx());
//     assert(event.event_type == EventType::kPacketTX);

//     size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
//     size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
//     size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

//     size_t data_symbol_idx_dl = cfg->get_dl_symbol_idx(frame_id, symbol_id);
//     size_t offset
//         = (c->get_total_data_symbol_idx_dl(frame_id, data_symbol_idx_dl)
//               * c->BS_ANT_NUM)
//         + ant_id;

//     if (kDebugPrintInTask) {
//         printf("In TX thread %d: Transmitted frame %zu, symbol %zu, "
//                "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
//             tid, frame_id, symbol_id, ant_id, gen_tag_t(event.tags[0])._tag,
//             offset, message_queue_->size_approx());
//     }

//     char* cur_buffer_ptr = tx_buffer_ + offset * c->packet_length;
//     auto* pkt = (Packet*)cur_buffer_ptr;
//     new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

//     struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
//     tx_bufs[0] = rte_pktmbuf_alloc(mbuf_pool);
//     struct rte_ether_hdr* eth_hdr
//         = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
//     eth_hdr->ether_type = rte_be_to_cpu_16(RTE_ETHER_TYPE_IPV4);

//     struct rte_ipv4_hdr* ip_h
//         = (struct rte_ipv4_hdr*)((char*)eth_hdr + sizeof(struct rte_ether_hdr));
//     ip_h->src_addr = bs_server_addr;
//     ip_h->dst_addr = bs_rru_addr;
//     ip_h->next_proto_id = IPPROTO_UDP;

//     struct rte_udp_hdr* udp_h
//         = (struct rte_udp_hdr*)((char*)ip_h + sizeof(struct rte_ipv4_hdr));
//     udp_h->src_port = rte_cpu_to_be_16(cfg->bs_server_port + tid);
//     udp_h->dst_port = rte_cpu_to_be_16(cfg->bs_rru_port + tid);

//     tx_bufs[0]->pkt_len = cfg->packet_length + kPayloadOffset;
//     tx_bufs[0]->data_len = cfg->packet_length + kPayloadOffset;
//     char* payload = (char*)eth_hdr + kPayloadOffset;
//     DpdkTransport::fastMemcpy(payload, (char*)pkt, cfg->packet_length);

//     // Send data (one OFDM symbol)
//     size_t nb_tx_new = rte_eth_tx_burst(0, tid, tx_bufs, 1);
//     if (unlikely(nb_tx_new != 1)) {
//         printf("rte_eth_tx_burst() failed\n");
//         exit(0);
//     }
//     rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
//                   Event_data(EventType::kPacketTX, event.tags[0])),
//         "Socket message enqueue failed\n");
//     return 1;
// }
