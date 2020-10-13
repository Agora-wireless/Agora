/**
 * @file txrx_DPDK.cpp
 * @brief Implementation of PacketTXRX datapath functions for communicating 
 * with DPDK
 */

#include "txrx.hpp"

static constexpr bool kDebugDPDK = false;

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset)
    : cfg(cfg)
    , core_offset(core_offset)
    , socket_thread_num(cfg->socket_thread_num)
{
    DpdkTransport::dpdk_init(core_offset - 1, socket_thread_num);
    mbuf_pool = DpdkTransport::create_mempool();

    const uint16_t port_id = 0; // The DPDK port ID
    if (DpdkTransport::nic_init(port_id, mbuf_pool, socket_thread_num) != 0)
        rte_exit(EXIT_FAILURE, "Cannot init port %u\n", port_id);

    int ret = inet_pton(AF_INET, cfg->bs_rru_addr.c_str(), &bs_rru_addr);
    rt_assert(ret == 1, "Invalid sender IP address");
    ret = inet_pton(AF_INET, cfg->bs_server_addr.c_str(), &bs_server_addr);
    rt_assert(ret == 1, "Invalid server IP address");

    for (size_t i = 0; i < socket_thread_num; i++) {
        uint16_t src_port = rte_cpu_to_be_16(cfg->bs_rru_port + i);
        uint16_t dst_port = rte_cpu_to_be_16(cfg->bs_server_port + i);

        printf("Adding steering rule for src IP %s, dest IP %s, src port: %zu, "
               "dst port: %zu, queue: %zu\n",
            cfg->bs_rru_addr.c_str(), cfg->bs_server_addr.c_str(),
            cfg->bs_rru_port + i, cfg->bs_server_port + i, i);
        DpdkTransport::install_flow_rule(
            port_id, i, bs_rru_addr, bs_server_addr, src_port, dst_port);
    }

    printf("Number of DPDK cores: %d\n", rte_lcore_count());
}

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset,
    moodycamel::ConcurrentQueue<Event_data>* queue_message,
    moodycamel::ConcurrentQueue<Event_data>* queue_task,
    moodycamel::ProducerToken** rx_ptoks, moodycamel::ProducerToken** tx_ptoks)
    : PacketTXRX(cfg, core_offset)
{
    message_queue_ = queue_message;
    task_queue_ = queue_task;
    rx_ptoks_ = rx_ptoks;
    tx_ptoks_ = tx_ptoks;
}

PacketTXRX::~PacketTXRX() { rte_mempool_free(mbuf_pool); }

bool PacketTXRX::startTXRX(Table<char>& buffer, Table<int>& buffer_status,
    size_t packet_num_in_buffer, Table<size_t>& frame_start, char* tx_buffer)
{
    buffer_ = &buffer;
    buffer_status_ = &buffer_status;
    frame_start_ = &frame_start;

    packet_num_in_buffer_ = packet_num_in_buffer;
    tx_buffer_ = tx_buffer;

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
        }
        worker_id++;
    }
    return true;
}

void PacketTXRX::send_beacon(int tid, size_t frame_id)
{
    // TODO: implement beacon transmission for DPDK mode
    _unused(tid);
    _unused(frame_id);
}

void* PacketTXRX::loop_tx_rx(int tid)
{
    size_t rx_offset = 0;
    size_t prev_frame_id = SIZE_MAX;

    while (cfg->running) {
        dpdk_recv(tid, prev_frame_id, rx_offset);
        dequeue_send(tid);
    }
    return 0;
}

uint16_t PacketTXRX::dpdk_recv(
    int tid, size_t& prev_frame_id, size_t& rx_offset)
{
    rte_mbuf* rx_bufs[kRxBatchSize];
    uint16_t nb_rx = rte_eth_rx_burst(0, tid, rx_bufs, kRxBatchSize);
    if (unlikely(nb_rx == 0))
        return 0;

    for (size_t i = 0; i < nb_rx; i++) {
        // If the RX buffer is full, it means that the base station processing
        // hasn't kept up, so exit.
        if ((*buffer_status_)[tid][rx_offset] == 1) {
            printf(
                "TXRX thread %d rx_buffer full, offset: %zu\n", tid, rx_offset);
            cfg->running = false;
            return 0;
        }

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
            fprintf(stderr, "DPDK: Source addr does not match\n");
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }
        if (ip_hdr->dst_addr != bs_server_addr) {
            fprintf(stderr, "DPDK: Destination addr does not match\n");
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }

        auto* payload = reinterpret_cast<uint8_t*>(eth_hdr) + kPayloadOffset;
        auto* pkt = reinterpret_cast<Packet*>(
            &(*buffer_)[tid][rx_offset * cfg->packet_length]);
        DpdkTransport::fastMemcpy(
            reinterpret_cast<uint8_t*>(pkt), payload, cfg->packet_length);

        rte_pktmbuf_free(rx_bufs[i]);

        if (kIsWorkerTimingEnabled) {
            if (prev_frame_id == SIZE_MAX or pkt->frame_id > prev_frame_id) {
                (*frame_start_)[tid][pkt->frame_id % kNumStatsFrames] = rdtsc();
                prev_frame_id = pkt->frame_id;
            }
        }

        if (!message_queue_->enqueue(*rx_ptoks_[tid],
                Event_data(
                    EventType::kPacketRX, rx_tag_t(tid, rx_offset)._tag))) {
            printf("Failed to enqueue socket message\n");
            exit(-1);
        }

        rx_offset = (rx_offset + 1) % packet_num_in_buffer_;
    }
    return nb_rx;
}

// TODO: check correctness of this funcion
int PacketTXRX::dequeue_send(int tid)
{
    auto& c = cfg;
    Event_data event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], event))
        return -1;

    // printf("tx queue length: %d\n", task_queue_->size_approx());
    assert(event.event_type == EventType::kPacketTX);

    size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
    size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

    size_t data_symbol_idx_dl = cfg->get_dl_symbol_idx(frame_id, symbol_id);
    size_t offset
        = (c->get_total_data_symbol_idx_dl(frame_id, data_symbol_idx_dl)
              * c->BS_ANT_NUM)
        + ant_id;

    if (kDebugPrintInTask) {
        printf("In TX thread %d: Transmitted frame %zu, symbol %zu, "
               "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, symbol_id, ant_id, gen_tag_t(event.tags[0])._tag,
            offset, message_queue_->size_approx());
    }

    char* cur_buffer_ptr = tx_buffer_ + offset * c->packet_length;
    auto* pkt = (Packet*)cur_buffer_ptr;
    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

    struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
    tx_bufs[0] = rte_pktmbuf_alloc(mbuf_pool);
    struct rte_ether_hdr* eth_hdr
        = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
    eth_hdr->ether_type = rte_be_to_cpu_16(RTE_ETHER_TYPE_IPV4);

    struct rte_ipv4_hdr* ip_h
        = (struct rte_ipv4_hdr*)((char*)eth_hdr + sizeof(struct rte_ether_hdr));
    ip_h->src_addr = bs_server_addr;
    ip_h->dst_addr = bs_rru_addr;
    ip_h->next_proto_id = IPPROTO_UDP;

    struct rte_udp_hdr* udp_h
        = (struct rte_udp_hdr*)((char*)ip_h + sizeof(struct rte_ipv4_hdr));
    udp_h->src_port = rte_cpu_to_be_16(cfg->bs_server_port + tid);
    udp_h->dst_port = rte_cpu_to_be_16(cfg->bs_rru_port + tid);

    tx_bufs[0]->pkt_len = cfg->packet_length + kPayloadOffset;
    tx_bufs[0]->data_len = cfg->packet_length + kPayloadOffset;
    char* payload = (char*)eth_hdr + kPayloadOffset;
    DpdkTransport::fastMemcpy(payload, (char*)pkt, cfg->packet_length);

    // Send data (one OFDM symbol)
    size_t nb_tx_new = rte_eth_tx_burst(0, tid, tx_bufs, 1);
    if (unlikely(nb_tx_new != 1)) {
        printf("rte_eth_tx_burst() failed\n");
        exit(0);
    }
    rt_assert(message_queue_->enqueue(*rx_ptoks_[tid],
                  Event_data(EventType::kPacketTX, event.tags[0])),
        "Socket message enqueue failed\n");
    return 1;
}
