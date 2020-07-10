/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "txrx.hpp"

static constexpr bool kDebugDPDK = false;

PacketTXRX::PacketTXRX(Config* cfg, size_t core_offset)
    : cfg(cfg)
    , core_offset(core_offset)
    , socket_thread_num(cfg->socket_thread_num)
{
    // Use one core to allocate launch threads, and use one core to run each
    // socket thread
    std::string core_list = std::to_string(core_offset - 1) + "-"
        + std::to_string(core_offset - 1 + socket_thread_num);

    // n: channels, m: maximum memory in megabytes
    const char* rte_argv[] = { "txrx", "-l", core_list.c_str(), NULL };
    int rte_argc = static_cast<int>(sizeof(rte_argv) / sizeof(rte_argv[0])) - 1;

    printf("rte_eal_init argv: ");
    for (int i = 0; i < rte_argc; i++) {
        printf("%s, ", rte_argv[i]);
    }
    printf("\n");
    // Initialize DPDK environment
    int ret = rte_eal_init(rte_argc, const_cast<char**>(rte_argv));
    rt_assert(ret >= 0, "Failed to initialize DPDK");

    unsigned int nb_ports = rte_eth_dev_count_avail();
    printf("Number of ports: %d, socket: %d\n", nb_ports, rte_socket_id());

    size_t mbuf_size = JUMBO_FRAME_MAX_SIZE + MBUF_CACHE_SIZE;
    mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL", NUM_MBUFS * nb_ports,
        MBUF_CACHE_SIZE, 0, mbuf_size, rte_socket_id());

    rt_assert(mbuf_pool != NULL, "Cannot create mbuf pool");

    uint16_t portid = 0;

    if (DpdkTransport::nic_init(portid, mbuf_pool, socket_thread_num) != 0)
        rte_exit(EXIT_FAILURE, "Cannot init port %u\n", portid);

    ret = inet_pton(AF_INET, cfg->sender_addr.c_str(), &sender_addr);
    rt_assert(ret == 1, "Invalid sender IP address");
    ret = inet_pton(AF_INET, cfg->server_addr.c_str(), &server_addr);
    rt_assert(ret == 1, "Invalid server IP address");

    rte_flow_error error;
    rte_flow* flow;
    /* create flow for send packet with */
    for (size_t i = 0; i < socket_thread_num; i++) {
        uint16_t src_port = rte_cpu_to_be_16(cfg->ue_tx_port + i);
        uint16_t dst_port = rte_cpu_to_be_16(cfg->bs_port + i);
        flow = DpdkTransport::generate_ipv4_flow(0, i, sender_addr, FULL_MASK,
            server_addr, FULL_MASK, src_port, 0xffff, dst_port, 0xffff, &error);
        printf("Add rule for src port: %d, dst port: %d, queue: %zu\n",
            src_port, dst_port, i);
        if (!flow)
            rte_exit(
                EXIT_FAILURE, "Error in creating flow: %s\n", error.message);
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

    printf("create TXRX threads\n");

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
                    pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopTXRX>,
                context, lcore_id);
            printf("DPDK TXRX thread %zu: pinned to core %d\n", worker_id,
                lcore_id);
        }
        worker_id++;
    }
    return true;
}

void* PacketTXRX::loopTXRX(int tid)
{
    size_t rx_offset = 0;
    int prev_frame_id = -1;

    while (cfg->running) {
        if (-1 != dequeue_send(tid))
            continue;
        uint16_t nb_rx = dpdk_recv_enqueue(tid, prev_frame_id, rx_offset);
        if (nb_rx == 0)
            continue;
    }
    return 0;
}

uint16_t PacketTXRX::dpdk_recv_enqueue(
    int tid, int& prev_frame_id, size_t& rx_offset)
{
    char* rx_buffer = (*buffer_)[tid];
    int* rx_buffer_status = (*buffer_status_)[tid];
    size_t* rx_frame_start = (*frame_start_)[tid];
    // use token to speed up
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    struct rte_mbuf* rx_bufs[kRxBatchSize] __attribute__((aligned(64)));
    uint16_t nb_rx = rte_eth_rx_burst(0, tid, rx_bufs, kRxBatchSize);
    if (unlikely(nb_rx == 0))
        return 0;

    for (int i = 0; i < nb_rx; i++) {
        // if buffer is full, exit
        if (rx_buffer_status[rx_offset] == 1) {
            printf("Receive thread %d rx_buffer full, offset: %zu\n", tid,
                rx_offset);
            cfg->running = false;
            return 0;
        }

        struct rte_mbuf* dpdk_pkt = rx_bufs[i];
        /* parse the header */
        struct rte_ether_hdr* eth_hdr
            = rte_pktmbuf_mtod(dpdk_pkt, struct rte_ether_hdr*);
        uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);
        struct rte_ipv4_hdr* ip_h = (struct rte_ipv4_hdr*)((char*)eth_hdr
            + sizeof(struct rte_ether_hdr));
        if (kDebugDPDK) {
            struct rte_udp_hdr* udp_h = (struct rte_udp_hdr*)((char*)ip_h
                + sizeof(struct rte_ipv4_hdr));
            DpdkTransport::print_pkt(ip_h->src_addr, ip_h->dst_addr,
                udp_h->src_port, udp_h->dst_port, dpdk_pkt->data_len, tid);
            printf("pkt_len: %d, nb_segs: %d, Header type: %d, IPV4: %d\n",
                dpdk_pkt->pkt_len, dpdk_pkt->nb_segs, eth_type,
                RTE_ETHER_TYPE_IPV4);
            printf("UDP: %d, %d\n", ip_h->next_proto_id, IPPROTO_UDP);
        }

        if (eth_type != RTE_ETHER_TYPE_IPV4
            or ip_h->next_proto_id != IPPROTO_UDP) {
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }

        if (ip_h->src_addr != sender_addr) {
            printf("Source addr does not match\n");
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }
        if (ip_h->dst_addr != server_addr) {
            printf("Destination addr does not match\n");
            rte_pktmbuf_free(rx_bufs[i]);
            continue;
        }

        char* payload = (char*)eth_hdr + kPayloadOffset;

        struct Packet* pkt
            = (struct Packet*)&rx_buffer[rx_offset * cfg->packet_length];
        DpdkTransport::fastMemcpy((char*)pkt, payload, cfg->packet_length);
        // rte_memcpy((char*)pkt, payload, c->packet_length);
        rte_pktmbuf_free(rx_bufs[i]);

        if (kIsWorkerTimingEnabled) {
            int frame_id = pkt->frame_id;
            if (frame_id > prev_frame_id) {
                rx_frame_start[frame_id] = rdtsc();
                prev_frame_id = frame_id;
            }
        }
        // printf("thread %d received packet frame %u, symbol %u, ant %u\n",
        //     tid, pkt->frame_id, pkt->symbol_id, pkt->ant_id);

        // get the position in rx_buffer
        // move ptr & set status to full
        // rx_buffer_status[rx_offset] = 1;

        // Push kPacketRX event into the queue.
        Event_data rx_message(
            EventType::kPacketRX, rx_tag_t(tid, rx_offset)._tag);
        if (!message_queue_->enqueue(*local_ptok, rx_message)) {
            printf("socket message enqueue failed\n");
            exit(0);
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
    size_t data_symbol_idx = gen_tag_t(event.tags[0]).symbol_id;

    size_t offset = (c->get_total_data_symbol_idx(frame_id, data_symbol_idx)
                        * c->BS_ANT_NUM)
        + ant_id;

    if (kDebugPrintInTask) {
        printf("In TX thread %d: Transmitted frame %zu, symbol %zu, "
               "ant %zu, tag %zu, offset: %zu, msg_queue_length: %zu\n",
            tid, frame_id, data_symbol_idx, ant_id,
            gen_tag_t(event.tags[0])._tag, offset,
            message_queue_->size_approx());
    }

    size_t socket_symbol_offset = offset
        % (SOCKET_BUFFER_FRAME_NUM * c->data_symbol_num_perframe
              * c->BS_ANT_NUM);
    char* cur_buffer_ptr = tx_buffer_ + socket_symbol_offset * c->packet_length;
    auto* pkt = (Packet*)cur_buffer_ptr;
    new (pkt) Packet(frame_id, data_symbol_idx, 0 /* cell_id */, ant_id);

    struct rte_mbuf* tx_bufs[kTxBatchSize] __attribute__((aligned(64)));
    tx_bufs[0] = rte_pktmbuf_alloc(mbuf_pool);
    struct rte_ether_hdr* eth_hdr
        = rte_pktmbuf_mtod(tx_bufs[0], struct rte_ether_hdr*);
    eth_hdr->ether_type = rte_be_to_cpu_16(RTE_ETHER_TYPE_IPV4);

    struct rte_ipv4_hdr* ip_h
        = (struct rte_ipv4_hdr*)((char*)eth_hdr + sizeof(struct rte_ether_hdr));
    ip_h->src_addr = server_addr;
    ip_h->dst_addr = sender_addr;
    ip_h->next_proto_id = IPPROTO_UDP;

    struct rte_udp_hdr* udp_h
        = (struct rte_udp_hdr*)((char*)ip_h + sizeof(struct rte_ipv4_hdr));
    udp_h->src_port = rte_cpu_to_be_16(cfg->bs_port + tid);
    udp_h->dst_port = rte_cpu_to_be_16(cfg->ue_rx_port + tid);

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
