/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */

#include "txrx.hpp"

inline const struct rte_eth_conf port_conf_default()
{
    struct rte_eth_conf rte = rte_eth_conf();
    // rte.rxmode.max_rx_pkt_len = ETHER_MAX_LEN;
    rte.rxmode.max_rx_pkt_len = MAX_JUMBO_FRAME_SIZE;
    return rte;
}

static struct rte_flow* generate_ipv4_flow(uint16_t port_id, uint16_t rx_q,
    uint32_t src_ip, uint32_t src_mask, uint32_t dest_ip, uint32_t dest_mask,
    uint16_t src_port, uint16_t src_port_mask, uint16_t dst_port,
    uint16_t dst_port_mask, struct rte_flow_error* error);

PacketTXRX::PacketTXRX(Config* cfg, int COMM_THREAD_NUM, int in_core_offset)
{
    socket_ = new int[COMM_THREAD_NUM];
    config_ = cfg;
    comm_thread_num_ = COMM_THREAD_NUM;

    core_id_ = in_core_offset;
    tx_core_id_ = in_core_offset + COMM_THREAD_NUM;

    BS_ANT_NUM = config_->BS_ANT_NUM;
    UE_NUM = config_->UE_NUM;
    OFDM_CA_NUM = config_->OFDM_CA_NUM;
    OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    symbol_num_perframe = config_->symbol_num_perframe;
    data_symbol_num_perframe = config_->data_symbol_num_perframe;
    ul_data_symbol_num_perframe = config_->ul_data_symbol_num_perframe;
    dl_data_symbol_num_perframe = config_->dl_data_symbol_num_perframe;
    packet_length = config_->packet_length;

    // frameID = new int[TASK_BUFFER_FRAME_NUM];
    /* initialize random seed: */
    srand(time(NULL));
    rx_context = new EventHandlerContext<PacketTXRX>[comm_thread_num_];
    tx_context = new EventHandlerContext<PacketTXRX>[comm_thread_num_];

    std::string core_list = std::to_string(core_id_) + "-"
        + std::to_string(core_id_ + comm_thread_num_ + comm_thread_num_);
    int argc = 5;
    char* argv[] = { (char*)"txrx", (char*)"-l", &core_list[0u], (char*)"-n",
        (char*)"8", NULL };
    // Initialize DPDK environment
    for (int i = 0; i < argc; i++) {
        printf("%s\n", argv[i]);
    }
    int ret = rte_eal_init(argc, argv);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");

    struct rte_mempool* mbuf_pool;
    unsigned int nb_ports = rte_eth_dev_count_avail();
    printf("Number of ports: %d, socket: %d\n", nb_ports, rte_socket_id());

    // Initialize memory pool
    // mbuf_pool = rte_mempool_create("MBUF pool",
    //                 NUM_MBUFS * nb_ports,
    //                 MBUF_SIZE + 128*4, MBUF_CACHE_SIZE, 64, NULL, NULL, NULL,
    //                 NULL,rte_socket_id(), 0);

    // rte_pktmbuf_pool_init(mp, &mbp_priv);

    // rte_mempool_populate_default(mp);
    mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL", NUM_MBUFS * nb_ports,
        MBUF_CACHE_SIZE, 0, MBUF_SIZE, rte_socket_id());

    if (mbuf_pool == NULL)
        rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

    uint16_t portid;
    RTE_ETH_FOREACH_DEV(portid)
    if (nic_dpdk_init(portid, mbuf_pool) != 0)
        rte_exit(EXIT_FAILURE, "Cannot init port %" PRIu16 "\n", portid);

    unsigned int nb_lcores = rte_lcore_count();
    uint16_t mtu_size = 0;
    rte_eth_dev_get_mtu(0, &mtu_size);
    printf("Number of DPDK cores: %d, MTU: %d\n", nb_lcores, mtu_size);

    src_addr = rte_cpu_to_be_32(IPv4(10, 0, 0, 2));
    dst_addr = rte_cpu_to_be_32(IPv4(10, 0, 0, 1));

    struct rte_flow_error error;
    struct rte_flow* flow;
    /* create flow for send packet with */
    for (int i = 0; i < comm_thread_num_; i++) {
        uint16_t src_port = rte_cpu_to_be_16(src_port_start + i);
        uint16_t dst_port = rte_cpu_to_be_16(dst_port_start + i);
        flow = generate_ipv4_flow(0, i, src_addr, FULL_MASK, dst_addr,
            FULL_MASK, src_port, 0xffff, dst_port, 0xffff, &error);
        // printf("Add rule for src port: %d, dst port: %d, queue: %d\n",
        // src_port, dst_port, i);
        if (!flow) {
            printf("Flow can't be created %d message: %s\n", error.type,
                error.message ? error.message : "(no stated reason)");
            rte_exit(EXIT_FAILURE, "error in creating flow");
        }
    }
}

PacketTXRX::PacketTXRX(Config* cfg, int COMM_THREAD_NUM, int in_core_offset,
    moodycamel::ConcurrentQueue<Event_data>* in_queue_message,
    moodycamel::ConcurrentQueue<Event_data>* in_queue_task,
    moodycamel::ProducerToken** in_rx_ptoks,
    moodycamel::ProducerToken** in_tx_ptoks)
    : PacketTXRX(cfg, COMM_THREAD_NUM, in_core_offset)
{
    message_queue_ = in_queue_message;
    task_queue_ = in_queue_task;
    rx_ptoks_ = in_rx_ptoks;
    tx_ptoks_ = in_tx_ptoks;
}

PacketTXRX::~PacketTXRX()
{
    delete[] socket_;
    delete[] tx_context;
    delete[] rx_context;
}

int PacketTXRX::nic_dpdk_init(uint16_t port, struct rte_mempool* mbuf_pool)
{
    struct rte_eth_conf port_conf = port_conf_default();
    const uint16_t rxRings = comm_thread_num_, txRings = 2 * comm_thread_num_;
    int retval;
    uint16_t q;
    uint16_t nb_rxd = RX_RING_SIZE;
    uint16_t nb_txd = TX_RING_SIZE;

    struct rte_eth_dev_info dev_info;
    struct rte_eth_rxconf rxconf;
    struct rte_eth_txconf txconf;

    if (rte_eth_dev_count() < port)
        rte_exit(EXIT_FAILURE, "Not Enough NICs\n");

    if (!rte_eth_dev_is_valid_port(port))
        rte_exit(EXIT_FAILURE, "NIC ID is invalid\n");

    rte_eth_dev_set_mtu(port, 9000);
    uint16_t mtu_size = 0;
    rte_eth_dev_get_mtu(port, &mtu_size);
    printf("MTU: %d\n", mtu_size);

    int promiscuous_en = rte_eth_promiscuous_get(port);
    printf("Promiscuous mode: %d\n", promiscuous_en);
    rte_eth_promiscuous_enable(port);
    promiscuous_en = rte_eth_promiscuous_get(port);
    printf("Promiscuous mode: %d\n", promiscuous_en);

    rte_eth_dev_info_get(port, &dev_info);
    if (dev_info.tx_offload_capa & DEV_TX_OFFLOAD_MBUF_FAST_FREE)
        port_conf.txmode.offloads |= DEV_TX_OFFLOAD_MBUF_FAST_FREE;

    port_conf.rxmode.max_rx_pkt_len = MAX_JUMBO_FRAME_SIZE;
    port_conf.rxmode.offloads |= DEV_RX_OFFLOAD_JUMBO_FRAME;

    retval = rte_eth_dev_configure(port, rxRings, txRings, &port_conf);
    if (retval != 0)
        return retval;
    printf("Max packet length: %d, dev max: %d\n",
        port_conf.rxmode.max_rx_pkt_len, dev_info.max_rx_pktlen);
    retval = rte_eth_dev_adjust_nb_rx_tx_desc(port, &nb_rxd, &nb_txd);
    if (retval != 0)
        return retval;

    rxconf = dev_info.default_rxconf;
    for (q = 0; q < rxRings; q++) {
        retval = rte_eth_rx_queue_setup(
            port, q, nb_rxd, rte_eth_dev_socket_id(port), &rxconf, mbuf_pool);
        if (retval < 0)
            return retval;
    }

    txconf = dev_info.default_txconf;
    txconf.offloads = port_conf.txmode.offloads;

    for (q = 0; q < txRings; q++) {
        retval = rte_eth_tx_queue_setup(
            port, q, nb_txd, rte_eth_dev_socket_id(port), &txconf);
        if (retval < 0)
            return retval;
    }

    retval = rte_eth_dev_start(port);
    if (retval < 0)
        return retval;

    struct ether_addr addr;
    rte_eth_macaddr_get(port, &addr);
    printf("NIC %u MAC: %02" PRIx8 " %02" PRIx8 " %02" PRIx8 " %02" PRIx8
           " %02" PRIx8 " %02" PRIx8 " \n",
        port, addr.addr_bytes[0], addr.addr_bytes[1], addr.addr_bytes[2],
        addr.addr_bytes[3], addr.addr_bytes[4], addr.addr_bytes[5]);
    server_eth_addr = addr;

    struct rte_eth_link link;
    rte_eth_link_get_nowait(port, &link);
    while (!link.link_status) {
        printf("Waiting for link up on NIC %" PRIu16 "\n", port);
        sleep(1);
        rte_eth_link_get_nowait(port, &link);
    }
    if (!link.link_status) {
        printf("Link down on NIC %" PRIx16 "\n", port);
        return 0;
    }

    return 0;
}

bool PacketTXRX::startTXRX(char** in_buffer, int** in_buffer_status,
    int in_buffer_frame_num, long long in_buffer_length,
    size_t** in_frame_start)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == packet_length * buffer_frame_num_); // should
    // be integer
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer; // for save data
    buffer_status_ = in_buffer_status; // for save status
    frame_start_ = in_frame_start;
    tx_buffer_ = in_tx_buffer; // for save data
    tx_buffer_status_ = in_tx_buffer_status; // for save status
    tx_buffer_frame_num_ = in_tx_buffer_frame_num;
    // assert(in_tx_buffer_length == packet_length * buffer_frame_num_); //
    // should be integer
    tx_buffer_length_ = in_tx_buffer_length;

    // new thread
    // pin_to_core_with_offset(RX, core_id_, 0);

    std::vector<pthread_t> created_threads;

    if (config_->dl_data_symbol_num_perframe == 0) {
        printf("create RX threads\n");

        unsigned int nb_lcores = rte_lcore_count();
        printf("Number of DPDK cores: %d\n", nb_lcores);
        unsigned int lcore_id;
        int worker_id = 0;
        // Launch specific task to cores
        RTE_LCORE_FOREACH_SLAVE(lcore_id)
        {
            // launch communication and task thread onto specific core
            if (worker_id < comm_thread_num_) {
                rx_context[worker_id].obj_ptr = this;
                rx_context[worker_id].id = worker_id;
                rte_eal_remote_launch((lcore_function_t*)loopRecv_DPDK,
                    &rx_context[worker_id], lcore_id);
                printf(
                    "RX: launched thread %d on core %d\n", worker_id, lcore_id);
            }
            worker_id++;
        }
    } else {
        printf("create TX or TXRX threads\n");

        unsigned int lcore_id;
        int worker_id = 0;
        int thread_id;
        RTE_LCORE_FOREACH_SLAVE(lcore_id)
        {
            // launch communication and task thread onto specific core
            if (worker_id >= comm_thread_num_) {
                thread_id = worker_id - comm_thread_num_;
                tx_context[thread_id].obj_ptr = this;
                tx_context[thread_id].id = thread_id;
                rte_eal_remote_launch((lcore_function_t*)loopSend,
                    &tx_context[thread_id], lcore_id);
                printf(
                    "TX: launched thread %d on core %d\n", thread_id, lcore_id);
            }
            worker_id++;
        }
    }
    return true;
}

static void fastMemcpy(void* pvDest, void* pvSrc, size_t nBytes)
{
    // printf("pvDest: 0x%lx, pvSrc: 0x%lx, Dest: %lx, Src,
    // %lx\n",intptr_t(pvDest), intptr_t(pvSrc), (intptr_t(pvDest) & 31),
    // (intptr_t(pvSrc) & 31) ); assert(nBytes % 32 == 0);
    // assert((intptr_t(pvDest) & 31) == 0);
    // assert((intptr_t(pvSrc) & 31) == 0);
    const __m256i* pSrc = reinterpret_cast<const __m256i*>(pvSrc);
    __m256i* pDest = reinterpret_cast<__m256i*>(pvDest);
    int64_t nVects = nBytes / sizeof(*pSrc);
    for (; nVects > 0; nVects--, pSrc++, pDest++) {
        const __m256i loaded = _mm256_stream_load_si256(pSrc);
        _mm256_stream_si256(pDest, loaded);
    }
    _mm_sfence();
}

static void print_pkt(int src_ip, int dst_ip, uint16_t src_port,
    uint16_t dst_port, int len, int tid)
{
    uint8_t b[12];
    uint16_t sp, dp;

    b[0] = src_ip & 0xFF;
    b[1] = (src_ip >> 8) & 0xFF;
    b[2] = (src_ip >> 16) & 0xFF;
    b[3] = (src_ip >> 24) & 0xFF;
    b[4] = src_port & 0xFF;
    b[5] = (src_port >> 8) & 0xFF;
    sp = ((b[4] << 8) & 0xFF00) | (b[5] & 0x00FF);
    b[6] = dst_ip & 0xFF;
    b[7] = (dst_ip >> 8) & 0xFF;
    b[8] = (dst_ip >> 16) & 0xFF;
    b[9] = (dst_ip >> 24) & 0xFF;
    b[10] = dst_port & 0xFF;
    b[11] = (dst_port >> 8) & 0xFF;
    dp = ((b[10] << 8) & 0xFF00) | (b[11] & 0x00FF);
    printf("In RX thread %d: rx: %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u (%d bytes)\n",
        tid, b[0], b[1], b[2], b[3], sp, b[6], b[7], b[8], b[9], dp, len);
}

int PacketTXRX::process_arp(
    struct rte_mbuf* mbuf, struct ether_hdr* eth, int len, int tid)
{
    printf("Processing ARP request\n");
    struct arp_hdr* ah = (struct arp_hdr*)((unsigned char*)eth + ETHER_HDR_LEN);
    // recv_arp_pkts++;
    if (len < (int)(sizeof(struct ether_hdr) + sizeof(struct arp_hdr))) {
        printf("len=%d is too small for arp packet\n", len);
        return 0;
    }
    if (rte_cpu_to_be_16(ah->arp_op) != ARP_OP_REQUEST) {
        printf("Not ARP Request\n");
        return 0;
    }

    if (dst_addr == ah->arp_data.arp_tip) {
        memcpy((unsigned char*)&eth->d_addr, (unsigned char*)&eth->s_addr, 6);
        memcpy(
            (unsigned char*)&eth->s_addr, (unsigned char*)&server_eth_addr, 6);

        ah->arp_op = rte_cpu_to_be_16(ARP_OP_REPLY);
        ah->arp_data.arp_tha = ah->arp_data.arp_sha;
        memcpy((unsigned char*)&ah->arp_data.arp_sha,
            (unsigned char*)&server_eth_addr, 6);
        ah->arp_data.arp_tip = ah->arp_data.arp_sip;
        ah->arp_data.arp_sip = dst_addr;
        if (likely(1 == rte_eth_tx_burst(0, tid, &mbuf, 1))) {
            return 1;
        }
        printf("Reply ARP\n");
    }
    return 0;
}

void* PacketTXRX::loopRecv_DPDK(void* in_context)
{
    // get the pointer of class & tid
    PacketTXRX* obj_ptr = ((PacketTXRXContext*)in_context)->ptr;
    int tid = ((PacketTXRXContext*)in_context)->tid;
    printf("packet receiver thread %d start\n", tid);
    // get pointer of message queue
    moodycamel::ConcurrentQueue<Event_data>* message_queue_
        = obj_ptr->message_queue_;
    Config* config_ = obj_ptr->config_;
    int core_id = config_->core_id_;
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int UE_NUM = config_->UE_NUM;
    int OFDM_CA_NUM = config_->OFDM_CA_NUM;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int symbol_num_perframe = config_->symbol_num_perframe;
    int data_symbol_num_perframe = config_->data_symbol_num_perframe;
    int ul_data_symbol_num_perframe = config_->ul_data_symbol_num_perframe;
    int dl_data_symbol_num_perframe = config_->dl_data_symbol_num_perframe;
    int packet_length = config_->packet_length;

    uint16_t nic;
    struct rte_mbuf* bufs[BURST_SIZE * 2] __attribute__((aligned(64)));
    struct udp_hdr* udp_h;
    struct ipv4_hdr* ip_h;
    uint16_t dst_port = rte_cpu_to_be_16((obj_ptr->dst_port_start + tid));
    uint16_t src_port = rte_cpu_to_be_16((obj_ptr->src_port_start + tid));

    uint16_t mtu_size;
    rte_eth_dev_get_mtu(0, &mtu_size);
    printf("MTU: %d\n", mtu_size);

    // use token to speed up
    // moodycamel::ProducerToken local_ptok(*message_queue_);
    moodycamel::ProducerToken* local_ptok = obj_ptr->rx_ptoks_[tid];

    char* buffer = obj_ptr->buffer_[tid];
    int* buffer_status = obj_ptr->buffer_status_[tid];
    long long buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;
    size_t* frame_start = obj_ptr->frame_start_[tid];

    char* cur_ptr_buffer = buffer;
    int* cur_ptr_buffer_status = buffer_status;
    // loop recv
    // socklen_t addrlen = sizeof(obj_ptr->servaddr_[tid]);
    int offset = 0;
    int packet_num = 0;
    auto begin = std::chrono::system_clock::now();

    int ret = 0;
    int max_symbol_id = config_->dl_data_symbol_num_perframe > 0
        ? UE_NUM
        : symbol_num_perframe;
    int prev_frame_id = -1;
    int packet_num_per_frame = 0;
    double start_time = get_time();

    // printf("Rx thread %d: on core %d\n", tid, sched_getcpu());

    while (true) {

        uint16_t nb_rx = rte_eth_rx_burst(0, tid, bufs, BURST_SIZE);
        // printf("Thread %d receives %d packets\n", tid, nb_rx);
        if (unlikely(nb_rx == 0)) {
            continue;
        }

        // printf("Thread %d receives %d packets\n", tid, nb_rx);
        for (int i = 0; i < nb_rx; i++) {
            // if buffer is full, exit
            if (cur_ptr_buffer_status[0] == 1) {
                printf(
                    "Receive thread %d buffer full, offset: %d\n", tid, offset);
                exit(0);
            }
            // struct rte_eth_stats eth_stats;
            // rte_eth_stats_get(0, &eth_stats);
            // printf("RX thread %d: total number of packets received %llu,
            // dropped rx full %llu and rest= %llu, %llu, %llu\n",
            //     tid, eth_stats.ipackets, eth_stats.imissed,
            //     eth_stats.ierrors, eth_stats.rx_nombuf,
            //     eth_stats.q_ipackets[tid]);

            int len = rte_pktmbuf_data_len(bufs[i]);
            struct rte_mbuf* pkt = bufs[i];
            // rte_prefetch0(rte_pktmbuf_mtod(pkt, void *));
            /* parse the header */
            struct ether_hdr* eth_hdr
                = rte_pktmbuf_mtod(pkt, struct ether_hdr*);
            uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);
            int l2_len = sizeof(*eth_hdr);

            ip_h = (struct ipv4_hdr*)((char*)eth_hdr + l2_len);
            udp_h = (struct udp_hdr*)((char*)ip_h + sizeof(*ip_h));
            // print_pkt(ip_h->src_addr, ip_h->dst_addr, udp_h->src_port,
            // udp_h->dst_port, pkt->data_len, tid); printf("Header type: %d,
            // IPV4: %d\n", eth_type, ETHER_TYPE_IPv4); printf("UDP: %d, %d\n",
            // ip_h->next_proto_id, IPPROTO_UDP);

            if (eth_type == ETHER_TYPE_ARP) {
                obj_ptr->process_arp(bufs[i], eth_hdr, len, tid);
                rte_pktmbuf_free(bufs[i]);
                continue;
            }

            if (eth_type != ETHER_TYPE_IPv4) {
                rte_pktmbuf_free(bufs[i]);
                continue;
            }
            // ip_h = (struct ipv4_hdr *) ((char *) eth_hdr + l2_len);
            if (ip_h->next_proto_id != IPPROTO_UDP) {
                rte_pktmbuf_free(bufs[i]);
                continue;
            }
            // udp_h = (struct udp_hdr *) ((char *) ip_h + sizeof(*ip_h));
            // print_pkt(ip_h->src_addr, ip_h->dst_addr, udp_h->src_port,
            // udp_h->dst_port, pkt->data_len);

            if (ip_h->src_addr != obj_ptr->src_addr) {
                printf("Source addr does not match\n");
                rte_pktmbuf_free(bufs[i]);
                continue;
            }
            if (ip_h->dst_addr != obj_ptr->dst_addr) {
                printf("Destination addr does not match\n");
                rte_pktmbuf_free(bufs[i]);
                continue;
            }

            // if (udp_h->src_port != src_port) {
            //     printf("Source port does not match\n");
            //     continue;
            // }
            // if (udp_h->dst_port != dst_port) {
            //     printf("Destination port does not match\n");
            //     continue;
            // }
            char* payload
                = (char*)eth_hdr + ETH_HDRLEN + IP4_HDRLEN + UDP_HDRLEN + 22;
            // printf("eth_hdr: 0x%lx, offset: %d\n", eth_hdr, ETH_HDRLEN +
            // IP4_HDRLEN + UDP_HDRLEN); rte_memcpy(cur_ptr_buffer, payload,
            // packet_length);
            fastMemcpy(cur_ptr_buffer, payload, packet_length);
            rte_pktmbuf_free(bufs[i]);

            if (kIsWorkerTimingEnabled) {
                int frame_id = ((struct Packet*)cur_buffer_ptr)->frame_id;
                if (frame_id > prev_frame_id) {
                    frame_start[frame_id] = rdtsc();
                    prev_frame_id = frame_id;
                }
            }

            // get the position in buffer
            offset = cur_ptr_buffer_status - buffer_status;
            // move ptr & set status to full
            cur_ptr_buffer_status[0]
                = 1; // has data, after doing fft, it is set to 0
            cur_ptr_buffer_status
                = buffer_status + (offset + 1) % buffer_frame_num;
            cur_ptr_buffer = buffer
                + (cur_ptr_buffer - buffer + packet_length) % buffer_length;

            // Push packet received event into the queue
            Event_data packet_message(
                EventType::kPacketRx, offset + tid * buffer_frame_num);
            if (!message_queue_->enqueue(*local_ptok, packet_message)) {
                printf("socket message enqueue failed\n");
                exit(0);
            }

            packet_num++;

            // Print some information
            if (packet_num == BS_ANT_NUM * max_symbol_id * 1000) {
                auto end = std::chrono::system_clock::now();
                double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2
                    * BS_ANT_NUM * max_symbol_id * 1000;
                std::chrono::duration<double> diff = end - begin;
                // print network throughput & maximum message queue length
                // during this period
                printf("RX thread %d receive 1000 frames in %f secs, "
                       "throughput %f MB/s\n",
                    tid, diff.count(), byte_len / diff.count() / 1024 / 1024);
                begin = std::chrono::system_clock::now();
                packet_num = 0;
            }
        }
    }
}

int PacketTXRX::dequeue_send(int tid, int socket_local, sockaddr_t* remote_addr)
{
    Event_data task_event;
    if (!task_queue_->try_dequeue_from_producer(*tx_ptoks_[tid], task_event))
        return -1;

    // printf("tx queue length: %d\n", task_queue_->size_approx());
    if (task_event.event_type != EventType::kPacketTX) {
        printf("Wrong event type!");
        exit(0);
    }

    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int data_symbol_num_perframe = config_->data_symbol_num_perframe;
    int packet_length = config_->packet_length;
    int offset = task_event.data;
    int ant_id = offset % BS_ANT_NUM;
    int symbol_id = offset / BS_ANT_NUM % data_symbol_num_perframe;
    symbol_id = config_->DLSymbols[0][symbol_id];
    int frame_id = offset / (BS_ANT_NUM * data_symbol_num_perframe);

    if (kDebugBSSender) {
        printf("In TX thread %d: Transmitted frame %d, symbol %d, ant %d, "
               "offset: %d, msg_queue_length: %d\n",
            tid, frame_id, symbol_id, ant_id, offset,
            message_queue_->size_approx());
    }

    char* cur_buffer_ptr = tx_buffer_ + offset * packet_length;
    struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
    new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

    // send data (one OFDM symbol)
    if (sendto(socket_local, (char*)cur_buffer_ptr, packet_length, 0,
            (struct sockaddr*)&remote_addr, sizeof(remote_addr))
        < 0) {
        perror("socket sendto failed");
        exit(0);
    }

    Event_data tx_message(EventType::kPacketTX, offset);
    moodycamel::ProducerToken* local_ptok = rx_ptoks_[tid];
    if (!message_queue_->enqueue(*local_ptok, tx_message)) {
        printf("socket message enqueue failed\n");
        exit(0);
    }
    return offset;
}

void* PacketTXRX::loopSend(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerTX, tx_core_id_, tid);

    int sock_buf_size = 1024 * 1024 * 64 * 8 - 1;
    int local_port_id = 0;
    int remote_port_id = 7000 + tid;
#if USE_IPV4
    int socket_local = setup_socket_ipv4(local_port_id, true, sock_buf_size);
    struct sockaddr_in remote_addr;
    setup_sockaddr_remote_ipv4(
        &remote_addr, remote_port_id, config_->tx_addr.c_str());
#else
    int socket_local = setup_socket_ipv6(local_port_id, true, sock_buf_size);
    struct sockaddr_in6 remote_addr;
    setup_sockaddr_remote_ipv6(
        &remote_addr, remote_port_id, config_->tx_addr.c_str());
#endif

    // auto begin = std::chrono::system_clock::now();

    // int maxMesgQLen = 0;
    // int maxTaskQLen = 0;

    while (true) {
        dequeue_send(tid, socket_local, &remote_addr);
    }
}

static struct rte_flow* generate_ipv4_flow(uint16_t port_id, uint16_t rx_q,
    uint32_t src_ip, uint32_t src_mask, uint32_t dest_ip, uint32_t dest_mask,
    uint16_t src_port, uint16_t src_port_mask, uint16_t dst_port,
    uint16_t dst_port_mask, struct rte_flow_error* error)
{
    struct rte_flow_attr attr;
    struct rte_flow_item pattern[4];
    struct rte_flow_action action[2];
    struct rte_flow* flow = NULL;
    struct rte_flow_action_queue queue = { .index = rx_q };
    struct rte_flow_item_ipv4 ip_spec;
    struct rte_flow_item_ipv4 ip_mask;
    struct rte_flow_item_udp udp_spec;
    struct rte_flow_item_udp udp_mask;
    struct rte_flow_item udp_item;
    int res;
    memset(pattern, 0, sizeof(pattern));
    memset(action, 0, sizeof(action));
    /*
     * set the rule attribute.
     * in this case only ingress packets will be checked.
     */
    memset(&attr, 0, sizeof(struct rte_flow_attr));
    attr.ingress = 1;
    attr.priority = 0;
    /*
     * create the action sequence.
     * one action only,  move packet to queue
     */
    action[0].type = RTE_FLOW_ACTION_TYPE_QUEUE;
    action[0].conf = &queue;
    action[1].type = RTE_FLOW_ACTION_TYPE_END;
    /*
     * set the first level of the pattern (ETH).
     * since in this example we just want to get the
     * ipv4 we set this level to allow all.
     */
    pattern[0].type = RTE_FLOW_ITEM_TYPE_ETH;

    /* the final level must be always type end */
    pattern[3].type = RTE_FLOW_ITEM_TYPE_END;
    /*
     * setting the second level of the pattern (IP).
     * in this example this is the level we care about
     * so we set it according to the parameters.
     */
    memset(&ip_spec, 0, sizeof(struct rte_flow_item_ipv4));
    memset(&ip_mask, 0, sizeof(struct rte_flow_item_ipv4));
    ip_spec.hdr.next_proto_id = IPPROTO_UDP;
    ip_mask.hdr.next_proto_id = 0xf; // protocol mask

    ip_spec.hdr.dst_addr = dest_ip; // htonl(dest_ip);
    ip_mask.hdr.dst_addr = dest_mask;
    ip_spec.hdr.src_addr = src_ip; // htonl(src_ip);
    ip_mask.hdr.src_addr = src_mask;

    pattern[1].type = RTE_FLOW_ITEM_TYPE_IPV4;
    pattern[1].spec = &ip_spec;
    pattern[1].mask = &ip_mask;

    udp_spec.hdr.src_port = src_port;
    udp_spec.hdr.dst_port = dst_port;
    udp_spec.hdr.dgram_len = 0;
    udp_spec.hdr.dgram_cksum = 0;

    udp_mask.hdr.src_port = src_port_mask;
    udp_mask.hdr.dst_port = dst_port_mask;
    udp_mask.hdr.dgram_len = 0;
    udp_mask.hdr.dgram_cksum = 0;

    udp_item.type = RTE_FLOW_ITEM_TYPE_UDP;
    udp_item.spec = &udp_spec;
    udp_item.mask = &udp_mask;
    udp_item.last = NULL;

    pattern[2] = udp_item;

    res = rte_flow_validate(port_id, &attr, pattern, action, error);
    if (!res)
        flow = rte_flow_create(port_id, &attr, pattern, action, error);
    return flow;
}
