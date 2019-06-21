/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */

#include "packageReceiver.hpp"
#include "cpu_attach.hpp"

/* assembly code to read the TSC */
static inline uint64_t RDTSC()
{
  unsigned int hi, lo;
  __asm__ volatile("rdtsc" : "=a" (lo), "=d" (hi));
  return ((uint64_t)hi << 32) | lo;
}


static inline double get_time(void)
{
#if USE_RDTSC
    return double(RDTSC())/2.3e3;
#else
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return tv.tv_sec * 1000000 + tv.tv_nsec / 1000.0;
#endif
}

inline const struct rte_eth_conf port_conf_default() {
    struct rte_eth_conf rte = rte_eth_conf();
    // rte.rxmode.max_rx_pkt_len = ETHER_MAX_LEN;
    rte.rxmode.max_rx_pkt_len = MAX_JUMBO_FRAME_SIZE;
    return rte;
}


static struct rte_flow *
generate_ipv4_flow(uint16_t port_id, uint16_t rx_q,
                uint32_t src_ip, uint32_t src_mask,
                uint32_t dest_ip, uint32_t dest_mask,
                uint16_t src_port, uint16_t src_port_mask,
                uint16_t dst_port, uint16_t dst_port_mask,
                struct rte_flow_error *error);

PackageReceiver::PackageReceiver(int RX_THREAD_NUM, int TX_THREAD_NUM, int in_core_offset)
{
    socket_ = new int[RX_THREAD_NUM];
    /*Configure settings in address struct*/
    // address of sender 
    // servaddr_.sin_family = AF_INET;
    // servaddr_.sin_port = htons(7891);
    // servaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
    // memset(servaddr_.sin_zero, 0, sizeof(servaddr_.sin_zero));  
    
//     for(int i = 0; i < RX_THREAD_NUM; i++)
//     {
// #if USE_IPV4
//         servaddr_[i].sin_family = AF_INET;
//         servaddr_[i].sin_port = htons(8000+i);
//         servaddr_[i].sin_addr.s_addr = INADDR_ANY;//inet_addr("10.225.92.16");//inet_addr("127.0.0.1");
//         memset(servaddr_[i].sin_zero, 0, sizeof(servaddr_[i].sin_zero)); 

//         if ((socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
//             printf("cannot create socket %d\n", i);
//             exit(0);
//         }
// #else
//         servaddr_[i].sin6_family = AF_INET6;
//         servaddr_[i].sin6_addr = in6addr_any;
//         servaddr_[i].sin6_port = htons(8000+i);
        
//         if ((socket_[i] = socket(AF_INET6, SOCK_DGRAM, 0)) < 0) { // UDP socket
//             printf("cannot create socket %d\n", i);
//             exit(0);
//         }
//         else{
//             printf("Created IPV6 socket %d\n", i);
//         }
// #endif
//         // use SO_REUSEPORT option, so that multiple sockets could receive packets simultaneously, though the load is not balance
//         int optval = 1;
//         setsockopt(socket_[i], SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

//         int sock_buf_size = 1024*1024*64*8;
//         if (setsockopt(socket_[i], SOL_SOCKET, SO_RCVBUF, (void*)&sock_buf_size, sizeof(sock_buf_size))<0)
//         {
//             printf("Error setting buffer size to %d\n", sock_buf_size);
//         }

//         // int readValue = 0;
//         // unsigned int readLen = sizeof(readValue);
//         // int res = getsockopt( socket_[i], SOL_SOCKET, SO_RCVBUF, (void*)&readValue, &readLen );
//         // if ( -1 == res )
//         // {
//         //     printf("ERROR reading socket buffer size\n");
//         // }
//         // else
//         // {
//         //     printf("Read socket buffer size:%d\n",readValue);
//         // }

//         if(bind(socket_[i], (struct sockaddr *) &servaddr_[i], sizeof(servaddr_[i])) != 0)
//         {
//             printf("socket bind failed %d\n", i);
//             exit(0);
//         }

//     }
    

    rx_thread_num_ = RX_THREAD_NUM;
    tx_thread_num_ = TX_THREAD_NUM;

    core_id_ = in_core_offset;
    tx_core_id_ = in_core_offset + RX_THREAD_NUM;
    /* initialize random seed: */
    srand (time(NULL));
    rx_context = new PackageReceiverContext[rx_thread_num_];
    tx_context = new PackageReceiverContext[tx_thread_num_];

#if USE_DPDK
    std::string core_list = std::to_string(core_id_)+"-"+std::to_string(core_id_+rx_thread_num_+tx_thread_num_);
    int argc = 5;
    char *argv[] = {
        (char*)"txrx",
        (char*)"-l",
        &core_list[0u],
        (char*)"-n",
        (char*)"8",
        NULL
    };
    // Initialize DPDK environment
    for (int i = 0; i< argc; i++) {
        printf("%s\n",argv[i]);
    }
    int ret = rte_eal_init(argc, argv);
    if (ret < 0)
        rte_exit(EXIT_FAILURE, "Error with EAL initialization\n");

    struct rte_mempool *mbuf_pool;
    unsigned int nb_ports = rte_eth_dev_count_avail();
    printf("Number of ports: %d, socket: %d\n",nb_ports, rte_socket_id());

    // Initialize memory pool
    // mbuf_pool = rte_mempool_create("MBUF pool",
    //                 NUM_MBUFS * nb_ports, 
    //                 MBUF_SIZE + 128*4, MBUF_CACHE_SIZE, 64, NULL, NULL, NULL, NULL,rte_socket_id(), 0);

    // rte_pktmbuf_pool_init(mp, &mbp_priv);
  
    // rte_mempool_populate_default(mp);
    mbuf_pool = rte_pktmbuf_pool_create("MBUF_POOL", NUM_MBUFS * nb_ports, 
        MBUF_CACHE_SIZE, 0, MBUF_SIZE, rte_socket_id());

    if (mbuf_pool == NULL)
        rte_exit(EXIT_FAILURE, "Cannot create mbuf pool\n");

    uint16_t portid;
    RTE_ETH_FOREACH_DEV(portid)
            if (nic_dpdk_init(portid, mbuf_pool) != 0)
                    rte_exit(EXIT_FAILURE, "Cannot init port %"PRIu16 "\n", portid);

    unsigned int nb_lcores = rte_lcore_count();
    uint16_t mtu_size = 0;
    rte_eth_dev_get_mtu(0, &mtu_size);
    printf("Number of DPDK cores: %d, MTU: %d\n", nb_lcores, mtu_size);


    src_addr = rte_cpu_to_be_32(IPv4(10,0,0,2));
    dst_addr = rte_cpu_to_be_32(IPv4(10,0,0,1));

    struct rte_flow_error error;
    struct rte_flow *flow;
    /* create flow for send packet with */
    for (int i = 0; i < rx_thread_num_; i++) {
        uint16_t src_port = rte_cpu_to_be_16(src_port_start + i);
        uint16_t dst_port = rte_cpu_to_be_16(dst_port_start + i);
        flow = generate_ipv4_flow(0, i,
                                src_addr, FULL_MASK,
                                dst_addr, FULL_MASK, 
                                src_port, 0xffff, 
                                dst_port, 0xffff, &error);
        // printf("Add rule for src port: %d, dst port: %d, queue: %d\n", src_port, dst_port, i);
        if (!flow) {
                printf("Flow can't be created %d message: %s\n",
                        error.type,
                        error.message ? error.message : "(no stated reason)");
                rte_exit(EXIT_FAILURE, "error in creating flow");
        }
    }

#endif
}

PackageReceiver::PackageReceiver(int RX_THREAD_NUM, int TX_THREAD_NUM, int in_core_offset, 
        moodycamel::ConcurrentQueue<Event_data> * in_queue_message, moodycamel::ConcurrentQueue<Event_data> * in_queue_task, 
        moodycamel::ProducerToken **in_rx_ptoks):
PackageReceiver(RX_THREAD_NUM, TX_THREAD_NUM, in_core_offset)
{
    message_queue_ = in_queue_message;
    task_queue_ = in_queue_task;
    rx_ptoks_ = in_rx_ptoks;

}

PackageReceiver::~PackageReceiver()
{
    delete[] socket_;
    delete[] tx_context;
    delete[] rx_context;
}




int PackageReceiver::nic_dpdk_init(uint16_t port, struct rte_mempool *mbuf_pool) {
    struct rte_eth_conf port_conf = port_conf_default();
    const uint16_t rxRings = rx_thread_num_, txRings = tx_thread_num_+rx_thread_num_;
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
    printf("Max packet length: %d, dev max: %d\n", port_conf.rxmode.max_rx_pkt_len,dev_info.max_rx_pktlen);
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
        retval = rte_eth_tx_queue_setup(port, q, nb_txd,
                                        rte_eth_dev_socket_id(port), &txconf);
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



std::vector<pthread_t> PackageReceiver::startRecv(char** in_buffer, int** in_buffer_status, int in_buffer_frame_num, long long in_buffer_length, double **in_frame_start)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == package_length * buffer_frame_num_); // should be integer
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status
    frame_start_ = in_frame_start;

    // core_id_ = in_core_id;
    printf("start Recv thread\n");
    // new thread

#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(core_id_) != 0)
    {
        printf("RX: stitch main thread to core %d failed\n", core_id_);
        exit(0);
    }
    else{
        printf("RX: stitch main thread to core %d succeeded\n", core_id_);
    }
#endif

    std::vector<pthread_t> created_threads;
#if USE_DPDK
    unsigned int nb_lcores = rte_lcore_count();
    printf("Number of DPDK cores: %d\n", nb_lcores);
    unsigned int lcore_id;
    int worker_id = 0;
    // Launch specific task to cores
    RTE_LCORE_FOREACH_SLAVE(lcore_id) {
    // launch communication and task thread onto specific core
        if (worker_id < rx_thread_num_) {
            rx_context[worker_id].ptr = this;
            rx_context[worker_id].tid = worker_id;
            rte_eal_remote_launch((lcore_function_t *)loopRecv,
                                &rx_context[worker_id], lcore_id);
            printf("RX: launched thread %d on core %d\n", worker_id, lcore_id);
        } 
        worker_id++;
    }
#else   
    
    for(int i = 0; i < rx_thread_num_; i++)
    {
        pthread_t recv_thread_;
        // record the thread id 
        rx_context[i].ptr = this;
        rx_context[i].tid = i;
        // start socket thread
        if(pthread_create( &recv_thread_, NULL, PackageReceiver::loopRecv, (void *)(&rx_context[i])) != 0)
        {
            perror("socket recv thread create failed");
            exit(0);
        }
        created_threads.push_back(recv_thread_);
    }
#endif
    
    return created_threads;
}





std::vector<pthread_t> PackageReceiver::startTX(char* in_buffer, int* in_buffer_status, float *in_data_buffer, int in_buffer_frame_num, int in_buffer_length)
{
    // check length
    tx_buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == package_length * buffer_frame_num_); // should be integer
    tx_buffer_length_ = in_buffer_length;
    tx_buffer_ = in_buffer;  // for save data
    tx_buffer_status_ = in_buffer_status; // for save status
    tx_data_buffer_ = in_data_buffer;

    // SOCKET_BUFFER_FRAME_NUM = 

    // tx_core_id_ = in_core_id;
    printf("start Transmit thread\n");
// create new threads
    std::vector<pthread_t> created_threads;
#if USE_DPDK
    unsigned int lcore_id;
    int worker_id = 0;
    int thread_id;
    RTE_LCORE_FOREACH_SLAVE(lcore_id) {
    // launch communication and task thread onto specific core
        if (worker_id >= rx_thread_num_) {
            thread_id = worker_id - rx_thread_num_;
            tx_context[thread_id].ptr = this;
            tx_context[thread_id].tid = thread_id;
            rte_eal_remote_launch((lcore_function_t *)loopSend,
                                &tx_context[thread_id], lcore_id);
            printf("TX: launched thread %d on core %d\n", thread_id, lcore_id);
        }
        worker_id++;
    }
#else
    
    for (int i = 0; i < tx_thread_num_; i++) {
        pthread_t send_thread_;
        
        tx_context[i].ptr = this;
        tx_context[i].tid = i;

        if (pthread_create( &send_thread_, NULL, PackageReceiver::loopSend, (void *)(&tx_context[i])) != 0) {
            perror("socket Transmit thread create failed");
            exit(0);
        }
        created_threads.push_back(send_thread_);
    }
#endif
    
    return created_threads;
}


static void print_pkt(int src_ip, int dst_ip, uint16_t src_port, uint16_t dst_port, int len, int tid)
{
    uint8_t     b[12];
    uint16_t    sp,
                dp;

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
    printf("In RX thread %d: rx: %u.%u.%u.%u:%u -> %u.%u.%u.%u:%u (%d bytes)\n", tid,
            b[0], b[1], b[2], b[3], sp,
            b[6], b[7], b[8], b[9], dp,
            len);
}




int PackageReceiver::process_arp(struct rte_mbuf *mbuf, struct ether_hdr *eth, int len, int tid) {
  printf("Processing ARP request\n");
  struct arp_hdr *ah = (struct arp_hdr *)((unsigned char *)eth + ETHER_HDR_LEN);
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
    memcpy((unsigned char *)&eth->d_addr, (unsigned char *)&eth->s_addr, 6);
    memcpy((unsigned char *)&eth->s_addr, (unsigned char *)&server_eth_addr, 6);

    ah->arp_op = rte_cpu_to_be_16(ARP_OP_REPLY);
    ah->arp_data.arp_tha = ah->arp_data.arp_sha;
    memcpy((unsigned char *)&ah->arp_data.arp_sha,
           (unsigned char *)&server_eth_addr, 6);
    ah->arp_data.arp_tip = ah->arp_data.arp_sip;
    ah->arp_data.arp_sip = dst_addr;
    if (likely(1 == rte_eth_tx_burst(0, tid, &mbuf, 1))) {
      return 1;
    }
    printf("Reply ARP\n");
  }
  return 0;
}



static void fastMemcpy(void *pvDest, void *pvSrc, size_t nBytes) {
    // printf("pvDest: 0x%lx, pvSrc: 0x%lx, Dest: %lx, Src, %lx\n",intptr_t(pvDest), intptr_t(pvSrc), (intptr_t(pvDest) & 31), (intptr_t(pvSrc) & 31) );
    // assert(nBytes % 32 == 0);
    // assert((intptr_t(pvDest) & 31) == 0);
    // assert((intptr_t(pvSrc) & 31) == 0);
    const __m256i *pSrc = reinterpret_cast<const __m256i*>(pvSrc);
    __m256i *pDest = reinterpret_cast<__m256i*>(pvDest);
    int64_t nVects = nBytes / sizeof(*pSrc);
    for (; nVects > 0; nVects--, pSrc++, pDest++) {
    const __m256i loaded = _mm256_stream_load_si256(pSrc);
    _mm256_stream_si256(pDest, loaded);
    }
    _mm_sfence();
}



void* PackageReceiver::loopRecv(void *in_context)
{
    // get the pointer of class & tid
    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    printf("package receiver thread %d start\n", tid);
    // get pointer of message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->core_id_;
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores

#if !USE_DPDK
    #ifdef ENABLE_CPU_ATTACH 
        if(stick_this_thread_to_core(core_id + tid + 1) != 0) {
            printf("RX thread: attach thread %d to core %d failed\n", tid, core_id + tid + 1);
            exit(0);
        }
        else {
            printf("RX thread: attached thread %d to core %d\n", tid, core_id + tid + 1);
        }
    #endif



    #if USE_IPV4
        struct sockaddr_in servaddr_local;
        int socket_local, socket_tcp_local;
        servaddr_local.sin_family = AF_INET;
        servaddr_local.sin_port = htons(8000+tid);
        servaddr_local.sin_addr.s_addr = INADDR_ANY;//inet_addr("10.225.92.16");//inet_addr("127.0.0.1");
        memset(servaddr_local.sin_zero, 0, sizeof(servaddr_local.sin_zero)); 

        if ((socket_local = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
            printf("RX thread %d cannot create IPV4 socket\n", tid);
            exit(0);
        }
        else{
            printf("RX thread %d created IPV4 socket\n", tid);
        }


        if ((socket_tcp_local = socket(AF_INET, SOCK_STREAM, 0)) < 0) { // UDP socket
            printf("RX thread %d cannot create IPV4 TCP socket\n", tid);
            exit(0);
        }
        else{
            printf("RX thread %d created IPV4 socket\n", tid);
        }
    #else
        struct sockaddr_in6 servaddr_local;
        int socket_local;
        servaddr_local.sin6_family = AF_INET6;
        servaddr_local.sin6_addr = in6addr_any;
        servaddr_local.sin6_port = htons(8000+tid);
        
        if ((socket_local = socket(AF_INET6, SOCK_DGRAM, 0)) < 0) { // UDP socket
            printf("RX thread %d cannot create IPV6 socket\n", tid);
            exit(0);
        }
        else{
            printf("RX thread %d Created IPV46 socket\n", tid);
        }
    #endif

    // use SO_REUSEPORT option, so that multiple sockets could receive packets simultaneously, though the load is not balance
    int optval = 1;

    setsockopt(socket_local, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
    socklen_t optlen;
    int sock_buf_size;
    optlen = sizeof(sock_buf_size);
    // int res = getsockopt(socket_local, SOL_SOCKET, SO_RCVBUF, &sock_buf_size, &optlen);
    // printf("Current socket %d buffer size %d\n", tid, sock_buf_size);
    sock_buf_size = 1024*1024*64*15-1;
    if (setsockopt(socket_local, SOL_SOCKET, SO_RCVBUF, &sock_buf_size, sizeof(sock_buf_size))<0) {
        printf("Error setting buffer size to %d\n", sock_buf_size);
    }
    else {     
        int res = getsockopt(socket_local, SOL_SOCKET, SO_RCVBUF, &sock_buf_size, &optlen);
        printf("Set socket %d buffer size to %d\n", tid, sock_buf_size);
    }

    if(bind(socket_local, (struct sockaddr *) &servaddr_local, sizeof(servaddr_local)) != 0) {
        printf("socket bind failed %d\n", tid);
        exit(0);
    }

    if(bind(socket_tcp_local, (struct sockaddr *) &servaddr_local, sizeof(servaddr_local)) != 0) {
        printf("TCP socket bind failed %d\n", tid);
        exit(0);
    }

    if (listen(socket_tcp_local, 3) < 0) 
    { 
        perror("listen"); 
        exit(EXIT_FAILURE); 
    } 

#else
        uint16_t nic;
        struct rte_mbuf *bufs[BURST_SIZE * 2] __attribute__( ( aligned (64) ) );
        struct udp_hdr    *udp_h;
        struct ipv4_hdr   *ip_h;
        uint16_t dst_port = rte_cpu_to_be_16((obj_ptr->dst_port_start + tid));
        uint16_t src_port = rte_cpu_to_be_16((obj_ptr->src_port_start + tid));

        uint16_t mtu_size;
        rte_eth_dev_get_mtu(0, &mtu_size);
        printf("MTU: %d\n", mtu_size);

#endif


    // use token to speed up
    // moodycamel::ProducerToken local_ptok(*message_queue_);
    moodycamel::ProducerToken *local_ptok = obj_ptr->rx_ptoks_[tid];


    char* buffer = obj_ptr->buffer_[tid];
    int* buffer_status = obj_ptr->buffer_status_[tid];
    long long buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;
    double *frame_start = obj_ptr->frame_start_[tid];

    // walk through all the pages
    double temp;
    for (int i = 0; i < 20; i++) {
        temp = frame_start[i * 512];
    }


    char* cur_ptr_buffer = buffer;
    int* cur_ptr_buffer_status = buffer_status;
    // loop recv
    // socklen_t addrlen = sizeof(obj_ptr->servaddr_[tid]);
    int offset = 0;
    int package_num = 0;
    auto begin = std::chrono::system_clock::now();

    int ret = 0;
    int max_subframe_id = ENABLE_DOWNLINK ? UE_NUM : subframe_num_perframe;
    int prev_frame_id = -1;
    int package_num_per_frame = 0;
    double start_time= get_time();


    // printf("Rx thread %d: on core %d\n", tid, sched_getcpu());


    while(true) {
#if !USE_DPDK
        // if buffer is full, exit
        if (cur_ptr_buffer_status[0] == 1) {
            printf("Receive thread %d buffer full, offset: %d\n", tid, offset);
            exit(0);
        }

        int recvlen = -1;



        // start_time= get_time();
        // if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &obj_ptr->servaddr_[tid], &addrlen)) < 0)
        if ((recvlen = recv(socket_local, (char*)cur_ptr_buffer, package_length, 0))<0) {
        // if ((recvlen = recvfrom(socket_local, (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &servaddr_local, &addrlen)) < 0) {
            perror("recv failed");
            exit(0);
        }


        
        

    #if MEASURE_TIME
        // read information from received packet
        int ant_id, frame_id, subframe_id, cell_id;
        frame_id = *((int *)cur_ptr_buffer);
        subframe_id = *((int *)cur_ptr_buffer + 1);
        // cell_id = *((int *)cur_ptr_buffer + 2);
        ant_id = *((int *)cur_ptr_buffer + 3);
        // printf("RX thread %d received frame %d subframe %d, ant %d\n", tid, frame_id, subframe_id, ant_id);
        if (frame_id > prev_frame_id) {
            *(frame_start + frame_id) = get_time();
            prev_frame_id = frame_id;
            if (frame_id % 512 == 200) {
                _mm_prefetch((char*)(frame_start+frame_id+512), _MM_HINT_T0);
                // double temp = frame_start[frame_id+3];
            }
        }
    #endif
        // get the position in buffer
        offset = cur_ptr_buffer_status - buffer_status;
        // move ptr & set status to full
        cur_ptr_buffer_status[0] = 1; // has data, after doing fft, it is set to 0
        cur_ptr_buffer_status = buffer_status + (offset + 1) % buffer_frame_num;
        cur_ptr_buffer = buffer + (cur_ptr_buffer - buffer + package_length) % buffer_length;
        // push EVENT_PACKAGE_RECEIVED event into the queue
        Event_data package_message;
        package_message.event_type = EVENT_PACKAGE_RECEIVED;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        package_message.data = offset + tid * buffer_frame_num;
        if ( !message_queue_->enqueue(*local_ptok, package_message ) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

        
        // for (int i = 0; i < (package_length/64); i++)
        //     _mm_prefetch((char*)cur_ptr_buffer + 64 * i, _MM_HINT_NTA);

        
        package_num++;
    // #if DEBUG_PRINT_PER_FRAME_DONE
    //     package_num_per_frame++;
    //     if (package_num_per_frame == BS_ANT_NUM * max_subframe_id/obj_ptr->rx_thread_num_) {
    //         printf("RX thread %d receive all packets in frame %d\n", tid, frame_id);
    //         package_num_per_frame = 0;
    //     }
    // #endif
        
        // print some information
        if (package_num == BS_ANT_NUM * max_subframe_id * 1000) {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * BS_ANT_NUM * max_subframe_id * 1000;
            std::chrono::duration<double> diff = end - begin;
            // print network throughput & maximum message queue length during this period
            printf("RX thread %d receive 1000 frames in %f secs, throughput %f MB/s\n", tid, diff.count(), 
                byte_len / diff.count() / 1024 / 1024);
            begin = std::chrono::system_clock::now();
            package_num = 0;
        } 

#else

        uint16_t nb_rx = rte_eth_rx_burst(0, tid, bufs, BURST_SIZE);
        // printf("Thread %d receives %d packets\n", tid, nb_rx);
        if (unlikely(nb_rx == 0)) {
            continue;
        }

        // printf("Thread %d receives %d packets\n", tid, nb_rx);
        for (int i = 0; i < nb_rx; i++) {
            // if buffer is full, exit
            if (cur_ptr_buffer_status[0] == 1) {
                printf("Receive thread %d buffer full, offset: %d\n", tid, offset);
                exit(0);
            }
            // struct rte_eth_stats eth_stats;
            // rte_eth_stats_get(0, &eth_stats);
            // printf("RX thread %d: total number of packets received %llu, dropped rx full %llu and rest= %llu, %llu, %llu\n", 
            //     tid, eth_stats.ipackets, eth_stats.imissed,
            //     eth_stats.ierrors, eth_stats.rx_nombuf, eth_stats.q_ipackets[tid]);

            int len = rte_pktmbuf_data_len(bufs[i]);
            struct rte_mbuf *pkt = bufs[i];
            // rte_prefetch0(rte_pktmbuf_mtod(pkt, void *));
            /* parse the header */
            struct ether_hdr *eth_hdr = rte_pktmbuf_mtod(pkt, struct ether_hdr *);
            uint16_t eth_type = rte_be_to_cpu_16(eth_hdr->ether_type);
            int l2_len = sizeof(*eth_hdr);

            ip_h = (struct ipv4_hdr *) ((char *) eth_hdr + l2_len);
            udp_h = (struct udp_hdr *) ((char *) ip_h + sizeof(*ip_h));
            // print_pkt(ip_h->src_addr, ip_h->dst_addr, udp_h->src_port, udp_h->dst_port, pkt->data_len, tid);
            // printf("Header type: %d, IPV4: %d\n", eth_type, ETHER_TYPE_IPv4);
            // printf("UDP: %d, %d\n", ip_h->next_proto_id, IPPROTO_UDP);
            
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
            // print_pkt(ip_h->src_addr, ip_h->dst_addr, udp_h->src_port, udp_h->dst_port, pkt->data_len);
            
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
            char *payload = (char *)eth_hdr + ETH_HDRLEN + IP4_HDRLEN + UDP_HDRLEN + 22;
            // printf("eth_hdr: 0x%lx, offset: %d\n", eth_hdr, ETH_HDRLEN + IP4_HDRLEN + UDP_HDRLEN);
            // rte_memcpy(cur_ptr_buffer, payload, package_length);
            fastMemcpy(cur_ptr_buffer, payload, package_length);
            rte_pktmbuf_free(bufs[i]);

        #if MEASURE_TIME
            // read information from received packet
            int ant_id, frame_id, subframe_id, cell_id;
            frame_id = *((int *)cur_ptr_buffer);
            subframe_id = *((int *)cur_ptr_buffer + 1);
            // cell_id = *((int *)cur_ptr_buffer + 2);
            ant_id = *((int *)cur_ptr_buffer + 3);
            // printf("RX thread %d received frame %d subframe %d, ant %d\n", tid, frame_id, subframe_id, ant_id);
            if (frame_id > prev_frame_id) {
                *(frame_start + frame_id) = get_time();
                prev_frame_id = frame_id;
                if (frame_id % 512 == 200) {
                    _mm_prefetch((char*)(frame_start+frame_id+512), _MM_HINT_T0);
                    // double temp = frame_start[frame_id+3];
                }
            }
        #endif

            // get the position in buffer
            offset = cur_ptr_buffer_status - buffer_status;
            // move ptr & set status to full
            cur_ptr_buffer_status[0] = 1; // has data, after doing fft, it is set to 0
            cur_ptr_buffer_status = buffer_status + (offset + 1) % buffer_frame_num;
            cur_ptr_buffer = buffer + (cur_ptr_buffer - buffer + package_length) % buffer_length;
            // push EVENT_PACKAGE_RECEIVED event into the queue
            Event_data package_message;
            package_message.event_type = EVENT_PACKAGE_RECEIVED;
            package_message.data = offset + tid * buffer_frame_num;
            if ( !message_queue_->enqueue(*local_ptok, package_message ) ) {
                printf("socket message enqueue failed\n");
                exit(0);
            }
            package_num++;
            // print some information
            if(package_num == BS_ANT_NUM * max_subframe_id * 1000) {
                auto end = std::chrono::system_clock::now();
                double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * BS_ANT_NUM * max_subframe_id * 1000;
                std::chrono::duration<double> diff = end - begin;
                // print network throughput & maximum message queue length during this period
                printf("RX thread %d receive 1000 frames in %f secs, throughput %f MB/s\n", tid, diff.count(), 
                    byte_len / diff.count() / 1024 / 1024);
                begin = std::chrono::system_clock::now();
                package_num = 0;
            } 
            
        }
#endif


    }


}






void* PackageReceiver::loopSend(void *in_context)
{


    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    printf("package sender thread %d start\n", tid);

    moodycamel::ConcurrentQueue<Event_data> *task_queue_ = obj_ptr->task_queue_;
    // get pointer to message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->tx_core_id_;
#if !USE_DPDK
    #ifdef ENABLE_CPU_ATTACH
        if(stick_this_thread_to_core(core_id + tid + 1) != 0) {
            printf("TX thread: attach thread %d to core %d failed\n", tid, core_id + tid + 1);
            exit(0);
        }
        else {
            printf("TX thread: attached thread %d to core %d\n", tid, core_id + tid + 1);
        }
    #endif
#endif

#if USE_IPV4
    struct sockaddr_in servaddr_local;
    struct sockaddr_in cliaddr_local;
    int socket_local;
    servaddr_local.sin_family = AF_INET;
    servaddr_local.sin_port = htons(6000+tid);
    servaddr_local.sin_addr.s_addr = inet_addr("10.0.0.2");//inet_addr("10.225.92.16");//inet_addr("127.0.0.1");
    memset(servaddr_local.sin_zero, 0, sizeof(servaddr_local.sin_zero)); 

    cliaddr_local.sin_family = AF_INET;
    cliaddr_local.sin_port = htons(0);  // out going port is random
    cliaddr_local.sin_addr.s_addr = htons(INADDR_ANY);
    memset(cliaddr_local.sin_zero, 0, sizeof(cliaddr_local.sin_zero));  

    if ((socket_local = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
        printf("TX thread %d cannot create IPV4 socket\n", tid);
        exit(0);
    }
    else{
        printf("TX thread %d created IPV4 socket\n", tid);
    }
#else
    struct sockaddr_in6 servaddr_local;
    struct sockaddr_in6 cliaddr_local;
    int socket_local;
    servaddr_local.sin6_family = AF_INET6;
    inet_pton(AF_INET6, "fe80::5a9b:5a2f:c20a:d4d5", &servaddr_local.sin6_addr);
    servaddr_local.sin6_port = htons(6000+tid);

    cliaddr_local.sin6_family = AF_INET;
    cliaddr_local.sin6_port = htons(6000+i);
    cliaddr_local.sin6_addr = in6addr_any;
    
    if ((socket_local = socket(AF_INET6, SOCK_DGRAM, 0)) < 0) { // UDP socket
        printf("TX thread %d cannot create IPV6 socket\n", tid);
        exit(0);
    }
    else{
        printf("TX thread %d created IPV6 socket\n", tid);
    }
#endif

    /*Bind socket with address struct*/
    if (bind(socket_local, (struct sockaddr *) &cliaddr_local, sizeof(cliaddr_local)) != 0) {
        printf("socket bind failed %d\n", tid);
        exit(0);
    }




    // downlink socket buffer
    char *buffer = obj_ptr->tx_buffer_;
    // downlink socket buffer status
    int *buffer_status = obj_ptr->tx_buffer_status_;
    // downlink data buffer
    float *data_buffer = obj_ptr->tx_data_buffer_;
    // buffer_length: package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    int buffer_length = obj_ptr->tx_buffer_length_;
    // buffer_frame_num: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    int buffer_frame_num = obj_ptr->tx_buffer_frame_num_;



    auto begin = std::chrono::system_clock::now();
    int package_count = 0;
    int ret;
    int offset;
    char *cur_ptr_buffer;
    int *cur_ptr_buffer_status;
    float *cur_ptr_data;
    int ant_id, frame_id, subframe_id, total_data_subframe_id, current_data_subframe_id;
    int cell_id = 0;
    int maxMesgQLen = 0;
    int maxTaskQLen = 0;

    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);
    moodycamel::ConsumerToken local_ctok(*task_queue_);
    while(true) {
    
        Event_data task_event;
        ret = task_queue_->try_dequeue(task_event); 
        if(!ret)
            continue;
        // printf("tx queue length: %d\n", task_queue_->size_approx());
        if (task_event.event_type!=TASK_SEND) {
            printf("Wrong event type!");
            exit(0);
        }

        // printf("In transmitter\n");

        offset = task_event.data;
        ant_id = offset % BS_ANT_NUM;
        total_data_subframe_id = offset / BS_ANT_NUM; 
        current_data_subframe_id = total_data_subframe_id % data_subframe_num_perframe;
        subframe_id = current_data_subframe_id + UE_NUM;
        frame_id = total_data_subframe_id / data_subframe_num_perframe;

        int socket_subframe_offset = frame_id * data_subframe_num_perframe + current_data_subframe_id;
        int data_subframe_offset = frame_id * data_subframe_num_perframe + current_data_subframe_id;
        cur_ptr_buffer = buffer + (socket_subframe_offset * BS_ANT_NUM + ant_id) * package_length;  
        cur_ptr_data = (data_buffer + 2 * data_subframe_offset * OFDM_CA_NUM * BS_ANT_NUM);   
        *((int *)cur_ptr_buffer) = frame_id;
        *((int *)cur_ptr_buffer + 1) = subframe_id;
        *((int *)cur_ptr_buffer + 2) = cell_id;
        *((int *)cur_ptr_buffer + 3) = ant_id;

        // send data (one OFDM symbol)
        if (sendto(socket_local, (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *)&servaddr_local, sizeof(servaddr_local)) < 0) {
            perror("socket sendto failed");
            exit(0);
        }

#if DEBUG_BS_SENDER
        printf("In TX thread %d: Transmitted frame %d, subframe %d, ant %d, offset: %d\n", tid, frame_id, subframe_id, ant_id, offset);
#endif
        
        Event_data package_message;
        package_message.event_type = EVENT_PACKAGE_SENT;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        package_message.data = offset;
        if ( !message_queue_->enqueue(local_ptok, package_message ) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

        // if (package_count % (BS_ANT_NUM) == 0)
        // {
        //     usleep(71);
        // }

        if(package_count == BS_ANT_NUM * dl_data_subframe_num_perframe * 1000)
        {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * BS_ANT_NUM * data_subframe_num_perframe * 1000;
            std::chrono::duration<double> diff = end - begin;
            // printf("TX thread %d send 1000 frames in %f secs, throughput %f MB/s, max Queue Length: message %d, tx task %d\n", tid, diff.count(), byte_len / diff.count() / 1024 / 1024, maxMesgQLen, maxTaskQLen);
            printf("TX thread %d send 1000 frames in %f secs, throughput %f MB/s\n", tid, diff.count(), byte_len / diff.count() / 1024 / 1024);
            begin = std::chrono::system_clock::now();
            package_count = 0;
        }
    }
    
}





static struct rte_flow *
generate_ipv4_flow(uint16_t port_id, uint16_t rx_q,
                uint32_t src_ip, uint32_t src_mask,
                uint32_t dest_ip, uint32_t dest_mask,
                uint16_t src_port, uint16_t src_port_mask,
                uint16_t dst_port, uint16_t dst_port_mask,
                struct rte_flow_error *error)
{
        struct rte_flow_attr attr;
        struct rte_flow_item pattern[4];
        struct rte_flow_action action[2];
        struct rte_flow *flow = NULL;
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

        ip_spec.hdr.dst_addr = dest_ip;//htonl(dest_ip);
        ip_mask.hdr.dst_addr = dest_mask;
        ip_spec.hdr.src_addr = src_ip;//htonl(src_ip);
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


