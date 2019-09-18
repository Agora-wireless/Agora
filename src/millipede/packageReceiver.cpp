/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */

#include "packageReceiver.hpp"
#include "cpu_attach.hpp"

#ifdef USE_DPDK
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
#endif

PackageReceiver::PackageReceiver(Config *cfg, int RX_THREAD_NUM, int TX_THREAD_NUM, int in_core_offset)
{
    socket_ = new int[RX_THREAD_NUM];
    config_ = cfg;
    rx_thread_num_ = RX_THREAD_NUM;
    tx_thread_num_ = TX_THREAD_NUM;

    core_id_ = in_core_offset;
    tx_core_id_ = in_core_offset + RX_THREAD_NUM;

    BS_ANT_NUM = cfg->BS_ANT_NUM;
    UE_NUM = cfg->UE_NUM;
    OFDM_CA_NUM = cfg->OFDM_CA_NUM;
    OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    subframe_num_perframe = cfg->symbol_num_perframe;
    data_subframe_num_perframe = cfg->data_symbol_num_perframe;
    ul_data_subframe_num_perframe = cfg->ul_data_symbol_num_perframe;
    dl_data_subframe_num_perframe = cfg->dl_data_symbol_num_perframe;
    downlink_mode = cfg->downlink_mode;
    package_length = cfg->package_length;
    package_header_offset = cfg->package_header_offset;

    //frameID = new int[TASK_BUFFER_FRAME_NUM];
    /* initialize random seed: */
    srand (time(NULL));
    rx_context = new PackageReceiverContext[rx_thread_num_];
    tx_context = new PackageReceiverContext[tx_thread_num_];

#ifdef USE_DPDK
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

#else
#ifdef USE_ARGOS
    radioconfig_ = new RadioConfig(config_);

#endif
#endif
}

PackageReceiver::PackageReceiver(Config *cfg, int RX_THREAD_NUM, int TX_THREAD_NUM, int in_core_offset, 
        moodycamel::ConcurrentQueue<Event_data> * in_queue_message, moodycamel::ConcurrentQueue<Event_data> * in_queue_task, 
        moodycamel::ProducerToken **in_rx_ptoks, moodycamel::ProducerToken **in_tx_ptoks):
PackageReceiver(cfg, RX_THREAD_NUM, TX_THREAD_NUM, in_core_offset)
{
    message_queue_ = in_queue_message;
    task_queue_ = in_queue_task;
    rx_ptoks_ = in_rx_ptoks;
    tx_ptoks_ = in_tx_ptoks;

}

PackageReceiver::~PackageReceiver()
{
    delete[] socket_;
    delete[] tx_context;
    delete[] rx_context;
#ifdef USE_ARGOS
    radioconfig_->radioStop();
    delete radioconfig_;
#endif
    delete config_;
}

#ifdef USE_DPDK
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

#endif


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
    printf("start recv thread\n");

    std::vector<pthread_t> created_threads;
#ifdef USE_ARGOS
    bool ret = radioconfig_->radioStart();
    if (!ret) return created_threads;
    calib_mat = radioconfig_->get_calib_mat();
    int nradio_per_thread = config_->nRadios/rx_thread_num_;
    int rem_thread_nradio = config_->nRadios%rx_thread_num_;
#endif

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

    if (!downlink_mode)
    {
    #ifdef USE_DPDK
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
                rte_eal_remote_launch((lcore_function_t *)loopRecv_DPDK,
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
        #ifdef USE_ARGOS
            rx_context[i].radios = (i < rem_thread_nradio) ? nradio_per_thread + 1 : nradio_per_thread;
            // start socket thread
            if(pthread_create( &recv_thread_, NULL, PackageReceiver::loopRecv_Argos, (void *)(&rx_context[i])) != 0) 
        #else
            // start socket thread
            if(pthread_create( &recv_thread_, NULL, PackageReceiver::loopRecv, (void *)(&rx_context[i])) != 0)
        #endif
            {
                perror("socket recv thread create failed");
                exit(0);
            }
            created_threads.push_back(recv_thread_);
        }
    #endif
    }
    else
    {
    #ifdef USE_ARGOS
        for(int i = 0; i < rx_thread_num_; i++)
        {
            pthread_t recv_thread_;
            // record the thread id 
            rx_context[i].ptr = this;
            rx_context[i].tid = i;
            rx_context[i].radios = (i < rem_thread_nradio) ? nradio_per_thread + 1 : nradio_per_thread;
            // start socket thread
            if(pthread_create( &recv_thread_, NULL, PackageReceiver::loopRecv_Argos, (void *)(&rx_context[i])) != 0) 
            {
                perror("socket recv thread create failed");
                exit(0);
            }
            created_threads.push_back(recv_thread_);
        }
    #endif
    }
    
#ifdef USE_ARGOS
    sleep(1);
    pthread_cond_broadcast(&cond);
    //sleep(1);
    radioconfig_->go();
#endif
    return created_threads;
}





std::vector<pthread_t> PackageReceiver::startTX(char* in_buffer, int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length)
{
    // check length
    tx_buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == package_length * buffer_frame_num_); // should be integer
    tx_buffer_length_ = in_buffer_length;
    tx_buffer_ = in_buffer;  // for save data
    tx_buffer_status_ = in_buffer_status; // for save status
    // tx_data_buffer_ = in_data_buffer;

    // SOCKET_BUFFER_FRAME_NUM = 

    // tx_core_id_ = in_core_id;
    printf("start Transmit thread\n");
// create new threads
    std::vector<pthread_t> created_threads;
#ifdef USE_DPDK
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
    
    // for (int i = 0; i < tx_thread_num_; i++) {
    //     pthread_t send_thread_;
        
    //     tx_context[i].ptr = this;
    //     tx_context[i].tid = i;

    //     if (pthread_create( &send_thread_, NULL, PackageReceiver::loopSend, (void *)(&tx_context[i])) != 0) {
    //         perror("socket Transmit thread create failed");
    //         exit(0);
    //     }

    //     created_threads.push_back(send_thread_);
    // }
    for (int i = 0; i < tx_thread_num_; i++) {
        pthread_t send_thread_;
        
        tx_context[i].ptr = this;
        tx_context[i].tid = i;

    #ifdef USE_ARGOS
        if (pthread_create( &send_thread_, NULL, PackageReceiver::loopSend_Argos, (void *)(&tx_context[i])) != 0)
    #else
        if (pthread_create( &send_thread_, NULL, PackageReceiver::loopTXRX, (void *)(&tx_context[i])) != 0)
    #endif
        {
            perror("socket Transmit thread create failed");
            exit(0);
        }
        
        created_threads.push_back(send_thread_);
    }
#endif
    
    return created_threads;
}



#ifdef USE_DPDK
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
#endif




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
    // int BS_ANT_NUM = obj_ptr->BS_ANT_NUM;
    // int UE_NUM = obj_ptr->UE_NUM;
    // int OFDM_CA_NUM = obj_ptr->OFDM_CA_NUM;
    // int OFDM_DATA_NUM = obj_ptr->OFDM_DATA_NUM;
    // int subframe_num_perframe = obj_ptr->subframe_num_perframe;
    // int data_subframe_num_perframe = obj_ptr->data_subframe_num_perframe;
    // int ul_data_subframe_num_perframe = obj_ptr->ul_data_subframe_num_perframe;
    // int dl_data_subframe_num_perframe = obj_ptr->dl_data_subframe_num_perframe;
    // bool downlink_mode = obj_ptr->downlink_mode;
    int package_length = obj_ptr->package_length;

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
    int socket_local;
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
    // fcntl(socket_local, F_SETFL, O_NONBLOCK);
    socklen_t optlen;
    int sock_buf_size;
    optlen = sizeof(sock_buf_size);
    // int res = getsockopt(socket_local, SOL_SOCKET, SO_RCVBUF, &sock_buf_size, &optlen);
    // printf("Current socket %d buffer size %d\n", tid, sock_buf_size);
    sock_buf_size = 1024*1024*64*8-1;
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




    // use token to speed up
    // moodycamel::ProducerToken local_ptok(*message_queue_);
    // moodycamel::ProducerToken *local_ptok = new moodycamel::ProducerToken(*message_queue_);
    moodycamel::ProducerToken *local_ptok = obj_ptr->rx_ptoks_[tid];

    char *buffer_ptr = obj_ptr->buffer_[tid];
    int *buffer_status_ptr = obj_ptr->buffer_status_[tid];
    long long buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;
    double *frame_start = obj_ptr->frame_start_[tid];

    // walk through all the pages
    double temp;
    for (int i = 0; i < 20; i++) {
        temp = frame_start[i * 512];
    }


    char* cur_buffer_ptr = buffer_ptr;
    int* cur_buffer_status_ptr = buffer_status_ptr;
    // loop recv
    // socklen_t addrlen = sizeof(obj_ptr->servaddr_[tid]);
    int offset = 0;
    // int package_num = 0;
    // int ret = 0;
    // int max_subframe_id = downlink_mode ? UE_NUM : subframe_num_perframe;
    int prev_frame_id = -1;
    // int package_num_per_frame = 0;
    // double start_time= get_time();


    // printf("Rx thread %d: on core %d\n", tid, sched_getcpu());


    while(true) {
        // if buffer is full, exit
        if (cur_buffer_status_ptr[0] == 1) {
            printf("Receive thread %d buffer full, offset: %d\n", tid, offset);
            exit(0);
        }

        int recvlen = -1;



        // start_time= get_time();
        // if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &obj_ptr->servaddr_[tid], &addrlen)) < 0)
        if ((recvlen = recv(socket_local, (char*)cur_buffer_ptr, package_length, 0))<0) {
        // if ((recvlen = recvfrom(socket_local, (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &servaddr_local, &addrlen)) < 0) {
            perror("recv failed");
            exit(0);
        } 

    #if MEASURE_TIME
        // read information from received packet
        int frame_id; //, subframe_id, ant_id, cell_id;
        frame_id = *((int *)cur_buffer_ptr);
        // subframe_id = *((int *)cur_buffer_ptr + 1);
        // cell_id = *((int *)cur_buffer_ptr + 2);
        // ant_id = *((int *)cur_buffer_ptr + 3);
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
        offset = cur_buffer_status_ptr - buffer_status_ptr;
        // move ptr & set status to full
        cur_buffer_status_ptr[0] = 1; // has data, after doing fft, it is set to 0
        cur_buffer_status_ptr = buffer_status_ptr + (offset + 1) % buffer_frame_num;
        cur_buffer_ptr = buffer_ptr + (cur_buffer_ptr - buffer_ptr + package_length) % buffer_length;
        // push EVENT_PACKAGE_RECEIVED event into the queue
        Event_data package_message;
        package_message.event_type = EVENT_PACKAGE_RECEIVED;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        package_message.data = generateOffset2d_setbits(tid, offset, 28);
        // package_message.data = offset + tid * buffer_frame_num;
        // if ( !message_queue_->enqueue(package_message ) ) {
        if ( !message_queue_->enqueue(*local_ptok, package_message) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

    }

}


#ifdef USE_DPDK

void* PackageReceiver::loopRecv_DPDK(void *in_context)
{
    // get the pointer of class & tid
    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    printf("package receiver thread %d start\n", tid);
    // get pointer of message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->core_id_;
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores


    int BS_ANT_NUM = obj_ptr->BS_ANT_NUM;
    int UE_NUM = obj_ptr->UE_NUM;
    int OFDM_CA_NUM = obj_ptr->OFDM_CA_NUM;
    int OFDM_DATA_NUM = obj_ptr->OFDM_DATA_NUM;
    int subframe_num_perframe = obj_ptr->subframe_num_perframe;
    int data_subframe_num_perframe = obj_ptr->data_subframe_num_perframe;
    int ul_data_subframe_num_perframe = obj_ptr->ul_data_subframe_num_perframe;
    int dl_data_subframe_num_perframe = obj_ptr->dl_data_subframe_num_perframe;
    bool downlink_mode = obj_ptr->downlink_mode;
    int package_length = obj_ptr->package_length;


    uint16_t nic;
    struct rte_mbuf *bufs[BURST_SIZE * 2] __attribute__( ( aligned (64) ) );
    struct udp_hdr    *udp_h;
    struct ipv4_hdr   *ip_h;
    uint16_t dst_port = rte_cpu_to_be_16((obj_ptr->dst_port_start + tid));
    uint16_t src_port = rte_cpu_to_be_16((obj_ptr->src_port_start + tid));

    uint16_t mtu_size;
    rte_eth_dev_get_mtu(0, &mtu_size);
    printf("MTU: %d\n", mtu_size);



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
    int max_subframe_id = downlink_mode ? UE_NUM : subframe_num_perframe;
    int prev_frame_id = -1;
    int package_num_per_frame = 0;
    double start_time= get_time();


    // printf("Rx thread %d: on core %d\n", tid, sched_getcpu());


    while(true) {

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
            if ( !message_queue_->enqueue(*local_ptok, package_message) ) {
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
    }
}
#endif

#ifdef USE_ARGOS
void* PackageReceiver::loopRecv_Argos(void *in_context)
{
    // get the pointer of class & tid
    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    //printf("Recv thread: thread %d start\n", tid);
    int nradio_cur_thread = ((PackageReceiverContext *)in_context)->radios;
    //printf("receiver thread %d has %d radios\n", tid, nradio_cur_thread);
    Config *cfg = obj_ptr->config_;
    // get pointer of message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->core_id_;
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    // printf("Recv thread: pinning thread %d to core %d\n", tid, core_id + tid);
    if(pin_to_core(core_id + tid) != 0)
    {
        printf("Recv thread: pinning thread %d to core %d failed\n", tid, core_id + tid);
        exit(0);
    }
    else {
        printf("Recv thread: pinning thread %d to core %d succeed\n", tid, core_id + tid);
    }
#endif

    //int BS_ANT_NUM = obj_ptr->BS_ANT_NUM;
    //int UE_NUM = obj_ptr->UE_NUM;
    //int OFDM_CA_NUM = obj_ptr->OFDM_CA_NUM;
    //int OFDM_DATA_NUM = obj_ptr->OFDM_DATA_NUM;
    //int subframe_num_perframe = obj_ptr->subframe_num_perframe;
    //int data_subframe_num_perframe = obj_ptr->data_subframe_num_perframe;
    //int ul_data_subframe_num_perframe = obj_ptr->ul_data_subframe_num_perframe;
    //int dl_data_subframe_num_perframe = obj_ptr->dl_data_subframe_num_perframe;
    int package_length = obj_ptr->package_length;
    int package_header_offset = obj_ptr->package_header_offset;
    //// Use mutex to sychronize data receiving across threads
    pthread_mutex_lock(&obj_ptr->mutex);
    printf("Thread %d: waiting for release\n", tid);

    pthread_cond_wait(&obj_ptr->cond, &obj_ptr->mutex);
    pthread_mutex_unlock(&obj_ptr->mutex); // unlocking for all other threads

    // use token to speed up
    //moodycamel::ProducerToken local_ptok(*message_queue_);
    moodycamel::ProducerToken *local_ptok = obj_ptr->rx_ptoks_[tid];

    char* buffer = (char*)obj_ptr->buffer_[tid];
    int* buffer_status = obj_ptr->buffer_status_[tid];
    int buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;
    double *frame_start = obj_ptr->frame_start_[tid];

    // downlink socket buffer
    char *tx_buffer_ptr = obj_ptr->tx_buffer_;
    char *tx_cur_buffer_ptr;
    size_t txSymsPerFrame = cfg->dlSymsPerFrame;
    std::vector<size_t> txSymbols = cfg->DLSymbols[0];
    std::vector<std::complex<int16_t>> zeros(cfg->sampsPerSymbol);

    // walk through all the pages
    double temp;
    for (int i = 0; i < 20; i++) {
        temp = frame_start[i * 512];
    }

    char* cur_ptr_buffer = buffer;
    int* cur_ptr_buffer_status = buffer_status;
    int nradio_per_thread = cfg->nRadios/obj_ptr->rx_thread_num_;
    int rem_thread_nradio = cfg->nRadios%obj_ptr->rx_thread_num_;//obj_ptr->thread_num_*(cfg->nRadios/obj_ptr->thread_num_);
    printf("receiver thread %d has %d radios\n", tid, nradio_cur_thread);
    RadioConfig *radio = obj_ptr->radioconfig_;

    // to handle second channel at each radio
    // this is assuming buffer_frame_num is at least 2 
    char* cur_ptr_buffer2;
    char* buffer2 = (char*)obj_ptr->buffer_[tid] + package_length; 
    int* buffer_status2 = obj_ptr->buffer_status_[tid] + 1;
    int *cur_ptr_buffer_status2 = buffer_status2;
    if (cfg->nChannels == 2) {
        cur_ptr_buffer2 = buffer2;
    }
    else {
        cur_ptr_buffer2 = (char*)calloc(package_length, sizeof(char)); 
    }
    int offset = 0;
    long long frameTime;
    int prev_frame_id = -1;

    while(cfg->running)
    {
        // if buffer is full, exit
        if(cur_ptr_buffer_status[0] == 1) {
            printf("Receive thread %d buffer full, offset: %d\n", tid, offset);
            //for (int l = 0 ; l < buffer_frame_num; l++) 
            //    printf("%d ", buffer_status[l]);
            //printf("\n\n");
            cfg->running = false;
            break;
        }
        int ant_id, frame_id, symbol_id;
        // receive data
        for (int it = 0 ; it < nradio_cur_thread; it++) // FIXME: this must be threaded
        {
            //int rid = tid * obj_ptr->radios_per_thread + it;
            int rid = (tid < rem_thread_nradio) ? tid * (nradio_per_thread + 1) + it : tid * (nradio_per_thread) + rem_thread_nradio + it ;
            // this is probably a really bad implementation, and needs to be revamped
            char * samp1 = cur_ptr_buffer + package_header_offset;
            char * samp2 = cur_ptr_buffer2 + package_header_offset;
            void *samp[2] = {(void*)samp1, (void*)samp2};
            while (cfg->running && radio->radioRx(rid, samp, frameTime) <= 0);
            frame_id = (int)(frameTime>>32);
            symbol_id = (int)((frameTime>>16)&0xFFFF);
            ant_id = rid * cfg->nChannels;
            int rx_symbol_id = cfg->getPilotSFIndex(frame_id, symbol_id);
            if (rx_symbol_id < 0)
                rx_symbol_id = cfg->getUlSFIndex(frame_id, symbol_id) + cfg->pilotSymsPerFrame;
            *((int *)cur_ptr_buffer) = frame_id;
            *((int *)cur_ptr_buffer + 1) = rx_symbol_id;
            *((int *)cur_ptr_buffer + 2) = 0; //cell_id 
            *((int *)cur_ptr_buffer + 3) = ant_id;
            if (cfg->nChannels == 2)
            {
                *((int *)cur_ptr_buffer2) = frame_id;
                *((int *)cur_ptr_buffer2 + 1) = rx_symbol_id;
                *((int *)cur_ptr_buffer2 + 2) = 0; //cell_id 
                *((int *)cur_ptr_buffer2 + 3) = ant_id + 1;
            }

        #if MEASURE_TIME
            // read information from received packet
            frame_id = *((int *)cur_ptr_buffer);
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
            cur_ptr_buffer_status[0] = 1; // has data, after it is read it should be set to 0
            cur_ptr_buffer_status = buffer_status + (cur_ptr_buffer_status - buffer_status + cfg->nChannels) % buffer_frame_num;
            cur_ptr_buffer = buffer + (cur_ptr_buffer - buffer + package_length * cfg->nChannels) % buffer_length;
            // push EVENT_RX_ENB event into the queue
            Event_data package_message;
            package_message.event_type = EVENT_PACKAGE_RECEIVED;
            // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
            //package_message.data = offset + tid * buffer_frame_num; // Note: offset < buffer_frame_num 
            package_message.data = generateOffset2d_setbits(tid, offset, 28);
            if ( !message_queue_->enqueue(*local_ptok, package_message ) ) {
                printf("socket message enqueue failed\n");
                exit(0);
            }
            if (cfg->nChannels == 2)
            {
                offset = cur_ptr_buffer_status2 - buffer_status; // offset is absolute 
                cur_ptr_buffer_status2[0] = 1; // has data, after doing fft, it is set to 0
                cur_ptr_buffer_status2 = buffer_status2 + (cur_ptr_buffer_status2 - buffer_status2 + cfg->nChannels) % buffer_frame_num;
                cur_ptr_buffer2 = buffer2 + (cur_ptr_buffer2 - buffer2 + package_length * cfg->nChannels) % buffer_length;
                // push EVENT_RX_ENB event into the queue
                Event_data package_message2;
                package_message2.event_type = EVENT_PACKAGE_RECEIVED;
                // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
                //package_message2.data = offset + tid * buffer_frame_num;
                package_message.data = generateOffset2d_setbits(tid, offset, 28);
                if ( !message_queue_->enqueue(*local_ptok, package_message2 ) ) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }
            }
        #if DEBUG_RECV
            printf("packageReceiver %d: receive frame_id %d, symbol_id %d, cell_id %d, ant_id %d, offset %d\n", tid, frame_id, rx_symbol_id, cell_id, ant_id, offset + tid*buffer_frame_num);
        #endif
        #if DEBUG_DOWNLINK && !SEPARATE_TX_RX
	    if (rx_symbol_id > 0) 
                continue;
            for (int sym_id = 0; sym_id < txSymsPerFrame; sym_id++)
            {
                symbol_id = txSymbols[sym_id];
		int tx_frame_id = frame_id + TX_FRAME_DELTA;
                void* txbuf[2];
                long long frameTime = ((long long)tx_frame_id << 32) | (symbol_id << 16);
                int flags = 1; // HAS_TIME
                if (symbol_id == txSymbols.back()) flags = 2; // HAS_TIME & END_BURST, fixme
	    	if (ant_id != cfg->ref_ant)
	    	    txbuf[0] = zeros.data(); 
	    	else if (cfg->getDownlinkPilotId(frame_id, symbol_id) >= 0)
                        txbuf[0] = cfg->pilot_ci16.data();
	    	else
                        txbuf[0] = (void *)cfg->dl_IQ_symbol[sym_id];
                radio->radioTx(ant_id/cfg->nChannels, txbuf, flags, frameTime);
            }
        #endif
        }
    }
    return 0;
}
#endif


void* PackageReceiver::loopSend(void *in_context)
{


    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    Config *cfg = obj_ptr->config_;
    printf("package sender thread %d start\n", tid);

    moodycamel::ConcurrentQueue<Event_data> *task_queue_ = obj_ptr->task_queue_;
    // get pointer to message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->tx_core_id_;
#ifndef USE_DPDK
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

    int BS_ANT_NUM = obj_ptr->BS_ANT_NUM;
    int UE_NUM = obj_ptr->UE_NUM;
    // int OFDM_CA_NUM = obj_ptr->OFDM_CA_NUM;
    // int OFDM_DATA_NUM = obj_ptr->OFDM_DATA_NUM;
    // int subframe_num_perframe = obj_ptr->subframe_num_perframe;
    int data_subframe_num_perframe = obj_ptr->data_subframe_num_perframe;
    // int ul_data_subframe_num_perframe = obj_ptr->ul_data_subframe_num_perframe;
    // int dl_data_subframe_num_perframe = obj_ptr->dl_data_subframe_num_perframe;
    int package_length = obj_ptr->package_length;


#if USE_IPV4
    struct sockaddr_in servaddr_local;
    struct sockaddr_in cliaddr_local;
    int socket_local;
    servaddr_local.sin_family = AF_INET;
    servaddr_local.sin_port = htons(6000+tid);
    servaddr_local.sin_addr.s_addr = inet_addr(cfg->tx_addr.c_str());//inet_addr("10.225.92.16");//inet_addr("127.0.0.1");
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
    char *dl_buffer = obj_ptr->tx_buffer_;
    // downlink socket buffer status
    // int *dl_buffer_status = obj_ptr->tx_buffer_status_;
    // downlink data buffer
    // float *dl_data_buffer = obj_ptr->tx_data_buffer_;
    // buffer_length: package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    // int dl_buffer_length = obj_ptr->tx_buffer_length_;
    // buffer_frame_num: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    // int dl_buffer_frame_num = obj_ptr->tx_buffer_frame_num_;



    // auto begin = std::chrono::system_clock::now();
    // int package_count = 0;
    int ret;
    int offset;
    char *cur_buffer_ptr;
    // int *cur_ptr_buffer_status;
    int ant_id, frame_id, subframe_id, total_data_subframe_id, current_data_subframe_id;
    int cell_id = 0;
    // int maxMesgQLen = 0;
    // int maxTaskQLen = 0;

    // use token to speed up
    moodycamel::ProducerToken *local_ptok = obj_ptr->rx_ptoks_[tid];
    // moodycamel::ProducerToken local_ptok(*message_queue_);
    moodycamel::ConsumerToken local_ctok(*task_queue_);
    while(true) {
    
        Event_data task_event;
        // ret = task_queue_->try_dequeue(task_event); 
        ret = task_queue_->try_dequeue_from_producer(*(obj_ptr->tx_ptoks_[tid]),task_event); 
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
        // int data_subframe_offset = frame_id * data_subframe_num_perframe + current_data_subframe_id;
        cur_buffer_ptr = dl_buffer + (socket_subframe_offset * BS_ANT_NUM + ant_id) * package_length;  
        // cur_ptr_data = (dl_data_buffer + 2 * data_subframe_offset * OFDM_CA_NUM * BS_ANT_NUM);   
        *((int *)cur_buffer_ptr) = frame_id;
        *((int *)cur_buffer_ptr + 1) = subframe_id;
        *((int *)cur_buffer_ptr + 2) = cell_id;
        *((int *)cur_buffer_ptr + 3) = ant_id;

        // send data (one OFDM symbol)
        if (sendto(socket_local, (char*)cur_buffer_ptr, package_length, 0, (struct sockaddr *)&servaddr_local, sizeof(servaddr_local)) < 0) {
            perror("socket sendto failed");
            exit(0);
        }

#if DEBUG_BS_SENDER
        printf("In TX thread %d: Transmitted frame %d, subframe %d, ant %d, offset: %d, msg_queue_length: %d\n", tid, frame_id, subframe_id, ant_id, offset,
            message_queue_->size_approx());
#endif
        
        Event_data package_message;
        package_message.event_type = EVENT_PACKAGE_SENT;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        package_message.data = offset;
        if ( !message_queue_->enqueue(*local_ptok, package_message) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

        // if (package_count % (BS_ANT_NUM) == 0)
        // {
        //     usleep(71);
        // }

        // if(package_count == BS_ANT_NUM * dl_data_subframe_num_perframe * 1000)
        // {
        //     auto end = std::chrono::system_clock::now();
        //     double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * BS_ANT_NUM * data_subframe_num_perframe * 1000;
        //     std::chrono::duration<double> diff = end - begin;
        //     // printf("TX thread %d send 1000 frames in %f secs, throughput %f MB/s, max Queue Length: message %d, tx task %d\n", tid, diff.count(), byte_len / diff.count() / 1024 / 1024, maxMesgQLen, maxTaskQLen);
        //     printf("TX thread %d send 1000 frames in %f secs, throughput %f MB/s\n", tid, diff.count(), byte_len / diff.count() / 1024 / 1024);
        //     begin = std::chrono::system_clock::now();
        //     package_count = 0;
        // }
    }
    
}





void* PackageReceiver::loopTXRX(void *in_context)
{
    // get the pointer of class & tid
    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    printf("package TXRX thread %d start\n", tid);
    int core_id = obj_ptr->core_id_;
    int rx_thread_num = obj_ptr->rx_thread_num_;
    int tx_thread_num = obj_ptr->tx_thread_num_;
    Config *cfg = obj_ptr->config_;

#ifdef ENABLE_CPU_ATTACH 
    if(stick_this_thread_to_core(core_id + tid + 1) != 0) {
        printf("TXRX thread: attach thread %d to core %d failed\n", tid, core_id + tid + 1);
        exit(0);
    }
    else {
        printf("TXRX thread: attached thread %d to core %d\n", tid, core_id + tid + 1);
    }
#endif

    int BS_ANT_NUM = obj_ptr->BS_ANT_NUM;
    int UE_NUM = obj_ptr->UE_NUM;
    // int OFDM_CA_NUM = obj_ptr->OFDM_CA_NUM;
    // int OFDM_DATA_NUM = obj_ptr->OFDM_DATA_NUM;
    // int subframe_num_perframe = obj_ptr->subframe_num_perframe;
    int data_subframe_num_perframe = obj_ptr->data_subframe_num_perframe;
    int ul_data_subframe_num_perframe = obj_ptr->ul_data_subframe_num_perframe;
    int dl_data_subframe_num_perframe = obj_ptr->dl_data_subframe_num_perframe;
    bool downlink_mode = obj_ptr->downlink_mode;
    int package_length = obj_ptr->package_length;


#if USE_IPV4
    struct sockaddr_in local_addr;
    struct sockaddr_in remote_addr;
    
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(8000+tid);
    local_addr.sin_addr.s_addr = INADDR_ANY;//inet_addr("10.225.92.16");//inet_addr("127.0.0.1");
    memset(local_addr.sin_zero, 0, sizeof(local_addr.sin_zero)); 

    remote_addr.sin_family = AF_INET;
    remote_addr.sin_port = htons(7000+tid);
    remote_addr.sin_addr.s_addr = inet_addr(cfg->tx_addr.c_str());//inet_addr("10.225.92.16");//inet_addr("127.0.0.1");
    memset(remote_addr.sin_zero, 0, sizeof(remote_addr.sin_zero)); 

    int socket_local;
    if ((socket_local = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
        printf("TXRX thread %d cannot create IPV4 socket\n", tid);
        exit(0);
    }
    else{
        printf("TXRX thread %d created IPV4 socket\n", tid);
    }
#else
    struct sockaddr_in6 local_addr;
    struct sockaddr_in6 remote_addr;
    local_addr.sin6_family = AF_INET6;
    local_addr.sin6_addr = in6addr_any;
    local_addr.sin6_port = htons(8000+tid);

    remote_addr.sin6_family = AF_INET6;
    remote_addr.sin6_port = htons(7000+tid);
    inet_pton(AF_INET6, "fe80::5a9b:5a2f:c20a:d4d5", &remote_addr.sin6_addr);

    int socket_local;
    if ((socket_local = socket(AF_INET6, SOCK_DGRAM, 0)) < 0) { // UDP socket
        printf("TXRX thread %d cannot create IPV6 socket\n", tid);
        exit(0);
    }
    else{
        printf("TXRX thread %d Created IPV46 socket\n", tid);
    }
#endif

    // use SO_REUSEPORT option, so that multiple sockets could receive packets simultaneously, though the load is not balance
    int optval = 1;
    setsockopt(socket_local, SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));
    socklen_t optlen;
    int sock_buf_size;
    optlen = sizeof(sock_buf_size);

    sock_buf_size = 1024*1024*64*8-1;
    if (setsockopt(socket_local, SOL_SOCKET, SO_RCVBUF, &sock_buf_size, sizeof(sock_buf_size))<0) {
        printf("Error setting buffer size to %d\n", sock_buf_size);
    }
    else {     
        int res = getsockopt(socket_local, SOL_SOCKET, SO_RCVBUF, &sock_buf_size, &optlen);
        printf("Set socket %d buffer size to %d\n", tid, sock_buf_size);
    }

    if(bind(socket_local, (struct sockaddr *) &local_addr, sizeof(local_addr)) != 0) {
        printf("socket bind failed %d\n", tid);
        exit(0);
    }

    // get pointer of message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    moodycamel::ConcurrentQueue<Event_data> *task_queue_ = obj_ptr->task_queue_;
    // use token to speed up
    moodycamel::ProducerToken *local_ptok = obj_ptr->rx_ptoks_[tid];
    moodycamel::ConsumerToken local_ctok(*task_queue_);


    // RX  pointers
    char* rx_buffer_ptr = obj_ptr->buffer_[tid];
    int* rx_buffer_status_ptr = obj_ptr->buffer_status_[tid];
    long long rx_buffer_length = obj_ptr->buffer_length_;
    int rx_buffer_frame_num = obj_ptr->buffer_frame_num_;
    double *rx_frame_start = obj_ptr->frame_start_[tid];
    char* rx_cur_buffer_ptr = rx_buffer_ptr;
    int* rx_cur_buffer_status_ptr = rx_buffer_status_ptr;
    int rx_offset = 0;
    int ant_id, frame_id, subframe_id;


    // walk through all the pages
    double temp;
    for (int i = 0; i < 20; i++) {
        temp = rx_frame_start[i * 512];
    }


    // TX pointers
    char *tx_buffer_ptr = obj_ptr->tx_buffer_;
    // float *tx_data_buffer = obj_ptr->tx_data_buffer_;
    // buffer_frame_num: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    // int tx_buffer_frame_num = obj_ptr->tx_buffer_frame_num_;
    int ret;
    int tx_offset;
    char *tx_cur_buffer_ptr;
    // float *tx_cur_ptr_data;
    int tx_ant_id, tx_frame_id, tx_subframe_id, tx_current_data_subframe_id; //, tx_total_data_subframe_id;
    int tx_cell_id = 0;


    int max_subframe_id = downlink_mode ? UE_NUM : (UE_NUM + ul_data_subframe_num_perframe);
    int max_rx_packet_num_per_frame = max_subframe_id * BS_ANT_NUM / rx_thread_num;
    int max_tx_packet_num_per_frame = dl_data_subframe_num_perframe * BS_ANT_NUM / tx_thread_num;
    printf("Maximum RX pkts: %d, TX pkts: %d\n", max_rx_packet_num_per_frame, max_tx_packet_num_per_frame);
    int prev_frame_id = -1;
    int rx_packet_num_per_frame = 0;
    int tx_packet_num_per_frame = 0;
    int do_tx = 0;
    int rx_pkts_in_frame_count[10000];
    // int last_finished_frame_id = 0;

    int tx_pkts_in_frame_count[10000];
    // int last_finished_tx_frame_id = 0;


    // double start_time= get_time();

    if (!downlink_mode)
    {
        while(true) {
            // if buffer is full, exit
            if (rx_cur_buffer_status_ptr[0] == 1) {
                printf("Receive thread %d buffer full, offset: %d\n", tid, rx_offset);
                exit(0);
            }

            int recvlen = -1;

            // start_time= get_time();
            // if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)rx_cur_buffer_ptr, package_length, 0, (struct sockaddr *) &obj_ptr->servaddr_[tid], &addrlen)) < 0)
            if ((recvlen = recv(socket_local, (char*)rx_cur_buffer_ptr, package_length, 0))<0) {
            // if ((recvlen = recvfrom(socket_local, (char*)rx_cur_buffer_ptr, package_length, 0, (struct sockaddr *) &servaddr_local, &addrlen)) < 0) {
                perror("recv failed");
                exit(0);
            } 

            // rx_package_num_per_frame++;

        #if MEASURE_TIME
            // read information from received packet 
            frame_id = *((int *)rx_cur_buffer_ptr);
            subframe_id = *((int *)rx_cur_buffer_ptr + 1);
            ant_id = *((int *)rx_cur_buffer_ptr + 3);
            // printf("RX thread %d received frame %d subframe %d, ant %d\n", tid, frame_id, subframe_id, ant_id);
            if (frame_id > prev_frame_id) {
                *(rx_frame_start + frame_id) = get_time();
                prev_frame_id = frame_id;
                if (frame_id % 512 == 200) {
                    _mm_prefetch((char*)(rx_frame_start+frame_id+512), _MM_HINT_T0);
                    // double temp = frame_start[frame_id+3];
                }
            }
        #endif
            // get the position in buffer
            rx_offset = rx_cur_buffer_status_ptr - rx_buffer_status_ptr;
            // move ptr & set status to full
            rx_cur_buffer_status_ptr[0] = 1; // has data, after doing fft, it is set to 0
            rx_cur_buffer_status_ptr = rx_buffer_status_ptr + (rx_offset + 1) % rx_buffer_frame_num;
            rx_cur_buffer_ptr = rx_buffer_ptr + (rx_cur_buffer_ptr - rx_buffer_ptr + package_length) % rx_buffer_length;

            // push EVENT_PACKAGE_RECEIVED event into the queue
            Event_data rx_message;
            rx_message.event_type = EVENT_PACKAGE_RECEIVED;
            // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
            rx_message.data = generateOffset2d_setbits(tid, rx_offset, 28);
            // rx_message.data = rx_offset + tid * rx_buffer_frame_num;
            if ( !message_queue_->enqueue(*local_ptok, rx_message ) ) {
                printf("socket message enqueue failed\n");
                exit(0);
            }

        }
    }
    else
    {
        while(true) {
            if (do_tx == 0) {
                // if buffer is full, exit
                if (rx_cur_buffer_status_ptr[0] > 0) {
                    printf("Receive thread %d buffer full, offset: %d, buffer value: %d, total length: %d\n", tid, rx_offset, rx_cur_buffer_status_ptr[0], rx_buffer_frame_num);
                    printf("Buffer status:\n");
                    for(int i = 0; i < rx_buffer_frame_num; i++) 
                        printf("%d ", *(rx_buffer_status_ptr+i));
                    printf("\n");
                    exit(0);
                }

                int recvlen = -1;

                // start_time= get_time();
                // if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)rx_cur_buffer_ptr, package_length, 0, (struct sockaddr *) &obj_ptr->servaddr_[tid], &addrlen)) < 0)
                if ((recvlen = recv(socket_local, (char*)rx_cur_buffer_ptr, package_length, 0))<0) {
                // if ((recvlen = recvfrom(socket_local, (char*)rx_cur_buffer_ptr, package_length, 0, (struct sockaddr *) &servaddr_local, &addrlen)) < 0) {
                    perror("recv failed");
                    exit(0);
                } 

                rx_packet_num_per_frame++;

            #if MEASURE_TIME
                // read information from received packet 
                frame_id = *((int *)rx_cur_buffer_ptr);
                subframe_id = *((int *)rx_cur_buffer_ptr + 1);
                ant_id = *((int *)rx_cur_buffer_ptr + 3);
                rx_pkts_in_frame_count[frame_id % 10000]++;
                // printf("RX thread %d received frame %d subframe %d, ant %d offset %d\n", tid, frame_id, subframe_id, ant_id, rx_offset);
                // printf("RX thread %d received frame %d subframe %d, ant %d offset %d, buffer status %d %d ptr_offset %d\n", 
                    // tid, frame_id, subframe_id, ant_id, rx_offset, *(rx_buffer_status_ptr+1424), *(rx_cur_buffer_status_ptr+1), rx_cur_buffer_status_ptr - rx_buffer_status_ptr);
                if (frame_id > prev_frame_id) {
                    *(rx_frame_start + frame_id) = get_time();
                    prev_frame_id = frame_id;
                    if (frame_id % 512 == 200) {
                        _mm_prefetch((char*)(rx_frame_start+frame_id+512), _MM_HINT_T0);
                        // double temp = frame_start[frame_id+3];
                    }
                }
            #endif
                // get the position in buffer
                rx_offset = rx_cur_buffer_status_ptr - rx_buffer_status_ptr;
                // move ptr & set status to full
                rx_cur_buffer_status_ptr[0] = 1; // has data, after doing fft, it is set to 0
                rx_cur_buffer_status_ptr = rx_buffer_status_ptr + (rx_offset + 1) % rx_buffer_frame_num;
                rx_cur_buffer_ptr = rx_buffer_ptr + (rx_cur_buffer_ptr - rx_buffer_ptr + package_length) % rx_buffer_length;

                // push EVENT_PACKAGE_RECEIVED event into the queue
                Event_data rx_message;
                rx_message.event_type = EVENT_PACKAGE_RECEIVED;
                // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
                rx_message.data = generateOffset2d_setbits(tid, rx_offset, 28);
                // rx_message.data = rx_offset + tid * rx_buffer_frame_num;
                if ( !message_queue_->enqueue(*local_ptok, rx_message ) ) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }

                // if(rx_packet_num_per_frame == max_rx_packet_num_per_frame) {
                if(rx_pkts_in_frame_count[frame_id] == max_rx_packet_num_per_frame) {
                    do_tx = 1;
                    rx_packet_num_per_frame = 0;
                    rx_pkts_in_frame_count[frame_id] = 0;
                    // printf("In TXRX thread %d: RX finished frame %d, current frame %d\n", tid, last_finished_frame_id, prev_frame_id);
                    // last_finished_frame_id++;
                    
                }
            }
            else {
                Event_data task_event;
                // ret = task_queue_->try_dequeue(task_event); 
                ret = task_queue_->try_dequeue_from_producer(*(obj_ptr->tx_ptoks_[tid]),task_event); 
                if(!ret)
                    continue;
                // printf("tx queue length: %d\n", task_queue_->size_approx());
                if (task_event.event_type!=TASK_SEND) {
                    printf("Wrong event type!");
                    exit(0);
                }

                tx_offset = task_event.data;
                interpreteOffset3d(tx_offset, &tx_current_data_subframe_id, &tx_ant_id, &tx_frame_id);
                // tx_ant_id = tx_offset % BS_ANT_NUM;
                // tx_total_data_subframe_id = tx_offset / BS_ANT_NUM; 
                // tx_current_data_subframe_id = tx_total_data_subframe_id % data_subframe_num_perframe;
                tx_subframe_id = tx_current_data_subframe_id + UE_NUM;
                // tx_frame_id = tx_total_data_subframe_id / data_subframe_num_perframe;
                int tx_frame_id_in_buffer = tx_frame_id % SOCKET_BUFFER_FRAME_NUM;
                int socket_subframe_offset = tx_frame_id_in_buffer * data_subframe_num_perframe + tx_current_data_subframe_id;
                // int data_subframe_offset = tx_frame_id_in_buffer * data_subframe_num_perframe + tx_current_data_subframe_id;
                tx_cur_buffer_ptr = tx_buffer_ptr + (socket_subframe_offset * BS_ANT_NUM + tx_ant_id) * package_length;  
                // tx_cur_ptr_data = (tx_data_buffer + 2 * data_subframe_offset * OFDM_CA_NUM * BS_ANT_NUM);   
                *((int *)tx_cur_buffer_ptr) = tx_frame_id;
                *((int *)tx_cur_buffer_ptr + 1) = tx_subframe_id;
                *((int *)tx_cur_buffer_ptr + 2) = tx_cell_id;
                *((int *)tx_cur_buffer_ptr + 3) = tx_ant_id;
                
                // send data (one OFDM symbol)
                if (sendto(socket_local, (char*)tx_cur_buffer_ptr, package_length, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr)) < 0) {
                    perror("socket sendto failed");
                    exit(0);
                }
                tx_packet_num_per_frame++;
                tx_pkts_in_frame_count[tx_frame_id_in_buffer]++;


        #if DEBUG_BS_SENDER
                printf("In TX thread %d: Transmitted frame %d, subframe %d, ant %d, offset: %d, msg_queue_length: %d\n", tid, tx_frame_id, tx_subframe_id, tx_ant_id, tx_offset,
                    message_queue_->size_approx());
        #endif
                Event_data tx_message;
                tx_message.event_type = EVENT_PACKAGE_SENT;
                tx_message.data = tx_offset;
                if ( !message_queue_->enqueue(*local_ptok, tx_message ) ) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }
                // if (tx_packet_num_per_frame == max_tx_packet_num_per_frame) {
                if (tx_pkts_in_frame_count[tx_frame_id_in_buffer] == max_tx_packet_num_per_frame) {
                    do_tx = 0;
                    tx_packet_num_per_frame = 0;
                    tx_pkts_in_frame_count[tx_frame_id_in_buffer] = 0;
                    // printf("In TXRX thread %d: TX finished frame %d, current frame %d\n", tid, last_finished_tx_frame_id, prev_frame_id);
                    // prev_frame_id = tx_frame_id;
                    // last_finished_tx_frame_id = (last_finished_tx_frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                    
                }
            }  

        }

    }


}

#ifdef USE_ARGOS
void* PackageReceiver::loopSend_Argos(void *in_context)
{
    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    printf("package sender thread %d start\n", tid);

    moodycamel::ConcurrentQueue<Event_data> *task_queue_ = obj_ptr->task_queue_;
    // get pointer to message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->tx_core_id_;

#ifdef ENABLE_CPU_ATTACH
    if(pin_to_core(core_id + tid) != 0) {
        printf("TX thread: stitch thread %d to core %d failed\n", tid, core_id+ tid);
        exit(0);
    }
    else {
        printf("TX thread: stitch thread %d to core %d succeeded\n", tid, core_id + tid);
    }
#endif

    int BS_ANT_NUM = obj_ptr->BS_ANT_NUM;
    int UE_NUM = obj_ptr->UE_NUM;
    //int OFDM_CA_NUM = obj_ptr->OFDM_CA_NUM;
    //int OFDM_DATA_NUM = obj_ptr->OFDM_DATA_NUM;
    //int subframe_num_perframe = obj_ptr->subframe_num_perframe;
    int data_subframe_num_perframe = obj_ptr->data_subframe_num_perframe;
    //int ul_data_subframe_num_perframe = obj_ptr->ul_data_subframe_num_perframe;
    //int dl_data_subframe_num_perframe = obj_ptr->dl_data_subframe_num_perframe;
    int package_length = obj_ptr->package_length;
    int package_header_offset = obj_ptr->package_header_offset;

    // downlink socket buffer
    char *tx_buffer_ptr = obj_ptr->tx_buffer_;
    char *tx_cur_buffer_ptr;
    // buffer_frame_num: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    //int tx_buffer_frame_num = obj_ptr->tx_buffer_frame_num_;
    //int* buffer_status = obj_ptr->tx_buffer_status_;

    Config *cfg = obj_ptr->config_;
    RadioConfig *radio = obj_ptr->radioconfig_;

    int ret;
    int tx_offset;
    //int ant_id, symbol_id, frame_id;
    int frame_id, tx_ant_id, tx_frame_id, tx_subframe_id, tx_current_data_subframe_id;
    //struct timespec tv, tv2;

    //int txSymsPerFrame = 0;
    std::vector<size_t> txSymbols;
    if (cfg->isUE)
    {
        //txSymsPerFrame = cfg->ulSymsPerFrame;
        txSymbols = cfg->ULSymbols[0];
    }
    else
    {
        //txSymsPerFrame = cfg->dlSymsPerFrame;
        txSymbols = cfg->DLSymbols[0];
    }
    std::vector<std::complex<int16_t>> zeros(cfg->sampsPerSymbol);
    // use token to speed up
    // moodycamel::ProducerToken local_ptok(*message_queue_);
    //moodycamel::ConsumerToken local_ctok = (*task_queue_);
    // moodycamel::ProducerToken *local_ctok = (obj_ptr->task_ptok[tid]);
    moodycamel::ProducerToken *local_ptok = obj_ptr->rx_ptoks_[tid];
    while(cfg->running) {
    
        Event_data task_event;
        //ret = task_queue_->try_dequeue(task_event); 
        ret = task_queue_->try_dequeue_from_producer(*obj_ptr->tx_ptoks_[tid], task_event); 
        if(!ret)
            continue;



        // printf("tx queue length: %d\n", task_queue_->size_approx());
        if (task_event.event_type!=TASK_SEND) {
            printf("Wrong event type!");
            exit(0);
        }

        //ant_id = task_event.data; //% cfg->getNumAntennas();

        tx_offset = task_event.data;
        interpreteOffset3d(tx_offset, &tx_current_data_subframe_id, &tx_ant_id, &tx_frame_id);
        tx_subframe_id = tx_current_data_subframe_id + UE_NUM;
        int tx_frame_id_in_buffer = tx_frame_id % SOCKET_BUFFER_FRAME_NUM;
        int socket_subframe_offset = tx_frame_id_in_buffer * data_subframe_num_perframe + tx_current_data_subframe_id;
        tx_cur_buffer_ptr = tx_buffer_ptr + (socket_subframe_offset * BS_ANT_NUM + tx_ant_id) * package_length + package_header_offset; 
        frame_id = tx_frame_id + TX_FRAME_DELTA;

        //symbol_id = task_event.data / cfg->getNumAntennas();
        //for (symbol_id = 0; symbol_id < txSymsPerFrame; symbol_id++)
        //{
            size_t symbol_id = tx_subframe_id; //txSymbols[tx_subframe_id];
            void* txbuf[2];
            long long frameTime = ((long long)frame_id << 32) | (symbol_id << 16);
            int flags = 1; // HAS_TIME
            if (symbol_id == txSymbols.back()) flags = 2; // HAS_TIME & END_BURST, fixme
            if (cfg->nChannels == 1 || tx_ant_id % 2 == 0)
            {
            #if DEBUG_DOWNLINK
		if (tx_ant_id != cfg->ref_ant)
		    txbuf[0] = zeros.data(); 
		else if (cfg->getDownlinkPilotId(frame_id, symbol_id) >= 0)
                    txbuf[0] = cfg->pilot_ci16.data();
		else
                    txbuf[0] = (void *)cfg->dl_IQ_symbol[tx_current_data_subframe_id];
            #else
                txbuf[0] = tx_cur_buffer_ptr;
            #endif
                //buffer_status[offset] = 0;
            }
            else if (cfg->nChannels == 2 && tx_ant_id % 2 == 1)
            {
                txbuf[1] = tx_cur_buffer_ptr + package_length; //FIXME
                //buffer_status[offset+1] = 0;
            }
        #if DEBUG_BS_SENDER
            printf("In TX thread %d: Transmitted frame %d, subframe %d, ant %d, offset: %d, msg_queue_length: %zu\n", tid, frame_id, symbol_id, tx_ant_id, tx_offset,
                message_queue_->size_approx());
        #endif
            //clock_gettime(CLOCK_MONOTONIC, &tv);
        #if SEPARATE_TX_RX
            radio->radioTx(tx_ant_id/cfg->nChannels, txbuf, flags, frameTime);
        #endif
            //clock_gettime(CLOCK_MONOTONIC, &tv2);

        //}

        Event_data tx_message;
        tx_message.event_type = EVENT_PACKAGE_SENT;
        tx_message.data = tx_offset;
        if ( !message_queue_->enqueue(*local_ptok, tx_message ) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

    }
    return 0; 
}
#endif

#ifdef USE_DPDK
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

#endif

