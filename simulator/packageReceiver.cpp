/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */

#include "packageReceiver.hpp"
#include "cpu_attach.hpp"


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

    /* initialize random seed: */
    srand (time(NULL));
    rx_context = new PackageReceiverContext[rx_thread_num_];
    tx_context = new PackageReceiverContext[tx_thread_num_];
}

PackageReceiver::PackageReceiver(Config *cfg, int RX_THREAD_NUM, int TX_THREAD_NUM, int in_core_offset, 
        moodycamel::ConcurrentQueue<Event_data> * in_queue_message, moodycamel::ConcurrentQueue<Event_data> * in_queue_task, 
        moodycamel::ProducerToken **in_rx_ptoks):
PackageReceiver(cfg, RX_THREAD_NUM, TX_THREAD_NUM, in_core_offset)
{
    message_queue_ = in_queue_message;
    task_queue_ = in_queue_task;
    rx_ptoks_ = in_rx_ptoks;
    // tx_ptoks_ = in_tx_ptoks;

}

PackageReceiver::~PackageReceiver()
{
    delete[] socket_;
    delete[] tx_context;
    delete[] rx_context;
    delete config_;
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
        printf("RX: stitch RX main thread to core %d failed\n", core_id_);
        exit(0);
    }
    else{
        printf("RX: stitch RX main thread to core %d succeeded\n", core_id_);
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
            rte_eal_remote_launch((lcore_function_t *)loopRecv_DPDK,
                                &rx_context[worker_id], lcore_id);
            printf("RX: launched thread %d on core %d\n", worker_id, lcore_id);
        } 
        worker_id++;
    }
#else   
    
    for(int i = 0; i < rx_thread_num_; i++) {
        pthread_t recv_thread_;
        // record the thread id 
        rx_context[i].ptr = this;
        rx_context[i].tid = i;
        // start socket thread
        if(pthread_create( &recv_thread_, NULL, PackageReceiver::loopRecv, (void *)(&rx_context[i])) != 0) {
            perror("socket recv thread create failed");
            exit(0);
        }
        created_threads.push_back(recv_thread_);
    }
#endif
    return created_threads;
}


void* PackageReceiver::loopRecv(void *in_context)
{
    // get the pointer of class & tid
    PackageReceiver* obj_ptr = ((PackageReceiverContext *)in_context)->ptr;
    int tid = ((PackageReceiverContext *)in_context)->tid;
    printf("package receiver thread %d start\n", tid);


    int BS_ANT_NUM = obj_ptr->BS_ANT_NUM;
    int UE_NUM = obj_ptr->UE_NUM;
    int OFDM_CA_NUM = obj_ptr->OFDM_CA_NUM;
    int OFDM_DATA_NUM = obj_ptr->OFDM_DATA_NUM;
    int subframe_num_perframe = obj_ptr->subframe_num_perframe;
    int data_subframe_num_perframe = obj_ptr->data_subframe_num_perframe;
    int ul_data_subframe_num_perframe = obj_ptr->ul_data_subframe_num_perframe;
    int dl_data_subframe_num_perframe = obj_ptr->dl_data_subframe_num_perframe;
    int package_length = obj_ptr->package_length;
    bool downlink_mode = obj_ptr->downlink_mode;

    // get pointer of message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->core_id_;
    // if ENABLE_CPU_ATTACH is enabled, attach threads to specific cores


#ifdef ENABLE_CPU_ATTACH 
    int cur_core = core_id + tid + 2 + obj_ptr->rx_thread_num_;
    if (cur_core >= 36)
        cur_core -= 36;
    if(stick_this_thread_to_core(cur_core) != 0) {
        printf("RX thread: attach RX thread %d to core %d failed\n", tid, cur_core);
        exit(0);
    }
    else {
        printf("RX thread: attached RX thread %d to core %d\n", tid, cur_core);
    }
#endif



#if USE_IPV4
    struct sockaddr_in servaddr_local;
    int socket_local;
    servaddr_local.sin_family = AF_INET;
    servaddr_local.sin_port = htons(7000+tid);
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
    servaddr_local.sin6_port = htons(7000+tid);
    
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
    socklen_t addrlen = sizeof(servaddr_local);
    int offset = 0;
    int package_num = 0;
    int ret = 0;
    int max_subframe_id = downlink_mode ? UE_NUM : subframe_num_perframe;
    int prev_frame_id = -1;
    int package_num_per_frame = 0;
    double start_time= get_time();


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
        // if ((recvlen = recv(socket_local, (char*)cur_buffer_ptr, package_length, 0))<0) {
        if ((recvlen = recvfrom(socket_local, (char*)cur_buffer_ptr, package_length, 0, (struct sockaddr *) &servaddr_local, &addrlen)) < 0) {
            perror("recv failed");
            exit(0);
        } 

    #if MEASURE_TIME
        // read information from received packet
        int ant_id, frame_id, subframe_id, cell_id;
        frame_id = *((int *)cur_buffer_ptr);
        subframe_id = *((int *)cur_buffer_ptr + 1);
        // cell_id = *((int *)cur_buffer_ptr + 2);
        ant_id = *((int *)cur_buffer_ptr + 3);
        // printf("RX thread %d received frame %d subframe %d, ant %d, msg queue length %d\n", tid, frame_id, subframe_id, ant_id, message_queue_->size_approx());
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



