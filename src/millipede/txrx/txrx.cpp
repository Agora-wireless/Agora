/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */

#include "txrx.hpp"

PacketTXRX::PacketTXRX(Config *cfg, int RX_THREAD_NUM, int TX_THREAD_NUM, int in_core_offset)
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
    packet_length = cfg->packet_length;
    packet_header_offset = cfg->packet_header_offset;

    /* initialize random seed: */
    srand (time(NULL));
    rx_context = new EventHandlerContext<PacketTXRX>[rx_thread_num_];
    tx_context = new EventHandlerContext<PacketTXRX>[tx_thread_num_];
}

PacketTXRX::PacketTXRX(Config *cfg, int RX_THREAD_NUM, int TX_THREAD_NUM, int in_core_offset, 
        moodycamel::ConcurrentQueue<Event_data> * in_queue_message, moodycamel::ConcurrentQueue<Event_data> * in_queue_task, 
        moodycamel::ProducerToken **in_rx_ptoks, moodycamel::ProducerToken **in_tx_ptoks):
PacketTXRX(cfg, RX_THREAD_NUM, TX_THREAD_NUM, in_core_offset)
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
    delete config_;
}


std::vector<pthread_t> PacketTXRX::startRecv(char** in_buffer, int** in_buffer_status, int in_buffer_frame_num, long long in_buffer_length, double **in_frame_start)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == packet_length * buffer_frame_num_); // should be integer
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status
    frame_start_ = in_frame_start;


    printf("create RX threads\n");
    // new thread
    // pin_to_core_with_offset(RX, core_id_, 0);

    std::vector<pthread_t> created_threads;
    if (!downlink_mode) {   
        for(int i = 0; i < rx_thread_num_; i++) {
            pthread_t recv_thread_;
            // record the thread id 
            rx_context[i].obj_ptr = this;
            rx_context[i].id = i;
            // start socket thread
            if (pthread_create(&recv_thread_, NULL, pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopRecv>, &rx_context[i]) != 0) {
                perror("socket recv thread create failed");
                exit(0);
            }
            created_threads.push_back(recv_thread_);
        }
    }
    return created_threads;
}





std::vector<pthread_t> PacketTXRX::startTX(char* in_buffer, int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length)
{
    // check length
    tx_buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == packet_length * buffer_frame_num_); // should be integer
    tx_buffer_length_ = in_buffer_length;
    tx_buffer_ = in_buffer;  // for save data
    tx_buffer_status_ = in_buffer_status; // for save status

    printf("create TX or TXRX threads\n");
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
            tx_context[thread_id].obj_ptr = this;
            tx_context[thread_id].id = thread_id;
            rte_eal_remote_launch((lcore_function_t *)loopSend,
                                &tx_context[thread_id], lcore_id);
            printf("TX: launched thread %d on core %d\n", thread_id, lcore_id);
        }
        worker_id++;
    }
#else
    
    for (int i = 0; i < tx_thread_num_; i++) {
        pthread_t send_thread_;
        
        tx_context[i].obj_ptr = this;
        tx_context[i].id = i;
        if (pthread_create(&send_thread_, NULL, pthread_fun_wrapper<PacketTXRX, &PacketTXRX::loopTXRX>, &tx_context[i]) != 0)
        // if (pthread_create( &send_thread_, NULL, PacketTXRX::loopTXRX, (void *)(&tx_context[i])) != 0)
        {
            perror("socket Transmit thread create failed");
            exit(0);
        }
        
        created_threads.push_back(send_thread_);
    }
#endif
    
    return created_threads;
}


void *PacketTXRX::loopRecv(int tid)
{
    pin_to_core_with_offset(Worker_RX, core_id_, tid);
    int sock_buf_size = 1024*1024*64*8-1;
    int local_port_id = 8000 + tid;
#if USE_IPV4
    int socket_local = setup_socket_ipv4(local_port_id, sock_buf_size);
    // struct sockaddr_in local_addr;
    // setup_sockaddr_local_ipv4(&local_addr, local_port_id);
#else
    int socket_local = setup_socket_ipv6(local_port_id, sock_buf_size);
    // struct sockaddr_in6 local_addr;
    // setup_sockaddr_local_ipv6(&local_addr, local_port_id);
#endif

    // use token to speed up
    // moodycamel::ProducerToken local_ptok(*message_queue_);
    // moodycamel::ProducerToken *local_ptok = new moodycamel::ProducerToken(*message_queue_);
    moodycamel::ProducerToken *local_ptok = rx_ptoks_[tid];

    char *buffer_ptr = buffer_[tid];
    int *buffer_status_ptr = buffer_status_[tid];
    long long buffer_length = buffer_length_;
    int buffer_frame_num = buffer_frame_num_;
    double *frame_start = frame_start_[tid];

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
    // int packet_num = 0;
    // int ret = 0;
    // int max_subframe_id = downlink_mode ? UE_NUM : subframe_num_perframe;
    int prev_frame_id = -1;
    // int packet_num_per_frame = 0;
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
        // if ((recvlen = recvfrom(socket_[tid], (char*)cur_ptr_buffer, packet_length, 0, (struct sockaddr *) &servaddr_[tid], &addrlen)) < 0)
        if ((recvlen = recv(socket_local, (char*)cur_buffer_ptr, packet_length, 0))<0) {
        // if ((recvlen = recvfrom(socket_local, (char*)cur_ptr_buffer, packet_length, 0, (struct sockaddr *) &local_addr, &addrlen)) < 0) {
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
        cur_buffer_ptr = buffer_ptr + (cur_buffer_ptr - buffer_ptr + packet_length) % buffer_length;
        // push EVENT_packet_RECEIVED event into the queue
        Event_data packet_message;
        packet_message.event_type = EVENT_PACKET_RECEIVED;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        packet_message.data = generateOffset2d_setbits(tid, offset, 28);
        // packet_message.data = offset + tid * buffer_frame_num;
        // if ( !message_queue_->enqueue(packet_message ) ) {
        if ( !message_queue_->enqueue(*local_ptok, packet_message) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
    }
}


void *PacketTXRX::loopSend(int tid)
{
    pin_to_core_with_offset(Worker_TX, tx_core_id_, tid);
    int sock_buf_size = 1024*1024*64*8-1;
    int local_port_id = 0;
    int remote_port_id = 7000 + tid;
#if USE_IPV4
    int socket_local = setup_socket_ipv4(local_port_id, sock_buf_size);
    struct sockaddr_in remote_addr;
    setup_sockaddr_remote_ipv4(&remote_addr, remote_port_id, config_->tx_addr.c_str());
#else
    int socket_local = setup_socket_ipv6(local_port_id, sock_buf_size);
    struct sockaddr_in6 remote_addr;
    setup_sockaddr_remote_ipv6(&remote_addr, remote_port_id, config_->tx_addr.c_str());
#endif

    // downlink socket buffer
    char *dl_buffer = tx_buffer_;
    

    // auto begin = std::chrono::system_clock::now();
    // int packet_count = 0;
    int ret;
    int offset;
    char *cur_buffer_ptr;
    // int *cur_ptr_buffer_status;
    int ant_id, frame_id, symbol_id, total_data_subframe_id, current_data_subframe_id;
    int cell_id = 0;
    // int maxMesgQLen = 0;
    // int maxTaskQLen = 0;

    // use token to speed up
    moodycamel::ProducerToken *local_ptok = rx_ptoks_[tid];
    // moodycamel::ProducerToken local_ptok(*message_queue_);
    moodycamel::ConsumerToken local_ctok(*task_queue_);
    while(true) {
    
        Event_data task_event;
        // ret = task_queue_->try_dequeue(task_event); 
        ret = task_queue_->try_dequeue_from_producer(*(tx_ptoks_[tid]),task_event); 
        if(!ret)
            continue;
        // printf("tx queue length: %d\n", task_queue_->size_approx());
        if (task_event.event_type!=TASK_SEND) {
            printf("Wrong event type!");
            exit(0);
        }

        // printf("In transmitter\n");

        offset = task_event.data;
        interpreteOffset3d(offset, &current_data_subframe_id, &ant_id, &frame_id);
        symbol_id = current_data_subframe_id + UE_NUM;

        int socket_subframe_offset = frame_id * data_subframe_num_perframe + current_data_subframe_id;
        // int data_subframe_offset = frame_id * data_subframe_num_perframe + current_data_subframe_id;
        cur_buffer_ptr = dl_buffer + (socket_subframe_offset * BS_ANT_NUM + ant_id) * packet_length;  
        // cur_ptr_data = (dl_data_buffer + 2 * data_subframe_offset * OFDM_CA_NUM * BS_ANT_NUM);   
        struct Packet *pkt = (struct Packet *)cur_buffer_ptr;
        new (pkt) Packet(frame_id, symbol_id, 0 /* cell_id */, ant_id);

        // send data (one OFDM symbol)
        if (sendto(socket_local, (char*)cur_buffer_ptr, packet_length, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr)) < 0) {
            perror("socket sendto failed");
            exit(0);
        }

#if DEBUG_BS_SENDER
        printf("In TX thread %d: Transmitted frame %d, subframe %d, ant %d, offset: %d, msg_queue_length: %d\n", tid, frame_id, symbol_id, ant_id, offset,
            message_queue_->size_approx());
#endif
        
        Event_data packet_message;
        packet_message.event_type = EVENT_PACKET_SENT;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        packet_message.data = offset;
        if ( !message_queue_->enqueue(*local_ptok, packet_message) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

        // if (packet_count % (BS_ANT_NUM) == 0)
        // {
        //     usleep(71);
        // }

        // if(packet_count == BS_ANT_NUM * dl_data_subframe_num_perframe * 1000)
        // {
        //     auto end = std::chrono::system_clock::now();
        //     double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * BS_ANT_NUM * data_subframe_num_perframe * 1000;
        //     std::chrono::duration<double> diff = end - begin;
        //     // printf("TX thread %d send 1000 frames in %f secs, throughput %f MB/s, max Queue Length: message %d, tx task %d\n", tid, diff.count(), byte_len / diff.count() / 1024 / 1024, maxMesgQLen, maxTaskQLen);
        //     printf("TX thread %d send 1000 frames in %f secs, throughput %f MB/s\n", tid, diff.count(), byte_len / diff.count() / 1024 / 1024);
        //     begin = std::chrono::system_clock::now();
        //     packet_count = 0;
        // }
    }   
}





void *PacketTXRX::loopTXRX(int tid)
{
    pin_to_core_with_offset(Worker_TXRX, core_id_, tid);
    int sock_buf_size = 1024*1024*64*8-1;
    int local_port_id = 8000 + tid;
    int remote_port_id = 7000 + tid;
#if USE_IPV4
    int socket_local = setup_socket_ipv4(local_port_id, sock_buf_size);
    struct sockaddr_in remote_addr;
    setup_sockaddr_remote_ipv4(&remote_addr, remote_port_id, config_->tx_addr.c_str());
    // struct sockaddr_in local_addr;
    // setup_sockaddr_local_ipv4(&local_addr, local_port_id);
#else
    int socket_local = setup_socket_ipv6(local_port_id, sock_buf_size);
    struct sockaddr_in6 remote_addr;
    setup_sockaddr_remote_ipv6(&remote_addr, remote_port_id, config_->tx_addr.c_str());
    // struct sockaddr_in6 local_addr;
    // setup_sockaddr_local_ipv6(&local_addr, local_port_id);
#endif

    // use token to speed up
    moodycamel::ProducerToken *local_ptok = rx_ptoks_[tid];
    moodycamel::ConsumerToken local_ctok(*task_queue_);


    // RX  pointers
    char* rx_buffer_ptr = buffer_[tid];
    int* rx_buffer_status_ptr = buffer_status_[tid];
    long long rx_buffer_length = buffer_length_;
    int rx_buffer_frame_num = buffer_frame_num_;
    double *rx_frame_start = frame_start_[tid];
    char* rx_cur_buffer_ptr = rx_buffer_ptr;
    int* rx_cur_buffer_status_ptr = rx_buffer_status_ptr;
    int rx_offset = 0;
    int ant_id, frame_id, symbol_id;


    // walk through all the pages
    double temp;
    for (int i = 0; i < 20; i++) {
        temp = rx_frame_start[i * 512];
    }


    // TX pointers
    char *tx_buffer_ptr = tx_buffer_;
    // float *tx_data_buffer = obj_ptr->tx_data_buffer_;
    // buffer_frame_num: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    int ret;
    int tx_offset;
    char *tx_cur_buffer_ptr;
    // float *tx_cur_ptr_data;
    int tx_ant_id, tx_frame_id, tx_symbol_id, tx_current_data_subframe_id; //, tx_total_data_subframe_id;


    int max_subframe_id = downlink_mode ? UE_NUM : (UE_NUM + ul_data_subframe_num_perframe);
    int max_rx_packet_num_per_frame = max_subframe_id * BS_ANT_NUM / rx_thread_num_;
    int max_tx_packet_num_per_frame = dl_data_subframe_num_perframe * BS_ANT_NUM / tx_thread_num_;
    printf("Maximum RX pkts: %d, TX pkts: %d\n", max_rx_packet_num_per_frame, max_tx_packet_num_per_frame);
    int prev_frame_id = -1;
    int rx_packet_num_per_frame = 0;
    int tx_packet_num_per_frame = 0;
    int do_tx = 0;
    int rx_pkts_in_frame_count[10000] = {0};
    // int last_finished_frame_id = 0;

    int tx_pkts_in_frame_count[10000] = {0};
    // int last_finished_tx_frame_id = 0;


    // double start_time= get_time();

    if (!downlink_mode) {
        while(true) {
            // if buffer is full, exit
            if (rx_cur_buffer_status_ptr[0] == 1) {
                printf("Receive thread %d buffer full, offset: %d\n", tid, rx_offset);
                exit(0);
            }

            int recvlen = -1;

            // start_time= get_time();
            // if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)rx_cur_buffer_ptr, packet_length, 0, (struct sockaddr *) &obj_ptr->servaddr_[tid], &addrlen)) < 0)
            if ((recvlen = recv(socket_local, (char*)rx_cur_buffer_ptr, packet_length, 0))<0) {
            // if ((recvlen = recvfrom(socket_local, (char*)rx_cur_buffer_ptr, packet_length, 0, (struct sockaddr *) &local_addr, &addrlen)) < 0) {
                perror("recv failed");
                exit(0);
            } 

            // rx_packet_num_per_frame++;

        #if MEASURE_TIME
            // read information from received packet 
            struct Packet *pkt = (struct Packet *)rx_cur_buffer_ptr;
            frame_id = pkt->frame_id;
            symbol_id = pkt->symbol_id;
            ant_id = pkt->ant_id;
            // printf("RX thread %d received frame %d subframe %d, ant %d\n", tid, frame_id, symbol_id, ant_id);
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
            rx_cur_buffer_ptr = rx_buffer_ptr + (rx_cur_buffer_ptr - rx_buffer_ptr + packet_length) % rx_buffer_length;

            // push EVENT_PACKET_RECEIVED event into the queue
            Event_data rx_message;
            rx_message.event_type = EVENT_PACKET_RECEIVED;
            // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
            rx_message.data = generateOffset2d_setbits(tid, rx_offset, 28);
            // rx_message.data = rx_offset + tid * rx_buffer_frame_num;
            if ( !message_queue_->enqueue(*local_ptok, rx_message ) ) {
                printf("socket message enqueue failed\n");
                exit(0);
            }

        }
    }
    else {
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
                // if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)rx_cur_buffer_ptr, packet_length, 0, (struct sockaddr *) &obj_ptr->servaddr_[tid], &addrlen)) < 0)
                if ((recvlen = recv(socket_local, (char*)rx_cur_buffer_ptr, packet_length, 0))<0) {
                // if ((recvlen = recvfrom(socket_local, (char*)rx_cur_buffer_ptr, packet_length, 0, (struct sockaddr *) &servaddr_local, &addrlen)) < 0) {
                    perror("recv failed");
                    exit(0);
                } 

                rx_packet_num_per_frame++;
                
                struct Packet *pkt = (struct Packet *)rx_cur_buffer_ptr;
                frame_id = pkt->frame_id;
                symbol_id = pkt->symbol_id;
                ant_id = pkt->ant_id;

            #if MEASURE_TIME
                // printf("RX thread %d received frame %d subframe %d, ant %d offset %d\n", tid, frame_id, symbol_id, ant_id, rx_offset);
                // printf("RX thread %d received frame %d subframe %d, ant %d offset %d, buffer status %d %d ptr_offset %d\n", 
                    // tid, frame_id, symbol_id, ant_id, rx_offset, *(rx_buffer_status_ptr+1424), *(rx_cur_buffer_status_ptr+1), rx_cur_buffer_status_ptr - rx_buffer_status_ptr);
                if (frame_id > prev_frame_id) {
                    *(rx_frame_start + frame_id) = get_time();
                    prev_frame_id = frame_id;
                    if (frame_id % 512 == 200) {
                        _mm_prefetch((char*)(rx_frame_start+frame_id+512), _MM_HINT_T0);
                    }
                }
            #endif
                
                // get the position in buffer
                rx_offset = rx_cur_buffer_status_ptr - rx_buffer_status_ptr;
                // move ptr & set status to full
                rx_cur_buffer_status_ptr[0] = 1; // has data, after doing fft, it is set to 0
                rx_cur_buffer_status_ptr = rx_buffer_status_ptr + (rx_offset + 1) % rx_buffer_frame_num;
                rx_cur_buffer_ptr = rx_buffer_ptr + (rx_cur_buffer_ptr - rx_buffer_ptr + packet_length) % rx_buffer_length;

                // push EVENT_PACKET_RECEIVED event into the queue
                Event_data rx_message;
                rx_message.event_type = EVENT_PACKET_RECEIVED;
                // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
                rx_message.data = generateOffset2d_setbits(tid, rx_offset, 28);
                if ( !message_queue_->enqueue(*local_ptok, rx_message ) ) {
                    printf("socket message enqueue failed\n");
                    exit(0);
                }

                rx_pkts_in_frame_count[frame_id % 10000]++;
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
                ret = task_queue_->try_dequeue_from_producer(*(tx_ptoks_[tid]),task_event); 
                if(!ret)
                    continue;
                // printf("tx queue length: %d\n", task_queue_->size_approx());
                if (task_event.event_type!=TASK_SEND) {
                    printf("Wrong event type!");
                    exit(0);
                }

                tx_offset = task_event.data;
                interpreteOffset3d(tx_offset, &tx_current_data_subframe_id, &tx_ant_id, &tx_frame_id);
                tx_symbol_id = tx_current_data_subframe_id + UE_NUM;
                int tx_frame_id_in_buffer = tx_frame_id % SOCKET_BUFFER_FRAME_NUM;
                int socket_subframe_offset = tx_frame_id_in_buffer * data_subframe_num_perframe + tx_current_data_subframe_id;
                // int data_subframe_offset = tx_frame_id_in_buffer * data_subframe_num_perframe + tx_current_data_subframe_id;
                tx_cur_buffer_ptr = tx_buffer_ptr + (socket_subframe_offset * BS_ANT_NUM + tx_ant_id) * packet_length;  
                // tx_cur_ptr_data = (tx_data_buffer + 2 * data_subframe_offset * OFDM_CA_NUM * BS_ANT_NUM);   
                struct Packet *pkt = (struct Packet *)tx_cur_buffer_ptr;
                new (pkt) Packet(tx_frame_id, tx_symbol_id, 0 /* cell_id */, tx_ant_id);
                
                // send data (one OFDM symbol)
                if (sendto(socket_local, (char*)tx_cur_buffer_ptr, packet_length, 0, (struct sockaddr *)&remote_addr, sizeof(remote_addr)) < 0) {
                    perror("socket sendto failed");
                    exit(0);
                }
                tx_packet_num_per_frame++;
                tx_pkts_in_frame_count[tx_frame_id_in_buffer]++;


        #if DEBUG_BS_SENDER
                printf("In TX thread %d: Transmitted frame %d, subframe %d, ant %d, offset: %d, msg_queue_length: %d\n", tid, tx_frame_id, tx_symbol_id, tx_ant_id, tx_offset,
                    message_queue_->size_approx());
        #endif
                Event_data tx_message;
                tx_message.event_type = EVENT_PACKET_SENT;
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



