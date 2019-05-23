/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */

#include "packageReceiver.hpp"
#include "cpu_attach.hpp"

static double get_time(void)
{
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return tv.tv_sec * 1000000 + tv.tv_nsec / 1000.0;
}

PackageReceiver::PackageReceiver(int RX_THREAD_NUM, int TX_THREAD_NUM)
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
    /* initialize random seed: */
    srand (time(NULL));
    rx_context = new PackageReceiverContext[rx_thread_num_];
    tx_context = new PackageReceiverContext[tx_thread_num_];

}

PackageReceiver::PackageReceiver(int RX_THREAD_NUM, int TX_THREAD_NUM, moodycamel::ConcurrentQueue<Event_data> * in_queue_message, moodycamel::ConcurrentQueue<Event_data> * in_queue_task):
PackageReceiver(RX_THREAD_NUM, TX_THREAD_NUM)
{
    message_queue_ = in_queue_message;
    task_queue_ = in_queue_task;
}

PackageReceiver::~PackageReceiver()
{
    delete[] socket_;
    delete[] tx_context;
    delete[] rx_context;
}

std::vector<pthread_t> PackageReceiver::startRecv(char** in_buffer, int** in_buffer_status, int in_buffer_frame_num, long long in_buffer_length, double **in_frame_start, int in_core_id)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == package_length * buffer_frame_num_); // should be integer
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status
    frame_start_ = in_frame_start;

    core_id_ = in_core_id;
    printf("start Recv thread\n");
    // new thread

#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(core_id_ - 1) != 0)
    {
        printf("RX: stitch main thread to core %d failed\n", core_id_-1);
        exit(0);
    }
    else{
        printf("RX: stitch main thread to core %d succeeded\n", core_id_-1);
    }
#endif
    
    std::vector<pthread_t> created_threads;
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
    
    return created_threads;
}





std::vector<pthread_t> PackageReceiver::startTX(char* in_buffer, int* in_buffer_status, float *in_data_buffer, int in_buffer_frame_num, int in_buffer_length, int in_core_id)
{
    // check length
    tx_buffer_frame_num_ = in_buffer_frame_num;
    // assert(in_buffer_length == package_length * buffer_frame_num_); // should be integer
    tx_buffer_length_ = in_buffer_length;
    tx_buffer_ = in_buffer;  // for save data
    tx_buffer_status_ = in_buffer_status; // for save status
    tx_data_buffer_ = in_data_buffer;

    // SOCKET_BUFFER_FRAME_NUM = 

    tx_core_id_ = in_core_id;
    printf("start Transmit thread\n");

    // create new threads
    std::vector<pthread_t> created_threads;
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
    
    return created_threads;
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
#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(core_id + tid) != 0)
    {
        printf("RX thread: attach thread %d to core %d failed\n", tid, core_id + tid);
        exit(0);
    }
    else{
        printf("RX thread: attached thread %d to core %d\n", tid, core_id + tid);
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

    int sock_buf_size = 1024*1024*64*8;
    if (setsockopt(socket_local, SOL_SOCKET, SO_RCVBUF, (void*)&sock_buf_size, sizeof(sock_buf_size))<0)
    {
        printf("Error setting buffer size to %d\n", sock_buf_size);
    }

    if(bind(socket_local, (struct sockaddr *) &servaddr_local, sizeof(servaddr_local)) != 0)
    {
        printf("socket bind failed %d\n", tid);
        exit(0);
    }


    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);

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
    socklen_t addrlen = sizeof(obj_ptr->servaddr_[tid]);
    int offset = 0;
    int package_num = 0;
    auto begin = std::chrono::system_clock::now();

    int maxQueueLength = 0;
    int ret = 0;
    int maxSymbolNum = 0;
    int Symbol_offset = 0;
    int max_subframe_id = ENABLE_DOWNLINK ? UE_NUM : subframe_num_perframe;
    int prev_frame_id = -1;
    double start_time= get_time();
    while(true)
    {
        // start_time= get_time();
        // if buffer is full, exit
        if (cur_ptr_buffer_status[0] == 1)
        {
            printf("Receive thread %d buffer full, offset: %d\n", tid, offset);
            exit(0);
        }

        // for (int i = 0; i < (package_length/64); i++)
        //     _mm256_stream_load_si256 ((__m256i const*) (cur_ptr_buffer + i*64));

        int recvlen = -1;
        // start_time= get_time();
        // if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &obj_ptr->servaddr_[tid], &addrlen)) < 0)
        if ((recvlen = recv(socket_local, (char*)cur_ptr_buffer, package_length, 0))<0) {
        // if ((recvlen = recvfrom(socket_local, (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &servaddr_local, &addrlen)) < 0) {
            perror("recv failed");
            exit(0);
        }



        // int count2;
        // ioctl(obj_ptr->socket_[tid], FIONREAD, &count2);
        
        // double cur_time = get_time();
        
        // get the position in buffer
        offset = cur_ptr_buffer_status - buffer_status;

        // read information from received packet
        int ant_id, frame_id, subframe_id, cell_id;
        frame_id = *((int *)cur_ptr_buffer);
        subframe_id = *((int *)cur_ptr_buffer + 1);
        // cell_id = *((int *)cur_ptr_buffer + 2);
        ant_id = *((int *)cur_ptr_buffer + 3);
        // printf("RX thread %d received frame %d subframe %d, ant %d\n", tid, frame_id, subframe_id, ant_id);
    #if MEASURE_TIME
        if (frame_id > prev_frame_id) {
            *(frame_start + frame_id) = get_time();
            prev_frame_id = frame_id;
            if (frame_id % 512 == 200) {
                _mm_prefetch((char*)(frame_start+frame_id+512), _MM_HINT_T0);
                // double temp = frame_start[frame_id+3];
            }
        }
    #endif
        // move ptr & set status to full
        cur_ptr_buffer_status[0] = 1; // has data, after doing fft, it is set to 0
        cur_ptr_buffer_status = buffer_status + (offset + 1) % buffer_frame_num;
        cur_ptr_buffer = buffer + (cur_ptr_buffer - buffer + package_length) % buffer_length;
        // push EVENT_PACKAGE_RECEIVED event into the queue
        Event_data package_message;
        package_message.event_type = EVENT_PACKAGE_RECEIVED;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        package_message.data = offset + tid * buffer_frame_num;
        if ( !message_queue_->enqueue(local_ptok, package_message ) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }

        
        // for (int i = 0; i < (package_length/64); i++)
        //     _mm_prefetch((char*)cur_ptr_buffer + 64 * i, _MM_HINT_NTA);

        
        
        // for (int i = 0; i < (package_length/64); i++)
            // _mm256_stream_load_si256 ((__m256i const*) (cur_ptr_buffer + i*64));
        //printf("enqueue offset %d\n", offset);
        // int cur_queue_len = message_queue_->size_approx();
        // maxQueueLength = maxQueueLength > cur_queue_len ? maxQueueLength : cur_queue_len;
        // int total_symbols = (frame_id * subframe_num_perframe + subframe_id) * BS_ANT_NUM + ant_id;

        // maxSymbolNum = (maxSymbolNum > total_symbols) ? maxSymbolNum : total_symbols;
        // printf("Frame %d, subframe %d, ant %d, recvlen: %d, total_symbols: %d, maxSymbolNum: %d, received %d\n", frame_id, subframe_id, ant_id, recvlen, total_symbols, maxSymbolNum, package_num);

        package_num++;
        // print some information
        if(package_num == BS_ANT_NUM * max_subframe_id * 1000)
        {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * BS_ANT_NUM * max_subframe_id * 1000;
            std::chrono::duration<double> diff = end - begin;
            // float package_loss_rate = 1.0e5 / (maxSymbolNum - Symbol_offset);

            // print network throughput & maximum message queue length during this period
            printf("RX thread %d receive 1000 frames in %f secs, throughput %f MB/s\n", tid, diff.count(), 
                byte_len / diff.count() / 1024 / 1024);
            // printf("RX thread %d receive %f bytes in %f secs, throughput %f MB/s, max Message Queue Length %d, package loss rate: 100000/%d = %.4f\n", tid, byte_len, diff.count(), 
                // byte_len / diff.count() / 1024 / 1024, maxQueueLength, (maxSymbolNum - Symbol_offset), package_loss_rate);
            // maxQueueLength = 0;
            // Symbol_offset = maxSymbolNum;
            begin = std::chrono::system_clock::now();
            package_num = 0;
        }
        // double cur_time = get_time();
        // printf("In RX thread %d: received frame %d, subframe %d, ant %d at %.2f, duration %.2f, recvlen: %d\n", tid, frame_id, subframe_id, ant_id, cur_time, cur_time-start_time, recvlen);
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

#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(core_id + tid) != 0) {
        printf("TX thread: attach thread %d to core %d failed\n", tid, core_id+ tid);
        exit(0);
    }
    else {
        printf("TX thread: attached thread %d to core %d\n", tid, core_id + tid);
    }
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

