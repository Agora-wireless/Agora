/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */

#include "packageReceiver.hpp"
#include "cpu_attach.hpp"
PackageReceiver::PackageReceiver(int N_THREAD)
{
    socket_ = new int[N_THREAD];
    /*Configure settings in address struct*/
    // address of sender 
    // servaddr_.sin_family = AF_INET;
    // servaddr_.sin_port = htons(7891);
    // servaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
    // memset(servaddr_.sin_zero, 0, sizeof(servaddr_.sin_zero));  
    
    for(int i = 0; i < N_THREAD; i++)
    {
        servaddr_[i].sin_family = AF_INET;
        servaddr_[i].sin_port = htons(8000+i);
        servaddr_[i].sin_addr.s_addr = INADDR_ANY;//inet_addr("10.225.92.16");//inet_addr("127.0.0.1");
        memset(servaddr_[i].sin_zero, 0, sizeof(servaddr_[i].sin_zero)); 

        if ((socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
            printf("cannot create socket %d\n", i);
            exit(0);
        }
        // use SO_REUSEPORT option, so that multiple sockets could receive packets simultaneously, though the load is not balance
        int optval = 1;
        setsockopt(socket_[i], SOL_SOCKET, SO_REUSEPORT, &optval, sizeof(optval));

        int sock_buf_size = 1024*1024*64*8;
        if (setsockopt(socket_[i], SOL_SOCKET, SO_RCVBUF, (void*)&sock_buf_size, sizeof(sock_buf_size))<0)
        {
            printf("Error setting buffer size to %d\n", sock_buf_size);
        }

        // int readValue = 0;
        // unsigned int readLen = sizeof(readValue);
        // int res = getsockopt( socket_[i], SOL_SOCKET, SO_RCVBUF, (void*)&readValue, &readLen );
        // if ( -1 == res )
        // {
        //     printf("ERROR reading socket buffer size\n");
        // }
        // else
        // {
        //     printf("Read socket buffer size:%d\n",readValue);
        // }

        if(bind(socket_[i], (struct sockaddr *) &servaddr_[i], sizeof(servaddr_[i])) != 0)
        {
            printf("socket bind failed %d\n", i);
            exit(0);
        }

    }
    

    thread_num_ = N_THREAD;
    /* initialize random seed: */
    srand (time(NULL));
    context = new PackageReceiverContext[thread_num_];

}

PackageReceiver::PackageReceiver(int N_THREAD, moodycamel::ConcurrentQueue<Event_data> * in_queue):
PackageReceiver(N_THREAD)
{
    message_queue_ = in_queue;
}

PackageReceiver::~PackageReceiver()
{
    delete[] socket_;
    delete[] context;
}

std::vector<pthread_t> PackageReceiver::startRecv(char** in_buffer, int** in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length == package_length * buffer_frame_num_); // should be integer
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status

    core_id_ = in_core_id;
    printf("start Recv thread\n");
    // new thread
    
    std::vector<pthread_t> created_threads;
    for(int i = 0; i < thread_num_; i++)
    {
        pthread_t recv_thread_;
        // record the thread id 
        context[i].ptr = this;
        context[i].tid = i;
        // start socket thread
        if(pthread_create( &recv_thread_, NULL, PackageReceiver::loopRecv, (void *)(&context[i])) != 0)
        {
            perror("socket recv thread create failed");
            exit(0);
        }
        created_threads.push_back(recv_thread_);
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
        printf("RX thread: stitch thread %d to core %d failed\n", tid, core_id + tid);
        exit(0);
    }
    else{
        printf("RX thread: stitch thread %d to core %d succeeded\n", tid, core_id + tid);
    }
#endif
    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);

    char* buffer = obj_ptr->buffer_[tid];
    int* buffer_status = obj_ptr->buffer_status_[tid];
    int buffer_length = obj_ptr->buffer_length_;
    int buffer_frame_num = obj_ptr->buffer_frame_num_;

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
    while(true)
    {
        // if buffer is full, exit
        if (cur_ptr_buffer_status[0] == 1)
        {
            printf("Receive thread %d buffer full, offset: %d\n", tid, offset);
            // int sum = 0;
            // printf("Status:\n");
            // for (int i = 0; i < BS_ANT_NUM * subframe_num_perframe; i++) {
            //     if ((i % (BS_ANT_NUM * subframe_num_perframe)) == 0) {
            //         printf("\n New frame\n");
            //     }
            //     if (i % (BS_ANT_NUM * subframe_num_perframe) == UE_NUM * BS_ANT_NUM) {
            //         printf("\nPilots ended\n");
            //     }
            //     sum += *(buffer_status + i);
            //     printf(" (%d,%d) ", i, *(buffer_status + i));
            // }
            // printf("\n Sum: %d, total: %d", sum, buffer_frame_num);
            exit(0);
        }
        // receive data
        int recvlen = -1;
        // int ant_id, frame_id, subframe_id, cell_id;
        if ((recvlen = recvfrom(obj_ptr->socket_[tid], (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *) &obj_ptr->servaddr_[tid], &addrlen)) < 0)
        {
            perror("recv failed");
            exit(0);
        }
        // // read information from received packet
        // frame_id = *((int *)cur_ptr_buffer);
        // subframe_id = *((int *)cur_ptr_buffer + 1);
        // cell_id = *((int *)cur_ptr_buffer + 2);
        // ant_id = *((int *)cur_ptr_buffer + 3);
       
        
        // get the position in buffer
        offset = cur_ptr_buffer_status - buffer_status;

        // printf("receive frame_id %d, subframe_id %d, cell_id %d, ant_id %d, offset %d\n", frame_id, subframe_id, cell_id, ant_id, offset);
        // move ptr & set status to full
        cur_ptr_buffer_status[0] = 1; // has data, after doing fft, it is set to 0
        cur_ptr_buffer_status = buffer_status + (cur_ptr_buffer_status - buffer_status + 1) % buffer_frame_num;
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
        //printf("enqueue offset %d\n", offset);
        // int cur_queue_len = message_queue_->size_approx();
        // maxQueueLength = maxQueueLength > cur_queue_len ? maxQueueLength : cur_queue_len;
        // int total_symbols = (frame_id * subframe_num_perframe + subframe_id) * BS_ANT_NUM + ant_id;

        // maxSymbolNum = (maxSymbolNum > total_symbols) ? maxSymbolNum : total_symbols;
        // printf("Frame %d, subframe %d, ant %d, recvlen: %d, total_symbols: %d, maxSymbolNum: %d, received %d\n", frame_id, subframe_id, ant_id, recvlen, total_symbols, maxSymbolNum, package_num);

        package_num++;
        // print some information
        if(package_num == BS_ANT_NUM * max_subframe_id * 100)
        {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * BS_ANT_NUM * max_subframe_id * 100;
            std::chrono::duration<double> diff = end - begin;
            // float package_loss_rate = 1.0e5 / (maxSymbolNum - Symbol_offset);

            // print network throughput & maximum message queue length during this period
            printf("RX thread %d receive 100 frames in %f secs, throughput %f MB/s\n", tid, diff.count(), 
                byte_len / diff.count() / 1024 / 1024);
            // printf("RX thread %d receive %f bytes in %f secs, throughput %f MB/s, max Message Queue Length %d, package loss rate: 100000/%d = %.4f\n", tid, byte_len, diff.count(), 
                // byte_len / diff.count() / 1024 / 1024, maxQueueLength, (maxSymbolNum - Symbol_offset), package_loss_rate);
            // maxQueueLength = 0;
            // Symbol_offset = maxSymbolNum;
            begin = std::chrono::system_clock::now();
            package_num = 0;
        }
    }

}
