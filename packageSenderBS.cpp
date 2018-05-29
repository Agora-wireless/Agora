#include "packageSenderBS.hpp"
#include "cpu_attach.hpp"

packageSenderBS::packageSenderBS(int N_THREAD)
{
    socket_ = new int[N_THREAD];

    /*Configure settings in address struct*/
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(7891);
    servaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
    memset(servaddr_.sin_zero, 0, sizeof(servaddr_.sin_zero));  

    for(int i = 0; i < N_THREAD; i++)
    {
        int rand_port = rand() % 65536;
        cliaddr_.sin_family = AF_INET;
        cliaddr_.sin_port = htons(0);  // out going port is random
        cliaddr_.sin_addr.s_addr = htons(INADDR_ANY);
        memset(cliaddr_.sin_zero, 0, sizeof(cliaddr_.sin_zero));  

        if ((socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
            printf("cannot create socket\n");
            exit(0);
        }
        else{
            printf("Created socket: %d\n",i);
        }

        /*Bind socket with address struct*/
        if(bind(socket_[i], (struct sockaddr *) &cliaddr_, sizeof(cliaddr_)) != 0)
            perror("socket bind failed");
    }


    thread_num_ = N_THREAD;
    /* initialize random seed: */
    srand (time(NULL));
    context = new PackageSenderContext[thread_num_];
    
}



packageSenderBS::packageSenderBS(int N_THREAD, moodycamel::ConcurrentQueue<Event_data> * in_queue_message, moodycamel::ConcurrentQueue<Event_data> * in_queue_task):
packageSenderBS(N_THREAD)
{
    message_queue_ = in_queue_message;
    task_queue_ = in_queue_task;
}

packageSenderBS::~packageSenderBS()
{
    delete[] socket_;
    delete[] context;
    // pthread_mutex_destroy(&lock_);
}



std::vector<pthread_t> packageSenderBS::startTX(char* in_buffer, int* in_buffer_status, int in_buffer_frame_num, int in_buffer_length, int in_core_id)
{
    // check length
    buffer_frame_num_ = in_buffer_frame_num;
    assert(in_buffer_length == package_length * buffer_frame_num_); // should be integer
    buffer_length_ = in_buffer_length;
    buffer_ = in_buffer;  // for save data
    buffer_status_ = in_buffer_status; // for save status

    core_id_ = in_core_id;
    printf("start Transmit thread\n");

    // create new threads
    std::vector<pthread_t> created_threads;
    for(int i = 0; i < thread_num_; i++)
    {
        pthread_t send_thread_;
        
        context[i].ptr = this;
        context[i].tid = i;

        if(pthread_create( &send_thread_, NULL, packageSenderBS::loopSend, (void *)(&context[i])) != 0)
        {
            perror("socket Transmit thread create failed");
            exit(0);
        }
        created_threads.push_back(send_thread_);
    }
       
    return created_threads;
}



void* packageSenderBS::loopSend(void *in_context)
{


    packageSenderBS* obj_ptr = ((PackageSenderContext *)in_context)->ptr;
    int tid = ((PackageSenderContext *)in_context)->tid;
    printf("package sender thread %d start\n", tid);

    moodycamel::ConcurrentQueue<Event_data> *task_queue_ = obj_ptr->task_queue_;
    // get pointer to message queue
    moodycamel::ConcurrentQueue<Event_data> *message_queue_ = obj_ptr->message_queue_;
    int core_id = obj_ptr->core_id_;

#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(core_id + tid) != 0)
    {
        printf("TX thread: stitch thread %d to core %d failed\n", tid, core_id+ tid);
        exit(0);
    }
    else
    {
        printf("TX thread: stitch thread %d to core %d succeeded\n", tid, core_id + tid);
    }
#endif

    // pointer to the downlink socket buffer
    // Frist dimension: SOCKET_TX_THREAD_NUM
    // Second dimension: package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    char* buffer = obj_ptr->buffer_;
    // pointer to the downlink socket buffer status
    // Frist dimension: SOCKET_TX_THREAD_NUM
    // Second dimension: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    int* buffer_status = obj_ptr->buffer_status_;
    // buffer_length: package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    int buffer_length = obj_ptr->buffer_length_;
    // buffer_frame_num: subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
    int buffer_frame_num = obj_ptr->buffer_frame_num_;



    auto begin = std::chrono::system_clock::now();
    int package_count = 0;
    int ret;
    int offset;
    char* cur_ptr_buffer;
    int* cur_ptr_buffer_status;

    // use token to speed up
    moodycamel::ProducerToken local_ptok(*message_queue_);
    moodycamel::ConsumerToken local_ctok(*task_queue_);
    while(true)
    {
    
        Event_data task_event;
        ret = task_queue_->try_dequeue(task_event); 
        if(!ret)
            continue;
        if (task_event.event_type!=TASK_SEND)
        {
            printf("Wrong event type!");
            exit(0);
        }

        offset = task_event.data;
        cur_ptr_buffer = buffer+offset*package_length;       
        cur_ptr_buffer_status = buffer_status+offset;

        // if buffer is empty, exit
        if(cur_ptr_buffer_status[0] == 0)
        {
            printf("thread %d buffer empty\n", tid);
            exit(0);
        }

        // send data (one OFDM symbol)
        if (sendto(obj_ptr->socket_[tid], (char*)cur_ptr_buffer, package_length, 0, (struct sockaddr *)&obj_ptr->servaddr_, sizeof(obj_ptr->servaddr_)) < 0) 
        {
            perror("socket sendto failed");
            exit(0);
        }

        if (DEBUG_BS_SENDER) 
        {
            // int* ptr = (int *)cur_ptr_buffer;
            // printf("Transmit frame %d, subframe %d, ant %d\n", *ptr, *(ptr+1), *(ptr+3));
        
            int ant_id, frame_id, subframe_id, cell_id;
            
            // read information from received packet
            frame_id = *((int *)cur_ptr_buffer);
            subframe_id = *((int *)cur_ptr_buffer + 1);
            cell_id = *((int *)cur_ptr_buffer + 2);
            ant_id = *((int *)cur_ptr_buffer + 3);
            printf("Transmitted frame %d, subframe %d, ant %d\n", frame_id, subframe_id, ant_id);
        }
        
        Event_data package_message;
        package_message.event_type = EVENT_PACKAGE_SENT;
        // data records the position of this packet in the buffer & tid of this socket (so that task thread could know which buffer it should visit) 
        package_message.data = offset;
        if ( !message_queue_->enqueue(local_ptok, package_message ) ) {
            printf("socket message enqueue failed\n");
            exit(0);
        }
        //printf("enqueue offset %d\n", offset);
        int cur_queue_len = message_queue_->size_approx();

        package_count++;

        // if (package_count % (BS_ANT_NUM) == 0)
        // {
        //     usleep(71);
        // }

        if(package_count == (int)1e5)
        {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * 1e5;
            std::chrono::duration<double> diff = end - begin;
            printf("thread %d send %f bytes in %f secs, throughput %f MB/s\n", tid, byte_len, diff.count(), byte_len / diff.count() / 1024 / 1024);
            begin = std::chrono::system_clock::now();
            package_count = 0;
        }
    }
    
}
