#include "packageSender.hpp"
#include "cpu_attach.hpp"


static double get_time(void)
{
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return tv.tv_sec * 1000000 + tv.tv_nsec / 1000.0;
}

PackageSender::PackageSender(int in_socket_num, int in_thread_num, int in_core_offset, int in_delay):
ant_id(0), frame_id(0), subframe_id(0), thread_num(in_thread_num), 
socket_num(in_socket_num), cur_ptr_(0), core_offset(in_core_offset), delay(in_delay)
{
    socket_ = new int[socket_num];

    /*Configure settings in address struct*/
    // servaddr_.sin_family = AF_INET;
    // servaddr_.sin_port = htons(7891);
    // servaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
    // memset(servaddr_.sin_zero, 0, sizeof(servaddr_.sin_zero));  

    for (int i = 0; i < socket_num; i++) {
        servaddr_[i].sin_family = AF_INET;
        servaddr_[i].sin_port = htons(8000+i);
        servaddr_[i].sin_addr.s_addr = inet_addr("168.6.245.88");
        memset(servaddr_[i].sin_zero, 0, sizeof(servaddr_[i].sin_zero)); 

        int rand_port = rand() % 65536;
        cliaddr_.sin_family = AF_INET;
        cliaddr_.sin_port = htons(0);  // out going port is random
        //cliaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
        cliaddr_.sin_addr.s_addr = htons(INADDR_ANY);
        memset(cliaddr_.sin_zero, 0, sizeof(cliaddr_.sin_zero));  

        if ((socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
            printf("cannot create socket\n");
            exit(0);
        }
        else{
            printf("Created socket: %d\n",i);
        }

        //int sock_buf_size = 1024*1024*64*8;
        //if (setsockopt(socket_[i], SOL_SOCKET, SO_SNDBUF, (void*)&sock_buf_size, sizeof(sock_buf_size))<0)
        //{
        //    printf("Error setting buffer size to %d\n", sock_buf_size);
       // }

        /*Bind socket with address struct*/
        if(bind(socket_[i], (struct sockaddr *) &cliaddr_, sizeof(cliaddr_)) != 0)
            perror("socket bind failed");
    }

    /* initialize random seed: */
    srand (time(NULL));


    memset(packet_count_per_frame, 0, sizeof(int) * BUFFER_FRAME_NUM);
    for (int i = 0; i < BUFFER_FRAME_NUM; i++) {
        memset(packet_count_per_subframe[i], 0, sizeof(int) * max_subframe_id);
    }

    IQ_data = new float*[subframe_num_perframe * BS_ANT_NUM];
    IQ_data_coded = new ushort*[subframe_num_perframe * BS_ANT_NUM];
    for(int i = 0; i < subframe_num_perframe * BS_ANT_NUM; i++)
    {
        IQ_data[i] = new float[OFDM_FRAME_LEN * 2];
        IQ_data_coded[i] = new ushort[OFDM_FRAME_LEN * 2];
    }
    
    // read from file
    FILE* fp = fopen("../rx_data_2048.bin","rb");
    if (fp==NULL) {
        printf("open file faild");
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for(int i = 0; i < subframe_num_perframe * BS_ANT_NUM; i++)
    {
        fread(IQ_data[i], sizeof(float), OFDM_FRAME_LEN * 2, fp);
        // range [-2,2]
        for(int j = 0; j < OFDM_FRAME_LEN * 2; j++)
        {
            IQ_data_coded[i][j] = (ushort)(IQ_data[i][j] * 32768);
            // printf("i:%d, j:%d, Coded: %d, orignal: %.4f\n",i,j/2,IQ_data_coded[i][j],IQ_data[i][j]);
        }

    }
    fclose(fp);

    trans_buffer_.resize(max_length_);
    for (int i = 0; i < max_length_; ++i)
    {
        trans_buffer_[i].resize(buffer_length);
    }
    printf("start sender\n");

    for (int i = 0; i < thread_num; i++) 
        task_ptok[i].reset(new moodycamel::ProducerToken(task_queue_));

    context = new PackageSenderContext[thread_num];
    std::vector<pthread_t> created_threads;
    for(int i = 0; i < thread_num; i++)
    {
        pthread_t send_thread_;
        
        context[i].ptr = this;
        context[i].tid = i;

        if(pthread_create( &send_thread_, NULL, PackageSender::loopSend, (void *)(&context[i])) != 0)
        {
            perror("socket send thread create failed");
            exit(0);
        }
        created_threads.push_back(send_thread_);
    }

#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(core_offset) != 0)
    {
        printf("stitch main thread to core %d failed\n", core_offset);
        exit(0);
    }
    else
    {
        printf("stitch main thread to core %d succeeded\n", core_offset);
    }
#endif

    // create send threads
    int cell_id = 0;
    // int max_subframe_id = ENABLE_DOWNLINK ? UE_NUM : subframe_num_perframe;
    int ret;
    // pthread_mutex_init(&lock_, NULL);
    // gen data

    // load data to buffer

    for (int i = 0; i < max_length_; i++) {
        cur_ptr_ = i;
        int data_index = subframe_id * BS_ANT_NUM + ant_id;
        int* ptr = (int *)trans_buffer_[cur_ptr_].data();
        (*ptr) = frame_id;
        (*(ptr+1)) = subframe_id;
        (*(ptr+2)) = cell_id;
        (*(ptr+3)) = ant_id;
        memcpy(trans_buffer_[cur_ptr_].data() + data_offset, (char *)IQ_data_coded[data_index], sizeof(ushort) * OFDM_FRAME_LEN * 2);   
        
        ant_id++;
        if(ant_id == BS_ANT_NUM)
        {
            ant_id = 0;
            subframe_id++;
            if(subframe_id == max_subframe_id)
            {
                subframe_id = 0;
                frame_id++;
                if(frame_id == MAX_FRAME_ID)
                    frame_id = 0;
            }
        }
    }

    

    double start_time = get_time();
    // push tasks of the first subframe into task queue
    for (int i = 0; i < BS_ANT_NUM; i++) {
        int ptok_id = i % thread_num;
        if ( !task_queue_.enqueue(*task_ptok[ptok_id], i) ) {
            printf("send task enqueue failed\n");
            exit(0);
        }
    }
    while(true)
    {
        // pthread_mutex_lock( &lock_ );
        // if(buffer_len_ == max_length_-10) // full
        // {
        //     pthread_mutex_unlock( &lock_ );
        //     // wait some time
        //     //for(int p = 0; p < 1e4; p++)
        //     //    rand();
        //     // printf("buffer full, buffer length: %d\n", buffer_len_);
        //     continue;
        // }
        // else // not full
        // {
        //     // printf("buffer_len_: %d\n", buffer_len_);
        //     buffer_len_ ++;  // take the place first
        // }
        // pthread_mutex_unlock( &lock_ );

        int data_ptr;
        ret = message_queue_.try_dequeue(data_ptr); 
        if(!ret)
            continue;
        int data_index = subframe_id * BS_ANT_NUM + ant_id;
        int* ptr = (int *)trans_buffer_[data_ptr].data();
        (*ptr) = frame_id;
        (*(ptr+1)) = subframe_id;
        (*(ptr+2)) = cell_id;
        (*(ptr+3)) = ant_id;
        memcpy(trans_buffer_[data_ptr].data() + data_offset, (char *)IQ_data_coded[data_index], sizeof(ushort) * OFDM_FRAME_LEN * 2);

        int tx_ant_id = data_ptr % BS_ANT_NUM;
        int tx_total_subframe_id = data_ptr / BS_ANT_NUM;
        int tx_current_subframe_id = tx_total_subframe_id % max_subframe_id;
        int tx_frame_id = tx_total_subframe_id / max_subframe_id;
        packet_count_per_subframe[tx_frame_id][tx_current_subframe_id]++;

        // printf("data_ptr: %d, tx_frame_id: %d, tx_total_subframe_id: %d, tx_current_subframe_id %d, tx_ant_id %d, max_subframe_id %d\n", data_ptr, 
        //     tx_frame_id, tx_total_subframe_id, tx_current_subframe_id, tx_ant_id, max_subframe_id);
        if (packet_count_per_subframe[tx_frame_id][tx_current_subframe_id] == BS_ANT_NUM) {
            packet_count_per_frame[tx_frame_id]++;
            
            printf("Finished transmit all antennas in frame: %d, subframe: %d, in %.5f us\n", tx_frame_id, tx_current_subframe_id, get_time()-start_time);
            // usleep(delay);
            // struct timespec tim, tim2;
            // tim.tv_sec = 0;
            // tim.tv_nsec = 0;
            // nanosleep(&tim , &tim2);
            // std::this_thread::sleep_for(std::chrono::nanoseconds(1000));

            

            if (packet_count_per_frame[tx_frame_id] == max_subframe_id) {
                packet_count_per_frame[tx_frame_id] = 0;
#if ENABLE_DOWNLINK
                if (frame_id < 500)
                    std::this_thread::sleep_for(std::chrono::microseconds(data_subframe_num_perframe*120));
                else
                    std::this_thread::sleep_for(std::chrono::microseconds(data_subframe_num_perframe*70));
#endif
                printf("Finished transmit all antennas in frame: %d in %.5f us\n", tx_frame_id,  get_time()-start_time);
                start_time = get_time();
            }

            packet_count_per_subframe[tx_frame_id][tx_current_subframe_id] = 0;

            int next_subframe_ptr = ((tx_total_subframe_id + 1) * BS_ANT_NUM) % max_length_;
            for (int i = 0; i < BS_ANT_NUM; i++) {
                int ptok_id = i % thread_num;
                if ( !task_queue_.enqueue(*task_ptok[ptok_id], i + next_subframe_ptr) ) {
                    printf("send task enqueue failed\n");
                    exit(0);
                }
            }
   
            // printf("buffer_len_: %d, cur_ptr_: %d\n",buffer_len_,cur_ptr_);
        }



        // int data_index = subframe_id * BS_ANT_NUM + ant_id;
        // int* ptr = (int *)trans_buffer_[data_ptr].data();
        // (*ptr) = frame_id;
        // (*(ptr+1)) = subframe_id;
        // (*(ptr+2)) = cell_id;
        // (*(ptr+3)) = ant_id;
        // memcpy(trans_buffer_[data_ptr].data() + data_offset, (char *)IQ_data_coded[data_index], sizeof(ushort) * OFDM_FRAME_LEN * 2);   
        // // printf("buffer_len_: %d, cur_ptr_: %d\n",buffer_len_,cur_ptr_);

        // if ( !task_queue_.enqueue(data_ptr) ) {
        //     printf("send task enqueue failed\n");
        //     exit(0);
        // }

        // if (DEBUG_SENDER) 
        // {
        //     printf("Enqueue for frame %d, subframe %d, ant %d, buffer %d\n", frame_id, subframe_id, ant_id,buffer_len_);
        // }

        // cur_ptr_ = (cur_ptr_ + 1) % max_length_;
        ant_id++;
        if(ant_id == BS_ANT_NUM)
        {
            ant_id = 0;
            subframe_id++;
            if(subframe_id == max_subframe_id)
            {
                subframe_id = 0;
                frame_id++;
                if(frame_id == MAX_FRAME_ID)
                    frame_id = 0;
            }
        }
    }
    
}

PackageSender::~PackageSender()
{
    for(int i = 0; i < subframe_num_perframe * BS_ANT_NUM; i++)
    {
        delete[] IQ_data_coded[i];
        delete[] IQ_data[i];
    }
    delete[] IQ_data;
    delete[] IQ_data_coded;

    delete[] socket_;
    delete[] context;
    pthread_mutex_destroy(&lock_);
}



void* PackageSender::loopSend(void *in_context)
{


    PackageSender* obj_ptr = ((PackageSenderContext *)in_context)->ptr;
    int tid = ((PackageSenderContext *)in_context)->tid;
    printf("package sender thread %d start\n", tid);

    moodycamel::ConcurrentQueue<int> *task_queue_ = &obj_ptr->task_queue_;
    moodycamel::ConcurrentQueue<int> *message_queue_ = &obj_ptr->message_queue_;


#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(obj_ptr->core_offset + 1 + tid) != 0)
    {
        printf("stitch thread %d to core %d failed\n", tid, obj_ptr->core_offset + 1 + tid);
        exit(0);
    }
    else
    {
        printf("stitch thread %d to core %d succeeded\n", tid, obj_ptr->core_offset + 1 + tid);
    }
#endif

    auto begin = std::chrono::system_clock::now();
    int package_count = 0;
    //std::iota(ant_seq.begin(), ant_seq.end(), 0);

    int used_socker_id = 0;
    int ret;
    int socket_per_thread = obj_ptr->socket_num / obj_ptr->thread_num;
    int total_tx_packets = 0;
    // int max_subframe_id = ENABLE_DOWNLINK ? UE_NUM : subframe_num_perframe;
    printf("max_subframe_id: %d\n", max_subframe_id);
    int ant_num_this_thread = BS_ANT_NUM / obj_ptr->thread_num + (tid < BS_ANT_NUM % obj_ptr->thread_num ? 1: 0);
    printf("In thread %d, %d antennas, BS_ANT_NUM: %d, thread number: %d\n", tid, ant_num_this_thread, BS_ANT_NUM, obj_ptr->thread_num);
    while(true)
    {
        int data_ptr;
        ret = task_queue_->try_dequeue_from_producer(*(obj_ptr->task_ptok[tid]),data_ptr); 
        if(!ret)
            continue;

        // get data
        // pthread_mutex_lock( &obj_ptr->lock_ );
        // obj_ptr->buffer_len_ --;
        // pthread_mutex_unlock( &obj_ptr->lock_ );

        used_socker_id = data_ptr % obj_ptr->socket_num;     
        int* ptr = (int *)obj_ptr->trans_buffer_[data_ptr].data();
        int subframe_id = (*(ptr+1));
        if (!ENABLE_DOWNLINK || subframe_id < UE_NUM) {
            /* send a message to the server */
            if (sendto(obj_ptr->socket_[used_socker_id], obj_ptr->trans_buffer_[data_ptr].data(), obj_ptr->buffer_length, 0, (struct sockaddr *)&obj_ptr->servaddr_[used_socker_id], sizeof(obj_ptr->servaddr_[used_socker_id])) < 0) {
                perror("socket sendto failed");
                exit(0);
            }
        }

        if (DEBUG_SENDER) 
        {
            int* ptr = (int *)obj_ptr->trans_buffer_[data_ptr].data();
            printf("Thread %d transmit frame %d, subframe %d, ant %d, total: %d, at %.5f\n", tid,  *ptr, *(ptr+1), *(ptr+3), total_tx_packets, get_time());
        }
        
        package_count++;
	total_tx_packets++;

        if ( !message_queue_->enqueue(data_ptr) ) {
            printf("send message enqueue failed\n");
            exit(0);
        }

        if (package_count % ant_num_this_thread == 0) {
    	    if (total_tx_packets < 500 * ant_num_this_thread * max_subframe_id) {
    		    // usleep(2000);
                std::this_thread::sleep_for(std::chrono::microseconds(100));
    		    // printf("In thread %d delayed\n", tid);
    	    }
	        else {
                std::this_thread::sleep_for(std::chrono::microseconds(obj_ptr->delay));
                // usleep(obj_ptr->delay);
            }
        }
	
    	if (total_tx_packets > 1e9)
    	    total_tx_packets = 0;
        if(package_count == ant_num_this_thread * max_subframe_id * 1000)
        {
            auto end = std::chrono::system_clock::now();
            double byte_len = sizeof(ushort) * OFDM_FRAME_LEN * 2 * ant_num_this_thread * max_subframe_id * 1000;
            std::chrono::duration<double> diff = end - begin;
            printf("thread %d send %d frames in %f secs, throughput %f MB/s\n", tid, total_tx_packets/(ant_num_this_thread* max_subframe_id), diff.count(), byte_len / diff.count() / 1024 / 1024);
            begin = std::chrono::system_clock::now();
            package_count = 0;
        }
    }
    
}
