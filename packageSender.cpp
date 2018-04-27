#include "packageSender.hpp"
#include "cpu_attach.hpp"

PackageSender::PackageSender(int in_socket_num, int in_thread_num, int in_core_offset):
ant_id(0), frame_id(0), subframe_id(0), thread_num(in_thread_num), 
socket_num(in_socket_num), cur_ptr_(0), core_offset(in_core_offset)
{
    socket_ = new int[socket_num];

    /*Configure settings in address struct*/
    servaddr_.sin_family = AF_INET;
    servaddr_.sin_port = htons(7891);
    servaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
    memset(servaddr_.sin_zero, 0, sizeof(servaddr_.sin_zero));  

    for(int i = 0; i < socket_num; i++)
    {
        int rand_port = rand() % 65536;
        cliaddr_.sin_family = AF_INET;
        cliaddr_.sin_port = htons(0);  // out going port is random
        //cliaddr_.sin_addr.s_addr = inet_addr("127.0.0.1");
        memset(cliaddr_.sin_zero, 0, sizeof(cliaddr_.sin_zero));  

        if ((socket_[i] = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { // UDP socket
            printf("cannot create socket\n");
            exit(0);
        }
        else{
            printf("Created socket: %d\n",socket_num);
        }

        /*Bind socket with address struct*/
        if(bind(socket_[i], (struct sockaddr *) &cliaddr_, sizeof(cliaddr_)) != 0)
            perror("socket bind failed");
    }

    /* initialize random seed: */
    srand (time(NULL));

    IQ_data = new float*[subframe_num_perframe * BS_ANT_NUM];
    IQ_data_coded = new ushort*[subframe_num_perframe * BS_ANT_NUM];
    for(int i = 0; i < subframe_num_perframe * BS_ANT_NUM; i++)
    {
        IQ_data[i] = new float[OFDM_FRAME_LEN * 2];
        IQ_data_coded[i] = new ushort[OFDM_FRAME_LEN * 2];
    }
    
    // read from file
    FILE* fp = fopen("../data.bin","rb");
    for(int i = 0; i < subframe_num_perframe * BS_ANT_NUM; i++)
    {
        fread(IQ_data[i], sizeof(float), OFDM_FRAME_LEN * 2, fp);
        // range [-2,2]
        for(int j = 0; j < OFDM_FRAME_LEN * 2; j++)
        {
            IQ_data_coded[i][j] = (ushort)(IQ_data[i][j] * 65536 / 4 + 65536 / 2);
            // printf("Coded: %d, orignal: %.4f\n", IQ_data_coded[i][j],IQ_data[i][j]);
        }

    }
    fclose(fp);

    trans_buffer_.resize(max_length_);
    for (int i = 0; i < max_length_; ++i)
    {
        trans_buffer_[i].resize(buffer_length);
    }
    printf("start sender\n");

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
#endif

    // create send threads
    int cell_id = 0;
    pthread_mutex_init(&lock_, NULL);
    // gen data
    while(true)
    {
        pthread_mutex_lock( &lock_ );
        if(buffer_len_ == max_length_) // full
        {
            pthread_mutex_unlock( &lock_ );
            // wait some time
            //for(int p = 0; p < 1e4; p++)
            //    rand();
            //printf("buffer full\n");
            continue;
        }
        else // not full
        {
            buffer_len_ ++;  // take the place first
        }
        pthread_mutex_unlock( &lock_ );

        int data_index = subframe_id * BS_ANT_NUM + ant_id;
        int* ptr = (int *)trans_buffer_[cur_ptr_].data();
        (*ptr) = frame_id;
        (*(ptr+1)) = subframe_id;
        (*(ptr+2)) = cell_id;
        (*(ptr+3)) = ant_id;
        memcpy(trans_buffer_[cur_ptr_].data() + data_offset, (char *)IQ_data_coded[data_index], sizeof(ushort) * OFDM_FRAME_LEN * 2);   
        printf("buffer_len_: %d, cur_ptr_: %d\n",buffer_len_,cur_ptr_);

        if ( !task_queue_.enqueue( cur_ptr_ ) ) {
            printf("send message enqueue failed\n");
            exit(0);
        }

        cur_ptr_ = (cur_ptr_ + 1) % max_length_;
        ant_id++;
        if(ant_id == BS_ANT_NUM)
        {
            ant_id = 0;
            subframe_id++;
            if(subframe_id == subframe_num_perframe)
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


#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(obj_ptr->core_offset + 1 + tid) != 0)
    {
        printf("stitch thread %d to core %d failed\n", tid, obj_ptr->core_offset + 1 + tid);
        exit(0);
    }
#endif

    auto begin = std::chrono::system_clock::now();
    int package_count = 0;
    //std::iota(ant_seq.begin(), ant_seq.end(), 0);

    int used_socker_id = 0;
    int ret;
    int socket_per_thread = obj_ptr->socket_num / obj_ptr->thread_num;
    while(true)
    {
        int data_ptr;
        ret = task_queue_->try_dequeue(data_ptr); 
        if(!ret)
            continue;

        // get data
        pthread_mutex_lock( &obj_ptr->lock_ );
        obj_ptr->buffer_len_ --;
        pthread_mutex_unlock( &obj_ptr->lock_ );

        used_socker_id = data_ptr % socket_per_thread + socket_per_thread * tid;
        /* send a message to the server */
        if (sendto(obj_ptr->socket_[used_socker_id], obj_ptr->trans_buffer_[data_ptr].data(), obj_ptr->buffer_length, 0, (struct sockaddr *)&obj_ptr->servaddr_, sizeof(obj_ptr->servaddr_)) < 0) {
            perror("socket sendto failed");
            exit(0);
        }
        
        package_count++;
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
