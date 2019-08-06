#include "CoMP.hpp"

// using namespace arma;
typedef cx_float COMPLEX;


bool keep_running = true;

void intHandler(int) {
    std::cout << "will exit..." << std::endl;
    keep_running = false;
}


CoMP::CoMP()
{
    printf("Main thread: on core %d\n", sched_getcpu());
    putenv( "MKL_THREADING_LAYER=sequential" );
    std::cout << "MKL_THREADING_LAYER =  " << getenv("MKL_THREADING_LAYER") << std::endl; 
    csi_format_offset = 1.0/32768;
    // openblas_set_num_threads(1);
    printf("enter constructor\n");
    // initialize socket buffer


    // read pilots from file
    pilots_ = (float *)aligned_alloc(64, OFDM_CA_NUM * sizeof(float));
    FILE* fp = fopen("../data/pilot_f_2048.bin","rb");
    fread(pilots_, sizeof(float), OFDM_CA_NUM, fp);
    fclose(fp);


#if DEBUG_PRINT_PILOT
    cout<<"Pilot data"<<endl;
    for (int i = 0; i<OFDM_CA_NUM;i++) 
        cout<<pilots_[i]<<",";
    cout<<endl;
#endif

    printf("initialize buffers\n");
    socket_buffer_size_ = (long long)PackageReceiver::package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM; 
    socket_buffer_status_size_ = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++) {
        socket_buffer_[i].buffer = (char *)aligned_alloc(64, socket_buffer_size_ * sizeof(char));
        socket_buffer_[i].buffer_status = (int *)aligned_alloc(64, socket_buffer_status_size_ * sizeof(int));
        // printf("socket buffer pointer: %llx, socket buffer size: %lld, %d, %d\n",socket_buffer_[i].buffer, socket_buffer_size_, PackageReceiver::package_length, SOCKET_BUFFER_FRAME_NUM);
    }

    // initialize CSI buffer
    int csi_buffer_size = UE_NUM * TASK_BUFFER_FRAME_NUM;
    csi_buffer_.CSI = (complex_float **)malloc(csi_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < csi_buffer_size; i++)
        csi_buffer_.CSI[i] = (complex_float *)aligned_alloc(64, BS_ANT_NUM * OFDM_DATA_NUM * sizeof(complex_float));


    int data_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    data_buffer_.data = (complex_float **)malloc(data_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < data_buffer_size; i++)
        data_buffer_.data[i] = (complex_float *)aligned_alloc(64, BS_ANT_NUM * OFDM_DATA_NUM * sizeof(complex_float));


    if (DO_PREDICTION) {
        int pred_csi_buffer_size = OFDM_DATA_NUM;
        pred_csi_buffer_.CSI = (complex_float **)malloc(pred_csi_buffer_size * sizeof(complex_float *));
        
        for(int i = 0; i < pred_csi_buffer_size; i++)
            pred_csi_buffer_.CSI[i] = (complex_float *)aligned_alloc(BS_ANT_NUM * UE_NUM * sizeof(complex_float), BS_ANT_NUM * UE_NUM * sizeof(complex_float));
    }


    // initialize precoder buffer
    int precoder_buffer_size = OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM;
    precoder_buffer_.precoder = (complex_float **)malloc(precoder_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < precoder_buffer_size; i++)
        precoder_buffer_.precoder[i] = (complex_float *)aligned_alloc(BS_ANT_NUM * UE_NUM * sizeof(complex_float), UE_NUM * BS_ANT_NUM * sizeof(complex_float));

    // initialize equalized data buffer
    int equal_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    equal_buffer_.data = (complex_float **)malloc(equal_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < equal_buffer_size; i++)
        equal_buffer_.data[i] = (complex_float *)aligned_alloc(64, OFDM_DATA_NUM * UE_NUM * sizeof(complex_float));



    // initialize demultiplexed data buffer
    int demul_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    demul_hard_buffer_ = (uint8_t **)malloc(demul_buffer_size * sizeof(uint8_t *));
    for (int i = 0; i < demul_buffer_size; i++)
        demul_hard_buffer_[i] = (uint8_t *)aligned_alloc(64, OFDM_CA_NUM * UE_NUM * sizeof(uint8_t));

    demul_soft_buffer_ = (float **)malloc(demul_buffer_size * sizeof(float *));
    for (int i = 0; i < demul_buffer_size; i++)
        demul_soft_buffer_[i] = (float *)aligned_alloc(64, NUM_BITS * OFDM_CA_NUM * UE_NUM * sizeof(float));


    int decoded_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    decoded_buffer_ = (int **)malloc(decoded_buffer_size * sizeof(int *));
    for (int i = 0; i < decoded_buffer_size; i++)
        decoded_buffer_[i] = (int *)aligned_alloc(64, NUM_BITS * ORIG_CODE_LEN * NUM_CODE_BLOCK * UE_NUM * sizeof(int));



    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        precoder_buffer_temp[i] = (complex_float *)aligned_alloc(BS_ANT_NUM * UE_NUM * sizeof(complex_float), BS_ANT_NUM * UE_NUM * sizeof(complex_float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        equaled_buffer_T_temp[i] = (complex_float *)aligned_alloc(64, demul_block_size * UE_NUM * sizeof(complex_float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        demul_hard_buffer_temp[i] = (uint8_t *)aligned_alloc(64, demul_block_size * UE_NUM * sizeof(uint8_t));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        demul_soft_buffer_temp[i] = (float *)aligned_alloc(64, NUM_BITS * CODED_LEN * UE_NUM * sizeof(float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        coded_buffer_temp[i] = (int*)aligned_alloc(64, NUM_BITS * CODED_LEN * sizeof(int));
    // printf("Demultiplexed data buffer initialized\n");

    moodycamel::ProducerToken *rx_ptoks_ptr[SOCKET_RX_THREAD_NUM];
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++) { 
        rx_ptok[i].reset(new moodycamel::ProducerToken(message_queue_));
        rx_ptoks_ptr[i] = rx_ptok[i].get();
    }


    moodycamel::ProducerToken *tx_ptoks_ptr[SOCKET_RX_THREAD_NUM];
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++) { 
        tx_ptok[i].reset(new moodycamel::ProducerToken(tx_queue_));
        tx_ptoks_ptr[i] = tx_ptok[i].get();
    }

    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        memset(CSI_task_duration[i], 0, sizeof(double) * 4 * 8);
        memset(FFT_task_duration[i], 0, sizeof(double) * 4 * 8);
        memset(ZF_task_duration[i], 0, sizeof(double) * 4 * 8);
        memset(Demul_task_duration[i], 0, sizeof(double) * 4 * 8);
    }

    memset(CSI_task_count, 0, sizeof(int) * TASK_THREAD_NUM * 16);
    memset(FFT_task_count, 0, sizeof(int) * TASK_THREAD_NUM * 16);
    memset(ZF_task_count, 0, sizeof(int) * TASK_THREAD_NUM * 16);
    memset(Demul_task_count, 0, sizeof(int) * TASK_THREAD_NUM * 16);



#if ENABLE_DOWNLINK
    printf("Downlink enabled.\n");
    int dl_IQ_data_size = data_subframe_num_perframe * UE_NUM;
    dl_IQ_data = (int **)malloc(dl_IQ_data_size * sizeof(int *));
    // dl_IQ_data = new int * [data_subframe_num_perframe * UE_NUM];
    dl_IQ_data_long = new long long * [data_subframe_num_perframe * UE_NUM];
    for (int i = 0; i < dl_IQ_data_size; i++) {
        dl_IQ_data[i] = (int *)aligned_alloc(64, OFDM_CA_NUM * sizeof(int));
        // dl_IQ_data[i] = new int[OFDM_CA_NUM];
        dl_IQ_data_long[i] = new long long[PackageReceiver::OFDM_FRAME_LEN];
    }
    // read data from file
    std::string filename1 = "../data/orig_data_2048_ant" + std::to_string(BS_ANT_NUM) + ".bin";
    fp = fopen(filename1.c_str(),"rb");
    if (fp==NULL) {
        printf("open file faild");
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for (int i = 0; i < data_subframe_num_perframe * UE_NUM; i++) {
        fread(dl_IQ_data[i], sizeof(int), OFDM_CA_NUM, fp);
        // range [-2,2]
        for(int j = 0; j < OFDM_CA_NUM; j++) {
            dl_IQ_data_long[i][j] = (long long)dl_IQ_data[i][j];
            // printf("i:%d, j:%d, Coded: %d, orignal: %.4f\n",i,j/2,IQ_data_coded[i][j],IQ_data[i][j]);
        }
    }
    fclose(fp);

    int IFFT_buffer_block_num = BS_ANT_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    dl_ifft_buffer_.IFFT_inputs = (complex_float **)malloc(IFFT_buffer_block_num * sizeof(complex_float *));
    dl_ifft_buffer_.IFFT_outputs = (complex_float **)malloc(IFFT_buffer_block_num * sizeof(complex_float *));
    for (int i = 0; i < IFFT_buffer_block_num; i++) {
        dl_ifft_buffer_.IFFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
        memset(dl_ifft_buffer_.IFFT_inputs[i], 0, sizeof(complex_float) * OFDM_CA_NUM);
    }
    for (int i = 0; i < IFFT_buffer_block_num; i++) {
        dl_ifft_buffer_.IFFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
        memset(dl_ifft_buffer_.IFFT_outputs[i], 0, sizeof(complex_float) * OFDM_CA_NUM);
    }


    // initialize downlink precoded data buffer
    int dl_precoded_data_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    dl_precoded_data_buffer_.data = (complex_float **)malloc(dl_precoded_data_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < dl_precoded_data_buffer_size; i++)
        dl_precoded_data_buffer_.data[i] = (complex_float *)aligned_alloc(64, BS_ANT_NUM * OFDM_DATA_NUM * sizeof(complex_float));
    
    // initialize downlink modulated data buffer
    int dl_modulated_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    dl_modulated_buffer_.data = (complex_float **)malloc(dl_modulated_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < dl_modulated_buffer_size; i++)
        dl_modulated_buffer_.data[i] = (complex_float *)aligned_alloc(64, UE_NUM * OFDM_DATA_NUM * sizeof(complex_float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        dl_spm_buffer[i] = (complex_float *)aligned_alloc(64, 8 * UE_NUM * sizeof(complex_float));


    // initialize downlink socket buffer
    dl_socket_buffer_size_ = (long long) data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM * PackageReceiver::package_length * BS_ANT_NUM;
    dl_socket_buffer_status_size_ = data_subframe_num_perframe * BS_ANT_NUM * TASK_BUFFER_FRAME_NUM;
    dl_socket_buffer_.buffer = (char *)aligned_alloc(64, dl_socket_buffer_size_ * sizeof(char));
    dl_socket_buffer_.buffer_status = (int *)aligned_alloc(64, dl_socket_buffer_size_ * sizeof(int));

    for (int i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
        memset(dl_data_counter_scs_[i], 0, sizeof(int) * data_subframe_num_perframe);
        memset(modulate_checker_[i], 0, sizeof(int) * data_subframe_num_perframe);
        memset(tx_counter_ants_[i], 0, sizeof(int) * data_subframe_num_perframe);
    }

    memset(dl_data_counter_subframes_, 0, sizeof(int) * TASK_THREAD_NUM);
    memset(ifft_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(tx_counter_subframes_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 

    printf("initialize QAM16 table\n");
    init_qam16_table(qam16_table);
    // float scale = 1/sqrt(10);
    // float modvec_16qam[4] = {-3*scale, -1*scale, 3*scale, scale};
    // for (int i = 0; i < 16; i++) {
    //     qam16_table[0][i] = modvec_16qam[i / 4];
    //     qam16_table[1][i] = modvec_16qam[i % 4];
    // }

    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        memset(IFFT_task_duration[i], 0, sizeof(double) * 4 * 8);
        memset(Precode_task_duration[i], 0, sizeof(double) * 4 * 8);
    }

    memset(IFFT_task_count, 0, sizeof(int) * TASK_THREAD_NUM * 16);
    memset(Precode_task_count, 0, sizeof(int) * TASK_THREAD_NUM * 16);

#endif

    printf("new PackageReceiver\n");
    receiver_.reset(new PackageReceiver(SOCKET_RX_THREAD_NUM, SOCKET_TX_THREAD_NUM, CORE_OFFSET+1, &message_queue_, &tx_queue_, rx_ptoks_ptr, tx_ptoks_ptr));

    // initilize all kinds of checkers
    memset(fft_counter_ants_, 0, sizeof(int) * subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    memset(csi_counter_users_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(data_counter_subframes_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_counter_scs_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_exist_in_frame_, 0, sizeof(bool) * TASK_BUFFER_FRAME_NUM); 


    memset(demul_counter_subframes_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(fft_created_counter_packets_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(rx_counter_packets_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(rx_counter_packets_pilots_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);

    memset(decode_counter_subframes_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);

    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
        memset(demul_counter_scs_[i], 0, sizeof(int) * (subframe_num_perframe - UE_NUM));

    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
        memset(data_exist_in_subframe_[i], 0, sizeof(bool) * (subframe_num_perframe - UE_NUM));

    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
        memset(precoder_exist_in_sc_[i], 0, sizeof(bool) * (OFDM_DATA_NUM));

    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
        memset(decode_counter_blocks_[i], 0, sizeof(int) * (subframe_num_perframe - UE_NUM));



#if BIGSTATION
    for(int i = 0; i < FFT_THREAD_NUM; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        //printf("create thread %d\n", i);
        if(pthread_create( &task_threads[i], NULL, CoMP::fftThread, &context[i]) != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
    for(int i = FFT_THREAD_NUM; i < FFT_THREAD_NUM + ZF_THREAD_NUM; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        //printf("create thread %d\n", i);
        if(pthread_create( &task_threads[i], NULL, CoMP::zfThread, &context[i]) != 0) {
            perror("ZF thread create failed");
            exit(0);
        }
    }
    for(int i = FFT_THREAD_NUM + ZF_THREAD_NUM; i < TASK_THREAD_NUM; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        //printf("create thread %d\n", i);
        if(pthread_create( &task_threads[i], NULL, CoMP::demulThread, &context[i]) != 0) {
            perror("Demul thread create failed");
            exit(0);
        }
    }
#else
    // create task thread 
    for(int i = 0; i < TASK_THREAD_NUM; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        if(pthread_create( &task_threads[i], NULL, CoMP::taskThread, &context[i]) != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
#endif

}

CoMP::~CoMP()
{
    // for(int i = 0; i < TASK_THREAD_NUM; i++) {
    //     mufft_free_plan_1d(muplans_[i]);
    //     mufft_free_plan_1d(muplans_ifft_[i]);
    // }
    // release FFT_buffer
    // int FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    // for(int i = 0; i < FFT_buffer_block_num; i++) {
    //     mufft_free(fft_buffer_.FFT_inputs[i]);
    //     mufft_free(fft_buffer_.FFT_outputs[i]);
    //     mufft_free(dl_ifft_buffer_.IFFT_inputs[i]);
    //     mufft_free(dl_ifft_buffer_.IFFT_outputs[i]);
    // }

    for(int i = 0; i < data_subframe_num_perframe * UE_NUM; i++) {
        delete[] dl_IQ_data_long[i];
        delete[] dl_IQ_data[i];
    }
    delete[] dl_IQ_data;
    delete[] dl_IQ_data_long;

}


void CoMP::schedule_task(Event_data do_task, moodycamel::ConcurrentQueue<Event_data> * in_queue, moodycamel::ProducerToken const& ptok) 
{
    if ( !in_queue->try_enqueue(ptok, do_task ) ) {
        printf("need more memory\n");
        if ( !in_queue->enqueue(ptok, do_task ) ) {
            printf("task enqueue failed\n");
            exit(0);
        }
    }
}




void CoMP::start()
{
    // if ENABLE_CPU_ATTACH, attach main thread to core 0
#ifdef ENABLE_CPU_ATTACH
    int main_core_id = CORE_OFFSET + 1;
    if(stick_this_thread_to_core(main_core_id) != 0) {
        printf("Main thread: stitch main thread to core %d failed\n", main_core_id);
        exit(0);
    }
    else {
        printf("Main thread: stitch main thread to core %d succeeded\n", main_core_id);
    }
#endif
    // start uplink receiver
    // creare socket buffer and socket threads

    int buffer_frame_num = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    char *socket_buffer_ptrs[SOCKET_RX_THREAD_NUM];
    int *socket_buffer_status_ptrs[SOCKET_RX_THREAD_NUM];
    double *frame_start_ptrs[SOCKET_RX_THREAD_NUM];
    for(int i = 0; i < SOCKET_RX_THREAD_NUM; i++) {
        socket_buffer_ptrs[i] = socket_buffer_[i].buffer;
        socket_buffer_status_ptrs[i] = socket_buffer_[i].buffer_status;
        frame_start_ptrs[i] = frame_start[i];
        // printf("Socket buffer ptr: %llx, socket_buffer: %llx\n", socket_buffer_ptrs[i],socket_buffer_[i].buffer);
    }
    std::vector<pthread_t> rx_threads = receiver_->startRecv(socket_buffer_ptrs, 
        socket_buffer_status_ptrs, socket_buffer_status_size_, socket_buffer_size_, frame_start_ptrs);

    // start downlink transmitter
#if ENABLE_DOWNLINK
    char *dl_socket_buffer_ptr = dl_socket_buffer_.buffer;
    int *dl_socket_buffer_status_ptr = dl_socket_buffer_.buffer_status;
    float *dl_data_ptr = (float *)(&dl_precoded_data_buffer_.data[0][0]);
    std::vector<pthread_t> tx_threads = receiver_->startTX(dl_socket_buffer_ptr, 
        dl_socket_buffer_status_ptr, dl_data_ptr, dl_socket_buffer_status_size_, dl_socket_buffer_size_);
    // std::vector<pthread_t> tx_threads = transmitter_->startTX(dl_socket_buffer_ptr, 
    //     dl_socket_buffer_status_ptr, dl_data_ptr, dl_socket_buffer_status_size_, dl_socket_buffer_size_, main_core_id + 1 +SOCKET_RX_THREAD_NUM);
#endif

    // for fft_queue, main thread is producer, it is single-procuder & multiple consumer
    // for task queue
    // uplink

    // TODO: make the producertokens global and try "try_dequeue_from_producer(token,item)"
    //       combine the task queues into one queue
    moodycamel::ProducerToken ptok(fft_queue_);
    moodycamel::ProducerToken ptok_zf(zf_queue_);
    moodycamel::ProducerToken ptok_demul(demul_queue_);
    moodycamel::ProducerToken ptok_decode(decode_queue_);
    // downlink
    moodycamel::ProducerToken ptok_ifft(ifft_queue_);
    moodycamel::ProducerToken ptok_modul(modulate_queue_);
    moodycamel::ProducerToken ptok_precode(precode_queue_);
    moodycamel::ProducerToken ptok_tx(tx_queue_);
    // for message_queue, main thread is a comsumer, it is multiple producers
    // & single consumer for message_queue
    moodycamel::ConsumerToken ctok(message_queue_);
    moodycamel::ConsumerToken ctok_complete(complete_task_queue_);

    int delay_fft_queue[TASK_BUFFER_FRAME_NUM][subframe_num_perframe * BS_ANT_NUM] = {0};
    int delay_fft_queue_cnt[TASK_BUFFER_FRAME_NUM] = {0};

    // for (int i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
    //     memset(delay_fft_queue[i], 0, sizeof(int) * subframe_num_perframe * BS_ANT_NUM);
    // }

    // memset(delay_fft_queue_cnt, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);


    // counter for print log
    int demul_count = 0;
    auto demul_begin = std::chrono::system_clock::now();
    auto tx_begin = std::chrono::high_resolution_clock::now();
    auto ifft_begin = std::chrono::high_resolution_clock::now();
    auto zf_begin = std::chrono::high_resolution_clock::now();
    int miss_count = 0;
    int total_count = 0;
    int frame_count_pilot_fft = 0;
    int frame_count_zf = 0;
    int frame_count_demul = 0;
    int frame_count_decode = 0;

    int frame_count_precode = 0;
    int frame_count_ifft = 0;
    int frame_count_tx = 0;

    int tx_count = 0;
    int zf_count = 0;
    // auto pilot_received = std::chrono::system_clock::now();
    // std::chrono::time_point<std::chrono::high_resolution_clock> pilot_received[TASK_BUFFER_FRAME_NUM];
    double pilot_received[10000] __attribute__( ( aligned (4096) ) ) ;
    double pilot_all_received[10000] __attribute__( ( aligned (4096) ) ) ;
    double processing_started[10000] __attribute__( ( aligned (4096) ) ) ;
    double rx_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double fft_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double demul_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double zf_processed[10000] __attribute__( ( aligned (4096) ) ) ;

    double csi_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double fft_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double zf_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double demul_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double ifft_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;
    double precode_time_in_function[10000] __attribute__( ( aligned (4096) ) ) ;

    double precode_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double ifft_processed[10000] __attribute__( ( aligned (4096) ) ) ;
    double tx_processed_first[10000] __attribute__( ( aligned (4096) ) ) ;
    double tx_processed[10000] __attribute__( ( aligned (4096) ) ) ;


#if DEBUG_UPDATE_STATS_DETAILED
    double csi_time_in_function_details[3][10000] __attribute__( ( aligned (4096) ) ) ;
    double fft_time_in_function_details[3][10000] __attribute__( ( aligned (4096) ) ) ;
    double zf_time_in_function_details[3][10000] __attribute__( ( aligned (4096) ) ) ;
    double demul_time_in_function_details[3][10000] __attribute__( ( aligned (4096) ) ) ;
#endif


    double total_time = 0;
    int ifft_frame_count = 0;


    Event_data events_list[dequeue_bulk_size];
    int ret = 0;
    bool rx_start = false;
    bool prev_demul_scheduled = false;
    double fft_previous_time = get_time();

    double csi_time_sum[4] = {0};
    double fft_time_sum[4] = {0};
    double zf_time_sum[4] = {0};
    double demul_time_sum[4] = {0};
    double ifft_time_sum[4] = {0};
    double precode_time_sum[4] = {0};

    double csi_time_this_frame[4];
    double fft_time_this_frame[4];
    double zf_time_this_frame[4];
    double demul_time_this_frame[4];
    double ifft_time_this_frame[4];
    double precode_time_this_frame[4];

    double csi_time_per_thread[4][TASK_THREAD_NUM] = {0};
    double fft_time_per_thread[4][TASK_THREAD_NUM] = {0};
    double zf_time_per_thread[4][TASK_THREAD_NUM] = {0};
    double demul_time_per_thread[4][TASK_THREAD_NUM] = {0};
    double ifft_time_per_thread[4][TASK_THREAD_NUM] = {0};
    double precode_time_per_thread[4][TASK_THREAD_NUM] = {0};

    int csi_count_per_thread[TASK_THREAD_NUM];
    int fft_count_per_thread[TASK_THREAD_NUM];
    int zf_count_per_thread[TASK_THREAD_NUM];
    int demul_count_per_thread[TASK_THREAD_NUM];
    int ifft_count_per_thread[TASK_THREAD_NUM];
    int precode_count_per_thread[TASK_THREAD_NUM];


    int last_dequeue = 0;
    int cur_queue_itr = 0;
    double last_dequeue_time = get_time();
    double cur_dequeue_time = get_time();
    double diff_dequeue_time;
    double queue_process_time;
    double queue_process_start;

    signal(SIGINT, intHandler);
    while(keep_running) {
        // get a bulk of events
        // ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size);
        // if (ret == 0)
        //     ret = complete_task_queue_.try_dequeue_bulk(ctok_complete, events_list, dequeue_bulk_size);
        if (last_dequeue == 0) {
            // ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size_single);
            ret = 0;
            for (int rx_itr = 0; rx_itr < SOCKET_RX_THREAD_NUM; rx_itr ++) {
                
                ret += message_queue_.try_dequeue_bulk_from_producer(*rx_ptok[rx_itr], events_list + ret, dequeue_bulk_size_single);
            }
            // if (ret == 0);
            //     ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size_single);
            last_dequeue = 1;
            // ret = message_queue_.try_dequeue_bulk_from_producer(*rx_ptok[cur_queue_itr], events_list, dequeue_bulk_size_single);
            // cur_queue_itr++;
            // if (cur_queue_itr == SOCKET_RX_THREAD_NUM) {
            //     cur_queue_itr = 0;
            //     last_dequeue = 1;
            // }
        }
        else {   
            ret = complete_task_queue_.try_dequeue_bulk(ctok_complete, events_list, dequeue_bulk_size_single);
            last_dequeue = 0;
        }
        // if (ret == 0)
        //     ret = complete_task_queue_.try_dequeue_bulk(ctok_complete, events_list, dequeue_bulk_size);
            // ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size);
        total_count++;
        if(total_count == 1e9) {
            //printf("message dequeue miss rate %f\n", (float)miss_count / total_count);
            total_count = 0;
            miss_count = 0;
        }
        if(ret == 0) {
            miss_count++;
            continue;
        }

        cur_dequeue_time = get_time();
        diff_dequeue_time = cur_dequeue_time - last_dequeue_time;
        // if (diff_dequeue_time > 3000 && frame_count_zf > 1000)
        //     printf("Main thread: frame %d takes %.2f us since last dequeue\n", frame_count_zf, diff_dequeue_time);
        last_dequeue_time = get_time();
        // handle each event
        for(int bulk_count = 0; bulk_count < ret; bulk_count ++) {
            queue_process_start = get_time();
            Event_data& event = events_list[bulk_count];

            switch(event.event_type) {
            case EVENT_PACKAGE_RECEIVED: {         
                    int offset = event.data;                    
                    int socket_thread_id = offset / buffer_frame_num;
                    int offset_in_current_buffer = offset % buffer_frame_num;

                    if (!rx_start) {
                        rx_start = true;
                        tx_begin = std::chrono::high_resolution_clock::now();
                        demul_begin = std::chrono::system_clock::now();
                        // hpctoolkit_sampling_start();
                    }

                    // char *socket_buffer_ptr = socket_buffer_ptrs[socket_thread_id] + offset_in_current_buffer * PackageReceiver::package_length;
                    char *socket_buffer_ptr = socket_buffer_[socket_thread_id].buffer + (long long) offset_in_current_buffer * PackageReceiver::package_length;
                    int subframe_id = *((int *)socket_buffer_ptr + 1);                                    
                    int frame_id = *((int *)socket_buffer_ptr);
                    int ant_id = *((int *)socket_buffer_ptr + 3);
                    int rx_frame_id = (frame_id % TASK_BUFFER_FRAME_NUM);

                    if (isPilot(subframe_id)) { 
                        rx_counter_packets_pilots_[rx_frame_id]++;

                        if(rx_counter_packets_pilots_[rx_frame_id] == BS_ANT_NUM * UE_NUM) {
                            rx_counter_packets_pilots_[rx_frame_id] = 0;
                            pilot_all_received[frame_id] = get_time();
#if DEBUG_PRINT_PER_FRAME_DONE 
                        printf("Main thread: received all pilots in frame: %d, frame buffer: %d in %.5f us\n", frame_id, rx_frame_id, 
                                pilot_all_received[frame_id]-pilot_received[frame_id]);
#endif  
                        }
                    }

                    // printf("Main thread: data received from frame %d, subframe %d, ant %d, buffer ptr: %llx, offset %lld\n", frame_id, subframe_id, ant_id, 
                    //      socket_buffer_ptr, offset_in_current_buffer * PackageReceiver::package_length);
                    rx_counter_packets_[rx_frame_id]++;
                    // printf("Main thread: data received from frame %d, subframe %d, ant %d\n", frame_id, subframe_id, ant_id);
#if ENABLE_DOWNLINK
                    int rx_counter_packets_max = BS_ANT_NUM * UE_NUM;
#else
                    int rx_counter_packets_max = BS_ANT_NUM * subframe_num_perframe;
#endif
                    if (rx_counter_packets_[rx_frame_id] == 1) {   

                        // pilot_received[rx_frame_id] = get_time();      
                        pilot_received[frame_id] = get_time();  
#if DEBUG_PRINT_PER_FRAME_START 
                        if (frame_id > 0) {
                            int prev_frame_id = (frame_id > 1)? (frame_id - 1) %TASK_BUFFER_FRAME_NUM : 0;
                            printf("Main thread: data received from frame %d, subframe %d, ant %d, in %.5f us, previous frame: %d\n", frame_id, subframe_id, ant_id, 
                                pilot_received[frame_id]-pilot_received[frame_id-1], rx_counter_packets_[prev_frame_id]);
                        }
                        else {
                            printf("Main thread: data received from frame %d, subframe %d, ant %d, in %.5f us\n", frame_id, subframe_id, ant_id, 
                                pilot_received[frame_id]);
                        }
#endif                       
                    }
                    // else if (rx_counter_packets_[rx_frame_id] % BS_ANT_NUM == 0) {
                    //     printf("Main thread: received in frame: %d, subframe: %d ant %d, in %.5f us\n", frame_id, subframe_id, ant_id,
                    //             get_time() - pilot_received[frame_id]);
                    // }
                    else if (rx_counter_packets_[rx_frame_id] == rx_counter_packets_max) {  
                        rx_processed[frame_id] = get_time();
#if DEBUG_PRINT_PER_FRAME_DONE 
                        int prev_frame_id = (frame_id > 1)? (frame_id - 1) %TASK_BUFFER_FRAME_NUM : 0;
                        printf("Main thread: received data for all packets in frame: %d, frame buffer: %d in %.5f us, demul: %d done, FFT: %d,%d,  received %d\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, 
                                rx_processed[frame_id]-pilot_received[frame_id], demul_counter_subframes_[rx_frame_id],data_counter_subframes_[rx_frame_id], 
                                fft_counter_ants_[data_counter_subframes_[rx_frame_id]+subframe_num_perframe*(rx_frame_id)+UE_NUM], rx_counter_packets_[prev_frame_id] );
#endif                         
                        rx_counter_packets_[rx_frame_id] = 0;                                  
                    }  
                    // if (frame_id > 0)
                    //     printf("Main thread: data received from frame %d, subframe %d, ant %d, total: %d, in %.5f us, buffer ptr: %llx, offset %lld\n", frame_id, subframe_id, ant_id, 
                    //         rx_counter_packets_[rx_frame_id], get_time()-pilot_received[frame_id], socket_buffer_ptr, offset_in_current_buffer * PackageReceiver::package_length);

#if BIGSTATION 
                    Event_data do_fft_task;
                    do_fft_task.event_type = TASK_FFT;
                    do_fft_task.data = offset;
                    schedule_task(do_fft_task, &fft_queue_, ptok);
                    fft_created_counter_packets_[rx_frame_id]++;
    #if DEBUG_PRINT_PER_TASK_ENTER_QUEUE
                    printf("Main thread: created FFT tasks for frame: %d, frame buffer: %d, subframe: %d, ant: %d\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
    #endif   
    #if ENABLE_DOWNLINK
                    if (fft_created_counter_packets_[rx_frame_id] == BS_ANT_NUM * UE_NUM) {
        #if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE                            
                        printf("Main thread: created FFT tasks for all packets in frame: %d, frame buffer: %d in %.5f us\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, get_time()-pilot_received[frame_id]);
        #endif                            
                        fft_created_counter_packets_[rx_frame_id] = 0;
                        if(frame_id > 0) 
                            ifft_checker_[(frame_id-1)%TASK_BUFFER_FRAME_NUM] = 0;
                        else
                            ifft_checker_[TASK_BUFFER_FRAME_NUM-1] = 0;
                    }
    #else                          
                    if (fft_created_counter_packets_[rx_frame_id] == BS_ANT_NUM * subframe_num_perframe) {
        #if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE                            
                        printf("Main thread: created FFT tasks for all packets in frame: %d, frame buffer: %d in %.5f us\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, get_time()-pilot_received[frame_id]);
        #endif                       
                        fft_created_counter_packets_[rx_frame_id] = 0;
                        // if(frame_id > 0) 
                        //     demul_counter_subframes_[(frame_id-1)%TASK_BUFFER_FRAME_NUM] = 0;
                        // else
                        //     demul_counter_subframes_[TASK_BUFFER_FRAME_NUM-1] = 0;
                    }
    #endif
#else
                    // if this is the first frame or the previous frame is all processed, schedule FFT for this packet
                    int frame_id_prev = frame_id == 0 ? (TASK_BUFFER_FRAME_NUM-1) : (frame_id - 1) % TASK_BUFFER_FRAME_NUM;
    #if ENABLE_DOWNLINK
                    bool previous_frame_done = ifft_checker_[frame_id_prev] == BS_ANT_NUM * dl_data_subframe_num_perframe;
    #else
                    bool previous_frame_done = demul_counter_subframes_[frame_id_prev] == data_subframe_num_perframe;
    #endif
                    if ((frame_id == 0 && frame_count_pilot_fft < 100) || (frame_count_pilot_fft > 0 && previous_frame_done)) {
                        Event_data do_fft_task;
                        do_fft_task.event_type = TASK_FFT;
                        do_fft_task.data = offset;
                        schedule_task(do_fft_task, &fft_queue_, ptok);
                        fft_created_counter_packets_[rx_frame_id]++;
                        if (fft_created_counter_packets_[rx_frame_id] == 1) {
                            processing_started[frame_id] = get_time();
                        }
    #if DEBUG_PRINT_PER_TASK_ENTER_QUEUE
                        printf("Main thread: created FFT tasks for frame: %d, frame buffer: %d, subframe: %d, ant: %d\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
    #endif                    
    #if ENABLE_DOWNLINK
                        if (fft_created_counter_packets_[rx_frame_id] == BS_ANT_NUM * UE_NUM) {
        #if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE                            
                            printf("Main thread: created FFT tasks for all packets in frame: %d, frame buffer: %d in %.5f us\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, get_time()-pilot_received[frame_id]);
        #endif                            
                            fft_created_counter_packets_[rx_frame_id] = 0;
                            if(frame_id > 0) 
                                ifft_checker_[(frame_id-1)%TASK_BUFFER_FRAME_NUM] = 0;
                            else
                                ifft_checker_[TASK_BUFFER_FRAME_NUM-1] = 0;
                        }
    #else
                        if (fft_created_counter_packets_[rx_frame_id] == BS_ANT_NUM * subframe_num_perframe) {
        #if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE                            
                            printf("Main thread: created FFT tasks for all packets in frame: %d, frame buffer: %d in %.5f us\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, get_time()-pilot_received[frame_id]);
        #endif                            
                            fft_created_counter_packets_[rx_frame_id] = 0;
                            if(frame_id > 0) 
                                demul_counter_subframes_[(frame_id-1)%TASK_BUFFER_FRAME_NUM] = 0;
                            else
                                demul_counter_subframes_[TASK_BUFFER_FRAME_NUM-1] = 0;
                        }
    #endif
                        
                    }
                    else {
                        // if the previous frame is not finished, store offset in queue
                        // printf("previous frame not finished, frame_id %d, previous frame finished: %d, subframe_id %d, ant_id %d, delay_fft_queue_cnt %d\n", frame_id, demul_counter_subframes_[(frame_id-1)%TASK_BUFFER_FRAME_NUM], 
                        //      subframe_id, ant_id, delay_fft_queue_cnt[rx_frame_id]);
                        delay_fft_queue[rx_frame_id][delay_fft_queue_cnt[rx_frame_id]] = offset;
                        delay_fft_queue_cnt[rx_frame_id]++;
                    }

#endif
                                            
                }                
                break;
                
            
            case EVENT_FFT: {
                    int offset_fft = event.data;
                    int frame_id, subframe_id;
                    interpreteOffset2d(subframe_num_perframe, offset_fft, &frame_id, &subframe_id);
                    fft_counter_ants_[offset_fft] ++;
                    // printf("FFT done: frame %d, subframe %d, total subframes %d, ant counter %d \n", frame_id, subframe_id, offset_fft, fft_counter_ants_[offset_fft]);

                    // if FFT for all anetnnas in a subframe is done, schedule ZF or equalization+demodulation
                    if (fft_counter_ants_[offset_fft] == BS_ANT_NUM) {
                        fft_counter_ants_[offset_fft] = 0;
                        if (isPilot(subframe_id)) {   
#if DEBUG_PRINT_PER_SUBFRAME_DONE
                            printf("Main thread: pilot FFT done for frame: %d, subframe: %d, csi_counter_users: %d\n", frame_id,subframe_id, csi_counter_users_[frame_id] + 1);
#endif
                            csi_counter_users_[frame_id] ++;
                            // if csi of all UEs is ready, schedule ZF or prediction 
                            if (csi_counter_users_[frame_id] == UE_NUM) {
                                // fft_processed[frame_id] = get_time();  
                                fft_processed[frame_count_pilot_fft] = get_time();   
                                csi_counter_users_[frame_id] = 0; 
#if DEBUG_PRINT_PER_FRAME_DONE
                                printf("Main thread: pilot frame: %d, %d, finished FFT for all pilot subframes in %.5f us, pilot all received: %.2f\n", frame_id, frame_count_pilot_fft, 
                                    fft_processed[frame_count_pilot_fft]-pilot_received[frame_count_pilot_fft], pilot_all_received[frame_count_pilot_fft]-pilot_received[frame_count_pilot_fft]);
#endif
                                // count # of frame with CSI estimation
                                frame_count_pilot_fft ++;

                                // schedule ZF when traning data is not enough or prediction is not enabled
                                if (frame_count_pilot_fft<=INIT_FRAME_NUM || !DO_PREDICTION) {  
                                    // schedule normal ZF for all data subcarriers                                                                                       
                                    Event_data do_zf_task;
                                    do_zf_task.event_type = TASK_ZF;
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
                                    printf("Main thread: created ZF tasks for frame: %d\n", frame_id);
#endif
                                    for (int i = 0; i < zf_block_num; i++) {
                                        do_zf_task.data = generateOffset2d(OFDM_DATA_NUM, frame_id, i * zf_block_size);
                                        schedule_task(do_zf_task, &zf_queue_, ptok_zf);
                                    }
                                }
                                // schedule prediction when traning data is enough and prediction is enabled
                                // when frame count equals to INIT_FRAME_NUM, do both prediction and ZF
                                if (frame_count_pilot_fft>=INIT_FRAME_NUM && DO_PREDICTION) {                              
                                    Event_data do_pred_task;
                                    do_pred_task.event_type = TASK_PRED;
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
                                    printf("Main thread: created Prediction tasks for frame: %d\n", frame_id);
#endif
                                    for (int i = 0; i < OFDM_DATA_NUM; i++) {
                                        do_pred_task.data = generateOffset2d(OFDM_DATA_NUM, frame_id, i);
                                        schedule_task(do_pred_task, &zf_queue_, ptok_zf);
                                    }
                                }
                                // reset frame_count to avoid overflow
                                if (frame_count_pilot_fft==1e9) 
                                    frame_count_pilot_fft=0;
                            }
                        }
                        else if (isData(subframe_id)) {
#if DEBUG_PRINT_PER_SUBFRAME_DONE                           
                            printf("Main thread: finished FFT for frame %d, subframe %d, precoder status: %d, fft queue: %d, zf queue: %d, demul queue: %d, in %.5f\n", 
                                    frame_id, subframe_id, 
                                    precoder_exist_in_frame_[frame_id], fft_queue_.size_approx(), zf_queue_.size_approx(), demul_queue_.size_approx(),
                                    get_time()-pilot_received[frame_count_pilot_fft-1]);
#endif                            
                            data_exist_in_subframe_[frame_id][getULSFIndex(subframe_id)] = true;
                            // if precoder exist, schedule demodulation
                            if (precoder_exist_in_frame_[frame_id]) {
                                int start_sche_id = subframe_id;
                                if (!prev_demul_scheduled) {
                                    start_sche_id = UE_NUM;
                                    prev_demul_scheduled = true;
                                }
                                // printf("Main thread: schedule Demodulation task for frame: %d, start subframe: %d, end subframe: %d\n", frame_id, start_sche_id, data_counter_subframes_[frame_id] + UE_NUM);
                                int end_sche_id = UE_NUM + data_counter_subframes_[frame_id];
                                if (end_sche_id < subframe_id)
                                    end_sche_id = subframe_id;
                                for (int sche_subframe_id = start_sche_id; sche_subframe_id <= end_sche_id; sche_subframe_id++) {
                                    int data_subframe_id = getULSFIndex(sche_subframe_id);
                                    if (data_exist_in_subframe_[frame_id][data_subframe_id]) {
                                        Event_data do_demul_task;
                                        do_demul_task.event_type = TASK_DEMUL;

                                        // schedule demodulation task for subcarrier blocks
                                        for(int i = 0; i < demul_block_num; i++) {
                                            do_demul_task.data = generateOffset3d(OFDM_DATA_NUM, frame_id, data_subframe_id, i * demul_block_size);
                                            schedule_task(do_demul_task, &demul_queue_, ptok_demul);
                                        }
#if DEBUG_PRINT_PER_SUBFRAME_ENTER_QUEUE
                                            printf("Main thread: created Demodulation task for frame: %d,, start sc: %d, subframe: %d\n", frame_id, start_sche_id, sche_subframe_id);
#endif                                         
                                        // clear data status after scheduling
                                        data_exist_in_subframe_[frame_id][data_subframe_id] = false;
                                    }
                                }
                            }

                            data_counter_subframes_[frame_id] ++;
                            if (data_counter_subframes_[frame_id] == data_subframe_num_perframe) {                           
#if DEBUG_PRINT_PER_FRAME_DONE
                                    printf("Main thread: data frame: %d, %d, finished FFT for all data subframes in %.5f us\n", frame_id, frame_count_pilot_fft-1, get_time()-pilot_received[frame_count_pilot_fft-1]);
#endif                                
                                prev_demul_scheduled = false;
                            }     
                        }
                    }
                }
                break;
            
            case EVENT_ZF: {
                    int offset_zf = event.data;
                    int frame_id, sc_id;
                    interpreteOffset2d(OFDM_DATA_NUM, offset_zf, &frame_id, &sc_id);
                    precoder_counter_scs_[frame_id] ++;
                    // precoder_exist_in_sc_[frame_id][sc_id] = true;
                    // printf("Main thread: ZF done frame: %d, subcarrier %d\n", frame_id, sc_id);
                    if (precoder_counter_scs_[frame_id] == zf_block_num) { 
                        zf_processed[frame_count_zf] = get_time(); 
                        if (frame_count_zf % 512 == 200) {
                            // double temp = zf_processed[frame_count_zf+128];
                            _mm_prefetch((char*)(&zf_processed[frame_count_zf+512]), _MM_HINT_T0);
                        }

                        // if all the data in a frame has arrived when ZF is done
                        if (data_counter_subframes_[frame_id] == data_subframe_num_perframe) {  
                            for (int sche_subframe_id = UE_NUM; sche_subframe_id < subframe_num_perframe; sche_subframe_id++) {
                                int data_subframe_id = getULSFIndex(sche_subframe_id);
                                if (data_exist_in_subframe_[frame_id][data_subframe_id]) {
                                    Event_data do_demul_task;
                                    do_demul_task.event_type = TASK_DEMUL;

                                    // schedule demodulation task for subcarrier blocks
                                    for(int i = 0; i < demul_block_num; i++) {
                                        do_demul_task.data = generateOffset3d(OFDM_DATA_NUM, frame_id, data_subframe_id, i * demul_block_size);
                                        schedule_task(do_demul_task, &demul_queue_, ptok_demul);
                                    }
#if DEBUG_PRINT_PER_SUBFRAME_ENTER_QUEUE
                                    printf("Main thread: created Demodulation task in ZF for frame: %d, subframe: %d\n", frame_id, sche_subframe_id);
#endif 
                                    data_exist_in_subframe_[frame_id][data_subframe_id] = false;
                                }
                            }
                        }   
                        // if downlink data transmission is enabled, schedule downlink modulation for all data subframes
#if ENABLE_DOWNLINK
                        Event_data do_precode_task;
                        do_precode_task.event_type = TASK_PRECODE;
                        for(int j = 0; j < demul_block_num; j++) {
                            do_precode_task.data = generateOffset3d(OFDM_DATA_NUM, frame_id, dl_data_subframe_start, j * demul_block_size);
                            schedule_task(do_precode_task, &precode_queue_, ptok_precode);
                        }

                        // for (int i = 0; i < data_subframe_num_perframe; i++) {
                        //     for(int j = 0; j < OFDM_DATA_NUM / demul_block_size; j++) {
                        //         do_precode_task.data = generateOffset3d(OFDM_DATA_NUM, frame_id, i, j * demul_block_size);
                        //         schedule_task(do_precode_task, &precode_queue_, ptok_precode);
                        //     }
                        // }

#endif
#if DEBUG_PRINT_PER_FRAME_DONE                        
                        printf("Main thread: ZF done frame: %d, %d in %.5f us, total: %.5f us\n", frame_id, frame_count_zf, zf_processed[frame_count_zf]-fft_processed[frame_count_zf],
                            zf_processed[frame_count_zf]-pilot_received[frame_count_zf]);
#endif                    
                        frame_count_zf++;
                        precoder_counter_scs_[frame_id] = 0;
                        precoder_exist_in_frame_[frame_id] = true;
                        if (frame_count_zf == 1e9) 
                            frame_count_zf = 0;

#if DEBUG_PRINT_SUMMARY_100_FRAMES
                        zf_count++;
                        if (zf_count == 100) {
                            auto zf_end = std::chrono::high_resolution_clock::now();
                            std::chrono::duration<double> diff = zf_end - zf_begin;         
                            printf("Main thread: finished ZF for 100 frames in %f secs\n", diff.count());
                            zf_count = 0;
                            zf_begin = std::chrono::high_resolution_clock::now();
                        }
#endif
                    }
                }
                break;

            case EVENT_DEMUL: {
                    // do nothing
                    int offset_demul = event.data;                   
                    int frame_id, total_data_subframe_id, data_subframe_id, sc_id;
                    interpreteOffset3d(OFDM_DATA_NUM, offset_demul, &frame_id, &total_data_subframe_id, &data_subframe_id, &sc_id);

                    demul_counter_scs_[frame_id][data_subframe_id]++;
#if DEBUG_PRINT_PER_TASK_DONE
                    printf("Main thread: Demodulation done frame: %d, subframe: %d, subcarrier: %d\n", frame_id, data_subframe_id,  demul_counter_scs_[frame_id][data_subframe_id]);
#endif
                    // if this subframe is ready
                    if (demul_counter_scs_[frame_id][data_subframe_id] == demul_block_num) {
#if DEBUG_PRINT_PER_SUBFRAME_DONE
                        printf("Main thread: Demodulation done frame: %d, subframe: %d, counter: %d\n", frame_id, data_subframe_id, demul_counter_subframes_[frame_id]+1);
#endif

#if ENABLE_DECODE && !COMBINE_EQUAL_DECODE
                        // schedule decode
                        Event_data do_decode_task;
                        do_decode_task.event_type = TASK_DECODE;

                        for (int i = 0; i < UE_NUM; i++) {
                            for (int j = 0; j < NUM_CODE_BLOCK; j++) {
                                do_decode_task.data = generateOffset3d(OFDM_DATA_NUM * UE_NUM, frame_id, data_subframe_id, j * CODED_LEN + i);
                                schedule_task(do_decode_task, &decode_queue_, ptok_decode);
                            }
                        }
#endif
                        max_equaled_frame = frame_id;
                        demul_counter_scs_[frame_id][data_subframe_id] = 0;
                        demul_counter_subframes_[frame_id]++;
                        if (demul_counter_subframes_[frame_id] == data_subframe_num_perframe) {
                            // demul_processed[frame_id] = get_time(); 
                            demul_processed[frame_count_demul] = get_time(); 
                            precoder_exist_in_frame_[frame_id] = false;
                            data_counter_subframes_[frame_id] = 0;

                            // schedule fft for next frame
                            int frame_id_next = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                            if (delay_fft_queue_cnt[frame_id_next] > 0) {
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
                                printf("Main thread in demul: schedule fft for %d packets for frame %d is done\n", delay_fft_queue_cnt[frame_id_next], frame_id_next);
#endif                                
                                Event_data do_fft_task;
                                do_fft_task.event_type = TASK_FFT;
                                if (fft_created_counter_packets_[frame_id_next] == 0) {
                                    processing_started[frame_count_demul + 1] = get_time();
                                }
                                for (int i = 0; i < delay_fft_queue_cnt[frame_id_next]; i++) {
                                    int offset_rx = delay_fft_queue[frame_id_next][i];
                                    do_fft_task.data = offset_rx;
                                    schedule_task(do_fft_task, &fft_queue_, ptok);
                                    // update statistics for FFT tasks                                   
                                    fft_created_counter_packets_[frame_id_next]++;                          
                                    if (fft_created_counter_packets_[frame_id_next] == BS_ANT_NUM * subframe_num_perframe) {
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE                                    
                                        printf("Main thread in demul: created FFT tasks for all packets in frame: %d in %.5f us\n", frame_id + 1, get_time()-pilot_received[frame_id_next]);
#endif                                        
                                        fft_created_counter_packets_[frame_id_next] = 0;
                                        demul_counter_subframes_[frame_id] = 0;
                                    }
                                }
                                delay_fft_queue_cnt[frame_id_next] = 0;
                            }
#if BIGSTATION
                            demul_counter_subframes_[frame_id] = 0;
#endif
                            
#if DEBUG_PRINT_PER_FRAME_DONE
                            printf("Main thread: Demodulation done frame: %d, %d, subframe %d in %.5f us, total %.5f us\n", frame_id, frame_count_demul, data_subframe_id, 
                                demul_processed[frame_count_demul]-zf_processed[frame_count_demul], demul_processed[frame_count_demul]-pilot_received[frame_count_demul]);
#endif

/****************************/
/* stats update starts here*/

#if DEBUG_UPDATE_STATS                    
                            double sum_ZF_this_frame[4] = {0};
                            double sum_demul_this_frame[4] = {0};
                            double sum_FFT_this_frame[4] = {0};
                            double sum_CSI_this_frame[4] = {0};
                            int csi_count_this_frame = 0;
                            int fft_count_this_frame = 0;
                            int zf_count_this_frame = 0;
                            int demul_count_this_frame = 0;

#if BIGSTATION
                            for (int i = 0; i < FFT_THREAD_NUM; i++) {
                                int fft_count_this_frame_this_thread = FFT_task_count[i * 16]-fft_count_per_thread[i];
                                double fft_time_this_frame_this_thread[4];
                                for (int j = 0; j < 4; j++) {
                                    sum_FFT_this_frame[j] = sum_FFT_this_frame[j] + FFT_task_duration[i * 8][j];                        
                                    fft_time_this_frame_this_thread[j] = (FFT_task_duration[i * 8][j]-fft_time_per_thread[j][i])/fft_count_this_frame_this_thread;
                                    fft_time_per_thread[j][i] = FFT_task_duration[i * 8][j];
                                }
                                fft_count_this_frame += fft_count_this_frame_this_thread;
                                float sum_time_this_frame_this_thread = fft_time_this_frame_this_thread[0] * fft_count_this_frame_this_thread;


                                int csi_count_this_frame_this_thread = CSI_task_count[i * 16]-csi_count_per_thread[i];
                                double csi_time_this_frame_this_thread[4];
                                for (int j = 0; j < 4; j++) {
                                    sum_CSI_this_frame[j] = sum_CSI_this_frame[j] + CSI_task_duration[i * 8][j];                        
                                    csi_time_this_frame_this_thread[j] = (CSI_task_duration[i * 8][j]-csi_time_per_thread[j][i])/csi_count_this_frame_this_thread;
                                    csi_time_per_thread[j][i] = CSI_task_duration[i * 8][j];
                                }
                                csi_count_this_frame += csi_count_this_frame_this_thread;
                                sum_time_this_frame_this_thread += csi_time_this_frame_this_thread[0] * csi_count_this_frame_this_thread;
    #if DEBUG_PRINT_STATS_PER_THREAD
                                printf("In frame %d, thread %d, \t\t\t csi: %d tasks %.5f (%.5f, %.5f, %.5f), fft: %d tasks %.5f (%.5f, %.5f, %.5f), sum: %.5f\n",
                                        frame_id, i, csi_count_this_frame_this_thread, 
                                        csi_time_this_frame_this_thread[0], csi_time_this_frame_this_thread[1], csi_time_this_frame_this_thread[2], csi_time_this_frame_this_thread[3],
                                        fft_count_this_frame_this_thread, 
                                        fft_time_this_frame_this_thread[0], fft_time_this_frame_this_thread[1], fft_time_this_frame_this_thread[2], fft_time_this_frame_this_thread[3],
                                        sum_time_this_frame_this_thread);
    #endif
                                fft_count_per_thread[i] = FFT_task_count[i * 16];
                                csi_count_per_thread[i] = CSI_task_count[i * 16];
                            }   

                            for (int i = FFT_THREAD_NUM; i < FFT_THREAD_NUM + ZF_THREAD_NUM; i++) {
                                int zf_count_this_frame_this_thread = ZF_task_count[i * 16]-zf_count_per_thread[i];
                                double zf_time_this_frame_this_thread[4];
                                for (int j = 0; j < 4; j++) {
                                    sum_ZF_this_frame[j] = sum_ZF_this_frame[j] + ZF_task_duration[i * 8][j];                               
                                    zf_time_this_frame_this_thread[j] = (ZF_task_duration[i * 8][j]-zf_time_per_thread[j][i])/zf_count_this_frame_this_thread;
                                    zf_time_per_thread[j][i] = ZF_task_duration[i * 8][j];
                                }
                                zf_count_this_frame += zf_count_this_frame_this_thread;
                                float sum_time_this_frame_this_thread = zf_time_this_frame_this_thread[0] * zf_count_this_frame_this_thread;
    #if DEBUG_PRINT_STATS_PER_THREAD
                                printf("In frame %d, thread %d, \t\t\t zf: %d tasks %.5f (%.5f, %.5f, %.5f), sum: %.5f\n",
                                        frame_id, i, zf_count_this_frame_this_thread, 
                                        zf_time_this_frame_this_thread[0], zf_time_this_frame_this_thread[1], zf_time_this_frame_this_thread[2], zf_time_this_frame_this_thread[3],
                                        sum_time_this_frame_this_thread);
    #endif
                                zf_count_per_thread[i] = ZF_task_count[i * 16];
                            }   

                            for (int i = FFT_THREAD_NUM + ZF_THREAD_NUM; i < TASK_THREAD_NUM; i++) {
                                int demul_count_this_frame_this_thread = Demul_task_count[i * 16]-demul_count_per_thread[i];
                                double demul_time_this_frame_this_thread[4];
                                for (int j = 0; j < 4; j++) {
                                    sum_demul_this_frame[j] = sum_demul_this_frame[j] + Demul_task_duration[i * 8][j];
                                    demul_time_this_frame_this_thread[j] = (Demul_task_duration[i * 8][j]-demul_time_per_thread[j][i])/demul_count_this_frame_this_thread;
                                    demul_time_per_thread[j][i] = Demul_task_duration[i * 8][j];
                                }
                                demul_count_this_frame += demul_count_this_frame_this_thread;
                                float sum_time_this_frame_this_thread = demul_time_this_frame_this_thread[0] * demul_count_this_frame_this_thread;
    #if DEBUG_PRINT_STATS_PER_THREAD
                                printf("In frame %d, thread %d, \t\t\t demul: %d tasks %.5f (%.5f, %.5f, %.5f), sum: %.5f\n",
                                        frame_id, i, demul_count_this_frame_this_thread, demul_time_this_frame_this_thread[0], demul_time_this_frame_this_thread[1], 
                                        demul_time_this_frame_this_thread[2], demul_time_this_frame_this_thread[3], 
                                        sum_time_this_frame_this_thread);
    #endif
                                demul_count_per_thread[i] = Demul_task_count[i * 16];
                            }  

                            for (int i = 0; i < 4; i++) {
                                csi_time_this_frame[i] = (sum_CSI_this_frame[i]-csi_time_sum[i])/FFT_THREAD_NUM;
                                fft_time_this_frame[i] = (sum_FFT_this_frame[i]-fft_time_sum[i])/FFT_THREAD_NUM;
                                zf_time_this_frame[i] = (sum_ZF_this_frame[i]-zf_time_sum[i])/ZF_THREAD_NUM;
                                if (i < 2)
                                    demul_time_this_frame[i] = (sum_demul_this_frame[i]-demul_time_sum[i])/DEMUL_THREAD_NUM;
                            }

#else                           
                            for (int i = 0; i < TASK_THREAD_NUM; i++) {
                                int csi_count_this_frame_this_thread = CSI_task_count[i * 16]-csi_count_per_thread[i];
                                int fft_count_this_frame_this_thread = FFT_task_count[i * 16]-fft_count_per_thread[i];
                                int zf_count_this_frame_this_thread = ZF_task_count[i * 16]-zf_count_per_thread[i];
                                int demul_count_this_frame_this_thread = Demul_task_count[i * 16]-demul_count_per_thread[i];

                                double csi_time_this_frame_this_thread[4];
                                double fft_time_this_frame_this_thread[4];
                                double zf_time_this_frame_this_thread[4];
                                double demul_time_this_frame_this_thread[4];

                                for (int j = 0; j < 4; j++) {
                                    sum_CSI_this_frame[j] = sum_CSI_this_frame[j] + CSI_task_duration[i * 8][j];                                                              
                                    csi_time_this_frame_this_thread[j] = (CSI_task_duration[i * 8][j]-csi_time_per_thread[j][i])/csi_count_this_frame_this_thread;
                                    csi_time_per_thread[j][i] = CSI_task_duration[i * 8][j];

                                    sum_FFT_this_frame[j] = sum_FFT_this_frame[j] + FFT_task_duration[i * 8][j];                                                              
                                    fft_time_this_frame_this_thread[j] = (FFT_task_duration[i * 8][j]-fft_time_per_thread[j][i])/fft_count_this_frame_this_thread;
                                    fft_time_per_thread[j][i] = FFT_task_duration[i * 8][j];

                                    sum_ZF_this_frame[j] = sum_ZF_this_frame[j] + ZF_task_duration[i * 8][j];
                                    zf_time_this_frame_this_thread[j] = (ZF_task_duration[i * 8][j]-zf_time_per_thread[j][i])/zf_count_this_frame_this_thread;
                                    zf_time_per_thread[j][i] = ZF_task_duration[i * 8][j];

                                    // if (j < 2) {
                                    sum_demul_this_frame[j] = sum_demul_this_frame[j] + Demul_task_duration[i * 8][j];
                                    demul_time_this_frame_this_thread[j] = (Demul_task_duration[i * 8][j]-demul_time_per_thread[j][i])/demul_count_this_frame_this_thread;
                                    demul_time_per_thread[j][i] = Demul_task_duration[i * 8][j];
                                    // }
                                }

                                csi_count_this_frame += fft_count_this_frame_this_thread;
                                fft_count_this_frame += fft_count_this_frame_this_thread;
                                zf_count_this_frame += zf_count_this_frame_this_thread;
                                demul_count_this_frame += demul_count_this_frame_this_thread;

                                float sum_time_this_frame_this_thread = csi_time_this_frame_this_thread[0] * csi_count_this_frame_this_thread 
                                     + fft_time_this_frame_this_thread[0] * fft_count_this_frame_this_thread
                                     + zf_time_this_frame_this_thread[0] * zf_count_this_frame_this_thread + demul_time_this_frame_this_thread[0] * demul_count_this_frame_this_thread;
    #if DEBUG_PRINT_STATS_PER_THREAD
                                printf("In frame %d, thread %d, \t csi: %d tasks %.5f (%.5f, %.5f, %.5f), fft: %d tasks %.5f (%.5f, %.5f, %.5f), zf: %d tasks %.5f (%.5f, %.5f, %.5f), demul: %d tasks %.5f (%.5f, %.5f, %.5f), sum: %.5f\n",
                                        frame_id, i, 
                                        csi_count_this_frame_this_thread, csi_time_this_frame_this_thread[0], csi_time_this_frame_this_thread[1], csi_time_this_frame_this_thread[2], csi_time_this_frame_this_thread[3],
                                        fft_count_this_frame_this_thread, fft_time_this_frame_this_thread[0], fft_time_this_frame_this_thread[1], fft_time_this_frame_this_thread[2], fft_time_this_frame_this_thread[3],
                                        zf_count_this_frame_this_thread, zf_time_this_frame_this_thread[0], zf_time_this_frame_this_thread[1], zf_time_this_frame_this_thread[2], zf_time_this_frame_this_thread[3], 
                                        demul_count_this_frame_this_thread, demul_time_this_frame_this_thread[0], demul_time_this_frame_this_thread[1], demul_time_this_frame_this_thread[2], demul_time_this_frame_this_thread[3], 
                                        sum_time_this_frame_this_thread);
    #endif
                                csi_count_per_thread[i] = CSI_task_count[i * 16];
                                fft_count_per_thread[i] = FFT_task_count[i * 16];
                                zf_count_per_thread[i] = ZF_task_count[i * 16];
                                demul_count_per_thread[i] = Demul_task_count[i * 16];
                            }                  

                            for (int i = 0; i < 4; i++) {
                                csi_time_this_frame[i] = (sum_CSI_this_frame[i]-csi_time_sum[i])/TASK_THREAD_NUM;
                                fft_time_this_frame[i] = (sum_FFT_this_frame[i]-fft_time_sum[i])/TASK_THREAD_NUM;
                                zf_time_this_frame[i] = (sum_ZF_this_frame[i]-zf_time_sum[i])/TASK_THREAD_NUM;
                                // if (i < 2)
                                demul_time_this_frame[i] = (sum_demul_this_frame[i]-demul_time_sum[i])/TASK_THREAD_NUM;
                            }     
#endif
                            

                            double sum_time_this_frame = csi_time_this_frame[0] + fft_time_this_frame[0] + zf_time_this_frame[0] + demul_time_this_frame[0];
                            csi_time_in_function[frame_count_demul] = csi_time_this_frame[0];
                            fft_time_in_function[frame_count_demul] = fft_time_this_frame[0];
                            zf_time_in_function[frame_count_demul] = zf_time_this_frame[0];
                            demul_time_in_function[frame_count_demul] = demul_time_this_frame[0];
#if DEBUG_UPDATE_STATS_DETAILED
                            for (int i = 0; i < 3; i++) {
                                csi_time_in_function_details[i][frame_count_demul] = csi_time_this_frame[i+1];
                                fft_time_in_function_details[i][frame_count_demul] = fft_time_this_frame[i+1];
                                zf_time_in_function_details[i][frame_count_demul] = zf_time_this_frame[i+1];
                                demul_time_in_function_details[i][frame_count_demul] = demul_time_this_frame[i+1];
                            }
#endif
#if DEBUG_PRINT_PER_FRAME_DONE
                            printf("In frame %d, \t\t\t\t\t csi: %d tasks %.5f (%.5f, %.5f, %.5f), fft: %d tasks %.5f (%.5f, %.5f, %.5f), zf: %d tasks %.5f (%.5f, %.5f, %.5f), demul: %d tasks %.5f (%.5f, %.5f, %.5f), sum: %.5f\n", 
                                    frame_id, csi_count_this_frame, csi_time_this_frame[0], csi_time_this_frame[1], csi_time_this_frame[2], csi_time_this_frame[3], 
                                    fft_count_this_frame, fft_time_this_frame[0], fft_time_this_frame[1], fft_time_this_frame[2], fft_time_this_frame[3], 
                                    zf_count_this_frame, zf_time_this_frame[0], zf_time_this_frame[1], zf_time_this_frame[2], zf_time_this_frame[3], 
                                    demul_count_this_frame, demul_time_this_frame[0], demul_time_this_frame[1], demul_time_this_frame[2], demul_time_this_frame[3],
                                    sum_time_this_frame);
#endif
                            for (int i = 0; i < 4; i++) {
                                csi_time_sum[i] = sum_CSI_this_frame[i];
                                fft_time_sum[i] = sum_FFT_this_frame[i];
                                zf_time_sum[i] = sum_ZF_this_frame[i];
                                // if (i < 2)
                                demul_time_sum[i] = sum_demul_this_frame[i];
                            }   
                      
#endif

/* stats update ends here*/
/****************************/


                            frame_count_demul++;
                            if (frame_count_demul == 1e9) 
                                frame_count_demul = 0;          
                        }

#if WRITE_DEMUL
                        FILE* fp = fopen("demul_data.txt","a");
                        for (int cc = 0; cc < OFDM_DATA_NUM; cc++)
                        {
                            int *cx = &demul_hard_buffer_[total_data_subframe_id][cc * UE_NUM];
                            fprintf(fp, "SC: %d, Frame %d, subframe: %d, ", cc, frame_id, data_subframe_id);
                            for(int kk = 0; kk < UE_NUM; kk++)  
                                fprintf(fp, "%d ", cx[kk]);
                            fprintf(fp, "\n");
                        }
                        fclose(fp);
#endif
                        
                        demul_count += 1;
                        
                        
                        // print log per 100 frames
                        if (demul_count == data_subframe_num_perframe * 9000)
                        {
                            demul_count = 0;
                            auto demul_end = std::chrono::system_clock::now();
                            std::chrono::duration<double> diff = demul_end - demul_begin;
                            int samples_num_per_UE = OFDM_DATA_NUM * data_subframe_num_perframe * 1000;
                            printf("Frame %d: Receive %d samples (per-client) from %d clients in %f secs, throughtput %f bps per-client (16QAM), current task queue length %d\n", 
                                frame_count_demul, samples_num_per_UE, UE_NUM, diff.count(), samples_num_per_UE * log2(16.0f) / diff.count(), fft_queue_.size_approx());
#if DEBUG_PRINT_SUMMARY_100_FRAMES
                            printf("frame %d: rx: %.5f, fft: %.5f, zf: %.5f, demul: %.5f, total: %.5f us\n", 0,  
                                    pilot_received[0], fft_processed[0]-pilot_received[0], 
                                    zf_processed[0]-fft_processed[0], demul_processed[0]-zf_processed[0], demul_processed[0]-pilot_received[0]);
                            for (int i = frame_count_demul-100; i < frame_count_demul; i++) {
                                printf("frame %d: duration_rx: %.5f, fft: %.5f, zf: %.5f, demul: %.5f, total: %.5f us\n", i,  
                                    pilot_received[i]-pilot_received[i-1], fft_processed[i]-pilot_received[i], 
                                    zf_processed[i]-fft_processed[i], demul_processed[i]-zf_processed[i], demul_processed[i]-pilot_received[i]);
                            }
#endif
                            demul_begin = std::chrono::system_clock::now();
                        }                       
                    }
                }
                break;

            case EVENT_PRECODE: {
                    // Precoding is done, schedule ifft 
                    int offset_precode = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
                    interpreteOffset3d(OFDM_DATA_NUM, offset_precode, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);
                    dl_data_counter_scs_[frame_id][current_data_subframe_id]++;
#if DEBUG_PRINT_PER_TASK_DONE
                    printf("Main thread: Precoding done frame: %d, subframe: %d, subcarrier: %d, offset: %d, total SCs: %d\n", 
                        frame_id, current_data_subframe_id,  sc_id, offset_precode, dl_data_counter_scs_[frame_id][current_data_subframe_id]);
#endif
                    
                    if (dl_data_counter_scs_[frame_id][current_data_subframe_id] == demul_block_num) {
                        // add ifft task for each antenna
                        
                        dl_data_counter_scs_[frame_id][current_data_subframe_id] = 0;
                        Event_data do_ifft_task;
                        do_ifft_task.event_type = TASK_IFFT;
                        for (int i = 0; i < BS_ANT_NUM; i++) {
                            do_ifft_task.data = generateOffset3d(BS_ANT_NUM, frame_id, current_data_subframe_id, i);
                            schedule_task(do_ifft_task, &ifft_queue_, ptok_ifft);
                        }
                        if (current_data_subframe_id < data_subframe_num_perframe - 1) {
                            Event_data do_precode_task;
                            do_precode_task.event_type = TASK_PRECODE;
                            for(int j = 0; j < demul_block_num; j++) {
                                do_precode_task.data = generateOffset3d(OFDM_DATA_NUM, frame_id, current_data_subframe_id + 1, j * demul_block_size);
                                schedule_task(do_precode_task, &precode_queue_, ptok_precode);
                            }
                        }

#if DEBUG_PRINT_PER_SUBFRAME_DONE
                        printf("Main thread: Precoding done for all subcarriers in frame: %d, subframe: %d\n", frame_id, current_data_subframe_id);
#endif  
                        dl_data_counter_subframes_[frame_id]++;
                        if (dl_data_counter_subframes_[frame_id] == dl_data_subframe_num_perframe) {
                            precode_processed[frame_count_precode] = get_time();                    
                            dl_data_counter_subframes_[frame_id] = 0;
#if DEBUG_PRINT_PER_FRAME_DONE
                        printf("Main thread: Precoding done for all subframes in frame: %d, frame count: %d, in %.5f us, total: %.5f us\n", frame_id, 
                            frame_count_precode, precode_processed[frame_count_precode]-zf_processed[frame_count_precode], 
                            precode_processed[frame_count_precode]-pilot_received[frame_count_precode]);
#endif                    
                        frame_count_precode++;          
                        if (frame_count_precode == 1e9) 
                            frame_count_precode = 0;
                        }
                    }
                }
                break;
            case EVENT_IFFT: {
                    // IFFT is done, schedule data transmission 
                    int offset_ifft = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, ant_id;
                    interpreteOffset3d(BS_ANT_NUM, offset_ifft, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &ant_id);
#if DEBUG_PRINT_PER_TASK_DONE
                    printf("Main thread: IFFT done frame: %d, subframe: %d, antenna: %d, checker: %d\n", frame_id, current_data_subframe_id, ant_id, ifft_checker_[frame_id]+1);
#endif
                    Event_data do_tx_task;
                    do_tx_task.event_type = TASK_SEND;
                    do_tx_task.data = offset_ifft;      

                    int ptok_id = ant_id % SOCKET_TX_THREAD_NUM;          
                    schedule_task(do_tx_task, &tx_queue_, *tx_ptok[ptok_id]);
                    ifft_checker_[frame_id] += 1;
                    if (ifft_checker_[frame_id] == BS_ANT_NUM * dl_data_subframe_num_perframe) {
                        // ifft_checker_[frame_id] = 0;

                        // schedule fft for next frame
                        int frame_id_next = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                        if (delay_fft_queue_cnt[frame_id_next] > 0) {
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
                            printf("Main thread in IFFT: schedule fft for %d packets for frame %d is done\n", delay_fft_queue_cnt[frame_id_next], frame_id_next);
#endif                                
                            Event_data do_fft_task;
                            do_fft_task.event_type = TASK_FFT;
                            if (fft_created_counter_packets_[frame_id_next] == 0) {
                                processing_started[frame_count_ifft + 1] = get_time();
                            }
                            for (int i = 0; i < delay_fft_queue_cnt[frame_id_next]; i++) {
                                int offset_rx = delay_fft_queue[frame_id_next][i];
                                do_fft_task.data = offset_rx;
                                schedule_task(do_fft_task, &fft_queue_, ptok);
                                // update statistics for FFT tasks                                   
                                fft_created_counter_packets_[frame_id_next]++;
                                if (fft_created_counter_packets_[frame_id_next] == BS_ANT_NUM * UE_NUM) {
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE                                    
                                    printf("Main thread in IFFT: created FFT tasks for all packets in frame: %d, in %.5f us\n", frame_id + 1, get_time()-pilot_received[frame_id_next]);
#endif                                        
                                    fft_created_counter_packets_[frame_id_next] = 0;
                                    ifft_checker_[frame_id] = 0;
                                }
                            }
                            delay_fft_queue_cnt[frame_id_next] = 0;
                        }
                        ifft_processed[frame_count_ifft] = get_time();
#if DEBUG_PRINT_PER_FRAME_DONE
                        printf("Main thread: IFFT done for all antennas in frame: %d, frame count: %d, in %.5f us, total: %.5f us\n", frame_id, frame_count_ifft,
                            ifft_processed[frame_count_ifft]-precode_processed[frame_count_ifft], ifft_processed[frame_count_ifft]-pilot_received[frame_count_ifft]);
#endif  
                        
                        frame_count_ifft++;
                        if (frame_count_ifft == 1e9)
                            frame_count_ifft = 0;
                        ifft_frame_count++;
                        if (ifft_frame_count == 100) {   
                            auto ifft_end = std::chrono::high_resolution_clock::now();
                            std::chrono::duration<double> diff = ifft_end - ifft_begin;         
                            // printf("Main thread: finished IFFT for 100 frames in %f secs\n", diff.count());
                            ifft_frame_count = 0;
                            total_time = 0;
                            ifft_begin = std::chrono::high_resolution_clock::now();
                        }
                    }
                }
                break;
            case EVENT_PACKAGE_SENT: {
                    // Data is sent
                    int offset_tx = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, ant_id;
                    interpreteOffset3d(BS_ANT_NUM, offset_tx, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &ant_id);

                    tx_counter_ants_[frame_id][current_data_subframe_id] += 1;
#if DEBUG_PRINT_PER_TASK_DONE
                    printf("Main thread: TX done frame: %d, subframe: %d, antenna: %d, total: %d\n", frame_id, current_data_subframe_id, ant_id,
                        tx_counter_ants_[frame_id][current_data_subframe_id]);
#endif
                    if (tx_counter_ants_[frame_id][current_data_subframe_id] == BS_ANT_NUM) {
                        if (current_data_subframe_id == dl_data_subframe_start) {
                            tx_processed_first[frame_count_tx] = get_time();
                        }
                        tx_counter_subframes_[frame_id] += 1;
                        tx_counter_ants_[frame_id][current_data_subframe_id] = 0;

                        tx_count++;
                        // print log per 100 frames
                        if (tx_count == dl_data_subframe_num_perframe * 9000)
                        {
                            tx_count = 0;
                            auto tx_end = std::chrono::high_resolution_clock::now();
                            std::chrono::duration<double> diff = tx_end - tx_begin;
                            int samples_num_per_UE = OFDM_DATA_NUM * dl_data_subframe_num_perframe * 1000;
                            printf("Transmit %d samples (per-client) to %d clients in %f secs, throughtput %f bps per-client (16QAM), current tx queue length %d\n", 
                                samples_num_per_UE, UE_NUM, diff.count(), samples_num_per_UE * log2(16.0f) / diff.count(), tx_queue_.size_approx());
                            tx_begin = std::chrono::high_resolution_clock::now();
                        }
#if DEBUG_PRINT_PER_SUBFRAME_DONE  
                        printf("Main thread: TX done for all antennas in frame: %d, subframe: %d\n", frame_id, current_data_subframe_id);
#endif
#if DEBUG_PRINT_PER_FRAME_DONE
                        if(current_data_subframe_id == dl_data_subframe_start) {
                            printf("Main thread: TX done for first subframe in frame: %d, subframe: %d in %.5f us, total: %.5f us, deadline: %.5f\n", frame_id, current_data_subframe_id,
                                tx_processed_first[frame_count_tx]-zf_processed[frame_count_tx], tx_processed_first[frame_count_tx]-pilot_received[frame_count_tx], 5000.0/subframe_num_perframe*(UE_NUM+dl_data_subframe_start));
                        }
#endif                          
                        if (tx_counter_subframes_[frame_id] == dl_data_subframe_num_perframe) {
                            tx_processed[frame_count_tx] = get_time();
                               
#if DEBUG_PRINT_PER_FRAME_DONE                                            
                            printf("Main thread: TX done for all subframes in frame: %d in %.5f us, total: %.5f us\n", frame_id,
                                tx_processed[frame_count_tx]-ifft_processed[frame_count_tx], tx_processed[frame_count_tx]-pilot_received[frame_count_tx]);  
#endif                      


#if DEBUG_UPDATE_STATS

   

                            double sum_ZF_this_frame[4] = {0};
                            double sum_Precode_this_frame[4] = {0};
                            double sum_CSI_this_frame[4] = {0};
                            double sum_IFFT_this_frame[4] = {0};
                            int csi_count_this_frame = 0;
                            int zf_count_this_frame = 0;
                            int ifft_count_this_frame = 0;
                            int precode_count_this_frame = 0;
    #if BIGSTATION
                            for (int i = 0; i < FFT_THREAD_NUM; i++) {
                                int csi_count_this_frame_this_thread = CSI_task_count[i*16]-csi_count_per_thread[i];
                                int ifft_count_this_frame_this_thread = IFFT_task_count[i*16]-ifft_count_per_thread[i];

                                double csi_time_this_frame_this_thread[4];
                                double ifft_time_this_frame_this_thread[4];

                                for (int j = 0; j < 4; j++) {
                                    sum_CSI_this_frame[j] = sum_CSI_this_frame[j] + CSI_task_duration[i*8][j];
                                    sum_IFFT_this_frame[j] = sum_IFFT_this_frame[j] + IFFT_task_duration[i*8][j];               

                                    csi_time_this_frame_this_thread[j] = (CSI_task_duration[i*8][j]-csi_time_per_thread[j][i])/csi_count_this_frame_this_thread;
                                    ifft_time_this_frame_this_thread[j] = (IFFT_task_duration[i*8][j]-ifft_time_per_thread[j][i])/ifft_count_this_frame_this_thread;
                                    
                                    csi_time_per_thread[j][i] = CSI_task_duration[i*8][j];
                                    ifft_time_per_thread[j][i] = IFFT_task_duration[i*8][j];
                                }

                                csi_count_this_frame += csi_count_this_frame_this_thread;
                                ifft_count_this_frame += ifft_count_this_frame_this_thread;

                                float sum_time_this_frame_this_thread = csi_time_this_frame_this_thread[0] * csi_count_this_frame_this_thread +
                                     ifft_time_this_frame_this_thread[0] * ifft_count_this_frame_this_thread;
        #if DEBUG_PRINT_STATS_PER_THREAD
                                printf("In frame %d, thread %d, \t\t\t csi: %d tasks %.5f (%.5f, %.5f, %.5f), ifft: %d tasks %.5f, sum: %.5f\n",
                                        frame_id, i, csi_count_this_frame_this_thread, 
                                        csi_time_this_frame_this_thread[0], csi_time_this_frame_this_thread[1], csi_time_this_frame_this_thread[2], csi_time_this_frame_this_thread[3],
                                        ifft_count_this_frame_this_thread, ifft_time_this_frame_this_thread[0], sum_time_this_frame_this_thread);
        #endif
                                csi_count_per_thread[i] = CSI_task_count[i*16];
                                ifft_count_per_thread[i] = IFFT_task_count[i*16];
                            }        


                            for (int i = FFT_THREAD_NUM; i < FFT_THREAD_NUM + ZF_THREAD_NUM; i++) {
                                int zf_count_this_frame_this_thread = ZF_task_count[i*16]-zf_count_per_thread[i];

                                double zf_time_this_frame_this_thread[4];

                                for (int j = 0; j < 4; j++) {
                                    sum_ZF_this_frame[j] = sum_ZF_this_frame[j] + ZF_task_duration[i*8][j];                       
                                    zf_time_this_frame_this_thread[j] = (ZF_task_duration[i*8][j]-zf_time_per_thread[j][i])/zf_count_this_frame_this_thread;
                                    zf_time_per_thread[j][i] = ZF_task_duration[i*8][j];
                                }

                                zf_count_this_frame += zf_count_this_frame_this_thread;

                                float sum_time_this_frame_this_thread = 
                                     + zf_time_this_frame_this_thread[0] * zf_count_this_frame_this_thread;
        #if DEBUG_PRINT_STATS_PER_THREAD
                                printf("In frame %d, thread %d, \t\t\t  zf: %d tasks %.5f (%.5f, %.5f, %.5f), sum: %.5f\n",
                                        frame_id, i, zf_count_this_frame_this_thread, 
                                        zf_time_this_frame_this_thread[0], zf_time_this_frame_this_thread[1], zf_time_this_frame_this_thread[2], zf_time_this_frame_this_thread[3],
                                        sum_time_this_frame_this_thread);
        #endif
                                
                                zf_count_per_thread[i] = ZF_task_count[i*16];
                            }     

                            for (int i = FFT_THREAD_NUM + ZF_THREAD_NUM; i < TASK_THREAD_NUM; i++) {
                                int precode_count_this_frame_this_thread = Precode_task_count[i*16]-precode_count_per_thread[i];
                                double precode_time_this_frame_this_thread[4];


                                for (int j = 0; j < 4; j++) {
                                    sum_Precode_this_frame[j] = sum_Precode_this_frame[j] + Precode_task_duration[i*8][j];                    
                                    precode_time_this_frame_this_thread[j] = (Precode_task_duration[i*8][j]-precode_time_per_thread[j][i])/precode_count_this_frame_this_thread;
                                    precode_time_per_thread[j][i] = Precode_task_duration[i*8][j];
                                }
                                precode_count_this_frame += precode_count_this_frame_this_thread;

                                float sum_time_this_frame_this_thread = precode_time_this_frame_this_thread[0] * precode_count_this_frame_this_thread;
        #if DEBUG_PRINT_STATS_PER_THREAD
                                printf("In frame %d, thread %d, \t\t\t precode: %d tasks %.5f (%.5f, %.5f, %.5f), sum: %.5f\n",
                                        frame_id, i, precode_count_this_frame_this_thread, 
                                        precode_time_this_frame_this_thread[0], precode_time_this_frame_this_thread[1], precode_time_this_frame_this_thread[2], precode_time_this_frame_this_thread[3], 
                                        sum_time_this_frame_this_thread);
        #endif
                                precode_count_per_thread[i] = Precode_task_count[i*16];
                            }   


                            for (int i = 0; i < 4; i++) {
                                csi_time_this_frame[i] = (sum_CSI_this_frame[i]-csi_time_sum[i])/FFT_THREAD_NUM;
                                zf_time_this_frame[i] = (sum_ZF_this_frame[i]-zf_time_sum[i])/ZF_THREAD_NUM;
                                ifft_time_this_frame[i] = (sum_IFFT_this_frame[i]-ifft_time_sum[i])/FFT_THREAD_NUM;
                                precode_time_this_frame[i] = (sum_Precode_this_frame[i]-precode_time_sum[i])/DEMUL_THREAD_NUM;
                            }  
    #else
                            for (int i = 0; i < TASK_THREAD_NUM; i++) {
                                int csi_count_this_frame_this_thread = CSI_task_count[i*16]-csi_count_per_thread[i];
                                int zf_count_this_frame_this_thread = ZF_task_count[i*16]-zf_count_per_thread[i];
                                int precode_count_this_frame_this_thread = Precode_task_count[i*16]-precode_count_per_thread[i];
                                int ifft_count_this_frame_this_thread = IFFT_task_count[i*16]-ifft_count_per_thread[i];

                                double csi_time_this_frame_this_thread[4];
                                double zf_time_this_frame_this_thread[4];
                                double ifft_time_this_frame_this_thread[4];
                                double precode_time_this_frame_this_thread[4];


                                for (int j = 0; j < 4; j++) {
                                    sum_CSI_this_frame[j] = sum_CSI_this_frame[j] + CSI_task_duration[i*8][j];
                                    sum_ZF_this_frame[j] = sum_ZF_this_frame[j] + ZF_task_duration[i*8][j];      
                                    sum_IFFT_this_frame[j] = sum_IFFT_this_frame[j] + IFFT_task_duration[i*8][j];
                                    sum_Precode_this_frame[j] = sum_Precode_this_frame[j] + Precode_task_duration[i*8][j];                    

                                    csi_time_this_frame_this_thread[j] = (CSI_task_duration[i*8][j]-csi_time_per_thread[j][i])/csi_count_this_frame_this_thread;
                                    zf_time_this_frame_this_thread[j] = (ZF_task_duration[i*8][j]-zf_time_per_thread[j][i])/zf_count_this_frame_this_thread;
                                    ifft_time_this_frame_this_thread[j] = (IFFT_task_duration[i*8][j]-ifft_time_per_thread[j][i])/ifft_count_this_frame_this_thread;
                                    precode_time_this_frame_this_thread[j] = (Precode_task_duration[i*8][j]-precode_time_per_thread[j][i])/precode_count_this_frame_this_thread;

                                    csi_time_per_thread[j][i] = CSI_task_duration[i*8][j];
                                    zf_time_per_thread[j][i] = ZF_task_duration[i*8][j];
                                    ifft_time_per_thread[j][i] = IFFT_task_duration[i*8][j];
                                    precode_time_per_thread[j][i] = Precode_task_duration[i*8][j];
                                }

                                csi_count_this_frame += csi_count_this_frame_this_thread;
                                zf_count_this_frame += zf_count_this_frame_this_thread;
                                ifft_count_this_frame += ifft_count_this_frame_this_thread;
                                precode_count_this_frame += precode_count_this_frame_this_thread;

                                float sum_time_this_frame_this_thread = csi_time_this_frame_this_thread[0] * csi_count_this_frame_this_thread
                                     + zf_time_this_frame_this_thread[0] * zf_count_this_frame_this_thread + 
                                     ifft_time_this_frame_this_thread[0] * ifft_count_this_frame_this_thread + precode_time_this_frame_this_thread[0] * precode_count_this_frame_this_thread;
        #if DEBUG_PRINT_STATS_PER_THREAD
                                printf("In frame %d, thread %d, \t\t\t fft: %d tasks %.5f (%.5f, %.5f, %.5f), zf: %d tasks %.5f (%.5f, %.5f, %.5f), precode: %d tasks %.5f (%.5f, %.5f, %.5f), ifft: %d tasks %.5f, sum: %.5f\n",
                                        frame_id, i, csi_count_this_frame_this_thread, 
                                        csi_time_this_frame_this_thread[0], csi_time_this_frame_this_thread[1], csi_time_this_frame_this_thread[2], csi_time_this_frame_this_thread[3],
                                        zf_count_this_frame_this_thread, zf_time_this_frame_this_thread[0], zf_time_this_frame_this_thread[1], zf_time_this_frame_this_thread[2], zf_time_this_frame_this_thread[3], 
                                        precode_count_this_frame_this_thread, 
                                        precode_time_this_frame_this_thread[0], precode_time_this_frame_this_thread[1], precode_time_this_frame_this_thread[2], precode_time_this_frame_this_thread[3], 
                                        ifft_count_this_frame_this_thread, ifft_time_this_frame_this_thread[0], sum_time_this_frame_this_thread);
        #endif
                                csi_count_per_thread[i] = CSI_task_count[i*16];
                                zf_count_per_thread[i] = ZF_task_count[i*16];
                                ifft_count_per_thread[i] = IFFT_task_count[i*16];
                                precode_count_per_thread[i] = Precode_task_count[i*16];
                            }                  

                            for (int i = 0; i < 4; i++) {
                                csi_time_this_frame[i] = (sum_CSI_this_frame[i]-csi_time_sum[i])/TASK_THREAD_NUM;
                                zf_time_this_frame[i] = (sum_ZF_this_frame[i]-zf_time_sum[i])/TASK_THREAD_NUM;
                                ifft_time_this_frame[i] = (sum_IFFT_this_frame[i]-ifft_time_sum[i])/TASK_THREAD_NUM;
                                precode_time_this_frame[i] = (sum_Precode_this_frame[i]-precode_time_sum[i])/TASK_THREAD_NUM;
                            }     
    #endif
                            
                            double sum_time_this_frame = csi_time_this_frame[0] + zf_time_this_frame[0] + precode_time_this_frame[0] + ifft_time_this_frame[0];
                            csi_time_in_function[frame_count_tx] = csi_time_this_frame[0];
                            zf_time_in_function[frame_count_tx] = zf_time_this_frame[0];
                            ifft_time_in_function[frame_count_tx] = ifft_time_this_frame[0];
                            precode_time_in_function[frame_count_tx] = precode_time_this_frame[0];
    #if DEBUG_PRINT_PER_FRAME_DONE
                            printf("In frame %d, \t\t\t\t\t csi: %d tasks %.5f (%.5f, %.5f, %.5f), zf: %d tasks %.5f (%.5f, %.5f, %.5f), precode: %d tasks %.5f, ifft: %d tasks %.5f, sum: %.5f\n", 
                                    frame_id, csi_count_this_frame, csi_time_this_frame[0], csi_time_this_frame[1], csi_time_this_frame[2], csi_time_this_frame[3], 
                                    zf_count_this_frame, zf_time_this_frame[0], zf_time_this_frame[1], zf_time_this_frame[2], zf_time_this_frame[3], 
                                    precode_count_this_frame, precode_time_this_frame[0], ifft_count_this_frame, ifft_time_this_frame[0], sum_time_this_frame);
    #endif
                            for (int i = 0; i < 4; i++) {
                                csi_time_sum[i] = sum_CSI_this_frame[i];
                                zf_time_sum[i] = sum_ZF_this_frame[i];
                                ifft_time_sum[i] = sum_IFFT_this_frame[i];
                                precode_time_sum[i] = sum_Precode_this_frame[i];
                            }   


#endif





                            frame_count_tx++;
                            if (frame_count_tx == 1e9) 
                                frame_count_tx = 0;
                            tx_counter_subframes_[frame_id] = 0; 
                        }
        
                    }                    
                }
                break;
            default:
                printf("Wrong event type in message queue!");
                exit(0);
            }
            // queue_process_time = get_time() - queue_process_start;
            // if (queue_process_time > 50)
            //     printf("In frame %d, queue process time %.5f us\n", frame_count_zf, queue_process_time);
        }
    }
    // }


    printf("Print results\n");
    FILE* fp_debug = fopen("../matlab/timeresult.txt", "w");
    if (fp_debug==NULL) {
        printf("open file faild");
        std::cerr << "Error: " << strerror(errno) << std::endl;
        exit(0);
    }
    
    printf("Total processed frames %d ", frame_count_zf);
#if ENABLE_DOWNLINK
    for(int ii = 0; ii < frame_count_tx; ii++) { 
        if (SOCKET_RX_THREAD_NUM == 1) {
            fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], 
                precode_processed[ii], ifft_processed[ii], tx_processed[ii],tx_processed_first[ii],
                csi_time_in_function[ii], zf_time_in_function[ii], precode_time_in_function[ii], ifft_time_in_function[ii], processing_started[ii], frame_start[0][ii]);
        } 
        else {
            fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], 
                precode_processed[ii], ifft_processed[ii], tx_processed[ii],tx_processed_first[ii],
                csi_time_in_function[ii], zf_time_in_function[ii], precode_time_in_function[ii], ifft_time_in_function[ii], processing_started[ii], frame_start[0][ii], frame_start[1][ii]);
        }
    }
#else
    for(int ii = 0; ii < frame_count_demul; ii++) {  
        // switch (SOCKET_RX_THREAD_NUM) {
        //     case 1: {
        //         fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
        //                 csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii]);
        //     }
        //     break;
        //     case 2: {
        //         fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
        //             csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii], frame_start[1][ii] );
        //     }
        //     break;
        //     case 3: {
        //         fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
        //             csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii], frame_start[1][ii], frame_start[2][ii]);
        //     }
        //     break;
        //     case 4: {
        //         fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
        //             csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii], frame_start[1][ii], frame_start[2][ii], frame_start[3][ii]);
        //     }
        // }
        if (SOCKET_RX_THREAD_NUM == 1) {
            
                fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
                        csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii], pilot_all_received[ii]);
        }
        else {
                    
                fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
                    csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii], frame_start[1][ii], pilot_all_received[ii]);
        }
    }
    #if DEBUG_UPDATE_STATS_DETAILED
        printf("Print results detailed\n");
        FILE* fp_debug_detailed = fopen("../timeresult_detail.txt", "w");
        if (fp_debug_detailed==NULL) {
            printf("open file faild");
            std::cerr << "Error: " << strerror(errno) << std::endl;
            exit(0);
        }

        for(int ii = 0; ii < frame_count_demul; ii++) {    
            fprintf(fp_debug_detailed, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f \n", fft_time_in_function_details[0][ii], fft_time_in_function_details[1][ii],
                fft_time_in_function_details[2][ii], zf_time_in_function_details[0][ii], zf_time_in_function_details[1][ii], zf_time_in_function_details[2][ii],
                demul_time_in_function_details[0][ii], demul_time_in_function_details[1][ii], demul_time_in_function_details[2][ii] );
        }

    #endif
#endif
    
    // printf("\n");
    // fwrite(mat_demuled2.memptr(), sizeof(int),sizeof(mat_demuled), fp_debug);
    fclose(fp_debug);

    int sum_CSI = 0;
    int sum_FFT = 0;
    int sum_ZF = 0;
    int sum_demul = 0;
    int sum_IFFT = 0;
    int sum_Precode = 0;
    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        sum_CSI = sum_CSI + CSI_task_count[i * 16];
        sum_FFT = sum_FFT + FFT_task_count[i * 16];
        sum_ZF = sum_ZF + ZF_task_count[i * 16];
        sum_demul = sum_demul + Demul_task_count[i * 16];
        sum_IFFT = sum_IFFT + IFFT_task_count[i * 16];
        sum_Precode = sum_Precode + Precode_task_count[i * 16];
    }
    printf("Total dequeue trials: %d, missed %d\n", total_count, miss_count);
#if ENABLE_DOWNLINK
    double csi_frames = (double)sum_CSI/BS_ANT_NUM/UE_NUM;
    double precode_frames = (double)sum_Precode * demul_block_size/ OFDM_DATA_NUM / dl_data_subframe_num_perframe;
    double ifft_frames = (double)sum_IFFT / BS_ANT_NUM / dl_data_subframe_num_perframe;
    double zf_frames = (double)sum_ZF/OFDM_DATA_NUM;
    printf("Downlink: total performed FFT: %d (%.2f frames), ZF: %d (%.2f frames), precode: %d (%.2f frames), IFFT: %d (%.2f frames)\n", 
        sum_CSI, csi_frames, sum_ZF, zf_frames, sum_Precode, precode_frames, sum_IFFT, ifft_frames);
    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        double percent_FFT = 100*double(CSI_task_count[i * 16])/sum_CSI;
        double percent_ZF = 100*double(ZF_task_count[i * 16])/sum_ZF;
        double percent_Precode = 100*double(Precode_task_count[i * 16])/sum_Precode;
        double percent_IFFT = 100*double(IFFT_task_count[i * 16])/sum_IFFT;
        printf("thread %d performed FFT: %d (%.2f%%), ZF: %d (%.2f%%), precode: %d (%.2f%%), IFFT: %d (%.2f%%)\n", 
            i, CSI_task_count[i * 16], percent_FFT, ZF_task_count[i * 16], percent_ZF, Precode_task_count[i * 16], percent_Precode, IFFT_task_count[i * 16], percent_IFFT);
    }
#else
    double csi_frames = (double)sum_CSI/BS_ANT_NUM/UE_NUM;
    double fft_frames = (double)sum_FFT/BS_ANT_NUM/data_subframe_num_perframe;
    double demul_frames = (double)sum_demul/ OFDM_DATA_NUM / data_subframe_num_perframe;
    double zf_frames = (double)sum_ZF/OFDM_DATA_NUM;
    printf("Uplink: total performed CSI %d (%.2f frames), FFT: %d (%.2f frames), ZF: %d (%.2f frames), Demulation: %d (%.2f frames)\n", 
        sum_CSI, csi_frames, sum_FFT, fft_frames, sum_ZF, zf_frames, sum_demul, demul_frames);
    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        double percent_CSI = 100*double(CSI_task_count[i * 16])/sum_CSI;
        double percent_FFT = 100*double(FFT_task_count[i * 16])/sum_FFT;
        double percent_ZF = 100*double(ZF_task_count[i * 16])/sum_ZF;
        double percent_Demul = 100*double(Demul_task_count[i * 16])/sum_demul;
        printf("thread %d performed CSI: %d (%.2f%%), FFT: %d (%.2f%%), ZF: %d (%.2f%%), Demulation: %d (%.2f%%)\n", 
            i, CSI_task_count[i * 16], percent_CSI, FFT_task_count[i * 16], percent_FFT, ZF_task_count[i * 16], percent_ZF, Demul_task_count[i * 16], percent_Demul);
    }
#endif 
    exit(0);
    
}

void* CoMP::taskThread(void* context)
{

    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);
    
    // attach task threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    // int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
    int offset_id = SOCKET_RX_THREAD_NUM + CORE_OFFSET + 2;

    int tar_core_id = tid + offset_id;
    if (tar_core_id>=36) 
        tar_core_id = tar_core_id - 36 + 1;
    if(stick_this_thread_to_core(tar_core_id) != 0) {
        printf("Task thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else {
        printf("Task thread: stitch thread %d to core %d succeeded\n", tid, tar_core_id);
    }
#endif
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* fft_queue_ = &(obj_ptr->fft_queue_);
    moodycamel::ConcurrentQueue<Event_data>* zf_queue_ = &(obj_ptr->zf_queue_);
    moodycamel::ConcurrentQueue<Event_data>* demul_queue_ = &(obj_ptr->demul_queue_);
    moodycamel::ConcurrentQueue<Event_data>* decode_queue_ = &(obj_ptr->decode_queue_);
    moodycamel::ConcurrentQueue<Event_data>* ifft_queue_ = &(obj_ptr->ifft_queue_);
    moodycamel::ConcurrentQueue<Event_data>* modulate_queue_ = &(obj_ptr->modulate_queue_);
    moodycamel::ConcurrentQueue<Event_data>* precode_queue_ = &(obj_ptr->precode_queue_);
    // moodycamel::ConcurrentQueue<Event_data>* tx_queue_ = &(obj_ptr->tx_queue_);


    obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->complete_task_queue_));

    
    char *socket_buffer_ptrs[SOCKET_RX_THREAD_NUM];
    int *socket_buffer_status_ptrs[SOCKET_RX_THREAD_NUM];
    double *frame_start_ptrs[SOCKET_RX_THREAD_NUM];
    for(int i = 0; i < SOCKET_RX_THREAD_NUM; i++) {
        socket_buffer_ptrs[i] = obj_ptr->socket_buffer_[i].buffer;
        socket_buffer_status_ptrs[i] = obj_ptr->socket_buffer_[i].buffer_status;
    }


    int data_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *data_buffer_ptrs[data_buffer_size];
    for (int i = 0; i < data_buffer_size; i++)
        data_buffer_ptrs[i] = obj_ptr->data_buffer_.data[i];


    int csi_buffer_size = UE_NUM * TASK_BUFFER_FRAME_NUM;
    complex_float *csi_buffer_ptrs[csi_buffer_size];
    for (int i = 0; i < csi_buffer_size; i++)
        csi_buffer_ptrs[i] = obj_ptr->csi_buffer_.CSI[i];


    int IFFT_buffer_block_num = BS_ANT_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *dl_ifft_buffer_ptrs[IFFT_buffer_block_num];
    char *dl_socket_buffer_ptr;
#if ENABLE_DOWNLINK
    dl_socket_buffer_ptr = obj_ptr->dl_socket_buffer_.buffer;
    for (int i = 0; i < IFFT_buffer_block_num; i++) 
        dl_ifft_buffer_ptrs[i] = obj_ptr->dl_ifft_buffer_.IFFT_inputs[i];
#endif

    int duration_size = TASK_THREAD_NUM * 8;
    double *FFT_task_duration_ptrs[duration_size];
    double *CSI_task_duration_ptrs[duration_size];
    double *IFFT_task_duration_ptrs[duration_size];
    double *ZF_task_duration_ptrs[duration_size];
    double *Demul_task_duration_ptrs[duration_size];
    double *Precode_task_duration_ptrs[duration_size];


    for (int i = 0; i < duration_size; i++) {
        FFT_task_duration_ptrs[i] = obj_ptr->FFT_task_duration[i];
        CSI_task_duration_ptrs[i] = obj_ptr->CSI_task_duration[i];
        IFFT_task_duration_ptrs[i] = obj_ptr->IFFT_task_duration[i];
        ZF_task_duration_ptrs[i] = obj_ptr->ZF_task_duration[i];
        Demul_task_duration_ptrs[i] = obj_ptr->Demul_task_duration[i];
        Precode_task_duration_ptrs[i] = obj_ptr->Precode_task_duration[i];
    }

    moodycamel::ProducerToken *task_ptok_ptr;
    task_ptok_ptr = obj_ptr->task_ptok[tid].get();

    DoFFT *computeFFT = new DoFFT(tid, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
    socket_buffer_ptrs, socket_buffer_status_ptrs, data_buffer_ptrs, csi_buffer_ptrs, obj_ptr->pilots_,
    dl_ifft_buffer_ptrs, dl_socket_buffer_ptr, 
    FFT_task_duration_ptrs, CSI_task_duration_ptrs, obj_ptr->FFT_task_count, obj_ptr->CSI_task_count,
    IFFT_task_duration_ptrs, obj_ptr->IFFT_task_count);

    /* ========================== */

    int precoder_buffer_size = OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM;
    complex_float *precoder_buffer_ptrs[precoder_buffer_size];
    for (int i = 0; i < precoder_buffer_size; i++)
        precoder_buffer_ptrs[i] = obj_ptr->precoder_buffer_.precoder[i];

    int pred_csi_buffer_size = OFDM_DATA_NUM;
    complex_float *pred_csi_buffer_ptrs[pred_csi_buffer_size];
#if DO_PREDICTION
    for(int i = 0; i < pred_csi_buffer_size; i++)
        pred_csi_buffer_ptrs[i] = obj_ptr->pred_csi_buffer_.CSI[i];
#endif

    DoZF *computeZF = new DoZF(tid, obj_ptr->zf_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        csi_buffer_ptrs, precoder_buffer_ptrs, pred_csi_buffer_ptrs, ZF_task_duration_ptrs, obj_ptr->ZF_task_count);

    /* ========================== */

    int equal_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *equal_buffer_ptrs[equal_buffer_size];
    uint8_t *demul_hard_buffer_ptrs[equal_buffer_size];
    for (int i = 0; i < equal_buffer_size; i++) {
        equal_buffer_ptrs[i] = obj_ptr->equal_buffer_.data[i];
        demul_hard_buffer_ptrs[i] = obj_ptr->demul_hard_buffer_[i];
    }

    DoDemul *computeDemul = new DoDemul(tid, obj_ptr->demul_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        data_buffer_ptrs, precoder_buffer_ptrs, equal_buffer_ptrs, demul_hard_buffer_ptrs, Demul_task_duration_ptrs, obj_ptr->Demul_task_count);


    /* ========================== */

    int dl_precoded_data_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *dl_modulated_buffer_ptrs[dl_precoded_data_buffer_size];
    complex_float *dl_precoded_data_buffer_ptrs[dl_precoded_data_buffer_size];
#if ENABLE_DOWNLINK
    for (int i = 0; i < dl_precoded_data_buffer_size; i++) {
        dl_modulated_buffer_ptrs[i] = obj_ptr->dl_modulated_buffer_.data[i];
        dl_precoded_data_buffer_ptrs[i] = obj_ptr->dl_precoded_data_buffer_.data[i];
    }
#endif

    float *qam16_table_ptrs[2];
    for (int i = 0; i < 2; i++) 
        qam16_table_ptrs[i] = obj_ptr->qam16_table[i];

    DoPrecode *computePrecode = new DoPrecode(tid, obj_ptr->demul_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        dl_modulated_buffer_ptrs, precoder_buffer_ptrs, dl_precoded_data_buffer_ptrs, dl_ifft_buffer_ptrs, obj_ptr->dl_IQ_data, qam16_table_ptrs, 
        Precode_task_duration_ptrs, obj_ptr->Precode_task_count);

    int total_count = 0;
    int miss_count = 0;
    Event_data event;
    bool ret = false;
    bool ret_zf = false;
    bool ret_demul = false;
    bool ret_decode = false;
    bool ret_modul = false;
    bool ret_ifft = false;
    bool ret_precode = false;

    while(true) {
        if (ENABLE_DOWNLINK) {
            // do not process uplink data if downlink is enabled
            ret_ifft = ifft_queue_->try_dequeue(event);
            if (!ret_ifft) {
                ret_precode = precode_queue_->try_dequeue(event);
                if (!ret_precode) {
                    ret_zf = zf_queue_->try_dequeue(event);
                    if (!ret_zf) {
                        ret = fft_queue_->try_dequeue(event);
                        if (!ret) 
                            continue;
                        else
                            computeFFT->FFT(event.data);
                    }
                    else if (event.event_type == TASK_ZF) {
                        computeZF->ZF(event.data);
                    }
                    else if (event.event_type == TASK_PRED) {
                        computeZF->Predict(event.data);
                    }
                }
                else {
                    computePrecode->Precode(event.data);
                    // obj_ptr->do_precode(tid, event.data);
                }
            }
            else {
                computeFFT->IFFT(event.data);
                // obj_ptr->do_ifft(tid, event.data);
            }
        }
        else {
            ret_zf = zf_queue_->try_dequeue(event);
            if (!ret_zf) {
                // ret_decode = decode_queue_->try_dequeue(event);
                // if(!ret_decode) {
                    // ret_demul = demul_queue_->try_dequeue(event);
                    // if (!ret_demul) {   
                    //     ret = fft_queue_->try_dequeue(event);
                    //     if (!ret)
                    //         continue;
                    //     else 
                    //         obj_ptr->doFFT(tid, event.data);
                    // }
                    // else {
                    //     obj_ptr->doDemul(tid, event.data);
                    // }
                    ret = fft_queue_->try_dequeue(event);
                    if (!ret) {   
                        ret_demul = demul_queue_->try_dequeue(event);
                        if (!ret_demul)
                            continue;
                        else 
                            computeDemul->Demul(event.data);
                            // obj_ptr->doDemul(tid, event.data);
                    }
                    else {
                        computeFFT->FFT(event.data);
                    }
                // }
                // else {
                //     obj_ptr->doDecode(tid, event.data);
                // }
            }
            else if (event.event_type == TASK_ZF) {
                computeZF->ZF(event.data);
            }
            else if (event.event_type == TASK_PRED) {
                computeZF->Predict(event.data);
            }
        } 
    }
}



void* CoMP::fftThread(void* context)
{
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* fft_queue_ = &(obj_ptr->fft_queue_);
#if ENABLE_DOWNLINK
    moodycamel::ConcurrentQueue<Event_data>* ifft_queue_ = &(obj_ptr->ifft_queue_);
#endif
    int tid = ((EventHandlerContext *)context)->id;
    printf("FFT thread %d starts\n", tid);
    
    // attach task threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    // int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
    int offset_id = SOCKET_RX_THREAD_NUM + CORE_OFFSET + 2;
    int tar_core_id = tid + offset_id;
    if (tar_core_id>=36) 
        tar_core_id = tar_core_id - 36 + 1;
    if(stick_this_thread_to_core(tar_core_id) != 0) {
        printf("FFT thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else {
        printf("FFT thread: stitch thread %d to core %d succeeded\n", tid, tar_core_id);
    }
#endif

    obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->complete_task_queue_));

    char *socket_buffer_ptrs[SOCKET_RX_THREAD_NUM];
    int *socket_buffer_status_ptrs[SOCKET_RX_THREAD_NUM];
    double *frame_start_ptrs[SOCKET_RX_THREAD_NUM];
    for(int i = 0; i < SOCKET_RX_THREAD_NUM; i++) {
        socket_buffer_ptrs[i] = obj_ptr->socket_buffer_[i].buffer;
        socket_buffer_status_ptrs[i] = obj_ptr->socket_buffer_[i].buffer_status;
    }


    int data_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *data_buffer_ptrs[data_buffer_size];
    for (int i = 0; i < data_buffer_size; i++)
        data_buffer_ptrs[i] = obj_ptr->data_buffer_.data[i];


    int csi_buffer_size = UE_NUM * TASK_BUFFER_FRAME_NUM;
    complex_float *csi_buffer_ptrs[csi_buffer_size];
    for (int i = 0; i < csi_buffer_size; i++)
        csi_buffer_ptrs[i] = obj_ptr->csi_buffer_.CSI[i];


    int IFFT_buffer_block_num = BS_ANT_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *dl_ifft_buffer_ptrs[IFFT_buffer_block_num];
    


    char *dl_socket_buffer_ptr;
#if ENABLE_DOWNLINK
    dl_socket_buffer_ptr = obj_ptr->dl_socket_buffer_.buffer;
    for (int i = 0; i < IFFT_buffer_block_num; i++) 
        dl_ifft_buffer_ptrs[i] = obj_ptr->dl_ifft_buffer_.IFFT_inputs[i];
#endif

    int duration_size = TASK_THREAD_NUM * 8;
    double *FFT_task_duration_ptrs[duration_size];
    double *CSI_task_duration_ptrs[duration_size];
    double *IFFT_task_duration_ptrs[duration_size];

    for (int i = 0; i < duration_size; i++) {
        FFT_task_duration_ptrs[i] = obj_ptr->FFT_task_duration[i];
        CSI_task_duration_ptrs[i] = obj_ptr->CSI_task_duration[i];
        IFFT_task_duration_ptrs[i] = obj_ptr->IFFT_task_duration[i];
    }

    moodycamel::ProducerToken *task_ptok_ptr;
    task_ptok_ptr = obj_ptr->task_ptok[tid].get();

    DoFFT* computeFFT = new DoFFT(tid, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
    socket_buffer_ptrs, socket_buffer_status_ptrs, data_buffer_ptrs, csi_buffer_ptrs, obj_ptr->pilots_,
    dl_ifft_buffer_ptrs, dl_socket_buffer_ptr, 
    FFT_task_duration_ptrs, CSI_task_duration_ptrs, obj_ptr->FFT_task_count, obj_ptr->CSI_task_count,
    IFFT_task_duration_ptrs, obj_ptr->IFFT_task_count);


    int total_count = 0;
    int miss_count = 0;
    Event_data event;
    bool ret = false;

    while(true) {
        ret = fft_queue_->try_dequeue(event);
        if (!ret) {
#if ENABLE_DOWNLINK
            ret = ifft_queue_->try_dequeue(event);
            if (!ret)
                continue;
            else
                computeFFT->IFFT(event.data);
#else
            continue;
#endif
        }
        else
            computeFFT->FFT(event.data);
    }

}



void* CoMP::zfThread(void* context)
{
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* zf_queue_ = &(obj_ptr->zf_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("ZF thread %d starts\n", tid);
    
    // attach task threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    // int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
    int offset_id = SOCKET_RX_THREAD_NUM + CORE_OFFSET + 2;
    int tar_core_id = tid + offset_id;
    if (tar_core_id>=36) 
        tar_core_id = tar_core_id - 36 + 1;
    if(stick_this_thread_to_core(tar_core_id) != 0) {
        printf("ZF thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else {
        printf("ZF thread: stitch thread %d to core %d succeeded\n", tid, tar_core_id);
    }
#endif

    obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->complete_task_queue_));

    int duration_size = TASK_THREAD_NUM * 8;
    double *ZF_task_duration_ptrs[duration_size];


    for (int i = 0; i < duration_size; i++) {
        ZF_task_duration_ptrs[i] = obj_ptr->ZF_task_duration[i];
    }

    int csi_buffer_size = UE_NUM * TASK_BUFFER_FRAME_NUM;
    complex_float *csi_buffer_ptrs[csi_buffer_size];
    for (int i = 0; i < csi_buffer_size; i++)
        csi_buffer_ptrs[i] = obj_ptr->csi_buffer_.CSI[i];

    int precoder_buffer_size = OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM;
    complex_float *precoder_buffer_ptrs[precoder_buffer_size];
    for (int i = 0; i < precoder_buffer_size; i++)
        precoder_buffer_ptrs[i] = obj_ptr->precoder_buffer_.precoder[i];

    int pred_csi_buffer_size = OFDM_DATA_NUM;
    complex_float *pred_csi_buffer_ptrs[pred_csi_buffer_size];
#if DO_PREDICTION
    for(int i = 0; i < pred_csi_buffer_size; i++)
        pred_csi_buffer_ptrs[i] = obj_ptr->pred_csi_buffer_.CSI[i];
#endif

    moodycamel::ProducerToken *task_ptok_ptr;
    task_ptok_ptr = obj_ptr->task_ptok[tid].get();

    DoZF *computeZF = new DoZF(tid, obj_ptr->zf_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        csi_buffer_ptrs, precoder_buffer_ptrs, pred_csi_buffer_ptrs, ZF_task_duration_ptrs, obj_ptr->ZF_task_count);

    int total_count = 0;
    int miss_count = 0;
    Event_data event;
    bool ret_zf = false;

    while(true) {
        ret_zf = zf_queue_->try_dequeue(event);
        if (!ret_zf) 
            continue;
        else
            computeZF->ZF(event.data);
            // obj_ptr->doZF(tid, event.data);
    }

}

void* CoMP::demulThread(void* context)
{
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* demul_queue_ = &(obj_ptr->demul_queue_);
#if ENABLE_DOWNLINK
    moodycamel::ConcurrentQueue<Event_data>* precode_queue_ = &(obj_ptr->precode_queue_);
#endif
    int tid = ((EventHandlerContext *)context)->id;
    printf("Demul thread %d starts\n", tid);
    
    // attach task threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    // int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
    int offset_id = SOCKET_RX_THREAD_NUM + CORE_OFFSET + 2;
    int tar_core_id = tid + offset_id;
    if (tar_core_id>=36) 
        tar_core_id = tar_core_id - 36 + 1;
    if(stick_this_thread_to_core(tar_core_id) != 0) {
        printf("Demul thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else {
        printf("Demul thread: stitch thread %d to core %d succeeded\n", tid, tar_core_id);
    }
#endif

    obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->complete_task_queue_));


    int duration_size = TASK_THREAD_NUM * 8;
    double *Demul_task_duration_ptrs[duration_size];
    double *Precode_task_duration_ptrs[duration_size];


    for (int i = 0; i < duration_size; i++) {
        Demul_task_duration_ptrs[i] = obj_ptr->Demul_task_duration[i];
        Precode_task_duration_ptrs[i] = obj_ptr->Precode_task_duration[i];
    }

    int data_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *data_buffer_ptrs[data_buffer_size];
    for (int i = 0; i < data_buffer_size; i++)
        data_buffer_ptrs[i] = obj_ptr->data_buffer_.data[i];

    int precoder_buffer_size = OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM;
    complex_float *precoder_buffer_ptrs[precoder_buffer_size];
    for (int i = 0; i < precoder_buffer_size; i++)
        precoder_buffer_ptrs[i] = obj_ptr->precoder_buffer_.precoder[i];

    int equal_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *equal_buffer_ptrs[equal_buffer_size];
    uint8_t *demul_hard_buffer_ptrs[equal_buffer_size];
    for (int i = 0; i < equal_buffer_size; i++) {
        equal_buffer_ptrs[i] = obj_ptr->equal_buffer_.data[i];
        demul_hard_buffer_ptrs[i] = obj_ptr->demul_hard_buffer_[i];
    }

    moodycamel::ProducerToken *task_ptok_ptr;
    task_ptok_ptr = obj_ptr->task_ptok[tid].get();

    DoDemul *computeDemul = new DoDemul(tid, obj_ptr->demul_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        data_buffer_ptrs, precoder_buffer_ptrs, equal_buffer_ptrs, demul_hard_buffer_ptrs, Demul_task_duration_ptrs, obj_ptr->Demul_task_count);



    int IFFT_buffer_block_num = BS_ANT_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *dl_ifft_buffer_ptrs[IFFT_buffer_block_num];
#if ENABLE_DOWNLINK
    for (int i = 0; i < IFFT_buffer_block_num; i++) 
        dl_ifft_buffer_ptrs[i] = obj_ptr->dl_ifft_buffer_.IFFT_inputs[i];
#endif


    int dl_precoded_data_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    complex_float *dl_modulated_buffer_ptrs[dl_precoded_data_buffer_size];
    complex_float *dl_precoded_data_buffer_ptrs[dl_precoded_data_buffer_size];
#if ENABLE_DOWNLINK
    for (int i = 0; i < dl_precoded_data_buffer_size; i++) {
        dl_modulated_buffer_ptrs[i] = obj_ptr->dl_modulated_buffer_.data[i];
        dl_precoded_data_buffer_ptrs[i] = obj_ptr->dl_precoded_data_buffer_.data[i];
    }
#endif
    float *qam16_table_ptrs[2];
    for (int i = 0; i < 2; i++) 
        qam16_table_ptrs[i] = obj_ptr->qam16_table[i];

    DoPrecode *computePrecode = new DoPrecode(tid, obj_ptr->demul_block_size, obj_ptr->transpose_block_size, &(obj_ptr->complete_task_queue_), task_ptok_ptr,
        dl_modulated_buffer_ptrs, precoder_buffer_ptrs, dl_precoded_data_buffer_ptrs, dl_ifft_buffer_ptrs, obj_ptr->dl_IQ_data, qam16_table_ptrs, 
        Precode_task_duration_ptrs, obj_ptr->Precode_task_count);


    int total_count = 0;
    int miss_count = 0;
    Event_data event;
    bool ret_demul = false;
    bool ret_precode = false;
    int cur_frame_id = 0;

    while(true) {         
#if ENABLE_DOWNLINK
        ret_precode = precode_queue_->try_dequeue(event);
        if (!ret_precode)
            continue;
        else
            computePrecode->Precode(event.data);
            // obj_ptr->do_precode(tid, event.data);
#else
        ret_demul = demul_queue_->try_dequeue(event);
        if (!ret_demul) {
            continue;
        }
        else {
            // int frame_id = event.data / (OFDM_CA_NUM * data_subframe_num_perframe);
            // // check precoder status for the current frame
            // if (frame_id > cur_frame_id || frame_id == 0) {
            //     while (!precoder_status_[frame_id]);
            // }
            // obj_ptr->doDemul(tid, event.data);
            computeDemul->Demul(event.data);
        }
#endif   
    }

}




void CoMP::getDemulData(int **ptr, int *size)
{
    *ptr = (int *)&equal_buffer_.data[max_equaled_frame*data_subframe_num_perframe][0];
    *size = UE_NUM*FFT_LEN;
}

void CoMP::getEqualData(float **ptr, int *size)
{
    // max_equaled_frame = 0;
    *ptr = (float *)&equal_buffer_.data[max_equaled_frame*data_subframe_num_perframe][0];
    // *ptr = equal_output;
    *size = UE_NUM*FFT_LEN*2;
    
    // printf("In getEqualData()\n");
    // for(int ii = 0; ii < UE_NUM*FFT_LEN; ii++)
    // {
    //     // printf("User %d: %d, ", ii,demul_ptr2(ii));
    //     printf("[%.4f+j%.4f] ", *(*ptr+ii*UE_NUM*2), *(*ptr+ii*UE_NUM*2+1));
    // }
    // printf("\n");
    // printf("\n");
    
}



extern "C"
{
    EXPORT CoMP* CoMP_new() {
        // printf("Size of CoMP: %d\n",sizeof(CoMP *));
        CoMP *comp = new CoMP();
        
        return comp;
    }
    EXPORT void CoMP_start(CoMP *comp) {comp->start();}
    EXPORT void CoMP_getEqualData(CoMP *comp, float **ptr, int *size) {return comp->getEqualData(ptr, size);}
    EXPORT void CoMP_getDemulData(CoMP *comp, int **ptr, int *size) {return comp->getDemulData(ptr, size);}
}





