#include "CoMP.hpp"

using namespace arma;
typedef cx_float COMPLEX;


bool keep_running = true;

void intHandler(int) {
    std::cout << "will exit..." << std::endl;
    keep_running = false;
}


CoMP::CoMP()
{
    csi_format_offset = 1.0/32768;
    // openblas_set_num_threads(1);
    printf("enter constructor\n");
    // initialize socket buffer

    printf("initialize buffers\n");
    socket_buffer_size_ = PackageReceiver::package_length * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM; 
    socket_buffer_status_size_ = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++) {
        socket_buffer_[i].buffer = (char *)aligned_alloc(64, socket_buffer_size_ * sizeof(char));
        socket_buffer_[i].buffer_status = (int *)aligned_alloc(64, socket_buffer_status_size_ * sizeof(int));
    }

    

    // initialize FFT buffer
    // int FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    // fft_buffer_.FFT_inputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *));//new complex_float * [FFT_buffer_block_num];
    // fft_buffer_.FFT_outputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *));//new complex_float * [FFT_buffer_block_num];
    // for (int i = 0; i < FFT_buffer_block_num; i++) {
    //     fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    //     fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    // }

    int FFT_buffer_block_num = subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    fft_buffer_.FFT_inputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *));
    fft_buffer_.FFT_outputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *));
    for (int i = 0; i < FFT_buffer_block_num; i++) {
        fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(BS_ANT_NUM * OFDM_CA_NUM * sizeof(complex_float));
        fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(BS_ANT_NUM * OFDM_CA_NUM * sizeof(complex_float));
    }

    // initialize muplans for fft
    for (int i = 0; i < TASK_THREAD_NUM; i++) 
        muplans_[i] = mufft_create_plan_1d_c2c(OFDM_CA_NUM, MUFFT_FORWARD, MUFFT_FLAG_CPU_ANY);

    // initialize CSI buffer
    // csi_buffer_.CSI.resize(OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM);
    // for (int i = 0; i < csi_buffer_.CSI.size(); i++)
    //     csi_buffer_.CSI[i].resize(BS_ANT_NUM * UE_NUM);
    // printf("CSI buffer initialized\n");

    int csi_buffer_size = UE_NUM * TASK_BUFFER_FRAME_NUM;
    csi_buffer_.CSI = (complex_float **)malloc(csi_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < csi_buffer_size; i++)
        csi_buffer_.CSI[i] = (complex_float *)mufft_alloc(BS_ANT_NUM * OFDM_DATA_NUM * sizeof(complex_float));


    int data_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    data_buffer_.data = (complex_float **)malloc(data_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < data_buffer_size; i++)
        data_buffer_.data[i] = (complex_float *)mufft_alloc(BS_ANT_NUM * OFDM_DATA_NUM * sizeof(complex_float));


    if (DO_PREDICTION) {
        int pred_csi_buffer_size = OFDM_DATA_NUM;
        pred_csi_buffer_.CSI = (complex_float **)malloc(pred_csi_buffer_size * sizeof(complex_float *));
        
        for(int i = 0; i < pred_csi_buffer_size; i++)
            pred_csi_buffer_.CSI[i] = (complex_float *)mufft_alloc(BS_ANT_NUM * UE_NUM * sizeof(complex_float));
    }


    // initialize precoder buffer
    int precoder_buffer_size = OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM;
    precoder_buffer_.precoder = (complex_float **)malloc(precoder_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < precoder_buffer_size; i++)
        precoder_buffer_.precoder[i] = (complex_float *)aligned_alloc(64, UE_NUM * BS_ANT_NUM * sizeof(complex_float));

    // initialize equalized data buffer
    int equal_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    equal_buffer_.data = (complex_float **)malloc(equal_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < equal_buffer_size; i++)
        equal_buffer_.data[i] = (complex_float *)aligned_alloc(64, OFDM_DATA_NUM * UE_NUM * sizeof(complex_float));



    // initialize demultiplexed data buffer
    int demul_buffer_size = data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    demul_buffer_.data = (long long **)malloc(equal_buffer_size * sizeof(long long *));
    for (int i = 0; i < demul_buffer_size; i++)
        demul_buffer_.data[i] = (long long *)aligned_alloc(64, OFDM_DATA_NUM * UE_NUM * sizeof(long long));


    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        spm_buffer[i] = (complex_float *)aligned_alloc(64, 8 * BS_ANT_NUM * sizeof(complex_float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        csi_gather_buffer[i] = (complex_float *)aligned_alloc(64, BS_ANT_NUM * UE_NUM * sizeof(complex_float));
    // printf("Demultiplexed data buffer initialized\n");

    // read pilots from file
    pilots_.resize(OFDM_CA_NUM);
    FILE* fp = fopen("../pilot_f_2048.bin","rb");
    fread(pilots_.data(), sizeof(float), OFDM_CA_NUM, fp);
    fclose(fp);

    pilots_complex_.resize(OFDM_CA_NUM);
    for (int i = 0; i<OFDM_CA_NUM;i++) {
        pilots_complex_[i].real = pilots_[i];
        pilots_complex_[i].imag = 0;
    }

#ifdef DEBUG_PRINT_PILOT
    cout<<"Pilot data"<<endl;
    for (int i = 0; i<OFDM_CA_NUM;i++) 
        cout<<pilots_[i]<<",";
    cout<<endl;
#endif

    printf("new PackageReceiver\n");
    receiver_.reset(new PackageReceiver(SOCKET_RX_THREAD_NUM, &message_queue_));

    // initilize all kinds of checkers
    memset(fft_counter_ants_, 0, sizeof(int) * subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    memset(csi_counter_users_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(data_counter_subframes_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_counter_scs_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_exist_in_frame_, 0, sizeof(bool) * TASK_BUFFER_FRAME_NUM); 


    memset(demul_counter_subframes_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(fft_created_counter_packets_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(rx_counter_packets_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);

    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
        memset(demul_counter_scs_[i], 0, sizeof(int) * (subframe_num_perframe - UE_NUM));

    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
        memset(data_exist_in_subframe_[i], 0, sizeof(bool) * (subframe_num_perframe - UE_NUM));

    // create task thread 
    for(int i = 0; i < TASK_THREAD_NUM; i++) {
        context[i].obj_ptr = this;
        context[i].id = i;
        //printf("create thread %d\n", i);
        if(pthread_create( &task_threads[i], NULL, CoMP::taskThread, &context[i]) != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }

    memset(FFT_task_count, 0, sizeof(int) * TASK_THREAD_NUM);
    memset(ZF_task_count, 0, sizeof(int) * TASK_THREAD_NUM);
    memset(Demul_task_count, 0, sizeof(int) * TASK_THREAD_NUM);

    memset(FFT_task_duration, 0, sizeof(double) * TASK_THREAD_NUM);
    memset(ZF_task_duration, 0, sizeof(double) * TASK_THREAD_NUM);
    memset(Demul_task_duration, 0, sizeof(double) * TASK_THREAD_NUM);

    memset(FFT_task_duration_part1, 0, sizeof(double) * TASK_THREAD_NUM);
    memset(FFT_task_duration_part2, 0, sizeof(double) * TASK_THREAD_NUM);
    memset(FFT_task_duration_part3, 0, sizeof(double) * TASK_THREAD_NUM);



#if ENABLE_DOWNLINK
    dl_IQ_data = new int * [data_subframe_num_perframe * UE_NUM];
    dl_IQ_data_long = new long long * [data_subframe_num_perframe * UE_NUM];
    for (int i = 0; i < data_subframe_num_perframe * UE_NUM; i++) {
        dl_IQ_data[i] = new int[OFDM_CA_NUM];
        dl_IQ_data_long[i] = new long long[packageSenderBS::OFDM_FRAME_LEN];
    }
    // read data from file
    fp = fopen("../orig_data.bin","rb");
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
    dl_ifft_buffer_.IFFT_inputs = new complex_float*[IFFT_buffer_block_num];
    dl_ifft_buffer_.IFFT_outputs = new complex_float*[IFFT_buffer_block_num];
    for (int i = 0; i < IFFT_buffer_block_num; i++) {
        dl_ifft_buffer_.IFFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
        dl_ifft_buffer_.IFFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    }


    // initialize muplans for ifft
    for (int i = 0; i < TASK_THREAD_NUM; i++)
        muplans_ifft_[i] = mufft_create_plan_1d_c2c(OFDM_CA_NUM, MUFFT_INVERSE, MUFFT_FLAG_CPU_ANY);

    // // initialize downlink iffted data buffer
    // dl_iffted_data_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    // for (int i = 0; i < dl_iffted_data_buffer_.data.size(); i++)
    //     dl_iffted_data_buffer_.data[i].resize(UE_NUM * OFDM_CA_NUM);

    // initialize downlink precoded data buffer
    dl_precoded_data_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for (int i = 0; i < dl_precoded_data_buffer_.data.size(); i++)
        dl_precoded_data_buffer_.data[i].resize(BS_ANT_NUM * OFDM_CA_NUM);
    
    // initialize downlink modulated data buffer
    dl_modulated_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for (int i = 0; i < dl_modulated_buffer_.data.size(); i++)
        dl_modulated_buffer_.data[i].resize(OFDM_CA_NUM * UE_NUM);

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        dl_spm_buffer[i].resize(UE_NUM);

    // initialize downlink socket buffer
    dl_socket_buffer_.buffer.resize(data_subframe_num_perframe * SOCKET_BUFFER_FRAME_NUM 
                    * PackageReceiver::package_length * BS_ANT_NUM); // buffer SOCKET_BUFFER_FRAME_NUM entire frame
    printf("socket_buffer_ size: %d, data_subframe_num_perframe %d, SOCKET_BUFFER_FRAME_NUM %d, package_length %d, BS_ANT_NUM %d\n", dl_socket_buffer_.buffer.size(),
        data_subframe_num_perframe, SOCKET_BUFFER_FRAME_NUM, PackageReceiver::package_length, BS_ANT_NUM);
    dl_socket_buffer_.buffer_status.resize(data_subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM);


    memset(precode_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(modulate_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 

    for (int i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
        memset(precode_checker_[i], 0, sizeof(int) * data_subframe_num_perframe);
        memset(modulate_checker_[i], 0, sizeof(int) * data_subframe_num_perframe);
    }

    memset(ifft_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(tx_status_, 0, sizeof(int) * SOCKET_BUFFER_FRAME_NUM); 
    memset(tx_checker_, 0, sizeof(int) * SOCKET_BUFFER_FRAME_NUM); 

    for (int i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) 
        memset(tx_checker_[i], 0, sizeof(int) * data_subframe_num_perframe);

    printf("initialize QAM16 table\n");
    float scale = 1/sqrt(10);
    float modvec_16qam[4] = {-3*scale, -1*scale, 3*scale, scale};
    for (int i = 0; i < 16; i++) {
        qam16_table[0][i] = modvec_16qam[i / 4];
        qam16_table[1][i] = modvec_16qam[i % 4];
    }

    printf("new PackageSender\n");
    transmitter_.reset(new packageSenderBS(SOCKET_TX_THREAD_NUM, &message_queue_, &tx_queue_));

#endif

}

CoMP::~CoMP()
{
    for(int i = 0; i < TASK_THREAD_NUM; i++) {
        mufft_free_plan_1d(muplans_[i]);
        mufft_free_plan_1d(muplans_ifft_[i]);
    }
    // release FFT_buffer
    int FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    for(int i = 0; i < FFT_buffer_block_num; i++) {
        mufft_free(fft_buffer_.FFT_inputs[i]);
        mufft_free(fft_buffer_.FFT_outputs[i]);
        mufft_free(dl_ifft_buffer_.IFFT_inputs[i]);
        mufft_free(dl_ifft_buffer_.IFFT_outputs[i]);
    }

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

static double get_time(void)
{
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return tv.tv_sec * 1000000 + tv.tv_nsec / 1000.0;
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
    for(int i = 0; i < SOCKET_RX_THREAD_NUM; i++) {
        socket_buffer_ptrs[i] = socket_buffer_[i].buffer;
        socket_buffer_status_ptrs[i] = socket_buffer_[i].buffer_status;
    }
    std::vector<pthread_t> rx_threads = receiver_->startRecv(socket_buffer_ptrs, 
        socket_buffer_status_ptrs, socket_buffer_status_size_, socket_buffer_size_, main_core_id + 1);

    // start downlink transmitter
#if ENABLE_DOWNLINK
    char *dl_socket_buffer_ptr = dl_socket_buffer_.buffer.data();
    int *dl_socket_buffer_status_ptr = dl_socket_buffer_.buffer_status.data();
    float *dl_data_ptr = (float *)(&dl_precoded_data_buffer_.data[0][0]);
    std::vector<pthread_t> tx_threads = transmitter_->startTX(dl_socket_buffer_ptr, 
        dl_socket_buffer_status_ptr, dl_data_ptr, socket_buffer_[0].buffer_status.size(), socket_buffer_[0].buffer.size(), 1+SOCKET_RX_THREAD_NUM);
#endif

    // for task_queue, main thread is producer, it is single-procuder & multiple consumer
    // for task queue
    // uplink

    // TODO: make the producertokens global and try "try_dequeue_from_producer(token,item)"
    //       combine the task queues into one queue
    moodycamel::ProducerToken ptok(task_queue_);
    moodycamel::ProducerToken ptok_zf(zf_queue_);
    moodycamel::ProducerToken ptok_demul(demul_queue_);
    // downlink
    moodycamel::ProducerToken ptok_ifft(ifft_queue_);
    moodycamel::ProducerToken ptok_modul(modulate_queue_);
    moodycamel::ProducerToken ptok_precode(precode_queue_);
    moodycamel::ProducerToken ptok_tx(tx_queue_);
    // for message_queue, main thread is a comsumer, it is multiple producers
    // & single consumer for message_queue
    moodycamel::ConsumerToken ctok(message_queue_);
    moodycamel::ConsumerToken ctok_complete(complete_task_queue_);

    int delay_fft_queue[TASK_BUFFER_FRAME_NUM][subframe_num_perframe * BS_ANT_NUM];
    int delay_fft_queue_cnt[TASK_BUFFER_FRAME_NUM];

    for (int i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
        memset(delay_fft_queue[i], 0, sizeof(int) * subframe_num_perframe * BS_ANT_NUM);
    }

    memset(delay_fft_queue_cnt, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);


    // counter for print log
    int demul_count = 0;
    auto demul_begin = std::chrono::system_clock::now();
    auto tx_begin = std::chrono::high_resolution_clock::now();
    auto ifft_begin = std::chrono::high_resolution_clock::now();
    auto zf_begin = std::chrono::high_resolution_clock::now();
    int miss_count = 0;
    int total_count = 0;
    int frame_count = 0;
    int tx_count = 0;
    int zf_count = 0;
    // auto pilot_received = std::chrono::system_clock::now();
    // std::chrono::time_point<std::chrono::high_resolution_clock> pilot_received[TASK_BUFFER_FRAME_NUM];
    double pilot_received[TASK_BUFFER_FRAME_NUM];
    double fft_processed[TASK_BUFFER_FRAME_NUM];
    double demul_processed[TASK_BUFFER_FRAME_NUM];
    double zf_processed[TASK_BUFFER_FRAME_NUM];
    double total_time = 0;
    int ifft_frame_count = 0;


    Event_data events_list[dequeue_bulk_size];
    int ret = 0;
    bool rx_start = false;
    bool prev_demul_scheduled = false;
    double fft_previous_time = get_time();
    double fft_time_sum = 0;
    double zf_time_sum = 0;
    double demul_time_sum = 0;

    double fft_time_sum_part1 = 0;
    double fft_time_sum_part2 = 0;
    double fft_time_sum_part3 = 0;



    signal(SIGINT, intHandler);
    while(keep_running) {
        // get a bulk of events
        ret = complete_task_queue_.try_dequeue_bulk(ctok_complete, events_list, dequeue_bulk_size);
        if (ret == 0)
            ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size);
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
        // handle each event
        for(int bulk_count = 0; bulk_count < ret; bulk_count ++) {
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
                    }

                    // char *socket_buffer_ptr = socket_buffer_ptrs[socket_thread_id] + offset_in_current_buffer * PackageReceiver::package_length;
                    char *socket_buffer_ptr = socket_buffer_[socket_thread_id].buffer + offset_in_current_buffer * PackageReceiver::package_length;
                    int subframe_id = *((int *)socket_buffer_ptr + 1);                                    
                    int frame_id = *((int *)socket_buffer_ptr);
                    int ant_id = *((int *)socket_buffer_ptr + 3);
                    int rx_frame_id = (frame_id % TASK_BUFFER_FRAME_NUM);

                    rx_counter_packets_[rx_frame_id]++;
                    if (rx_counter_packets_[rx_frame_id] == 1) {   
#if DEBUG_PRINT_PER_FRAME_START 
                        printf("Main thread: data received from frame %d, subframe %d, ant %d, offset %d\n", frame_id, subframe_id, ant_id, offset);
#endif
                        pilot_received[rx_frame_id] = get_time();                           
                    }
                    else if (rx_counter_packets_[rx_frame_id] == BS_ANT_NUM * subframe_num_perframe) {  
#if DEBUG_PRINT_PER_FRAME_DONE 
                        printf("Main thread: received data for all packets in frame: %d, frame buffer: %d in %.5f us\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, get_time()-pilot_received[rx_frame_id]);
#endif                        
                        rx_counter_packets_[rx_frame_id] = 0;                                  
                    }  

                    // if this is the first frame or the previous frame is all processed, schedule FFT for this packet
                    int frame_id_prev = frame_id == 0 ? (TASK_BUFFER_FRAME_NUM-1) : (frame_id - 1) % TASK_BUFFER_FRAME_NUM;
                    if ((frame_id == 0 && frame_count < 100) || (frame_count > 0 && demul_counter_subframes_[frame_id_prev] == data_subframe_num_perframe)) {
                        Event_data do_crop_task;
                        do_crop_task.event_type = TASK_CROP;
                        do_crop_task.data = offset;
                        schedule_task(do_crop_task, &task_queue_, ptok);
                        fft_created_counter_packets_[rx_frame_id]++;
#if DEBUG_PRINT_PER_TASK_ENTER_QUEUE
                        printf("Main thread: created FFT tasks for frame: %d, frame buffer: %d, subframe: %d, ant: %d\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
#endif                        
                        if (fft_created_counter_packets_[rx_frame_id] == BS_ANT_NUM * subframe_num_perframe) {
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE                            
                            printf("Main thread: created FFT tasks for all packets in frame: %d, frame buffer: %d in %.5f us\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, get_time()-pilot_received[rx_frame_id]);
#endif                            
                            fft_created_counter_packets_[rx_frame_id] = 0;
                            if(frame_id > 0) 
                                demul_counter_subframes_[(frame_id-1)%TASK_BUFFER_FRAME_NUM] = 0;
                            else
                                demul_counter_subframes_[TASK_BUFFER_FRAME_NUM-1] = 0;
                        }
                        
                    }
                    else {
                        // if the previous frame is not finished, store offset in queue
                        // printf("previous frame not finished, frame_id %d, previous frame finished: %d, subframe_id %d, ant_id %d, delay_fft_queue_cnt %d\n", frame_id, demul_counter_subframes_[(frame_id-1)%TASK_BUFFER_FRAME_NUM], 
                        //      subframe_id, ant_id, delay_fft_queue_cnt[rx_frame_id]);
                        delay_fft_queue[rx_frame_id][delay_fft_queue_cnt[rx_frame_id]] = offset;
                        delay_fft_queue_cnt[rx_frame_id]++;
                    }
                                            
                }                
                break;
                
            
            case EVENT_CROPPED: {
                    int offset_fft = event.data;
                    int frame_id, subframe_id;
                    interpreteOffset2d(subframe_num_perframe, offset_fft, &frame_id, &subframe_id);
                    fft_counter_ants_[offset_fft] ++;

                    // if FFT for all anetnnas in a subframe is done, schedule ZF or equalization+demodulation
                    if (fft_counter_ants_[offset_fft] == BS_ANT_NUM) {
                        fft_counter_ants_[offset_fft] = 0;
                        if (isPilot(subframe_id)) {   
#if DEBUG_PRINT_PER_SUBFRAME_DONE
                            printf("Main thread: pilot FFT done for frame: %d, subframe: %d\n", frame_id,subframe_id);
#endif
                            csi_counter_users_[frame_id] ++;
                            // if csi of all UEs is ready, schedule ZF or prediction 
                            if (csi_counter_users_[frame_id] == UE_NUM) {
                                fft_processed[frame_id] = get_time();   
                                csi_counter_users_[frame_id] = 0; 
#if DEBUG_PRINT_PER_FRAME_DONE
                                // printf("Main thread: pilot frame: %d, finished collecting pilot frames\n", frame_id);
                                printf("Main thread: pilot frame: %d, finished FFT for all pilot subframes in %.5f us\n", frame_id, fft_processed[frame_id]-pilot_received[frame_id]);
#endif
                                // count # of frame with CSI estimation
                                frame_count ++;

                                // schedule ZF when traning data is not enough or prediction is not enabled
                                if (frame_count<=INIT_FRAME_NUM || !DO_PREDICTION) {  
                                    // schedule normal ZF for all data subcarriers                                                                                       
                                    Event_data do_zf_task;
                                    do_zf_task.event_type = TASK_ZF;
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
                                    printf("Main thread: created ZF tasks for frame: %d\n", frame_id);
#endif
                                    for (int i = 0; i < OFDM_DATA_NUM; i++) {
                                        do_zf_task.data = generateOffset2d(OFDM_DATA_NUM, frame_id, i);
                                        schedule_task(do_zf_task, &zf_queue_, ptok_zf);
                                    }
                                }
                                // schedule prediction when traning data is enough and prediction is enabled
                                // when frame count equals to INIT_FRAME_NUM, do both prediction and ZF
                                if (frame_count>=INIT_FRAME_NUM && DO_PREDICTION) {                              
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
                                if (frame_count==1e9) 
                                    frame_count=0;
                            }
                        }
                        else if (isData(subframe_id)) {
#if DEBUG_PRINT_PER_SUBFRAME_ENTER_QUEUE                            
                            printf("Main thread: finished FFT for frame %d, subframe %d, precoder status: %d, fft queue: %d, zf queue: %d, demul queue: %d\n", 
                                    frame_id, subframe_id, 
                                    precoder_exist_in_frame_[frame_id], task_queue_.size_approx(), zf_queue_.size_approx(), demul_queue_.size_approx());
                            fft_previous_time = get_time();
#endif                            
                            data_exist_in_subframe_[frame_id][getULSFIndex(subframe_id)] = true;
                            // if precoder exist, schedule demodulation
                            if (precoder_exist_in_frame_[frame_id]) {
                                int start_sche_id = subframe_id;
                                if (!prev_demul_scheduled) {
                                    start_sche_id = UE_NUM;
                                    prev_demul_scheduled = true;
                                }
                                for (int sche_subframe_id = start_sche_id; sche_subframe_id <= data_counter_subframes_[frame_id] + UE_NUM; sche_subframe_id++) {
                                    int data_subframe_id = getULSFIndex(sche_subframe_id);
                                    if (data_exist_in_subframe_[frame_id][data_subframe_id]) {
                                        Event_data do_demul_task;
                                        do_demul_task.event_type = TASK_DEMUL;

                                        // schedule demodulation task for subcarrier blocks
                                        for(int i = 0; i < OFDM_DATA_NUM / demul_block_size; i++) {
                                            do_demul_task.data = generateOffset3d(OFDM_DATA_NUM, frame_id, data_subframe_id, i * demul_block_size);
                                            schedule_task(do_demul_task, &demul_queue_, ptok_demul);
                                        }
#if DEBUG_PRINT_PER_SUBFRAME_ENTER_QUEUE
                                            printf("Main thread: created Demodulation task for frame: %d, start sc: %d, subframe: %d\n", frame_id, start_sche_id, sche_subframe_id);
#endif                                         
                                        // clear data status after scheduling
                                        data_exist_in_subframe_[frame_id][data_subframe_id] = false;
                                    }
                                }
                            }

                            data_counter_subframes_[frame_id] ++;
                            if (data_counter_subframes_[frame_id] == data_subframe_num_perframe) {                           
#if DEBUG_PRINT_PER_FRAME_DONE
                                    printf("Main thread: data frame: %d, finished FFT for all data subframes in %.5f us\n", frame_id, get_time()-pilot_received[frame_id]);
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
                    // printf("Main thread: ZF done frame: %d, subcarrier %d\n", frame_id, sc_id);
                    if (precoder_counter_scs_[frame_id] == OFDM_DATA_NUM) {
                        zf_processed[frame_id] = get_time(); 
                        // if all the data in a frame has arrived when ZF is done
                        if (data_counter_subframes_[frame_id] == data_subframe_num_perframe) {  
                            for (int sche_subframe_id = UE_NUM; sche_subframe_id < subframe_num_perframe; sche_subframe_id++) {
                                int data_subframe_id = getULSFIndex(sche_subframe_id);
                                if (data_exist_in_subframe_[frame_id][data_subframe_id]) {
                                    Event_data do_demul_task;
                                    do_demul_task.event_type = TASK_DEMUL;

                                    // schedule demodulation task for subcarrier blocks
                                    for(int i = 0; i < OFDM_DATA_NUM / demul_block_size; i++) {
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
                        Event_data do_modul_task;
                        do_modul_task.event_type = TASK_MODUL;

                        for (int i = 0; i < data_subframe_num_perframe; i++) {
                            for (int j = 0; j < UE_NUM; j++) {
                                do_modul_task.data = generateOffset3d(UE_NUM, frame_id, i, j);
                                schedule_task(do_modul_task, &modulate_queue_, ptok_modul);
                            }
                        }
#endif
#if DEBUG_PRINT_PER_FRAME_DONE                        
                        printf("Main thread: ZF done frame: %d in %.5f us, total: %.5f us\n", frame_id, zf_processed[frame_id]-fft_processed[frame_id],
                            zf_processed[frame_id]-pilot_received[frame_id]);
#endif                    
                        precoder_counter_scs_[frame_id] = 0;
                        precoder_exist_in_frame_[frame_id] = true;

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

                    demul_counter_scs_[frame_id][data_subframe_id] += demul_block_size;
#if DEBUG_PRINT_PER_TASK_DONE
                    printf("Main thread: Demodulation done frame: %d, subframe: %d, subcarrier: %d\n", frame_id, data_subframe_id,  demul_counter_scs_[frame_id][data_subframe_id]);
#endif
                    // if this subframe is ready
                    if (demul_counter_scs_[frame_id][data_subframe_id] == OFDM_DATA_NUM) {
#if DEBUG_PRINT_PER_SUBFRAME_DONE
                        printf("Main thread: Demodulation done frame: %d, subframe: %d\n", frame_id, data_subframe_id);
#endif
                        max_equaled_frame = frame_id;
                        demul_counter_scs_[frame_id][data_subframe_id] = 0;
                        demul_counter_subframes_[frame_id]++;
                        if (demul_counter_subframes_[frame_id] == data_subframe_num_perframe) {
                            demul_processed[frame_id] = get_time(); 
                            precoder_exist_in_frame_[frame_id] = false;
                            data_counter_subframes_[frame_id] = 0;

                            // schedule fft for next frame
                            int frame_id_next = (frame_id + 1) % TASK_BUFFER_FRAME_NUM;
                            if (delay_fft_queue_cnt[frame_id_next] > 0) {
#if DEBUG_PRINT_PER_FRAME_ENTER_QUEUE
                                printf("Main thread in demul: schedule fft for %d packets for frame %d is done\n", delay_fft_queue_cnt[frame_id_next], frame_id_next);
#endif                                
                                Event_data do_crop_task;
                                do_crop_task.event_type = TASK_CROP;
                                for (int i = 0; i < delay_fft_queue_cnt[frame_id_next]; i++) {
                                    int offset_rx = delay_fft_queue[frame_id_next][i];
                                    do_crop_task.data = offset_rx;
                                    schedule_task(do_crop_task, &task_queue_, ptok);
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

#if DEBUG_PRINT_PER_FRAME_DONE
                            printf("Main thread: Demodulation done frame: %d, demul_count %d, subframe %d in %.5f us, total %.5f us\n", frame_id, demul_count + 1, data_subframe_id, 
                                demul_processed[frame_id]-zf_processed[frame_id], demul_processed[frame_id]-pilot_received[frame_id]);
                            double sum_ZF_cur = 0;
                            double sum_demul_cur = 0;
                            double sum_FFT_cur = 0;

                            double sum_FFT_cur_part1 = 0;
                            double sum_FFT_cur_part2 = 0;
                            double sum_FFT_cur_part3 = 0;

                            for (int i = 0; i < TASK_THREAD_NUM; i++) {
                                sum_FFT_cur = sum_FFT_cur + FFT_task_duration[i];
                                sum_ZF_cur = sum_ZF_cur + ZF_task_duration[i];
                                sum_demul_cur = sum_demul_cur + Demul_task_duration[i];

                                sum_FFT_cur_part1 = sum_FFT_cur_part1 + FFT_task_duration_part1[i];
                                sum_FFT_cur_part2 = sum_FFT_cur_part2 + FFT_task_duration_part2[i];
                                sum_FFT_cur_part3 = sum_FFT_cur_part3 + FFT_task_duration_part3[i];

                            }
                            double fft_duration = (sum_FFT_cur-fft_time_sum)/TASK_THREAD_NUM;
                            double zf_duration = (sum_ZF_cur-zf_time_sum)/TASK_THREAD_NUM;
                            double demul_duration = (sum_demul_cur-demul_time_sum)/TASK_THREAD_NUM;
                            double sum_duration = fft_duration + zf_duration + demul_duration;

                            double fft_duration_part1 = (sum_FFT_cur_part1-fft_time_sum_part1)/TASK_THREAD_NUM;
                            double fft_duration_part2 = (sum_FFT_cur_part2-fft_time_sum_part2)/TASK_THREAD_NUM;
                            double fft_duration_part3 = (sum_FFT_cur_part3-fft_time_sum_part3)/TASK_THREAD_NUM;

                            printf("In frame %d, \t\t\t\t\t fft duration: %.5f, (%.5f, %.5f, %.5f ) zf: %.5f, demul: %.5f, sum: %.5f\n", 
                                    frame_id, fft_duration, fft_duration_part1, fft_duration_part2, fft_duration_part3, zf_duration, demul_duration, sum_duration);
                            fft_time_sum = sum_FFT_cur;
                            zf_time_sum = sum_ZF_cur;
                            demul_time_sum = sum_demul_cur;

                            fft_time_sum_part1 = sum_FFT_cur_part1;
                            fft_time_sum_part2 = sum_FFT_cur_part2;
                            fft_time_sum_part3 = sum_FFT_cur_part3;
                            
#endif
                        }

#if WRITE_DEMUL
                        FILE* fp = fopen("demul_data.txt","a");
                        for (int cc = 0; cc < OFDM_DATA_NUM; cc++)
                        {
                            long long* cx = &demul_buffer_.data[total_data_subframe_id][cc * UE_NUM];
                            fprintf(fp, "SC: %d, Frame %d, subframe: %d, ", cc, frame_id, data_subframe_id);
                            for(int kk = 0; kk < UE_NUM; kk++)  
                                fprintf(fp, "%d ", cx[kk]);
                            fprintf(fp, "\n");
                        }
                        fclose(fp);
#endif
                        
                        demul_count += 1;
                        
                        // print log per 100 frames
                        if (demul_count == data_subframe_num_perframe * 100)
                        {
                            demul_count = 0;
                            auto demul_end = std::chrono::system_clock::now();
                            std::chrono::duration<double> diff = demul_end - demul_begin;
                            int samples_num_per_UE = OFDM_DATA_NUM * data_subframe_num_perframe * 100;
                            printf("Receive %d samples (per-client) from %d clients in %f secs, throughtput %f bps per-client (16QAM), current task queue length %d\n", 
                                samples_num_per_UE, UE_NUM, diff.count(), samples_num_per_UE * log2(16.0f) / diff.count(), task_queue_.size_approx());
#if DEBUG_PRINT_SUMMARY_100_FRAMES
                            printf("frame %d: rx: %.5f, fft: %.5f, zf: %.5f, demul: %.5f, total: %.5f us\n", 0,  
                                    pilot_received[0], fft_processed[0]-pilot_received[0], 
                                    zf_processed[0]-fft_processed[0], demul_processed[0]-zf_processed[0], demul_processed[0]-pilot_received[0]);
                            for (int i = 1; i < TASK_BUFFER_FRAME_NUM; i++) {

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

            case EVENT_MODUL: {
                    // Modulation is done, schedule precoding when data for all users is ready
                    int offset_modul = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, user_id;
                    interpreteOffset3d(UE_NUM, offset_modul, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &user_id);

                    modulate_checker_[frame_id][current_data_subframe_id] += 1;
                    // if all users of this subframe is ready
                    if (modulate_checker_[frame_id][current_data_subframe_id] == UE_NUM) {
                        modulate_checker_[frame_id][current_data_subframe_id] = 0;
                        Event_data do_precode_task;
                        do_precode_task.event_type = TASK_PRECODE;
                        // add precode tasks for each subcarrier block
                        for (int i = 0; i < OFDM_DATA_NUM / demul_block_size; i++) {
                            do_precode_task.data = generateOffset3d(OFDM_DATA_NUM, frame_id, current_data_subframe_id, i * demul_block_size);
                            schedule_task(do_precode_task, &precode_queue_, ptok_precode);
                        }
                    } 
                }
                break; 
            case EVENT_PRECODE: {
                    // Precoding is done, schedule ifft 
                    int offset_precode = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
                    interpreteOffset3d(OFDM_DATA_NUM, offset_precode, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);
                    precode_checker_[frame_id][current_data_subframe_id] += demul_block_size;
                    Event_data do_ifft_task;
                    do_ifft_task.event_type = TASK_IFFT;

                    if (precode_checker_[frame_id][current_data_subframe_id] == OFDM_DATA_NUM) {
                        // add ifft task for each antenna
                        precode_checker_[frame_id][current_data_subframe_id] = 0;
                        for (int i = 0; i < BS_ANT_NUM; i++) {
                            do_ifft_task.data = generateOffset3d(BS_ANT_NUM, frame_id, current_data_subframe_id, i);
                            schedule_task(do_ifft_task, &tx_queue_, ptok_ifft);
                        }
                    }

                }
                break;
            case EVENT_IFFT: {
                    // IFFT is done, schedule data transmission 
                    int offset_ifft = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, ant_id;
                    interpreteOffset3d(BS_ANT_NUM, offset_ifft, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &ant_id);
                    Event_data do_tx_task;
                    do_tx_task.event_type = TASK_SEND;
                    do_tx_task.data = offset_ifft;                    
                    schedule_task(do_tx_task, &tx_queue_, ptok_tx);
                    ifft_checker_[frame_id] += 1;
                    if (ifft_checker_[frame_id] == BS_ANT_NUM * data_subframe_num_perframe) {
                        ifft_checker_[frame_id] = 0;
                        // printf("Finished IFFT for frame %d\n", frame_id);
                        ifft_frame_count++;
                        if (ifft_frame_count == 100) {   
                            auto ifft_end = std::chrono::high_resolution_clock::now();
                            std::chrono::duration<double> diff = ifft_end - ifft_begin;         
                            printf("Main thread: finished IFFT for 100 frames in %f secs\n", diff.count());
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

                    tx_checker_[frame_id][current_data_subframe_id] += 1;

                    if (tx_checker_[frame_id][current_data_subframe_id] == BS_ANT_NUM) {
                        tx_status_[frame_id] += 1;
                        tx_checker_[frame_id][current_data_subframe_id] = 0;
                        tx_count ++;
                        // print log per 100 frames
                        if (tx_count == data_subframe_num_perframe * 100)
                        {
                            tx_count = 0;
                            auto tx_end = std::chrono::high_resolution_clock::now();
                            std::chrono::duration<double> diff = tx_end - tx_begin;
                            int samples_num_per_UE = OFDM_CA_NUM * data_subframe_num_perframe * 100;
                            printf("Transmit %d samples (per-client) to %d clients in %f secs, throughtput %f bps per-client (16QAM), current tx queue length %d\n", 
                                samples_num_per_UE, UE_NUM, diff.count(), samples_num_per_UE * log2(16.0f) / diff.count(), tx_queue_.size_approx());
                            tx_begin = std::chrono::high_resolution_clock::now();
                        }
#if DEBUG_PRINT_SUMMARY || DEBUG_PRINT_TASK_DONE                        
                        if (DEBUG_PRINT_TASK_DONE)
                            printf("Main thread: tx done frame: %d, subframe: %d\n", frame_id, current_data_subframe_id);
                        if (tx_status_[frame_id] == data_subframe_num_perframe) {
                            tx_status_[frame_id] = 0;
                            if (DEBUG_PRINT_SUMMARY) {
                                printf("Main thread: tx done frame: %d, queue length: zf %d, ifft %d, precode %d, tx %d\n", frame_id,
                                    zf_queue_.size_approx(), ifft_queue_.size_approx(), precode_queue_.size_approx(), tx_queue_.size_approx());
                            }
                        }
#endif        
                    }                    
                }
                break;
            default:
                printf("Wrong event type in message queue!");
                exit(0);
            }
        }
    }
    // }
    int sum_FFT = 0;
    int sum_ZF = 0;
    int sum_demul = 0;
    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        sum_FFT = sum_FFT + FFT_task_count[i];
        sum_ZF = sum_ZF + ZF_task_count[i];
        sum_demul = sum_demul + Demul_task_count[i];
    }
    printf("Total dequeue trials: %d, missed %d\n", total_count, miss_count);
    printf("BS_ANT_NUM: %d, subframe_num_perframe: %d, OFDM_DATA_NUM: %d, demul_block_size: %d, data_subframe_num_perframe: %d\n", 
            BS_ANT_NUM, subframe_num_perframe, OFDM_DATA_NUM, demul_block_size, data_subframe_num_perframe);
    double fft_frames = (double)sum_FFT/BS_ANT_NUM/subframe_num_perframe;
    double zf_frames = (double)sum_ZF/OFDM_DATA_NUM;
    double demul_frames = (double)sum_demul * demul_block_size/ OFDM_DATA_NUM / data_subframe_num_perframe;
    printf("Total performed FFT: %d (%.2f frames), ZF: %d (%.2f frames), Demulation: %d (%.2f frames)\n", 
        sum_FFT, sum_ZF, sum_demul, fft_frames, zf_frames, demul_frames);
    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        double percent_FFT = 100*double(FFT_task_count[i])/sum_FFT;
        double percent_ZF = 100*double(ZF_task_count[i])/sum_ZF;
        double percent_Demul = 100*double(Demul_task_count[i])/sum_demul;
        printf("thread %d performed FFT: %d (%.2f%%), ZF: %d (%.2f%%), Demulation: %d (%.2f%%)\n", 
            i, FFT_task_count[i], ZF_task_count[i], Demul_task_count[i],
            percent_FFT, percent_Demul, percent_ZF);
    }
    exit(0);
    
}

void* CoMP::taskThread(void* context)
{
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_ = &(obj_ptr->task_queue_);
    moodycamel::ConcurrentQueue<Event_data>* zf_queue_ = &(obj_ptr->zf_queue_);
    moodycamel::ConcurrentQueue<Event_data>* demul_queue_ = &(obj_ptr->demul_queue_);
    moodycamel::ConcurrentQueue<Event_data>* ifft_queue_ = &(obj_ptr->ifft_queue_);
    moodycamel::ConcurrentQueue<Event_data>* modulate_queue_ = &(obj_ptr->modulate_queue_);
    moodycamel::ConcurrentQueue<Event_data>* precode_queue_ = &(obj_ptr->precode_queue_);
    // moodycamel::ConcurrentQueue<Event_data>* tx_queue_ = &(obj_ptr->tx_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);
    
    // attach task threads to specific cores
    // Note: cores 0-17, 36-53 are on the same socket
#ifdef ENABLE_CPU_ATTACH
    int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
    int tar_core_id = tid + offset_id;
    // if(tar_core_id >= 18)
    //     tar_core_id = (tar_core_id - 18) + 36;
    if(stick_this_thread_to_core(tar_core_id) != 0) {
        printf("Task thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else {
        printf("Task thread: stitch thread %d to core %d succeeded\n", tid, tar_core_id);
    }
#endif

    obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->message_queue_));

    int total_count = 0;
    int miss_count = 0;
    Event_data event;
    bool ret = false;
    bool ret_zf = false;
    bool ret_demul = false;
    bool ret_modul = false;
    bool ret_ifft = false;
    bool ret_precode = false;

    while(true) {
        // if (task_queue_->size_approx() > 0) {
            if (ENABLE_DOWNLINK) {
                // do not process uplink data if downlink is enabled
                ret_ifft = ifft_queue_->try_dequeue(event);
                if (!ret_ifft) {
                    ret_precode = precode_queue_->try_dequeue(event);
                    if (!ret_precode) {
                        ret_modul = modulate_queue_->try_dequeue(event);
                        if (!ret_modul) {
                            ret_zf = zf_queue_->try_dequeue(event);
                            if (!ret_zf) {
                                ret = task_queue_->try_dequeue(event);
                                if (!ret) 
                                    continue;
                                else
                                    obj_ptr->doCrop(tid, event.data);
                            }
                            else if (event.event_type == TASK_ZF) {
                                obj_ptr->doZF(tid, event.data);
                            }
                            else if (event.event_type == TASK_PRED) {
                                obj_ptr->doPred(tid, event.data);
                            }
                        }
                        else {
                            obj_ptr->do_modulate(tid, event.data);
                        }
                    }
                    else {
                        obj_ptr->do_precode(tid, event.data);
                    }
                }
                else {
                    obj_ptr->do_ifft(tid, event.data);
                }
            }
            else {
                ret_zf = zf_queue_->try_dequeue(event);
                if (!ret_zf) {
                    ret_demul = demul_queue_->try_dequeue(event);
                    if (!ret_demul) {   
                        ret = task_queue_->try_dequeue(event);
                        if (!ret)
                            continue;
                        else 
                            obj_ptr->doCrop(tid, event.data);
                    }
                    else {
                        // TODO: add precoder status check
                        obj_ptr->doDemul(tid, event.data);
                    }
                }
                else if (event.event_type == TASK_ZF) {
                    obj_ptr->doZF(tid, event.data);
                }
                else if (event.event_type == TASK_PRED) {
                    obj_ptr->doPred(tid, event.data);
                }
            } 
        // }
    }

}



inline int CoMP::generateOffset2d(int unit_total_num, int frame_id, int unit_id) 
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * unit_total_num + unit_id;
}

inline int CoMP::generateOffset3d(int unit_total_num, int frame_id, int current_data_subframe_id, int unit_id)
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    int total_data_subframe_id = frame_id * data_subframe_num_perframe + current_data_subframe_id;
    return total_data_subframe_id * unit_total_num + unit_id;
}

inline void CoMP::interpreteOffset2d(int unit_total_num, int offset, int *frame_id, int *unit_id)
{
    *unit_id = offset % unit_total_num;
    *frame_id = offset / unit_total_num;
}

inline void CoMP::interpreteOffset3d(int unit_total_num, int offset, int *frame_id, int *total_data_subframe_id, int *current_data_subframe_id, int *unit_id)
{
    *unit_id = offset % unit_total_num;
    *total_data_subframe_id = offset /unit_total_num;
    *current_data_subframe_id = (*total_data_subframe_id) % data_subframe_num_perframe;
    *frame_id = (*total_data_subframe_id) / data_subframe_num_perframe;
}


inline int CoMP::getFFTBufferIndex(int frame_id, int subframe_id, int ant_id) 
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * (BS_ANT_NUM * subframe_num_perframe) + subframe_id * BS_ANT_NUM + ant_id;
}


inline void CoMP::splitFFTBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id, int *ant_id)
{
    (*frame_id) = FFT_buffer_target_id / (BS_ANT_NUM * subframe_num_perframe);
    FFT_buffer_target_id = FFT_buffer_target_id - (*frame_id) * (BS_ANT_NUM * subframe_num_perframe);
    (*subframe_id) = FFT_buffer_target_id / BS_ANT_NUM;
    (*ant_id) = FFT_buffer_target_id - *subframe_id * BS_ANT_NUM;
}

inline complex_float CoMP::divide(complex_float e1, complex_float e2)
{
    complex_float re;
    float module = e2.real * e2.real + e2.imag * e2.imag;
    re.real = (e1.real * e2.real + e1.imag * e2.imag) / module;
    re.imag = (e1.imag * e2.real - e1.real * e2.imag) / module;
    return re;
}


inline imat CoMP::demod_16qam(cx_fmat x)
{
    imat re;
    mat zero_mat = zeros<mat>(size(x));
    // mat float_mat = 0.6325*ones<mat>(size(x));
    mat float_mat(size(x));
    float_mat.fill(0.6325);

    umat c1 = real(x)>zero_mat;
    imat c1_int = conv_to<imat>::from(c1);
    umat c2 = abs(real(x))<float_mat;
    imat c2_int = conv_to<imat>::from(c2);
    umat c3 = imag(x)>zero_mat;
    imat c3_int = conv_to<imat>::from(c3);
    umat c4 = abs(imag(x))<float_mat;
    imat c4_int = conv_to<imat>::from(c4);
    re = 8*c1_int+4*c2_int+2*c3_int+1*c4_int;
    // cout << "In demod_16qam: memory of x: " << x.memptr() << ",  memory of re: " << re.memptr() << endl;
    // cout << "x:" << endl;
    // cout << x.st() << endl;
    // cout <<imag(x).st() << endl;
    // cout << "Re:" << re.st() << endl;
    return re;
}


inline cx_fmat CoMP::mod_16qam(imat x)
{
    // cx_fmat re(size(x));
    fmat real_re = conv_to<fmat>::from(x);
    fmat imag_re = conv_to<fmat>::from(x);
    // float scale = 1/sqrt(10);
    // float modvec_16qam[4]  = {-3*scale, -1*scale, 3*scale, scale};

    // real_re.for_each([&modvec_16qam](fmat::elem_type& val) { val = modvec_16qam[(int)val/4]; } );
    // imag_re.for_each([&modvec_16qam](fmat::elem_type& val) { val = modvec_16qam[(int)val%4]; } );
    real_re.for_each([this](fmat::elem_type& val) { val = qam16_table[0][(int)val]; } );
    imag_re.for_each([this](fmat::elem_type& val) { val = qam16_table[1][(int)val]; } );
    cx_fmat re(real_re, imag_re);
    // re.set_real(real_re);
    // re.set_imag(imag_re);
    // cout << "In mod_16qam: memory of x: " << x.memptr() << ",  memory of re: " << re.memptr() << endl;
    // cout << "x:" << endl;
    // cout << x.st() << endl;
    // cout << "Re:" << real(re).st() << endl;
    return re;
}


inline complex_float CoMP::mod_16qam_single(int x) 
{
    complex_float re;
    re.real = qam16_table[0][x];
    re.imag = qam16_table[1][x];
    return re;
}

/*****************************************************
 * Uplink tasks
 *****************************************************/

void CoMP::doCrop(int tid, int offset)
{
    double start_time = get_time();
    int buffer_subframe_num = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    int socket_thread_id = offset / buffer_subframe_num;
    // offset = offset - socket_thread_id * buffer_subframe_num;
    offset = offset % buffer_subframe_num;
    // printf("In doCrop: socket_thread: %d offset %d\n", socket_thread_id, offset);
    // read info of one frame
    char *cur_ptr_buffer = socket_buffer_[socket_thread_id].buffer + offset * PackageReceiver::package_length;

    int ant_id, frame_id, subframe_id, cell_id;
    frame_id = *((int *)cur_ptr_buffer);
    subframe_id = *((int *)cur_ptr_buffer + 1);
    cell_id = *((int *)cur_ptr_buffer + 2);
    ant_id = *((int *)cur_ptr_buffer + 3);
    // printf("thread %d process frame_id %d, subframe_id %d, cell_id %d, ant_id %d\n", tid, frame_id, subframe_id, cell_id, ant_id);
    // remove CP, do FFT
    int delay_offset = 0;
    // int FFT_buffer_target_id = getFFTBufferIndex(frame_id, subframe_id, ant_id);
    int FFT_buffer_target_id = (frame_id % TASK_BUFFER_FRAME_NUM) * (subframe_num_perframe) + subframe_id;


    // transfer ushort to float
    short *cur_ptr_buffer_ushort = (short *)(cur_ptr_buffer + 64);
    // float *cur_fft_buffer_float = (float *)fft_buffer_.FFT_inputs[FFT_buffer_target_id];
    // float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM);
    float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[tid] + ant_id * OFDM_CA_NUM);

    // Copy data for fft from socket buffer to fft buffer
    // fmat mat_fft_float_input(cur_fft_buffer_float, OFDM_CA_NUM * 2, 1, false);
    // Mat <short> mat_fft_ushort(cur_ptr_buffer_ushort+delay_offset, OFDM_CA_NUM * 2, 1, false);
    // fmat mat_fft_float_raw = conv_to<fmat>::from(mat_fft_ushort);
    // // cout<<"Ushort: "<<mat_fft_ushort.st()<<endl;
    // // cout<<"float: "<<vec_fft_float_in.st()<<endl;
    // mat_fft_float_input = mat_fft_float_raw * csi_format_offset;
    // // cout<<"Output: "<<mat_fft_float_input.st()<<endl;
    // // cout<<"Complex mat: "<<conv_to<cx_fmat>::from(mat_fft_input).st()<<endl;
    // // cout<<endl;

    // Use SIMD
    // reference: https://stackoverflow.com/questions/50597764/convert-signed-short-to-float-in-c-simd
    // 0x4380'8000
    const __m256 magic = _mm256_set1_ps(float((1<<23) + (1<<15))/32768.f);
    const __m256i magic_i = _mm256_castps_si256(magic);
    for (int i = 0; i < OFDM_CA_NUM * 2; i += 8) {
        // get input:
        __m128i val = _mm_load_si128((__m128i*)(cur_ptr_buffer_ushort + i));
        // interleave with 0x0000
        __m256i val_unpacked = _mm256_cvtepu16_epi32(val);
        /// convert by xor-ing and subtracting magic value:
        // VPXOR avoids port5 bottlenecks on Intel CPUs before SKL
        __m256 val_f = _mm256_castsi256_ps(_mm256_xor_si256(val_unpacked, magic_i));
        __m256 converted = _mm256_sub_ps(val_f, magic);
        // store:
        _mm256_store_ps(cur_fft_buffer_float + i, converted);

    }
    FFT_task_duration_part1[tid] += get_time() - start_time;
    // __m256 format_offset = _mm256_set1_ps(csi_format_offset);
    // for (int i = 0; i < OFDM_CA_NUM * 2; i += 8)
    // {
        // //  Load 8 16-bit shorts.
        // //  vi = {a,b,c,d,e,f,g,h}
        // __m128i vi = _mm_load_si128((const __m128i*)(cur_ptr_buffer_ushort + i));

        // //  Convert to 32-bit integers
        // __m256i vi0 = _mm256_cvtepi16_epi32(vi);

        // //  Convert to float
        // __m256 vf0 = _mm256_cvtepi32_ps(vi0);

        // //  Multiply
        // vf0 = _mm256_mul_ps(vf0, format_offset);

        // //  Store
        // _mm256_store_ps(cur_fft_buffer_float + i, vf0);

    // }

    // for(int i = 0; i < (OFDM_CA_NUM - delay_offset) * 2; i++)
    //     cur_fft_buffer_float[i] = cur_ptr_buffer_ushort[OFDM_PREFIX_LEN + delay_offset + i] * csi_format_offset;
   
    // append zero
    // if(delay_offset > 0) 
    //     memset((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id] 
    //         + (OFDM_CA_NUM - delay_offset) * 2 * sizeof(float), 0, sizeof(float) * 2 * delay_offset);
    mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[tid] + ant_id * OFDM_CA_NUM, 
        fft_buffer_.FFT_inputs[tid] + ant_id * OFDM_CA_NUM);

    FFT_task_duration_part2[tid] += get_time() - start_time;

    // mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM, 
    //     fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM);
    // printf("FFT input\n");
    // for ( int i = 0; i< OFDM_CA_NUM; i++) {
    //     cout <<"("<<(*(fft_buffer_.FFT_inputs[tid] + ant_id * OFDM_CA_NUM+i)).real<<","<<(*(fft_buffer_.FFT_inputs[tid] + ant_id * OFDM_CA_NUM+i)).imag<<") ";
    //     // cout <<"("<<(*(fft_buffer_.FFT_inputs[FFT_buffer_target_id]+i)).real<<","<<(*(fft_buffer_.FFT_inputs[FFT_buffer_target_id]+i)).imag<<") ";
    //     // printf("(%.4f, %.4f) ", *((float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)), *((float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)+1));
    // }
    // printf("\n");
    // printf("\n FFT output: \n");
    // for ( int i = 0; i< OFDM_CA_NUM; i++) {
    //     printf("(%.4f, %.4f) ", *((float *)(fft_buffer_.FFT_outputs[tid]+i)), *((float *)(fft_buffer_.FFT_outputs[tid]+i)+1));
    //      // printf("(%.4f, %.4f) ", *((float *)(fft_buffer_.FFT_outputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)), *((float *)(fft_buffer_.FFT_outputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM+i)+1));
    // }
    // printf("\n");


#if DEBUG_PRINT_IN_TASK
        printf("In doCrop thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
#endif
    double start_time_part3 = get_time();

    // if it is pilot part, do CE
    if(isPilot(subframe_id)) {
        int UE_id = subframe_id;
        // int ca_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * OFDM_CA_NUM;
        // int csi_offset = ant_id + UE_id * BS_ANT_NUM;
        int subframe_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * UE_NUM + UE_id;
        // int csi_offset = UE_id + ant_id * UE_NUM;

        float* cur_fft_buffer_float_output = (float*)(fft_buffer_.FFT_outputs[tid] + ant_id * OFDM_CA_NUM);

        // Use SIMD
        float* csi_buffer_ptr = (float*)(csi_buffer_.CSI[subframe_offset]);
        int sc_idx = OFDM_DATA_START;
        int dst_idx = 0;

        for (int block_idx = 0; block_idx < OFDM_DATA_NUM / transpose_block_size; block_idx ++) {

            for (int sc_inblock_idx = 0; sc_inblock_idx < transpose_block_size; sc_inblock_idx += 4) {
                // load 8 floats (4 bytes) / 4 complex floats
                __m256 pilot_rx = _mm256_load_ps(cur_fft_buffer_float_output + sc_idx * 2);
                __m256 pilot_tx = _mm256_set_ps(pilots_[sc_idx+3], pilots_[sc_idx+3], pilots_[sc_idx+2], pilots_[sc_idx+2], 
                                                pilots_[sc_idx+1], pilots_[sc_idx+1], pilots_[sc_idx], pilots_[sc_idx]);
                __m256 csi_est = _mm256_mul_ps(pilot_rx, pilot_tx);
                _mm256_stream_ps(csi_buffer_ptr + (block_idx * BS_ANT_NUM + ant_id) * transpose_block_size * 2 + sc_inblock_idx * 2, csi_est);
                // printf("subcarrier index: %d, pilot: %.2f, %.2f, %.2f, %2.f\n", sc_idx, pilots_[sc_idx], pilots_[sc_idx+1], pilots_[sc_idx+2], pilots_[sc_idx+3]);
                sc_idx += 4;
                
            }
        }
        
        // cout<<"Before: "<<endl;
        // for (int i =0;i<OFDM_CA_NUM; i++) {
        //     cout<<"("<<i<<", "<<fft_buffer_.FFT_outputs[FFT_buffer_target_id][i].real<<","<<fft_buffer_.FFT_outputs[FFT_buffer_target_id][i].imag<<") ";
        // }
        // cout<<endl;

        // cout<<"After: "<<endl;
        // for (int i =0;i<OFDM_CA_NUM*BS_ANT_NUM; i++) {
        //     cout<<"("<<i<<", "<<csi_buffer_.CSI[subframe_offset][i].real<<","<<csi_buffer_.CSI[subframe_offset][i].imag<<") ";
        // }
        // cout<<endl;


        // for(int j = 0; j < (OFDM_CA_NUM); j++) {
        //     // divide fft output by pilot data to get CSI estimation
        //     float* csi_buffer_ptr = (float*)(csi_buffer_.CSI[ca_offset+j].data())+csi_offset*2;
        //     *(csi_buffer_ptr)= cur_fft_buffer_float_output[2*j]* pilots_[j];
        //     *(csi_buffer_ptr+1)= cur_fft_buffer_float_output[2*j+1]* pilots_[j];           
        // }     
    }
    if(isData(subframe_id)) {
        
        int data_subframe_id = subframe_id - UE_NUM;
        int frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * data_subframe_num_perframe + data_subframe_id;
        
        // cx_fmat mat_fft_cx_output((cx_float *)fft_buffer_.FFT_outputs[FFT_buffer_target_id], OFDM_CA_NUM, 1, false);


        /* //naive transpose
        for(int j = 0; j < OFDM_CA_NUM; j++)
        {
            data_buffer_.data[frame_offset][ant_id + j * BS_ANT_NUM] = fft_buffer_.FFT_outputs[FFT_buffer_target_id][j];
        }
        */
        // block transpose
        // src_ptr: point to the start of subframe (subframe size: OFDM_CA_NUM) 
        // 2048 float values
        // cx_float *src_ptr = (cx_float *)&fft_buffer_.FFT_outputs[FFT_buffer_target_id][0];
        // cx_mat mat_data_buffer(src_ptr, BS_ANT_NUM, )


        // float *src_ptr = (float *)&fft_buffer_.FFT_outputs[FFT_buffer_target_id][0];
        float *src_ptr = (float *)&fft_buffer_.FFT_outputs[tid][ant_id*OFDM_CA_NUM] + OFDM_DATA_START * 2;

        // printf("FFT output: \n");
        // for ( int i = 0; i< OFDM_CA_NUM; i++) {
        //      printf("(%.4f, %.4f) ", *(src_ptr+i*2), *(src_ptr+i*2+1));
        // }
        // printf("\n");

        // tar_ptr: point to the start of subframe with size BS_ANT_NUM * OFDM_CA_NUM
        // 96*1024*2 float values
        float *tar_ptr = (float *)&data_buffer_.data[frame_offset][0];
        // copy data from fft_outputs to data_buffer
        // 1024*2/8 = 256 iterations, copy 8 bytes every time
        // c2 = 0, 1, ..., 1024/64*2-1 = 31
        // c2*transpose_block_size = 0, 64, 128, ..., 2048-64
        for(int c2 = 0; c2 < OFDM_DATA_NUM / transpose_block_size; c2++) {
            // c3 = 0, 1, ..., transpose_block_size/8 -1 = 7
            // c3*8 = 0, 8, ..., 64-8
            for(int c3 = 0; c3 < transpose_block_size / 4; c3++) {
                // data: 256 bits = 32 bytes = 8 float values = 4 subcarriers

                __m256 data = _mm256_load_ps(src_ptr);
                // original data order: SCs of ant1, SCs of ant2, ..., SCs of ant 96
                // transposed data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
                _mm256_stream_ps(tar_ptr + (c2 * BS_ANT_NUM + ant_id)* transpose_block_size * 2 + c3 * 8, data);
                src_ptr += 8;
            }            
        }        
    }
    FFT_task_duration_part3[tid] += get_time() - start_time_part3;

    // after finish
    socket_buffer_[socket_thread_id].buffer_status[offset] = 0; // now empty
    // printf("In doCrop: emptied socket buffer frame: %d, subframe: %d, ant: %d, offset: %d\n",frame_id, subframe_id, ant_id, offset);
    // inform main thread
    Event_data crop_finish_event;
    crop_finish_event.event_type = EVENT_CROPPED;
    crop_finish_event.data = generateOffset2d(subframe_num_perframe, frame_id, subframe_id);
    // getSubframeBufferIndex(frame_id, subframe_id);
    FFT_task_count[tid] = FFT_task_count[tid]+1;
    
    FFT_task_duration[tid] += get_time() - start_time;
    if ( !complete_task_queue_.enqueue(*task_ptok[tid], crop_finish_event ) ) {
        printf("crop message enqueue failed\n");
        exit(0);
    }
}


void CoMP::doZF(int tid, int offset)
{
    double start_time = get_time();
    int frame_id, sc_id;
    interpreteOffset2d(OFDM_DATA_NUM, offset, &frame_id, &sc_id);

    if (DEBUG_PRINT_IN_TASK)
        printf("In doZF thread %d: frame: %d, subcarrier: %d\n", tid, frame_id, sc_id);

    // // directly gather data from FFT buffer
    // __m256i index = _mm256_setr_epi32(0, 1, OFDM_CA_NUM * 2, OFDM_CA_NUM * 2 + 1, OFDM_CA_NUM * 4, OFDM_CA_NUM * 4 + 1, OFDM_CA_NUM * 6, OFDM_CA_NUM * 6 + 1);
    __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2, transpose_block_size * 2 + 1, transpose_block_size * 4, transpose_block_size * 4 + 1, transpose_block_size * 6, transpose_block_size * 6 + 1);
    // __m256i index = _mm256_setr_epi64x(0, transpose_block_size/2, transpose_block_size/2 * 2, transpose_block_size/2 * 3);

    int transpose_block_id = sc_id / transpose_block_size;
    int sc_inblock_idx = sc_id % transpose_block_size;
    int offset_in_csi_buffer = transpose_block_id * BS_ANT_NUM * transpose_block_size  + sc_inblock_idx;
    // double *tar_csi_ptr = (double *)csi_gather_buffer[tid];
    int subframe_offset = frame_id * UE_NUM;
    // int subframe_offset = frame_id * subframe_num_perframe;
    float *tar_csi_ptr = (float *)csi_gather_buffer[tid];

    // if (sc_id == 4) {
    //     cout<<"csi_buffer_ for subframe "<<subframe_offset<<endl;
    //     for (int i=0;i<BS_ANT_NUM*OFDM_CA_NUM; i++) {
    //         cout<<"("<<i<<",  "<<csi_buffer_.CSI[subframe_offset][i].real<<","<<csi_buffer_.CSI[subframe_offset][i].imag<<") ";
    //     }

    //     cout<<endl;
    // }

    // gather data for all users and antennas
    // printf("In doZF thread %d: frame: %d, subcarrier: %d\n", tid, frame_id, sc_id);
    for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
        float *src_csi_ptr = (float *)csi_buffer_.CSI[subframe_offset + ue_idx] + offset_in_csi_buffer * 2;
        for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
            // fetch 4 complex floats for 4 ants
            __m256 t_csi = _mm256_i32gather_ps(src_csi_ptr, index, 4);
            _mm256_store_ps(tar_csi_ptr, t_csi);
            // printf("UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float *)tar_csi_ptr+1), 
            //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),  *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));
            src_csi_ptr += 8 * transpose_block_size;
            tar_csi_ptr += 8;
        }
    }


    // // gather data for all users and antennas
    // for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
    //     float *src_csi_ptr = (float *)fft_buffer_.FFT_outputs[subframe_offset+ue_idx] + sc_id * 2;
    //     for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
    //         // fetch 4 complex floats for 4 ants
    //         __m256 pilot_rx = _mm256_i32gather_ps(src_csi_ptr, index, 4);
           
    //         if (pilots_[sc_id] > 0) {
    //             _mm256_store_ps(tar_csi_ptr, pilot_rx);
    //         }     
    //         else if (pilots_[sc_id] < 0){
    //             __m256 pilot_tx = _mm256_set1_ps(pilots_[sc_id]);
    //             __m256 csi_est = _mm256_mul_ps(pilot_rx, pilot_tx);
    //             _mm256_store_ps(tar_csi_ptr, csi_est);
    //         }
    //         else {
    //             _mm256_store_ps(tar_csi_ptr, _mm256_setzero_ps());
    //         }

    //         // printf("Frame %d, sc: %d, UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", frame_id, sc_id, ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float *)tar_csi_ptr+1), 
    //         //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),  *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));
    //         src_csi_ptr += 8 * OFDM_CA_NUM;
    //         tar_csi_ptr += 8;

    //     }
    // }

    cx_float *ptr_in = (cx_float *)csi_gather_buffer[tid];
    //cx_float *ptr_in = (cx_float *)csi_buffer_.CSI[offset].data();
    cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
    // cout<<"CSI matrix"<<endl;
    // cout<<mat_input.st()<<endl;
    // cx_fmat mat_input(ptr_in, UE_NUM, BS_ANT_NUM, false);
    cx_float *ptr_out = (cx_float *)precoder_buffer_.precoder[offset];
    cx_fmat mat_output(ptr_out, UE_NUM, BS_ANT_NUM, false);
    // cx_fmat mat_output(ptr_out, BS_ANT_NUM, UE_NUM, false);
    
    pinv(mat_output, mat_input, 1e-1, "dc");


    // inform main thread
    Event_data ZF_finish_event;
    ZF_finish_event.event_type = EVENT_ZF;
    ZF_finish_event.data = offset;
    ZF_task_count[tid] = ZF_task_count[tid]+1;
    ZF_task_duration[tid] += get_time() - start_time;

    if ( !complete_task_queue_.enqueue(*task_ptok[tid], ZF_finish_event ) ) {
        printf("ZF message enqueue failed\n");
        exit(0);
    }
}


void CoMP::doPred(int tid, int offset) 
{
    int frame_id, sc_id;
    interpreteOffset2d(OFDM_DATA_NUM, offset, &frame_id, &sc_id);
    int offset_next_frame = ((frame_id+1)%TASK_BUFFER_FRAME_NUM)*OFDM_CA_NUM+sc_id;
    // Use stale CSI as predicted CSI
    // TODO: add prediction algorithm
    cx_float *ptr_in = (cx_float *)pred_csi_buffer_.CSI[sc_id];
    memcpy(ptr_in, (cx_float *)csi_buffer_.CSI[offset], sizeof(cx_float)*BS_ANT_NUM*UE_NUM);
    cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
    cx_float *ptr_out = (cx_float *)precoder_buffer_.precoder[offset_next_frame];
    cx_fmat mat_output(ptr_out, UE_NUM, BS_ANT_NUM, false);
    pinv(mat_output, mat_input, 1e-1, "dc");

    // inform main thread
    Event_data pred_finish_event;
    pred_finish_event.event_type = EVENT_ZF;
    pred_finish_event.data = offset_next_frame;

    if ( !complete_task_queue_.enqueue(*task_ptok[tid], pred_finish_event ) ) {
        printf("Prediction message enqueue failed\n");
        exit(0);
    }
}


void CoMP::doDemul(int tid, int offset)
{
    double start_time = get_time();
    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
    interpreteOffset3d(OFDM_DATA_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);
    int subframe_offset = subframe_num_perframe * frame_id + UE_NUM + current_data_subframe_id;
    

    // int gather_step_size = 8 * OFDM_CA_NUM;
    int gather_step_size = 8 * transpose_block_size;

#if DEBUG_PRINT_IN_TASK
        printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,sc_id);
#endif
        // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,sc_id);

    // index=[0,32,64,96]
    // __m256i index = _mm256_setr_epi64x(0, transpose_block_size/2, transpose_block_size/2 * 2, transpose_block_size/2 * 3);
    // __m256i index = _mm256_setr_epi64x(0, OFDM_CA_NUM, OFDM_CA_NUM * 2, OFDM_CA_NUM * 3);
    // __m256i index = _mm256_setr_epi32(0, 1, OFDM_CA_NUM * 2, OFDM_CA_NUM * 2 + 1, OFDM_CA_NUM * 4, OFDM_CA_NUM * 4 + 1, OFDM_CA_NUM * 6, OFDM_CA_NUM * 6 + 1);
       __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2, transpose_block_size * 2 + 1, transpose_block_size * 4, transpose_block_size * 4 + 1, transpose_block_size * 6, transpose_block_size * 6 + 1);
    // float* tar_ptr = (float *)&data_buffer_.data[total_data_subframe_id][0];
    // float* temp_buffer_ptr = (float *)&spm_buffer[tid][0];


    

    // i = 0, 1, ..., 31
    for (int i = 0; i < demul_block_size/8; i++) {
        
        // int cur_block_id = (sc_id + i) / transpose_block_size;
        // int sc_inblock_idx = i % transpose_block_size;
        // int cur_sc_offset = cur_block_id * transpose_block_size * BS_ANT_NUM + sc_inblock_idx;
        // float *tar_data_ptr = (float *)spm_buffer[tid];        
        // float *src_data_ptr = (float *)data_buffer_.data[total_data_subframe_id] + cur_sc_offset * 2;
        // for (int c2 = 0; c2 < BS_ANT_NUM / 4; c2++) {
        //     __m256 data_rx = _mm256_i32gather_ps(src_data_ptr, index, 4);
        //     _mm256_store_ps(tar_data_ptr, data_rx);

        //     // printf("Frame %d, sc: %d, UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", frame_id, sc_id, ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float *)tar_csi_ptr+1), 
        //         //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),  *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));           
        //     src_data_ptr += gather_step_size;
        //     tar_data_ptr += 8;
        // }


        // for a cache line size of 64 bytes, each read load 8 subcarriers
        // use spatial locality to reduce latency
        // float *tar_data_ptr = (float *)spm_buffer[tid];        
        // // float *src_data_ptr = (float *)fft_buffer_.FFT_outputs[subframe_offset] + (sc_id + i * 8) * 2;
        // float *src_data_ptr = (float *)fft_buffer_.FFT_outputs[subframe_offset] + (sc_id + i) * 2;

        int cur_block_id = (sc_id + i * 8) / transpose_block_size;
        int sc_inblock_idx = (i * 8) % transpose_block_size;
        int cur_sc_offset = cur_block_id * transpose_block_size * BS_ANT_NUM + sc_inblock_idx;
        float *tar_data_ptr = (float *)spm_buffer[tid];        
        float *src_data_ptr = (float *)data_buffer_.data[total_data_subframe_id] + cur_sc_offset * 2;

        for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
            // fetch 4 complex floats for 4 ants
            for (int j = 0; j < 8; j++) {
                __m256 data_rx = _mm256_i32gather_ps(src_data_ptr + j * 2, index, 4);
                _mm256_store_ps(tar_data_ptr + j * BS_ANT_NUM * 2, data_rx);

                // printf("Frame %d, sc: %d, UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", frame_id, sc_id, ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float *)tar_csi_ptr+1), 
                //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),  *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));   
            }
            src_data_ptr += gather_step_size;
            tar_data_ptr += 8;
        }
        

        for (int j = 0; j < 8; j++) {
            int cur_sc_id = i * 8 + j + sc_id;
            // mat_data size: BS_ANT_NUM \times 1
            cx_float* data_ptr = (cx_float *)(spm_buffer[tid] + j * BS_ANT_NUM);
            cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);
            // cout<< "Raw data: " << mat_data.st()<<endl;

            // mat_precoder size: UE_NUM \times BS_ANT_NUM
            int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
            cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];
            cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);
            // cout<<"Precoder: "<< mat_precoder<<endl;

            // mat_demuled size: UE_NUM \times 1
            cx_float* equal_ptr = (cx_float *)(&equal_buffer_.data[total_data_subframe_id][cur_sc_id * UE_NUM]);
            cx_fmat mat_equaled(equal_ptr, UE_NUM, 1, false);

            // Equalization
            mat_equaled = mat_precoder* mat_data;
            // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,cur_sc_id);

            // Demodulation
            sword* demul_ptr = (sword *)(&demul_buffer_.data[total_data_subframe_id][cur_sc_id * UE_NUM]);
            imat mat_demuled(demul_ptr, UE_NUM, 1, false);

            mat_demuled = demod_16qam(mat_equaled);
            // cout << "Demuled data: "<<mat_demuled.st()<<endl;

        }

        // for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
        //     // fetch 4 complex floats for 4 ants
            
        //     __m256 data_rx = _mm256_i32gather_ps(src_data_ptr, index, 4);
        //     _mm256_store_ps(tar_data_ptr, data_rx);

        //     // printf("Frame %d, sc: %d, UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", frame_id, sc_id, ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float *)tar_csi_ptr+1), 
        //         //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),  *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));           
        //     src_data_ptr += gather_step_size;
        //     tar_data_ptr += 8;
        // }
              
        // // mat_precoder size: UE_NUM \times BS_ANT_NUM
        // int precoder_offset = frame_id * OFDM_CA_NUM + sc_id + i;
        // cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];
        // cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);
        // // // cx_fmat mat_precoder(precoder_ptr, BS_ANT_NUM, UE_NUM, false);
        // // // cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

        // // mat_data size: BS_ANT_NUM \times 1
        // cx_float* data_ptr = (cx_float *)(spm_buffer[tid]);
        // cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);
        

        // // mat_demuled size: UE_NUM \times 1
        // cx_float* equal_ptr = (cx_float *)(&equal_buffer_.data[total_data_subframe_id][(sc_id + i) * UE_NUM]);
        // cx_fmat mat_equaled(equal_ptr, UE_NUM, 1, false);

        // // Equalization
        // mat_equaled = mat_precoder* mat_data;
        // // mat_equaled = mat_precoder.st() * mat_data;
        // // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,sc_id+i);
        // // cout<< "Raw data: " << mat_data.st()<<endl;
        // // cout<<"Precoder: "<< mat_precoder<<endl;
        // // cout << "Equaled data: "<<mat_equaled.st()<<endl;


        // // Demodulation
        // sword* demul_ptr = (sword *)(&demul_buffer_.data[total_data_subframe_id][(sc_id + i) * UE_NUM]);
        // imat mat_demuled(demul_ptr, UE_NUM, 1, false);
        
        // // cout << "Memory pointer before: " << mat_demuled.memptr() << ",  ";
        // mat_demuled = demod_16qam(mat_equaled);
        // cout << "Demuled data: "<<mat_demuled.st()<<endl;
        // cout << "Demuled data: " << mat_demuled.st() << endl;
        // cout << "Memory pointer after: " << mat_demuled.memptr() << endl;
        // cout << "Frame: "<< frame_id<<", subframe: "<< data_subframe_id<<", SC: " << sc_id+i << ", data: " << mat_demuled2.st() << endl;

        // if (frame_id==0) 
        // {
        //     FILE* fp_debug = fopen("tmpresult.txt", "a");
        //     if (fp_debug==NULL) {
        //         printf("open file faild");
        //         std::cerr << "Error: " << strerror(errno) << std::endl;
        //         exit(0);
        //     }
        //     for(int ii = 0; ii < UE_NUM; ii++)
        //     {
        //         // printf("User %d: %d, ", ii,demul_ptr2(ii));
        //         fprintf(fp_debug, "%d\n", mat_demuled2(ii));
        //     }
        //     // printf("\n");
        //     // fwrite(mat_demuled2.memptr(), sizeof(int),sizeof(mat_demuled), fp_debug);
        //     fclose(fp_debug);
        // }
    }

    // inform main thread
    Event_data demul_finish_event;
    demul_finish_event.event_type = EVENT_DEMUL;
    demul_finish_event.data = offset;
    Demul_task_count[tid] = Demul_task_count[tid]+1;
    Demul_task_duration[tid] += get_time() - start_time;
    
    if ( !complete_task_queue_.enqueue(*task_ptok[tid], demul_finish_event ) ) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }    
}


/*****************************************************
 * Downlink tasks
 *****************************************************/

void CoMP::do_modulate(int tid, int offset) 
{
    int frame_id, total_data_subframe_id, current_data_subframe_id, user_id;
    interpreteOffset3d(UE_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &user_id);
    int *input_ptr = &dl_IQ_data[current_data_subframe_id * UE_NUM + user_id][0];
    complex_float *output_ptr = &dl_modulated_buffer_.data[total_data_subframe_id][0];
    for (int i = 0; i < OFDM_CA_NUM; i++) {
        *(output_ptr + i * UE_NUM + user_id) = mod_16qam_single(*(input_ptr+i));
        // printf(" (%d, %.4f+j%.4f) ", *(input_ptr+i), (*(output_ptr + i * OFDM_CA_NUM + user_id)).real, (*(output_ptr + i * OFDM_CA_NUM + user_id)).imag);
    }

    // cout<<"Frame "<<frame_id<<", subframe "<<current_data_subframe_id<<", user "<<user_id<<", Data:\n";
    // for (int i = 0; i < OFDM_CA_NUM*UE_NUM; i++) {
    //     printf(" (%d, %.4f+j%.4f) ", i, (*(output_ptr + i)).real, (*(output_ptr + i)).imag);
    // }

    // inform main thread
    Event_data modulate_finish_event;
    modulate_finish_event.event_type = EVENT_MODUL;
    modulate_finish_event.data = offset;
    
    if ( !complete_task_queue_.enqueue(*task_ptok[tid], modulate_finish_event ) ) {
        printf("Modulation message enqueue failed\n");
        exit(0);
    }
}


void CoMP::do_precode(int tid, int offset) 
{
    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
    interpreteOffset3d(OFDM_CA_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);

    for (int i = 0; i < transpose_block_size/2; i++) { 
        int precoder_offset = frame_id * OFDM_CA_NUM + sc_id + i;
        // mat_precoder size: UE_NUM \times BS_ANT_NUM        
        cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];
        cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

        // mat_data size: UE_NUM \times 1
        cx_float* data_ptr = (cx_float *)(&dl_modulated_buffer_.data[total_data_subframe_id][UE_NUM * (sc_id+i)]);
        cx_fmat mat_data(data_ptr, UE_NUM, 1, false);
        // cout << "Frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", SC: " << sc_id+i << ", data: " << real(mat_data).st() << endl;

        // mat_precoded size: BS_ANT_NUM \times 1
        cx_float* precoded_ptr = (cx_float *)(&dl_precoded_data_buffer_.data[total_data_subframe_id][(sc_id + i) * BS_ANT_NUM]);
        cx_fmat mat_precoded(precoded_ptr, BS_ANT_NUM, 1, false);

        mat_precoded = mat_precoder.t() * mat_data;
        // cout << "Frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", SC: " << sc_id+i << ", data: " << real(mat_precoded).st() << endl;
        // cout << "Precoded data: \n" ;
        // for (int j = 0; j < BS_ANT_NUM; j++) {
        //     cout <<*((float *)(precoded_ptr+j)) << "+j"<<*((float *)(precoded_ptr+j)+1)<<",   ";
        // }
    }


    // copy data to ifft input
    __m256i index = _mm256_setr_epi64x(0, BS_ANT_NUM, BS_ANT_NUM * 2, BS_ANT_NUM * 3);
    float* precoded_ptr = (float *)&dl_precoded_data_buffer_.data[total_data_subframe_id][sc_id * BS_ANT_NUM];
    for (int ant_id = 0; ant_id < BS_ANT_NUM; ant_id++) {
        int ifft_buffer_offset = generateOffset3d(BS_ANT_NUM, frame_id, current_data_subframe_id, ant_id);
        float* ifft_ptr = (float *)&dl_ifft_buffer_.IFFT_inputs[ifft_buffer_offset][sc_id];
        for (int i = 0; i< demul_block_size/4; i++) {
            float *input_shifted_ptr = precoded_ptr + 4 * i * 2 * BS_ANT_NUM + ant_id * 2;
            __m256d t_data = _mm256_i64gather_pd((double *)input_shifted_ptr, index, 8);
            _mm256_store_pd((double *)(ifft_ptr + i * 8), t_data);
        }
    }

    // inform main thread
    Event_data precode_finish_event;
    precode_finish_event.event_type = EVENT_PRECODE;
    precode_finish_event.data = offset;
    
    if ( !complete_task_queue_.enqueue(*task_ptok[tid], precode_finish_event ) ) {
        printf("Precoding message enqueue failed\n");
        exit(0);
    }
}

void CoMP::do_ifft(int tid, int offset)
{
    int frame_id, total_data_subframe_id, current_data_subframe_id, ant_id;
    interpreteOffset3d(BS_ANT_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &ant_id);

    mufft_execute_plan_1d(muplans_ifft_[tid], dl_ifft_buffer_.IFFT_outputs[offset], 
        dl_ifft_buffer_.IFFT_inputs[offset]);

    // calculate data for downlink socket buffer 
    float *ifft_output_ptr = (float *)(&dl_ifft_buffer_.IFFT_outputs[offset][0]);
    int socket_subframe_offset = (frame_id % SOCKET_BUFFER_FRAME_NUM) * data_subframe_num_perframe + current_data_subframe_id;
    char *socket_ptr = &dl_socket_buffer_.buffer[socket_subframe_offset * BS_ANT_NUM * PackageReceiver::package_length];

    for (int sc_id = 0; sc_id < OFDM_CA_NUM; sc_id++) {
        float *shifted_input_ptr = (float *)(ifft_output_ptr + 2 * sc_id);
        int socket_offset = sizeof(int) * 4 + ant_id * PackageReceiver::package_length;
        *((ushort *)(socket_ptr + socket_offset) + 2 * sc_id ) = (ushort)(*shifted_input_ptr * 65536 / 4 + 65536 / 2);
        *((ushort *)(socket_ptr + socket_offset) + 2 * sc_id + 1 ) = (ushort)(*(shifted_input_ptr+1) * 65536 / 4 + 65536 / 2);
    }

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id << ", data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     int socket_offset = sizeof(int) * 4 + ant_id * PackageReceiver::package_length;
    //     cout <<*((ushort *)(socket_ptr + socket_offset) + 2 * j)  << "+j"<<*((ushort *)(socket_ptr + socket_offset) + 2 * j + 1 )<<",   ";
    // }
    // cout<<"\n\n"<<endl;

    // inform main thread
    Event_data ifft_finish_event;
    ifft_finish_event.event_type = EVENT_IFFT;
    ifft_finish_event.data = offset;

    if ( !complete_task_queue_.enqueue(*task_ptok[tid], ifft_finish_event ) ) {
        printf("IFFT message enqueue failed\n");
        exit(0);
    }
}



void CoMP::getDemulData(long long **ptr, int *size)
{
    *ptr = (long long *)&equal_buffer_.data[max_equaled_frame*data_subframe_num_perframe][0];
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
    EXPORT void CoMP_getDemulData(CoMP *comp, long long **ptr, int *size) {return comp->getDemulData(ptr, size);}
}





