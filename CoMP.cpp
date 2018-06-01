#include "CoMP.hpp"

using namespace arma;
typedef cx_float COMPLEX;

CoMP::CoMP()
{

    printf("enter constructor\n");
    // initialize socket buffer
    for (int i = 0; i < SOCKET_RX_THREAD_NUM; i++) {
        socket_buffer_[i].buffer.resize(PackageReceiver::package_length 
            * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM); // buffer SOCKET_BUFFER_FRAME_NUM entire frame
        socket_buffer_[i].buffer_status.resize(subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM);
    }
    printf("initialize buffers\n");

    // initialize FFT buffer
    int FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    fft_buffer_.FFT_inputs = new complex_float * [FFT_buffer_block_num];
    fft_buffer_.FFT_outputs = new complex_float * [FFT_buffer_block_num];
    for (int i = 0; i < FFT_buffer_block_num; i++) {
        fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
        fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    }

    // initialize muplans for fft
    for (int i = 0; i < TASK_THREAD_NUM; i++) 
        muplans_[i] = mufft_create_plan_1d_c2c(OFDM_CA_NUM, MUFFT_FORWARD, MUFFT_FLAG_CPU_ANY);

    // initialize CSI buffer
    csi_buffer_.CSI.resize(OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM);
    for (int i = 0; i < csi_buffer_.CSI.size(); i++)
        csi_buffer_.CSI[i].resize(BS_ANT_NUM * UE_NUM);
    // printf("CSI buffer initialized\n");


    if (DO_PREDICTION) {
        pred_csi_buffer_.CSI.resize(OFDM_CA_NUM);
        for(int i = 0; i < csi_buffer_.CSI.size(); i++)
            pred_csi_buffer_.CSI[i].resize(BS_ANT_NUM * UE_NUM);
    }

    // initialize data buffer
    data_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for (int i = 0; i < data_buffer_.data.size(); i++)
        data_buffer_.data[i].resize(BS_ANT_NUM * OFDM_CA_NUM);
    // printf("Data buffer initialized\n");

    // initialize precoder buffer
    precoder_buffer_.precoder.resize(OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM);
    for (int i = 0; i < precoder_buffer_.precoder.size(); i++)
        precoder_buffer_.precoder[i].resize(UE_NUM * BS_ANT_NUM);

    // initialize equalized data buffer
    equal_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for (int i = 0; i < equal_buffer_.data.size(); i++)
        equal_buffer_.data[i].resize(OFDM_CA_NUM * UE_NUM);

    // initialize demultiplexed data buffer
    demul_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for (int i = 0; i < demul_buffer_.data.size(); i++)
        demul_buffer_.data[i].resize(OFDM_CA_NUM * UE_NUM);

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        spm_buffer[i].resize(BS_ANT_NUM);
    // printf("Demultiplexed data buffer initialized\n");

    // read pilots from file
    pilots_.resize(OFDM_CA_NUM);
    FILE* fp = fopen("../pilot_f.bin","rb");
    fread(pilots_.data(), sizeof(float), OFDM_CA_NUM, fp);
    fclose(fp);

#ifdef DEBUG_PRINT_PILOT
    cout<<"Pilot data"<<endl;
    for (int i = 0; i<OFDM_CA_NUM;i++) 
        cout<<pilots_[i]<<",";
    cout<<endl;
#endif

    printf("new PackageReceiver\n");
    receiver_.reset(new PackageReceiver(SOCKET_RX_THREAD_NUM, &message_queue_));

    // initilize all kinds of checkers
    memset(cropper_checker_, 0, sizeof(int) * subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    memset(csi_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(data_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_status_, 0, sizeof(bool) * TASK_BUFFER_FRAME_NUM); 

    memset(demul_status_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(cropper_created_checker_, 0, sizeof(int) * subframe_num_perframe * TASK_BUFFER_FRAME_NUM);

    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
        memset(demul_checker_[i], 0, sizeof(int) * (subframe_num_perframe - UE_NUM));

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



#ifdef ENABLE_DOWNLINK
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

    int IFFT_buffer_block_num = UE_NUM * data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    dl_ifft_buffer_.IFFT_inputs = new complex_float*[IFFT_buffer_block_num];
    dl_ifft_buffer_.IFFT_outputs = new complex_float*[IFFT_buffer_block_num];
    for (int i = 0; i < IFFT_buffer_block_num; i++) {
        dl_ifft_buffer_.IFFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
        dl_ifft_buffer_.IFFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    }


    // initialize muplans for ifft
    for (int i = 0; i < TASK_THREAD_NUM; i++)
        muplans_ifft_[i] = mufft_create_plan_1d_c2c(OFDM_CA_NUM, MUFFT_INVERSE, MUFFT_FLAG_CPU_ANY);

    // initialize downlink iffted data buffer
    dl_iffted_data_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for (int i = 0; i < dl_iffted_data_buffer_.data.size(); i++)
        dl_iffted_data_buffer_.data[i].resize(UE_NUM * OFDM_CA_NUM);

    // initialize downlink precoded data buffer
    dl_precoded_data_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for (int i = 0; i < dl_precoded_data_buffer_.data.size(); i++)
        dl_precoded_data_buffer_.data[i].resize(BS_ANT_NUM * OFDM_CA_NUM);
    
    // initialize downlink modulated data buffer
    dl_modulated_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM * UE_NUM);
    for (int i = 0; i < dl_modulated_buffer_.data.size(); i++)
        dl_modulated_buffer_.data[i].resize(OFDM_CA_NUM);

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        dl_spm_buffer[i].resize(UE_NUM);

    // initialize downlink socket buffer
    dl_socket_buffer_.buffer.resize(subframe_num_perframe * SOCKET_BUFFER_FRAME_NUM 
                    * PackageReceiver::package_length * BS_ANT_NUM); // buffer SOCKET_BUFFER_FRAME_NUM entire frame
    dl_socket_buffer_.buffer_status.resize(subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM);


    memset(precode_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(ifft_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 

    for (int i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
        memset(precode_checker_[i], 0, sizeof(int) * data_subframe_num_perframe);
        memset(ifft_checker_[i], 0, sizeof(int) * data_subframe_num_perframe);
    }

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

void CoMP::start()
{
    // if ENABLE_CPU_ATTACH, attach main thread to core 0
#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(0) != 0) {
        perror("Main thread: stitch main thread to core 0 failed");
        exit(0);
    }
    else {
        printf("Main thread: stitch main thread to core 0 succeeded\n");
    }
#endif
    // start uplink receiver
    // creare socket buffer and socket threads
    char *socket_buffer_ptrs[SOCKET_RX_THREAD_NUM];
    int *socket_buffer_status_ptrs[SOCKET_RX_THREAD_NUM];
    for(int i = 0; i < SOCKET_RX_THREAD_NUM; i++) {
        socket_buffer_ptrs[i] = socket_buffer_[i].buffer.data();
        socket_buffer_status_ptrs[i] = socket_buffer_[i].buffer_status.data();
    }
    std::vector<pthread_t> rx_threads = receiver_->startRecv(socket_buffer_ptrs, 
        socket_buffer_status_ptrs, socket_buffer_[0].buffer_status.size(), socket_buffer_[0].buffer.size(), 1);

    // start downlink transmitter
#ifdef ENABLE_DOWNLINK
    char *dl_socket_buffer_ptr = dl_socket_buffer_.buffer.data();
    int *dl_socket_buffer_status_ptr = dl_socket_buffer_.buffer_status.data();
    double *dl_data_ptr = (double *)dl_precoded_data_buffer_.data.data();
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
    moodycamel::ProducerToken ptok_modul(modulation_queue_);
    moodycamel::ProducerToken ptok_precode(precode_queue_);
    moodycamel::ProducerToken ptok_tx(tx_queue_);
    // for message_queue, main thread is a comsumer, it is multiple producers
    // & single consumer for message_queue
    moodycamel::ConsumerToken ctok(message_queue_);

    // counter for print log
    int demul_count = 0;
    auto demul_begin = std::chrono::system_clock::now();
    int miss_count = 0;
    int total_count = 0;
    int frame_count = 0;
    int tx_count = 0;


    Event_data events_list[dequeue_bulk_size];
    int ret = 0;
    while(true) {
        // get a bulk of events
        ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size);
        total_count++;
        if(total_count == 1e7) {
            // print the message_queue_ miss rate is needed
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
            // if EVENT_PACKAGE_RECEIVED, do crop
            case EVENT_PACKAGE_RECEIVED: {
            
                    int offset = event.data;
                    Event_data do_crop_task;
                    do_crop_task.event_type = TASK_CROP;
                    do_crop_task.data = offset;

#ifdef DEBUG_PRINT_ENTER_QUEUE_FFT 
                    int buffer_frame_num = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
                    int buffer_id = offset / buffer_frame_num;
                    offset = offset - buffer_id * buffer_frame_num;
                    // read info of one frame
                    char* cur_ptr_buffer = socket_buffer_[buffer_id].buffer.data() + offset * PackageReceiver::package_length;
                    int ant_id, frame_id, subframe_id, cell_id;
                    frame_id = *((int *)cur_ptr_buffer);
                    subframe_id = *((int *)cur_ptr_buffer + 1);
                    cell_id = *((int *)cur_ptr_buffer + 2);
                    ant_id = *((int *)cur_ptr_buffer + 3);
                    int cropper_created_checker_id = (frame_id % TASK_BUFFER_FRAME_NUM) * subframe_num_perframe + subframe_id;
                    cropper_created_checker_[cropper_created_checker_id] ++;
                    // printf("Main thread: created FFT tasks for all ants in frame: %d, frame buffer: %d, subframe: %d, ant: %d\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
                    if (cropper_created_checker_[cropper_created_checker_id] == BS_ANT_NUM) {
                        printf("Main thread: created FFT tasks for all ants in frame: %d, frame buffer: %d, subframe: %d\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, subframe_id);
                        cropper_created_checker_[cropper_created_checker_id] = 0;
                    }
#endif
                    schedule_task(do_crop_task, &task_queue_, ptok);
                }                
                break;
                
            
            case EVENT_CROPPED: {
            
                    // if EVENT_CROPPED, check if all antennas are ready, do ZF if
                    // pilot, do deMul if data
                    int FFT_buffer_target_id = event.data;
                    // get frame_id & subframe_id
                    int frame_id, subframe_id, ant_id;
                    splitSubframeBufferIndex(FFT_buffer_target_id, &frame_id, &subframe_id);

                    // checker to count # of antennas in a subframe
                    // check if all antennas in this subframe is ready
                    int cropper_checker_id = frame_id * subframe_num_perframe + subframe_id;
                    cropper_checker_[cropper_checker_id] ++;

                    // if FFT for all anetnnas in a subframe is done, schedule ZF or equalization+demodulation
                    if (cropper_checker_[cropper_checker_id] == BS_ANT_NUM) {
#ifdef DEBUG_PRINT_TASK_DONE                    
                        printf("Main thread: finished FFT for frame: %d, subframe: %d\n", frame_id, subframe_id);
#endif
                        // Reset checker value
                        cropper_checker_[cropper_checker_id] = 0;

                        // if this subframe is pilot part
                        if (isPilot(subframe_id)) {   
                            
                            int csi_checker_id = frame_id;
#ifdef DEBUG_PRINT
                            printf("Main thread: pilot frame id: %d, subframe_id: %d\n", csi_checker_id,subframe_id);
#endif
                            // checker to count # of pilots/users
                            csi_checker_[csi_checker_id] ++;

                            // if csi of all UEs is ready, schedule ZF or prediction 
                            if (csi_checker_[csi_checker_id] == UE_NUM) {
                                csi_checker_[csi_checker_id] = 0; 
#ifdef DEBUG_PRINT_SUMMARY
                                printf("Main thread: pilot frame: %d, finished collecting pilot frames\n", csi_checker_id);
#endif
                                // count # of frame with CSI estimation
                                frame_count ++;

                                // schedule ZF when traning data is not enough or prediction is not enabled
                                if (frame_count<=INIT_FRAME_NUM || !DO_PREDICTION) {  
                                    // schedule normal ZF                                                                                        
                                    Event_data do_zf_task;
                                    do_zf_task.event_type = TASK_ZF;
#ifdef DEBUG_PRINT_ENTER_QUEUE
                                    printf("Main thread: created ZF tasks for frame: %d\n", frame_id);
#endif
                                    // add ZF tasks for each sub-carrier
                                    for (int i = 0; i < OFDM_CA_NUM; i++) {
                                        int csi_offset_id = frame_id * OFDM_CA_NUM + i;
                                        do_zf_task.data = csi_offset_id;
                                        schedule_task(do_zf_task, &zf_queue_, ptok_zf);
                                    }
                                }
                                // schedule prediction when traning data is enough and prediction is enabled
                                // when frame count equals to INIT_FRAME_NUM, do both prediction and ZF
                                if (frame_count>=INIT_FRAME_NUM && DO_PREDICTION) {                              
                                    Event_data do_pred_task;
                                    do_pred_task.event_type = TASK_PRED;
#ifdef DEBUG_PRINT_ENTER_QUEUE
                                    printf("Main thread: created Prediction tasks for frame: %d\n", frame_id);
#endif
                                    // add prediction tasks for each sub-carrier
                                    for(int i = 0; i < OFDM_CA_NUM; i++) {
                                        int csi_offset_id = frame_id * OFDM_CA_NUM + i;
                                        do_pred_task.data = csi_offset_id;
                                        schedule_task(do_pred_task, &zf_queue_, ptok_zf);
                                    }
                                }

                                // reset frame_count to avoid overflow
                                if (frame_count==MAX_FRAME_ID) 
                                    frame_count=0;
                            
                            // if downlink data transmission is enabled, schedule downlink modulation for all data subframes
#ifdef ENABLE_DOWNLINK
                                Event_data do_ifft_task;
                                do_ifft_task.event_type = TASK_IFFT;

                                for (int i = 0; i < data_subframe_num_perframe; i++) {
                                    for (int j = 0; j < UE_NUM; j++) {
                                        int ifft_offset_id = frame_id * UE_NUM * data_subframe_num_perframe
                                            + i * UE_NUM + j;
                                        do_ifft_task.data = ifft_offset_id;
                                        schedule_task(do_ifft_task, &ifft_queue_, ptok_ifft);
                                    }
                                }
#endif
                            }
                        }
                        else if (isData(subframe_id)) {
                            // if this subframe is data part
                            int data_checker_id = frame_id;
                            int data_subframe_id = subframe_id-UE_NUM;
                            Event_data do_demul_task;
                            do_demul_task.event_type = TASK_DEMUL;

#ifdef DEBUG_PRINT_ENTER_QUEUE_DEMUL
                            printf("Main thread: created Demodulation task for frame: %d, subframe: %d\n", data_checker_id,subframe_id);
#endif 
                            data_checker_[data_checker_id] ++;

                            // schedule demodulation task for subcarrier blocks
                            for(int i = 0; i < OFDM_CA_NUM / demul_block_size; i++) {
                                int demul_offset_id = frame_id * OFDM_CA_NUM * data_subframe_num_perframe
                                    + data_subframe_id * OFDM_CA_NUM + i * demul_block_size;
                                do_demul_task.data = demul_offset_id;
                                schedule_task(do_demul_task, &demul_queue_, ptok_demul);
                            }

                            if (data_checker_[data_checker_id] == data_subframe_num_perframe) {   
                                // do nothing...                         
                                if (DEBUG_PRINT_SUMMARY)
                                    printf("Main thread: frame: %d, finished collecting data frames\n", frame_id);
                                data_checker_[data_checker_id] = 0;
                            }     
                        }
                    }
                }
                break;
            
            case EVENT_ZF: {
                    // if ZF is ready 
                    int offset_zf = event.data;
                    // precoder is ready, do demodulation
                    int sc_idx = offset_zf % OFDM_CA_NUM;
                    int frame_id = (offset_zf - sc_idx) / OFDM_CA_NUM;

                    precoder_checker_[frame_id] ++;
                    if (precoder_checker_[frame_id] == OFDM_CA_NUM) {
                        if (DEBUG_PRINT_TASK_DONE || DEBUG_PRINT_SUMMARY) 
                            printf("Main thread: ZF done frame: %d\n", frame_id);
                        precoder_checker_[frame_id] = 0;
                        //TODO: this flag can be used to optimize deDemul logic
                        precoder_status_[frame_id] = true;
                    }
                }
                break;

            case EVENT_DEMUL: {
                    // do nothing
                    int offset_demul = event.data;
                    int sc_idx = offset_demul % OFDM_CA_NUM;
                    int total_data_subframe_idx = (offset_demul - sc_idx) / OFDM_CA_NUM;
                    int data_subframe_id = total_data_subframe_idx % (subframe_num_perframe - UE_NUM);
                    int frame_id = (total_data_subframe_idx - data_subframe_id) / (subframe_num_perframe - UE_NUM);
                    
                    demul_checker_[frame_id][data_subframe_id] += demul_block_size;
                    // if this subframe is ready
                    if (demul_checker_[frame_id][data_subframe_id] == OFDM_CA_NUM) {
#ifdef DEBUG_PRINT_TASK_DONE
                        printf("Main thread: Demodulation done frame: %d, subframe: %d\n", frame_id, data_subframe_id);
#endif
                        demul_checker_[frame_id][data_subframe_id] = 0;
                        demul_status_[frame_id]++;
                        if (demul_status_[frame_id]==data_subframe_num_perframe) {
#ifdef DEBUG_PRINT_SUMMARY
                            printf("Main thread: Demodulation done frame: %d \n", frame_id);
#endif
                            demul_status_[frame_id] = 0;
                        }

#ifdef WRITE_DEMUL
                        FILE* fp = fopen("demul_data.txt","a");
                        for (int cc = 0; cc < OFDM_CA_NUM; cc++)
                        {
                            long long* cx = &demul_buffer_.data[total_data_subframe_idx][cc * UE_NUM];
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
                            int samples_num_per_UE = OFDM_CA_NUM * data_subframe_num_perframe * 100;
                            printf("Receive %d samples (per-client) from %d clients in %f secs, throughtput %f bps per-client (16QAM), current task queue length %d\n", 
                                samples_num_per_UE, UE_NUM, diff.count(), samples_num_per_UE * log2(16.0f) / diff.count(), task_queue_.size_approx());
                            demul_begin = std::chrono::system_clock::now();
                        }
                    }
                }
                break;

            case EVENT_IFFT: {
                    // IFFT is done, schedule precoding when data for all users is ready
                    int offset_ifft = event.data;
                    int user_idx = offset_ifft % UE_NUM;
                    int total_data_subframe_idx = offset_ifft / UE_NUM;
                    int current_data_subframe_idx = total_data_subframe_idx % data_subframe_num_perframe;
                    int frame_idx = total_data_subframe_idx / data_subframe_num_perframe;

                    ifft_checker_[frame_idx][current_data_subframe_idx] += 1;
                    // if all users of this subframe is ready
                    if (ifft_checker_[frame_idx][current_data_subframe_idx] == UE_NUM) {
                        Event_data do_precode_task;
                        do_precode_task.event_type = TASK_PRECODE;
                        // add ZF tasks for each subcarrier block
                        for (int i = 0; i < OFDM_CA_NUM / demul_block_size; i++) {
                            int precode_offset_id = frame_idx * OFDM_CA_NUM * data_subframe_num_perframe
                                    + current_data_subframe_idx * OFDM_CA_NUM + i * demul_block_size;
                            do_precode_task.data = precode_offset_id;
                            schedule_task(do_precode_task, &precode_queue_, ptok_precode);
                        }
                    } 
                }
                break; 

            case EVENT_PRECODE: {
                    // Precoding is done, schedule data transmission 
                    int offset_precode = event.data;
                    int sc_idx = offset_precode % OFDM_CA_NUM;
                    int total_data_subframe_idx = offset_precode / OFDM_CA_NUM;
                    int current_data_subframe_idx = total_data_subframe_idx % data_subframe_num_perframe;
                    int frame_idx = total_data_subframe_idx / data_subframe_num_perframe;

                    precode_checker_[frame_idx][current_data_subframe_idx] += demul_block_size;
                    Event_data do_tx_task;
                    do_tx_task.event_type = TASK_SEND;

                    if (precode_checker_[frame_idx][current_data_subframe_idx] == demul_block_size) {
                        // add tx task for each antenna
                        for (int i = 0; i < BS_ANT_NUM; i++) {
                            do_tx_task.data = frame_idx * BS_ANT_NUM * data_subframe_num_perframe 
                                + current_data_subframe_idx * BS_ANT_NUM + i;
                            schedule_task(do_tx_task, &tx_queue_, ptok_tx);
                        }
                    }
                }
                break;
            case EVENT_PACKAGE_SENT: {
                    // Data is sent
                    tx_count ++;
                }
                break;
            default:
                printf("Wrong event type in message queue!");
                exit(0);
            }
        }
    }
    
}

void* CoMP::taskThread(void* context)
{
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_ = &(obj_ptr->task_queue_);
    moodycamel::ConcurrentQueue<Event_data>* zf_queue_ = &(obj_ptr->zf_queue_);
    moodycamel::ConcurrentQueue<Event_data>* demul_queue_ = &(obj_ptr->demul_queue_);
    moodycamel::ConcurrentQueue<Event_data>* ifft_queue_ = &(obj_ptr->ifft_queue_);
    moodycamel::ConcurrentQueue<Event_data>* modulation_queue_ = &(obj_ptr->modulation_queue_);
    moodycamel::ConcurrentQueue<Event_data>* precode_queue_ = &(obj_ptr->precode_queue_);
    moodycamel::ConcurrentQueue<Event_data>* tx_queue_ = &(obj_ptr->tx_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);
    
    // attach task threads to specific cores
    // Note: cores 0-17, 36-53 are on the same socket
#ifdef ENABLE_CPU_ATTACH
    int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + 1;
    int tar_core_id = tid + offset_id;
    if(tar_core_id >= 18)
        tar_core_id = (tar_core_id - 18) + 36;
    if(stick_this_thread_to_core(tar_core_id) != 0)
    {
        printf("Task thread: stitch thread %d to core %d failed\n", tid, tar_core_id);
        exit(0);
    }
    else 
    {
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
    while(true)
    {

        // ret_zf = false;
        // ret_demul = false;
        // ret = false;

        ret_zf = zf_queue_->try_dequeue(event);
        if(!ret_zf) 
        {
            ret_demul = demul_queue_->try_dequeue(event);
            if (!ret_demul)
            {   
                ret = task_queue_->try_dequeue(event);
                if (!ret)
                {
                    continue;
                }
                else if (event.event_type == TASK_CROP)
                {
                    // printf("ZF: %d, Demul: %d, FFT: %d\n", ret_zf,ret_demul,ret);
                    obj_ptr->doCrop(tid, event.data);
                }
                else 
                {
                    printf("Event type error\n");
                    exit(0);
                }
            }
            else if (event.event_type == TASK_DEMUL) // TODO: add precoder status check
            {
                obj_ptr->doDemul(tid, event.data);
            }
            else
            {
                printf("Event type error\n");
                exit(0);
            }
        }
        else if (event.event_type == TASK_ZF)
        {
            obj_ptr->doZF(tid, event.data);
        }
        else if (event.event_type == TASK_PRED)
        {
            obj_ptr->doPred(tid, event.data);
        }
        else
        {
            printf("Event type error\n");
            exit(0);
        }
        
    }
}


inline int CoMP::getFFTBufferIndex(int frame_id, int subframe_id, int ant_id) {
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * (BS_ANT_NUM * subframe_num_perframe) + subframe_id * BS_ANT_NUM + ant_id;
}

inline int CoMP::getSubframeBufferIndex(int frame_id, int subframe_id)
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * subframe_num_perframe + subframe_id;
}

inline void CoMP::splitSubframeBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id)
{
    (*frame_id) = FFT_buffer_target_id / subframe_num_perframe;
    (*subframe_id) = FFT_buffer_target_id - (*frame_id) * subframe_num_perframe;
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
    // cout << "x:" << endl;
    // cout << x.st() << endl;
    // cout <<imag(x).st() << endl;
    // cout << "Re:" << re.st() << endl;
    return re;
}


// TODO: test correctness
inline cx_fmat CoMP::mod_16qam(imat x)
{
    cx_fmat re;
    fmat real_re = conv_to<fmat>::from(x);
    fmat imag_re = conv_to<fmat>::from(x);
    // float scale = 1/sqrt(10);
    // float modvec_16qam[4]  = {-3*scale, -1*scale, 3*scale, scale};

    // real_re.for_each([&modvec_16qam](fmat::elem_type& val) { val = modvec_16qam[(int)val/4]; } );
    // imag_re.for_each([&modvec_16qam](fmat::elem_type& val) { val = modvec_16qam[(int)val%4]; } );
    real_re.for_each([this](fmat::elem_type& val) { val = qam16_table[0][(int)val]; } );
    imag_re.for_each([this](fmat::elem_type& val) { val = qam16_table[1][(int)val]; } );
    re.set_real(real_re);
    re.set_imag(imag_re);

    return re;
}

/*****************************************************
 * Uplink tasks
 *****************************************************/

void CoMP::doCrop(int tid, int offset)
{
    int buffer_subframe_num = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    int socket_thread_id = offset / buffer_subframe_num;
    offset = offset - socket_thread_id * buffer_subframe_num;
    // read info of one frame
    char *cur_ptr_buffer = socket_buffer_[socket_thread_id].buffer.data() + offset * PackageReceiver::package_length;

    int ant_id, frame_id, subframe_id, cell_id;
    frame_id = *((int *)cur_ptr_buffer);
    subframe_id = *((int *)cur_ptr_buffer + 1);
    cell_id = *((int *)cur_ptr_buffer + 2);
    ant_id = *((int *)cur_ptr_buffer + 3);
    //printf("thread %d process frame_id %d, subframe_id %d, cell_id %d, ant_id %d\n", tid, frame_id, subframe_id, cell_id, ant_id);
    // remove CP, do FFT
    int delay_offset = 0;
    int FFT_buffer_target_id = getFFTBufferIndex(frame_id, subframe_id, ant_id);

    // transfer ushort to float
    ushort *cur_ptr_buffer_ushort = (ushort *)(cur_ptr_buffer + sizeof(int) * 4);
    float *cur_fft_buffer_float = (float *)fft_buffer_.FFT_inputs[FFT_buffer_target_id];
    // Copy data for fft from socket buffer to fft buffer
    for(int i = 0; i < (OFDM_CA_NUM - delay_offset) * 2; i++)
        cur_fft_buffer_float[i] = (cur_ptr_buffer_ushort[OFDM_PREFIX_LEN + delay_offset + i] / 65536.f - 0.5f) * 4.f;

    // append zero
    if(delay_offset > 0) 
        memset((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id] 
            + (OFDM_CA_NUM - delay_offset) * 2 * sizeof(float), 0, sizeof(float) * 2 * delay_offset);

    // perform fft
    mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[FFT_buffer_target_id], 
        fft_buffer_.FFT_inputs[FFT_buffer_target_id]);

#ifdef DEBUG_PRINT_IN_TASK
        printf("In doCrop thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
#endif
    // if it is pilot part, do CE
    if(isPilot(subframe_id)) {
        int UE_id = subframe_id;
        int ca_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * OFDM_CA_NUM;
        int csi_offset = ant_id + UE_id * BS_ANT_NUM;

        float* cur_fft_buffer_float_output = (float*)fft_buffer_.FFT_outputs[FFT_buffer_target_id];



        for(int j = 0; j < (OFDM_CA_NUM); j++) {
            // divide fft output by pilot data to get CSI estimation
            float* csi_buffer_ptr = (float*)(csi_buffer_.CSI[ca_offset+j].data())+csi_offset*2;
            *(csi_buffer_ptr)= cur_fft_buffer_float_output[2*j]* pilots_[j];
            *(csi_buffer_ptr+1)= cur_fft_buffer_float_output[2*j+1]* pilots_[j];           
        }     
    }
    else if(isData(subframe_id)) {
        
        int data_subframe_id = subframe_id - UE_NUM;
        int frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * data_subframe_num_perframe + data_subframe_id;
        
        /* //naive transpose
        for(int j = 0; j < OFDM_CA_NUM; j++)
        {
            data_buffer_.data[frame_offset][ant_id + j * BS_ANT_NUM] = fft_buffer_.FFT_outputs[FFT_buffer_target_id][j];
        }
        */
        // block transpose
        // src_ptr: point to the start of subframe (subframe size: OFDM_CA_NUM) 
        // 2048 float values
        float *src_ptr = (float *)&fft_buffer_.FFT_outputs[FFT_buffer_target_id][0];

        // tar_ptr: point to the start of subframe with size BS_ANT_NUM * OFDM_CA_NUM
        // 96*1024*2 float values
        float *tar_ptr = (float *)&data_buffer_.data[frame_offset][0];
        // copy data from fft_outputs to data_buffer
        // 1024*2/8 = 256 iterations, copy 8 bytes every time
        // c2 = 0, 1, ..., 1024/64*2-1 = 31
        // c2*transpose_block_size = 0, 64, 128, ..., 2048-64
        for(int c2 = 0; c2 < OFDM_CA_NUM / transpose_block_size * 2; c2++) {
            // c3 = 0, 1, ..., transpose_block_size/8 -1 = 7
            // c3*8 = 0, 8, ..., 64-8
            for(int c3 = 0; c3 < transpose_block_size / 8; c3++) {
                // data: 256 bits = 32 bytes = 8 float values = 4 subcarriers
                __m256 data = _mm256_load_ps(src_ptr + c2 * transpose_block_size + c3 * 8);
                // original data order: SCs of ant1, SCs of ant2, ..., SCs of ant 96
                // transposed data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
                _mm256_store_ps(tar_ptr + c2 * BS_ANT_NUM * transpose_block_size + transpose_block_size * ant_id + c3 * 8, data);
            }            
        }        
    }


    // after finish
    socket_buffer_[socket_thread_id].buffer_status[offset] = 0; // now empty
    // inform main thread
    Event_data crop_finish_event;
    crop_finish_event.event_type = EVENT_CROPPED;
    crop_finish_event.data = getSubframeBufferIndex(frame_id, subframe_id);

    if ( !message_queue_.enqueue(*task_ptok[tid], crop_finish_event ) ) {
        printf("crop message enqueue failed\n");
        exit(0);
    }
}


void CoMP::doZF(int tid, int offset)
{

    int sc_idx = offset % OFDM_CA_NUM;
    int frame_id = (offset - sc_idx) / OFDM_CA_NUM;
    if (DEBUG_PRINT_IN_TASK)
        printf("In doZF thread %d: frame: %d, subcarrier: %d\n", tid, frame_id, sc_idx);

    cx_float *ptr_in = (cx_float *)csi_buffer_.CSI[offset].data();
    cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
    cx_float *ptr_out = (cx_float *)precoder_buffer_.precoder[offset].data();
    cx_fmat mat_output(ptr_out, UE_NUM, BS_ANT_NUM, false);
    
    pinv(mat_output, mat_input, 1e-1, "dc");


    // inform main thread
    Event_data ZF_finish_event;
    ZF_finish_event.event_type = EVENT_ZF;
    ZF_finish_event.data = offset;

    if ( !message_queue_.enqueue(*task_ptok[tid], ZF_finish_event ) ) {
        printf("ZF message enqueue failed\n");
        exit(0);
    }
}


void CoMP::doPred(int tid, int offset) 
{
    int sc_idx = offset % OFDM_CA_NUM;
    int frame_id = (offset - sc_idx) / OFDM_CA_NUM;
    int offset_next_frame = ((frame_id+1)%TASK_BUFFER_FRAME_NUM)*OFDM_CA_NUM+sc_idx;
    // Use stale CSI as predicted CSI
    // TODO: add prediction algorithm
    cx_float *ptr_in = (cx_float *)pred_csi_buffer_.CSI[sc_idx].data();
    memcpy(ptr_in, (cx_float *)csi_buffer_.CSI[offset].data(), sizeof(cx_float)*BS_ANT_NUM*UE_NUM);
    cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
    cx_float *ptr_out = (cx_float *)precoder_buffer_.precoder[offset_next_frame].data();
    cx_fmat mat_output(ptr_out, UE_NUM, BS_ANT_NUM, false);
    pinv(mat_output, mat_input, 1e-1, "dc");

    // inform main thread
    Event_data pred_finish_event;
    pred_finish_event.event_type = EVENT_ZF;
    pred_finish_event.data = offset_next_frame;

    if ( !message_queue_.enqueue(*task_ptok[tid], pred_finish_event ) ) {
        printf("Prediction message enqueue failed\n");
        exit(0);
    }
}


void CoMP::doDemul(int tid, int offset)
{
    // subcarrier index
    int sc_idx = offset % OFDM_CA_NUM;
    // total data subframe index
    int total_data_subframe_idx = (offset - sc_idx) / OFDM_CA_NUM;
    // data subframe index in the current frame
    int data_subframe_idx = total_data_subframe_idx % data_subframe_num_perframe;
    // frame index
    int frame_idx = total_data_subframe_idx  / data_subframe_num_perframe;

#ifdef DEBUG_PRINT_IN_TASK
        printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_idx,data_subframe_idx,sc_idx);
#endif

    // index=[0,32,64,96]
    __m256i index = _mm256_setr_epi64x(0, transpose_block_size/2, transpose_block_size/2 * 2, transpose_block_size/2 * 3);

    float* tar_ptr = (float *)&data_buffer_.data[total_data_subframe_idx][0];
    float* temp_buffer_ptr = (float *)&spm_buffer[tid][0];
    // i = 0, 1, ..., 31
    for (int i = 0; i < demul_block_size; i++) {
        int c1_base = (sc_idx + i) * 2 / transpose_block_size;
        int c1_offset = (sc_idx + i) * 2 % transpose_block_size;
        /**
         * block-wisely copy data of all antennas of the i-th subcarrier from data_buffer to spm_buffer
         * copy data of 4 ants of the i-th subcarrier in each iteration 
         */
        for (int c2 = 0; c2 < BS_ANT_NUM / 4; c2++) {
            float *base_ptr = tar_ptr + c1_base * transpose_block_size * BS_ANT_NUM + c1_offset + c2 * transpose_block_size * 4;
            // t_data holds 4 double values (32 bytes) (subcarriers of 4 ants c2*4-c2*4+3)
            // t_data[0] = *(base_ptr+0*8)  data of i-th subcarrier of (c2*4+0)-th antenna
            // ...
            // t_data[3] = *(base_ptr+96*8)  data of i-th subcarrier of (c2*4+3)-th antenna
            __m256d t_data = _mm256_i64gather_pd((double *)base_ptr, index, 8);
            _mm256_store_pd((double *)(temp_buffer_ptr + c2 * 8), t_data);
        }

        // mat_precoder size: UE_NUM \times BS_ANT_NUM
        int precoder_offset = frame_idx * OFDM_CA_NUM + sc_idx + i;
        cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset].data();
        cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

        // mat_data size: BS_ANT_NUM \times 1
        cx_float* data_ptr = (cx_float *)(&spm_buffer[tid][0]);
        cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);

        // mat_demuled size: UE_NUM \times 1
        cx_float* equal_ptr = (cx_float *)(&equal_buffer_.data[total_data_subframe_idx][(sc_idx + i) * UE_NUM]);
        cx_fmat mat_equaled(equal_ptr, UE_NUM, 1, false);

        // Equalization
        mat_equaled = mat_precoder * mat_data;

        // Demodulation
        sword* demul_ptr = (sword *)(&demul_buffer_.data[total_data_subframe_idx][(sc_idx + i) * UE_NUM]);
        imat mat_demuled(demul_ptr, UE_NUM, 1, false);
        
        mat_demuled = demod_16qam(mat_equaled);
        // cout << "Frame: "<< frame_id<<", subframe: "<< data_subframe_id<<", SC: " << sc_idx+i << ", data: " << mat_demuled2.st() << endl;

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
    
    if ( !message_queue_.enqueue(*task_ptok[tid], demul_finish_event ) ) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }    
}


/*****************************************************
 * Downlink tasks
 *****************************************************/

void CoMP::do_ifft(int tid, int offset)
{
    int user_idx = offset % UE_NUM;
    int total_data_subframe_idx = offset / UE_NUM;
    int current_data_subframe_idx = total_data_subframe_idx % data_subframe_num_perframe;
    int frame_idx = total_data_subframe_idx / data_subframe_num_perframe;

    long long *modul_ptr = (long long *)(&dl_IQ_data_long[current_data_subframe_idx * UE_NUM + user_idx][0]);
    imat mat_rawdata(modul_ptr, OFDM_CA_NUM, 1, false);

    cx_float *ifft_ptr = (cx_float *)(&dl_ifft_buffer_.IFFT_inputs[offset][0]);
    cx_fmat mat_moduled(ifft_ptr, OFDM_CA_NUM, 1, false);  
    
    mat_moduled = mod_16qam(mat_rawdata);

    mufft_execute_plan_1d(muplans_ifft_[tid], dl_ifft_buffer_.IFFT_outputs[offset], 
        dl_ifft_buffer_.IFFT_inputs[offset]);


    int output_offset = frame_idx * data_subframe_num_perframe + current_data_subframe_idx;
    
    // block transpose
    float *ifft_output_ptr = (float *)(&dl_ifft_buffer_.IFFT_outputs[offset][0]);
    float *output_ptr = (float *)&dl_iffted_data_buffer_.data[output_offset][0];
    // copy data from ifft_outputs to iffted_data_buffer
    for (int c2 = 0; c2 < OFDM_CA_NUM / transpose_block_size * 2; c2++) {
        for (int c3 = 0; c3 < transpose_block_size / 8; c3++) {
            // copy 4 subcarriers (32 bytes) at a time
            __m256 data = _mm256_load_ps(ifft_output_ptr + c2 * transpose_block_size + c3 * 8);
            _mm256_store_ps(output_ptr + c2 * UE_NUM * transpose_block_size + transpose_block_size * user_idx + c3 * 8, data);
        }            
    }        

    // inform main thread
    Event_data ifft_finish_event;
    ifft_finish_event.event_type = EVENT_IFFT;
    ifft_finish_event.data = offset;

    if ( !message_queue_.enqueue(*task_ptok[tid], ifft_finish_event ) ) {
        printf("IFFT message enqueue failed\n");
        exit(0);
    }
}


void CoMP::do_precode(int tid, int offset) 
{
    int sc_idx = offset % OFDM_CA_NUM;
    int total_data_subframe_idx = offset / OFDM_CA_NUM;
    int current_data_subframe_idx = total_data_subframe_idx % data_subframe_num_perframe;
    int frame_idx = total_data_subframe_idx / data_subframe_num_perframe;

    // index=[0,32,64,96]
    __m256i index = _mm256_setr_epi64x(0, transpose_block_size/2, transpose_block_size/2 * 2, transpose_block_size/2 * 3);

    float* input_ptr = (float *)&dl_iffted_data_buffer_.data[total_data_subframe_idx][0];
    float* helpler_buffer_ptr = (float *)&dl_spm_buffer[tid][0];

    for (int i = 0; i < demul_block_size; i++) {
        /**
         * block-wise copy for all UEs of the i-th subcarrier
         * move data store in data_buffer to spm_buffer
         * copy data of 4 UEs of the i-th subcarrier in each iteration 
         */
        for (int c2 = 0; c2 < UE_NUM / 4; c2++) {
            float *input_shifted_ptr = input_ptr + sc_idx * UE_NUM + i + c2 * transpose_block_size * 4;
            __m256d t_data = _mm256_i64gather_pd((double *)input_shifted_ptr, index, 8);
            _mm256_store_pd((double *)(helpler_buffer_ptr + c2 * 8), t_data);
        }

        // TODO: speed up this?
        for (int c2 = UE_NUM - UE_NUM % 4; c2 < UE_NUM; c2++) 
            dl_spm_buffer[tid][c2] = dl_iffted_data_buffer_.data[total_data_subframe_idx][sc_idx * UE_NUM + i + c2 * transpose_block_size];
        
        int precoder_offset = frame_idx * OFDM_CA_NUM + sc_idx + i;
        // mat_precoder size: UE_NUM \times BS_ANT_NUM        
        cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset].data();
        cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

        // mat_data size: UE_NUM \times 1
        cx_float* data_ptr = (cx_float *)(&dl_spm_buffer[tid][0]);
        cx_fmat mat_data(data_ptr, UE_NUM, 1, false);

        // mat_precoded size: BS_ANT_NUM \times 1
        cx_float* precoded_ptr = (cx_float *)(&dl_precoded_data_buffer_.data[total_data_subframe_idx][(sc_idx + i) * BS_ANT_NUM]);
        cx_fmat mat_precoded(precoded_ptr, BS_ANT_NUM, 1, false);

        mat_precoded = mat_precoder.t() * mat_data;

    }
    // inform main thread
    Event_data precode_finish_event;
    precode_finish_event.event_type = EVENT_PRECODE;
    precode_finish_event.data = offset;
    
    if ( !message_queue_.enqueue(*task_ptok[tid], precode_finish_event ) ) {
        printf("Precoding message enqueue failed\n");
        exit(0);
    }
}


// void CoMP::do_tx(int tid, int offset) 
// {
//     int ant_idx = offset % BS_ANT_NUM;
//     int total_data_subframe_idx = offset / BS_ANT_NUM;
//     int current_data_subframe_idx = total_data_subframe_idx % data_subframe_num_perframe;
//     int frame_idx = total_data_subframe_idx / data_subframe_num_perframe;


//     // inform main thread
//     Event_data finish_event;
//     finish_event.event_type = EVENT_PACKAGE_SENT;
//     finish_event.data = offset;
    
//     if ( !message_queue_.enqueue(*task_ptok[tid], finish_event ) ) {
//         printf("Transmit message enqueue failed\n");
//         exit(0);
//     }
// }






