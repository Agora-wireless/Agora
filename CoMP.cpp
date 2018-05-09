#include "CoMP.hpp"

using namespace arma;
typedef cx_float COMPLEX;

CoMP::CoMP()
{

    printf("enter constructor\n");
    // initialize socket buffer
    for(int i = 0; i < SOCKET_THREAD_NUM; i++)
    {
        socket_buffer_[i].buffer.resize(PackageReceiver::package_length 
            * subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM); // buffer SOCKET_BUFFER_FRAME_NUM entire frame
        socket_buffer_[i].buffer_status.resize(subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM);
    }
    printf("initialize buffers\n");

    // initialize FFT buffer
    int FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    fft_buffer_.FFT_inputs = new complex_float*[FFT_buffer_block_num];
    fft_buffer_.FFT_outputs = new complex_float*[FFT_buffer_block_num];
    for(int i = 0; i < FFT_buffer_block_num; i++)
    {
        fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
        fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    }
    // printf("FFT buffers initialized\n");
    // initialize muplans for fft
    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        muplans_[i] = mufft_create_plan_1d_c2c(OFDM_CA_NUM, MUFFT_FORWARD, MUFFT_FLAG_CPU_ANY);
    }
    // printf("Muplans initialized\n");

    // initialize CSI buffer
    csi_buffer_.CSI.resize(OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM);
    for(int i = 0; i < csi_buffer_.CSI.size(); i++)
        csi_buffer_.CSI[i].resize(BS_ANT_NUM * UE_NUM);
    // printf("CSI buffer initialized\n");

    // initialize data buffer
    data_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for(int i = 0; i < data_buffer_.data.size(); i++)
        data_buffer_.data[i].resize(BS_ANT_NUM * OFDM_CA_NUM);
    // printf("Data buffer initialized\n");

    // initialize precoder buffer
    precoder_buffer_.precoder.resize(OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM);
    for(int i = 0; i < precoder_buffer_.precoder.size(); i++)
        precoder_buffer_.precoder[i].resize(UE_NUM * BS_ANT_NUM);

    // initialize demultiplexed data buffer
    demul_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for(int i = 0; i < demul_buffer_.data.size(); i++)
        demul_buffer_.data[i].resize(OFDM_CA_NUM * UE_NUM);

    // initialize demultiplexed data buffer
    demul_buffer2_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    for(int i = 0; i < demul_buffer2_.data.size(); i++)
        demul_buffer2_.data[i].resize(OFDM_CA_NUM * UE_NUM);

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        spm_buffer[i].resize(BS_ANT_NUM);
    // printf("Demultiplexed data buffer initialized\n");

    // read pilots from file
    pilots_.resize(OFDM_CA_NUM);
    FILE* fp = fopen("../pilot_f.bin","rb");
    fread(pilots_.data(), sizeof(float), OFDM_CA_NUM, fp);
    fclose(fp);
    cout<<"Pilot data"<<endl;
    for (int i = 0; i<OFDM_CA_NUM;i++)
    {
        cout<<pilots_[i]<<",";
    }
    cout<<endl;
    // printf("Pilot data read\n");

    printf("new PackageReceiver\n");
    receiver_.reset(new PackageReceiver(SOCKET_THREAD_NUM, &message_queue_));

    // initilize all kinds of checkers
    memset(cropper_checker_, 0, sizeof(int) * subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    memset(csi_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(data_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(precoder_status_, 0, sizeof(bool) * TASK_BUFFER_FRAME_NUM); 
    for(int i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
    {
        memset(demul_checker_[i], 0, sizeof(int) * (subframe_num_perframe - UE_NUM));
    }

    // create task thread
    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        context[i].obj_ptr = this;
        context[i].id = i;
        //printf("create thread %d\n", i);
        if(pthread_create( &task_threads[i], NULL, CoMP::taskThread, &context[i]) != 0)
        {
            perror("task thread create failed");
            exit(0);
        }
    }

}

CoMP::~CoMP()
{
    for(int i = 0; i < TASK_THREAD_NUM; i++)
    {
        mufft_free_plan_1d(muplans_[i]);
    }
    // release FFT_buffer
    int FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    for(int i = 0; i < FFT_buffer_block_num; i++)
    {
        mufft_free(fft_buffer_.FFT_inputs[i]);
        mufft_free(fft_buffer_.FFT_outputs[i]);
    }
}

void CoMP::start()
{
    // if ENABLE_CPU_ATTACH, attach main thread to core 0
#ifdef ENABLE_CPU_ATTACH
    if(stick_this_thread_to_core(0) != 0)
    {
        perror("Main thread: stitch main thread to core 0 failed");
        exit(0);
    }
    else 
    {
        printf("Main thread: stitch main thread to core 0 succeeded\n");
    }
#endif
    // creare socket buffer and socket threads
    char* socket_buffer_ptrs[SOCKET_THREAD_NUM];
    int* socket_buffer_status_ptrs[SOCKET_THREAD_NUM];
    for(int i = 0; i < SOCKET_THREAD_NUM; i++)
    {
        socket_buffer_ptrs[i] = socket_buffer_[i].buffer.data();
        socket_buffer_status_ptrs[i] = socket_buffer_[i].buffer_status.data();
    }
    std::vector<pthread_t> recv_thread = receiver_->startRecv(socket_buffer_ptrs, 
        socket_buffer_status_ptrs, socket_buffer_[0].buffer_status.size(), socket_buffer_[0].buffer.size(), 1);
    // for task_queue, main thread is producer, it is single-procuder & multiple consumer
    // for task queue
    moodycamel::ProducerToken ptok(task_queue_);
    // for message_queue, main thread is a comsumer, it is multiple producers
    // & single consumer for message_queue
    moodycamel::ConsumerToken ctok(message_queue_);

    // counter for print log
    int demul_count = 0;
    auto demul_begin = std::chrono::system_clock::now();
    int miss_count = 0;
    int total_count = 0;

    Event_data events_list[dequeue_bulk_size];
    int ret = 0;
    while(true)
    {
        // get a bulk of events
        ret = message_queue_.try_dequeue_bulk(ctok, events_list, dequeue_bulk_size);
        total_count++;
        if(total_count == 1e7)
        {
            // print the message_queue_ miss rate is needed
            //printf("message dequeue miss rate %f\n", (float)miss_count / total_count);
            total_count = 0;
            miss_count = 0;
        }
        if(ret == 0)
        {
            miss_count++;
            continue;
        }
        // handle each event
        for(int bulk_count = 0; bulk_count < ret; bulk_count ++)
        {
            Event_data& event = events_list[bulk_count];

            // if EVENT_PACKAGE_RECEIVED, do crop
            if(event.event_type == EVENT_PACKAGE_RECEIVED)
            {
                int offset = event.data;
                Event_data do_crop_task;
                do_crop_task.event_type = TASK_CROP;
                do_crop_task.data = offset;
                if ( !task_queue_.try_enqueue(ptok, do_crop_task ) ) {
                    printf("need more memory\n");
                    if ( !task_queue_.enqueue(ptok, do_crop_task ) ) {
                        printf("crop task enqueue failed\n");
                        exit(0);
                    }
                }

                
            }
            else if(event.event_type == EVENT_CROPPED)
            {
                // if EVENT_CROPPED, check if all antennas are ready, do ZF if
                // pilot, do deMul if data
                int FFT_buffer_target_id = event.data;
                // get frame_id & subframe_id
                int frame_id, subframe_id, ant_id;
                splitSubframeBufferIndex(FFT_buffer_target_id, &frame_id, &subframe_id);
                // check if all antennas in this subframe is ready
                int cropper_checker_id = frame_id * subframe_num_perframe + subframe_id;
                cropper_checker_[cropper_checker_id] ++;

                if(cropper_checker_[cropper_checker_id] == BS_ANT_NUM) // this sub-frame is finished
                {
                    //printf("subframe %d, frame %d\n", subframe_id, frame_id);
                    
                    cropper_checker_[cropper_checker_id] = 0; // this subframe is finished
                    // if this subframe is pilot part
                    if(isPilot(subframe_id))
                    {   
                        // check if csi of all UEs are ready
                        int csi_checker_id = frame_id;
                        if (DEBUG_PRINT)
                            printf("Main thread: pilot frame id: %d, subframe_id: %d\n", csi_checker_id,subframe_id);
                        csi_checker_[csi_checker_id] ++;
                        if(csi_checker_[csi_checker_id] == UE_NUM)
                        {
                            if (DEBUG_PRINT)
                                printf("Main thread: pilot frame: %d, finished collecting pilot frames: %d\n", csi_checker_id,subframe_id);
                            // if CSI from all UEs are ready, do ZF
                            csi_checker_[csi_checker_id] = 0;
                            //if(frame_id == 4)
                            //    printf("Frame %d, csi ready, assign ZF\n", frame_id);
                            
                            Event_data do_ZF_task;
                            do_ZF_task.event_type = TASK_ZF;
                            // add ZF tasks for each sub-carrier
                            for(int i = 0; i < OFDM_CA_NUM; i++)
                            {
                                int csi_offset_id = frame_id * OFDM_CA_NUM + i;
                                do_ZF_task.data = csi_offset_id;
                                if ( !task_queue_.try_enqueue(ptok, do_ZF_task ) ) {
                                    printf("need more memory\n");
                                    if ( !task_queue_.enqueue(ptok, do_ZF_task ) ) {
                                        printf("ZF task enqueue failed\n");
                                        exit(0);
                                    }
                                }
                            }
                        }
                    }
                    else if(isData(subframe_id))
                    {
                        // if this subframe is data part
                        int data_checker_id = frame_id;
                        if (DEBUG_PRINT)
                            printf("Main thread: data frame id: %d, subframe_id: %d\n", data_checker_id,subframe_id);
                        data_checker_[data_checker_id] ++;
                        if(data_checker_[data_checker_id] == data_subframe_num_perframe)
                        {
                            // if all data part is ready. TODO: optimize the
                            // logic, do deMul when CSI is ready, do not wait
                            // the entire frame is ready
                            
                            //printf("do demul for frame %d\n", frame_id);
                            //printf("frame %d data received\n", frame_id);
                            if (DEBUG_PRINT)
                                printf("Main thread: data frame: %d, finished collecting data frames: %d\n", data_checker_id,subframe_id);
                            data_checker_[data_checker_id] = 0;
                            Event_data do_demul_task;
                            do_demul_task.event_type = TASK_DEMUL;
                            // add deMul tasks
                            for(int j = 0; j < data_subframe_num_perframe; j++)
                            {
                                for(int i = 0; i < OFDM_CA_NUM / demul_block_size; i++)
                                {
                                    int demul_offset_id = frame_id * OFDM_CA_NUM * data_subframe_num_perframe
                                        + j * OFDM_CA_NUM + i * demul_block_size;
                                    do_demul_task.data = demul_offset_id;
                                    if ( !task_queue_.try_enqueue(ptok, do_demul_task ) ) {
                                        printf("need more memory\n");
                                        if ( !task_queue_.enqueue(ptok, do_demul_task ) ) {
                                            printf("Demultiplexing task enqueue failed\n");
                                            exit(0);
                                        }
                                    }
                                }
                            }
                        }     
                    }
                }
            }
            else if(event.event_type == EVENT_ZF)
            {
                // if ZF is ready 
                int offset_zf = event.data;
                // precoder is ready, do demodulation
                int ca_id = offset_zf % OFDM_CA_NUM;
                int frame_id = (offset_zf - ca_id) / OFDM_CA_NUM;

                if (DEBUG_PRINT_TASK_DONE) 
                    printf("Main thread: ZF done frame: %d, subcarrier: %d\n", frame_id,ca_id);

                precoder_checker_[frame_id] ++;
                if(precoder_checker_[frame_id] == OFDM_CA_NUM)
                {
                    precoder_checker_[frame_id] = 0;
                    //TODO: this flag can be used to optimize deDemul logic
                    precoder_status_[frame_id] = true;
                }
            }
            else if(event.event_type == EVENT_DEMUL)
            {
                // do nothing
                int offset_demul = event.data;
                int ca_id = offset_demul % OFDM_CA_NUM;
                int offset_without_ca = (offset_demul - ca_id) / OFDM_CA_NUM;
                int data_subframe_id = offset_without_ca % (subframe_num_perframe - UE_NUM);
                int frame_id = (offset_without_ca - data_subframe_id) / (subframe_num_perframe - UE_NUM);

                if (DEBUG_PRINT_TASK_DONE) 
                    printf("Main thread: Demodulation done frame: %d, subframe: %d, subcarrier: %d\n", frame_id, data_subframe_id, ca_id);

                demul_checker_[frame_id][data_subframe_id] += demul_block_size;
                // if this subframe is ready
                if(demul_checker_[frame_id][data_subframe_id] == OFDM_CA_NUM)
                {
                    demul_checker_[frame_id][data_subframe_id] = 0;
                    /*
                    // debug
                    if(frame_id == 4 && data_subframe_id == 4)
                    {
                        printf("save demul data\n");
                        FILE* fp = fopen("ver_data.txt","w");
                        for(int cc = 0; cc < OFDM_CA_NUM; cc++)
                        {
                            complex_float* cx = &demul_buffer_.data[offset_without_ca][cc * UE_NUM];
                            for(int kk = 0; kk < UE_NUM; kk++)  
                                fprintf(fp, "%f %f\n", cx[kk].real, cx[kk].imag);
                        }
                        fclose(fp);
                        exit(0);
                    }
                    */

                    demul_count += 1;
                    // print log per 100 frames
                    if(demul_count == data_subframe_num_perframe * 100)
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

        }
    }

    
}

void* CoMP::taskThread(void* context)
{
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* task_queue_ = &(obj_ptr->task_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);
    
    // attach task threads to specific cores
    // Note: cores 0-17, 36-53 are on the same socket
#ifdef ENABLE_CPU_ATTACH
    int offset_id = SOCKET_THREAD_NUM + 1;
    int tar_core_id = tid + offset_id;
    // if(tar_core_id >= 18)
    //     tar_core_id = (tar_core_id - 18) + 36;
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
    while(true)
    {
        ret = task_queue_->try_dequeue(event);
        if(tid == 0)
            total_count++;
        if(tid == 0 && total_count == 1e6)
        {
            // print the task queue miss rate if required
            //printf("thread 0 task dequeue miss rate %f, queue length %d\n", (float)miss_count / total_count, task_queue_->size_approx());
            total_count = 0;
            miss_count = 0;
        }
        if(!ret)
        {
            if(tid == 0)
                miss_count++;
            continue;
        }

        // do different tasks according to task type
        if(event.event_type == TASK_CROP)
        {   
            obj_ptr->doCrop(tid, event.data);
        }
        else if(event.event_type == TASK_ZF)
        {
            obj_ptr->doZF(tid, event.data);
        }
        else if(event.event_type == TASK_DEMUL)
        {
            obj_ptr->doDemul(tid, event.data);
        }
        
    }
}

inline int CoMP::getFFTBufferIndex(int frame_id, int subframe_id, int ant_id)
{
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


inline imat CoMP::demod_16qam(arma::cx_fmat x)
{
    imat re;
    mat zero_mat = zeros<mat>(size(x));
    mat float_mat = 0.6325*ones<mat>(size(x));

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

void CoMP::doDemul(int tid, int offset)
{
    // demodulation for 1 block (32 subcarriers)

    // get information 
    // offset = frame_id * OFDM_CA_NUM * data_subframe_num_perframe+ j * OFDM_CA_NUM + n * demul_block_size;
    // n = 0, 1, ..., OFDM_CA_NUM / demul_block_size-1 = 31
    // j = 0, 1, ..., data_subframe_num_perframe-1 = 39

    // subcarrier index
    // ca_id = n*demul_block_size
    int ca_id = offset % OFDM_CA_NUM;
    // subframe index
    // offset_without_ca = frame_id*data_subframe_num_perframe+j
    int offset_without_ca = (offset - ca_id) / OFDM_CA_NUM;
    // data_subframe_id = j
    int data_subframe_id = offset_without_ca % (subframe_num_perframe - UE_NUM);
    // frame index
    // frame_id = frame_id
    int frame_id = (offset_without_ca - data_subframe_id) / (subframe_num_perframe - UE_NUM);

    if (DEBUG_PRINT)
        printf("In doDemul: frame: %d, subframe: %d, subcarrier: %d \n", frame_id,data_subframe_id,ca_id);

    //printf("do demul, %d %d %d\n", frame_id, data_subframe_id, ca_id);
    // see the slides for more details
    // index=[0,32,64,96]
    __m256i index = _mm256_setr_epi64x(0, transpose_block_size/2, transpose_block_size/2 * 2, transpose_block_size/2 * 3);
    // int64_t * index_int = (int64_t *) &index;
    // printf("In doDemul: index: %d %d %d %d\n", index_int[0],index_int[1],index_int[2],index_int[3]);

    // tar_ptr: point to the start of subframe with size BS_ANT_NUM * OFDM_CA_NUM
    // 96*1024*2 float values
    float* tar_ptr = (float *)&data_buffer_.data[offset_without_ca][0];
    // temp_buffer_ptr: point to the start of a buffer with size BS_ANT_NUM
    float* temp_buffer_ptr = (float *)&spm_buffer[tid][0];
    // i = 0, 1, ..., 31
    for(int i = 0; i < demul_block_size; i++)
    {
        // c2 = 0, 1, ..., 23
        // c2* transpose_block_size*4 = 0, 64*4, ...,64*(96-4) 
        for(int c2 = 0; c2 < BS_ANT_NUM / 4; c2++)
        {
            // (n*demul_block_size+i)*2/64  (location of the i-th subcarrier in the n-th block)
            // c1_base: n
            int c1_base = (ca_id + i) * 2 / transpose_block_size;
            // c1_offset: i 
            int c1_offset = (ca_id + i) * 2 % transpose_block_size;
            // move 32 subcarriers * 4 ants in each iteration 
            // point to the c2*4-th antenna and i-th subcarrier in the n-th block
            float* base_ptr = tar_ptr + c1_base * transpose_block_size * BS_ANT_NUM + c1_offset + c2 * transpose_block_size * 4;

            // move data store in data_buffer to spm_buffer
            // t_data holds 4 double values (32 bytes) (subcarriers of 4 ants c2*4-c2*4+3)
            // t_data[0] = *(base_ptr+0*8 bits)  data of i-th subcarrier of (c2*4+0)-th antenna
            // t_data[1] = *(base_ptr+32*8 bits)  data of i-th subcarrier of (c2*4+1)-th antenna
            // t_data[2] = *(base_ptr+64*8 bits)  data of i-th subcarrier of (c2*4+2)-th antenna
            // t_data[3] = *(base_ptr+96*8 bits)  data of i-th subcarrier of (c2*4+3)-th antenna
            __m256d t_data = _mm256_i64gather_pd((double*)base_ptr, index, 8);
            _mm256_store_pd((double*)(temp_buffer_ptr + c2 * 8), t_data);
        }

        // mat_precoder size: UE_NUM \times BS_ANT_NUM
        int precoder_offset = frame_id * OFDM_CA_NUM + ca_id + i;
        cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset].data();
        cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

        // mat_data size: BS_ANT_NUM \times 1
        cx_float* data_ptr = (cx_float *)(&spm_buffer[tid][0]);
        cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);

        // mat_demuled size: UE_NUM \times 1
        // point to the (n*demul_block_size+i)-th subcarrier and first user
        cx_float* demul_ptr = (cx_float *)(&demul_buffer_.data[offset_without_ca][(ca_id + i) * UE_NUM]);
        cx_fmat mat_demuled(demul_ptr, UE_NUM, 1, false);

        // Equalization
        // calculate mat_demuled for i-th subcarrier in the n-th block
        mat_demuled = mat_precoder * mat_data;

/*
        //debug
        if(ca_id == 640 && i == 9 && frame_id == 4 && data_subframe_id == 0)
        {
            printf("thread %d, save mat precoder\n", tid);
            FILE* fp_debug = fopen("tmpPrecoder.txt", "w");
            for(int i = 0; i < UE_NUM; i++)
            {
                for(int j = 0; j < BS_ANT_NUM; j++)
                    fprintf(fp_debug, "%f %f ", mat_precoder.at(i,j).real(), mat_precoder.at(i,j).imag());
                fprintf(fp_debug, "\n" );
            }
            fclose(fp_debug);

            fp_debug = fopen("tmpData.txt","w");
            for(int i = 0; i < BS_ANT_NUM; i++)
                fprintf(fp_debug, "%f %f\n", mat_data.at(i,0).real(), mat_data.at(i,0).imag());
            fclose(fp_debug);

            fp_debug = fopen("tmpDemul.txt","w");
            for(int i = 0; i < UE_NUM; i++)
                fprintf(fp_debug, "%f %f\n", mat_demuled.at(i,0).real(), mat_demuled.at(i,0).imag());
            fclose(fp_debug);
            
            exit(0);

        }
*/

        // Demodulation
        // int* demul_ptr2 = (int *)(&demul_buffer2_.data[offset_without_ca][(ca_id + i) * UE_NUM]);


        // for(int ue_idx = 0; ue_idx<UE_NUM; ue_idx++)
        // {
        //     *demul_ptr2+ue_idx=demod_16qam((complex_float)mat_demuled[ue_idx]);
        // }
        int* demul_ptr2 = (int *)(&demul_buffer2_.data[offset_without_ca][(ca_id + i) * UE_NUM]);
        imat  mat_demuled2 = zeros<imat>(UE_NUM,1);
        
        mat_demuled2 = demod_16qam(mat_demuled);
        // cout << "Frame: "<< frame_id<<", subframe: "<< data_subframe_id<<", SC: " << ca_id+i << ", data: " << mat_demuled2.st() << endl;

        if (frame_id==0) 
        {
            FILE* fp_debug = fopen("tmpresult.txt", "a");
            if (fp_debug==NULL) {
                printf("open file faild");
                std::cerr << "Error: " << strerror(errno) << std::endl;
                exit(0);
            }
            for(int ii = 0; ii < UE_NUM; ii++)
            {
                // printf("User %d: %d, ", ii,mat_demuled2(ii));
                fprintf(fp_debug, "%d\n", mat_demuled2(ii));
            }
            // printf("\n");
            // fwrite(mat_demuled2.memptr(), sizeof(int),sizeof(mat_demuled), fp_debug);
            fclose(fp_debug);
        }


    }

    // inform main thread
    Event_data demul_finish_event;
    demul_finish_event.event_type = EVENT_DEMUL;
    demul_finish_event.data = offset;
    
    if ( !message_queue_.enqueue(*task_ptok[tid], demul_finish_event ) ) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }
    
    //printf("put demul event\n");
}

// do ZF
void CoMP::doZF(int tid, int offset)
{

    int ca_id = offset % OFDM_CA_NUM;
    int frame_id = (offset - ca_id) / OFDM_CA_NUM;
    if (DEBUG_PRINT)
        printf("In doZF: frame: %d, subcarrier: %d\n", frame_id, ca_id);

    cx_float* ptr_in = (cx_float *)csi_buffer_.CSI[offset].data();
    cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
    cx_float* ptr_out = (cx_float *)precoder_buffer_.precoder[offset].data();
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

// do Crop
void CoMP::doCrop(int tid, int offset)
{
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
    //printf("thread %d process frame_id %d, subframe_id %d, cell_id %d, ant_id %d\n", tid, frame_id, subframe_id, cell_id, ant_id);
    // remove CP, do FFT
    int delay_offset = 0;
    int FFT_buffer_target_id = getFFTBufferIndex(frame_id, subframe_id, ant_id);

    // transfer ushort to float
    ushort* cur_ptr_buffer_ushort = (ushort*)(cur_ptr_buffer + sizeof(int) * 4);
    float* cur_fft_buffer_float = (float*)fft_buffer_.FFT_inputs[FFT_buffer_target_id];
    // Copy data for fft from socket buffer to fft buffer
    for(int i = 0; i < (OFDM_CA_NUM - delay_offset) * 2; i++)
        cur_fft_buffer_float[i] = (cur_ptr_buffer_ushort[OFDM_PREFIX_LEN + delay_offset + i] / 65536.f - 0.5f) * 4.f;
    //memcpy((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id], 
    //    cur_ptr_buffer + sizeof(int) * 4 + (OFDM_PREFIX_LEN + delay_offset) * 2 * sizeof(float), 
    //    sizeof(float) * (OFDM_CA_NUM - delay_offset) * 2); // COPY
    if(delay_offset > 0) // append zero
    {
        memset((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id] 
            + (OFDM_CA_NUM - delay_offset) * 2 * sizeof(float), 0, sizeof(float) * 2 * delay_offset);
    }
    // perform fft
    mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[FFT_buffer_target_id], 
        fft_buffer_.FFT_inputs[FFT_buffer_target_id]);

    // if it is pilot part, do CE
    if(isPilot(subframe_id))
    {
        int UE_id = subframe_id;
        int ca_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * OFDM_CA_NUM;
        int csi_offset = ant_id + UE_id * BS_ANT_NUM;
        float* cur_fft_buffer_float_output = (float*)fft_buffer_.FFT_outputs[FFT_buffer_target_id];
        

        if (DEBUG_PRINT)
            printf("In doCrop: pilot: frame: %d, subframe: %d, ant: %d\n", frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);

        for(int j = 0; j < (OFDM_CA_NUM); j++)
        {
            // divide fft output by pilot data to get CSI estimation
            float* csi_buffer_ptr = (float*)(csi_buffer_.CSI[ca_offset+j].data())+csi_offset*2;
            *(csi_buffer_ptr)= cur_fft_buffer_float_output[2*j]* pilots_[j];
            *(csi_buffer_ptr+1)= cur_fft_buffer_float_output[2*j+1]* pilots_[j];

            // if (csi_buffer_.CSI[ca_offset + j][csi_offset].real!=*(csi_buffer_ptr))
            // {
            //     printf("UE: %d, Ant %d, SC: %d, FFT_output: %.2f+%.2fj, CSI1: %.2f+%.2fj, CSI2: %.2f+%.2fj, Pilot: %.1f, Add_diff: %d\n", UE_id, ant_id, j,
            //         cur_fft_buffer_float_output[2*j], cur_fft_buffer_float_output[2*j+1],
            //         csi_buffer_.CSI[ca_offset + j][csi_offset].real,csi_buffer_.CSI[ca_offset + j][csi_offset].imag, 
            //         *(csi_buffer_ptr),*(csi_buffer_ptr+1),pilots_[j],
            //         (int)((float *)&(csi_buffer_.CSI[ca_offset + j][csi_offset])-(csi_buffer_ptr))
            //         );
            // }
            
        }

        // for(int j = 0; j < OFDM_CA_NUM; j++)
        // {
        //     csi_buffer_.CSI[ca_offset + j][csi_offset] = divide(fft_buffer_.FFT_outputs[FFT_buffer_target_id][j], pilots_[j]);          
        //     printf("Ant id: %d, SC: %d, raw CSI: %.4f+%.4fj, FFT_buffer: %.2f+%.2fj, Pilot: %d\n", ant_id,j,
        //                     fft_buffer_.FFT_inputs[FFT_buffer_target_id][j].real,
        //                     fft_buffer_.FFT_inputs[FFT_buffer_target_id][j].imag, fft_buffer_.FFT_outputs[FFT_buffer_target_id][j].real,
        //                     fft_buffer_.FFT_outputs[FFT_buffer_target_id][j].imag, pilots_[j]);
        // }       
    }
    else if(isData(subframe_id)) // if it is data part, just transpose
    {
        
        int data_subframe_id = subframe_id - UE_NUM;
        int frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * data_subframe_num_perframe + data_subframe_id;
        

        if (DEBUG_PRINT)
            printf("In doCrop: data: frame: %d, subframe: %d, ant: %d\n", frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);


        /* //naive transpose
        for(int j = 0; j < OFDM_CA_NUM; j++)
        {
            data_buffer_.data[frame_offset][ant_id + j * BS_ANT_NUM] = fft_buffer_.FFT_outputs[FFT_buffer_target_id][j];
        }
        */
        // block transpose
        // src_ptr: point to the start of subframe (subframe size: OFDM_CA_NUM) 
        // 2048 float values
        float* src_ptr = (float *)&fft_buffer_.FFT_outputs[FFT_buffer_target_id][0];
        // tar_ptr: point to the start of subframe with size BS_ANT_NUM * OFDM_CA_NUM
        // 96*1024*2 float values
        float* tar_ptr = (float *)&data_buffer_.data[frame_offset][0];
        // copy data from fft_outputs to data_buffer
        // 1024*2/8 = 256 iterations, copy 8 bytes every time
        // c2 = 0, 1, ..., 1024/64*2-1 = 31
        // c2*transpose_block_size = 0, 64, 128, ..., 2048-64
        for(int c2 = 0; c2 < OFDM_CA_NUM / transpose_block_size * 2; c2++)
        {
            // c3 = 0, 1, ..., transpose_block_size/8 -1 = 7
            // c3*8 = 0, 8, ..., 64-8
            for(int c3 = 0; c3 < transpose_block_size / 8; c3++)
            {
                // point to the (c2 * transpose_block_size + c3 * 8)th subcarrier
                // data: 256 bits = 32 bytes = 8 float values = 4 subcarriers
                __m256 data = _mm256_load_ps(src_ptr + c2 * transpose_block_size + c3 * 8);
                // original data order: SCs of ant1, SCs of ant2, ..., SCs of ant 96
                // transposed data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
                // point to the (c2 * BS_ANT_NUM * transpose_block_size + transpose_block_size * ant_id + c3 * 8)th subcarrier
                _mm256_store_ps(tar_ptr + c2 * BS_ANT_NUM * transpose_block_size + transpose_block_size * ant_id + c3 * 8, data);
            }
            
        }
        
    }


    // after finish
    socket_buffer_[buffer_id].buffer_status[offset] = 0; // now empty
    // inform main thread
    Event_data crop_finish_event;
    crop_finish_event.event_type = EVENT_CROPPED;
    crop_finish_event.data = getSubframeBufferIndex(frame_id, subframe_id);

    if ( !message_queue_.enqueue(*task_ptok[tid], crop_finish_event ) ) {
        printf("crop message enqueue failed\n");
        exit(0);
    }
}

