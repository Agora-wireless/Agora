#include "CoMP.hpp"

using namespace arma;
using namespace aff3ct;
typedef cx_float COMPLEX;


bool keep_running = true;

void intHandler(int) {
    std::cout << "will exit..." << std::endl;
    keep_running = false;
}


CoMP::CoMP()
{
    putenv( "MKL_THREADING_LAYER=sequential" );
    std::cout << "MKL_THREADING_LAYER =  " << getenv("MKL_THREADING_LAYER") << std::endl; 
    csi_format_offset = 1.0/32768;
    // openblas_set_num_threads(1);
    printf("enter constructor\n");
    // initialize socket buffer


    // read pilots from file
    // pilots_.resize(OFDM_CA_NUM);
    pilots_ = (float *)aligned_alloc(64, OFDM_CA_NUM * sizeof(float));
    FILE* fp = fopen("../pilot_f_2048.bin","rb");
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

    

    std::string filename = "/home/argos/Jian/my_project_with_aff3ct/lib/aff3ct/conf/dec/LDPC/CCSDS_64_128.alist";


    
    // aff3ct::tools::Sparse_matrix H[TASK_THREAD_NUM];

    // H = aff3ct::tools::LDPC_matrix_handler::read(filename, &info_bits_pos);

    
    
    tools::Sigma<float> noise;
    float ebn0 = 10.0f;
    // const int K = ORIG_CODE_LEN * NUM_BITS;
    // const int N = CODED_LEN * NUM_BITS;
    const float R = (float)K / (float)N;
    const auto esn0  = tools::ebn0_to_esn0 (ebn0, R);
    const auto sigma = tools::esn0_to_sigma(esn0   );
    

    // const auto up_rule = tools::Update_rule_NMS_simd <float>(0.75);

    noise.set_noise(sigma, ebn0, esn0);
    
    // up_rules.reserve(TASK_THREAD_NUM);
    Encoders.reserve(TASK_THREAD_NUM);
    // Decoders.reserve(TASK_THREAD_NUM);
    Modems.reserve(TASK_THREAD_NUM);

    // std::unique_ptr<tools::Constellation<float>> cstl(new tools::Constellation_QAM <float>(NUM_BITS));
    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        H[i] = aff3ct::tools::LDPC_matrix_handler::read(filename, &info_bits_pos[i]);
        if (info_bits_pos[i].empty()) {
            // generate a default vector [0, 1, 2, 3, ..., K-1]
            info_bits_pos[i].resize(ORIG_CODE_LEN * NUM_BITS);
            std::iota(info_bits_pos[i].begin(), info_bits_pos[i].end(), 0);
        }
        // up_rules.push_back(tools::Update_rule_NMS_simd <float,0>(0.75));
    }

    const auto &msg_chk_to_var_id = H[0].get_col_to_rows();
    const auto &msg_var_to_chk_id = H[0].get_row_to_cols();
    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        Encoders.push_back(new module::Encoder_LDPC_from_QC<>(K, N, H[i]));
        // Decoders.push_back(new module::Decoder_LDPC_BP_flooding_inter<>(K, N, N_ITE, H[i], info_bits_pos[i],up_rules[i]));
        // Decoders[i].reset(new module::Decoder_LDPC_BP_flooding_inter<>(K, N, N_ITE, H[i], info_bits_pos[i],up_rules[i]));
        std::unique_ptr<tools::Constellation<float>> cstl(new tools::Constellation_QAM <float>(NUM_BITS));
        Modems.push_back(new module::Modem_generic<>(N, std::move(cstl)));
        Modems[i]->set_noise(noise);
    }    





    // initialize FFT buffer
    // int FFT_buffer_block_num = BS_ANT_NUM * subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    // fft_buffer_.FFT_inputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *));//new complex_float * [FFT_buffer_block_num];
    // fft_buffer_.FFT_outputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *));//new complex_float * [FFT_buffer_block_num];
    // for (int i = 0; i < FFT_buffer_block_num; i++) {
    //     fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    //     fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    // }

    // int FFT_buffer_block_num = subframe_num_perframe * TASK_BUFFER_FRAME_NUM;
    int FFT_buffer_block_num = TASK_THREAD_NUM;
    fft_buffer_.FFT_inputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *));
    
    for (int i = 0; i < FFT_buffer_block_num; i++) {
        // fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(BS_ANT_NUM * OFDM_CA_NUM * sizeof(complex_float));
        // fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(BS_ANT_NUM * OFDM_CA_NUM * sizeof(complex_float));
        fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
        memset(fft_buffer_.FFT_inputs[i], 0, sizeof(OFDM_CA_NUM * sizeof(complex_float)));
        // fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    }

    fft_buffer_.FFT_outputs = (complex_float **)malloc(FFT_buffer_block_num * sizeof(complex_float *));

    for (int i = 0; i < FFT_buffer_block_num; i++) {
        // fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(BS_ANT_NUM * OFDM_CA_NUM * sizeof(complex_float));
        // fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(BS_ANT_NUM * OFDM_CA_NUM * sizeof(complex_float));
        // fft_buffer_.FFT_inputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
        fft_buffer_.FFT_outputs[i] = (complex_float *)mufft_alloc(OFDM_CA_NUM * sizeof(complex_float));
    }

    // initialize muplans for fft
    for (int i = 0; i < TASK_THREAD_NUM; i++) 
        muplans_[i] = mufft_create_plan_1d_c2c(OFDM_CA_NUM, MUFFT_FORWARD, MUFFT_FLAG_CPU_ANY);


    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        mkl_statuses[i] = DftiCreateDescriptor(&mkl_handles[i], DFTI_SINGLE, DFTI_COMPLEX, 1, OFDM_CA_NUM);
        mkl_statuses[i] = DftiCommitDescriptor(mkl_handles[i]);
    }

    // initialize CSI buffer
    // csi_buffer_.CSI.resize(OFDM_CA_NUM * TASK_BUFFER_FRAME_NUM);
    // for (int i = 0; i < csi_buffer_.CSI.size(); i++)
    //     csi_buffer_.CSI[i].resize(BS_ANT_NUM * UE_NUM);
    // printf("CSI buffer initialized\n");

    int csi_buffer_size = UE_NUM * TASK_BUFFER_FRAME_NUM;
    csi_buffer_.CSI = (complex_float **)malloc(csi_buffer_size * sizeof(complex_float *));
    for (int i = 0; i < csi_buffer_size; i++)
        csi_buffer_.CSI[i] = (complex_float *)aligned_alloc(64, BS_ANT_NUM * OFDM_DATA_NUM * sizeof(complex_float));
    // int csi_buffer_size = OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM;
    // csi_buffer_.CSI = (complex_float **)malloc(csi_buffer_size * sizeof(complex_float *));
    // for (int i = 0; i < csi_buffer_size; i++)
    //     csi_buffer_.CSI[i] = (complex_float *)aligned_alloc(64, BS_ANT_NUM * UE_NUM * sizeof(complex_float));


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
        spm_buffer[i] = (complex_float *)aligned_alloc(64, 8 * BS_ANT_NUM * sizeof(complex_float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        csi_gather_buffer[i] = (complex_float *)aligned_alloc(BS_ANT_NUM * UE_NUM * sizeof(complex_float), BS_ANT_NUM * UE_NUM * sizeof(complex_float));


    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        precoder_buffer_temp[i] = (complex_float *)aligned_alloc(BS_ANT_NUM * UE_NUM * sizeof(complex_float), BS_ANT_NUM * UE_NUM * sizeof(complex_float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        equaled_buffer_temp[i] = (complex_float *)aligned_alloc(64, demul_block_size * UE_NUM * sizeof(complex_float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        equaled_buffer_T_temp[i] = (complex_float *)aligned_alloc(64, demul_block_size * UE_NUM * sizeof(complex_float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        demul_hard_buffer_temp[i] = (uint8_t *)aligned_alloc(64, demul_block_size * UE_NUM * sizeof(uint8_t));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        demul_soft_buffer_temp[i] = (float *)aligned_alloc(64, NUM_BITS * CODED_LEN * UE_NUM * sizeof(float));

    for (int i = 0; i < TASK_THREAD_NUM; ++i)
        coded_buffer_temp[i] = (int*)aligned_alloc(64, NUM_BITS * CODED_LEN * sizeof(int));
    // printf("Demultiplexed data buffer initialized\n");


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
    dl_IQ_data = new int * [data_subframe_num_perframe * UE_NUM];
    dl_IQ_data_long = new long long * [data_subframe_num_perframe * UE_NUM];
    for (int i = 0; i < data_subframe_num_perframe * UE_NUM; i++) {
        dl_IQ_data[i] = new int[OFDM_CA_NUM];
        dl_IQ_data_long[i] = new long long[packageSenderBS::OFDM_FRAME_LEN];
    }
    // read data from file
    std::string filename1 = "../orig_data_2048_ant" + std::to_string(BS_ANT_NUM) + ".bin";
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

    // initialize muplans for ifft
    for (int i = 0; i < TASK_THREAD_NUM; i++)
        muplans_ifft_[i] = mufft_create_plan_1d_c2c(OFDM_CA_NUM, MUFFT_INVERSE, MUFFT_FLAG_CPU_ANY);



    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        mkl_statuses_dl[i] = DftiCreateDescriptor(&mkl_handles_dl[i], DFTI_SINGLE, DFTI_COMPLEX, 1, OFDM_CA_NUM);
        mkl_statuses_dl[i] = DftiCommitDescriptor(mkl_handles_dl[i]);
    }

    // // initialize downlink iffted data buffer
    // dl_iffted_data_buffer_.data.resize(data_subframe_num_perframe * TASK_BUFFER_FRAME_NUM);
    // for (int i = 0; i < dl_iffted_data_buffer_.data.size(); i++)
    //     dl_iffted_data_buffer_.data[i].resize(UE_NUM * OFDM_CA_NUM);

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


    // memset(dl_data_counter_scs_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    // memset(modulate_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 

    for (int i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
        memset(dl_data_counter_scs_[i], 0, sizeof(int) * data_subframe_num_perframe);
        memset(modulate_checker_[i], 0, sizeof(int) * data_subframe_num_perframe);
        memset(tx_counter_ants_[i], 0, sizeof(int) * data_subframe_num_perframe);
    }

    memset(dl_data_counter_subframes_, 0, sizeof(int) * TASK_THREAD_NUM);
    memset(ifft_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    memset(tx_counter_subframes_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM); 
    // memset(tx_counter_ants_, 0, sizeof(int) * SOCKET_BUFFER_FRAME_NUM); 

    // for (int i = 0; i < SOCKET_BUFFER_FRAME_NUM; i++) 
    //     memset(tx_counter_ants_[i], 0, sizeof(int) * data_subframe_num_perframe);

    printf("initialize QAM16 table\n");
    float scale = 1/sqrt(10);
    float modvec_16qam[4] = {-3*scale, -1*scale, 3*scale, scale};
    for (int i = 0; i < 16; i++) {
        qam16_table[0][i] = modvec_16qam[i / 4];
        qam16_table[1][i] = modvec_16qam[i % 4];
    }

    printf("new PackageSender\n");
    transmitter_.reset(new packageSenderBS(SOCKET_TX_THREAD_NUM, &complete_task_queue_, &tx_queue_));

    for (int i = 0; i < TASK_THREAD_NUM; i++) {
        memset(IFFT_task_duration[i], 0, sizeof(double) * 4 * 8);
        memset(Precode_task_duration[i], 0, sizeof(double) * 4 * 8);
    }

    memset(IFFT_task_count, 0, sizeof(int) * TASK_THREAD_NUM * 16);
    memset(Precode_task_count, 0, sizeof(int) * TASK_THREAD_NUM * 16);

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

   
    for (auto it = Encoders.begin(); it != Encoders.end(); ++it) {
        delete *it;
    }

    for (auto it = Modems.begin(); it != Modems.end(); ++it) {
        delete *it;
    }

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
    double *frame_start_ptrs[SOCKET_RX_THREAD_NUM];
    for(int i = 0; i < SOCKET_RX_THREAD_NUM; i++) {
        socket_buffer_ptrs[i] = socket_buffer_[i].buffer;
        socket_buffer_status_ptrs[i] = socket_buffer_[i].buffer_status;
        frame_start_ptrs[i] = frame_start[i];
        // printf("Socket buffer ptr: %llx, socket_buffer: %llx\n", socket_buffer_ptrs[i],socket_buffer_[i].buffer);
    }
    std::vector<pthread_t> rx_threads = receiver_->startRecv(socket_buffer_ptrs, 
        socket_buffer_status_ptrs, socket_buffer_status_size_, socket_buffer_size_, frame_start_ptrs, main_core_id + 1);

    // start downlink transmitter
#if ENABLE_DOWNLINK
    char *dl_socket_buffer_ptr = dl_socket_buffer_.buffer;
    int *dl_socket_buffer_status_ptr = dl_socket_buffer_.buffer_status;
    float *dl_data_ptr = (float *)(&dl_precoded_data_buffer_.data[0][0]);
    std::vector<pthread_t> tx_threads = transmitter_->startTX(dl_socket_buffer_ptr, 
        dl_socket_buffer_status_ptr, dl_data_ptr, dl_socket_buffer_status_size_, dl_socket_buffer_size_, main_core_id + 1 +SOCKET_RX_THREAD_NUM);
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
                        // hpctoolkit_sampling_start();
                    }

                    // char *socket_buffer_ptr = socket_buffer_ptrs[socket_thread_id] + offset_in_current_buffer * PackageReceiver::package_length;
                    char *socket_buffer_ptr = socket_buffer_[socket_thread_id].buffer + (long long) offset_in_current_buffer * PackageReceiver::package_length;
                    int subframe_id = *((int *)socket_buffer_ptr + 1);                                    
                    int frame_id = *((int *)socket_buffer_ptr);
                    int ant_id = *((int *)socket_buffer_ptr + 3);
                    int rx_frame_id = (frame_id % TASK_BUFFER_FRAME_NUM);

                    // printf("Main thread: data received from frame %d, subframe %d, ant %d, buffer ptr: %llx, offset %lld\n", frame_id, subframe_id, ant_id, 
                    //      socket_buffer_ptr, offset_in_current_buffer * PackageReceiver::package_length);
                    rx_counter_packets_[rx_frame_id]++;
                    
#if ENABLE_DOWNLINK
                    int rx_counter_packets_max = BS_ANT_NUM * UE_NUM;
#else
                    int rx_counter_packets_max = BS_ANT_NUM * subframe_num_perframe;
#endif
                    if (rx_counter_packets_[rx_frame_id] == 1) {   

                        // pilot_received[rx_frame_id] = get_time();      
                        pilot_received[frame_id] = get_time();  
#if DEBUG_PRINT_PER_FRAME_START 
                        if (frame_id > 0)
                            printf("Main thread: data received from frame %d, subframe %d, ant %d, in %.5f us\n", frame_id, subframe_id, ant_id, 
                                pilot_received[frame_id]-pilot_received[frame_id-1]);
                        else
                            printf("Main thread: data received from frame %d, subframe %d, ant %d, in %.5f us\n", frame_id, subframe_id, ant_id, 
                                pilot_received[frame_id]);
#endif                       
                    }
                    else if (rx_counter_packets_[rx_frame_id] == rx_counter_packets_max) {  
                        rx_processed[frame_id] = get_time();
#if DEBUG_PRINT_PER_FRAME_DONE 
                        printf("Main thread: received data for all packets in frame: %d, frame buffer: %d in %.5f us\n", frame_id, frame_id% TASK_BUFFER_FRAME_NUM, rx_processed[frame_id]-pilot_received[frame_id]);
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
                        if(frame_id > 0) 
                            demul_counter_subframes_[(frame_id-1)%TASK_BUFFER_FRAME_NUM] = 0;
                        else
                            demul_counter_subframes_[TASK_BUFFER_FRAME_NUM-1] = 0;
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
                                // fft_processed[frame_id] = get_time();  
                                fft_processed[frame_count_pilot_fft] = get_time();   
                                csi_counter_users_[frame_id] = 0; 
#if DEBUG_PRINT_PER_FRAME_DONE
                                printf("Main thread: pilot frame: %d, %d, finished FFT for all pilot subframes in %.5f us\n", frame_id, frame_count_pilot_fft, 
                                    fft_processed[frame_count_pilot_fft]-pilot_received[frame_count_pilot_fft]);
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
                            printf("Main thread: finished FFT for frame %d, subframe %d, precoder status: %d, fft queue: %d, zf queue: %d, demul queue: %d\n", 
                                    frame_id, subframe_id, 
                                    precoder_exist_in_frame_[frame_id], fft_queue_.size_approx(), zf_queue_.size_approx(), demul_queue_.size_approx());
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
                                int end_sche_id = max(UE_NUM + data_counter_subframes_[frame_id], subframe_id);
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
                        for(int j = 0; j < OFDM_DATA_NUM / demul_block_size; j++) {
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
                        printf("Main thread: Demodulation done frame: %d, subframe: %d\n", frame_id, data_subframe_id);
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
                        if (demul_count == data_subframe_num_perframe * 1000)
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

            case EVENT_DECODE: {
                    int offset_decode = event.data;                   
                    int frame_id, total_data_subframe_id, data_subframe_id, ue_offset;
                    interpreteOffset3d(OFDM_DATA_NUM*UE_NUM, offset_decode, &frame_id, &total_data_subframe_id, &data_subframe_id, &ue_offset);
                    int ue_id = ue_offset % CODED_LEN;
                    int block_id = ue_offset / CODED_LEN;

                    decode_counter_blocks_[frame_id][data_subframe_id]++;

#if DEBUG_PRINT_PER_TASK_DONE
                    printf("Main thread: Decode done frame: %d, subframe: %d, block: %d, user: %d,offset %d, count: %d\n", frame_id, data_subframe_id, block_id, ue_id, offset_decode, decode_counter_blocks_[frame_id][data_subframe_id]);
#endif
                    if (decode_counter_blocks_[frame_id][data_subframe_id] == UE_NUM * NUM_CODE_BLOCK) {
#if DEBUG_PRINT_PER_SUBFRAME_DONE
                        printf("Main thread: Decode done frame: %d, subframe: %d\n", frame_id, data_subframe_id);
#endif
                        decode_counter_subframes_[frame_id] ++;
                        if (decode_counter_subframes_[frame_id] == data_subframe_num_perframe) {
#if DEBUG_PRINT_PER_FRAME_DONE
                            printf("Main thread: Decode done frame: %d, %d, subframe %d in %.5f us, total %.5f us\n", frame_id, frame_count_decode, data_subframe_id, 
                                get_time()-zf_processed[frame_count_decode], get_time()-pilot_received[frame_count_decode]);
#endif                  
                            frame_count_decode++;
                            if (frame_count_decode == 1e9) 
                                frame_count_decode = 0;

                            decode_counter_subframes_[frame_id] = 0;
                        }
                        
                        decode_counter_blocks_[frame_id][data_subframe_id] = 0;
                    }

                }
                break;

            case EVENT_MODUL: {
                    // Modulation is done, schedule precoding when data for all users is ready
                    int offset_modul = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, user_id;
                    interpreteOffset3d(UE_NUM, offset_modul, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &user_id);
                    modulate_checker_[frame_id][current_data_subframe_id] += 1;
#if DEBUG_PRINT_PER_TASK_DONE
                    printf("Main thread: Modulation done frame: %d, subframe: %d, user: %d\n", frame_id, current_data_subframe_id,  user_id);
#endif
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
#if DEBUG_PRINT_PER_SUBFRAME_DONE
                        printf("Main thread: Modulation done for all users in frame: %d, subframe: %d\n", frame_id, current_data_subframe_id);
#endif                       
                    } 
                }
                break; 
            case EVENT_PRECODE: {
                    // Precoding is done, schedule ifft 
                    int offset_precode = event.data;
                    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
                    interpreteOffset3d(OFDM_DATA_NUM, offset_precode, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);
                    dl_data_counter_scs_[frame_id][current_data_subframe_id] += demul_block_size;
#if DEBUG_PRINT_PER_TASK_DONE
                    printf("Main thread: Precoding done frame: %d, subframe: %d, subcarrier: %d, offset: %d, total SCs: %d\n", 
                        frame_id, current_data_subframe_id,  sc_id, offset_precode, dl_data_counter_scs_[frame_id][current_data_subframe_id]);
#endif
                    
                    if (dl_data_counter_scs_[frame_id][current_data_subframe_id] == OFDM_DATA_NUM) {
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
                            for(int j = 0; j < OFDM_DATA_NUM / demul_block_size; j++) {
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
                    schedule_task(do_tx_task, &tx_queue_, ptok_tx);
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
                        if (tx_count == dl_data_subframe_num_perframe * 1000)
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
                                printf("In frame %d, thread %d, \t\t\t csi: %d tasks %.5f (%.5f, %.5f, %.5f), zf: %d tasks %.5f (%.5f, %.5f, %.5f), ifft: %d tasks %.5f, sum: %.5f\n",
                                        frame_id, i, csi_count_this_frame_this_thread, 
                                        csi_time_this_frame_this_thread[0], csi_time_this_frame_this_thread[1], csi_time_this_frame_this_thread[2], csi_time_this_frame_this_thread[3],
                                        ifft_count_this_frame_this_thread, ifft_time_this_frame_this_thread[0], sum_time_this_frame_this_thread);
        #endif
                                csi_count_per_thread[i] = CSI_task_count[i];
                                ifft_count_per_thread[i] = IFFT_task_count[i];
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
                                        frame_id, i, csi_count_this_frame_this_thread, 
                                        zf_count_this_frame_this_thread, zf_time_this_frame_this_thread[0], zf_time_this_frame_this_thread[1], zf_time_this_frame_this_thread[2], zf_time_this_frame_this_thread[3]);
        #endif
                                
                                zf_count_per_thread[i] = ZF_task_count[i];
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
                                printf("In frame %d, thread %d, \t\t\t precode: %d tasks %.5f, sum: %.5f\n",
                                        frame_id, i, csi_count_this_frame_this_thread, 
                                        precode_count_this_frame_this_thread, precode_time_this_frame_this_thread[0], sum_time_this_frame_this_thread);
        #endif
                                precode_count_per_thread[i] = Precode_task_count[i];
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
                                printf("In frame %d, thread %d, \t\t\t fft: %d tasks %.5f (%.5f, %.5f, %.5f), zf: %d tasks %.5f (%.5f, %.5f, %.5f), precode: %d tasks %.5f, ifft: %d tasks %.5f, sum: %.5f\n",
                                        frame_id, i, csi_count_this_frame_this_thread, 
                                        csi_time_this_frame_this_thread[0], csi_time_this_frame_this_thread[1], csi_time_this_frame_this_thread[2], csi_time_this_frame_this_thread[3],
                                        zf_count_this_frame_this_thread, zf_time_this_frame_this_thread[0], zf_time_this_frame_this_thread[1], zf_time_this_frame_this_thread[2], zf_time_this_frame_this_thread[3], 
                                        precode_count_this_frame_this_thread, precode_time_this_frame_this_thread[0], ifft_count_this_frame_this_thread, ifft_time_this_frame_this_thread[0], sum_time_this_frame_this_thread);
        #endif
                                csi_count_per_thread[i] = CSI_task_count[i];
                                zf_count_per_thread[i] = ZF_task_count[i];
                                ifft_count_per_thread[i] = IFFT_task_count[i];
                                precode_count_per_thread[i] = Precode_task_count[i];
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
        }
    }
    // }


    printf("Print results\n");
    FILE* fp_debug = fopen("../timeresult.txt", "w");
    if (fp_debug==NULL) {
        printf("open file faild");
        std::cerr << "Error: " << strerror(errno) << std::endl;
        exit(0);
    }
    
        // printf("User %d: %d, ", ii,demul_ptr2(ii));
#if ENABLE_DOWNLINK
    for(int ii = 0; ii < frame_count_tx; ii++) { 
        fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], 
            precode_processed[ii], ifft_processed[ii], tx_processed[ii],tx_processed_first[ii],
                csi_time_in_function[ii], zf_time_in_function[ii], precode_time_in_function[ii], ifft_time_in_function[ii], processing_started[ii]);
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
            
                fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
                        csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii]);
        }
        else {
                    
                fprintf(fp_debug, "%.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f %.5f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
                    csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii], frame_start[1][ii] );
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
    
    CoMP* obj_ptr = ((EventHandlerContext *)context)->obj_ptr;
    moodycamel::ConcurrentQueue<Event_data>* fft_queue_ = &(obj_ptr->fft_queue_);
    moodycamel::ConcurrentQueue<Event_data>* zf_queue_ = &(obj_ptr->zf_queue_);
    moodycamel::ConcurrentQueue<Event_data>* demul_queue_ = &(obj_ptr->demul_queue_);
    moodycamel::ConcurrentQueue<Event_data>* decode_queue_ = &(obj_ptr->decode_queue_);
    moodycamel::ConcurrentQueue<Event_data>* ifft_queue_ = &(obj_ptr->ifft_queue_);
    moodycamel::ConcurrentQueue<Event_data>* modulate_queue_ = &(obj_ptr->modulate_queue_);
    moodycamel::ConcurrentQueue<Event_data>* precode_queue_ = &(obj_ptr->precode_queue_);
    // moodycamel::ConcurrentQueue<Event_data>* tx_queue_ = &(obj_ptr->tx_queue_);
    int tid = ((EventHandlerContext *)context)->id;
    printf("task thread %d starts\n", tid);
    
    // attach task threads to specific cores
#ifdef ENABLE_CPU_ATTACH
    int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;

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

    obj_ptr->task_ptok[tid].reset(new moodycamel::ProducerToken(obj_ptr->complete_task_queue_));
    obj_ptr->Decoders[tid].reset(new module::Decoder_LDPC_BP_horizontal_layered_ONMS_inter<>(obj_ptr->K, obj_ptr->N, N_ITE, obj_ptr->H[tid], 
                obj_ptr->info_bits_pos[tid],0.825f,0,0,1,UE_NUM));

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
                    ret_modul = modulate_queue_->try_dequeue(event);
                    if (!ret_modul) {
                        ret_zf = zf_queue_->try_dequeue(event);
                        if (!ret_zf) {
                            ret = fft_queue_->try_dequeue(event);
                            if (!ret) 
                                continue;
                            else
                                obj_ptr->doFFT(tid, event.data);
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
                // ret_decode = decode_queue_->try_dequeue(event);
                // if(!ret_decode) {
                    ret_demul = demul_queue_->try_dequeue(event);
                    if (!ret_demul) {   
                        ret = fft_queue_->try_dequeue(event);
                        if (!ret)
                            continue;
                        else 
                            obj_ptr->doFFT(tid, event.data);
                    }
                    else {
                        // TODO: add precoder status check
                        obj_ptr->doDemul(tid, event.data);
                    }
                // }
                // else {
                //     obj_ptr->doDecode(tid, event.data);
                // }
            }
            else if (event.event_type == TASK_ZF) {
                obj_ptr->doZF(tid, event.data);
            }
            else if (event.event_type == TASK_PRED) {
                obj_ptr->doPred(tid, event.data);
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
    int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
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
                obj_ptr->do_ifft(tid, event.data);
#else
            continue;
#endif
        }
        else
            obj_ptr->doFFT(tid, event.data);
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
    int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
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

    int total_count = 0;
    int miss_count = 0;
    Event_data event;
    bool ret_zf = false;

    while(true) {
        ret_zf = zf_queue_->try_dequeue(event);
        if (!ret_zf) 
            continue;
        else
            obj_ptr->doZF(tid, event.data);
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
    int offset_id = SOCKET_RX_THREAD_NUM + SOCKET_TX_THREAD_NUM + CORE_OFFSET + 2;
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
            obj_ptr->do_precode(tid, event.data);
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
            obj_ptr->doDemul(tid, event.data);
        }
#endif   
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


inline void CoMP::demod_16qam_loop(float *vec_in, uint8_t *vec_out, int ue_num)
{
    float float_val = 0.6325;
    for (int i = 0; i < ue_num; i++) {
        float real_val = *(vec_in + i * 2);
        float imag_val = *(vec_in + i * 2 + 1);
        
        *(vec_out + i) = 0;
        if (real_val > 0)
            *(vec_out + i) |= 1UL << 3;
            //*(vec_out + i) += 8;
        if (std::abs(real_val) < float_val)
            *(vec_out + i) |= 1UL << 2;
            //*(vec_out + i) += 4;
        if (imag_val > 0)
            *(vec_out + i) |= 1UL << 1;
            //*(vec_out + i) += 2;
        if (std::abs(imag_val) < float_val)
            *(vec_out + i) |= 1UL ;
            //*(vec_out + i) += 1;

    }
    // __m128 vec_zero = _mm_set1_ps(0);
    // __m128 vec_float = _mm_set1_ps(0.6325);
    // for (int i = 0; i < ue_num; i += 4) {
    //     __m128 vec_in_cur = 
    //     __m128 vec_real_cmp = _mm_cmpgt_ps()
    // }
}


inline void CoMP::demod_16qam_soft(float *vec_in, float *vec_out, int length )
{
    unsigned char re, im;
    for (int i = 0; i < length; i++) {
        float re_float = *(vec_in + i * 2);
        float im_float = *(vec_in + i * 2+1);
        re = (unsigned char)((re_float+1) * 255);
        im = (unsigned char)((im_float+1) * 255);
        *(vec_out + 4 * i) = m_bpsk_lut[re];
        *(vec_out + 4 * i + 1) = m_qam16_lut2[re];
        *(vec_out + 4 * i + 2) = m_bpsk_lut[im];
        *(vec_out + 4 * i + 3) = m_qam16_lut2[im];
    }
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

void CoMP::doFFT(int tid, int offset)
{
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
    int buffer_subframe_num = subframe_num_perframe * BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM;
    int socket_thread_id = offset / buffer_subframe_num;
    // offset = offset - socket_thread_id * buffer_subframe_num;
    offset = offset % buffer_subframe_num;
    // printf("In doFFT: socket_thread: %d offset %d\n", socket_thread_id, offset);
    // read info of one frame
    char *cur_ptr_buffer = socket_buffer_[socket_thread_id].buffer + (long long) offset * PackageReceiver::package_length;

    int ant_id, frame_id, subframe_id, cell_id;
    frame_id = *((int *)cur_ptr_buffer);
    subframe_id = *((int *)cur_ptr_buffer + 1);
    cell_id = *((int *)cur_ptr_buffer + 2);
    ant_id = *((int *)cur_ptr_buffer + 3);
    // printf("thread %d process frame_id %d, subframe_id %d, cell_id %d, ant_id %d\n", tid, frame_id, subframe_id, cell_id, ant_id);
    // remove CP, do FFT
    int delay_offset = 0;
    // int FFT_buffer_target_id = getFFTBufferIndex(frame_id, subframe_id, ant_id);
    // int FFT_buffer_target_id = (frame_id % TASK_BUFFER_FRAME_NUM) * (subframe_num_perframe) + subframe_id;


    // transfer ushort to float
    short *cur_ptr_buffer_ushort = (short *)(cur_ptr_buffer + 64);
    // float *cur_fft_buffer_float = (float *)fft_buffer_.FFT_inputs[FFT_buffer_target_id];
    // float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM);
    // float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[tid] + ant_id * OFDM_CA_NUM);
    float *cur_fft_buffer_float = (float *)(fft_buffer_.FFT_inputs[tid]);

    int pilot_symbol = -1;
    if (isPilot(subframe_id))
        pilot_symbol = 1;
    else if (isData(subframe_id))
        pilot_symbol = 0;


    // Use SIMD
    // reference: https://stackoverflow.com/questions/50597764/convert-signed-short-to-float-in-c-simd
    // 0x4380'8000
    const __m256 magic = _mm256_set1_ps(float((1<<23) + (1<<15))/32768.f);
    const __m256i magic_i = _mm256_castps_si256(magic);
    for (int i = 0; i < OFDM_CA_NUM * 2; i += 16) {
        // get input:
        __m128i val = _mm_load_si128((__m128i*)(cur_ptr_buffer_ushort + i)); // port 2,3

        __m128i val1 = _mm_load_si128((__m128i*)(cur_ptr_buffer_ushort + i + 8)); 
        // interleave with 0x0000
        __m256i val_unpacked = _mm256_cvtepu16_epi32(val); // port 5
        /// convert by xor-ing and subtracting magic value:
        // VPXOR avoids port5 bottlenecks on Intel CPUs before SKL
        __m256i val_f_int = _mm256_xor_si256(val_unpacked, magic_i); // port 0,1,5
        __m256 val_f = _mm256_castsi256_ps(val_f_int);  // no instruction
        __m256 converted = _mm256_sub_ps(val_f, magic); // port 1,5 ?
        // store:
        // __m256 converted = _mm256_set1_ps(0); 
        _mm256_store_ps(cur_fft_buffer_float + i, converted); // port 2,3,4,7
        // _mm256_load_ps((cur_fft_buffer_float + i));


        __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val); // port 5
        /// convert by xor-ing and subtracting magic value:
        // VPXOR avoids port5 bottlenecks on Intel CPUs before SKL
        __m256i val_f_int1 = _mm256_xor_si256(val_unpacked1, magic_i); // port 0,1,5
        __m256 val_f1 = _mm256_castsi256_ps(val_f_int1);  // no instruction
        __m256 converted1 = _mm256_sub_ps(val_f1, magic); // port 1,5 ?
        // store:
        // __m256 converted = _mm256_set1_ps(0); 
        _mm256_store_ps(cur_fft_buffer_float + i + 8, converted1); // port 2,3,4,7
        // _mm256_load_ps((cur_fft_buffer_float + i));

    }


    // for(int j = 0; j < OFDM_CA_NUM * 2; j++)
    //     cur_fft_buffer_float[j] = cur_ptr_buffer_ushort[j] * csi_format_offset;
    
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
#if DEBUG_UPDATE_STATS_DETAILED
    double start_time1 = get_time();
    double duration1 = start_time1 - start_time;
    if (pilot_symbol == 0)
        FFT_task_duration[tid * 8][1] += duration1;
    else 
        CSI_task_duration[tid * 8][1] += duration1;
#endif
    // for(int i = 0; i < (OFDM_CA_NUM - delay_offset) * 2; i++)
    //     cur_fft_buffer_float[i] = cur_ptr_buffer_ushort[OFDM_PREFIX_LEN + delay_offset + i] * csi_format_offset;
    
    // append zero
    // if(delay_offset > 0) 
    //     memset((char *)fft_buffer_.FFT_inputs[FFT_buffer_target_id] 
    //         + (OFDM_CA_NUM - delay_offset) * 2 * sizeof(float), 0, sizeof(float) * 2 * delay_offset);
    // mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[tid] + ant_id * OFDM_CA_NUM, 
    //     fft_buffer_.FFT_inputs[tid] + ant_id * OFDM_CA_NUM);
    // mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[tid], fft_buffer_.FFT_inputs[tid]);

    DftiComputeForward(mkl_handles[tid],fft_buffer_.FFT_inputs[tid]);


#if DEBUG_UPDATE_STATS_DETAILED
    double start_time2 = get_time();
    double duration2 = start_time2 - start_time1;
    if (pilot_symbol == 0)
        FFT_task_duration[tid * 8][2] += duration2;
    else
        CSI_task_duration[tid * 8][2] += duration2;
#endif
    // mufft_execute_plan_1d(muplans_[tid], fft_buffer_.FFT_outputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM, 
    //     fft_buffer_.FFT_inputs[FFT_buffer_target_id] + ant_id * OFDM_CA_NUM);
    // printf("In doFFT thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
    // printf("FFT input\n");
    // for ( int i = 0; i< OFDM_CA_NUM; i++) {
    //     cout <<"("<<(*(fft_buffer_.FFT_inputs[tid] +i)).real<<","<<(*(fft_buffer_.FFT_inputs[tid] +i)).imag<<") ";
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
        printf("In doFFT thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
#endif
#if DEBUG_UPDATE_STATS_DETAILED
    double start_time_part3 = get_time();
#endif
    // if it is pilot part, do CE
    if(pilot_symbol == 1) {
        int UE_id = subframe_id;
        // int ca_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * OFDM_CA_NUM;
        // int csi_offset = ant_id + UE_id * BS_ANT_NUM;
        int subframe_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * UE_NUM + UE_id;
        // int csi_offset = UE_id + ant_id * UE_NUM;

        // float* cur_fft_buffer_float_output = (float*)(fft_buffer_.FFT_outputs[tid] + ant_id * OFDM_CA_NUM);
        float* cur_fft_buffer_float_output = (float*)(fft_buffer_.FFT_inputs[tid]); //+ OFDM_DATA_START * 2;

        // Use SIMD
        float* csi_buffer_ptr = (float*)(csi_buffer_.CSI[subframe_offset]);
        int sc_idx = OFDM_DATA_START;
        int dst_idx = 0;

        int cache_line_num = transpose_block_size / 8;
        int iteration_per_page = 64 / cache_line_num;
        int offset_in_page = OFDM_DATA_START / 8;
        int block_num = OFDM_DATA_NUM / transpose_block_size;
        _mm_prefetch((char*)pilots_, _MM_HINT_T0);
        for (int block_idx = 0; block_idx < block_num; block_idx ++) {
            // if (block_idx % iteration_per_page == 0 && block_idx < block_num - iteration_per_page)
            //     float temp = *(cur_fft_buffer_float_output + sc_idx * 2 + 1024);
            for (int sc_inblock_idx = 0; sc_inblock_idx < transpose_block_size; sc_inblock_idx += 8) {
                // load 8 floats (4 bytes) / 4 complex floats
                float *src_ptr_cur = cur_fft_buffer_float_output + sc_idx * 2;
                // _mm_prefetch((char*)(src_ptr_cur + 16), _MM_HINT_T0);
                // _mm_prefetch((char*)(pilots_ + sc_idx * 2 + 16), _MM_HINT_T0);
                __m256 pilot_rx = _mm256_load_ps(src_ptr_cur);
                __m256 pilot_tx = _mm256_set_ps(pilots_[sc_idx+3], pilots_[sc_idx+3], pilots_[sc_idx+2], pilots_[sc_idx+2], 
                                                pilots_[sc_idx+1], pilots_[sc_idx+1], pilots_[sc_idx], pilots_[sc_idx]);
                __m256 csi_est = _mm256_mul_ps(pilot_rx, pilot_tx);

                __m256 pilot_rx1 = _mm256_load_ps(src_ptr_cur + 8);
                __m256 pilot_tx1 = _mm256_set_ps(pilots_[sc_idx+7], pilots_[sc_idx+7], pilots_[sc_idx+6], pilots_[sc_idx+6], 
                                                pilots_[sc_idx+5], pilots_[sc_idx+5], pilots_[sc_idx+4], pilots_[sc_idx+4]);
                __m256 csi_est1 = _mm256_mul_ps(pilot_rx1, pilot_tx1);

                float *tar_ptr_cur = csi_buffer_ptr + (block_idx * BS_ANT_NUM + ant_id) * transpose_block_size * 2 + sc_inblock_idx * 2;
                _mm256_stream_ps(tar_ptr_cur, csi_est);
                _mm256_stream_ps(tar_ptr_cur + 8, csi_est1);
                // printf("subcarrier index: %d, pilot: %.2f, %.2f, %.2f, %2.f\n", sc_idx, pilots_[sc_idx], pilots_[sc_idx+1], pilots_[sc_idx+2], pilots_[sc_idx+3]);
                // float *temp = (float *) &csi_est;
                // float *temp_rx = (float *)&pilot_rx;
                // float *temp_tx = (float *)&pilot_tx;
                // printf("Pilot_rx: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n ", temp_rx[0],temp_rx[1],temp_rx[2],temp_rx[3],temp_rx[4],temp_rx[5],temp_rx[6],temp_rx[7]);
                // printf("Pilot_tx: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n ", temp_tx[0],temp_tx[1],temp_tx[2],temp_tx[3],temp_tx[4],temp_tx[5],temp_tx[6],temp_tx[7]);
                // printf("CSI: %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f\n ", temp[0],temp[1],temp[2],temp[3],temp[4],temp[5],temp[6],temp[7]);
                sc_idx += 8;
                
            }
        }
        
        // cout<<"Before: "<<endl;
        // for (int i =0;i<OFDM_CA_NUM; i++) {
        //     cout<<"("<<i<<", "<<fft_buffer_.FFT_outputs[tid][i].real<<","<<fft_buffer_.FFT_outputs[tid][i].imag<<") ";
        // }
        // cout<<endl;
        // printf("In doFFT thread %d: frame: %d, subframe: %d, ant: %d\n", tid, frame_id%TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
        // cout<<"After: "<<endl;
        // // for (int i =0;i<OFDM_CA_NUM; i++) {
        // //     cout<<"("<<i<<", "<<fft_buffer_.FFT_outputs[tid][i].real*pilots_[i]<<","<<fft_buffer_.FFT_outputs[tid][i].imag*pilots_[i]<<") ";
        // // }
        // for (int i =0;i<OFDM_DATA_NUM*BS_ANT_NUM; i++) {
        //     cout<<"("<<i<<", "<<csi_buffer_.CSI[subframe_offset][i].real<<"+j"<<csi_buffer_.CSI[subframe_offset][i].imag<<") ";
        // }
        // cout<<endl;

        // int subframe_offset = (frame_id % TASK_BUFFER_FRAME_NUM) * OFDM_DATA_NUM ;
        // int csi_offset = ant_id + UE_id * BS_ANT_NUM;
        // float temp[2];
        // for(int j = 0; j < (OFDM_DATA_NUM); j++) {
        //     // divide fft output by pilot data to get CSI estimation
        //     float* csi_buffer_ptr = (float*)(csi_buffer_.CSI[subframe_offset + j])+csi_offset*2;
        //     // temp[0] = cur_fft_buffer_float_output[2*j]* pilots_[j];
        //     // temp[1] = cur_fft_buffer_float_output[2*j+1]* pilots_[j];  
        //     // _mm_stream_si64((long long int*)csi_buffer_ptr, *((long long int *)temp));
        //     *(csi_buffer_ptr)= cur_fft_buffer_float_output[2*j]* pilots_[j];
        //     *(csi_buffer_ptr+1)= cur_fft_buffer_float_output[2*j+1]* pilots_[j];      
        // } 
    }
    else if (pilot_symbol == 0) {
        
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
        // float *src_ptr = (float *)&fft_buffer_.FFT_outputs[tid][ant_id*OFDM_CA_NUM] + OFDM_DATA_START * 2;
        float *src_ptr = (float *)fft_buffer_.FFT_inputs[tid] + OFDM_DATA_START * 2;

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
        int cache_line_num = transpose_block_size / 8;
        int iteration_per_page = 64 / cache_line_num;
        int offset_in_page = OFDM_DATA_START / 8;
        int block_num = OFDM_DATA_NUM / transpose_block_size;
        for(int c2 = 0; c2 < block_num; c2++) {
            // c3 = 0, 1, ..., transpose_block_size/8 -1 = 7
            // c3*8 = 0, 8, ..., 64-8
            // if (c2 % iteration_per_page == 0 && c2 < block_num - iteration_per_page)
            //     float temp = *(src_ptr + 1024);
            for(int c3 = 0; c3 < cache_line_num; c3++) {
                // data: 256 bits = 32 bytes = 8 float values = 4 subcarriers

                // __m256 data = _mm256_load_ps(src_ptr);
                // original data order: SCs of ant1, SCs of ant2, ..., SCs of ant 96
                // transposed data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
                // prefetch a cache line
                // _mm_prefetch((char*)(src_ptr + 16), _MM_HINT_T0);
                float *tar_ptr_cur = tar_ptr + (c2 * BS_ANT_NUM + ant_id)* transpose_block_size * 2 + c3 * 16;
                _mm256_stream_ps(tar_ptr_cur, _mm256_load_ps(src_ptr));
                _mm256_stream_ps(tar_ptr_cur + 8, _mm256_load_ps(src_ptr + 8));
                // printf("In deFFT thread %d: frame %d, subframe %d, subcarrier %d %d, address offset: %d\n", tid, frame_id, subframe_id, c2, c3, tar_ptr_cur - src_ptr);
                src_ptr += 16;
            }            
        }        
    }
#if DEBUG_UPDATE_STATS_DETAILED
    double end_time = get_time();
    double duration3 = end_time - start_time_part3;
    if (pilot_symbol == 0)
        FFT_task_duration[tid * 8][3] += duration3;
    else
        CSI_task_duration[tid * 8][3] += duration2;
#endif    

    // after finish
    socket_buffer_[socket_thread_id].buffer_status[offset] = 0; // now empty
    // printf("In doFFT: emptied socket buffer frame: %d, subframe: %d, ant: %d, offset: %d\n",frame_id, subframe_id, ant_id, offset);
    // inform main thread
#if DEBUG_UPDATE_STATS
    double duration = get_time() - start_time;
    if (pilot_symbol == 0) {
        FFT_task_count[tid * 16] = FFT_task_count[tid * 16]+1;
        FFT_task_duration[tid * 8][0] += duration;
    }
    else {
        CSI_task_count[tid * 16] = CSI_task_count[tid * 16]+1;
        CSI_task_duration[tid * 8][0] += duration;
    }
#endif
    Event_data fft_finish_event;
    fft_finish_event.event_type = EVENT_FFT;
    fft_finish_event.data = generateOffset2d(subframe_num_perframe, frame_id, subframe_id);
    // getSubframeBufferIndex(frame_id, subframe_id);
    
    

    if ( !complete_task_queue_.enqueue(*task_ptok[tid], fft_finish_event ) ) {
        printf("fft message enqueue failed\n");
        exit(0);
    }


}


void CoMP::doZF(int tid, int offset)
{
// #if DEBUG_UPDATE_STATS
//     double start_time = get_time();
// #endif 
    int frame_id, sc_id;
    interpreteOffset2d(OFDM_DATA_NUM, offset, &frame_id, &sc_id);

#if DEBUG_PRINT_IN_TASK
        printf("In doZF thread %d: frame: %d, subcarrier: %d\n", tid, frame_id, sc_id);
#endif
    int max_sc_ite;
    if (sc_id + zf_block_size <= OFDM_DATA_NUM) 
        max_sc_ite = zf_block_size;
    else
        max_sc_ite = OFDM_DATA_NUM - sc_id;
    // i = 0, 1, ..., 31
    for (int i = 0; i < max_sc_ite; i++) {

#if DEBUG_UPDATE_STATS
        double start_time1 = get_time();
#endif 
        int cur_sc_id = sc_id + i;
        int cur_offset = offset + i;
        // // directly gather data from FFT buffer
        // __m256i index = _mm256_setr_epi32(0, 1, OFDM_CA_NUM * 2, OFDM_CA_NUM * 2 + 1, OFDM_CA_NUM * 4, OFDM_CA_NUM * 4 + 1, OFDM_CA_NUM * 6, OFDM_CA_NUM * 6 + 1);
        __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2, transpose_block_size * 2 + 1, transpose_block_size * 4, transpose_block_size * 4 + 1, transpose_block_size * 6, transpose_block_size * 6 + 1);
        // __m256i index = _mm256_setr_epi64x(0, transpose_block_size/2, transpose_block_size/2 * 2, transpose_block_size/2 * 3);

        int transpose_block_id = cur_sc_id / transpose_block_size;
        int sc_inblock_idx = cur_sc_id % transpose_block_size;
        int offset_in_csi_buffer = transpose_block_id * BS_ANT_NUM * transpose_block_size  + sc_inblock_idx;
        // double *tar_csi_ptr = (double *)csi_gather_buffer[tid];
        int subframe_offset = frame_id * UE_NUM;
        // int subframe_offset = frame_id * subframe_num_perframe;
        float *tar_csi_ptr = (float *)csi_gather_buffer[tid];

        // // if (sc_id == 4) {
        // //     cout<<"csi_buffer_ for subframe "<<subframe_offset<<endl;
        // //     for (int i=0;i<BS_ANT_NUM*OFDM_CA_NUM; i++) {
        // //         cout<<"("<<i<<",  "<<csi_buffer_.CSI[subframe_offset][i].real<<","<<csi_buffer_.CSI[subframe_offset][i].imag<<") ";
        // //     }

        // //     cout<<endl;
        // // }

        // gather data for all users and antennas
        // printf("In doZF thread %d: frame: %d, subcarrier: %d\n", tid, frame_id, sc_id);
        int mat_elem = UE_NUM * BS_ANT_NUM;
        int cache_line_num = mat_elem / 8;
        // for (int line_idx = 0; line_idx < cache_line_num; line_idx ++) {
        //     _mm_prefetch((char *)precoder_buffer_.precoder[cur_offset + 8 * line_idx], _MM_HINT_ET1);
        //     // _mm_prefetch((char *)(tar_csi_ptr + 16 * line_idx), _MM_HINT_ET1);
        // }

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

    #if DEBUG_UPDATE_STATS_DETAILED
        double duration1 = get_time() - start_time1;
        ZF_task_duration[tid * 8][1] += duration1;
        // double start_time2 = get_time();
    #endif
        cx_float *ptr_in = (cx_float *)csi_gather_buffer[tid];
        // cx_float *ptr_in = (cx_float *)csi_buffer_.CSI[offset];
        cx_fmat mat_input(ptr_in, BS_ANT_NUM, UE_NUM, false);
        // cout<<"CSI matrix"<<endl;
        // cout<<mat_input.st()<<endl;
        // cx_fmat mat_input(ptr_in, UE_NUM, BS_ANT_NUM, false);
        // cx_float *ptr_out = (cx_float *)precoder_buffer_temp[tid];
        cx_float *ptr_out = (cx_float *)precoder_buffer_.precoder[cur_offset];
        cx_fmat mat_output(ptr_out, UE_NUM, BS_ANT_NUM, false);
        // cx_fmat mat_output(ptr_out, BS_ANT_NUM, UE_NUM, false);

    #if DEBUG_UPDATE_STATS_DETAILED
        double start_time2 = get_time();
        double duration2 = start_time2 - start_time1;
        ZF_task_duration[tid * 8][2] += duration2;
        // double start_time2 = get_time();
    #endif
   
        
        pinv(mat_output, mat_input, 1e-1, "dc");

        // cout<<"Precoder:" <<mat_output<<endl;
    #if DEBUG_UPDATE_STATS_DETAILED    
        double duration3 = get_time() - start_time2;
        ZF_task_duration[tid * 8][3] += duration3;
        // double start_time3 = get_time();
    #endif
        
    
        // float *tar_ptr = (float *)precoder_buffer_.precoder[cur_offset];
        // // float temp = *tar_ptr;
        // float *src_ptr = (float *)ptr_out;

        // // int mat_elem = UE_NUM * BS_ANT_NUM;
        // // int cache_line_num = mat_elem / 8;
        // for (int line_idx = 0; line_idx < cache_line_num; line_idx++) {
        //     _mm256_stream_ps(tar_ptr, _mm256_load_ps(src_ptr));
        //     _mm256_stream_ps(tar_ptr + 8, _mm256_load_ps(src_ptr + 8));
        //     tar_ptr += 16;
        //     src_ptr += 16;
        // }
    // #if DEBUG_UPDATE_STATS    
    //     double duration3 = get_time() - start_time3;
    //     ZF_task_duration[tid][3] += duration3;
    // #endif
    #if DEBUG_UPDATE_STATS   
        ZF_task_count[tid * 16] = ZF_task_count[tid * 16]+1; 
        double duration = get_time() - start_time1;
        ZF_task_duration[tid * 8][0] += duration;
    #endif

    }



    // inform main thread
    Event_data ZF_finish_event;
    ZF_finish_event.event_type = EVENT_ZF;
    ZF_finish_event.data = offset;
    

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

    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
    interpreteOffset3d(OFDM_DATA_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);
    int subframe_offset = subframe_num_perframe * frame_id + UE_NUM + current_data_subframe_id;

#if DEBUG_UPDATE_STATS    
    double start_time = get_time();
#endif
    

#if DEBUG_PRINT_IN_TASK
        printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,sc_id);
#endif

    __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2, transpose_block_size * 2 + 1, transpose_block_size * 4, transpose_block_size * 4 + 1, transpose_block_size * 6, transpose_block_size * 6 + 1);
    int gather_step_size = 8 * transpose_block_size;

    int mat_elem = UE_NUM * BS_ANT_NUM;
    int cache_line_num = mat_elem / 8;
    int ue_data_cache_line_num = UE_NUM/8;
    int max_sc_ite;
    if (sc_id + demul_block_size <= OFDM_DATA_NUM) 
        max_sc_ite = demul_block_size;
    else
        max_sc_ite = OFDM_DATA_NUM - sc_id;
    /* i = 0, 1, ..., 32/8
     * iterate through cache lines (each cache line has 8 subcarriers) */
    for (int i = 0; i < max_sc_ite/8; i++) {        
        /* for a cache line size of 64 bytes, each read load 8 subcarriers
         * use spatial locality to reduce latency */
    #if DEBUG_UPDATE_STATS_DETAILED    
        double start_time1 = get_time();
    #endif

        // for (int j = 0; j < 1; j++) {
        //     int cur_sc_id = i * 8 + j + sc_id;
        //     int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
        //     cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];
        //     for (int line_idx = 0; line_idx < cache_line_num; line_idx ++) {
        //         _mm_prefetch((char *)(precoder_ptr + 8 * line_idx), _MM_HINT_T2);
        //     }
        // }


        int cur_block_id = (sc_id + i * 8) / transpose_block_size;
        int sc_inblock_idx = (i * 8) % transpose_block_size;
        int cur_sc_offset = cur_block_id * transpose_block_size * BS_ANT_NUM + sc_inblock_idx;
        float *tar_data_ptr = (float *)spm_buffer[tid];        
        float *src_data_ptr = (float *)data_buffer_.data[total_data_subframe_id] + cur_sc_offset * 2;

        /* gather data for all antennas and 8 subcarriers in the same cache line */
        for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
            /* 1 subcarrier and 4 ants per iteration */
            for (int j = 0; j < 8; j++) {
                __m256 data_rx = _mm256_i32gather_ps(src_data_ptr + j * 2, index, 4);
                _mm256_store_ps(tar_data_ptr + j * BS_ANT_NUM * 2, data_rx);
                // printf("Frame %d, sc: %d, UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", frame_id, sc_id, ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float *)tar_csi_ptr+1), 
                //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),  *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));   
            }
            src_data_ptr += gather_step_size;
            tar_data_ptr += 8;
        }
    #if DEBUG_UPDATE_STATS_DETAILED   
        double duration1 = get_time() - start_time1;
        Demul_task_duration[tid * 8][1] += duration1;
    #endif

        // Demul_task_duration_part1[tid] += get_time() - start_time1;
        
        /* perform computation for 8 subcarriers */
        for (int j = 0; j < 8; j++) {
            /* create input data matrix */
            cx_float* data_ptr = (cx_float *)(spm_buffer[tid] + j * BS_ANT_NUM);
            cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);

            /* create input precoder matrix */
            int cur_sc_id = i * 8 + j + sc_id;
            int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
            cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];
            // for (int line_idx = 0; line_idx < cache_line_num; line_idx ++) {
            //     _mm_prefetch((char *)(precoder_ptr + 8 * line_idx), _MM_HINT_NTA);
            // }
            cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);


            // cout<<"Precoder: "<< mat_precoder<<endl;

            /* create output matrix for equalization */ 
#if EXPORT_CONSTELLATION
            cx_float* equal_ptr = (cx_float *)(&equal_buffer_.data[total_data_subframe_id][cur_sc_id * UE_NUM]);
#else
            cx_float* equal_ptr = (cx_float *)(&equaled_buffer_temp[tid][(cur_sc_id-sc_id) * UE_NUM]);
#endif
            cx_fmat mat_equaled(equal_ptr, UE_NUM, 1, false);

#if DEBUG_UPDATE_STATS_DETAILED    
    double start_time2 = get_time();
#endif      
            /* perform computation for equalization */
            mat_equaled = mat_precoder * mat_data;


#if DEBUG_UPDATE_STATS_DETAILED   
    double start_time3 = get_time(); 
    double duration2 = get_time() - start_time2;   
    Demul_task_duration[tid * 8][2] += duration2;
#endif
            // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d, sc_id: %d \n", tid, frame_id, current_data_subframe_id,cur_sc_id, sc_id);
            // cout << "Equaled data: "<<mat_equaled.st()<<endl;
            
#if !ENABLE_DECODE 
            /* decode with hard decision */
            uint8_t *demul_ptr = (&demul_hard_buffer_[total_data_subframe_id][cur_sc_id * UE_NUM]);
            // if (i * 8 + j < max_sc_ite -1)
            //     _mm_prefetch((char *)(demul_ptr+UE_NUM), _MM_HINT_T1);
            // demod_16qam_loop((float *)equal_ptr, demul_ptr, UE_NUM);
            demod_16qam_loop((float *)equal_ptr, (uint8_t *)data_ptr, UE_NUM);
#if DEBUG_UPDATE_STATS_DETAILED   
    double duration3 = get_time() - start_time3;   
    Demul_task_duration[tid * 8][3] += duration3;
#endif

            // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d, sc_id: %d \n", tid, frame_id, current_data_subframe_id,cur_sc_id, sc_id);
            // cout<< "Demuled data: ";
            // for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
            //     // cout<<demul_hard_buffer_[total_data_subframe_id][cur_sc_id * UE_NUM+ue_idx]<<" "<<endl;
            //     cout<<+*(demul_ptr+ue_idx)<<"  ";
            // }
            // cout<<endl; 
#endif      
        #if DEBUG_UPDATE_STATS    
            Demul_task_count[tid * 16] = Demul_task_count[tid * 16] + 1;
        #endif
        }

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



//     for (int i = 0; i < max_sc_ite; i++) {        
//     #if DEBUG_UPDATE_STATS_DETAILED    
//         double start_time1 = get_time();
//     #endif
//         int cur_block_id = (sc_id + i) / transpose_block_size;
//         int sc_inblock_idx = i % transpose_block_size;
//         int cur_sc_offset = cur_block_id * transpose_block_size * BS_ANT_NUM + sc_inblock_idx;
//         float *tar_data_ptr = (float *)spm_buffer[tid];        
//         float *src_data_ptr = (float *)data_buffer_.data[total_data_subframe_id] + cur_sc_offset * 2;
//         for (int c2 = 0; c2 < BS_ANT_NUM / 4; c2++) {
//             __m256 data_rx = _mm256_i32gather_ps(src_data_ptr, index, 4);
//             _mm256_store_ps(tar_data_ptr, data_rx);
//             // printf("Frame %d, sc: %d, UE %d, ant %d, data: %.4f, %.4f, %.4f, %.4f, %.4f, %.4f\n", frame_id, sc_id, ue_idx, ant_idx, *((float *)tar_csi_ptr), *((float *)tar_csi_ptr+1), 
//                 //         *((float *)tar_csi_ptr+2), *((float *)tar_csi_ptr+3),  *((float *)tar_csi_ptr+4), *((float *)tar_csi_ptr+5));           
//             src_data_ptr += gather_step_size;
//             tar_data_ptr += 8;
//         }
       
//     #if DEBUG_UPDATE_STATS_DETAILED   
//         double start_time2 = get_time();
//         double duration1 = start_time2 - start_time1;
//         Demul_task_duration[tid * 8][1] += duration1;
//     #endif

//         // Demul_task_duration_part1[tid] += get_time() - start_time1;
        
//         /* perform computation for 8 subcarriers */
        
// // #if DEBUG_UPDATE_STATS_DETAILED    
// //     double start_time2 = get_time();
// // #endif

//         /* create input data matrix */
//         cx_float* data_ptr = (cx_float *)(spm_buffer[tid]);
//         cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);

//         /* create input precoder matrix */
//         int cur_sc_id = i + sc_id;
//         int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
//         cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];
//         // for (int line_idx = 0; line_idx < cache_line_num; line_idx ++) {
//         //     _mm_prefetch((char *)(precoder_ptr + 8 * line_idx), _MM_HINT_NTA);
//         // }
//         cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);
//         // cout<<"Precoder: "<< mat_precoder<<endl;

//         /* create output matrix for equalization */ 
// #if EXPORT_CONSTELLATION
//         cx_float* equal_ptr = (cx_float *)(&equal_buffer_.data[total_data_subframe_id][cur_sc_id * UE_NUM]);
// #else
//         cx_float* equal_ptr = (cx_float *)(&equaled_buffer_temp[tid][(cur_sc_id-sc_id) * UE_NUM]);
// #endif
//         cx_fmat mat_equaled(equal_ptr, UE_NUM, 1, false);
      
//         /* perform computation for equalization */
//         mat_equaled = mat_precoder * mat_data;

// #if DEBUG_UPDATE_STATS_DETAILED   
//     double start_time3 = get_time(); 
//     double duration2 = start_time3 - start_time2;   
//     Demul_task_duration[tid * 8][2] += duration2;
// #endif
// #if !ENABLE_DECODE 
//         /* decode with hard decision */
//         uint8_t *demul_ptr = (&demul_hard_buffer_[total_data_subframe_id][cur_sc_id * UE_NUM]);
//         demod_16qam_loop((float *)equal_ptr, demul_ptr, UE_NUM);
// #if DEBUG_UPDATE_STATS_DETAILED   
//     double duration3 = get_time() - start_time3;   
//     Demul_task_duration[tid * 8][3] += duration3;
// #endif
//         // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d, sc_id: %d \n", tid, frame_id, current_data_subframe_id,cur_sc_id, sc_id);
//         // cout<< "Demuled data: ";
//         // for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
//         //     // cout<<demul_hard_buffer_[total_data_subframe_id][cur_sc_id * UE_NUM+ue_idx]<<" "<<endl;
//         //     cout<<+*(demul_ptr+ue_idx)<<"  ";
//         // }
//         // cout<<endl; 
// #endif      
//         #if DEBUG_UPDATE_STATS    
//             Demul_task_count[tid * 16] = Demul_task_count[tid * 16] + 1;
//         #endif
        
//     }




#if ENABLE_DECODE
    __m256i index_out = _mm256_setr_epi32(0, 1, UE_NUM * 2, UE_NUM * 2 + 1, UE_NUM * 4, UE_NUM * 4 + 1, UE_NUM * 6, UE_NUM * 6 + 1);

    #if COMBINE_EQUAL_DECODE
        if (sc_id < MAX_CODED_SC) {
            // int cache_line_num = max_sc_ite * sizeof(float) * 2 / 64;
            float *temp_ptr = (float *)&equaled_buffer_temp[tid][0];
            // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d\n", tid, frame_id, current_data_subframe_id, sc_id);
            // cout<< "Equalized data: ";
            // for (int sc_idx = 0; sc_idx < CODED_LEN * UE_NUM; sc_idx++) {
            //     cout<<*(temp_ptr+sc_idx)<<"  ";
            // }
            // cout<<endl;
            
            for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
                float *src_ptr = (float *)&equaled_buffer_temp[tid][ue_idx];
                for (int j = 0; j < max_sc_ite / 4; j++) {
                    float *tar_ptr = (float *)(&equaled_buffer_T_temp[tid][j * 4 + ue_idx * max_sc_ite]);
                    __m256 equaled_data = _mm256_i32gather_ps(src_ptr + j * 8 * UE_NUM, index_out, 4);
                    _mm256_store_ps(tar_ptr, equaled_data);
                }
                
            }
            // float *equal_t_ptr = (float *)(&equaled_buffer_T_temp[tid][ue_idx * max_sc_ite]);
            float *equal_t_ptr = (float *)(&equaled_buffer_T_temp[tid][0]);
            float *demul_ptr = (&demul_soft_buffer_temp[tid][0]);
            demod_16qam_soft(equal_t_ptr, demul_ptr, max_sc_ite * UE_NUM );
            int block_id = sc_id / demul_block_size;
            // int *decoded_buf_ptr = &decoded_buffer_[total_data_subframe_id][block_id * ORIG_CODE_LEN * NUM_BITS + ue_idx * ORIG_CODE_LEN * NUM_CODE_BLOCK * NUM_BITS];
            int *decoded_buf_ptr = &decoded_buffer_[total_data_subframe_id][block_id * ORIG_CODE_LEN * NUM_BITS * UE_NUM];
            Decoders[tid]->decode_siho(demul_ptr, decoded_buf_ptr);
            // if (tid == 10) {
            //     printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d\n", tid, frame_id, current_data_subframe_id, sc_id);
            //     // cout<< "Equalized data: ";
            //     // for (int sc_idx = 0; sc_idx < CODED_LEN * UE_NUM; sc_idx++) {
            //     //     cout<<*(equal_t_ptr+sc_idx)<<"  ";
            //     // }
            //     // cout<<endl;
            //     // cout<< "Demodulated data: ";
            //     // for (int sc_idx = 0; sc_idx < ORIG_CODE_LEN * NUM_BITS * UE_NUM; sc_idx++) {
            //     //     cout<<*(demul_ptr+sc_idx)<<"  ";
            //     // }
            //     // cout<<endl;
            //     cout<< "Decoded data after decode: ";
            //     for (int sc_idx = 0; sc_idx < ORIG_CODE_LEN * NUM_BITS * UE_NUM; sc_idx++) {
            //         cout<<*(decoded_buf_ptr+sc_idx)<<"  ";
            //     }
            //     cout<<endl;
            // }
        }
    #else          
        if (sc_id < MAX_CODED_SC) {
            int cache_line_num = max_sc_ite * sizeof(float) * 2 / 64;
            for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
                float *src_ptr = (float *)&equaled_buffer_temp[tid][ue_idx];
                float *tar_ptr =  (float *)&equal_buffer_.data[total_data_subframe_id][sc_id + ue_idx * OFDM_DATA_NUM];
                for (int j = 0; j < max_sc_ite / 4; j++) {
                    __m256 equaled_data = _mm256_i32gather_ps(src_ptr + j * 8 * UE_NUM, index_out, 4);
                    _mm256_stream_ps(tar_ptr + j * 8, equaled_data);

                }
            }
        }
    #endif
#endif


    // inform main thread
#if DEBUG_UPDATE_STATS   
    double duration = get_time() - start_time;
    Demul_task_duration[tid * 8][0] += duration;
#endif
    Event_data demul_finish_event;
    demul_finish_event.event_type = EVENT_DEMUL;
    demul_finish_event.data = offset;
    

    if ( !complete_task_queue_.enqueue(*task_ptok[tid], demul_finish_event ) ) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }  

      
}

void CoMP::doDemulSingleSC(int tid, int offset)
{
    double start_time = get_time();
    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
    interpreteOffset3d(OFDM_DATA_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);
    int subframe_offset = subframe_num_perframe * frame_id + UE_NUM + current_data_subframe_id;
    
    int gather_step_size = 8 * transpose_block_size;

#if DEBUG_PRINT_IN_TASK
        printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,sc_id);
#endif

    __m256i index = _mm256_setr_epi32(0, 1, transpose_block_size * 2, transpose_block_size * 2 + 1, transpose_block_size * 4, transpose_block_size * 4 + 1, transpose_block_size * 6, transpose_block_size * 6 + 1);



    int cur_block_id = sc_id / transpose_block_size;
    int sc_inblock_idx = sc_id % transpose_block_size;
    int cur_sc_offset = cur_block_id * transpose_block_size * BS_ANT_NUM + sc_inblock_idx;
    float *tar_data_ptr = (float *)spm_buffer[tid];        
    float *src_data_ptr = (float *)data_buffer_.data[total_data_subframe_id] + cur_sc_offset * 2;
    for (int ant_idx = 0; ant_idx < BS_ANT_NUM; ant_idx += 4) {
        __m256 data_rx = _mm256_i32gather_ps(src_data_ptr, index, 4);
        _mm256_store_ps(tar_data_ptr, data_rx);     
        src_data_ptr += gather_step_size;
        tar_data_ptr += 8;
    }

    // mat_data size: BS_ANT_NUM \times 1
    cx_float* data_ptr = (cx_float *)(spm_buffer[tid]);
    cx_fmat mat_data(data_ptr, BS_ANT_NUM, 1, false);
    // cout<< "Raw data: " << mat_data.st()<<endl;

    // mat_precoder size: UE_NUM \times BS_ANT_NUM
    int precoder_offset = frame_id * OFDM_DATA_NUM + sc_id;
    cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];


    cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);
    // cout<<"Precoder: "<< mat_precoder<<endl;

    // mat_demuled size: UE_NUM \times 1
    cx_float* equal_ptr = (cx_float *)(&equal_buffer_.data[total_data_subframe_id][sc_id * UE_NUM]);
    cx_fmat mat_equaled(equal_ptr, UE_NUM, 1, false);

    // Demodulation
    // sword* demul_ptr = (sword *)(&demul_hard_buffer_.data[total_data_subframe_id][sc_id * UE_NUM]);
    // imat mat_demuled(demul_ptr, UE_NUM, 1, false);
    uint8_t *demul_ptr = (&demul_hard_buffer_[total_data_subframe_id][sc_id * UE_NUM]);
    
    // Equalization
    mat_equaled = mat_precoder * mat_data;
    // cout << "Equaled data: "<<mat_equaled.st()<<endl;

    

    // Hard decision
    demod_16qam_loop((float *)equal_ptr, demul_ptr, UE_NUM);
    // printf("In doDemul thread %d: frame: %d, subframe: %d, subcarrier: %d \n", tid, frame_id, current_data_subframe_id,sc_id);
    // cout<< "Demuled data: ";
    // for (int ue_idx = 0; ue_idx < UE_NUM; ue_idx++) {
    //     cout<<*(demul_ptr+ue_idx)<<"  ";
    // }
    // cout<<endl;

    // inform main thread
    double duration3 = get_time() - start_time;
    Demul_task_duration[tid][1] += duration3;
    Event_data demul_finish_event;
    demul_finish_event.event_type = EVENT_DEMUL;
    demul_finish_event.data = offset;
    Demul_task_count[tid] = Demul_task_count[tid]+1;

    if ( !complete_task_queue_.enqueue(*task_ptok[tid], demul_finish_event ) ) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }  
    double duration = get_time() - start_time;
    Demul_task_duration[tid][0] += duration;
      
}


void CoMP::doDecode(int tid, int offset) 
{
    int frame_id, total_data_subframe_id, current_data_subframe_id, user_offset;
    interpreteOffset3d(OFDM_DATA_NUM * UE_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &user_offset);
    int user_id = user_offset % CODED_LEN;
    int block_id = user_offset / CODED_LEN;
#if DEBUG_PRINT_IN_TASK
    printf("In doDecode thread %d: frame: %d, subframe: %d, block: %d, user: %d\n", tid, frame_id, current_data_subframe_id,block_id, user_id);
#endif
    float *equal_ptr = (float *)(&equal_buffer_.data[total_data_subframe_id][block_id * CODED_LEN + user_id * OFDM_DATA_NUM]);
    float *demul_ptr = (&demul_soft_buffer_temp[tid][0]);
    // Soft decision
    // Modems[tid]->demodulate(equal_ptr, demul_ptr);
    demod_16qam_soft(equal_ptr, demul_ptr, CODED_LEN * 2);

    // printf("In doDecode thread %d: frame: %d, subframe: %d, code block: %d user: %d\n", tid, frame_id, current_data_subframe_id,block_id, user_id);


    // cout<< "Demodulated data: ";
    // for (int sc_idx = 0; sc_idx < CODED_LEN*NUM_BITS; sc_idx++) {
    //     cout<<*(demul_ptr+sc_idx)<<"  ";
    // }
    // cout<<endl;

    int *decoded_buf_ptr = &decoded_buffer_[total_data_subframe_id][block_id * ORIG_CODE_LEN * NUM_BITS + user_id * ORIG_CODE_LEN * NUM_CODE_BLOCK * NUM_BITS];

    Decoders[tid]->decode_siho(demul_ptr, decoded_buf_ptr);

    // cout<< "Decoded data after decode: ";
    // for (int sc_idx = 0; sc_idx < ORIG_CODE_LEN * NUM_BITS; sc_idx++) {
    //     cout<<*(decoded_buf_ptr+sc_idx)<<"  ";
    // }
    // cout<<endl;


    Event_data decode_finish_event;
    decode_finish_event.event_type = EVENT_DECODE;
    decode_finish_event.data = offset;

    if ( !complete_task_queue_.enqueue(*task_ptok[tid], decode_finish_event ) ) {
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

    printf("In doModulate thread %d: frame: %d, subframe: %d, user: %d\n", tid, frame_id, current_data_subframe_id, user_id);
    int *input_ptr = &dl_IQ_data[current_data_subframe_id * UE_NUM + user_id][0];
    complex_float *output_ptr = &dl_modulated_buffer_.data[total_data_subframe_id][0];
    for (int i = 0; i < OFDM_DATA_NUM; i++) {
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
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
    int frame_id, total_data_subframe_id, current_data_subframe_id, sc_id;
    interpreteOffset3d(OFDM_DATA_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &sc_id);

    // printf("In doPrecode thread %d: frame: %d, subframe: %d, subcarrier: %d\n", tid, frame_id, current_data_subframe_id, sc_id);
    // double start_time = get_time();
    for (int i = 0; i < demul_block_size; i++) { 
        int cur_sc_id = sc_id + i;

        complex_float *data_ptr = &dl_modulated_buffer_.data[total_data_subframe_id][UE_NUM * cur_sc_id];
        for (int user_id = 0; user_id < UE_NUM; user_id ++) {
            int *raw_data_ptr = &dl_IQ_data[current_data_subframe_id * UE_NUM + user_id][cur_sc_id];
            // cout<<*raw_data_ptr<<", ";
            *(data_ptr + user_id) = mod_16qam_single(*(raw_data_ptr));
            // cout<<(*(data_ptr + user_id)).real<<"+"<<(*(data_ptr + user_id)).imag<<"j, ";
        }
        // cout<<endl;

        int precoder_offset = frame_id * OFDM_DATA_NUM + cur_sc_id;
        // mat_precoder size: UE_NUM \times BS_ANT_NUM        
        cx_float* precoder_ptr = (cx_float *)precoder_buffer_.precoder[precoder_offset];
        cx_fmat mat_precoder(precoder_ptr, UE_NUM, BS_ANT_NUM, false);

        // mat_data size: UE_NUM \times 1
        // cx_float* data_ptr = (cx_float *)(&dl_modulated_buffer_.data[total_data_subframe_id][UE_NUM * (sc_id+i)]);
        cx_fmat mat_data((cx_float *)data_ptr, UE_NUM, 1, false);
        // cout << "Frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", SC: " << sc_id+i << ", data: " << real(mat_data).st() << endl;

        // mat_precoded size: BS_ANT_NUM \times 1
        cx_float* precoded_ptr = (cx_float *)(&dl_precoded_data_buffer_.data[total_data_subframe_id][cur_sc_id * BS_ANT_NUM]);
        cx_fmat mat_precoded(precoded_ptr, BS_ANT_NUM, 1, false);

        mat_precoded = mat_precoder.st() * mat_data;
        // cout<<"Precoder: \n"<<mat_precoder<<endl;
        // cout<<"Precoder transposed: \n"<<mat_precoder.st()<<endl;
        // cout<<"Data: "<<mat_data<<endl;
        // cout << "Frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", SC: " << sc_id+i << ", data: " << real(mat_precoded).st() << endl;
        // cout << "Precoded data:" ;
        // for (int j = 0; j < BS_ANT_NUM; j++) {
        //     cout <<*((float *)(precoded_ptr+j)) << "+j"<<*((float *)(precoded_ptr+j)+1)<<",   ";
        // }
        // cout<<endl;
    }

    // copy data to ifft input
    // int cache_line_num = transpose_block_size / 8;
    // int iteration_per_page = 64 / cache_line_num;
    // int offset_in_page = OFDM_DATA_START / 8;
    // int block_num = OFDM_DATA_NUM / transpose_block_size;

    // for(int c2 = 0; c2 < block_num; c2++) {
    //     // c3 = 0, 1, ..., transpose_block_size/8 -1 = 7
    //     // c3*8 = 0, 8, ..., 64-8
    //     if (c2 % iteration_per_page == 0 && c2 < block_num - iteration_per_page)
    //         float temp = *(src_ptr + 1024);
    //     for(int c3 = 0; c3 < cache_line_num; c3++) {
    //         // data: 256 bits = 32 bytes = 8 float values = 4 subcarriers

    //         // __m256 data = _mm256_load_ps(src_ptr);
    //         // original data order: SCs of ant1, SCs of ant2, ..., SCs of ant 96
    //         // transposed data order: SC1-32 of ants, SC33-64 of ants, ..., SC993-1024 of ants (32 blocks each with 32 subcarriers)
    //         // prefetch a cache line
    //         _mm_prefetch((char*)(src_ptr + 16), _MM_HINT_T0);
    //         float *tar_ptr_cur = tar_ptr + (c2 * BS_ANT_NUM + ant_id)* transpose_block_size * 2 + c3 * 16;
    //         _mm256_stream_ps(tar_ptr_cur, _mm256_load_ps(src_ptr));
    //         _mm256_stream_ps(tar_ptr_cur + 8, _mm256_load_ps(src_ptr + 8));
    //         // printf("In deFFT thread %d: frame %d, subframe %d, subcarrier %d %d, address offset: %d\n", tid, frame_id, subframe_id, c2, c3, tar_ptr_cur - src_ptr);
    //         src_ptr += 16;
    //     }            
    // } 
    __m256i index = _mm256_setr_epi64x(0, BS_ANT_NUM, BS_ANT_NUM * 2, BS_ANT_NUM * 3);
    float* precoded_ptr = (float *)&dl_precoded_data_buffer_.data[total_data_subframe_id][sc_id * BS_ANT_NUM];
    for (int ant_id = 0; ant_id < BS_ANT_NUM; ant_id++) {
        int ifft_buffer_offset = generateOffset3d(BS_ANT_NUM, frame_id, current_data_subframe_id, ant_id);
        float* ifft_ptr = (float *)&dl_ifft_buffer_.IFFT_inputs[ifft_buffer_offset][sc_id + OFDM_DATA_START];
        for (int i = 0; i< demul_block_size/4; i++) {
            float *input_shifted_ptr = precoded_ptr + 4 * i * 2 * BS_ANT_NUM + ant_id * 2;
            __m256d t_data = _mm256_i64gather_pd((double *)input_shifted_ptr, index, 8);
            _mm256_stream_pd((double *)(ifft_ptr + i * 8), t_data);
        }
    }

#if DEBUG_UPDATE_STATS
    Precode_task_count[tid*16] = Precode_task_count[tid*16]+1;
    Precode_task_duration[tid*8][0] += get_time() - start_time;
#endif
    // inform main thread
    Event_data precode_finish_event;
    precode_finish_event.event_type = EVENT_PRECODE;
    precode_finish_event.data = offset;
    
    if ( !complete_task_queue_.enqueue(*task_ptok[tid], precode_finish_event ) ) {
        printf("Precoding message enqueue failed\n");
        exit(0);
    }
#if DEBUG_PRINT_IN_TASK
    printf("In doPrecode thread %d: finished frame: %d, subframe: %d, subcarrier: %d , offset: %d\n", tid, 
        frame_id, current_data_subframe_id, sc_id, offset);
#endif

}

void CoMP::do_ifft(int tid, int offset)
{
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
    int frame_id, total_data_subframe_id, current_data_subframe_id, ant_id;
    interpreteOffset3d(BS_ANT_NUM, offset, &frame_id, &total_data_subframe_id, &current_data_subframe_id, &ant_id);
#if DEBUG_PRINT_IN_TASK
        printf("In doIFFT thread %d: frame: %d, subframe: %d, antenna: %d\n", tid, frame_id, current_data_subframe_id, ant_id);
#endif

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id << ", input data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     cout << dl_ifft_buffer_.IFFT_inputs[offset][j].real << "+" << dl_ifft_buffer_.IFFT_inputs[offset][j].imag << "j,   ";
    // }
    // cout<<"\n\n"<<endl;
    // mufft_execute_plan_1d(muplans_ifft_[tid], dl_ifft_buffer_.IFFT_outputs[offset], 
    //     dl_ifft_buffer_.IFFT_inputs[offset]);

    DftiComputeBackward(mkl_handles_dl[tid],dl_ifft_buffer_.IFFT_inputs[offset]);

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id <<", offset: "<<offset <<", output data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     cout << dl_ifft_buffer_.IFFT_outputs[offset][j].real << "+" << dl_ifft_buffer_.IFFT_outputs[offset][j].imag << "j,   ";
    // }
    // cout<<"\n\n"<<endl;

    // calculate data for downlink socket buffer 
    float *ifft_output_ptr = (float *)(&dl_ifft_buffer_.IFFT_inputs[offset][0]);
    int socket_subframe_offset = (frame_id % SOCKET_BUFFER_FRAME_NUM) * data_subframe_num_perframe + current_data_subframe_id;
    char *socket_ptr = &dl_socket_buffer_.buffer[socket_subframe_offset * BS_ANT_NUM * PackageReceiver::package_length];

    for (int sc_id = 0; sc_id < OFDM_CA_NUM; sc_id++) {
        float *shifted_input_ptr = (float *)(ifft_output_ptr + 2 * sc_id);
        int socket_offset = sizeof(int) * 16 + ant_id * PackageReceiver::package_length;
        // ifft scaled results by 2048, 16 = 2^15/2048
        *((short *)(socket_ptr + socket_offset) + 2 * sc_id ) = (short)(*shifted_input_ptr * 16);
        *((short *)(socket_ptr + socket_offset) + 2 * sc_id + 1 ) = (short)(*(shifted_input_ptr+1) * 16);
    }

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<< current_data_subframe_id<<", ant: " << ant_id << ", data: ";
    // for (int j = 0; j <OFDM_CA_NUM; j++) {
    //     int socket_offset = sizeof(int) * 16 + ant_id * PackageReceiver::package_length;
    //     cout <<*((short *)(socket_ptr + socket_offset) + 2 * j)  << "+j"<<*((short *)(socket_ptr + socket_offset) + 2 * j + 1 )<<",   ";
    // }
    // cout<<"\n\n"<<endl;

#if DEBUG_UPDATE_STATS   
    IFFT_task_count[tid*16] = IFFT_task_count[tid*16]+1;
    IFFT_task_duration[tid*8][0] += get_time() - start_time;
#endif

    // inform main thread
    Event_data ifft_finish_event;
    ifft_finish_event.event_type = EVENT_IFFT;
    ifft_finish_event.data = offset;

    if ( !complete_task_queue_.enqueue(*task_ptok[tid], ifft_finish_event ) ) {
        printf("IFFT message enqueue failed\n");
        exit(0);
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





