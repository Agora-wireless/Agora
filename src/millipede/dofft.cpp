/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "dofft.hpp"
#include "Consumer.hpp"

DoFFT::DoFFT(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    Consumer& in_consumer, Table<char>& in_socket_buffer,
    Table<int>& in_socket_buffer_status, Table<complex_short>& in_data_buffer,
    Table<complex_short>& in_csi_buffer, Table<complex_short>& in_calib_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, in_consumer)
    , socket_buffer_(in_socket_buffer)
    , socket_buffer_status_(in_socket_buffer_status)
    , data_buffer_(in_data_buffer)
    , csi_buffer_(in_csi_buffer)
    , calib_buffer_(in_calib_buffer)
    , FFT_task_duration(&in_stats_manager->fft_stats_worker.task_duration)
    , FFT_task_count(in_stats_manager->fft_stats_worker.task_count)
    , CSI_task_duration(&in_stats_manager->csi_stats_worker.task_duration)
    , CSI_task_count(in_stats_manager->csi_stats_worker.task_count)
{
    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->OFDM_CA_NUM);
    // auto mkl_status = DftiSetValue(mkl_handle, DFTI_PLACEMENT,
    // DFTI_NOT_INPLACE);
    (void)DftiCommitDescriptor(mkl_handle);

    int FFT_buffer_block_num = 1;
    fft_buffer_.FFT_inputs.calloc(FFT_buffer_block_num, cfg->OFDM_CA_NUM, 64);
    fft_buffer_.FFT_outputs.calloc(FFT_buffer_block_num, cfg->OFDM_CA_NUM, 64);
    alloc_buffer_1d(&fft_buf_temp, cfg->OFDM_CA_NUM * 2, 64, 1);
}

DoFFT::~DoFFT()
{
    DftiFreeDescriptor(&mkl_handle);
    fft_buffer_.FFT_inputs.free();
    fft_buffer_.FFT_outputs.free();
}

Event_data DoFFT::launch(int tag)
{
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif

    int socket_thread_id = fft_req_tag_t(tag).tid;
    size_t buf_offset = fft_req_tag_t(tag).offset;

    /* read info of one frame */
    int packet_length = cfg->packet_length;
    char* cur_buffer_ptr
        = socket_buffer_[socket_thread_id] + buf_offset * packet_length;
    struct Packet* pkt = (struct Packet*)cur_buffer_ptr;
    size_t frame_id = pkt->frame_id % 10000;
    size_t subframe_id = pkt->symbol_id;
    size_t ant_id = pkt->ant_id;

    short* cur_buffer_ptr_ushort = &pkt->data[2 * cfg->OFDM_PREFIX_LEN];
    run_fft((vcs*)cur_buffer_ptr_ushort, (vcs*)fft_buf_temp, cfg->OFDM_CA_NUM);

    // printf("In doFFT thread %d: frame: %d, subframe: %d, ant: %d\n", tid,
    //     frame_id % TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
    // printf("FFT output\n");
    // for (int i = 0; i < cfg->OFDM_CA_NUM; i++) {
    //     printf("%d+%dj ", *(fft_buf_temp + i * 2), *(fft_buf_temp + i * 2 + 1));
    // }
    // printf("\n");

    auto cur_symbol_type = SymbolType::kUnknown;
    if (cfg->isUplink(frame_id, subframe_id))
        cur_symbol_type = SymbolType::kUL;
    else if (cfg->isPilot(frame_id, subframe_id))
        cur_symbol_type = SymbolType::kPilot;
    else if (cfg->isCalDlPilot(frame_id, subframe_id))
        cur_symbol_type = SymbolType::kCalDL;
    else if (cfg->isCalUlPilot(frame_id, subframe_id))
        cur_symbol_type = SymbolType::kCalUL;

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time1 = get_time();
    double duration1 = start_time1 - start_time;
    if (cur_symbol_type == SymbolType::kUL) {
        (*FFT_task_duration)[tid * 8][1] += duration1;
    } else if (cur_symbol_type == SymbolType::kPilot) {
        (*CSI_task_duration)[tid * 8][1] += duration1;
    }
    // else if (cur_symbol_type == CAL_DL || cur_symbol_type == CAL_UL)
    //    (*RC_task_duration)[tid * 8][1] += duration1;
#endif
    /* transfer ushort to float */
    // cvtShortToFloatSIMD(
    //     (short*)fft_buf_temp, fft_buf_ptr, cfg->OFDM_CA_NUM * 2);

    /* compute FFT */
    // DftiComputeForward(mkl_handle, fft_buffer_.FFT_inputs[0]);
    // DftiComputeForward(mkl_handle, fft_buffer_.FFT_inputs[0],
    // fft_buffer_.FFT_outputs[0]);

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time2 = get_time();
    double duration2 = start_time2 - start_time1;
    if (cur_symbol_type == SymbolType::kUL) {
        (*FFT_task_duration)[tid * 8][2] += duration2;
    } else if (cur_symbol_type == SymbolType::kPilot) {
        (*CSI_task_duration)[tid * 8][2] += duration2;
    }
    // else if (cur_symbol_type == CAL_DL || cur_symbol_type == CAL_UL)
    //    RC_task_duration[tid * 8][2] += duration2;
    double start_time_part3 = get_time();
#endif

#if DEBUG_PRINT_IN_TASK
    printf("In doFFT thread %d: frame: %zu, subframe: %zu, ant: %zu\n", tid,
        frame_id % TASK_BUFFER_FRAME_NUM, subframe_id, ant_id);
#endif
    short* fft_buf_ptr = fft_buf_temp;
    if (cur_symbol_type == SymbolType::kPilot) {
        int pilot_id = cfg->getPilotSFIndex(frame_id, subframe_id);
        int subframe_offset = (frame_id % TASK_BUFFER_FRAME_NUM)
                * cfg->pilot_symbol_num_perframe
            + pilot_id;
        // float* csi_buffer_ptr = (float*)(csi_buffer_[subframe_offset]);
        short* csi_buffer_ptr = (short*)(csi_buffer_[subframe_offset]);
        simd_store_to_buf_short(
            fft_buf_ptr, csi_buffer_ptr, ant_id, SymbolType::kPilot);
    } else if (cur_symbol_type == SymbolType::kUL) {
        int data_subframe_id = cfg->getUlSFIndex(frame_id, subframe_id);
        int data_subframe_num_perframe = cfg->ul_data_symbol_num_perframe;
        int subframe_offset
            = (frame_id % TASK_BUFFER_FRAME_NUM) * data_subframe_num_perframe
            + data_subframe_id;
        short* data_buf_ptr = (short*)&data_buffer_[subframe_offset][0];
        simd_store_to_buf_short(
            fft_buf_ptr, data_buf_ptr, ant_id, SymbolType::kUL);
    } else if (((cur_symbol_type == SymbolType::kCalDL)
                   && (ant_id == cfg->ref_ant))
        || ((cur_symbol_type == SymbolType::kCalUL)
               && (ant_id != cfg->ref_ant))) {
        int frame_offset = (frame_id % TASK_BUFFER_FRAME_NUM);
        int ant_offset = ant_id * cfg->OFDM_DATA_NUM;
        short* calib_buf_ptr = (short*)&calib_buffer_[frame_offset][ant_offset];
        simd_store_to_buf_short(
            fft_buf_ptr, calib_buf_ptr, ant_id, SymbolType::kCalUL);
    } else {
        printf("unkown or unsupported symbol type.\n");
    }

#if DEBUG_UPDATE_STATS_DETAILED
    double end_time = get_time();
    double duration3 = end_time - start_time_part3;
    if (cur_symbol_type == SymbolType::kUL) {
        (*FFT_task_duration)[tid * 8][3] += duration3;
    } else {
        (*CSI_task_duration)[tid * 8][3] += duration3;
    }
#endif

    /* After finish, reset socket buffer status */
    socket_buffer_status_[socket_thread_id][buf_offset] = 0;

#if DEBUG_UPDATE_STATS
    double duration = get_time() - start_time;
    if (cur_symbol_type == SymbolType::kUL) {
        FFT_task_count[tid * 16] = FFT_task_count[tid * 16] + 1;
        (*FFT_task_duration)[tid * 8][0] += duration;
    } else {
        CSI_task_count[tid * 16] = CSI_task_count[tid * 16] + 1;
        (*CSI_task_duration)[tid * 8][0] += duration;
    }
#endif

    /* Inform main thread */
    return Event_data(EventType::kFFT,
        fft_resp_tag_t(frame_id % TASK_BUFFER_FRAME_NUM, subframe_id)._tag);
}

void DoFFT::simd_store_to_buf(
    float* fft_buf, float*& out_buf, size_t ant_id, SymbolType symbol_type)
{
    size_t block_num = cfg->OFDM_DATA_NUM / cfg->transpose_block_size;
    size_t sc_idx = cfg->OFDM_DATA_START;

    for (size_t block_idx = 0; block_idx < block_num; block_idx++) {
        for (size_t sc_inblock_idx = 0;
             sc_inblock_idx < cfg->transpose_block_size; sc_inblock_idx += 8) {

            float* src_ptr_cur = fft_buf + sc_idx * 2;
            float* tar_ptr_cur = out_buf + sc_inblock_idx * 2;
            if (symbol_type == SymbolType::kCalUL) {
                tar_ptr_cur += block_idx * cfg->transpose_block_size * 2;
            } else {
                tar_ptr_cur += (block_idx * cfg->BS_ANT_NUM + ant_id)
                    * cfg->transpose_block_size * 2;
            }
#ifdef __AVX512F__
            /* load 256 bits = 32 bytes = 8 float values = 4 subcarriers */
            __m512 fft_result = _mm512_load_ps(src_ptr_cur);
            if (symbol_type == SymbolType::kPilot) {
                __m512 pilot_tx = _mm512_set_ps(cfg->pilots_[sc_idx + 7],
                    cfg->pilots_[sc_idx + 7], cfg->pilots_[sc_idx + 6],
                    cfg->pilots_[sc_idx + 6], cfg->pilots_[sc_idx + 5],
                    cfg->pilots_[sc_idx + 5], cfg->pilots_[sc_idx + 4],
                    cfg->pilots_[sc_idx + 4], cfg->pilots_[sc_idx + 3],
                    cfg->pilots_[sc_idx + 3], cfg->pilots_[sc_idx + 2],
                    cfg->pilots_[sc_idx + 2], cfg->pilots_[sc_idx + 1],
                    cfg->pilots_[sc_idx + 1], cfg->pilots_[sc_idx],
                    cfg->pilots_[sc_idx]);
                fft_result = _mm512_mul_ps(fft_result, pilot_tx);
            }
            _mm512_stream_ps(tar_ptr_cur, fft_result);
#else
            /* load 256 bits = 32 bytes = 8 float values = 4 subcarriers */
            __m256 fft_result = _mm256_load_ps(src_ptr_cur);
            __m256 fft_result1 = _mm256_load_ps(src_ptr_cur + 8);
            if (symbol_type == SymbolType::kPilot) {
                __m256 pilot_tx = _mm256_set_ps(cfg->pilots_[sc_idx + 3],
                    cfg->pilots_[sc_idx + 3], cfg->pilots_[sc_idx + 2],
                    cfg->pilots_[sc_idx + 2], cfg->pilots_[sc_idx + 1],
                    cfg->pilots_[sc_idx + 1], cfg->pilots_[sc_idx],
                    cfg->pilots_[sc_idx]);
                fft_result = _mm256_mul_ps(fft_result, pilot_tx);

                __m256 pilot_tx1 = _mm256_set_ps(cfg->pilots_[sc_idx + 7],
                    cfg->pilots_[sc_idx + 7], cfg->pilots_[sc_idx + 6],
                    cfg->pilots_[sc_idx + 6], cfg->pilots_[sc_idx + 5],
                    cfg->pilots_[sc_idx + 5], cfg->pilots_[sc_idx + 4],
                    cfg->pilots_[sc_idx + 4]);
                fft_result1 = _mm256_mul_ps(fft_result1, pilot_tx1);
            }
            _mm256_stream_ps(tar_ptr_cur, fft_result);
            _mm256_stream_ps(tar_ptr_cur + 8, fft_result1);
#endif
            sc_idx += 8;
        }
    }
}

void DoFFT::simd_store_to_buf_short(
    short* fft_buf, short*& out_buf, size_t ant_id, SymbolType symbol_type)
{
    size_t block_num = cfg->OFDM_DATA_NUM / cfg->transpose_block_size;
    size_t sc_idx = cfg->OFDM_DATA_START;

    for (size_t block_idx = 0; block_idx < block_num; block_idx++) {
        for (size_t sc_inblock_idx = 0;
             sc_inblock_idx < cfg->transpose_block_size; sc_inblock_idx += 16) {

            short* src_ptr_cur = fft_buf + sc_idx * 2;
            short* tar_ptr_cur = out_buf + sc_inblock_idx * 2;
            if (symbol_type == SymbolType::kCalUL) {
                tar_ptr_cur += block_idx * cfg->transpose_block_size * 2;
            } else {
                tar_ptr_cur += (block_idx * cfg->BS_ANT_NUM + ant_id)
                    * cfg->transpose_block_size * 2;
            }
#ifdef __AVX512F__
            /* load 512 bits = 64 bytes = 32 short values = 16 subcarriers */
            __m512i fft_result = _mm512_load_epi32(src_ptr_cur);
            if (symbol_type == SymbolType::kPilot) {
                __m512i pilot_tx
                    = _mm512_set_epi16(cfg->pilots_short[sc_idx + 15],
                        cfg->pilots_short[sc_idx + 15],
                        cfg->pilots_short[sc_idx + 14],
                        cfg->pilots_short[sc_idx + 14],
                        cfg->pilots_short[sc_idx + 13],
                        cfg->pilots_short[sc_idx + 13],
                        cfg->pilots_short[sc_idx + 12],
                        cfg->pilots_short[sc_idx + 12],
                        cfg->pilots_short[sc_idx + 11],
                        cfg->pilots_short[sc_idx + 11],
                        cfg->pilots_short[sc_idx + 10],
                        cfg->pilots_short[sc_idx + 10],
                        cfg->pilots_short[sc_idx + 9],
                        cfg->pilots_short[sc_idx + 9],
                        cfg->pilots_short[sc_idx + 8],
                        cfg->pilots_short[sc_idx + 8],
                        cfg->pilots_short[sc_idx + 7],
                        cfg->pilots_short[sc_idx + 7],
                        cfg->pilots_short[sc_idx + 6],
                        cfg->pilots_short[sc_idx + 6],
                        cfg->pilots_short[sc_idx + 5],
                        cfg->pilots_short[sc_idx + 5],
                        cfg->pilots_short[sc_idx + 4],
                        cfg->pilots_short[sc_idx + 4],
                        cfg->pilots_short[sc_idx + 3],
                        cfg->pilots_short[sc_idx + 3],
                        cfg->pilots_short[sc_idx + 2],
                        cfg->pilots_short[sc_idx + 2],
                        cfg->pilots_short[sc_idx + 1],
                        cfg->pilots_short[sc_idx + 1],
                        cfg->pilots_short[sc_idx], cfg->pilots_short[sc_idx]);
                fft_result = _mm512_mullo_epi16(fft_result, pilot_tx);
            }
            _mm512_stream_si512((void*)tar_ptr_cur, fft_result);
#else
            /* load 256 bits = 32 bytes = 16 short values = 8 subcarriers */
            __m256i fft_result = _mm256_load_si256((__m256i*)src_ptr_cur);
            __m256i fft_result1
                = _mm256_load_si256((__m256i*)(src_ptr_cur + 16));
            if (symbol_type == SymbolType::kPilot) {
                __m256i pilot_tx
                    = _mm256_set_epi16(cfg->pilots_short[sc_idx + 7],
                        cfg->pilots_short[sc_idx + 7],
                        cfg->pilots_short[sc_idx + 6],
                        cfg->pilots_short[sc_idx + 6],
                        cfg->pilots_short[sc_idx + 5],
                        cfg->pilots_short[sc_idx + 5],
                        cfg->pilots_short[sc_idx + 4],
                        cfg->pilots_short[sc_idx + 4],
                        cfg->pilots_short[sc_idx + 3],
                        cfg->pilots_short[sc_idx + 3],
                        cfg->pilots_short[sc_idx + 2],
                        cfg->pilots_short[sc_idx + 2],
                        cfg->pilots_short[sc_idx + 1],
                        cfg->pilots_short[sc_idx + 1],
                        cfg->pilots_short[sc_idx], cfg->pilots_short[sc_idx]);
                fft_result = _mm256_mullo_epi16(fft_result, pilot_tx);

                __m256i pilot_tx1
                    = _mm256_set_epi16(cfg->pilots_short[sc_idx + 15],
                        cfg->pilots_short[sc_idx + 15],
                        cfg->pilots_short[sc_idx + 14],
                        cfg->pilots_short[sc_idx + 14],
                        cfg->pilots_short[sc_idx + 13],
                        cfg->pilots_short[sc_idx + 13],
                        cfg->pilots_short[sc_idx + 12],
                        cfg->pilots_short[sc_idx + 12],
                        cfg->pilots_short[sc_idx + 11],
                        cfg->pilots_short[sc_idx + 11],
                        cfg->pilots_short[sc_idx + 10],
                        cfg->pilots_short[sc_idx + 10],
                        cfg->pilots_short[sc_idx + 9],
                        cfg->pilots_short[sc_idx + 9],
                        cfg->pilots_short[sc_idx + 8],
                        cfg->pilots_short[sc_idx + 8]);
                fft_result1 = _mm256_mullo_epi16(fft_result1, pilot_tx1);
            }
            _mm256_stream_si256((__m256i*)tar_ptr_cur, fft_result);
            _mm256_stream_si256((__m256i*)(tar_ptr_cur + 16), fft_result1);
#endif
            sc_idx += 16;
        }
    }
}

DoIFFT::DoIFFT(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    Consumer& in_consumer, Table<complex_float>& in_dl_ifft_buffer,
    char* in_dl_socket_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, in_consumer)
    , dl_ifft_buffer_(in_dl_ifft_buffer)
    , dl_socket_buffer_(in_dl_socket_buffer)
    , task_duration(&in_stats_manager->ifft_stats_worker.task_duration)
    , task_count(in_stats_manager->ifft_stats_worker.task_count)
{
    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->OFDM_CA_NUM);
    (void)DftiCommitDescriptor(mkl_handle);
}

DoIFFT::~DoIFFT() { DftiFreeDescriptor(&mkl_handle); }

Event_data DoIFFT::launch(int offset)
{
#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif
#if DEBUG_PRINT_IN_TASK
    int ant_id = offset % cfg->BS_ANT_NUM;
    int total_data_subframe_id = offset / cfg->BS_ANT_NUM;
    int frame_id = total_data_subframe_id / cfg->data_symbol_num_perframe;
    int current_data_subframe_id
        = total_data_subframe_id % cfg->data_symbol_num_perframe;
    printf("In doIFFT thread %d: frame: %d, subframe: %d, antenna: %d\n", tid,
        frame_id, current_data_subframe_id, ant_id);
#endif

    int dl_ifft_buffer_size = cfg->BS_ANT_NUM * cfg->data_symbol_num_perframe
        * TASK_BUFFER_FRAME_NUM;
    int buffer_subframe_offset = offset % dl_ifft_buffer_size;

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time1 = get_time();
    double duration1 = start_time1 - start_time;
    (*task_duration)[tid * 8][1] += duration1;
#endif

    float* ifft_buf_ptr = (float*)dl_ifft_buffer_[buffer_subframe_offset];
    memset(ifft_buf_ptr, 0, sizeof(float) * cfg->OFDM_DATA_START * 2);
    memset(ifft_buf_ptr + (cfg->OFDM_DATA_START + cfg->OFDM_DATA_NUM) * 2, 0,
        sizeof(float) * cfg->OFDM_DATA_START * 2);

    DftiComputeBackward(mkl_handle, ifft_buf_ptr);
    // printf("data after ifft\n");
    // for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++)
    //     printf("%.1f+%.1fj ", dl_ifft_buffer_[buffer_subframe_offset][i].re,
    //         dl_ifft_buffer_[buffer_subframe_offset][i].im);
    // printf("\n");

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time2 = get_time();
    double duration2 = start_time2 - start_time1;
    (*task_duration)[tid * 8][2] += duration2;
#endif

    int dl_socket_buffer_status_size = cfg->BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
        * cfg->data_symbol_num_perframe;
    int socket_subframe_offset = offset % dl_socket_buffer_status_size;
    int packet_length = cfg->packet_length;
    struct Packet* pkt
        = (struct Packet*)&dl_socket_buffer_[socket_subframe_offset
            * packet_length];
    short* socket_ptr = &pkt->data[2 * cfg->TX_PREFIX_LEN];

    for (size_t sc_id = 0; sc_id < cfg->OFDM_CA_NUM; sc_id += 8) {
        /* ifft scaled results by OFDM_CA_NUM */
        __m256 scale_factor = _mm256_set1_ps(32768.0 / cfg->OFDM_CA_NUM);
        __m256 ifft1 = _mm256_load_ps(ifft_buf_ptr + 2 * sc_id);
        __m256 ifft2 = _mm256_load_ps(ifft_buf_ptr + 2 * sc_id + 8);
        __m256 scaled_ifft1 = _mm256_mul_ps(ifft1, scale_factor);
        __m256 scaled_ifft2 = _mm256_mul_ps(ifft2, scale_factor);
        __m256i integer1 = _mm256_cvtps_epi32(scaled_ifft1);
        __m256i integer2 = _mm256_cvtps_epi32(scaled_ifft2);
        integer1 = _mm256_packs_epi32(integer1, integer2);
        integer1 = _mm256_permute4x64_epi64(integer1, 0xD8);
        //_mm256_stream_si256((__m256i*)&socket_ptr[2 * sc_id], integer1);
        _mm256_stream_si256(
            (__m256i*)&socket_ptr[2 * (sc_id + cfg->CP_LEN)], integer1);
        if (sc_id >= cfg->OFDM_CA_NUM - cfg->CP_LEN) // add CP
            _mm256_stream_si256((__m256i*)&socket_ptr[2
                                    * (sc_id + cfg->CP_LEN - cfg->OFDM_CA_NUM)],
                integer1);
    }

#if DEBUG_UPDATE_STATS_DETAILED
    double start_time3 = get_time();
    double duration3 = start_time3 - start_time2;
    (*task_duration)[tid * 8][3] += duration3;
#endif

    // cout << "In ifft: frame: "<< frame_id<<", subframe: "<<
    // current_data_subframe_id<<", ant: " << ant_id << ", data: "; for (int j =
    // 0; j <OFDM_CA_NUM; j++) {
    //     int socket_offset = sizeof(int) * 16 + ant_id *
    //     packetReceiver::packet_length; cout <<*((short *)(socket_ptr +
    //     socket_offset) + 2 * j)  << "+j"<<*((short *)(socket_ptr +
    //     socket_offset) + 2 * j + 1 )<<",   ";
    // }
    // cout<<"\n\n"<<endl;

#if DEBUG_UPDATE_STATS
    task_count[tid * 16] = task_count[tid * 16] + 1;
    (*task_duration)[tid * 8][0] += get_time() - start_time;
#endif

    /* Inform main thread */
    Event_data ifft_finish_event(EventType::kIFFT, offset);
    // consumer_.handle(ifft_finish_event);
    return ifft_finish_event;
}
