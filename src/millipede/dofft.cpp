/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "dofft.hpp"
#include "concurrent_queue_wrapper.hpp"

using namespace arma;

static constexpr bool kPrintFFTInput = false;

/**
 * Use SIMD to vectorize data type conversion from short to float
 * reference:
 * https://stackoverflow.com/questions/50597764/convert-signed-short-to-float-in-c-simd
 * 0x4380'8000
 */
void cvtShortToFloatSIMD(short*& in_buf, float*& out_buf, size_t length)
{
#ifdef __AVX512F__
    const __m512 magic = _mm512_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m512i magic_i = _mm512_castps_si512(magic);
    for (size_t i = 0; i < length; i += 16) {
        /* get input */
        __m256i val = _mm256_load_si256((__m256i*)(in_buf + i)); // port 2,3
        /* interleave with 0x0000 */
        __m512i val_unpacked = _mm512_cvtepu16_epi32(val); // port 5
        /* convert by xor-ing and subtracting magic value:
         * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
        __m512i val_f_int
            = _mm512_xor_si512(val_unpacked, magic_i); // port 0,1,5
        __m512 val_f = _mm512_castsi512_ps(val_f_int); // no instruction
        __m512 converted = _mm512_sub_ps(val_f, magic); // port 1,5 ?
        _mm512_store_ps(out_buf + i, converted); // port 2,3,4,7
    }
#else
    const __m256 magic = _mm256_set1_ps(float((1 << 23) + (1 << 15)) / 32768.f);
    const __m256i magic_i = _mm256_castps_si256(magic);
    for (size_t i = 0; i < length; i += 16) {
        /* get input */
        __m128i val = _mm_load_si128((__m128i*)(in_buf + i)); // port 2,3

        __m128i val1 = _mm_load_si128((__m128i*)(in_buf + i + 8));
        /* interleave with 0x0000 */
        __m256i val_unpacked = _mm256_cvtepu16_epi32(val); // port 5
        /* convert by xor-ing and subtracting magic value:
         * VPXOR avoids port5 bottlenecks on Intel CPUs before SKL */
        __m256i val_f_int
            = _mm256_xor_si256(val_unpacked, magic_i); // port 0,1,5
        __m256 val_f = _mm256_castsi256_ps(val_f_int); // no instruction
        __m256 converted = _mm256_sub_ps(val_f, magic); // port 1,5 ?
        _mm256_store_ps(out_buf + i, converted); // port 2,3,4,7

        __m256i val_unpacked1 = _mm256_cvtepu16_epi32(val1); // port 5
        __m256i val_f_int1
            = _mm256_xor_si256(val_unpacked1, magic_i); // port 0,1,5
        __m256 val_f1 = _mm256_castsi256_ps(val_f_int1); // no instruction
        __m256 converted1 = _mm256_sub_ps(val_f1, magic); // port 1,5 ?
        _mm256_store_ps(out_buf + i + 8, converted1); // port 2,3,4,7
    }
#endif
}

DoFFT::DoFFT(Config* in_config, int in_tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<char>& in_socket_buffer, Table<int>& in_socket_buffer_status,
    Table<complex_float>& in_data_buffer, Table<complex_float>& in_csi_buffer,
    Table<complex_float>& in_calib_buffer, Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , socket_buffer_(in_socket_buffer)
    , socket_buffer_status_(in_socket_buffer_status)
    , data_buffer_(in_data_buffer)
    , csi_buffer_(in_csi_buffer)
    , calib_buffer_(in_calib_buffer)
{
    duration_stat_fft
        = in_stats_manager->get_duration_stat(DoerType::kFFT, in_tid);
    duration_stat_csi
        = in_stats_manager->get_duration_stat(DoerType::kCSI, in_tid);
    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->OFDM_CA_NUM);
    // auto mkl_status = DftiSetValue(mkl_handle, DFTI_PLACEMENT,
    // DFTI_NOT_INPLACE);
    (void)DftiCommitDescriptor(mkl_handle);

    int FFT_buffer_block_num = 1;
    fft_buffer_.FFT_inputs.calloc(FFT_buffer_block_num, cfg->OFDM_CA_NUM, 64);
    fft_buffer_.FFT_outputs.calloc(FFT_buffer_block_num, cfg->OFDM_CA_NUM, 64);
}

DoFFT::~DoFFT()
{
    DftiFreeDescriptor(&mkl_handle);
    fft_buffer_.FFT_inputs.free();
    fft_buffer_.FFT_outputs.free();
}

Event_data DoFFT::launch(size_t tag)
{
    DurationStat dummy_duration_stat; // For calibration symbols for now
    size_t start_tsc = worker_rdtsc();

    size_t socket_thread_id = fft_req_tag_t(tag).tid;
    size_t buf_offset = fft_req_tag_t(tag).offset;

    char* cur_buffer_ptr
        = socket_buffer_[socket_thread_id] + buf_offset * cfg->packet_length;
    auto* pkt = (Packet*)cur_buffer_ptr;
    size_t frame_id = pkt->frame_id;
    size_t symbol_id = pkt->symbol_id;
    size_t ant_id = pkt->ant_id;

    short* cur_buffer_ptr_ushort = &pkt->data[2 * cfg->OFDM_PREFIX_LEN];
    float* fft_buf_ptr = (float*)(fft_buffer_.FFT_inputs[0]);

    /* transfer ushort to float */
    cvtShortToFloatSIMD(
        cur_buffer_ptr_ushort, fft_buf_ptr, cfg->OFDM_CA_NUM * 2);

    if (kDebugPrintInTask) {
        printf("In doFFT thread %d: frame: %zu, symbol: %zu, ant: %zu\n", tid,
            frame_id, symbol_id, ant_id);
        if (kPrintFFTInput) {
            printf("FFT input\n");
            for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++) {
                printf(
                    "%.4f+%.4fi ", fft_buf_ptr[i * 2], fft_buf_ptr[i * 2 + 1]);
            }
            printf("\n");
        }
    }

    DurationStat* duration_stat = nullptr;
    auto cur_symbol_type = SymbolType::kUnknown;
    if (cfg->isUplink(frame_id, symbol_id)) {
        cur_symbol_type = SymbolType::kUL;
        duration_stat = duration_stat_fft;
    } else if (cfg->isPilot(frame_id, symbol_id)) {
        cur_symbol_type = SymbolType::kPilot;
        duration_stat = duration_stat_csi;
    } else if (cfg->isCalDlPilot(frame_id, symbol_id)) {
        cur_symbol_type = SymbolType::kCalDL;
        duration_stat = &dummy_duration_stat;
    } else if (cfg->isCalUlPilot(frame_id, symbol_id)) {
        cur_symbol_type = SymbolType::kCalUL;
        duration_stat = &dummy_duration_stat;
    }

    size_t start_tsc1 = worker_rdtsc();
    duration_stat->task_duration[1] += start_tsc1 - start_tsc;

    DftiComputeForward(mkl_handle, fft_buffer_.FFT_inputs[0]); // Compute FFT

    size_t start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

    size_t start_tsc_part3 = worker_rdtsc();
    if (cur_symbol_type == SymbolType::kPilot) {
        int pilot_id = cfg->getPilotSFIndex(frame_id, symbol_id);
        size_t symbol_offset = ((frame_id % TASK_BUFFER_FRAME_NUM)
                                   * cfg->pilot_symbol_num_perframe)
            + pilot_id;
        float* csi_buffer_ptr = (float*)(csi_buffer_[symbol_offset]);
        simd_store_to_buf(
            fft_buf_ptr, csi_buffer_ptr, ant_id, SymbolType::kPilot);
    } else if (cur_symbol_type == SymbolType::kUL) {
        int data_symbol_id = cfg->getUlSFIndex(frame_id, symbol_id);
        int data_symbol_num_perframe = cfg->ul_data_symbol_num_perframe;
        size_t symbol_offset
            = ((frame_id % TASK_BUFFER_FRAME_NUM) * data_symbol_num_perframe)
            + data_symbol_id;
        float* data_buf_ptr = (float*)&data_buffer_[symbol_offset][0];
        simd_store_to_buf(fft_buf_ptr, data_buf_ptr, ant_id, SymbolType::kUL);
    } else if (((cur_symbol_type == SymbolType::kCalDL)
                   && (ant_id == cfg->ref_ant))
        || ((cur_symbol_type == SymbolType::kCalUL)
               && (ant_id != cfg->ref_ant))) {
        size_t frame_offset = frame_id % TASK_BUFFER_FRAME_NUM;
        int ant_offset = ant_id * cfg->OFDM_DATA_NUM;
        float* calib_buf_ptr = (float*)&calib_buffer_[frame_offset][ant_offset];
        simd_store_to_buf(
            fft_buf_ptr, calib_buf_ptr, ant_id, SymbolType::kCalUL);
    } else {
        rt_assert(false, "Unknown or unsupported symbol type");
    }

    duration_stat->task_duration[3] += worker_rdtsc() - start_tsc_part3;

    /* After finish, reset socket buffer status */
    socket_buffer_status_[socket_thread_id][buf_offset] = 0;

    duration_stat->task_count++;
    duration_stat->task_duration[0] += worker_rdtsc() - start_tsc;
    return Event_data(EventType::kFFT,
        gen_tag_t::frm_sym(pkt->frame_id, pkt->symbol_id)._tag);
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

DoIFFT::DoIFFT(Config* in_config, int in_tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<complex_float>& in_dl_ifft_buffer, char* in_dl_socket_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , dl_ifft_buffer_(in_dl_ifft_buffer)
    , dl_socket_buffer_(in_dl_socket_buffer)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kIFFT, in_tid);
    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, cfg->OFDM_CA_NUM);
    (void)DftiCommitDescriptor(mkl_handle);
}

DoIFFT::~DoIFFT() { DftiFreeDescriptor(&mkl_handle); }

Event_data DoIFFT::launch(size_t tag)
{
    size_t start_tsc = worker_rdtsc();
    size_t ant_id = gen_tag_t(tag).ant_id;
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t data_symbol_idx = gen_tag_t(tag).symbol_id;

    if (kDebugPrintInTask) {
        printf("In doIFFT thread %d: frame: %zu, symbol: %zu, antenna: %zu\n",
            tid, frame_id, data_symbol_idx, ant_id);
    }

    size_t offset = (cfg->get_total_data_symbol_idx(frame_id, data_symbol_idx)
                        * cfg->BS_ANT_NUM)
        + ant_id;

    size_t num_dl_ifft_buffers = cfg->BS_ANT_NUM * cfg->data_symbol_num_perframe
        * TASK_BUFFER_FRAME_NUM;

    size_t start_tsc1 = worker_rdtsc();
    duration_stat->task_duration[1] += start_tsc1 - start_tsc;

    float* ifft_buf_ptr = (float*)dl_ifft_buffer_[offset % num_dl_ifft_buffers];
    memset(ifft_buf_ptr, 0, sizeof(float) * cfg->OFDM_DATA_START * 2);
    memset(ifft_buf_ptr + (cfg->OFDM_DATA_START + cfg->OFDM_DATA_NUM) * 2, 0,
        sizeof(float) * cfg->OFDM_DATA_START * 2);

    DftiComputeBackward(mkl_handle, ifft_buf_ptr);
    // printf("data after ifft\n");
    // for (size_t i = 0; i < cfg->OFDM_CA_NUM; i++)
    //     printf("%.1f+%.1fj ", dl_ifft_buffer_[buffer_symbol_offset][i].re,
    //         dl_ifft_buffer_[buffer_symbol_offset][i].im);
    // printf("\n");

    cx_fmat mat_data((cx_float*)ifft_buf_ptr, 1, cfg->OFDM_CA_NUM, false);
    float post_scale = abs(mat_data).max();
    mat_data /= post_scale;

    size_t start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

    int dl_socket_buffer_status_size = cfg->BS_ANT_NUM * SOCKET_BUFFER_FRAME_NUM
        * cfg->data_symbol_num_perframe;
    int socket_symbol_offset = offset % dl_socket_buffer_status_size;
    int packet_length = cfg->packet_length;
    struct Packet* pkt = (struct Packet*)&dl_socket_buffer_[socket_symbol_offset
        * packet_length];
    short* socket_ptr = &pkt->data[2 * cfg->TX_PREFIX_LEN];

    for (size_t sc_id = 0; sc_id < cfg->OFDM_CA_NUM; sc_id += 8) {
        /* ifft scaled results by OFDM_CA_NUM */
        __m256 scale_factor = _mm256_set1_ps(32768.0);
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

    duration_stat->task_duration[2] += worker_rdtsc() - start_tsc2;

    // cout << "In ifft: frame: "<< frame_id<<", symbol: "<<
    // current_data_symbol_id<<", ant: " << ant_id << ", data: "; for (int j =
    // 0; j <OFDM_CA_NUM; j++) {
    //     int socket_offset = sizeof(int) * 16 + ant_id *
    //     packetReceiver::packet_length; cout <<*((short *)(socket_ptr +
    //     socket_offset) + 2 * j)  << "+j"<<*((short *)(socket_ptr +
    //     socket_offset) + 2 * j + 1 )<<",   ";
    // }
    // cout<<"\n\n"<<endl;

    duration_stat->task_count++;
    duration_stat->task_duration[0] += worker_rdtsc() - start_tsc;
    return Event_data(EventType::kIFFT, tag);
}
