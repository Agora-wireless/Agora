#include "bigstation.hpp"
#include "Symbols.hpp"
#include "profiler.h"
#include "diagnosis.hpp"
#include <rte_ethdev.h>
using namespace std;

BigStation::BigStation(Config* config)
    : freq_ghz_(measure_rdtsc_freq())
    , config_(config)
    , bigstation_state_(config)
{
    std::string directory = TOSTRING(PROJECT_DIRECTORY);
    printf("BigStation: project directory [%s], RDTSC frequency = %.2f GHz\n",
        directory.c_str(), freq_ghz_);

    pin_to_core_with_offset(
        ThreadType::kMaster, config->core_offset, 0, false /* quiet */);

    initializeBigStationBuffers();

    bigstation_tx_rx_.reset(new BigStationTXRX(config, config->core_offset, time_iq_buffer_, freq_iq_buffer_to_send_,
        freq_iq_buffer_, post_zf_buffer_to_send_, post_zf_buffer_, post_demul_buffer_to_send_, post_demul_buffer_, 
        post_decode_buffer_, &bigstation_state_));

    base_worker_core_offset_ = config_->core_offset + kNumMasterThread + 
        config_->rx_thread_num + config_->tx_thread_num;
    
    do_fft_threads_.resize(config_->num_fft_workers[config_->bs_server_addr_idx]);
    for (size_t i = 0; i < do_fft_threads_.size(); ++i) {
        do_fft_threads_[i] = std::thread(&BigStation::fftWorker, this, i + config_->fft_thread_offset);
    }
    do_zf_threads_.resize(config_->num_zf_workers[config_->bs_server_addr_idx]);
    for (size_t i = 0; i < do_zf_threads_.size(); ++i) {
        do_zf_threads_[i] = std::thread(&BigStation::zfWorker, this, i + config_->zf_thread_offset);
    }
    do_demul_threads_.resize(config_->num_demul_workers[config_->bs_server_addr_idx]);
    for (size_t i = 0; i < do_demul_threads_.size(); ++i) {
        do_demul_threads_[i] = std::thread(&BigStation::demulWorker, this, i + config_->demul_thread_offset);
    }
    do_decode_threads_.resize(config_->num_decode_workers[config_->bs_server_addr_idx]);
    for (size_t i = 0; i < do_decode_threads_.size(); ++i) {
        do_decode_threads_[i] = std::thread(&BigStation::decodeWorker, this, i + config_->decode_thread_offset);
    }

    printf("Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
           "cores %zu--%zu\n",
        config->core_offset, config->core_offset + kNumMasterThread,
        config->core_offset + kNumMasterThread + config->rx_thread_num + config->tx_thread_num - 1,
        base_worker_core_offset_,
        base_worker_core_offset_ + do_fft_threads_.size() + do_zf_threads_.size() + 
        do_demul_threads_.size() + do_decode_threads_.size() - 1);
}

BigStation::~BigStation()
{
    freeBigStationBuffers();

    for (auto& t : do_fft_threads_) {
        t.join();
    }
    for (auto& t : do_zf_threads_) {
        t.join();
    }
    for (auto& t : do_demul_threads_) {
        t.join();
    }
    for (auto& t : do_decode_threads_) {
        t.join();
    }
}

void BigStation::Stop()
{
    static const size_t kSleepBeforeTxRx = 1000;
    std::cout << "BigStation: stopping threads" << std::endl;
    config_->running = false;
    usleep(kSleepBeforeTxRx);
    bigstation_tx_rx_.reset();
}

void BigStation::Start()
{
    auto& cfg = config_;

    rte_eth_stats start_stats;
    rte_eth_stats_get(0, &start_stats);

    // Start packet I/O
    if (!bigstation_tx_rx_->StartTXRX()) {
        Stop();
        return;
    }

    while (cfg->running && !SignalHandler::gotExitSignal()) {
        if (bigstation_state_.cur_frame_ == cfg->frames_to_test) {
            cfg->running = false;
            goto finish;
        }
        sleep(1);
    }
    cfg->running = false;
    goto finish;
    return;

finish:

    // if (cfg->error) {
    //     Diagnosis(cfg, &shared_state_, bottleneck_subcarrier_,
    //         bottleneck_decode_, bottleneck_encode_);
    // } else {
    //     printf("Agora: No error detected\n");
    // }

    rte_eth_stats end_stats;
    rte_eth_stats_get(0, &end_stats);

    printf("BigStation: Input traffic rate is %.2lfGbps, output traffic rate is %.2lfGbps\n", (double)(end_stats.ibytes - start_stats.ibytes) * 8 / (cfg->frames_to_test * 0.001) / 1000000000.0,
        (double)(end_stats.obytes - start_stats.obytes) * 8 / (cfg->frames_to_test * 0.001) / 1000000000.0);

    Stop();
}

void BigStation::fftWorker(int tid)
{
    size_t ant_start = tid * config_->BS_ANT_NUM / config_->total_fft_workers;
    size_t ant_end = (tid + 1) * config_->BS_ANT_NUM / config_->total_fft_workers;

    size_t cur_frame = 0;
    size_t cur_symbol = 0;
    size_t cur_ant = ant_start;

    DoFFT *do_fft = new DoFFT(config_, tid, freq_ghz_, Range(ant_start, ant_end), time_iq_buffer_,
        freq_iq_buffer_to_send_, nullptr);

    while (config_->running && !SignalHandler::gotExitSignal()) {
        if (bigstation_state_.received_all_time_iq_pkts(cur_frame, cur_symbol)) {
            do_fft->Launch(cur_frame, cur_symbol, cur_ant);

            bigstation_state_.prepare_freq_iq_pkt(cur_frame, cur_symbol, cur_ant);
            cur_ant++;
            if (cur_ant == ant_end) {
                cur_ant = ant_start;
                cur_symbol++;
                if (cur_symbol == config_->symbol_num_perframe) {
                    cur_symbol = 0;
                    cur_frame++;
                    if (cur_frame == config_->frames_to_test) {
                        break;
                    }
                }
            }
        }
    }

    delete do_fft;
}

void BigStation::runCsi(size_t frame_id, size_t base_sc_id, size_t sc_block_size,
    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffer)
{
    const size_t frame_slot = frame_id % kFrameWnd;

    complex_float converted_sc[kSCsPerCacheline];

    size_t sc_start = base_sc_id;
    size_t sc_end = sc_start + sc_block_size;

    for (size_t i = 0; i < config_->pilot_symbol_num_perframe; i++) {
        for (size_t j = 0; j < config_->BS_ANT_NUM; j++) {
            auto* pkt = reinterpret_cast<Packet*>(freq_iq_buffer_[j]
                + (frame_slot * config_->symbol_num_perframe
                        * config_->packet_length)
                + i * config_->packet_length);

            // Subcarrier ranges should be aligned with kTransposeBlockSize
            // for (size_t block_idx = sc_range_.start / kTransposeBlockSize;
            //         block_idx < sc_range_.end / kTransposeBlockSize;
            //         block_idx++) {
            for (size_t block_idx = sc_start / kTransposeBlockSize;
                    block_idx < sc_end / kTransposeBlockSize;
                    block_idx++) {

                const size_t block_base_offset
                    = block_idx * (kTransposeBlockSize * config_->BS_ANT_NUM);

                for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
                        sc_j += kSCsPerCacheline) {
                    const size_t sc_idx
                        = (block_idx * kTransposeBlockSize) + sc_j;

                    simd_convert_float16_to_float32(
                        reinterpret_cast<float*>(converted_sc),
                        reinterpret_cast<float*>(pkt->data_
                            + (config_->OFDM_DATA_START + sc_idx) * 2),
                        kSCsPerCacheline * 2);

                    const complex_float* src = converted_sc;
                    complex_float* dst = csi_buffer[frame_slot][i]
                        + block_base_offset + (j * kTransposeBlockSize)
                        + sc_j;

                    // With either of AVX-512 or AVX2, load one cacheline =
                    // 16 float values = 8 subcarriers = kSCsPerCacheline
                    // TODO: AVX512 complex multiply support below
                    // size_t pilots_sgn_offset = cfg_->bs_server_addr_idx
                    //     * cfg_->get_num_sc_per_server();
                    size_t pilots_sgn_offset = 0;

                    __m256 fft_result0 = _mm256_load_ps(
                        reinterpret_cast<const float*>(src));
                    __m256 fft_result1 = _mm256_load_ps(
                        reinterpret_cast<const float*>(src + 4));
                    __m256 pilot_tx0 = _mm256_set_ps(
                        config_->pilots_sgn_[sc_idx + 3 + pilots_sgn_offset].im,
                        config_->pilots_sgn_[sc_idx + 3 + pilots_sgn_offset].re,
                        config_->pilots_sgn_[sc_idx + 2 + pilots_sgn_offset].im,
                        config_->pilots_sgn_[sc_idx + 2 + pilots_sgn_offset].re,
                        config_->pilots_sgn_[sc_idx + 1 + pilots_sgn_offset].im,
                        config_->pilots_sgn_[sc_idx + 1 + pilots_sgn_offset].re,
                        config_->pilots_sgn_[sc_idx + pilots_sgn_offset].im,
                        config_->pilots_sgn_[sc_idx + pilots_sgn_offset].re);
                    fft_result0 = CommsLib::__m256_complex_cf32_mult(
                        fft_result0, pilot_tx0, true);

                    __m256 pilot_tx1 = _mm256_set_ps(
                        config_->pilots_sgn_[sc_idx + 7 + pilots_sgn_offset].im,
                        config_->pilots_sgn_[sc_idx + 7 + pilots_sgn_offset].re,
                        config_->pilots_sgn_[sc_idx + 6 + pilots_sgn_offset].im,
                        config_->pilots_sgn_[sc_idx + 6 + pilots_sgn_offset].re,
                        config_->pilots_sgn_[sc_idx + 5 + pilots_sgn_offset].im,
                        config_->pilots_sgn_[sc_idx + 5 + pilots_sgn_offset].re,
                        config_->pilots_sgn_[sc_idx + 4 + pilots_sgn_offset].im,
                        config_->pilots_sgn_[sc_idx + 4 + pilots_sgn_offset]
                            .re);
                    fft_result1 = CommsLib::__m256_complex_cf32_mult(
                        fft_result1, pilot_tx1, true);
                    _mm256_stream_ps(
                        reinterpret_cast<float*>(dst), fft_result0);
                    _mm256_stream_ps(
                        reinterpret_cast<float*>(dst + 4), fft_result1);
                }
            }
        }
    }
}

void BigStation::zfWorker(int tid)
{
    size_t sc_start = tid * config_->OFDM_DATA_NUM / config_->total_zf_workers;
    size_t sc_end = (tid + 1) * config_->OFDM_DATA_NUM / config_->total_zf_workers;

    size_t cur_zf_frame = 0;

    PtrGrid<kFrameWnd, kMaxUEs, complex_float> csi_buffer;
    csi_buffer.alloc(kFrameWnd, config_->UE_NUM, config_->BS_ANT_NUM * config_->OFDM_DATA_NUM);
    Table<complex_float> calib_buffer;
    std::vector<std::vector<ControlInfo> > dummy_table;
    std::vector<size_t> dummy_list;

    DyZF *do_zf = new DyZF(config_, tid, freq_ghz_, csi_buffer, calib_buffer, post_zf_buffer_,
        post_zf_buffer_, dummy_table, dummy_list);

    while (config_->running && !SignalHandler::gotExitSignal()) {
        if (bigstation_state_.received_all_pilot_pkts(cur_zf_frame)) {
            size_t sc_offset = simple_hash(cur_zf_frame) % config_->UE_NUM;
            int first_possible_sc = sc_start - sc_start % config_->UE_NUM + sc_offset;
            int last_possible_sc = sc_end - sc_end % config_->UE_NUM + sc_offset;
            if (first_possible_sc < (int)sc_start) {
                first_possible_sc += config_->UE_NUM;
            }
            if (last_possible_sc >= (int)sc_end) {
                last_possible_sc -= config_->UE_NUM;
            }
            if (last_possible_sc >= first_possible_sc) {
                for (size_t sc_id = first_possible_sc; sc_id <= last_possible_sc; sc_id += config_->UE_NUM) {
                    runCsi(cur_zf_frame, sc_id - sc_id % config_->UE_NUM, config_->UE_NUM, csi_buffer);
                    do_zf->ZFFreqOrthogonalStatic(gen_tag_t::frm_sym_sc(cur_zf_frame, 0, sc_id - (sc_id % config_->UE_NUM))._tag);
                }
            }
            if (!bigstation_state_.prepare_zf_pkt(cur_zf_frame)) {
                config_->error = true;
                config_->running = false;
            }
            cur_zf_frame++;
        }
    }

    delete do_zf;
}

void BigStation::demulWorker(int tid)
{
    size_t sc_start = tid * config_->OFDM_DATA_NUM / config_->total_demul_workers;
    size_t sc_end = (tid + 1) * config_->OFDM_DATA_NUM / config_->total_demul_workers;

    size_t cur_demul_frame = 0;
    size_t cur_demul_symbol_ul = 0;
    size_t cur_demul_sc = sc_start;

    const size_t task_buffer_symbol_num_ul
        = config_->ul_data_symbol_num_perframe * kFrameWnd;

    Table<complex_float> equal_buffer;
    equal_buffer.malloc(
        task_buffer_symbol_num_ul, config_->OFDM_DATA_NUM * config_->UE_NUM, 64);
    std::vector<std::vector<ControlInfo> > dummy_table;
    std::vector<size_t> dummy_list;

    DyDemul *do_demul = new DyDemul(config_, tid, freq_ghz_, freq_iq_buffer_, post_zf_buffer_, 
        equal_buffer, post_demul_buffer_to_send_, dummy_table, dummy_list);

    while (config_->running && !SignalHandler::gotExitSignal()) {
        if (bigstation_state_.received_all_zf_pkts(cur_demul_frame) && 
            bigstation_state_.received_all_ul_data_pkts(cur_demul_frame, cur_demul_symbol_ul)) {
            do_demul->LaunchStatic(cur_demul_frame, cur_demul_symbol_ul, sc_start, sc_end - sc_start);
            if (!bigstation_state_.prepare_demod_pkt(cur_demul_frame, cur_demul_symbol_ul)) {
                config_->error = true;
                config_->running = false;
            }
            cur_demul_symbol_ul++;
            if (cur_demul_symbol_ul >= config_->ul_data_symbol_num_perframe) {
                cur_demul_symbol_ul = 0;
                cur_demul_frame++;
            }
        }
    }

    delete do_demul;
}

void BigStation::decodeWorker(int tid)
{
    size_t tid_offset = tid - config_->decode_thread_offset;
    size_t cur_decode_frame = 0;
    size_t cur_decode_idx = tid_offset;

    std::vector<std::vector<ControlInfo> > dummy_table;
    std::vector<size_t> dummy_list;

    BottleneckDecode bottleneck_decode;

    DyDecode *do_decode = new DyDecode(config_, tid, freq_ghz_, post_demul_buffer_, post_decode_buffer_,
        dummy_table, dummy_list, nullptr, bottleneck_decode);

    while (config_->running && !SignalHandler::gotExitSignal()) {
        size_t cur_symbol_ul = cur_decode_idx / config_->get_num_ues_to_process();
        size_t cur_ue = cur_decode_idx % config_->get_num_ues_to_process();
        if (bigstation_state_.received_all_demod_pkts(cur_decode_frame, cur_symbol_ul)) {
            do_decode->LaunchStatic(cur_decode_frame, cur_symbol_ul, cur_ue);
            cur_decode_idx += config_->num_demul_workers[config_->bs_server_addr_idx];
            if (cur_decode_idx >= config_->get_num_ues_to_process() * config_->ul_data_symbol_num_perframe) {
                cur_decode_idx = 0;
                if (!bigstation_state_.decode_done(cur_decode_frame)) {
                    config_->error = true;
                    config_->running = false;
                }
                cur_decode_frame++;
            }
        }
    }

    delete do_decode;
}

void BigStation::initializeBigStationBuffers()
{
    auto& cfg = config_;

    size_t packet_buffer_size = cfg->packet_length * kFrameWnd * cfg->symbol_num_perframe;
    const size_t task_buffer_symbol_num_ul
        = cfg->ul_data_symbol_num_perframe * kFrameWnd;

    // time_iq_buffer_.alloc(kFrameWnd, cfg->symbol_num_perframe, cfg->BS_ANT_NUM, cfg->OFDM_CA_NUM * sizeof(short) * 2);
    time_iq_buffer_.malloc(cfg->BS_ANT_NUM, packet_buffer_size, 64);
    // pilot_buffer_.alloc(kFrameWnd, cfg->BS_ANT_NUM, cfg->OFDM_DATA_NUM * sizeof(short) * 2);
    // freq_iq_buffer_to_send_.alloc(kFrameWnd, cfg->symbol_num_perframe, cfg->BS_ANT_NUM, cfg->OFDM_DATA_NUM * sizeof(short) * 2);
    freq_iq_buffer_to_send_.malloc(cfg->BS_ANT_NUM, packet_buffer_size, 64);
    // data_buffer_.alloc(kFrameWnd, cfg->symbol_num_perframe, cfg->BS_ANT_NUM, cfg->OFDM_DATA_NUM * sizeof(short) * 2);
    freq_iq_buffer_.malloc(cfg->BS_ANT_NUM, packet_buffer_size, 64);
    post_zf_buffer_to_send_.alloc(kFrameWnd, cfg->OFDM_DATA_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM * sizeof(complex_float)); // TODO: this could make packet size over limit
    post_zf_buffer_.alloc(kFrameWnd, cfg->OFDM_DATA_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM);
    post_demul_buffer_to_send_.alloc(kFrameWnd, cfg->ul_data_symbol_num_perframe, cfg->UE_NUM, cfg->OFDM_DATA_NUM * kMaxModType);
    // post_demul_buffer_.alloc(kFrameWnd, cfg->ul_data_symbol_num_perframe, cfg->UE_NUM, cfg->OFDM_DATA_NUM * kMaxModType);
    post_demul_buffer_.malloc(task_buffer_symbol_num_ul, kMaxModType * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    post_decode_buffer_.alloc(kFrameWnd, cfg->ul_data_symbol_num_perframe, cfg->UE_NUM, cfg->OFDM_DATA_NUM * kMaxModType);
}

void BigStation::freeBigStationBuffers()
{
    time_iq_buffer_.free();
    freq_iq_buffer_to_send_.free();
    freq_iq_buffer_.free();
    // post_zf_buffer_to_send_.free();
    // post_zf_buffer_.free();
    // post_demul_buffer_to_send_.free();
    // post_demul_buffer_.free();
    // post_decode_buffer_.free();
}