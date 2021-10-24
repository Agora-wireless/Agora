#include "agora.hpp"
#include "Symbols.hpp"
#include <rte_ethdev.h>
using namespace std;

Agora::Agora(Config* cfg)
    : freq_ghz_(measure_rdtsc_freq())
    , shared_state_(cfg)
{
    std::string directory = TOSTRING(PROJECT_DIRECTORY);
    printf("Agora: project directory [%s], RDTSC frequency = %.2f GHz\n",
        directory.c_str(), freq_ghz_);

    config_ = cfg;

    pin_to_core_with_offset(
        ThreadType::kMaster, cfg->core_offset, 0, false /* quiet */);
    initializeUplinkBuffers();

    if (config_->dl_data_symbol_num_perframe > 0) {
        printf("Agora: Initializing downlink buffers\n");
        initializeDownlinkBuffers();
    }

    initControlInfo();

    /* Initialize TXRX threads */
    packet_tx_rx_.reset(new PacketTXRX(cfg, cfg->core_offset + kNumMasterThread,
        time_domain_iq_buffer_, freq_domain_iq_buffer_to_send_,
        freq_domain_iq_buffer_, demod_buffer_to_send_, demod_buffer_to_decode_, 
        &shared_state_));
    
    base_worker_core_offset_ = config_->core_offset + kNumMasterThread + 
        config_->rx_thread_num + kNumDemodTxThread;
    // TODO: Add other possible threads

    if (cfg->use_time_domain_iq) {
        do_fft_threads_.resize(cfg->fft_thread_num);
        for (size_t i = 0; i < do_fft_threads_.size(); i ++) {
            do_fft_threads_[i]
                = std::thread(&Agora::fftWorker, this, i);
        }
    }

    /* Create worker threads */
    do_subcarrier_threads_.resize(
        (cfg->get_num_sc_to_process() + cfg->subcarrier_block_size - 1) / cfg->subcarrier_block_size);
    for (size_t i = 0; i < do_subcarrier_threads_.size(); i++) {
        do_subcarrier_threads_[i]
            = std::thread(&Agora::subcarrierWorker, this, i);
    }

    do_decode_threads_.resize(cfg->decode_thread_num);
    for (size_t i = 0; i < do_decode_threads_.size(); i++) {
        do_decode_threads_[i]
            = std::thread(&Agora::decodeWorker, this, i);
    }

    printf("Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
           "cores %zu--%zu\n",
        cfg->core_offset, cfg->core_offset + kNumMasterThread,
        cfg->core_offset + kNumMasterThread + cfg->rx_thread_num + kNumDemodTxThread - 1,
        base_worker_core_offset_,
        base_worker_core_offset_ + do_subcarrier_threads_.size() + do_decode_threads_.size() + 
        do_encode_threads_.size());
}

Agora::~Agora()
{
    freeUplinkBuffers();
    /* Downlink */
    if (config_->dl_data_symbol_num_perframe > 0)
        freeDownlinkBuffers();

    for (auto& t : do_subcarrier_threads_)
        t.join();
    if (config_->downlink_mode) {
        for (auto& t : do_encode_threads_)
            t.join();
    } else {
        for (auto& t : do_decode_threads_)
            t.join();
    }
    if (config_->use_time_domain_iq) {
        for (auto& t : do_fft_threads_)
            t.join();
    }
}

void Agora::Stop()
{
    static const size_t kSleepBeforeTxRx = 1000;
    std::cout << "Agora: stopping threads" << std::endl;
    config_->running = false;
    usleep(kSleepBeforeTxRx);
    packet_tx_rx_.reset();
}

void Agora::Start()
{
    auto& cfg = config_;

    rte_eth_stats start_stats;
    rte_eth_stats_get(0, &start_stats);

    // Start packet I/O
    if (!packet_tx_rx_->StartTXRX()) {
        Stop();
        return;
    }

    while (cfg->running && !SignalHandler::gotExitSignal()) {
        if (shared_state_.cur_frame_ == cfg->frames_to_test) {
            cfg->running = false;
            goto finish;
        }
        sleep(1);
    }
    cfg->running = false;
    goto finish;
    return;

finish:

    printf("Agora: printing stats and saving to file\n");
    if (flags_.enable_save_decode_data_to_file_) {
        // TODO: fix it
        saveDecodeDataToFile(0);
    }
    if (flags_.enable_save_tx_data_to_file_) {
        // TODO: fix it
        saveTxDataToFile(0);
    }

    rte_eth_stats end_stats;
    rte_eth_stats_get(0, &end_stats);

    printf("Agora: Input traffic rate is %.2lfGbps, output traffic rate is %.2lfGbps\n", (double)(end_stats.ibytes - start_stats.ibytes) * 8 / (cfg->frames_to_test * 0.001) / 1000000000.0,
        (double)(end_stats.obytes - start_stats.obytes) * 8 / (cfg->frames_to_test * 0.001) / 1000000000.0);

    // Printing latency stats
    saveLatencyDataToFile();

    Stop();
}

void* Agora::fftWorker(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerFFT, base_worker_core_offset_, tid);

    size_t ant_block = config_->get_num_ant_to_process() / config_->fft_thread_num;
    size_t ant_off = config_->get_num_ant_to_process() % config_->fft_thread_num;

    Range ant_range((size_t)tid < ant_off ? tid * (ant_block + 1) : tid * (ant_block + 1) - (tid - ant_off),
        (size_t)tid < ant_off ? (tid + 1) * (ant_block + 1) : (tid + 1) * (ant_block + 1) - (tid + 1 - ant_off));

    auto computeFFT = new DoFFT(config_, tid, freq_ghz_, ant_range, time_domain_iq_buffer_,
        freq_domain_iq_buffer_to_send_, &shared_state_);

    computeFFT->StartWork();
    delete computeFFT;

    return nullptr;
}

void* Agora::subcarrierWorker(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerSubcarrier, base_worker_core_offset_, tid);

    Range sc_range(tid * config_->subcarrier_block_size + config_->subcarrier_start,
        min((tid + 1) * config_->subcarrier_block_size + config_->subcarrier_start,
        config_->subcarrier_end));

    auto computeSubcarrier = new DySubcarrier(config_, tid, freq_ghz_,
        sc_range,
        freq_domain_iq_buffer_, csi_buffer_, calib_buffer_,
        dl_encoded_buffer_to_precode_, demod_buffer_to_send_, dl_ifft_buffer_,
        equal_buffer_, ul_zf_matrices_, dl_zf_matrices_,
        control_info_table_, control_idx_list_,
        &shared_state_);

    computeSubcarrier->StartWork();
    delete computeSubcarrier;

    return nullptr;
}

void* Agora::decodeWorker(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerDecode, base_worker_core_offset_,
        tid + do_subcarrier_threads_.size());

    auto computeDecoding = new DyDecode(config_, tid, freq_ghz_,
        demod_buffer_to_decode_,
        decoded_buffer_, control_info_table_, control_idx_list_, 
        &shared_state_);

    computeDecoding->StartWork();
    delete computeDecoding;
    
    return nullptr;
}

void Agora::initializeUplinkBuffers()
{
    auto& cfg = config_;
    const size_t task_buffer_symbol_num_ul
        = cfg->ul_data_symbol_num_perframe * kFrameWnd;

    size_t packet_buffer_size_ = cfg->packet_length * kFrameWnd * cfg->symbol_num_perframe;

    if (cfg->use_time_domain_iq) {
        time_domain_iq_buffer_.malloc(cfg->BS_ANT_NUM, packet_buffer_size_, 64);
        freq_domain_iq_buffer_to_send_.malloc(cfg->BS_ANT_NUM, packet_buffer_size_, 64);
    }

    freq_domain_iq_buffer_.malloc(cfg->BS_ANT_NUM,
        packet_buffer_size_, 64);

    csi_buffer_.alloc(kFrameWnd, cfg->UE_NUM, cfg->BS_ANT_NUM * cfg->OFDM_DATA_NUM);
    ul_zf_matrices_.alloc(kFrameWnd, cfg->OFDM_DATA_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM);
    dl_zf_matrices_.alloc(kFrameWnd, cfg->OFDM_DATA_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM);

    demod_buffer_to_send_.alloc(kFrameWnd, cfg->symbol_num_perframe, cfg->UE_NUM, kMaxModType * cfg->OFDM_DATA_NUM);
    decoded_buffer_.alloc(kFrameWnd, cfg->symbol_num_perframe, cfg->UE_NUM, cfg->LDPC_config.nblocksInSymbol * roundup<64>(cfg->num_bytes_per_cb));

    equal_buffer_.malloc(
        task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    demod_buffer_to_decode_.malloc(
        task_buffer_symbol_num_ul, kMaxModType * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
}

void Agora::initializeDownlinkBuffers()
{
    auto& cfg = config_;
    const size_t task_buffer_symbol_num
        = cfg->dl_data_symbol_num_perframe * kFrameWnd;

    size_t dl_socket_buffer_status_size = cfg->BS_ANT_NUM
        * kFrameWnd * cfg->dl_data_symbol_num_perframe;
    size_t dl_socket_buffer_size
        = cfg->packet_length * dl_socket_buffer_status_size;
    alloc_buffer_1d(&dl_socket_buffer_, dl_socket_buffer_size, 64, 0);
    alloc_buffer_1d(
        &dl_socket_buffer_status_, dl_socket_buffer_status_size, 64, 1);

    dl_bits_buffer_.calloc(
        task_buffer_symbol_num, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    size_t dl_bits_buffer_status_size
        = task_buffer_symbol_num * cfg->LDPC_config.nblocksInSymbol;
    dl_bits_buffer_status_.calloc(cfg->UE_NUM, dl_bits_buffer_status_size, 64);

    dl_ifft_buffer_.calloc(
        cfg->BS_ANT_NUM * task_buffer_symbol_num, cfg->OFDM_CA_NUM, 64);
    calib_buffer_.calloc(
        kFrameWnd, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);
    dl_encoded_buffer_.calloc(task_buffer_symbol_num,
        roundup<64>(cfg->OFDM_DATA_NUM) * cfg->UE_NUM, 64);
    dl_encoded_buffer_to_precode_.calloc(task_buffer_symbol_num,
        roundup<64>(cfg->OFDM_DATA_NUM) * cfg->UE_NUM, 64);
}

void Agora::freeUplinkBuffers()
{
    if (config_->use_time_domain_iq) {
        time_domain_iq_buffer_.free();
        freq_domain_iq_buffer_to_send_.free();
    }
    freq_domain_iq_buffer_.free();
    equal_buffer_.free();
}

void Agora::freeDownlinkBuffers()
{
    free_buffer_1d(&dl_socket_buffer_);
    free_buffer_1d(&dl_socket_buffer_status_);

    dl_ifft_buffer_.free();
    calib_buffer_.free();
    dl_encoded_buffer_.free();
}

void Agora::saveDecodeDataToFile(int frame_id)
{
    auto& cfg = config_;
    const size_t num_decoded_bytes
        = cfg->num_bytes_per_cb * cfg->LDPC_config.nblocksInSymbol;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/decode_data.bin";
    printf("Saving decode data to %s, num_decoded_bytes = %lu\n",
        filename.c_str(), num_decoded_bytes);
    FILE* fp = fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->ul_data_symbol_num_perframe; i++) {
        for (size_t j = 0; j < cfg->UE_NUM; j++) {
            uint8_t* ptr = decoded_buffer_[frame_id % kFrameWnd][i][j];
            fwrite(ptr, num_decoded_bytes, sizeof(uint8_t), fp);
        }
    }
    fclose(fp);
}

void Agora::saveTxDataToFile(UNUSED int frame_id)
{
    auto& cfg = config_;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/tx_data.bin";
    printf("Saving TX data to %s\n", filename.c_str());
    FILE* fp = fopen(filename.c_str(), "wb");

    for (size_t i = 0; i < cfg->dl_data_symbol_num_perframe; i++) {
        size_t total_data_symbol_id
            = cfg->get_total_data_symbol_idx_dl(frame_id, i);

        for (size_t ant_id = 0; ant_id < cfg->BS_ANT_NUM; ant_id++) {
            size_t offset = total_data_symbol_id * cfg->BS_ANT_NUM + ant_id;
            size_t packet_length = config_->packet_length;
            struct Packet* pkt
                = (struct Packet*)(&dl_socket_buffer_[offset * packet_length]);
            short* socket_ptr = pkt->data_;
            fwrite(socket_ptr, cfg->sampsPerSymbol * 2, sizeof(short), fp);
        }
    }
    fclose(fp);
}

void Agora::saveLatencyDataToFile()
{
    auto& cfg = config_;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/frame_latency.txt";
    printf("Saving frame latency data to %s, ghz=%lf\n", filename.c_str(), freq_ghz_);
    FILE* fp = fopen(filename.c_str(), "w");

    for (size_t i = 0; i < cfg->frames_to_test; i ++) {
        fprintf(fp, "%zu %lu %lu %lu %lu %lu\n", i, shared_state_.frame_start_time_[i],
            shared_state_.frame_iq_time_[i],
            shared_state_.frame_sc_time_[i],
            shared_state_.frame_decode_time_[i],
            shared_state_.frame_end_time_[i]);
    }
    fclose(fp);
}

void Agora::initControlInfo()
{
    auto& cfg = config_;
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);

    std::string filename_input = cur_directory
        + "/data/control_ue_template.bin";
    FILE* fp_input = fopen(filename_input.c_str(), "rb");
    for (size_t i = 0; i < cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
        size_t num_ue = cfg->user_level_list[i / cfg->num_load_levels];
        std::vector<ControlInfo> info_list;
        ControlInfo tmp;
        for (size_t j = 0; j < num_ue; j ++) {
            fread(&tmp, sizeof(ControlInfo), 1, fp_input);
            info_list.push_back(tmp);
        }
        control_info_table_.push_back(info_list);
    }
    fclose(fp_input);

    control_idx_list_.resize(cfg->frames_to_test);
    filename_input = cur_directory + "/data/control_ue.bin";
    fp_input = fopen(filename_input.c_str(), "rb");
    for (size_t i = 0; i < cfg->frames_to_test; i ++) {
        fread(&control_idx_list_[i], sizeof(size_t), 1, fp_input);
    }
    fclose(fp_input);
}

extern "C" {
EXPORT Agora* Agora_new(Config* cfg)
{
    // printf("Size of Agora: %d\n",sizeof(Agora *));
    auto* agora = new Agora(cfg);

    return agora;
}
EXPORT void Agora_start(Agora* agora) { agora->Start(); }
EXPORT void Agora_stop(/*Agora *agora*/)
{
    SignalHandler::setExitSignal(true); /*agora->stop();*/
}
EXPORT void Agora_destroy(Agora* agora) { delete agora; }
}
