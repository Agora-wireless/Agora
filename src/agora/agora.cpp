#include "agora.hpp"
using namespace std;

Agora::Agora(Config* cfg)
    : freq_ghz(measure_rdtsc_freq())
    , base_worker_core_offset(cfg->core_offset + 2 + cfg->socket_thread_num)
    , rx_status_(cfg)
    , demul_status_(cfg)
    , demod_status_(cfg)
    , encode_status_(cfg)
    , precode_status_(cfg)
{
    std::string directory = TOSTRING(PROJECT_DIRECTORY);
    printf("Agora: project directory [%s], RDTSC frequency = %.2f GHz\n",
        directory.c_str(), freq_ghz);

    this->config_ = cfg;
    if (kDebugPrintPilot) {
        cout << "Agora: Pilot data: " << endl;
        for (size_t i = 0; i < cfg->OFDM_DATA_NUM; i++)
            cout << config_->pilots_[i].re << "+1i*" << config_->pilots_[i].im
                 << ",";
        cout << endl;
    }

    pin_to_core_with_offset(
        ThreadType::kMaster, cfg->core_offset, 0, false /* quiet */);
    initialize_uplink_buffers();

    if (config_->dl_data_symbol_num_perframe > 0) {
        printf("Agora: Initializing downlink buffers\n");
        initialize_downlink_buffers();
    }

    stats = new Stats(cfg, kMaxStatBreakdown, freq_ghz);
    phy_stats = new PhyStats(cfg);

    if (config_->dynamic_workload) {
        init_control_info();
    }

    /* Initialize TXRX threads */
    packet_tx_rx_.reset(new PacketTXRX(cfg, cfg->core_offset + 1,
        &rx_status_, &demul_status_, &demod_status_, &encode_status_, &precode_status_));

    // if (kEnableMac) {
    //     const size_t mac_cpu_core = cfg->core_offset + cfg->socket_thread_num
    //         + cfg->worker_thread_num + 1;
    //     mac_thread_ = new MacThread(MacThread::Mode::kServer, cfg, mac_cpu_core,
    //         decoded_buffer_, nullptr /* ul bits */,
    //         nullptr /* ul bits status */, &dl_bits_buffer_,
    //         &dl_bits_buffer_status_, &mac_request_queue_, &mac_response_queue_);

    //     mac_std_thread_ = std::thread(&MacThread::run_event_loop, mac_thread_);
    // }

    // if (config_->test_mode >= 2) {
    //     goto creation_end;
    // }
    
    do_fft_threads_.resize(1);
    do_fft_threads_[0] = std::thread(&Agora::fft_worker, this, 0);

    /* Create worker threads */
    do_subcarrier_threads_.resize(
        (cfg->get_num_sc_per_server() + cfg->subcarrier_block_size - 1) / cfg->subcarrier_block_size);

    for (size_t i = 0; i < do_subcarrier_threads_.size(); i++) {
        do_subcarrier_threads_[i]
            = std::thread(&Agora::subcarrier_worker, this, i);
    }

    // if (config_->test_mode >= 1) {
    //     goto creation_end;
    // }

    if (cfg->downlink_mode) {
        do_encode_threads_.resize(cfg->get_num_ues_to_process());
        for (size_t i = 0; i < do_encode_threads_.size(); i ++) {
            do_encode_threads_[i]
                = std::thread(&Agora::encode_worker, this, i);
        }
    } else {
        do_decode_threads_.resize(cfg->get_num_ues_to_process() * cfg->decode_thread_num_per_ue);
        for (size_t i = 0; i < do_decode_threads_.size(); i++) {
            do_decode_threads_[i]
                = std::thread(&Agora::decode_worker, this, i);
        }
    }

creation_end:
    printf("Master thread core %zu, TX/RX thread cores %zu--%zu, worker thread "
           "cores %zu--%zu\n",
        cfg->core_offset, cfg->core_offset + 1,
        cfg->core_offset + 1 + cfg->socket_thread_num - 1,
        base_worker_core_offset,
        base_worker_core_offset + do_subcarrier_threads_.size() + cfg->get_num_ues_to_process() - 1);

}

Agora::~Agora()
{
    free_uplink_buffers();
    /* Downlink */
    if (config_->dl_data_symbol_num_perframe > 0)
        free_downlink_buffers();

    if (kEnableMac)
        mac_std_thread_.join();
    delete mac_thread_;

    for (auto& t : do_subcarrier_threads_)
        t.join();
    if (config_->downlink_mode) {
        for (auto& t : do_encode_threads_)
            t.join();
    } else {
        for (auto& t : do_decode_threads_)
            t.join();
    }
}

void Agora::stop()
{
    std::cout << "Agora: stopping threads" << std::endl;
    config_->running = false;
    usleep(1000);
    packet_tx_rx_.reset();
}

void Agora::start()
{
    auto& cfg = config_;

    // Start packet I/O
    if (!packet_tx_rx_->startTXRX(socket_buffer_,
            stats->frame_start, &dl_ifft_buffer_,
            &demod_buffers_, &demod_soft_buffer_to_decode_, &dl_encoded_buffer_,
            &dl_encoded_buffer_to_precode_)) {
        this->stop();
        return;
    }

    while (cfg->running && !SignalHandler::gotExitSignal()) {
        if (cfg->downlink_mode) {
            for (size_t i = 0; i < cfg->socket_thread_num; i ++) {
                if (packet_tx_rx_->frame_to_send_[i] < cfg->frames_to_test) {
                    goto keep_sleep;
                }
            }
            cfg->running = false;
            goto finish;
        } else {
            if (rx_status_.cur_frame_ == cfg->frames_to_test) {
                cfg->running = false;
                goto finish;
            }
        }
    keep_sleep:
        sleep(1);
    }
    cfg->running = false;
    goto finish;
    return;

finish:

    printf("Agora: printing stats and saving to file\n");
    stats->print_summary();
    stats->save_to_file();
    if (flags.enable_save_decode_data_to_file) {
        // TODO: fix it
        save_decode_data_to_file(0);
    }
    if (flags.enable_save_tx_data_to_file) {
        // TODO: fix it
        save_tx_data_to_file(0);
    }

    // Printing latency stats
    save_latency_data_to_file();

    // Calculate and print per-user BER
    if (!kEnableMac && kPrintPhyStats) {
        phy_stats->print_phy_stats();
    }
    this->stop();
}

void* Agora::subcarrier_worker(int tid)
{
    pin_to_core_with_offset(
        ThreadType::kWorkerSubcarrier, base_worker_core_offset + 1, tid + do_fft_threads_.size());

    Range sc_range(tid * config_->subcarrier_block_size + 
        config_->bs_server_addr_idx * config_->get_num_sc_per_server(),
        min((tid + 1) * config_->subcarrier_block_size + 
        config_->bs_server_addr_idx * config_->get_num_sc_per_server(), 
        (config_->bs_server_addr_idx + 1) * config_->get_num_sc_per_server()));

    if (config_->dynamic_workload) {
        auto computeSubcarrier = new DySubcarrier(config_, tid, freq_ghz,
            sc_range,
            socket_buffer_, csi_buffers_, calib_buffer_,
            dl_encoded_buffer_to_precode_, demod_buffers_, dl_ifft_buffer_,
            ue_spec_pilot_buffer_, equal_buffer_, ul_zf_matrices_, dl_zf_matrices_,
            control_info_table_, control_idx_list_,
            phy_stats, stats, &rx_status_, &demul_status_, &precode_status_);

        computeSubcarrier->start_work();
        delete computeSubcarrier;
    } else {
        auto computeSubcarrier = new DoSubcarrier(config_, tid, freq_ghz,
            sc_range,
            socket_buffer_, csi_buffers_, calib_buffer_,
            dl_encoded_buffer_to_precode_, demod_buffers_, dl_ifft_buffer_,
            ue_spec_pilot_buffer_, equal_buffer_, ul_zf_matrices_, dl_zf_matrices_,
            phy_stats, stats, &rx_status_, &demul_status_, &precode_status_);

        computeSubcarrier->start_work();
        delete computeSubcarrier;
    }

    return nullptr;
}

void* Agora::decode_worker(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerDecode, base_worker_core_offset + 1,
        tid + do_fft_threads_.size() + do_subcarrier_threads_.size());

    if (config_->dynamic_workload) {
        auto computeDecoding = new DyDecode(config_, tid, freq_ghz,
            demod_buffers_, demod_soft_buffer_to_decode_,
            decoded_buffer_, control_info_table_, control_idx_list_, 
            phy_stats, stats, &rx_status_, &demod_status_);

        computeDecoding->start_work();
        delete computeDecoding;
    } else {
        auto computeDecoding = new DoDecode(config_, tid, freq_ghz,
            demod_buffers_, demod_soft_buffer_to_decode_,
            decoded_buffer_, phy_stats, stats, &rx_status_, &demod_status_);

        computeDecoding->start_work();
        delete computeDecoding;
    }
    
    return nullptr;
}

void* Agora::fft_worker(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorkerFFT, base_worker_core_offset + 1,
        tid);
}

void* Agora::encode_worker(int tid)
{
    pin_to_core_with_offset(ThreadType::kWorker, base_worker_core_offset + 1,
        tid + do_subcarrier_threads_.size());

    auto computeEncoding = new DoEncode(config_, tid, freq_ghz,
        config_->dl_bits, dl_encoded_buffer_, stats, &rx_status_, &encode_status_);

    computeEncoding->start_work();
    delete computeEncoding;
    return nullptr;
}

void Agora::update_ran_config(RanConfig rc)
{
    config_->update_mod_cfgs(rc.mod_order_bits);
}

// Not used
void Agora::update_rx_counters(size_t frame_id, size_t symbol_id)
{
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    if (config_->isPilot(frame_id, symbol_id)) {
        rx_counters_.num_pilot_pkts[frame_slot]++;
        if (rx_counters_.num_pilot_pkts[frame_slot]
            == rx_counters_.num_pilot_pkts_per_frame) {
            rx_counters_.num_pilot_pkts[frame_slot] = 0;
            stats->master_set_tsc(TsType::kPilotAllRX, frame_id);
            print_per_frame_done(PrintType::kPacketRXPilots, frame_id);
        }
    } else if (config_->isCalDlPilot(frame_id, symbol_id)
        or config_->isCalUlPilot(frame_id, symbol_id)) {
        if (++rx_counters_.num_reciprocity_pkts[frame_slot]
            == rx_counters_.num_reciprocity_pkts_per_frame) {
            rx_counters_.num_reciprocity_pkts[frame_slot] = 0;
            stats->master_set_tsc(TsType::kRCAllRX, frame_id);
        }
    }
    if (rx_counters_.num_pkts[frame_slot] == 0) {
        stats->master_set_tsc(TsType::kPilotRX, frame_id);
        if (kDebugPrintPerFrameStart) {
            const size_t prev_frame_slot
                = (frame_slot + TASK_BUFFER_FRAME_NUM - 1)
                % TASK_BUFFER_FRAME_NUM;
            printf("Main thread: data received from frame %zu, symbol %zu, in "
                   "%.2f us. RX in prev frame: %zu\n",
                frame_id, symbol_id,
                stats->master_get_delta_us(
                    TsType::kPilotRX, frame_id, frame_id - 1),
                rx_counters_.num_pkts[prev_frame_slot]);
        }
    }

    rx_counters_.num_pkts[frame_slot]++;
    if (rx_counters_.num_pkts[frame_slot] == rx_counters_.num_pkts_per_frame) {
        stats->master_set_tsc(TsType::kRXDone, frame_id);
        print_per_frame_done(PrintType::kPacketRX, frame_id);
        rx_counters_.num_pkts[frame_slot] = 0;
    }
}

void Agora::print_per_frame_done(PrintType print_type, size_t frame_id)
{
    if (!kDebugPrintPerFrameDone)
        return;
    switch (print_type) {
    case (PrintType::kPacketRXPilots):
        printf("Main [frame %zu + %.2f ms]: Received all pilots\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kPilotAllRX, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketRX): {
        printf("Main [frame %zu + %.2f ms]: Received all packets\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kRXDone, TsType::kPilotRX, frame_id));
    } break;
    case (PrintType::kFFTPilots):
        printf("Main [frame %zu + %.2f ms]: FFT-ed all pilots\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kFFTPilotsDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kFFTCal):
        printf("Main [frame %zu + %.2f ms]: FFT-ed all calibration symbols\n",
            frame_id,
            stats->master_get_us_since(TsType::kRCAllRX, frame_id) / 1000.0);
        break;
    case (PrintType::kRC):
        printf("Main thread: Reciprocity Calculation done frame: %zu in "
               "%.2f us since reciprocity pilots all received\n",
            frame_id,
            stats->master_get_delta_us(
                TsType::kRCDone, TsType::kRCAllRX, frame_id));
        break;
    case (PrintType::kZF):
        printf("Main [frame %zu + %.2f ms]: Completed zero-forcing\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kZFDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kDemul):
        printf("Main [frame %zu + %.2f ms]: Completed demodulation\n", frame_id,
            stats->master_get_delta_ms(
                TsType::kDemulDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kDecode):
        printf("Main [frame %zu + %.2f ms]: Completed LDPC decoding\n",
            frame_id,
            stats->master_get_delta_ms(
                TsType::kDecodeDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kEncode):
        printf("Main thread: Encoding done frame: %zu in %.2f us since ZF "
               "done, total %.2f us\n",
            frame_id,
            stats->master_get_delta_us(
                TsType::kEncodeDone, TsType::kZFDone, frame_id),
            stats->master_get_delta_us(
                TsType::kEncodeDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPrecode):
        printf("Main thread: Precoding done frame: %zu in %.2f us since ZF "
               "done, total: %.2f us\n",
            frame_id,
            stats->master_get_delta_us(
                TsType::kPrecodeDone, TsType::kZFDone, frame_id),
            stats->master_get_delta_us(
                TsType::kPrecodeDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kIFFT):
        printf("Main thread: IFFT done frame: %zu in %.2f us since precode "
               "done, total: %.2f us\n",
            frame_id,
            stats->master_get_delta_us(
                TsType::kIFFTDone, TsType::kPrecodeDone, frame_id),
            stats->master_get_delta_us(
                TsType::kIFFTDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketTXFirst):
        printf("Main thread: TX of first symbol done frame: %zu in %.2f "
               "us since ZF done, total: %.2f us\n",
            frame_id,
            stats->master_get_delta_us(
                TsType::kTXProcessedFirst, TsType::kZFDone, frame_id),
            stats->master_get_delta_us(
                TsType::kTXProcessedFirst, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketTX):
        printf("Main thread: TX done frame: %zu (%zu DL symbols) in %.2f us "
               "since ZF done, total: %.2f us\n",
            frame_id, config_->dl_data_symbol_num_perframe,
            stats->master_get_delta_us(
                TsType::kTXDone, TsType::kZFDone, frame_id),
            stats->master_get_delta_us(
                TsType::kTXDone, TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketToMac):
        printf("Main thread: MAC TX done frame: %zu, in %.2f us\n", frame_id,
            stats->master_get_us_since(TsType::kPilotRX, frame_id));
        break;
    default:
        printf("Wrong task type in frame done print!");
    }
}

void Agora::print_per_symbol_done(
    PrintType print_type, size_t frame_id, size_t symbol_id)
{
    if (!kDebugPrintPerSymbolDone)
        return;
    switch (print_type) {
    case (PrintType::kFFTPilots):
        printf("Main thread: pilot FFT done frame: %zu, symbol: %zu, num "
               "symbols done: %zu\n",
            frame_id, symbol_id, fft_stats_.get_symbol_count(frame_id));
        break;
    case (PrintType::kFFTData):
        printf(
            "Main thread: data FFT done frame %zu, symbol %zu, precoder "
            "status: %d, fft queue: %zu, zf queue: %zu, demul queue: %zu, in "
            "%.2f\n",
            frame_id, symbol_id, zf_stats_.coded_frame == frame_id,
            get_conq(EventType::kFFT)->size_approx(),
            get_conq(EventType::kZF)->size_approx(),
            get_conq(EventType::kDemul)->size_approx(),
            stats->master_get_us_since(TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kRC):
        printf("Main thread: cal symbol FFT done frame: %zu, symbol: %zu, "
               "num symbols done: %zu\n",
            frame_id, symbol_id, fft_stats_.symbol_rc_count[frame_id]);
        break;
    case (PrintType::kDemul):
        printf("Main thread: Demodulation done frame %zu, symbol: %zu, num "
               "symbols done: %zu in %.2f\n",
            frame_id, symbol_id, demul_stats_.get_symbol_count(frame_id),
            stats->master_get_us_since(TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kDecode):
        printf("Main thread: Decoding done frame %zu, symbol: %zu, num "
               "symbols done: %zu\n",
            frame_id, symbol_id, decode_stats_.get_symbol_count(frame_id));
        break;
    case (PrintType::kEncode):
        printf("Main thread: Encoding done frame %zu, symbol: %zu, num "
               "symbols done: %zu\n",
            frame_id, symbol_id, encode_stats_.get_symbol_count(frame_id));
        break;
    case (PrintType::kPrecode):
        printf("Main thread: Precoding done frame: %zu, symbol: %zu in %.2f "
               "us\n",
            frame_id, symbol_id,
            stats->master_get_us_since(TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketTX):
        printf("Main thread: TX done frame: %zu, symbol: %zu in %.2f us\n",
            frame_id, symbol_id,
            stats->master_get_us_since(TsType::kPilotRX, frame_id));
        break;
    case (PrintType::kPacketToMac):
        printf("Main thread: MAC TX done frame: %zu, symbol: %zu in %.2f us\n",
            frame_id, symbol_id,
            stats->master_get_us_since(TsType::kPilotRX, frame_id));
        break;
    default:
        printf("Wrong task type in frame done print!");
    }
}

void Agora::print_per_task_done(PrintType print_type, size_t frame_id,
    size_t symbol_id, size_t ant_or_sc_id)
{
    if (!kDebugPrintPerTaskDone)
        return;
    switch (print_type) {
    case (PrintType::kZF):
        printf("Main thread: ZF done frame: %zu, subcarrier %zu\n", frame_id,
            ant_or_sc_id);
        break;
    case (PrintType::kRC):
        printf("Main thread: RC done frame: %zu, subcarrier %zu\n", frame_id,
            ant_or_sc_id);
        break;
    case (PrintType::kDemul):
        printf("Main thread: Demodulation done frame: %zu, symbol: %zu, sc: "
               "%zu, num blocks done: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            demul_stats_.get_task_count(frame_id, symbol_id));
        break;
    case (PrintType::kDecode):
        printf("Main thread: Decoding done frame: %zu, symbol: %zu, sc: %zu, "
               "num blocks done: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            decode_stats_.get_task_count(frame_id, symbol_id));
        break;
    case (PrintType::kPrecode):
        printf("Main thread: Precoding done frame: %zu, symbol: %zu, "
               "subcarrier: %zu, total SCs: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            precode_stats_.get_task_count(frame_id, symbol_id));
        break;
    case (PrintType::kIFFT):
        printf("Main thread: IFFT done frame: %zu, symbol: %zu, antenna: %zu, "
               "total ants: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            ifft_stats_.get_task_count(frame_id, symbol_id));
        break;
    case (PrintType::kPacketTX):
        printf("Main thread: TX done frame: %zu, symbol: %zu, antenna: %zu, "
               "total packets: %zu\n",
            frame_id, symbol_id, ant_or_sc_id,
            tx_stats_.get_task_count(frame_id, symbol_id));
        break;
    default:
        printf("Wrong task type in frame done print!");
    }
}

void Agora::initialize_uplink_buffers()
{
    auto& cfg = config_;
    const size_t task_buffer_symbol_num_ul
        = cfg->ul_data_symbol_num_perframe * kFrameWnd;

    socket_buffer_size_ = cfg->packet_length * kFrameWnd * cfg->symbol_num_perframe;

    socket_buffer_.malloc(cfg->BS_ANT_NUM,
        socket_buffer_size_, 64);

    after_fft_buffer_.malloc(cfg->BS_ANT_NUM, socket_buffer_size_, 64);
    after_fft_buffer_to_subcarrier_.malloc(cfg->BS_ANT_NUM, socket_buffer_size_, 64);

    csi_buffers_.alloc(kFrameWnd, cfg->UE_NUM, cfg->BS_ANT_NUM * cfg->OFDM_DATA_NUM);
    ul_zf_matrices_.alloc(kFrameWnd, cfg->OFDM_DATA_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM);
    dl_zf_matrices_.alloc(kFrameWnd, cfg->OFDM_DATA_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM);

    demod_buffers_.alloc(kFrameWnd, cfg->symbol_num_perframe, cfg->UE_NUM, kMaxModType * cfg->OFDM_DATA_NUM);
    decoded_buffer_.alloc(kFrameWnd, cfg->symbol_num_perframe, cfg->UE_NUM, cfg->LDPC_config.nblocksInSymbol * roundup<64>(cfg->num_bytes_per_cb));

    equal_buffer_.malloc(
        task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
    ue_spec_pilot_buffer_.calloc(
        TASK_BUFFER_FRAME_NUM, cfg->UL_PILOT_SYMS * cfg->UE_NUM, 64);
    demod_soft_buffer_to_decode_.malloc(
        task_buffer_symbol_num_ul, 8 * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);

    rx_counters_.num_pkts_per_frame = cfg->BS_ANT_NUM
        * (cfg->pilot_symbol_num_perframe + cfg->ul_data_symbol_num_perframe);
    rx_counters_.num_pilot_pkts_per_frame
        = cfg->BS_ANT_NUM * cfg->pilot_symbol_num_perframe;
    rx_counters_.num_reciprocity_pkts_per_frame = cfg->BS_ANT_NUM;

    fft_created_count = 0;
    fft_stats_.init(cfg->BS_ANT_NUM, cfg->pilot_symbol_num_perframe,
        cfg->symbol_num_perframe);
    fft_stats_.max_symbol_data_count = cfg->ul_data_symbol_num_perframe;
    fft_stats_.symbol_rc_count.fill(0);
    fft_stats_.max_symbol_rc_count = cfg->BS_ANT_NUM;
    fft_stats_.cur_frame_for_symbol
        = std::vector<size_t>(cfg->ul_data_symbol_num_perframe, SIZE_MAX);

    zf_stats_.init(config_->zf_events_per_symbol);

    demul_stats_.init(config_->demul_events_per_symbol,
        cfg->ul_data_symbol_num_perframe, cfg->data_symbol_num_perframe);

    decode_stats_.init(config_->LDPC_config.nblocksInSymbol * cfg->UE_NUM,
        cfg->ul_data_symbol_num_perframe, cfg->data_symbol_num_perframe);

    tomac_stats_.init(cfg->UE_NUM, cfg->ul_data_symbol_num_perframe,
        cfg->data_symbol_num_perframe);
}

void Agora::initialize_downlink_buffers()
{
    auto& cfg = config_;
    const size_t task_buffer_symbol_num
        = cfg->dl_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

    size_t dl_socket_buffer_status_size = cfg->BS_ANT_NUM
        * SOCKET_BUFFER_FRAME_NUM * cfg->dl_data_symbol_num_perframe;
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
        TASK_BUFFER_FRAME_NUM, cfg->OFDM_DATA_NUM * cfg->BS_ANT_NUM, 64);
    dl_encoded_buffer_.calloc(task_buffer_symbol_num,
        roundup<64>(cfg->OFDM_DATA_NUM) * cfg->UE_NUM, 64);
    dl_encoded_buffer_to_precode_.calloc(task_buffer_symbol_num,
        roundup<64>(cfg->OFDM_DATA_NUM) * cfg->UE_NUM, 64);

    frommac_stats_.init(config_->UE_NUM, cfg->dl_data_symbol_num_perframe,
        cfg->data_symbol_num_perframe);
    encode_stats_.init(config_->LDPC_config.nblocksInSymbol * cfg->UE_NUM,
        cfg->dl_data_symbol_num_perframe, cfg->data_symbol_num_perframe);
    precode_stats_.init(config_->demul_events_per_symbol,
        cfg->dl_data_symbol_num_perframe, cfg->data_symbol_num_perframe);
    ifft_stats_.init(cfg->BS_ANT_NUM, cfg->dl_data_symbol_num_perframe,
        cfg->data_symbol_num_perframe);
    tx_stats_.init(cfg->BS_ANT_NUM, cfg->dl_data_symbol_num_perframe,
        cfg->data_symbol_num_perframe);
}

void Agora::free_uplink_buffers()
{
    socket_buffer_.free();
    equal_buffer_.free();

    fft_stats_.fini();
    demul_stats_.fini();
    decode_stats_.fini();
}

void Agora::free_downlink_buffers()
{
    free_buffer_1d(&dl_socket_buffer_);
    free_buffer_1d(&dl_socket_buffer_status_);

    dl_ifft_buffer_.free();
    calib_buffer_.free();
    dl_encoded_buffer_.free();

    encode_stats_.fini();
    precode_stats_.fini();
    ifft_stats_.fini();
    tx_stats_.fini();
}

void Agora::save_decode_data_to_file(int frame_id)
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

void Agora::save_tx_data_to_file(UNUSED int frame_id)
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
            short* socket_ptr = pkt->data;
            fwrite(socket_ptr, cfg->sampsPerSymbol * 2, sizeof(short), fp);
        }
    }
    fclose(fp);
}

void Agora::save_latency_data_to_file()
{
    auto& cfg = config_;

    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/frame_latency.txt";
    printf("Saving frame latency data to %s, ghz=%lf\n", filename.c_str(), freq_ghz);
    FILE* fp = fopen(filename.c_str(), "w");

    for (size_t i = 0; i < cfg->frames_to_test; i ++) {
        // fprintf(fp, "%u %lu %lf %lf %lf %lf %lf\n", i, rx_status_.frame_start_time_[i],
        //     ((int64_t)rx_status_.frame_end_time_[i] - (int64_t)rx_status_.frame_start_time_[i]) / 1000.0, 
        //     ((int64_t)rx_status_.frame_iq_time_[i] - (int64_t)rx_status_.frame_start_time_[i]) / 1000.0,
        //     ((int64_t)demul_status_.frame_sc_time_[i] - (int64_t)rx_status_.frame_iq_time_[i]) / 1000.0,
        //     ((int64_t)demod_status_.frame_decode_time_[i] - (int64_t)demul_status_.frame_sc_time_[i]) / 1000.0,
        //     ((int64_t)rx_status_.frame_end_time_[i] - (int64_t)demod_status_.frame_decode_time_[i]) / 1000.0);
        fprintf(fp, "%u %lu %lu %lu %lu %lu\n", i, rx_status_.frame_start_time_[i],
            rx_status_.frame_iq_time_[i],
            demul_status_.frame_sc_time_[i],
            demod_status_.frame_decode_time_[i],
            rx_status_.frame_end_time_[i]);
    }
    fclose(fp);
}

void Agora::getEqualData(float** ptr, int* size)
{
    auto& cfg = config_;
    auto offset = cfg->get_total_data_symbol_idx_ul(
        max_equaled_frame, cfg->UL_PILOT_SYMS);
    *ptr = (float*)&equal_buffer_[offset][0];
    *size = cfg->UE_NUM * cfg->OFDM_DATA_NUM * 2;
}

void Agora::init_control_info()
{
    auto& cfg = config_;
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);

    std::string filename_input = cur_directory
        + "/data/control_ue_template.bin";
    FILE* fp_input = fopen(filename_input.c_str(), "rb");
    // for (size_t i = 0; i < kNumSlot; i++) {
    for (size_t i = 0; i < cfg->user_level_list.size() * cfg->num_load_levels; i ++) {
        // size_t num_ue = (i / kNumLoadSetting + 1) * (cfg->UE_NUM / kNumUESetting);
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
EXPORT void Agora_start(Agora* agora) { agora->start(); }
EXPORT void Agora_stop(/*Agora *agora*/)
{
    SignalHandler::setExitSignal(true); /*agora->stop();*/
}
EXPORT void Agora_destroy(Agora* agora) { delete agora; }
EXPORT void Agora_getEqualData(Agora* agora, float** ptr, int* size)
{
    return agora->getEqualData(ptr, size);
}
}
