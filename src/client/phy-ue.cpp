#include "phy-ue.hpp"

Phy_UE::Phy_UE(Config* config)
    : ul_bits(config->ul_bits)
    , ul_iq_f(config->ul_iq_f)
{
    srand(time(NULL));

    this->config_ = config;
    initialize_vars_from_cfg();

    std::vector<size_t> data_sc_ind_;
    for (size_t i = data_sc_start; i < data_sc_start + data_sc_len; i++)
        data_sc_ind_.push_back(i);

    non_null_sc_len = data_sc_len;
    non_null_sc_ind_.insert(
        non_null_sc_ind_.end(), data_sc_ind_.begin(), data_sc_ind_.end());
    std::sort(non_null_sc_ind_.begin(), non_null_sc_ind_.end());

    ue_pilot_vec.resize(antenna_num);
    for (size_t i = 0; i < antenna_num; i++) {
        for (size_t j = prefix_len; j < config_->sampsPerSymbol - postfix_len;
             j++) {
            ue_pilot_vec[i].push_back(std::complex<float>(
                config_->ue_specific_pilot_t[i][j].real() / 32768.0,
                config_->ue_specific_pilot_t[i][j].imag() / 32768.0));
        }
    }

    fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * dl_symbol_perframe * antenna_num * 36);
    demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * dl_data_symbol_perframe * antenna_num * 36);
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * symbol_perframe * antenna_num * 36);
    modul_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * ul_data_symbol_perframe * antenna_num * 36);
    ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * ul_symbol_perframe * antenna_num * 36);
    tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * ul_symbol_perframe * antenna_num * 36);

    for (size_t i = 0; i < rx_thread_num; i++) {
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);
        tx_ptoks_ptr[i] = new moodycamel::ProducerToken(tx_queue_);
    }

    ru_.reset(new RU(config_, rx_thread_num, config_->core_offset + 1,
        &message_queue_, &tx_queue_, rx_ptoks_ptr, tx_ptoks_ptr));

    printf("initializing buffers...\n");

    //////////////////////////////////////
    //////// uplink buffers init (tx) ////
    //////////////////////////////////////

    // initialize ul data buffer
    // l2_buffer_status_.resize(TASK_BUFFER_FRAME_NUM *
    // ul_data_symbol_perframe);

    // initialize modulation buffer
    modul_buffer_.calloc(ul_data_symbol_perframe * TASK_BUFFER_FRAME_NUM,
        data_sc_len * antenna_num, 64);

    // initialize IFFT buffer
    size_t ifft_buffer_block_num
        = antenna_num * ul_data_symbol_perframe * TASK_BUFFER_FRAME_NUM;
    ifft_buffer_.calloc(ifft_buffer_block_num, FFT_LEN, 64);

    alloc_buffer_1d(&tx_buffer_, tx_buffer_size, 64, 0);
    alloc_buffer_1d(&tx_buffer_status_, tx_buffer_status_size, 64, 1);

    ////////////////////////////////////////
    //////// downlink buffers init (rx) ////
    ////////////////////////////////////////

    // initialize rx buffer
    rx_buffer_.malloc(rx_thread_num, rx_buffer_size, 64);
    rx_buffer_status_.calloc(rx_thread_num, rx_buffer_status_size, 64);

    // initialize FFT buffer
    size_t FFT_buffer_block_num
        = antenna_num * dl_symbol_perframe * TASK_BUFFER_FRAME_NUM;
    fft_buffer_.calloc(FFT_buffer_block_num, FFT_LEN, 64);

    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, FFT_LEN);
    (void)DftiCommitDescriptor(mkl_handle);

    // initialize CSI buffer
    csi_buffer_.resize(antenna_num * TASK_BUFFER_FRAME_NUM);
    for (size_t i = 0; i < csi_buffer_.size(); i++)
        csi_buffer_[i].resize(non_null_sc_len);

    if (dl_data_symbol_perframe > 0) {
        // initialize equalized data buffer
        equal_buffer_.resize(
            antenna_num * dl_data_symbol_perframe * TASK_BUFFER_FRAME_NUM);
        for (size_t i = 0; i < equal_buffer_.size(); i++)
            equal_buffer_[i].resize(non_null_sc_len);

        // initialize data buffer
        dl_data_buffer_.resize(
            antenna_num * dl_data_symbol_perframe * TASK_BUFFER_FRAME_NUM);
        for (size_t i = 0; i < dl_data_buffer_.size(); i++)
            dl_data_buffer_[i].resize(data_sc_len);
    }

    init_modulation_table(qam_table, config_->mod_type);

    // initilize all kinds of checkers
    memset(csi_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(data_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM * antenna_num);

    memset(demul_status_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    if (dl_data_symbol_perframe > 0) {
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            demul_checker_[i] = new size_t[dl_data_symbol_perframe];
            memset(
                demul_checker_[i], 0, sizeof(int) * (dl_data_symbol_perframe));
        }
    }

    // create task thread
    for (size_t i = 0; i < worker_thread_num; i++) {
        auto* context = new EventHandlerContext();
        context->obj_ptr = this;
        context->id = i;

        // printf("create thread %d\n", i);
        if (pthread_create(&task_threads[i], NULL, taskThread_launch, context)
            != 0) {
            perror("task thread create failed");
            exit(0);
        }
    }
}

Phy_UE::~Phy_UE()
{
    DftiFreeDescriptor(&mkl_handle);
    // release FFT_buffer
    fft_buffer_.free();
    ifft_buffer_.free();
}

void Phy_UE::schedule_task(Event_data do_task,
    moodycamel::ConcurrentQueue<Event_data>* in_queue,
    moodycamel::ProducerToken const& ptok)
{
    if (!in_queue->try_enqueue(ptok, do_task)) {
        printf("need more memory\n");
        if (!in_queue->enqueue(ptok, do_task)) {
            printf("task enqueue failed\n");
            exit(0);
        }
    }
}

//////////////////////////////////////////////////////////
//                   UPLINK Operations                  //
//////////////////////////////////////////////////////////
void Phy_UE::stop()
{
    std::cout << "stopping threads " << std::endl;
    config_->running = false;
    usleep(1000);
    ru_.reset();
}

void Phy_UE::start()
{
    pin_to_core_with_offset(ThreadType::kMaster, core_offset, 0);

    if (!ru_->startTXRX(rx_buffer_, rx_buffer_status_, rx_buffer_status_size,
            rx_buffer_size, tx_buffer_, tx_buffer_status_,
            tx_buffer_status_size, tx_buffer_size)) {
        this->stop();
        return;
    }

    // for task_queue, main thread is producer, it is single-procuder & multiple
    // consumer for task queue uplink

    // TODO: make the producertokens global and try
    // "try_dequeue_from_producer(token,item)"
    //       combine the task queues into one queue
    moodycamel::ProducerToken ptok_fft(fft_queue_);
    moodycamel::ProducerToken ptok_modul(modul_queue_);
    moodycamel::ProducerToken ptok_demul(demul_queue_);
    moodycamel::ProducerToken ptok_ifft(ifft_queue_);
    // moodycamel::ProducerToken ptok_tx(tx_queue_);

    // for message_queue, main thread is a consumer, it is multiple producers
    // & single consumer for message_queue
    moodycamel::ConsumerToken ctok(message_queue_);

    // counter for print log
    size_t demul_count = 0;
    auto demul_begin = std::chrono::system_clock::now();
    int miss_count = 0;
    int total_count = 0;

    Event_data events_list[dequeue_bulk_size];
    int ret = 0;
    max_equaled_frame = 0;
    size_t frame_id, symbol_id, ant_id;
    while (config_->running && !SignalHandler::gotExitSignal()) {
        // get a bulk of events
        ret = message_queue_.try_dequeue_bulk(
            ctok, events_list, dequeue_bulk_size);
        total_count++;
        if (total_count == 1e7) {
            // print the message_queue_ miss rate is needed
            // printf("message dequeue miss rate %f\n", (float)miss_count /
            // total_count);
            total_count = 0;
            miss_count = 0;
        }
        if (ret == 0) {
            miss_count++;
            continue;
        }
        // handle each event
        for (int bulk_count = 0; bulk_count < ret; bulk_count++) {
            Event_data& event = events_list[bulk_count];

            switch (event.event_type) {

                // if EVENT_RX_SYMBOL, do crop

            case EventType::kPacketRX: {
                //int offset = event.tags[0];
                size_t rx_thread_id = rx_tag_t(event.tags[0]).tid;
                size_t offset_in_current_buffer
                    = rx_tag_t(event.tags[0]).offset;

                struct Packet* pkt = (struct Packet*)(rx_buffer_[rx_thread_id]
                    + offset_in_current_buffer * packet_length);
                frame_id = pkt->frame_id;
                symbol_id = pkt->symbol_id;
                ant_id = pkt->ant_id;

                if (ul_data_symbol_perframe > 0
                    && symbol_id == config_->DLSymbols[0].front()
                    && ant_id % config_->nChannels == 0) {
                    /*Event_data do_ifft_task(EventType::kIFFT,
                        gen_tag_t::frm_sym_ant(
                            frame_id, symbol_id, ant_id / config_->nChannels)
                            ._tag);
                    schedule_task(do_ifft_task, &ifft_queue_, ptok_ifft);*/
                    Event_data do_modul_task(EventType::kModul,
                        gen_tag_t::frm_sym_ant(
                            frame_id, symbol_id, ant_id / config_->nChannels)
                            ._tag);
                    schedule_task(do_modul_task, &modul_queue_, ptok_modul);
                }

                if (dl_data_symbol_perframe > 0
                    && (config_->isPilot(frame_id, symbol_id)
                           || config_->isDownlink(frame_id, symbol_id))) {
                    Event_data do_fft_task(EventType::kFFT, event.tags[0]);
                    schedule_task(do_fft_task, &fft_queue_, ptok_fft);
                } else { // if we are not entering doFFT, reset buffer here
                    rx_buffer_status_[rx_thread_id][offset_in_current_buffer]
                        = 0; // now empty
                }
            } break;

            case EventType::kFFT: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                // checker to count # of pilots/users
                csi_checker_[frame_id]++;

                if (csi_checker_[frame_id]
                    == dl_pilot_symbol_perframe * antenna_num) {
                    csi_checker_[frame_id] = 0;
                    if (kDebugPrintPerFrameDone)
                        printf("Main thread: pilot frame: %zu, finished "
                               "collecting "
                               "pilot frames\n",
                            frame_id);
                }

            } break;

            case EventType::kModul: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).frame_id;
                size_t ue_id = gen_tag_t(event.tags[0]).ant_id;
                size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
                size_t modul_status_idx = frame_slot * nUEs + ue_id;

                data_checker_[modul_status_idx]++;
                if (data_checker_[modul_status_idx]
                    == ul_data_symbol_perframe) {
                    data_checker_[modul_status_idx] = 0;
                    Event_data do_ifft_task(EventType::kIFFT,
                        gen_tag_t::frm_sym_ant(frame_id, symbol_id, ue_id)
                            ._tag);
                    schedule_task(do_ifft_task, &ifft_queue_, ptok_ifft);
                    if (kDebugPrintPerFrameDone)
                        printf("Main thread: frame: %zu, finished modulating "
                               "uplink data for user %zu\n",
                            frame_id, ue_id);
                }
            } break;

            case EventType::kDemul: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

                demul_checker_[frame_id][symbol_id]++;
                // if this symbol is ready
                if (demul_checker_[frame_id][symbol_id] == antenna_num) {

                    if (kDebugPrintInTask)
                        printf("Main thread: Demodulation done frame: %zu, "
                               "symbol %zu\n",
                            frame_id, symbol_id);
                    max_equaled_frame = frame_id;
                    demul_checker_[frame_id][symbol_id] = 0;
                    demul_status_[frame_id]++;
                    if (demul_status_[frame_id] == dl_data_symbol_perframe) {
                        if (kDebugPrintPerTaskDone)
                            printf(
                                "Main thread: Demodulation done frame: %zu \n",
                                frame_id);
                        demul_status_[frame_id] = 0;
                    }
                    demul_count += 1;
                    // print log per 100 frames
                    if (demul_count == dl_data_symbol_perframe * 100) {
                        demul_count = 0;
                        auto demul_end = std::chrono::system_clock::now();
                        std::chrono::duration<double> diff
                            = demul_end - demul_begin;
                        int samples_num_per_UE
                            = FFT_LEN * dl_data_symbol_perframe * 100;
                        printf(
                            "Receive %d samples (per-client) from %zu clients "
                            "in %f secs, throughtput %f bps per-client "
                            "(16QAM), current task queue length %zu\n",
                            samples_num_per_UE, nUEs, diff.count(),
                            samples_num_per_UE * log2(16.0f) / diff.count(),
                            demul_queue_.size_approx());
                        demul_begin = std::chrono::system_clock::now();
                    }
                }
            } break;

            case EventType::kIFFT: {
                size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
                Event_data task(EventType::kPacketTX, event.tags[0]);
                try_enqueue_fallback(
                    &tx_queue_, tx_ptoks_ptr[ant_id % rx_thread_num], task);
            } break;

            case EventType::kPacketTX: {
                if (kDebugPrintPerSymbolDone) {
                    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                    size_t symbol_id = gen_tag_t(event.tags[0]).frame_id;
                    size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
                    printf("Main thread: finished TX for frame %zu, symbol "
                           "%zu, ant %zu\n",
                        frame_id, symbol_id, ant_id);
                }
            } break;

            default:
                printf("Wrong event type in message queue!");
                exit(0);
            }
        }
    }
    this->stop();
}

void* Phy_UE::taskThread_launch(void* in_context)
{
    EventHandlerContext* context = (EventHandlerContext*)in_context;
    Phy_UE* me = context->obj_ptr;
    int tid = context->id;
    delete context;
    me->taskThread(tid);
    return 0;
}

void Phy_UE::taskThread(int tid)
{
    // printf("task thread %d starts\n", tid);

    // attach task threads to specific cores
    // Note: cores 0-17, 36-53 are on the same socket
#ifdef ENABLE_CPU_ATTACH
    size_t offset_id = core_offset + rx_thread_num + 1;
    size_t tar_core_id = tid + offset_id;
    if (tar_core_id >= nCPUs) // FIXME: read the number of cores
        tar_core_id = (tar_core_id - nCPUs) + 2 * nCPUs;
    if (pin_to_core(tar_core_id) != 0) {
        printf("Task thread: pinning thread %d to core %zu failed\n", tid,
            tar_core_id);
        exit(0);
    } else {
        printf("Task thread: pinning thread %d to core %zu succeeded\n", tid,
            tar_core_id);
    }
#endif

    task_ptok[tid].reset(new moodycamel::ProducerToken(message_queue_));

    Event_data event;
    while (config_->running) {
        if (demul_queue_.try_dequeue(event))
            doDemul(tid, event.tags[0]);
        else if (ifft_queue_.try_dequeue(event))
            doIFFT(tid, event.tags[0]);
        if (modul_queue_.try_dequeue(event))
            doModul(tid, event.tags[0]);
        else if (fft_queue_.try_dequeue(event))
            doFFT(tid, event.tags[0]);
    }
}

//////////////////////////////////////////////////////////
//                   DOWNLINK Operations                  //
//////////////////////////////////////////////////////////

void Phy_UE::doFFT(int tid, size_t tag)
{
    //int buffer_frame_num = rx_buffer_status_size;
    //int rx_thread_id = offset / buffer_frame_num;
    //int offset_in_current_buffer = offset % buffer_frame_num;

    size_t rx_thread_id = fft_req_tag_t(tag).tid;
    size_t offset_in_current_buffer = fft_req_tag_t(tag).offset;

    // read info of one frame
    struct Packet* pkt = (struct Packet*)(rx_buffer_[rx_thread_id]
        + offset_in_current_buffer * packet_length);
    size_t frame_id = pkt->frame_id;
    size_t symbol_id = pkt->symbol_id;
    // int cell_id = pkt->cell_id;
    size_t ant_id = pkt->ant_id;
    size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;

    if (!config_->isPilot(frame_id, symbol_id)
        && !(config_->isDownlink(frame_id, symbol_id)))
        return;

#if DEBUG_DL_PILOT
    size_t sym_offset = 0;
    if (config_->isPilot(frame_id, symbol_id)) {
        std::vector<std::complex<float>> vec;
        size_t seq_len = ue_pilot_vec[ant_id].size();
        for (size_t i = 0; i < config_->sampsPerSymbol; i++)
            vec.push_back(std::complex<float>(
                pkt->data[2 * i] / 32768.0, pkt->data[2 * i + 1] / 32768.0));
        sym_offset
            = CommsLib::find_pilot_seq(vec, ue_pilot_vec[ant_id], seq_len);
        sym_offset = sym_offset < seq_len ? 0 : sym_offset - seq_len;
        float noise_power = 0;
        for (size_t i = 0; i < sym_offset; i++)
            noise_power += std::pow(std::abs(vec[i]), 2);
        float signal_power = 0;
        for (size_t i = sym_offset; i < 2 * sym_offset; i++)
            signal_power += std::pow(std::abs(vec[i]), 2);
        float SNR = 10 * std::log10(signal_power / noise_power);
        printf("frame %zu symbol %zu ant %zu: corr offset %zu, SNR %2.1f \n",
            frame_id, symbol_id, ant_id, sym_offset, SNR);
        if (SNR > 15 && sym_offset >= 230 && sym_offset <= 250) {
            record_frame = frame_id;
        }
        if (frame_id == record_frame) {
            std::string fname = "rxpilot" + std::to_string(symbol_id) + ".bin";
            FILE* f = fopen(fname.c_str(), "wb");
            fwrite(pkt->data, 2 * sizeof(int16_t), config_->sampsPerSymbol, f);
            fclose(f);
        }

    } else {
        if (frame_id == record_frame) {
            std::string fname = "rxdata" + std::to_string(symbol_id) + ".bin";
            FILE* f = fopen(fname.c_str(), "wb");
            fwrite(pkt->data, 2 * sizeof(int16_t), config_->sampsPerSymbol, f);
            fclose(f);
            //record_frame = -1;
        }
    }
#endif

    // remove CP, do FFT
    size_t dl_symbol_id = config_->get_dl_symbol_idx(frame_id, symbol_id);
    size_t total_dl_symbol_id = frame_slot * dl_symbol_perframe + dl_symbol_id;
    size_t FFT_buffer_target_id = total_dl_symbol_id * antenna_num + ant_id;

    // transfer ushort to float
    size_t delay_offset = (dl_prefix_len + CP_LEN) * 2;
    float* cur_fft_buffer_float = (float*)fft_buffer_[FFT_buffer_target_id];

    for (size_t i = 0; i < (FFT_LEN)*2; i++)
        cur_fft_buffer_float[i] = pkt->data[delay_offset + i] / 32768.2f;

    // perform fft
    DftiComputeForward(mkl_handle, fft_buffer_[FFT_buffer_target_id]);

    if (kDebugPrintInTask) {
        printf("In doCrop thread %zu: frame: %zu, symbol: %zu, ant: %zu\n",
            rx_thread_id, frame_id % TASK_BUFFER_FRAME_NUM, symbol_id, ant_id);
    }
    size_t csi_offset = frame_slot * antenna_num + ant_id;
    cx_float* csi_buffer_ptr = (cx_float*)(csi_buffer_[csi_offset].data());
    cx_float* fft_buffer_ptr = (cx_float*)fft_buffer_[FFT_buffer_target_id];

    Event_data crop_finish_event;

    // If it is pilot part, do CE
    if (dl_symbol_id < config_->DL_PILOT_SYMS) {

        cx_float avg_csi(0, 0);
        for (size_t j = 0; j < non_null_sc_len; j++) {
            // divide fft output by pilot data to get CSI estimation
            if (dl_symbol_id == 0) {
                csi_buffer_ptr[j] = 0;
            }
            complex_float p = config_->ue_specific_pilot[ant_id][j];
            size_t sc_id = non_null_sc_ind_[j];
            csi_buffer_ptr[j] += (fft_buffer_ptr[sc_id] / cx_float(p.re, p.im));
            if (dl_symbol_id == dl_pilot_symbol_perframe - 1)
                csi_buffer_ptr[j] /= dl_pilot_symbol_perframe;
        }

        crop_finish_event = Event_data(EventType::kFFT,
            gen_tag_t::frm_sym_ant(frame_id, symbol_id, ant_id)._tag);
    } else {
        size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe
            + dl_symbol_id - dl_pilot_symbol_perframe;
        size_t eq_buffer_offset = total_dl_symbol_id * antenna_num + ant_id;

        cx_float* equ_buffer_ptr
            = (cx_float*)(equal_buffer_[eq_buffer_offset].data());
        cx_float csi(1, 0);
        cx_float* dl_iq_f_ptr = (cx_float*)&config_->dl_iq_f[dl_symbol_id
            - dl_pilot_symbol_perframe][ant_id * FFT_LEN];
        float evm = 0;

        float theta = 0;
        for (size_t j = 0; j < non_null_sc_len; j++) {
            if (j % config_->OFDM_PILOT_SPACING == 0) {
                equ_buffer_ptr[j] = 0;
                if (dl_pilot_symbol_perframe > 0) {
                    csi = csi_buffer_ptr[j];
                }
                size_t sc_id = non_null_sc_ind_[j];
                cx_float y = fft_buffer_ptr[sc_id];
                auto pilot_eq = y / csi;
                auto p = config_->ue_specific_pilot[ant_id][j];
                theta += arg(pilot_eq * cx_float(p.re, -p.im));
            }
        }
        theta /= config_->OFDM_PILOT_NUM;
        auto phc = exp(cx_float(0, -theta));
        for (size_t j = 0; j < non_null_sc_len; j++) {
            if (j % config_->OFDM_PILOT_SPACING != 0) {
                // divide fft output by pilot data to get CSI estimation
                size_t sc_id = non_null_sc_ind_[j];
                if (dl_pilot_symbol_perframe > 0) {
                    csi = csi_buffer_ptr[j];
                }
                cx_float y = fft_buffer_ptr[sc_id];
                equ_buffer_ptr[j] = (y / csi) * phc;
                evm += std::norm(equ_buffer_ptr[j] - dl_iq_f_ptr[sc_id]);
            }
        }
        evm = std::sqrt(
            evm / (config_->OFDM_DATA_NUM - config_->OFDM_PILOT_NUM));
        if (kPrintPhyStats)
            std::cout << "Frame: " << frame_id << " EVM: " << 100 * evm
                      << "%, SNR: " << -10 * std::log10(evm) << std::endl;

        crop_finish_event = Event_data(EventType::kDemul,
            gen_tag_t::frm_sym_ant(frame_id, symbol_id, ant_id)._tag);
    }

    rx_buffer_status_[rx_thread_id][offset_in_current_buffer] = 0; // now empty
    if (!message_queue_.enqueue(*task_ptok[tid], crop_finish_event)) {
        printf("crop message enqueue failed\n");
        exit(0);
    }
}

void Phy_UE::doDemul(int tid, size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t symbol_id = gen_tag_t(tag).symbol_id;
    const size_t ant_id = gen_tag_t(tag).ant_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    size_t dl_symbol_id = config_->get_dl_symbol_idx(frame_id, symbol_id);
    size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe
        + dl_symbol_id - dl_pilot_symbol_perframe;
    size_t offset = total_dl_symbol_id * antenna_num + ant_id;
    cx_float* tar_ptr = (cx_float*)&equal_buffer_[offset][0];
    uint8_t* demul_ptr = (uint8_t*)(&dl_data_buffer_[offset][0]);

    demod_16qam_hard_loop((float*)tar_ptr, (uint8_t*)demul_ptr, antenna_num);

    // Inform main thread
    Event_data demul_finish_event(EventType::kDemul, tag);

    if (!message_queue_.enqueue(*task_ptok[tid], demul_finish_event)) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }
}

//////////////////////////////////////////////////////////
//                   UPLINK Operations                //
//////////////////////////////////////////////////////////

void Phy_UE::doModul(int tid, size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t ue_id = gen_tag_t(tag).ant_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    for (size_t ch = 0; ch < config_->nChannels; ch++) {
        size_t ant_id = ue_id * config_->nChannels + ch;
        for (size_t ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe;
             ul_symbol_id++) {
            size_t total_ul_symbol_id
                = frame_slot * ul_data_symbol_perframe + ul_symbol_id;
            complex_float* modul_buf
                = &modul_buffer_[total_ul_symbol_id][ant_id * data_sc_len];
            int8_t* ul_bits
                = &config_->ul_bits[ul_symbol_id][ant_id * data_sc_len];
            for (size_t sc = 0; sc < data_sc_len; sc++) {
                modul_buf[sc]
                    = mod_single_uint8((uint8_t)ul_bits[sc], qam_table);
            }
        }
    }
    Event_data modul_event(EventType::kModul, tag);

    if (!message_queue_.enqueue(*task_ptok[tid], modul_event)) {
        printf("Muliplexing message enqueue failed\n");
        exit(0);
    }
}

void Phy_UE::doIFFT(int tid, size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    const size_t ue_id = gen_tag_t(tag).ant_id;
    for (size_t ch = 0; ch < config_->nChannels; ch++) {
        //float scale = 0;
        size_t ant_id = ue_id * config_->nChannels + ch;
        for (size_t ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe;
             ul_symbol_id++) {
            size_t total_ul_data_symbol_id
                = frame_slot * ul_data_symbol_perframe + ul_symbol_id;

            size_t buff_offset = total_ul_data_symbol_id * antenna_num + ant_id;
            complex_float* ifft_buff = ifft_buffer_[buff_offset];
            memset(
                ifft_buff, 0, sizeof(complex_float) * config_->OFDM_DATA_START);
            /*memcpy((void*)ifft_buf,
                (void*)&ul_iq_f[ul_symbol_id][FFT_LEN * (ant_id)],
                FFT_LEN * sizeof(complex_float));*/
            complex_float* modul_buff
                = &modul_buffer_[total_ul_data_symbol_id][ant_id * data_sc_len];
            memcpy((void*)ifft_buff, (void*)modul_buff,
                FFT_LEN * sizeof(complex_float));
            memset(ifft_buff + config_->OFDM_DATA_STOP, 0,
                sizeof(complex_float) * config_->OFDM_DATA_START);

            //DftiComputeBackward(mkl_handle, ifft_buf);
            CommsLib::IFFT(ifft_buff, FFT_LEN, false);
            //cx_float* ifft_out_buffer = (cx_float*)ifft_buf;
            //cx_fmat mat_ifft_out(ifft_out_buffer, FFT_LEN, 1, false);
            //float max_val = abs(mat_ifft_out).max();
            //if (max_val > scale)
            //    scale = max_val;
        }
        //scale *= 4;
        for (size_t ul_symbol_id = 0; ul_symbol_id < ul_symbol_perframe;
             ul_symbol_id++) {
            size_t total_ul_symbol_id
                = frame_slot * ul_symbol_perframe + ul_symbol_id;

            size_t buff_offset = total_ul_symbol_id * antenna_num + ant_id;
            size_t tx_offset = buff_offset * packet_length;
            char* cur_tx_buffer = &tx_buffer_[tx_offset];
            struct Packet* pkt = (struct Packet*)cur_tx_buffer;
            std::complex<short>* tx_data_ptr = (std::complex<short>*)pkt->data;
            if (ul_symbol_id < config_->UL_PILOT_SYMS) {
                memcpy(tx_data_ptr, config_->ue_specific_pilot_t[ant_id],
                    config_->sampsPerSymbol * sizeof(std::complex<short>));
            } else {
                size_t total_ul_data_symbol_id
                    = frame_slot * ul_data_symbol_perframe + ul_symbol_id
                    - config_->UL_PILOT_SYMS;

                size_t buff_offset
                    = total_ul_data_symbol_id * antenna_num + ant_id;
                complex_float* ifft_buff = ifft_buffer_[buff_offset];
                CommsLib::ifft2tx(ifft_buff, tx_data_ptr, FFT_LEN, prefix_len,
                    CP_LEN, config_->scale);
            }
        }
    }

    Event_data ifft_event(EventType::kIFFT, tag);

    if (!message_queue_.enqueue(*task_ptok[tid], ifft_event)) {
        printf("Muliplexing message enqueue failed\n");
        exit(0);
    }
}

void Phy_UE::initialize_vars_from_cfg(void)
{
    packet_length = config_->packet_length;

    symbol_perframe = config_->symbol_num_perframe;
    dl_pilot_symbol_perframe = config_->DL_PILOT_SYMS;
    ul_pilot_symbol_perframe = config_->UL_PILOT_SYMS;
    ul_symbol_perframe = config_->ul_data_symbol_num_perframe;
    dl_symbol_perframe = config_->dl_data_symbol_num_perframe;
    dl_data_symbol_perframe = dl_symbol_perframe - dl_pilot_symbol_perframe;
    ul_data_symbol_perframe = ul_symbol_perframe - ul_pilot_symbol_perframe;
    prefix_len = config_->prefix;
    dl_prefix_len = config_->dl_prefix;
    postfix_len = config_->postfix;
    symbol_len = config_->sampsPerSymbol - prefix_len - postfix_len;
    CP_LEN = config_->CP_LEN;
    FFT_LEN = config_->OFDM_CA_NUM;
    ofdm_syms = (int)(symbol_len / (FFT_LEN + CP_LEN));
    data_sc_len = config_->OFDM_DATA_NUM;
    data_sc_start = config_->OFDM_DATA_START;
    nUEs = config_->UE_NUM;
    antenna_num = config_->UE_ANT_NUM;
    nCPUs = std::thread::hardware_concurrency();
    rx_thread_num = std::min(nUEs, config_->socket_thread_num);
    worker_thread_num = config_->worker_thread_num;
    core_offset = config_->core_offset;
#ifdef ENABLE_CPU_ATTACH
    size_t max_core = 1 + rx_thread_num + worker_thread_num + core_offset;
    if (max_core >= nCPUs) {
        printf("Cannot allocate cores: max_core %zu, available cores %zu\n",
            max_core, nCPUs);
        exit(1);
    }
#endif
    printf("ofdm_syms %zu, %zu symbols, %zu pilot symbols, %zu UL data "
           "symbols, %zu DL data symbols\n",
        ofdm_syms, symbol_perframe, ul_pilot_symbol_perframe,
        ul_data_symbol_perframe, dl_data_symbol_perframe);

    tx_buffer_status_size
        = (ul_symbol_perframe * antenna_num * TASK_BUFFER_FRAME_NUM);
    tx_buffer_size = packet_length * tx_buffer_status_size;
    rx_buffer_status_size
        = (dl_symbol_perframe * antenna_num * TASK_BUFFER_FRAME_NUM);
    rx_buffer_size = packet_length * rx_buffer_status_size;
}

void Phy_UE::getDemulData(long long** ptr, int* size)
{
    *ptr = (long long*)&equal_buffer_[max_equaled_frame
        * dl_data_symbol_perframe][0];
    *size = antenna_num * FFT_LEN;
}

void Phy_UE::getEqualData(float** ptr, int* size, int ue_id)
{
    *ptr = (float*)&equal_buffer_[max_equaled_frame * dl_data_symbol_perframe
            * antenna_num
        + ue_id][0];
    *size = antenna_num * non_null_sc_len * 2;
}

extern "C" {
EXPORT Phy_UE* Phy_UE_new(Config* cfg)
{
    auto* usr = new Phy_UE(cfg);
    return usr;
}
EXPORT void Phy_UE_start(Phy_UE* usr) { usr->start(); }
EXPORT void Phy_UE_stop(/*Phy_UE *usr*/)
{
    SignalHandler::setExitSignal(true); /*usr->stop();*/
}
EXPORT void Phy_UE_destroy(Phy_UE* usr) { delete usr; }
EXPORT void Phy_UE_getEqualData(Phy_UE* usr, float** ptr, int* size, int ue)
{
    return usr->getEqualData(ptr, size, ue);
}
EXPORT void Phy_UE_getDemulData(Phy_UE* usr, long long** ptr, int* size)
{
    return usr->getDemulData(ptr, size);
}
}
