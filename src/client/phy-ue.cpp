#include "phy-ue.hpp"

using namespace arma;
typedef cx_float COMPLEX;

Phy_UE::Phy_UE(Config* config)
    : ul_IQ_data(config->ul_IQ_data)
    , ul_IQ_modul(config->ul_IQ_modul)
{
    srand(time(NULL));

    this->config_ = config;
    initialize_vars_from_cfg();

    pilot_sc_ind_ = CommsLib::getPilotScInd(FFT_LEN);
    pilot_sc_val_ = CommsLib::getPilotSc(FFT_LEN);
    if (FFT_LEN == 64)
        data_sc_ind_ = CommsLib::getDataSc(FFT_LEN);
    else
        for (int i = data_sc_start; i < data_sc_start + data_sc_len; i++)
            data_sc_ind_.push_back(i);

    pilot_sc_len = pilot_sc_ind_.size();
    non_null_sc_len = data_sc_len + pilot_sc_len;
    non_null_sc_ind_.insert(
        non_null_sc_ind_.end(), data_sc_ind_.begin(), data_sc_ind_.end());
    non_null_sc_ind_.insert(
        non_null_sc_ind_.end(), pilot_sc_ind_.begin(), pilot_sc_ind_.end());
    std::sort(non_null_sc_ind_.begin(), non_null_sc_ind_.end());

    task_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * rx_symbol_perframe * numAntennas * 36);
    demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * dl_data_symbol_perframe * numAntennas * 36);
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * symbol_perframe * numAntennas * 36);
    fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * ul_data_symbol_perframe * numAntennas * 36);
    tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * ul_data_symbol_perframe * numAntennas * 36);

    printf("initializing buffers...\n");

    //////////////////////////////////////
    //////// uplink buffers init (tx) ////
    //////////////////////////////////////

    // initialize ul data buffer
    // l2_buffer_status_.resize(TASK_BUFFER_FRAME_NUM *
    // ul_data_symbol_perframe);

    // initialize modulation buffer
    // modul_buffer_.resize(ul_data_symbol_perframe * TASK_BUFFER_FRAME_NUM);
    // for (int i = 0; i < modul_buffer_.size(); i++)
    //    modul_buffer_[i].resize(data_sc_len * nUEs);

    // initialize IFFT buffer
    size_t IFFT_buffer_block_num
        = numAntennas * ul_data_symbol_perframe * TASK_BUFFER_FRAME_NUM;
    ifft_buffer_.IFFT_inputs.calloc(IFFT_buffer_block_num, FFT_LEN, 64);

    alloc_buffer_1d(&tx_buffer_, tx_buffer_size, 64, 0);
    alloc_buffer_1d(&tx_buffer_status_, tx_buffer_status_size, 64, 1);
#ifndef USE_ARGOS
    // read pilot
    int pilot_len = (FFT_LEN + CP_LEN);
    // ul_pilot_aligned = new char[packet_length];
    // memcpy((void*)&ul_pilot_aligned[prefix_len * sizeof(uint32_t) +
    // packet_header_offset * sizeof(int)], pilot_ci16.data(), pilot_len *
    // sizeof(uint32_t));
#endif

    ////////////////////////////////////////
    //////// downlink buffers init (rx) ////
    ////////////////////////////////////////

    // initialize rx buffer
    rx_buffer_.malloc(rx_thread_num, rx_buffer_size, 64);
    rx_buffer_status_.calloc(rx_thread_num, rx_buffer_status_size, 64);

    // initialize FFT buffer
    size_t FFT_buffer_block_num
        = numAntennas * dl_symbol_perframe * TASK_BUFFER_FRAME_NUM;
    fft_buffer_.FFT_inputs.calloc(FFT_buffer_block_num, FFT_LEN, 64);

    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, FFT_LEN);
    (void)DftiCommitDescriptor(mkl_handle);

    // initialize CSI buffer
    csi_buffer_.resize(numAntennas * TASK_BUFFER_FRAME_NUM);
    for (size_t i = 0; i < csi_buffer_.size(); i++)
        csi_buffer_[i].resize(non_null_sc_len);

    if (dl_data_symbol_perframe > 0) {
        // initialize equalized data buffer
        equal_buffer_.resize(
            numAntennas * dl_data_symbol_perframe * TASK_BUFFER_FRAME_NUM);
        for (size_t i = 0; i < equal_buffer_.size(); i++)
            equal_buffer_[i].resize(non_null_sc_len);

        // initialize equalized data buffer
        equal_pc_buffer_.resize(
            numAntennas * dl_data_symbol_perframe * TASK_BUFFER_FRAME_NUM);
        for (size_t i = 0; i < equal_pc_buffer_.size(); i++)
            equal_pc_buffer_[i].resize(data_sc_len);

        // initialize data buffer
        dl_data_buffer_.resize(
            numAntennas * dl_data_symbol_perframe * TASK_BUFFER_FRAME_NUM);
        for (size_t i = 0; i < dl_data_buffer_.size(); i++)
            dl_data_buffer_[i].resize(data_sc_len);
    }

    ru_.reset(new RU(
        rx_thread_num, tx_thread_num, config_, &message_queue_, &tx_queue_));

    // initilize all kinds of checkers
    cropper_checker_ = new size_t[dl_symbol_perframe * TASK_BUFFER_FRAME_NUM];
    memset(cropper_checker_, 0,
        sizeof(int) * dl_symbol_perframe * TASK_BUFFER_FRAME_NUM);
    memset(csi_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(data_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);

    memset(demul_status_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(cropper_created_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
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
    fft_buffer_.FFT_inputs.free();
    ifft_buffer_.IFFT_inputs.free();
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
    // if ENABLE_CPU_ATTACH, attach main thread to core 0
    int main_core_id = core_offset;
#ifdef ENABLE_CPU_ATTACH
    if (pin_to_core(main_core_id) != 0) {
        printf(
            "Main thread: stitch main thread to core %d failed", main_core_id);
        exit(0);
    } else {
        printf("Main thread: stitch main thread to core %d succeeded\n",
            main_core_id);
    }
#endif
    // start radios
    ru_->startRadios();

    std::vector<pthread_t> rx_threads
        = ru_->startTXRX(rx_buffer_, rx_buffer_status_, rx_buffer_status_size,
            rx_buffer_size, core_offset + 1);

    std::vector<pthread_t> tx_threads = ru_->startTX(tx_buffer_,
        ul_pilot_aligned, tx_buffer_status_, tx_buffer_status_size,
        tx_buffer_size, core_offset + 1 + rx_thread_num);

    // for task_queue, main thread is producer, it is single-procuder & multiple
    // consumer for task queue uplink

    // TODO: make the producertokens global and try
    // "try_dequeue_from_producer(token,item)"
    //       combine the task queues into one queue
    moodycamel::ProducerToken ptok(task_queue_);
    moodycamel::ProducerToken ptok_demul(demul_queue_);
    moodycamel::ProducerToken ptok_fft(fft_queue_);
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
    int l2_offset = 0;
    max_equaled_frame = 0;
    int frame_id, symbol_id, dl_symbol_id;
    // int ant_id, total_symbol_id;
    size_t frame_id_t, symbol_id_t, dl_symbol_id_t, total_symbol_id_t, ant_id_t;
    int prev_frame_id = config_->maxFrame;
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
                int offset = event.data;

                int buffer_frame_num = rx_buffer_status_size;
                int rx_thread_id = offset / buffer_frame_num;
                int offset_in_current_buffer = offset % buffer_frame_num;
                struct Packet* pkt = (struct Packet*)(rx_buffer_[rx_thread_id]
                    + offset_in_current_buffer * packet_length);
                frame_id = pkt->frame_id;
                symbol_id = pkt->symbol_id;
#if WRITE_RECV
                if (frame_id < 10
                    && config_->getDlSFIndex(frame_id, symbol_id) == 0) {
                    int ant_id = pkt->ant_id;
                    int len = config_->sampsPerSymbol;
                    void* cur_buf = pkt->data;
                    std::string filename = "sig_ant" + std::to_string(ant_id)
                        + "_f" + std::to_string(frame_id) + ".bin";
                    fp = fopen(filename.c_str(), "wb");
                    fwrite(cur_buf, sizeof(std::complex<short>),
                        config_->sampsPerSymbol, fp);
                    fclose(fp);
                }
#endif

                // if (ul_data_symbol_perframe > 0 && prev_frame_id ==
                // config_->maxFrame)
                //    printf("received start indication frame with frame_id
                //    %zu\n", frame_id);
                // check if downlink is enabled, and a new frame has
                // started. if yes, schedule l2 traffic
                if (ul_data_symbol_perframe > 0 && frame_id != prev_frame_id) {
                    // schedule L2 downlink traffic for
                    // frame=frame_id+TX_RX_FRAME_OFFSET
                    int tx_frame_id = (frame_id + TX_RX_FRAME_OFFSET);
                    for (size_t i = 0; i < ul_data_symbol_perframe; i++) {
                        l2_offset = generateOffset2d(
                            tx_frame_id % TASK_BUFFER_FRAME_NUM, i);
                        // if (l2_buffer_status_[l2_offset] == 0)
                        {
                            // modul_buffer_[l2_offset] = ul_IQ_modul[i];
                            // l2_buffer_status_[l2_offset] = 1;
                            // printf("At frame %d (prev is %d) scheduling tx
                            // for frame %d with l2_offset %d\n", frame_id,
                            // prev_frame_id, tx_frame_id, l2_offset);
                            Event_data do_modul_task;
                            do_modul_task.event_type = EventType::kIFFT;
                            do_modul_task.data = l2_offset;
                            schedule_task(do_modul_task, &fft_queue_, ptok_fft);
                        }
                    }
                    prev_frame_id = frame_id;
                }
                // demul_begin = std::chrono::system_clock::now();

                if (dl_data_symbol_perframe > 0
                    && (config_->isPilot(frame_id, symbol_id)
                           || config_->isDownlink(frame_id, symbol_id))) {
                    Event_data do_crop_task;
                    do_crop_task.event_type = EventType::kFFT;
                    do_crop_task.data = offset;
                    schedule_task(do_crop_task, &task_queue_, ptok);
#if DEBUG_PRINT_ENTER_QUEUE_FFT

                    int cropper_created_checker_id
                        = (frame_id % TASK_BUFFER_FRAME_NUM);
                    cropper_created_checker_[cropper_created_checker_id]++;
#endif
                } else { // if we are not entering doFFT, let's reset buffer
                         // here
                    rx_buffer_status_[rx_thread_id][offset_in_current_buffer]
                        = 0; // now empty
                }
            } break;

            case EventType::kFFT: {
                int offset_csi = event.data;
                interpretOffset2d(
                    numAntennas, offset_csi, &frame_id_t, &ant_id_t);
                frame_id = frame_id_t;
                // ant_id = ant_id_t;
                // checker to count # of pilots/users
                csi_checker_[frame_id]++;

                if (csi_checker_[frame_id] == DL_PILOT_SYMS * numAntennas) {
#if WRITE_CSI_MATRIX
                    int csi_offset = generateOffset2d(
                        TASK_BUFFER_FRAME_NUM, numAntennas, frame_id, 0);
                    fp = fopen("bin/csi_dl.bin", "wb");
                    for (int i = 0; i < numAntennas; i++) {
                        fwrite(csi_buffer_[csi_offset].data(),
                            sizeof(complex_float), non_null_sc_len, fp);
                        csi_offset++;
                    }
                    fclose(fp);
#endif
                    csi_checker_[frame_id] = 0;
#if DEBUG_PRINT_SUMMARY
                    printf("Main thread: pilot frame: %d, finished collecting "
                           "pilot frames\n",
                        frame_id);
#endif
                }

            } break;

            case EventType::kZF: {
                int offset_eq = event.data;
                interpretOffset3d(dl_data_symbol_perframe, numAntennas,
                    offset_eq, &frame_id_t, &total_symbol_id_t, &dl_symbol_id_t,
                    &ant_id_t);
                frame_id = frame_id_t;
                // total_symbol_id = total_symbol_id_t;
                dl_symbol_id = dl_symbol_id_t;
                // ant_id = ant_id_t;

                Event_data do_demul_task(EventType::kDemul, offset_eq);

#if DEBUG_PRINT_ENTER_QUEUE_DEMUL
                printf("Main thread: created Demodulation task for frame: %d, "
                       "symbol: %d\n",
                    frame_id, symbol_id);
#endif
                data_checker_[frame_id]++;

                // schedule demodulation task for subcarrier blocks
                schedule_task(do_demul_task, &demul_queue_, ptok_demul);

                if (data_checker_[frame_id]
                    == dl_data_symbol_perframe * numAntennas) {
#if WRITE_FFT_DATA
                    int FFT_buffer_offset
                        = generateOffset3d(TASK_BUFFER_FRAME_NUM,
                            rx_symbol_perframe, numAntennas, frame_id, 0, 0);
                    fp = fopen("bin/fft_dl.bin", "wb");
                    for (int i = 0; i < dl_symbol_perframe * numAntennas; i++) {
                        fwrite(
                            (void*)fft_buffer_.FFT_outputs[FFT_buffer_offset],
                            sizeof(complex_float), FFT_LEN, fp);
                        FFT_buffer_offset++;
                    }
                    fclose(fp);
#endif
#if WRITE_DEMUL
                    int eq_buffer_offset = generateOffset3d(
                        TASK_BUFFER_FRAME_NUM, dl_data_symbol_perframe,
                        numAntennas, frame_id, 0, 0);
                    fp = fopen("bin/equal_dl.bin", "wb");
                    for (int i = 0; i < numAntennas * dl_data_symbol_perframe;
                         i++) {
                        fwrite(equal_buffer_[eq_buffer_offset].data(),
                            sizeof(complex_float), non_null_sc_len, fp);
                        eq_buffer_offset++;
                    }
                    fclose(fp);
#endif
#if DEBUG_PRINT_SUMMARY
                    printf("Main thread: frame: %d, finished collecting data "
                           "frames\n",
                        frame_id);
#endif
                    data_checker_[frame_id] = 0;
                }
            } break;

            case EventType::kDemul: {
                // do nothing
                int offset_demul = event.data;
                interpretOffset3d(dl_data_symbol_perframe, numAntennas,
                    offset_demul, &frame_id_t, &total_symbol_id_t,
                    &dl_symbol_id_t, &ant_id_t);
                frame_id = frame_id_t;
                // total_symbol_id = total_symbol_id_t;
                dl_symbol_id = dl_symbol_id_t;
                // ant_id = ant_id_t;

                demul_checker_[frame_id][dl_symbol_id]++;
                // if this subframe is ready
                if (demul_checker_[frame_id][dl_symbol_id] == numAntennas) {

#if DEBUG_PRINT_TASK_DONE
                    printf(
                        "Main thread: Demodulation done frame: %d, symbol %d\n",
                        frame_id, dl_symbol_id);
#endif
                    max_equaled_frame = frame_id;
                    demul_checker_[frame_id][dl_symbol_id] = 0;
                    demul_status_[frame_id]++;
                    if (demul_status_[frame_id] == dl_data_symbol_perframe) {
#if DEBUG_PRINT_SUMMARY
                        printf("Main thread: Demodulation done frame: %d \n",
                            frame_id);
#endif
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
                            task_queue_.size_approx());
                        demul_begin = std::chrono::system_clock::now();
                    }
                }
            } break;

            case EventType::kIFFT: {
                // Do nothing for now
#if WRITE_MODUL
                std::string filename = "bin/ul_modul.bin";
                FILE* fp = fopen(filename.c_str(), "wb");
                // fwrite(modul_buffer_[offset].data(), sizeof(complex_float),
                // data_sc_len * nUEs * ul_data_symbol_perframe, fp);
                fclose(fp);
#endif

#if WRITE_IFFT
                std::string filename = "bin/ul_ifft.bin";
                FILE* fp = fopen(filename.c_str(), "wb");
                for (int ii = 0; ii < ul_data_symbol_perframe; ii++) {
                    int IFFT_buffer_target_id
                        = offset * (numAntennas * ul_data_symbol_perframe)
                        + ii * numAntennas;
                    for (int jj = 0; jj < numAntennas; jj++) {
                        fwrite(ifft_buffer_
                                   .IFFT_inputs[IFFT_buffer_target_id + jj],
                            sizeof(complex_float), FFT_LEN, fp);
                    }
                }
                fclose(fp);
#endif

#if WRITE_TX_BUFFER
                std::string filename = "bin/ul_tx_buf.bin";
                FILE* fp = fopen(filename.c_str(), "wb");
                int frame_samp_size = (config_->getTxPackageLength()
                    * numAntennas * ul_data_symbol_perframe);
                fwrite(&tx_buffer_[offset * frame_samp_size],
                    sizeof(complex_float),
                    (CP_LEN + FFT_LEN + offsetof(Packet, data)) * numAntennas
                        * dl_data_symbol_perframe,
                    fp);
                // fwrite(tx_buffer_.buffer[offset].data(),
                // sizeof(std::complex<short>), (CP_LEN+FFT_LEN+4) * numAntennas
                // * dl_data_symbol_perframe, fp);
                fclose(fp);
#endif
            } break;

            case EventType::kPacketTX: {
                int offset = event.data;
                interpretOffset3d(config_->ul_data_symbol_num_perframe,
                    config_->getNumAntennas(), offset, &frame_id_t,
                    &total_symbol_id_t, &symbol_id_t, &ant_id_t);
#if DEBUG_PRINT_PER_SUBFRAME_DONE
                printf("Main thread: finished TX for frame %d, symbol %d, ant "
                       "%d\n",
                    frame_id_t, symbol_id_t, ant_id_t);
#endif
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
    size_t offset_id = core_offset + rx_thread_num + tx_thread_num + 1;
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
            doDemul(tid, event.data);
        else if (fft_queue_.try_dequeue(event))
            doTransmit(tid, event.data, 0); //, event.more_data);
        else if (task_queue_.try_dequeue(event))
            doFFT(tid, event.data);
    }
}

//////////////////////////////////////////////////////////
//                   DOWNLINK Operations                  //
//////////////////////////////////////////////////////////

void Phy_UE::doFFT(int tid, int offset)
{
    int buffer_frame_num = rx_buffer_status_size;
    int rx_thread_id = offset / buffer_frame_num;
    int offset_in_current_buffer = offset % buffer_frame_num;

    // read info of one frame
    struct Packet* pkt = (struct Packet*)(rx_buffer_[rx_thread_id]
        + offset_in_current_buffer * packet_length);
    int frame_id = pkt->frame_id;
    int symbol_id = pkt->symbol_id;
    // int cell_id = pkt->cell_id;
    int ant_id = pkt->ant_id;

    if (!config_->isPilot(frame_id, symbol_id)
        && !(config_->isDownlink(frame_id, symbol_id)))
        return;

    // remove CP, do FFT
    int dl_symbol_id = config_->getDlSFIndex(frame_id, symbol_id);
    int FFT_buffer_target_id = generateOffset3d(TASK_BUFFER_FRAME_NUM,
        rx_symbol_perframe, numAntennas, frame_id, dl_symbol_id, ant_id);

    size_t sym_offset = 0;
    if (config_->isPilot(frame_id, symbol_id)) {
        if (frame_id == 3 * TX_FRAME_DELTA) {
            std::string fname = "rxpilot" + std::to_string(symbol_id) + ".bin";
            FILE* f = fopen(fname.c_str(), "wb");
            fwrite(pkt->data, 2 * sizeof(int16_t), config_->sampsPerSymbol, f);
            fclose(f);
        }

#if DEBUG_DL_PILOT
        std::vector<std::complex<double>> vec;
        for (size_t i = 0; i < config_->sampsPerSymbol; i++)
            vec.push_back(std::complex<double>(
                pkt->data[2 * i] / 32768.0, pkt->data[2 * i + 1] / 32768.0));
        sym_offset = CommsLib::find_pilot_seq(
            vec, config_->pilot_cd64, config_->pilot_cd64.size());
        sym_offset = sym_offset < config_->pilot_cd64.size()
            ? 0
            : sym_offset - config_->pilot_cd64.size();
        double noise_power = 0;
        for (size_t i = 0; i < sym_offset; i++)
            noise_power += std::pow(std::abs(vec[i]), 2);
        double signal_power = 0;
        for (size_t i = sym_offset; i < 2 * sym_offset; i++)
            signal_power += std::pow(std::abs(vec[i]), 2);
        double SNR = 10 * std::log10(signal_power / noise_power);
        printf("frame %d symbol %d ant %d: corr offset %zu, SNR %2.1f \n",
            frame_id, symbol_id, ant_id, sym_offset, SNR);
#endif
    } else {
        if (frame_id == 3 * TX_FRAME_DELTA) {
            std::string fname = "rxdata" + std::to_string(symbol_id) + ".bin";
            FILE* f = fopen(fname.c_str(), "wb");
            fwrite(pkt->data, 2 * sizeof(int16_t), config_->sampsPerSymbol, f);
            fclose(f);
        }
    }

    // transfer ushort to float
    size_t delay_offset = (dl_prefix_len + CP_LEN)
        * 2; // GetFrameStart(pkt->data, prefix_len, postfix_len);
    // float *cur_radio_buffer = (float *)(cur_ptr_buffer + sizeof(int) *
    // packet_header_offset);
    float* cur_fft_buffer_float
        = (float*)fft_buffer_.FFT_inputs[FFT_buffer_target_id];

    for (size_t i = 0; i < (FFT_LEN)*2; i++)
        cur_fft_buffer_float[i] = pkt->data[delay_offset + i] / 32768.2f;
    // memcpy((void *)cur_fft_buffer_float, (void *)(cur_radio_buffer +
    // delay_offset), FFT_LEN * 2 * sizeof(float));

    // perform fft
    DftiComputeForward(
        mkl_handle, fft_buffer_.FFT_inputs[FFT_buffer_target_id]);

#if DEBUG_PRINT_IN_TASK
    printf("In doCrop thread %d: frame: %d, symbol: %d, ant: %d\n", tid,
        frame_id % TASK_BUFFER_FRAME_NUM, symbol_id, ant_id);
#endif
    int csi_offset = generateOffset2d(TASK_BUFFER_FRAME_NUM, numAntennas,
        frame_id,
        ant_id); //(frame_id % TASK_BUFFER_FRAME_NUM) * numAntennas + ant_id;
    float* csi_buffer_ptr = (float*)(csi_buffer_[csi_offset].data());
    float* fft_buffer_ptr
        = (float*)fft_buffer_.FFT_inputs[FFT_buffer_target_id];

    Event_data crop_finish_event;

    // If it is pilot part, do CE
    if (config_->isPilot(frame_id, symbol_id)) {

        int csi_fftshift_offset = 0;
        int pilot_id = config_->getDownlinkPilotId(frame_id, symbol_id);
        for (int j = 0; j < non_null_sc_len; j++) {
            // if (j < FFT_LEN / 2)
            //    csi_fftshift_offset = FFT_LEN/2;
            // else
            //    csi_fftshift_offset = -FFT_LEN/2;
            // divide fft output by pilot data to get CSI estimation
            int i = non_null_sc_ind_[j];
            if (pilot_id == 0) {
                *(csi_buffer_ptr + 2 * j) = 0;
                *(csi_buffer_ptr + 2 * j + 1) = 0;
            }
            // printf("%.4f+j%.4f  ",cur_fft_buffer_float_output[2*j],
            // cur_fft_buffer_float_output[2*j+1]);
            // TODO: here it is assumed pilots_ is real-valued (in LTS case it
            // is), whereas it could be complex
            *(csi_buffer_ptr + 2 * j)
                += fft_buffer_ptr[2 * (i + csi_fftshift_offset)]
                * pilots_[i + csi_fftshift_offset];
            *(csi_buffer_ptr + 2 * j + 1)
                += fft_buffer_ptr[2 * (i + csi_fftshift_offset) + 1]
                * pilots_[i + csi_fftshift_offset];
            if (pilot_id == DL_PILOT_SYMS - 1) {
                *(csi_buffer_ptr + 2 * j)
                    /= (DL_PILOT_SYMS > 0 ? DL_PILOT_SYMS : 1);
                *(csi_buffer_ptr + 2 * j + 1)
                    /= (DL_PILOT_SYMS > 0 ? DL_PILOT_SYMS : 1);
            }
        }

        crop_finish_event.event_type = EventType::kFFT;
        crop_finish_event.data = csi_offset;
    } else if (config_->isDownlink(frame_id, symbol_id)) {
        int eq_buffer_offset
            = generateOffset3d(TASK_BUFFER_FRAME_NUM, dl_data_symbol_perframe,
                numAntennas, frame_id, dl_symbol_id - DL_PILOT_SYMS,
                ant_id); //(frame_id % TASK_BUFFER_FRAME_NUM) *
                         // dl_data_symbol_perframe + dl_data_symbol_id;

        float* equ_buffer_ptr
            = (float*)(equal_buffer_[eq_buffer_offset].data());
        // int csi_fftshift_offset = 0;
        float csi_re = 1;
        float csi_im = 0;

        for (int j = 0; j < non_null_sc_len; j++) {
            // if (j < FFT_LEN / 2)
            //    csi_fftshift_offset = FFT_LEN/2;
            // else
            //    csi_fftshift_offset = -FFT_LEN/2;
            // divide fft output by pilot data to get CSI estimation
            int i = non_null_sc_ind_[j];
            if (DL_PILOT_SYMS > 0) {
                csi_re = csi_buffer_ptr[2 * j];
                csi_im = csi_buffer_ptr[2 * j + 1];
            }
            float y_re = fft_buffer_ptr[2 * i];
            float y_im = fft_buffer_ptr[2 * i + 1];
            equ_buffer_ptr[2 * j] = (y_re * csi_re + y_im * csi_im)
                / (csi_re * csi_re
                      + csi_im * csi_im); // fft_buffer_ptr[2*i] / csi_re;
            equ_buffer_ptr[2 * j + 1] = (y_im * csi_re - y_re * csi_im)
                / (csi_re * csi_re
                      + csi_im * csi_im); // fft_buffer_ptr[2*i+1] / csi_im;
        }

        crop_finish_event.event_type = EventType::kZF;
        crop_finish_event.data = eq_buffer_offset;

        // generateOffset3d(numAntennas, dl_symbol_perframe, frame_id,
        // dl_symbol_id, ant_id);
    }

    // after finish
    rx_buffer_status_[rx_thread_id][offset_in_current_buffer] = 0; // now empty
    // printf("In doCrop: emptied socket buffer frame: %d, symbol: %d, ant: %d,
    // offset: %d\n",frame_id, symbol_id, ant_id, offset); inform main thread
    if (!message_queue_.enqueue(*task_ptok[tid], crop_finish_event)) {
        printf("crop message enqueue failed\n");
        exit(0);
    }
}

void Phy_UE::doDemul(int tid, int offset)
{
    cx_float tot_phase_err = 0;

    for (int pilot_id = 0; pilot_id < pilot_sc_len; pilot_id++) {
        cx_float cur_pilot = pilot_sc_val_[pilot_id];
        cx_float* cur_sc = (cx_float*)&equal_buffer_[offset][pilot_id];
        tot_phase_err += (*cur_sc * cur_pilot);
    }

    tot_phase_err = tot_phase_err / (float)pilot_sc_len;
    cx_float* src_ptr = (cx_float*)&equal_buffer_[offset][0];
    cx_float* tar_ptr = (cx_float*)&equal_pc_buffer_[offset][0];
    float phase_err = std::arg(tot_phase_err);

    std::complex<float> phase_err_expo(0, -phase_err);
    for (int sc_id = 0; sc_id < data_sc_len; sc_id++) {
        int non_null_sc_id
            = data_sc_ind_[sc_id] - non_null_sc_ind_[sc_id] + sc_id;
        *(tar_ptr + sc_id) = *(src_ptr + non_null_sc_id) * exp(phase_err_expo);
    }

    uint8_t* demul_ptr = (uint8_t*)(&dl_data_buffer_[offset][0]);
    // cx_fmat mat_equaled_pc(tar_ptr, numAntennas, 1, false);
    // sword* demul_ptr = (sword *)(&dl_data_buffer_[offset][0]);
    // imat mat_demuled(demul_ptr, numAntennas, 1, false);

    // mat_demuled = demod_16qam(mat_equaled_pc);
    demod_16qam_hard_loop((float*)tar_ptr, (uint8_t*)demul_ptr, numAntennas);

    // Inform main thread
    Event_data demul_finish_event(EventType::kDemul, offset);

    if (!message_queue_.enqueue(*task_ptok[tid], demul_finish_event)) {
        printf("Demuliplexing message enqueue failed\n");
        exit(0);
    }
}

//////////////////////////////////////////////////////////
//                   UPLINK Operations                //
//////////////////////////////////////////////////////////

void Phy_UE::doTransmit(int tid, int offset, int frame)
{
    // int buffer_symbol_num = TASK_BUFFER_FRAME_NUM * dl_data_symbol_perframe ;
    // int l2_thread_id = 0; //offset / buffer_symbol_num;
    // offset = offset - l2_thread_id * buffer_symbol_num;
    int frame_offset = 0; // offset / TASK_BUFFER_FRAME_NUM;
    int ul_symbol_id = 0; // offset % TASK_BUFFER_FRAME_NUM;
    interpreteOffset2d(offset, &frame_offset, &ul_symbol_id);

    size_t frame_id = frame;

    size_t frame_samp_size
        = (tx_packet_length * numAntennas * ul_data_symbol_perframe);

    // for (int ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe;
    // ul_symbol_id++)
    //{
#ifndef USE_ARGOS
    int frame_period_id = frame_id % config_->framePeriod;
    int symbol_id = config_->ULSymbols[frame_period_id][ul_symbol_id];
#endif
    // int modulbuf_offset = (data_sc_len * numAntennas * ul_symbol_id);
    size_t txbuf_offset = frame_offset * frame_samp_size
        + (tx_packet_length * numAntennas * ul_symbol_id);

    size_t IFFT_buffer_target_id
        = frame_offset * (numAntennas * ul_data_symbol_perframe)
        + ul_symbol_id * numAntennas;
    for (size_t ant_id = 0; ant_id < numAntennas;
         ant_id++) // TODO consider nChannels=2 case
    {
        // cx_float* modul_ptr = (cx_float *)(&modul_buffer_[offset][ant_id *
        // data_sc_len]); cx_float *tar_out = (cx_float
        // *)ifft_buffer_.IFFT_inputs[IFFT_buffer_target_id+ant_id]; cx_fmat
        // mat_ifft_in(tar_out, FFT_LEN, 1, false); for (int ul_sc_id = 0;
        // ul_sc_id < data_sc_len; ul_sc_id++)
        //{
        //    int sc_id = data_sc_ind_[ul_sc_id];
        //    //printf("frame_id %d, ul_sym_id %d, ue_id %d, ul_sc_id %d, sc_id
        //    %d\n", frame_id, ul_symbol_id, ant_id, ul_sc_id, sc_id);
        //    mat_ifft_in(sc_id, 0) = *(modul_ptr + ul_sc_id);
        //}
        // for (int p_sc_id = 0; p_sc_id < pilot_sc_len; p_sc_id++)
        //{
        //    int sc_id = pilot_sc_ind_[p_sc_id];
        //    mat_ifft_in(sc_id, 0) = pilot_sc_val_[p_sc_id];
        //}
        complex_float* cur_modul_buf
            = &ifft_buffer_.IFFT_inputs[IFFT_buffer_target_id + ant_id][0];
        for (size_t n = 0; n < FFT_LEN; n++) {
            // printf("ul_symbol_id %d, ue_id %d, sc %d\n", ul_symbol_id,
            // ant_id, n);
            cur_modul_buf[n]
                = ul_IQ_modul[ul_symbol_id * numAntennas + ant_id][n];
        }

        DftiComputeBackward(mkl_handle,
            ifft_buffer_.IFFT_inputs[IFFT_buffer_target_id + ant_id]);
        cx_float* ifft_out_buffer
            = (cx_float*)
                  ifft_buffer_.IFFT_inputs[IFFT_buffer_target_id + ant_id];
        cx_fmat mat_ifft_out(ifft_out_buffer, FFT_LEN, 1, false);
        float max_val = abs(mat_ifft_out).max();
        mat_ifft_out /= max_val;

        size_t tx_offset = txbuf_offset + ant_id * tx_packet_length;
        char* cur_tx_buffer = &tx_buffer_[tx_offset];
#ifndef USE_ARGOS
        // complex_float* tx_buffer_ptr = (complex_float*)(cur_tx_buffer +
        // prefix_len*sizeof(complex_float) + config_->packet_header_offset);
        struct Packet* pkt = (struct Packet*)cur_tx_buffer;
        pkt->frame_id = frame_id;
        pkt->symbol_id = symbol_id;
        pkt->cell_id = 0;
        pkt->ant_id = ant_id;
        short* tx_buffer_ptr = &pkt->data[2 * prefix_len];
#else
        // complex_float* tx_buffer_ptr = (complex_float*)((char*)cur_tx_buffer
        // + config_->prefix*sizeof(complex_float));
        short* tx_buffer_ptr = (short*)cur_tx_buffer + 2 * prefix_len;
#endif
        // fft shift
        // for(int j = 0; j < (FFT_LEN); j++) {
        //    if (j < FFT_LEN / 2)
        //        tx_fftshift_offset = FFT_LEN/2;
        //    else
        //        tx_fftshift_offset = -FFT_LEN/2;
        //    float* tx_buffer_ptr =
        //    (float*)tx_buffer_.buffer[ca_offset].data()+tx_offset*2+tx_fftshift_offset+prefix_len;
        //    *(tx_buffer_ptr)   = cur_fft_buffer_float_output[2*j];
        //    *(tx_buffer_ptr+1) = cur_fft_buffer_float_output[2*j+1];
        //}

        short* cur_buffer;
        for (size_t i = 0; i < ofdm_syms; i++) {
            size_t sym_offset = i * (FFT_LEN + CP_LEN);
            cur_buffer
                = tx_buffer_ptr + (sym_offset * sizeof(std::complex<short>));
            for (size_t j = CP_LEN; j < CP_LEN + FFT_LEN; j++) {
                *(cur_buffer + 2 * j)
                    = (short)(ifft_out_buffer[j - CP_LEN].real() * 32768);
                *(cur_buffer + 2 * j + 1)
                    = (short)(ifft_out_buffer[j - CP_LEN].imag() * 32768);
            }
            memcpy((void*)cur_buffer, (void*)(cur_buffer + 2 * FFT_LEN),
                CP_LEN * sizeof(std::complex<short>)); // add CP
        }
    }

    // l2_buffer_status_[offset] = 0; // now empty

    Event_data tx_finish_event(EventType::kIFFT, frame_id);

    if (!message_queue_.enqueue(*task_ptok[tid], tx_finish_event)) {
        printf("Muliplexing message enqueue failed\n");
        exit(0);
    }
}

void Phy_UE::initialize_vars_from_cfg(void)
{
    pilots_ = config_->pilots_;

#if DEBUG_PRINT_PILOT
    cout << "Pilot data" << endl;
    for (size_t i = 0; i < config_->OFDM_CA_NUM; i++)
        cout << pilots_[i] << ",";
    cout << endl;
#endif

    // demul_block_size = config_->demul_block_size;
    // //OFDM_CA_NUM*2/transpose_block_size; demul_block_num = OFDM_DATA_NUM /
    // demul_block_size + (OFDM_DATA_NUM % demul_block_size == 0 ? 0 : 1);

    // downlink_mode = config_->downlink_mode;
    // dl_data_subframe_start = config_->dl_data_symbol_start;
    // dl_data_subframe_end = config_->dl_data_symbol_end;
    packet_length = config_->packet_length;
#ifndef USE_ARGOS
    tx_packet_length = config_->packet_length;
#else
    tx_packet_length = packet_length - offsetof(Packet, data);
#endif

    symbol_perframe = config_->symbol_num_perframe;
    dl_pilot_symbol_perframe = DL_PILOT_SYMS;
    ul_pilot_symbol_perframe = config_->pilot_symbol_num_perframe;
    ul_data_symbol_perframe = config_->ul_data_symbol_num_perframe;
    dl_symbol_perframe = config_->dl_data_symbol_num_perframe;
    dl_data_symbol_perframe
        = config_->dl_data_symbol_num_perframe - dl_pilot_symbol_perframe;
    rx_symbol_perframe = dl_symbol_perframe;
#ifndef USE_ARGOS
    tx_symbol_perframe = ul_pilot_symbol_perframe + ul_data_symbol_perframe;
#else
    tx_symbol_perframe
        = ul_data_symbol_perframe; // pilots are preloaded into radio hw buffers
#endif
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
    nCPUs = std::thread::hardware_concurrency();
    rx_thread_num = config_->socket_thread_num;
    // nCPUs >= 2 * RX_THREAD_NUM and nUEs >= RX_THREAD_NUM
    //? RX_THREAD_NUM
    //: nUEs;
    tx_thread_num = SEPARATE_TX_RX_UE ? config_->socket_thread_num : 0;
    worker_thread_num = config_->worker_thread_num;
    core_offset = config_->core_offset;
#ifdef ENABLE_CPU_ATTACH
    size_t max_core
        = 1 + rx_thread_num + tx_thread_num + worker_thread_num + core_offset;
    if (max_core >= nCPUs) {
        printf("Cannot allocate cores: max_core %zu, available cores %zu\n",
            max_core, nCPUs);
        exit(1);
    }
#endif
    numAntennas = config_->UE_ANT_NUM;
    printf("ofdm_syms %zu, %zu symbols, %zu pilot symbols, %zu UL data "
           "symbols, %zu DL data symbols\n",
        ofdm_syms, symbol_perframe, ul_pilot_symbol_perframe,
        ul_data_symbol_perframe, dl_data_symbol_perframe);

    tx_buffer_status_size
        = (ul_data_symbol_perframe * numAntennas * TASK_BUFFER_FRAME_NUM * 36);
    tx_buffer_size = tx_packet_length * tx_buffer_status_size;
    rx_buffer_status_size
        = (dl_symbol_perframe * numAntennas * TASK_BUFFER_FRAME_NUM * 36);
    rx_buffer_size = packet_length * rx_buffer_status_size;
}

void Phy_UE::getDemulData(long long** ptr, int* size)
{
    *ptr = (long long*)&equal_buffer_[max_equaled_frame
        * dl_data_symbol_perframe][0];
    *size = numAntennas * FFT_LEN;
}

void Phy_UE::getEqualPCData(float** ptr, int* size, int ue_id)
{
    *ptr = (float*)&equal_pc_buffer_[max_equaled_frame * dl_data_symbol_perframe
            * numAntennas
        + ue_id][0];
    *size = data_sc_len * 2;
}

void Phy_UE::getEqualData(float** ptr, int* size, int ue_id)
{
    *ptr = (float*)&equal_buffer_[max_equaled_frame * dl_data_symbol_perframe
            * numAntennas
        + ue_id][0];
    *size = numAntennas * non_null_sc_len * 2;
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
EXPORT void Phy_UE_getEqualPCData(Phy_UE* usr, float** ptr, int* size, int ue)
{
    return usr->getEqualPCData(ptr, size, ue);
}
EXPORT void Phy_UE_getDemulData(Phy_UE* usr, long long** ptr, int* size)
{
    return usr->getDemulData(ptr, size);
}
}
