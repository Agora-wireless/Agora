#include "phy-ue.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "utils_ldpc.hpp"

static constexpr bool kDebugPrintPacketsFromMac = false;
static constexpr bool kDebugPrintPacketsToMac = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

Phy_UE::Phy_UE(Config* config)
{
    srand(time(NULL));

    this->config_ = config;
    initialize_vars_from_cfg();

    std::vector<size_t> data_sc_ind_;
    for (size_t i = config_->OFDM_DATA_START;
         i < config_->OFDM_DATA_START + config_->OFDM_DATA_NUM; i++)
        data_sc_ind_.push_back(i);

    non_null_sc_ind_.insert(
        non_null_sc_ind_.end(), data_sc_ind_.begin(), data_sc_ind_.end());
    std::sort(non_null_sc_ind_.begin(), non_null_sc_ind_.end());

    ue_pilot_vec.resize(config_->UE_ANT_NUM);
    for (size_t i = 0; i < config_->UE_ANT_NUM; i++) {
        for (size_t j = config->ofdm_tx_zero_prefix_;
             j < config_->sampsPerSymbol - config->ofdm_tx_zero_postfix_; j++) {
            ue_pilot_vec[i].push_back(std::complex<float>(
                config_->ue_specific_pilot_t[i][j].real() / 32768.0,
                config_->ue_specific_pilot_t[i][j].imag() / 32768.0));
        }
    }

    fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * dl_symbol_perframe * config_->UE_ANT_NUM * 36);
    demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(TASK_BUFFER_FRAME_NUM
        * dl_data_symbol_perframe * config_->UE_ANT_NUM * 36);
    decode_queue_
        = moodycamel::ConcurrentQueue<Event_data>(TASK_BUFFER_FRAME_NUM
            * dl_data_symbol_perframe * config_->UE_ANT_NUM * 36);
    message_queue_
        = moodycamel::ConcurrentQueue<Event_data>(TASK_BUFFER_FRAME_NUM
            * config_->symbol_num_perframe * config_->UE_ANT_NUM * 36);
    encode_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * config_->UE_NUM * 36);
    modul_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * config_->UE_NUM * 36);
    ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * config_->UE_NUM * 36);
    tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * config_->UE_NUM * 36);
    to_mac_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        TASK_BUFFER_FRAME_NUM * config_->UE_NUM * 36);

    for (size_t i = 0; i < rx_thread_num; i++) {
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);
        tx_ptoks_ptr[i] = new moodycamel::ProducerToken(tx_queue_);
    }

    for (size_t i = 0; i < rx_thread_num; i++) {
        mac_rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);
        mac_tx_ptoks_ptr[i] = new moodycamel::ProducerToken(to_mac_queue_);
    }

    for (size_t i = 0; i < config_->worker_thread_num; i++) {
        task_ptok[i] = new moodycamel::ProducerToken(message_queue_);
    }

    ru_.reset(new RadioTXRX(config_, rx_thread_num, config_->core_offset + 1,
        &message_queue_, &tx_queue_, rx_ptoks_ptr, tx_ptoks_ptr));

    if (kEnableMac) {
        // TODO [ankalia]: dummy_decoded_buffer is used at the base station
        // server only, but MacThread for now requires it for the UE client too
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t> dummy_decoded_buffer;

        const size_t mac_cpu_core = config_->core_offset + 1 + rx_thread_num;
        mac_thread_ = new MacThread(MacThread::Mode::kClient, config_,
            mac_cpu_core, dummy_decoded_buffer, &ul_bits_buffer_,
            &ul_bits_buffer_status_, nullptr /* dl bits buffer */,
            nullptr /* dl bits buffer status */, &to_mac_queue_,
            &message_queue_);

        mac_std_thread_ = std::thread(&MacThread::run_event_loop, mac_thread_);
    }

    printf("initializing buffers...\n");

    // uplink buffers init (tx)
    initialize_uplink_buffers();
    // downlink buffers init (rx)
    initialize_downlink_buffers();

    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, config_->OFDM_CA_NUM);
    (void)DftiCommitDescriptor(mkl_handle);

    // initilize all kinds of checkers
    memset(csi_checker_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    memset(data_checker_, 0,
        sizeof(int) * TASK_BUFFER_FRAME_NUM * config_->UE_ANT_NUM);

    memset(demul_status_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    if (dl_data_symbol_perframe > 0) {
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            demul_checker_[i] = new size_t[dl_data_symbol_perframe];
            memset(
                demul_checker_[i], 0, sizeof(int) * (dl_data_symbol_perframe));
        }
    }

    memset(decode_status_, 0, sizeof(int) * TASK_BUFFER_FRAME_NUM);
    if (dl_data_symbol_perframe > 0) {
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            decode_checker_[i] = new size_t[dl_data_symbol_perframe];
            memset(
                decode_checker_[i], 0, sizeof(int) * (dl_data_symbol_perframe));
        }
    }

    // create task thread
    for (size_t i = 0; i < config_->worker_thread_num; i++) {
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

    if (kEnableMac)
        mac_std_thread_.join();
    delete mac_thread_;
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
    pin_to_core_with_offset(ThreadType::kMaster, config_->core_offset, 0);

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
    moodycamel::ProducerToken ptok_decode(decode_queue_);
    moodycamel::ProducerToken ptok_mac(to_mac_queue_);
    moodycamel::ProducerToken ptok_ifft(ifft_queue_);
    moodycamel::ProducerToken ptok_encode(encode_queue_);

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
    size_t cur_frame_id = 0;
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

            case EventType::kPacketRX: {
                // int offset = event.tags[0];
                size_t rx_thread_id = rx_tag_t(event.tags[0]).tid;
                size_t offset_in_current_buffer
                    = rx_tag_t(event.tags[0]).offset;

                struct Packet* pkt = (struct Packet*)(rx_buffer_[rx_thread_id]
                    + offset_in_current_buffer * config_->packet_length);
                frame_id = pkt->frame_id;
                symbol_id = pkt->symbol_id;
                ant_id = pkt->ant_id;
                rt_assert(pkt->frame_id < cur_frame_id + TASK_BUFFER_FRAME_NUM,
                    "Error: Received packet for future frame beyond frame "
                    "window. This can happen if PHY is running "
                    "slowly, e.g., in debug mode");

                if (symbol_id
                    == 0) { // To send uplink pilots in simulation mode
                    Event_data do_tx_pilot_task(EventType::kPacketPilotTX,
                        gen_tag_t::frm_sym_ue(
                            frame_id, config_->pilotSymbols[0][ant_id], ant_id)
                            ._tag);
                    schedule_task(do_tx_pilot_task, &tx_queue_,
                        *tx_ptoks_ptr[ant_id % rx_thread_num]);
                }

                size_t dl_symbol_id = 0;
                if (config_->DLSymbols.size() > 0
                    && config_->DLSymbols[0].size() > 0)
                    dl_symbol_id = config_->DLSymbols[0][0];
                if (ul_data_symbol_perframe > 0
                    && (symbol_id == 0 || symbol_id == dl_symbol_id)
                    && ant_id % config_->nChannels == 0) {
                    Event_data do_encode_task(EventType::kEncode,
                        gen_tag_t::frm_sym_ue(
                            frame_id, symbol_id, ant_id / config_->nChannels)
                            ._tag);
                    schedule_task(do_encode_task, &encode_queue_, ptok_encode);
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
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                // checker to count # of pilots/users
                size_t dl_symbol_id
                    = config_->get_dl_symbol_idx(frame_id, symbol_id);
                if (dl_symbol_id >= dl_pilot_symbol_perframe) {
                    Event_data do_demul_task(EventType::kDemul, event.tags[0]);
                    schedule_task(do_demul_task, &demul_queue_, ptok_demul);
                }
                csi_checker_[frame_id]++;
                if (csi_checker_[frame_id]
                    == dl_pilot_symbol_perframe * config_->UE_ANT_NUM) {
                    csi_checker_[frame_id] = 0;
                    if (kDebugPrintPerFrameDone)
                        printf("Main thread: pilot frame: %zu, finished "
                               "collecting "
                               "pilot frames\n",
                            frame_id);
                }

            } break;

            case EventType::kDemul: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
                Event_data do_decode_task(EventType::kDecode, event.tags[0]);
                schedule_task(do_decode_task, &decode_queue_, ptok_decode);
                demul_checker_[frame_slot][symbol_id]++;
                // if this symbol is ready
                if (demul_checker_[frame_slot][symbol_id]
                    == config_->UE_ANT_NUM) {

                    if (kDebugPrintInTask)
                        printf("Main thread: Demodulation done frame: %zu, "
                               "symbol %zu\n",
                            frame_id, symbol_id);
                    max_equaled_frame = frame_id;
                    demul_checker_[frame_slot][symbol_id] = 0;
                    demul_status_[frame_slot]++;
                    if (demul_status_[frame_slot] == dl_data_symbol_perframe) {
                        if (kDebugPrintPerTaskDone)
                            printf(
                                "Main thread: Demodulation done frame: %zu \n",
                                frame_id);
                        demul_status_[frame_slot] = 0;
                    }
                    demul_count++;
                    if (demul_count == dl_data_symbol_perframe * 9000) {
                        demul_count = 0;
                        auto demul_end = std::chrono::system_clock::now();
                        std::chrono::duration<double> diff
                            = demul_end - demul_begin;
                        int samples_num_per_UE = config_->OFDM_DATA_NUM
                            * dl_data_symbol_perframe * 1000;
                        printf("Frame %zu: RX %d samples (per-client) from %zu "
                               "clients "
                               "in %f secs, throughtput %f bps per-client "
                               "(16QAM), current task queue length %zu\n",
                            frame_id, samples_num_per_UE, config_->UE_ANT_NUM,
                            diff.count(),
                            samples_num_per_UE * log2(16.0f) / diff.count(),
                            demul_queue_.size_approx());
                        demul_begin = std::chrono::system_clock::now();
                    }
                }
            } break;

            case EventType::kDecode: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
                if (kEnableMac)
                    schedule_task(Event_data(EventType::kDecode, event.tags[0]),
                        &to_mac_queue_, ptok_mac);
                decode_checker_[frame_slot][symbol_id]++;
                // if this symbol is ready
                if (decode_checker_[frame_slot][symbol_id]
                    == config_->UE_ANT_NUM) {

                    if (kDebugPrintInTask)
                        printf("Main thread: Decoding done frame: %zu, "
                               "symbol %zu\n",
                            frame_id, symbol_id);
                    decode_checker_[frame_slot][symbol_id] = 0;
                    decode_status_[frame_slot]++;
                    if (decode_status_[frame_slot] == dl_data_symbol_perframe) {
                        if (kDebugPrintPerTaskDone)
                            printf("Main thread: Decoding done frame: %zu \n",
                                frame_id);
                        decode_status_[frame_slot] = 0;
                    }
                }
            } break;

            case EventType::kPacketToMac: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;

                if (kDebugPrintPacketsToMac) {
                    printf("Main thread: sent decoded packet for frame %zu"
                           ", symbol %zu to MAC\n",
                        frame_id, symbol_id);
                }

            } break;

            case EventType::kPacketFromMac: {
                // size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t ue_id = rx_tag_t(event.tags[0]).tid;
                size_t radio_buf_id = rx_tag_t(event.tags[0]).offset;
                rt_assert(radio_buf_id
                    == expected_frame_id_from_mac_ % TASK_BUFFER_FRAME_NUM);

                MacPacket* pkt = reinterpret_cast<MacPacket*>(
                    &ul_bits_buffer_[ue_id][radio_buf_id
                        * config_->mac_bytes_num_perframe]);
                rt_assert(pkt->frame_id == expected_frame_id_from_mac_,
                    "Incorrect frame ID from MAC");
                current_frame_user_num_
                    = (current_frame_user_num_ + 1) % config_->UE_ANT_NUM;
                if (current_frame_user_num_ == 0)
                    expected_frame_id_from_mac_++;

                if (kDebugPrintPacketsFromMac) {
                    printf("Main thread: received packet for frame %u with "
                           "modulation %zu\n",
                        pkt->frame_id, pkt->rb_indicator.mod_order_bits);
                    std::stringstream ss;
                    ss << "PhyUE kPacketFromMac, frame ID " << pkt->frame_id
                       << ", bytes: ";
                    for (size_t i = 0; i < 4; i++) {
                        ss << std::to_string(
                                  (reinterpret_cast<uint8_t*>(pkt->data)[i]))
                           << ", ";
                    }
                    printf("%s\n", ss.str().c_str());
                }

            } break;

            case EventType::kEncode: {
                // size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                // size_t data_symbol_idx = gen_tag_t(event.tags[0]).symbol_id;
                // size_t ue_id = gen_tag_t(event.tags[0]).ant_id;
                Event_data do_modul_task(EventType::kModul, event.tags[0]);
                schedule_task(do_modul_task, &modul_queue_, ptok_modul);

            } break;

            case EventType::kModul: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                size_t ue_id = gen_tag_t(event.tags[0]).ue_id;

                Event_data do_ifft_task(EventType::kIFFT,
                    gen_tag_t::frm_sym_ue(frame_id, symbol_id, ue_id)._tag);
                schedule_task(do_ifft_task, &ifft_queue_, ptok_ifft);
                if (kDebugPrintPerTaskDone)
                    printf("Main thread: frame: %zu, symbol: %zu, finished "
                           "modulating "
                           "uplink data for user %zu\n",
                        frame_id, symbol_id, ue_id);
                //}
            } break;

            case EventType::kIFFT: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t ue_id = gen_tag_t(event.tags[0]).ue_id;
                Event_data do_tx_task(EventType::kPacketTX, event.tags[0]);
                schedule_task(do_tx_task, &tx_queue_,
                    *tx_ptoks_ptr[ue_id % rx_thread_num]);
                if (kDebugPrintPerTaskDone)
                    printf("Main thread: frame: %zu, finished IFFT of "
                           "uplink data for user %zu\n",
                        frame_id, ue_id);
            } break;

            case EventType::kPacketPilotTX: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                size_t ue_id = gen_tag_t(event.tags[0]).ue_id;
                if (ul_data_symbol_perframe == 0)
                    cur_frame_id++;
                if (kDebugPrintPerSymbolDone) {
                    printf("Main thread: finished Pilot TX for user %zu"
                           " in frame %zu, symbol %zu\n",
                        ue_id, frame_id, symbol_id);
                }
            } break;

            case EventType::kPacketTX: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t ue_id = gen_tag_t(event.tags[0]).ue_id;
                cur_frame_id = frame_id;
                rt_assert(frame_id == next_frame_processed_[ue_id],
                    "Unexpected frame_id was transmitted!");

                // printf("PhyUE kPacketTX: Freeing buffer %zu for UE %zu\n",
                //    num_frames_consumed_[ue_id] % TASK_BUFFER_FRAME_NUM, ue_id);
                ul_bits_buffer_status_[ue_id][next_frame_processed_[ue_id]
                    % TASK_BUFFER_FRAME_NUM]
                    = 0;
                next_frame_processed_[ue_id]++;

                if (kDebugPrintPerSymbolDone) {
                    size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                    printf("Main thread: finished TX for frame %zu, symbol "
                           "%zu, user %zu\n",
                        frame_id, symbol_id, ue_id);
                }
            } break;

            default:
                std::cout << "Invalid Event Type!" << std::endl;
                exit(0);
            }
        }
    }
    if (kPrintPhyStats) {
        const size_t task_buffer_symbol_num_dl
            = dl_data_symbol_perframe * TASK_BUFFER_FRAME_NUM;
        for (size_t ue_id = 0; ue_id < config_->UE_ANT_NUM; ue_id++) {
            size_t total_decoded_bits(0);
            size_t total_bit_errors(0);
            size_t total_decoded_blocks(0);
            size_t total_block_errors(0);
            for (size_t i = 0; i < task_buffer_symbol_num_dl; i++) {
                total_decoded_bits += decoded_bits_count_[ue_id][i];
                total_bit_errors += bit_error_count_[ue_id][i];
                total_decoded_blocks += decoded_blocks_count_[ue_id][i];
                total_block_errors += block_error_count_[ue_id][i];
            }
            std::cout << "UE " << ue_id << ": bit errors (BER) "
                      << total_bit_errors << "/" << total_decoded_bits << "("
                      << 1.0 * total_bit_errors / total_decoded_bits
                      << "), block errors (BLER) " << total_block_errors << "/"
                      << total_decoded_blocks << " ("
                      << 1.0 * total_block_errors / total_decoded_blocks << ")"
                      << std::endl;
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
    pin_to_core_with_offset(ThreadType::kWorker,
        config_->core_offset + rx_thread_num + 1
            + (kEnableMac ? rx_thread_num : 0),
        tid);

    // task_ptok[tid].reset(new moodycamel::ProducerToken(message_queue_));

    Event_data event;
    while (config_->running) {
        if (decode_queue_.try_dequeue(event))
            doDecode(tid, event.tags[0]);
        else if (demul_queue_.try_dequeue(event))
            doDemul(tid, event.tags[0]);
        else if (ifft_queue_.try_dequeue(event))
            doIFFT(tid, event.tags[0]);
        else if (modul_queue_.try_dequeue(event))
            doModul(tid, event.tags[0]);
        else if (encode_queue_.try_dequeue(event))
            doEncode(tid, event.tags[0]);
        else if (fft_queue_.try_dequeue(event))
            doFFT(tid, event.tags[0]);
    }
}

//////////////////////////////////////////////////////////
//                   DOWNLINK Operations                  //
//////////////////////////////////////////////////////////

void Phy_UE::doFFT(int tid, size_t tag)
{

    size_t rx_thread_id = fft_req_tag_t(tag).tid;
    size_t offset_in_current_buffer = fft_req_tag_t(tag).offset;

    // read info of one frame
    struct Packet* pkt = (struct Packet*)(rx_buffer_[rx_thread_id]
        + offset_in_current_buffer * config_->packet_length);
    size_t frame_id = pkt->frame_id;
    size_t symbol_id = pkt->symbol_id;
    // int cell_id = pkt->cell_id;
    size_t ant_id = pkt->ant_id;
    size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;

    if (!config_->isPilot(frame_id, symbol_id)
        && !(config_->isDownlink(frame_id, symbol_id)))
        return;

    if (kDebugPrintInTask) {
        printf("In doFFT TID %d: frame %zu, symbol %zu, ant_id %zu\n", tid,
            frame_id, symbol_id, ant_id);
    }

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
            // record_frame = -1;
        }
    }
#endif

    // remove CP, do FFT
    size_t dl_symbol_id = config_->get_dl_symbol_idx(frame_id, symbol_id);
    size_t total_dl_symbol_id = frame_slot * dl_symbol_perframe + dl_symbol_id;
    size_t FFT_buffer_target_id
        = total_dl_symbol_id * config_->UE_ANT_NUM + ant_id;

    // transfer ushort to float
    size_t delay_offset
        = (config_->ofdm_rx_zero_prefix_client_ + config_->CP_LEN) * 2;
    float* cur_fft_buffer_float = (float*)fft_buffer_[FFT_buffer_target_id];

    for (size_t i = 0; i < (config_->OFDM_CA_NUM) * 2; i++)
        cur_fft_buffer_float[i] = pkt->data[delay_offset + i] / 32768.2f;

    // perform fft
    DftiComputeForward(mkl_handle, fft_buffer_[FFT_buffer_target_id]);

    size_t csi_offset = frame_slot * config_->UE_ANT_NUM + ant_id;
    cx_float* csi_buffer_ptr = (cx_float*)(csi_buffer_[csi_offset].data());
    cx_float* fft_buffer_ptr = (cx_float*)fft_buffer_[FFT_buffer_target_id];

    Event_data fft_finish_event;

    // In TDD massive MIMO, a pilot symbol needs to be sent
    // in the downlink for the user to estimate the channel
    // due to relative reciprocity calibration,
    // see Argos paper (Mobicom'12)
    if (dl_symbol_id < dl_pilot_symbol_perframe) {
        for (size_t j = 0; j < config_->OFDM_DATA_NUM; j++) {
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
    } else {
        size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe
            + dl_symbol_id - dl_pilot_symbol_perframe;
        size_t eq_buffer_offset
            = total_dl_symbol_id * config_->UE_ANT_NUM + ant_id;

        cx_float* equ_buffer_ptr
            = (cx_float*)(equal_buffer_[eq_buffer_offset].data());

        // use pilot subcarriers for phase tracking and correction
        float theta = 0;
        cx_float csi(1, 0);
        for (size_t j = 0; j < config_->OFDM_DATA_NUM; j++) {
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
        if (config_->get_ofdm_pilot_num() > 0)
            theta /= config_->get_ofdm_pilot_num();
        auto phc = exp(cx_float(0, -theta));
        for (size_t j = 0; j < config_->OFDM_DATA_NUM; j++) {
            if (j % config_->OFDM_PILOT_SPACING != 0) {
                // divide fft output by pilot data to get CSI estimation
                size_t sc_id = non_null_sc_ind_[j];
                if (dl_pilot_symbol_perframe > 0) {
                    csi = csi_buffer_ptr[j];
                }
                cx_float y = fft_buffer_ptr[sc_id];
                equ_buffer_ptr[j] = (y / csi) * phc;
                // FIXME: this seems to not work for ant_id > 0,
                /*
                complex_float tx
                    = config_
                          ->dl_iq_f[dl_symbol_id][ant_id * config_->OFDM_CA_NUM
                              + config_->OFDM_DATA_START + j];
                evm += std::norm(equ_buffer_ptr[j] - cx_float(tx.re, tx.im));
		*/
            }
        }
        /*
        evm = std::sqrt(
            evm / (config_->OFDM_DATA_NUM - config_->get_ofdm_pilot_num()));
        if (kPrintPhyStats)
            std::cout << "Frame: " << frame_id << ", Symbol: " << symbol_id
                      << ", User: " << ant_id << ", EVM: " << 100 * evm
                      << "%, SNR: " << -10 * std::log10(evm) << std::endl;
        */
    }

    rx_buffer_status_[rx_thread_id][offset_in_current_buffer] = 0; // now empty
    fft_finish_event = Event_data(EventType::kFFT,
        gen_tag_t::frm_sym_ant(frame_id, symbol_id, ant_id)._tag);
    rt_assert(message_queue_.enqueue(*task_ptok[tid], fft_finish_event),
        "FFT message enqueue failed");
}

void Phy_UE::doDemul(int tid, size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t symbol_id = gen_tag_t(tag).symbol_id;
    const size_t ant_id = gen_tag_t(tag).ant_id;
    if (kDebugPrintInTask) {
        printf("In doDemul TID %d: frame %zu, symbol %zu, ant_id %zu\n", tid,
            frame_id, symbol_id, ant_id);
    }

    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    size_t dl_symbol_id = config_->get_dl_symbol_idx(frame_id, symbol_id);
    size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe
        + dl_symbol_id - dl_pilot_symbol_perframe;
    size_t offset = total_dl_symbol_id * config_->UE_ANT_NUM + ant_id;
    float* equal_ptr = (float*)&equal_buffer_[offset][0];
    auto* demul_ptr = dl_demod_buffer_[offset];

    // demod_16qam_hard_loop(
    //    equal_ptr, (uint8_t*)demul_ptr, config_->UE_ANT_NUM);

    switch (config_->mod_order_bits) {
    case (CommsLib::QAM16):
        demod_16qam_soft_avx2(equal_ptr, demul_ptr, config_->OFDM_DATA_NUM);
        break;
    case (CommsLib::QAM64):
        demod_64qam_soft_avx2(equal_ptr, demul_ptr, config_->OFDM_DATA_NUM);
        break;
    default:
        printf("Demodulation: modulation type %s not supported!\n",
            config_->modulation.c_str());
    }

    if (kPrintLLRData) {
        printf("LLR data, symbol_offset: %zu\n", offset);
        for (size_t i = 0; i < config_->OFDM_DATA_NUM; i++) {
            printf("%x ", (uint8_t) * (demul_ptr + i));
        }
        printf("\n");
    }

    rt_assert(message_queue_.enqueue(
                  *task_ptok[tid], Event_data(EventType::kDemul, tag)),
        "Demodulation message enqueue failed");
}

void Phy_UE::doDecode(int tid, size_t tag)
{
    LDPCconfig LDPC_config = config_->LDPC_config;
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t ant_id = gen_tag_t(tag).ant_id;
    if (kDebugPrintInTask) {
        printf("In doDecode TID %d: frame %zu, symbol %zu, ant_id %zu\n", tid,
            frame_id, symbol_id, ant_id);
    }

    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    size_t dl_symbol_id = config_->get_dl_symbol_idx(frame_id, symbol_id);
    size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe
        + dl_symbol_id - dl_pilot_symbol_perframe;
    size_t symbol_ant_offset
        = total_dl_symbol_id * config_->UE_ANT_NUM + ant_id;

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };

    // Decoder setup
    int16_t numFillerBits = 0;
    int16_t numChannelLlrs = LDPC_config.cbCodewLen;

    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = LDPC_config.decoderIter;
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = LDPC_config.earlyTermination;
    ldpc_decoder_5gnr_request.Zc = LDPC_config.Zc;
    ldpc_decoder_5gnr_request.baseGraph = LDPC_config.Bg;
    ldpc_decoder_5gnr_request.nRows = LDPC_config.nRows;

    int numMsgBits = LDPC_config.cbLen - numFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

    for (size_t cb_id = 0; cb_id < config_->LDPC_config.nblocksInSymbol;
         cb_id++) {
        size_t demod_buffer_offset
            = cb_id * LDPC_config.cbCodewLen * config_->mod_order_bits;
        size_t decode_buffer_offset
            = cb_id * roundup<64>(config_->num_bytes_per_cb);
        auto* llr_buffer_ptr
            = &dl_demod_buffer_[symbol_ant_offset][demod_buffer_offset];
        auto* decoded_buffer_ptr
            = &dl_decode_buffer_[symbol_ant_offset][decode_buffer_offset];
        ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
        ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;
        bblib_ldpc_decoder_5gnr(
            &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

        if (kPrintPhyStats) {
            decoded_bits_count_[ant_id][total_dl_symbol_id]
                += 8 * config_->num_bytes_per_cb;
            decoded_blocks_count_[ant_id][total_dl_symbol_id]++;
            size_t block_error(0);
            for (size_t i = 0; i < config_->num_bytes_per_cb; i++) {
                uint8_t rx_byte = decoded_buffer_ptr[i];
                uint8_t tx_byte = (uint8_t)config_->get_info_bits(
                    config_->dl_bits, dl_symbol_id, ant_id, cb_id)[i];
                uint8_t xor_byte(tx_byte ^ rx_byte);
                size_t bit_errors = 0;
                for (size_t j = 0; j < 8; j++) {
                    bit_errors += xor_byte & 1;
                    xor_byte >>= 1;
                }
                if (rx_byte != tx_byte)
                    block_error++;
                bit_error_count_[ant_id][total_dl_symbol_id] += bit_errors;
            }
            block_error_count_[ant_id][total_dl_symbol_id] += (block_error > 0);
        }

        if (kPrintDecodedData) {
            printf("Decoded data (original byte)\n");
            for (size_t i = 0; i < config_->num_bytes_per_cb; i++) {
                uint8_t rx_byte = decoded_buffer_ptr[i];
                uint8_t tx_byte = (uint8_t)config_->get_info_bits(
                    config_->dl_bits, dl_symbol_id, ant_id, cb_id)[i];
                printf("%x(%x) ", rx_byte, tx_byte);
            }
            printf("\n");
        }
    }

    rt_assert(message_queue_.enqueue(
                  *task_ptok[tid], Event_data(EventType::kDecode, tag)),
        "Decoding message enqueue failed");
}

//////////////////////////////////////////////////////////
//                   UPLINK Operations                //
//////////////////////////////////////////////////////////

void Phy_UE::doEncode(int tid, size_t tag)
{
    LDPCconfig LDPC_config = config_->LDPC_config;
    // size_t ue_id = rx_tag_t(tag).tid;
    // size_t offset = rx_tag_t(tag).offset;
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t ue_id = gen_tag_t(tag).ue_id;
    size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    auto& cfg = config_;
    // size_t start_tsc = worker_rdtsc();

    int8_t* encoded_buffer_temp = (int8_t*)memalign(64,
        ldpc_encoding_encoded_buf_size(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
    int8_t* parity_buffer = (int8_t*)memalign(64,
        ldpc_encoding_parity_buf_size(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));

    size_t bytes_per_block = kEnableMac
        ? (LDPC_config.cbLen) >> 3
        : roundup<64>(bits_to_bytes(LDPC_config.cbLen));
    size_t encoded_bytes_per_block = (LDPC_config.cbCodewLen + 7) >> 3;

    for (size_t ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe;
         ul_symbol_id++) {
        size_t total_ul_symbol_id
            = frame_slot * ul_data_symbol_perframe + ul_symbol_id;
        for (size_t cb_id = 0; cb_id < config_->LDPC_config.nblocksInSymbol;
             cb_id++) {
            int8_t* input_ptr;
            if (kEnableMac) {
                uint8_t* ul_bits = ul_bits_buffer_[ue_id]
                    + frame_slot * config_->mac_bytes_num_perframe;

                int input_offset = bytes_per_block
                        * cfg->LDPC_config.nblocksInSymbol * ul_symbol_id
                    + bytes_per_block * cb_id;
                input_ptr = (int8_t*)ul_bits + input_offset;
            } else {
                size_t cb_offset
                    = (ue_id * cfg->LDPC_config.nblocksInSymbol + cb_id)
                    * bytes_per_block;
                input_ptr = &cfg->ul_bits[ul_symbol_id + config_->UL_PILOT_SYMS]
                                         [cb_offset];
            }

            ldpc_encode_helper(LDPC_config.Bg, LDPC_config.Zc,
                LDPC_config.nRows, encoded_buffer_temp, parity_buffer,
                input_ptr);

            int cbCodedBytes = LDPC_config.cbCodewLen / cfg->mod_order_bits;
            int output_offset = total_ul_symbol_id * config_->OFDM_DATA_NUM
                + cbCodedBytes * cb_id;

            adapt_bits_for_mod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
                &ul_syms_buffer_[ue_id][output_offset], encoded_bytes_per_block,
                cfg->mod_order_bits);
        }
    }
    // double duration = worker_rdtsc() - start_tsc;
    // if (cycles_to_us(duration, freq_ghz) > 500) {
    //    printf("Thread %d Encode takes %.2f\n", tid,
    //        cycles_to_us(duration, freq_ghz));
    //}

    rt_assert(message_queue_.enqueue(
                  *task_ptok[tid], Event_data(EventType::kEncode, tag)),
        "Encoding message enqueue failed");
}

void Phy_UE::doModul(int tid, size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t ue_id = gen_tag_t(tag).ue_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    for (size_t ch = 0; ch < config_->nChannels; ch++) {
        size_t ant_id = ue_id * config_->nChannels + ch;
        for (size_t ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe;
             ul_symbol_id++) {
            size_t total_ul_symbol_id
                = frame_slot * ul_data_symbol_perframe + ul_symbol_id;
            complex_float* modul_buf = &modul_buffer_[total_ul_symbol_id][ant_id
                * config_->OFDM_DATA_NUM];
            int8_t* ul_bits
                = (int8_t*)&ul_syms_buffer_[ant_id][total_ul_symbol_id
                    * config_->OFDM_DATA_NUM];
            for (size_t sc = 0; sc < config_->OFDM_DATA_NUM; sc++) {
                modul_buf[sc] = mod_single_uint8(
                    (uint8_t)ul_bits[sc], config_->mod_table);
            }
        }
    }
    rt_assert(message_queue_.enqueue(
                  *task_ptok[tid], Event_data(EventType::kModul, tag)),
        "Muliplexing message enqueue failed");
}

void Phy_UE::doIFFT(int tid, size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
    const size_t ue_id = gen_tag_t(tag).ue_id;
    for (size_t ch = 0; ch < config_->nChannels; ch++) {
        size_t ant_id = ue_id * config_->nChannels + ch;
        for (size_t ul_symbol_id = 0; ul_symbol_id < ul_symbol_perframe;
             ul_symbol_id++) {

            size_t total_ul_symbol_id
                = frame_slot * ul_symbol_perframe + ul_symbol_id;
            size_t buff_offset
                = total_ul_symbol_id * config_->UE_ANT_NUM + ant_id;
            complex_float* ifft_buff = ifft_buffer_[buff_offset];

            memset(
                ifft_buff, 0, sizeof(complex_float) * config_->OFDM_DATA_START);
            if (ul_symbol_id < config_->UL_PILOT_SYMS) {
                memcpy(ifft_buff + config_->OFDM_DATA_START,
                    config_->ue_specific_pilot[ant_id],
                    config_->OFDM_DATA_NUM * sizeof(complex_float));
            } else {
                size_t total_ul_data_symbol_id
                    = frame_slot * ul_data_symbol_perframe + ul_symbol_id
                    - config_->UL_PILOT_SYMS;
                complex_float* modul_buff
                    = &modul_buffer_[total_ul_data_symbol_id]
                                    [ant_id * config_->OFDM_DATA_NUM];
                memcpy(ifft_buff + config_->OFDM_DATA_START, modul_buff,
                    config_->OFDM_DATA_NUM * sizeof(complex_float));
            }
            memset(ifft_buff + config_->OFDM_DATA_STOP, 0,
                sizeof(complex_float) * config_->OFDM_DATA_START);

            CommsLib::IFFT(ifft_buff, config_->OFDM_CA_NUM, false);

            size_t tx_offset = buff_offset * config_->packet_length;
            char* cur_tx_buffer = &tx_buffer_[tx_offset];
            struct Packet* pkt = (struct Packet*)cur_tx_buffer;
            std::complex<short>* tx_data_ptr = (std::complex<short>*)pkt->data;
            CommsLib::ifft2tx(ifft_buff, tx_data_ptr, config_->OFDM_CA_NUM,
                config_->ofdm_tx_zero_prefix_, config_->CP_LEN, config_->scale);
        }
    }

    rt_assert(message_queue_.enqueue(
                  *task_ptok[tid], Event_data(EventType::kIFFT, tag)),
        "Muliplexing message enqueue failed");
}

void Phy_UE::initialize_vars_from_cfg(void)
{
    dl_pilot_symbol_perframe = config_->DL_PILOT_SYMS;
    ul_pilot_symbol_perframe = config_->UL_PILOT_SYMS;
    ul_symbol_perframe = config_->ul_data_symbol_num_perframe;
    dl_symbol_perframe = config_->dl_data_symbol_num_perframe;
    dl_data_symbol_perframe = dl_symbol_perframe - dl_pilot_symbol_perframe;
    ul_data_symbol_perframe = ul_symbol_perframe - ul_pilot_symbol_perframe;
    nCPUs = std::thread::hardware_concurrency();
    rx_thread_num = (kUseArgos && config_->hw_framer)
        ? std::min(config_->UE_NUM, config_->rx_thread_num)
        : kUseArgos ? config_->UE_NUM : config_->rx_thread_num;

    tx_buffer_status_size
        = (ul_symbol_perframe * config_->UE_ANT_NUM * TASK_BUFFER_FRAME_NUM);
    tx_buffer_size = config_->packet_length * tx_buffer_status_size;
    rx_buffer_status_size
        = (dl_symbol_perframe + config_->beacon_symbol_num_perframe)
        * config_->UE_ANT_NUM * TASK_BUFFER_FRAME_NUM;
    rx_buffer_size = config_->packet_length * rx_buffer_status_size;
}

void Phy_UE::initialize_uplink_buffers()
{
    // initialize ul data buffer
    ul_bits_buffer_size_
        = TASK_BUFFER_FRAME_NUM * config_->mac_bytes_num_perframe;
    ul_bits_buffer_.malloc(config_->UE_ANT_NUM, ul_bits_buffer_size_, 64);
    ul_bits_buffer_status_.calloc(
        config_->UE_ANT_NUM, TASK_BUFFER_FRAME_NUM, 64);
    ul_syms_buffer_size_ = TASK_BUFFER_FRAME_NUM * ul_data_symbol_perframe
        * config_->OFDM_DATA_NUM;
    ul_syms_buffer_.calloc(config_->UE_ANT_NUM, ul_syms_buffer_size_, 64);

    // initialize modulation buffer
    modul_buffer_.calloc(ul_data_symbol_perframe * TASK_BUFFER_FRAME_NUM,
        config_->OFDM_DATA_NUM * config_->UE_ANT_NUM, 64);

    // initialize IFFT buffer
    size_t ifft_buffer_block_num
        = config_->UE_ANT_NUM * ul_symbol_perframe * TASK_BUFFER_FRAME_NUM;
    ifft_buffer_.calloc(ifft_buffer_block_num, config_->OFDM_CA_NUM, 64);

    alloc_buffer_1d(&tx_buffer_, tx_buffer_size, 64, 0);
    alloc_buffer_1d(&tx_buffer_status_, tx_buffer_status_size, 64, 1);
}

void Phy_UE::initialize_downlink_buffers()
{
    // initialize rx buffer
    rx_buffer_.malloc(rx_thread_num, rx_buffer_size, 64);
    rx_buffer_status_.calloc(rx_thread_num, rx_buffer_status_size, 64);

    // initialize FFT buffer
    size_t FFT_buffer_block_num
        = config_->UE_ANT_NUM * dl_symbol_perframe * TASK_BUFFER_FRAME_NUM;
    fft_buffer_.calloc(FFT_buffer_block_num, config_->OFDM_CA_NUM, 64);

    // initialize CSI buffer
    csi_buffer_.resize(config_->UE_ANT_NUM * TASK_BUFFER_FRAME_NUM);
    for (size_t i = 0; i < csi_buffer_.size(); i++)
        csi_buffer_[i].resize(config_->OFDM_DATA_NUM);

    if (dl_data_symbol_perframe > 0) {
        // initialize equalized data buffer
        const size_t task_buffer_symbol_num_dl
            = dl_data_symbol_perframe * TASK_BUFFER_FRAME_NUM;
        size_t buffer_size = config_->UE_ANT_NUM * task_buffer_symbol_num_dl;
        equal_buffer_.resize(buffer_size);
        for (size_t i = 0; i < equal_buffer_.size(); i++)
            equal_buffer_[i].resize(config_->OFDM_DATA_NUM);

        // initialize demod buffer
        dl_demod_buffer_.calloc(
            buffer_size, config_->OFDM_DATA_NUM * kMaxModType, 64);

        // initialize decode buffer
        dl_decode_buffer_.resize(buffer_size);
        for (size_t i = 0; i < dl_decode_buffer_.size(); i++)
            dl_decode_buffer_[i].resize(roundup<64>(config_->num_bytes_per_cb)
                * config_->LDPC_config.nblocksInSymbol);
        resp_var_nodes = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));

        decoded_bits_count_.calloc(
            config_->UE_ANT_NUM, task_buffer_symbol_num_dl, 64);
        bit_error_count_.calloc(
            config_->UE_ANT_NUM, task_buffer_symbol_num_dl, 64);

        decoded_blocks_count_.calloc(
            config_->UE_ANT_NUM, task_buffer_symbol_num_dl, 64);
        block_error_count_.calloc(
            config_->UE_ANT_NUM, task_buffer_symbol_num_dl, 64);
    }
}

void Phy_UE::getDemulData(long long** ptr, int* size)
{
    *ptr = (long long*)&equal_buffer_[max_equaled_frame
        * dl_data_symbol_perframe][0];
    *size = config_->UE_ANT_NUM * config_->OFDM_CA_NUM;
}

void Phy_UE::getEqualData(float** ptr, int* size, int ue_id)
{
    *ptr = (float*)&equal_buffer_[max_equaled_frame * dl_data_symbol_perframe
            * config_->UE_ANT_NUM
        + ue_id][0];
    *size = config_->UE_ANT_NUM * config_->OFDM_DATA_NUM * 2;
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
