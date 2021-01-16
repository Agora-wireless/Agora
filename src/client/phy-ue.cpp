#include "phy-ue.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "utils_ldpc.hpp"

static constexpr bool kDebugPrintPacketsFromMac = false;
static constexpr bool kDebugPrintPacketsToMac = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;
static constexpr bool kPrintDownlinkPilotStats = false;
static constexpr size_t kRecordFrameIndex = 1000;

Phy_UE::Phy_UE(Config* config)
{
    srand(time(NULL));

    this->config_ = config;
    initialize_vars_from_cfg();

    std::vector<size_t> data_sc_ind_;
    for (size_t i = config_->ofdm_data_start();
         i < config_->ofdm_data_start() + config_->ofdm_data_num(); i++)
        data_sc_ind_.push_back(i);

    non_null_sc_ind_.insert(
        non_null_sc_ind_.end(), data_sc_ind_.begin(), data_sc_ind_.end());
    std::sort(non_null_sc_ind_.begin(), non_null_sc_ind_.end());

    ue_pilot_vec.resize(config_->ue_ant_num());
    for (size_t i = 0; i < config_->ue_ant_num(); i++) {
        for (size_t j = config->ofdm_tx_zero_prefix();
             j < config_->samps_per_symbol() - config->ofdm_tx_zero_postfix();
             j++) {
            ue_pilot_vec[i].push_back(std::complex<float>(
                config_->ue_specific_pilot_t()[i][j].real() / 32768.0f,
                config_->ue_specific_pilot_t()[i][j].imag() / 32768.0f));
        }
    }

    fft_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        kFrameWnd * dl_symbol_perframe * config_->ue_ant_num() * 36);
    demul_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        kFrameWnd * dl_data_symbol_perframe * config_->ue_ant_num() * 36);
    decode_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        kFrameWnd * dl_data_symbol_perframe * config_->ue_ant_num() * 36);
    message_queue_ = moodycamel::ConcurrentQueue<Event_data>(kFrameWnd
        * config_->frame().NumTotalSyms() * config_->ue_ant_num() * 36);
    encode_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        kFrameWnd * config_->ue_num() * 36);
    modul_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        kFrameWnd * config_->ue_num() * 36);
    ifft_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        kFrameWnd * config_->ue_num() * 36);
    tx_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        kFrameWnd * config_->ue_num() * 36);
    to_mac_queue_ = moodycamel::ConcurrentQueue<Event_data>(
        kFrameWnd * config_->ue_num() * 36);

    for (size_t i = 0; i < rx_thread_num; i++) {
        rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);
        tx_ptoks_ptr[i] = new moodycamel::ProducerToken(tx_queue_);
    }

    for (size_t i = 0; i < rx_thread_num; i++) {
        mac_rx_ptoks_ptr[i] = new moodycamel::ProducerToken(message_queue_);
        mac_tx_ptoks_ptr[i] = new moodycamel::ProducerToken(to_mac_queue_);
    }

    for (size_t i = 0; i < config_->worker_thread_num(); i++) {
        task_ptok[i] = new moodycamel::ProducerToken(message_queue_);
    }

    ru_.reset(new RadioTXRX(config_, rx_thread_num, config_->core_offset() + 1,
        &message_queue_, &tx_queue_, rx_ptoks_ptr, tx_ptoks_ptr));

    if (kEnableMac) {
        // TODO [ankalia]: dummy_decoded_buffer is used at the base station
        // server only, but MacThread for now requires it for the UE client too
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t> dummy_decoded_buffer;

        const size_t mac_cpu_core = config_->core_offset() + 1 + rx_thread_num;
        mac_thread_ = new MacThread(MacThread::Mode::kClient, config_,
            mac_cpu_core, dummy_decoded_buffer, &ul_bits_buffer_,
            &ul_bits_buffer_status_, nullptr /* dl bits buffer */,
            nullptr /* dl bits buffer status */, &to_mac_queue_,
            &message_queue_);

        mac_std_thread_ = std::thread(&MacThread::run_event_loop, mac_thread_);
    }

    std::printf("initializing buffers...\n");

    // uplink buffers init (tx)
    initialize_uplink_buffers();
    // downlink buffers init (rx)
    initialize_downlink_buffers();

    (void)DftiCreateDescriptor(
        &mkl_handle, DFTI_SINGLE, DFTI_COMPLEX, 1, config_->ofdm_ca_num());
    (void)DftiCommitDescriptor(mkl_handle);

    // initilize all kinds of checkers
    std::memset(fft_status_, 0, sizeof(size_t) * kFrameWnd);
    for (size_t i = 0; i < kFrameWnd; i++) {
        fft_checker_[i] = new size_t[config_->ue_ant_num()];
        std::memset(
            fft_checker_[i], 0, sizeof(size_t) * (config_->ue_ant_num()));
    }

    std::memset(demul_status_, 0, sizeof(size_t) * kFrameWnd);
    if (dl_data_symbol_perframe > 0) {
        for (size_t i = 0; i < kFrameWnd; i++) {
            demul_checker_[i] = new size_t[config_->ue_ant_num()];
            std::memset(
                demul_checker_[i], 0, sizeof(size_t) * (config_->ue_ant_num()));
        }
    }

    std::memset(decode_status_, 0, sizeof(size_t) * kFrameWnd);
    if (dl_data_symbol_perframe > 0) {
        for (size_t i = 0; i < kFrameWnd; i++) {
            decode_checker_[i] = new size_t[config_->ue_ant_num()];
            std::memset(decode_checker_[i], 0,
                sizeof(size_t) * (config_->ue_ant_num()));
        }
    }

    std::memset(
        frame_dl_process_time_, 0, sizeof(size_t) * kFrameWnd * kMaxUEs);

    // create task thread
    for (size_t i = 0; i < config_->worker_thread_num(); i++) {
        auto* context = new EventHandlerContext();
        context->obj_ptr = this;
        context->id = i;

        // std::printf("create thread %d\n", i);
        if (pthread_create(&task_threads[i], NULL, taskThread_launch, context)
            != 0) {
            perror("task thread create failed");
            std::exit(0);
        }
    }
}

Phy_UE::~Phy_UE()
{
    DftiFreeDescriptor(&mkl_handle);
    // release FFT_buffer
    fft_buffer_.free();
    ifft_buffer_.free();
    std::free(rx_samps_tmp);
    if (kEnableMac == true) {
        mac_std_thread_.join();
    }
    delete mac_thread_;
}

void Phy_UE::schedule_task(Event_data do_task,
    moodycamel::ConcurrentQueue<Event_data>* in_queue,
    moodycamel::ProducerToken const& ptok)
{
    if (!in_queue->try_enqueue(ptok, do_task)) {
        std::printf("need more memory\n");
        if (!in_queue->enqueue(ptok, do_task)) {
            std::printf("task enqueue failed\n");
            std::exit(0);
        }
    }
}

//////////////////////////////////////////////////////////
//                   UPLINK Operations                  //
//////////////////////////////////////////////////////////
void Phy_UE::stop()
{
    std::cout << "stopping threads " << std::endl;
    config_->running(false);
    usleep(1000);
    ru_.reset();
}

void Phy_UE::start()
{
    pin_to_core_with_offset(ThreadType::kMaster, config_->core_offset(), 0);

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
    int miss_count = 0;
    int total_count = 0;

    Event_data events_list[kDequeueBulkSizeTXRX];
    int ret = 0;
    max_equaled_frame = 0;
    size_t frame_id, symbol_id, ant_id;
    size_t cur_frame_id = 0;
    while ((config_->running() == true)
        && (SignalHandler::gotExitSignal() == false)) {
        // get a bulk of events
        ret = message_queue_.try_dequeue_bulk(
            ctok, events_list, kDequeueBulkSizeTXRX);
        total_count++;
        if (total_count == 1e7) {
            // print the message_queue_ miss rate is needed
            // std::printf("message dequeue miss rate %f\n", (float)miss_count /
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
                    + offset_in_current_buffer * config_->packet_length());
                frame_id = pkt->frame_id;
                symbol_id = pkt->symbol_id;
                ant_id = pkt->ant_id;
                rt_assert(pkt->frame_id < cur_frame_id + kFrameWnd,
                    "Error: Received packet for future frame beyond frame "
                    "window. This can happen if PHY is running "
                    "slowly, e.g., in debug mode");

                size_t dl_symbol_id = 0;
                if ((config_->frame().NumDLSyms() > 0)) {
                    dl_symbol_id = config_->frame().GetDLSymbol(0);
                }

                if ((symbol_id == 0) // Beacon in Sim mode!
                    || ((config_->hw_framer() == false)
                           && (ul_data_symbol_perframe == 0)
                           && (symbol_id
                                  == dl_symbol_id))) { // Send uplink pilots
                    Event_data do_tx_pilot_task(EventType::kPacketPilotTX,
                        gen_tag_t::frm_sym_ue(frame_id,
                            config_->frame().GetPilotSymbol(ant_id), ant_id)
                            ._tag);
                    schedule_task(do_tx_pilot_task, &tx_queue_,
                        *tx_ptoks_ptr[ant_id % rx_thread_num]);
                }

                if (ul_data_symbol_perframe > 0
                    && (symbol_id == 0 || symbol_id == dl_symbol_id)
                    && ant_id % config_->num_channels() == 0) {
                    Event_data do_encode_task(EventType::kEncode,
                        gen_tag_t::frm_sym_ue(frame_id, symbol_id,
                            ant_id / config_->num_channels())
                            ._tag);
                    schedule_task(do_encode_task, &encode_queue_, ptok_encode);
                }

                if (dl_data_symbol_perframe > 0
                    && (config_->IsPilot(frame_id, symbol_id)
                           || config_->IsDownlink(frame_id, symbol_id))) {
                    if (dl_symbol_id == config_->frame().GetDLSymbol(0))
                        frame_dl_process_time_[(frame_id % kFrameWnd) * kMaxUEs
                            + ant_id]
                            = get_time_us();
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
                size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
                size_t frame_slot = frame_id % kFrameWnd;
                size_t dl_symbol_idx
                    = config_->GetDLSymbolIdx(frame_id, symbol_id);
                if (dl_symbol_idx >= dl_pilot_symbol_perframe) {
                    Event_data do_demul_task(EventType::kDemul, event.tags[0]);
                    schedule_task(do_demul_task, &demul_queue_, ptok_demul);
                }
                fft_checker_[frame_slot][ant_id]++;
                if (fft_checker_[frame_slot][ant_id] == dl_symbol_perframe) {
                    if (kDebugPrintPerTaskDone)
                        std::printf(
                            "Main thread: Equalization done frame: %zu, "
                            "ant_id %zu\n",
                            frame_id, ant_id);
                    fft_checker_[frame_slot][ant_id] = 0;
                    fft_status_[frame_slot]++;
                    if (fft_status_[frame_slot] == config_->ue_ant_num()) {
                        if (kDebugPrintPerFrameDone)
                            std::printf("Main thread: Equalization done on all "
                                        "antennas at frame: %zu\n",
                                frame_id);
                        fft_status_[frame_slot] = 0;
                    }
                }

            } break;

            case EventType::kDemul: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                //size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
                size_t frame_slot = frame_id % kFrameWnd;
                //size_t dl_symbol_idx
                //    = config_->GetDLSymbolIdx(frame_id, symbol_id)
                //    - dl_pilot_symbol_perframe;
                Event_data do_decode_task(EventType::kDecode, event.tags[0]);
                schedule_task(do_decode_task, &decode_queue_, ptok_decode);
                demul_checker_[frame_slot][ant_id]++;
                if (demul_checker_[frame_slot][ant_id]
                    == dl_data_symbol_perframe) {
                    if (kDebugPrintPerTaskDone)
                        std::printf(
                            "Main thread: Demodulation done frame: %zu, "
                            "ant %zu\n",
                            frame_id, ant_id);
                    max_equaled_frame = frame_id;
                    demul_checker_[frame_slot][ant_id] = 0;
                    demul_status_[frame_slot]++;
                    if (demul_status_[frame_slot] == config_->ue_ant_num()) {
                        if (kDebugPrintPerFrameDone)
                            std::printf("Main thread: Demodulation done on all "
                                        "antennas at frame: %zu \n",
                                frame_id);
                        demul_status_[frame_slot] = 0;
                    }
                }
            } break;

            case EventType::kDecode: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                // size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                size_t ant_id = gen_tag_t(event.tags[0]).ant_id;
                size_t frame_slot = frame_id % kFrameWnd;
                //size_t dl_symbol_idx
                //    = config_->GetDLSymbolIdx(frame_id, symbol_id)
                //    - dl_pilot_symbol_perframe;
                if (kEnableMac)
                    schedule_task(
                        Event_data(EventType::kPacketToMac, event.tags[0]),
                        &to_mac_queue_, ptok_mac);
                decode_checker_[frame_slot][ant_id]++;
                if (decode_checker_[frame_slot][ant_id]
                    == dl_data_symbol_perframe) {

                    if (kDebugPrintPerTaskDone)
                        std::printf("Main thread: Decoding done frame: %zu, "
                                    "ant %zu\n",
                            frame_id, ant_id);
                    decode_checker_[frame_slot][ant_id] = 0;
                    decode_status_[frame_slot]++;
                    frame_dl_process_time_[frame_slot * kMaxUEs + ant_id]
                        = get_time_us()
                        - frame_dl_process_time_[frame_slot * kMaxUEs + ant_id];
                    if (decode_status_[frame_slot] == config_->ue_ant_num()) {
                        double frame_time_total = 0;
                        for (size_t i = 0; i < config_->ue_ant_num(); i++)
                            frame_time_total
                                += frame_dl_process_time_[frame_slot * kMaxUEs
                                    + i];
                        if (kDebugPrintPerFrameDone)
                            std::printf(
                                "Main thread: Decode done on all antennas "
                                "at frame %zu"
                                " in %.2f us\n",
                                frame_id, frame_time_total);
                        decode_status_[frame_slot] = 0;
                        if (!kEnableMac)
                            cur_frame_id = frame_id;
                    }
                }
            } break;

            case EventType::kPacketToMac: {
                size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t symbol_id = gen_tag_t(event.tags[0]).symbol_id;
                cur_frame_id = frame_id;

                if (kDebugPrintPacketsToMac) {
                    std::printf("Main thread: sent decoded packet for frame %zu"
                                ", symbol %zu to MAC\n",
                        frame_id, symbol_id);
                }

            } break;

            case EventType::kPacketFromMac: {
                // size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                size_t ue_id = rx_tag_t(event.tags[0]).tid;
                size_t radio_buf_id = rx_tag_t(event.tags[0]).offset;
                rt_assert(
                    radio_buf_id == expected_frame_id_from_mac_ % kFrameWnd);

                MacPacket* pkt = reinterpret_cast<MacPacket*>(
                    &ul_bits_buffer_[ue_id][radio_buf_id
                        * config_->mac_bytes_num_perframe()]);
                rt_assert(pkt->frame_id == expected_frame_id_from_mac_,
                    "Incorrect frame ID from MAC");
                current_frame_user_num_
                    = (current_frame_user_num_ + 1) % config_->ue_ant_num();
                if (current_frame_user_num_ == 0)
                    expected_frame_id_from_mac_++;

                if (kDebugPrintPacketsFromMac) {
                    std::printf(
                        "Main thread: received packet for frame %u with "
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
                    std::printf("%s\n", ss.str().c_str());
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
                    std::printf(
                        "Main thread: frame: %zu, symbol: %zu, finished "
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
                    std::printf("Main thread: frame: %zu, finished IFFT of "
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
                    std::printf("Main thread: finished Pilot TX for user %zu"
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

                // std::printf("PhyUE kPacketTX: Freeing buffer %zu for UE %zu\n",
                //    num_frames_consumed_[ue_id] % kFrameWnd, ue_id);
                ul_bits_buffer_status_[ue_id]
                                      [next_frame_processed_[ue_id] % kFrameWnd]
                    = 0;
                next_frame_processed_[ue_id]++;

                if (kDebugPrintPerFrameDone) {
                    std::printf("Main thread: finished TX for frame %zu, "
                                "user %zu\n",
                        frame_id, ue_id);
                }
            } break;

            default:
                std::cout << "Invalid Event Type!" << std::endl;
                std::exit(0);
            }
        }
    }
    if (kPrintPhyStats) {
        const size_t task_buffer_symbol_num_dl
            = dl_data_symbol_perframe * kFrameWnd;
        for (size_t ue_id = 0; ue_id < config_->ue_ant_num(); ue_id++) {
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
                      << 1.0 * total_block_errors / total_decoded_blocks
                      << "), symbol errors " << symbol_error_count_[ue_id]
                      << "/" << decoded_symbol_count_[ue_id] << " ("
                      << 1.0 * symbol_error_count_[ue_id]
                    / decoded_symbol_count_[ue_id]
                      << ")" << std::endl;
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
    // std::printf("task thread %d starts\n", tid);
    pin_to_core_with_offset(ThreadType::kWorker,
        config_->core_offset() + rx_thread_num + 1
            + (kEnableMac ? rx_thread_num : 0),
        tid);

    // task_ptok[tid].reset(new moodycamel::ProducerToken(message_queue_));

    Event_data event;
    while (config_->running() == true) {
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
    size_t start_tsc = rdtsc();

    // read info of one frame
    struct Packet* pkt = (struct Packet*)(rx_buffer_[rx_thread_id]
        + offset_in_current_buffer * config_->packet_length());
    size_t frame_id = pkt->frame_id;
    size_t symbol_id = pkt->symbol_id;
    // int cell_id = pkt->cell_id;
    size_t ant_id = pkt->ant_id;
    size_t frame_slot = frame_id % kFrameWnd;

    if (!config_->IsPilot(frame_id, symbol_id)
        && !(config_->IsDownlink(frame_id, symbol_id)))
        return;

    if (kDebugPrintInTask) {
        std::printf("In doFFT TID %d: frame %zu, symbol %zu, ant_id %zu\n", tid,
            frame_id, symbol_id, ant_id);
    }

    size_t sig_offset = config_->ofdm_rx_zero_prefix_client();
    if (kPrintDownlinkPilotStats && config_->ue_ant_num() == 1) {
        if (config_->IsPilot(frame_id, symbol_id)) {
            simd_convert_short_to_float(pkt->data,
                reinterpret_cast<float*>(rx_samps_tmp),
                2 * config_->samps_per_symbol());
            std::vector<std::complex<float>> samples_vec(
                rx_samps_tmp, rx_samps_tmp + config_->samps_per_symbol());
            size_t seq_len = ue_pilot_vec[ant_id].size();
            std::vector<std::complex<float>> pilot_corr
                = CommsLib::correlate_avx(samples_vec, ue_pilot_vec[ant_id]);
            std::vector<float> pilot_corr_abs = CommsLib::abs2_avx(pilot_corr);
            size_t peak_offset
                = std::max_element(pilot_corr_abs.begin(), pilot_corr_abs.end())
                - pilot_corr_abs.begin();
            sig_offset = peak_offset < seq_len ? 0 : peak_offset - seq_len;
            float noise_power = 0;
            for (size_t i = 0; i < sig_offset; i++)
                noise_power += std::pow(std::abs(samples_vec[i]), 2);
            float signal_power = 0;
            for (size_t i = sig_offset; i < 2 * sig_offset; i++)
                signal_power += std::pow(std::abs(samples_vec[i]), 2);
            float SNR = 10 * std::log10(signal_power / noise_power);
            std::printf(
                "frame %zu symbol %zu ant %zu: sig offset %zu, SNR %2.1f \n",
                frame_id, symbol_id, ant_id, sig_offset, SNR);
            if (frame_id == kRecordFrameIndex) {
                std::string fname
                    = "rxpilot" + std::to_string(symbol_id) + ".bin";
                FILE* f = std::fopen(fname.c_str(), "wb");
                std::fwrite(pkt->data, 2 * sizeof(int16_t),
                    config_->samps_per_symbol(), f);
                std::fclose(f);
            }

        } else {
            if (frame_id == kRecordFrameIndex) {
                std::string fname
                    = "rxdata" + std::to_string(symbol_id) + ".bin";
                FILE* f = std::fopen(fname.c_str(), "wb");
                std::fwrite(pkt->data, 2 * sizeof(int16_t),
                    config_->samps_per_symbol(), f);
                std::fclose(f);
            }
        }
    }

    // remove CP, do FFT
    size_t dl_symbol_id = config_->GetDLSymbolIdx(frame_id, symbol_id);
    size_t total_dl_symbol_id = frame_slot * dl_symbol_perframe + dl_symbol_id;
    size_t FFT_buffer_target_id
        = total_dl_symbol_id * config_->ue_ant_num() + ant_id;

    // transfer ushort to float
    sig_offset = (sig_offset / 16) * 16;
    size_t delay_offset = (sig_offset + config_->cp_len()) * 2;
    float* fft_buff = (float*)fft_buffer_[FFT_buffer_target_id];

    simd_convert_short_to_float(
        &pkt->data[delay_offset], fft_buff, config_->ofdm_ca_num() * 2);

    // perform fft
    DftiComputeForward(mkl_handle, fft_buffer_[FFT_buffer_target_id]);

    size_t csi_offset = frame_slot * config_->ue_ant_num() + ant_id;
    cx_float* csi_buffer_ptr = (cx_float*)(csi_buffer_[csi_offset].data());
    cx_float* fft_buffer_ptr = (cx_float*)fft_buffer_[FFT_buffer_target_id];

    Event_data fft_finish_event;

    // In TDD massive MIMO, a pilot symbol needs to be sent
    // in the downlink for the user to estimate the channel
    // due to relative reciprocity calibration,
    // see Argos paper (Mobicom'12)
    if (dl_symbol_id < dl_pilot_symbol_perframe) {
        for (size_t j = 0; j < config_->ofdm_data_num(); j++) {
            // divide fft output by pilot data to get CSI estimation
            if (dl_symbol_id == 0) {
                csi_buffer_ptr[j] = 0;
            }
            complex_float p = config_->ue_specific_pilot()[ant_id][j];
            size_t sc_id = non_null_sc_ind_[j];
            csi_buffer_ptr[j] += (fft_buffer_ptr[sc_id] / cx_float(p.re, p.im));
            if (dl_symbol_id == dl_pilot_symbol_perframe - 1)
                csi_buffer_ptr[j] /= dl_pilot_symbol_perframe;
        }
    } else {
        size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe
            + dl_symbol_id - dl_pilot_symbol_perframe;
        size_t eq_buffer_offset
            = total_dl_symbol_id * config_->ue_ant_num() + ant_id;

        cx_float* equ_buffer_ptr
            = (cx_float*)(equal_buffer_[eq_buffer_offset].data());

        // use pilot subcarriers for phase tracking and correction
        float theta = 0;
        cx_float csi(1, 0);
        for (size_t j = 0; j < config_->ofdm_data_num(); j++) {
            if (j % config_->ofdm_pilot_spacing() == 0) {
                equ_buffer_ptr[j] = 0;
                if (dl_pilot_symbol_perframe > 0) {
                    csi = csi_buffer_ptr[j];
                }
                size_t sc_id = non_null_sc_ind_[j];
                cx_float y = fft_buffer_ptr[sc_id];
                auto pilot_eq = y / csi;
                auto p = config_->ue_specific_pilot()[ant_id][j];
                theta += arg(pilot_eq * cx_float(p.re, -p.im));
            }
        }
        if (config_->GetOFDMPilotNum() > 0)
            theta /= config_->GetOFDMPilotNum();
        auto phc = exp(cx_float(0, -theta));
        for (size_t j = 0; j < config_->ofdm_data_num(); j++) {
            if (j % config_->ofdm_pilot_spacing() != 0) {
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
                          ->dl_iq_f[dl_symbol_id][ant_id * config_->ofdm_ca_num()
                              + config_->ofdm_data_start() + j];
                evm += std::norm(equ_buffer_ptr[j] - cx_float(tx.re, tx.im));
		*/
            }
        }
        /*
        evm = std::sqrt(
            evm / (config_->ofdm_data_num() - config_->GetOFDMPilotNum()));
        if (kPrintPhyStats)
            std::cout << "Frame: " << frame_id << ", Symbol: " << symbol_id
                      << ", User: " << ant_id << ", EVM: " << 100 * evm
                      << "%, SNR: " << -10 * std::log10(evm) << std::endl;
        */
    }

    size_t fft_duration_stat = rdtsc() - start_tsc;
    if (kDebugPrintPerTaskDone)
        std::printf("FFT Duration (%zu, %zu, %zu): %2.4f us\n", frame_id,
            symbol_id, ant_id,
            cycles_to_us(fft_duration_stat, measure_rdtsc_freq()));

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
        std::printf("In doDemul TID %d: frame %zu, symbol %zu, ant_id %zu\n",
            tid, frame_id, symbol_id, ant_id);
    }
    size_t start_tsc = rdtsc();

    const size_t frame_slot = frame_id % kFrameWnd;
    size_t dl_symbol_id = config_->GetDLSymbolIdx(frame_id, symbol_id);
    size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe
        + dl_symbol_id - dl_pilot_symbol_perframe;
    size_t offset = total_dl_symbol_id * config_->ue_ant_num() + ant_id;
    float* equal_ptr = (float*)&equal_buffer_[offset][0];
    auto* demul_ptr = dl_demod_buffer_[offset];

    // demod_16qam_hard_loop(
    //    equal_ptr, (uint8_t*)demul_ptr, config_->ue_ant_num());

    switch (config_->mod_order_bits()) {
    case (CommsLib::QPSK):
        demod_qpsk_soft_sse(equal_ptr, demul_ptr, config_->ofdm_data_num());
        break;
    case (CommsLib::QAM16):
        demod_16qam_soft_avx2(equal_ptr, demul_ptr, config_->ofdm_data_num());
        break;
    case (CommsLib::QAM64):
        demod_64qam_soft_avx2(equal_ptr, demul_ptr, config_->ofdm_data_num());
        break;
    default:
        std::printf("Demodulation: modulation type %s not supported!\n",
            config_->modulation().c_str());
    }

    size_t dem_duration_stat = rdtsc() - start_tsc;
    if (kDebugPrintPerTaskDone)
        std::printf("Demodul Duration (%zu, %zu, %zu): %2.4f us\n", frame_id,
            symbol_id, ant_id,
            cycles_to_us(dem_duration_stat, measure_rdtsc_freq()));

    if (kPrintLLRData) {
        std::printf("LLR data, symbol_offset: %zu\n", offset);
        for (size_t i = 0; i < config_->ofdm_data_num(); i++) {
            std::printf("%x ", (uint8_t) * (demul_ptr + i));
        }
        std::printf("\n");
    }

    rt_assert(message_queue_.enqueue(
                  *task_ptok[tid], Event_data(EventType::kDemul, tag)),
        "Demodulation message enqueue failed");
}

void Phy_UE::doDecode(int tid, size_t tag)
{
    const LDPCconfig& LDPC_config = config_->ldpc_config();
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t ant_id = gen_tag_t(tag).ant_id;
    if (kDebugPrintInTask) {
        std::printf("In doDecode TID %d: frame %zu, symbol %zu, ant_id %zu\n",
            tid, frame_id, symbol_id, ant_id);
    }
    size_t start_tsc = rdtsc();

    const size_t frame_slot = frame_id % kFrameWnd;
    size_t dl_symbol_id = config_->GetDLSymbolIdx(frame_id, symbol_id);
    size_t total_dl_symbol_id = frame_slot * dl_data_symbol_perframe
        + dl_symbol_id - dl_pilot_symbol_perframe;
    size_t symbol_ant_offset
        = total_dl_symbol_id * config_->ue_ant_num() + ant_id;

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };

    // Decoder setup
    int16_t numFillerBits = 0;
    int16_t numChannelLlrs = LDPC_config.num_cb_codew_len();

    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = LDPC_config.max_decoder_iter();
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = LDPC_config.early_termination();
    ldpc_decoder_5gnr_request.Zc = LDPC_config.expansion_factor();
    ldpc_decoder_5gnr_request.baseGraph = LDPC_config.base_graph();
    ldpc_decoder_5gnr_request.nRows = LDPC_config.num_rows();

    int numMsgBits = LDPC_config.num_cb_len() - numFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

    size_t block_error(0);
    for (size_t cb_id = 0;
         cb_id < config_->ldpc_config().num_blocks_in_symbol(); cb_id++) {
        size_t demod_buffer_offset = cb_id * LDPC_config.num_cb_codew_len()
            * config_->mod_order_bits();
        size_t decode_buffer_offset
            = cb_id * roundup<64>(config_->num_bytes_per_cb());
        auto* llr_buffer_ptr
            = &dl_demod_buffer_[symbol_ant_offset][demod_buffer_offset];
        auto* decoded_buffer_ptr
            = &dl_decode_buffer_[symbol_ant_offset][decode_buffer_offset];
        ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
        ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;
        bblib_ldpc_decoder_5gnr(
            &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

        if (kCollectPhyStats) {
            decoded_bits_count_[ant_id][total_dl_symbol_id]
                += 8 * config_->num_bytes_per_cb();
            decoded_blocks_count_[ant_id][total_dl_symbol_id]++;
            size_t byte_error(0);
            for (size_t i = 0; i < config_->num_bytes_per_cb(); i++) {
                uint8_t rx_byte = decoded_buffer_ptr[i];
                uint8_t tx_byte = (uint8_t)config_->GetInfoBits(
                    config_->dl_bits(), dl_symbol_id, ant_id, cb_id)[i];
                uint8_t xor_byte(tx_byte ^ rx_byte);
                size_t bit_errors = 0;
                for (size_t j = 0; j < 8; j++) {
                    bit_errors += xor_byte & 1;
                    xor_byte >>= 1;
                }
                if (rx_byte != tx_byte)
                    byte_error++;

                bit_error_count_[ant_id][total_dl_symbol_id] += bit_errors;
            }
            block_error_count_[ant_id][total_dl_symbol_id] += (byte_error > 0);
            block_error += (byte_error > 0);
        }

        if (kPrintDecodedData) {
            std::printf("Decoded data (original byte)\n");
            for (size_t i = 0; i < config_->num_bytes_per_cb(); i++) {
                uint8_t rx_byte = decoded_buffer_ptr[i];
                uint8_t tx_byte = (uint8_t)config_->GetInfoBits(
                    config_->dl_bits(), dl_symbol_id, ant_id, cb_id)[i];
                std::printf("%x(%x) ", rx_byte, tx_byte);
            }
            std::printf("\n");
        }
    }
    if (kCollectPhyStats) {
        decoded_symbol_count_[ant_id]++;
        symbol_error_count_[ant_id] += (block_error > 0);
    }

    size_t dec_duration_stat = rdtsc() - start_tsc;
    if (kDebugPrintPerTaskDone)
        std::printf("Decode Duration (%zu, %zu, %zu): %2.4f us\n", frame_id,
            symbol_id, ant_id,
            cycles_to_us(dec_duration_stat, measure_rdtsc_freq()));

    rt_assert(message_queue_.enqueue(
                  *task_ptok[tid], Event_data(EventType::kDecode, tag)),
        "Decoding message enqueue failed");
}

//////////////////////////////////////////////////////////
//                   UPLINK Operations                //
//////////////////////////////////////////////////////////

void Phy_UE::doEncode(int tid, size_t tag)
{
    const LDPCconfig& LDPC_config = config_->ldpc_config();
    // size_t ue_id = rx_tag_t(tag).tid;
    // size_t offset = rx_tag_t(tag).offset;
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t ue_id = gen_tag_t(tag).ue_id;
    size_t frame_slot = frame_id % kFrameWnd;
    auto& cfg = config_;
    // size_t start_tsc = worker_rdtsc();

    int8_t* encoded_buffer_temp = static_cast<int8_t*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            ldpc_encoding_encoded_buf_size(cfg->ldpc_config().base_graph(),
                cfg->ldpc_config().expansion_factor())));
    int8_t* parity_buffer = static_cast<int8_t*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align,
            ldpc_encoding_parity_buf_size(cfg->ldpc_config().base_graph(),
                cfg->ldpc_config().expansion_factor())));

    size_t bytes_per_block = kEnableMac
        ? (LDPC_config.num_cb_len()) >> 3
        : roundup<64>(bits_to_bytes(LDPC_config.num_cb_len()));
    size_t encoded_bytes_per_block = (LDPC_config.num_cb_codew_len() + 7) >> 3;

    for (size_t ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe;
         ul_symbol_id++) {
        size_t total_ul_symbol_id
            = frame_slot * ul_data_symbol_perframe + ul_symbol_id;
        for (size_t cb_id = 0;
             cb_id < config_->ldpc_config().num_blocks_in_symbol(); cb_id++) {
            int8_t* input_ptr;
            if (kEnableMac) {
                uint8_t* ul_bits = ul_bits_buffer_[ue_id]
                    + frame_slot * config_->mac_bytes_num_perframe();

                int input_offset = bytes_per_block
                        * cfg->ldpc_config().num_blocks_in_symbol()
                        * ul_symbol_id
                    + bytes_per_block * cb_id;
                input_ptr = reinterpret_cast<int8_t*>(ul_bits + input_offset);
            } else {
                size_t cb_offset
                    = (ue_id * cfg->ldpc_config().num_blocks_in_symbol()
                          + cb_id)
                    * bytes_per_block;
                input_ptr = &cfg->ul_bits()[ul_symbol_id
                    + config_->frame().client_ul_pilot_symbols()][cb_offset];
            }

            ldpc_encode_helper(LDPC_config.base_graph(),
                LDPC_config.expansion_factor(), LDPC_config.num_rows(),
                encoded_buffer_temp, parity_buffer, input_ptr);

            int cbCodedBytes
                = LDPC_config.num_cb_codew_len() / cfg->mod_order_bits();
            int output_offset = total_ul_symbol_id * config_->ofdm_data_num()
                + cbCodedBytes * cb_id;

            adapt_bits_for_mod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
                &ul_syms_buffer_[ue_id][output_offset], encoded_bytes_per_block,
                cfg->mod_order_bits());
        }
    }
    // double duration = worker_rdtsc() - start_tsc;
    // if (cycles_to_us(duration, freq_ghz) > 500) {
    //    std::printf("Thread %d Encode takes %.2f\n", tid,
    //        cycles_to_us(duration, freq_ghz));
    //}

    std::free(encoded_buffer_temp);
    std::free(parity_buffer);

    rt_assert(message_queue_.enqueue(
                  *task_ptok[tid], Event_data(EventType::kEncode, tag)),
        "Encoding message enqueue failed");
}

void Phy_UE::doModul(int tid, size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t ue_id = gen_tag_t(tag).ue_id;
    const size_t frame_slot = frame_id % kFrameWnd;
    for (size_t ch = 0; ch < config_->num_channels(); ch++) {
        size_t ant_id = ue_id * config_->num_channels() + ch;
        for (size_t ul_symbol_id = 0; ul_symbol_id < ul_data_symbol_perframe;
             ul_symbol_id++) {
            size_t total_ul_symbol_id
                = frame_slot * ul_data_symbol_perframe + ul_symbol_id;
            complex_float* modul_buf = &modul_buffer_[total_ul_symbol_id][ant_id
                * config_->ofdm_data_num()];
            int8_t* ul_bits
                = (int8_t*)&ul_syms_buffer_[ant_id][total_ul_symbol_id
                    * config_->ofdm_data_num()];
            for (size_t sc = 0; sc < config_->ofdm_data_num(); sc++) {
                modul_buf[sc] = mod_single_uint8(
                    (uint8_t)ul_bits[sc], config_->mod_table());
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
    const size_t frame_slot = frame_id % kFrameWnd;
    const size_t ue_id = gen_tag_t(tag).ue_id;
    for (size_t ch = 0; ch < config_->num_channels(); ch++) {
        size_t ant_id = ue_id * config_->num_channels() + ch;
        for (size_t ul_symbol_id = 0; ul_symbol_id < ul_symbol_perframe;
             ul_symbol_id++) {

            size_t total_ul_symbol_id
                = frame_slot * ul_symbol_perframe + ul_symbol_id;
            size_t buff_offset
                = total_ul_symbol_id * config_->ue_ant_num() + ant_id;
            complex_float* ifft_buff = ifft_buffer_[buff_offset];

            std::memset(ifft_buff, 0,
                sizeof(complex_float) * config_->ofdm_data_start());
            if (ul_symbol_id < config_->frame().client_ul_pilot_symbols()) {
                std::memcpy(ifft_buff + config_->ofdm_data_start(),
                    config_->ue_specific_pilot()[ant_id],
                    config_->ofdm_data_num() * sizeof(complex_float));
            } else {
                size_t total_ul_data_symbol_id
                    = frame_slot * ul_data_symbol_perframe + ul_symbol_id
                    - config_->frame().client_ul_pilot_symbols();
                complex_float* modul_buff
                    = &modul_buffer_[total_ul_data_symbol_id]
                                    [ant_id * config_->ofdm_data_num()];
                std::memcpy(ifft_buff + config_->ofdm_data_start(), modul_buff,
                    config_->ofdm_data_num() * sizeof(complex_float));
            }
            std::memset(ifft_buff + config_->ofdm_data_stop(), 0,
                sizeof(complex_float) * config_->ofdm_data_start());

            CommsLib::IFFT(ifft_buff, config_->ofdm_ca_num(), false);

            size_t tx_offset = buff_offset * config_->packet_length();
            char* cur_tx_buffer = &tx_buffer_[tx_offset];
            struct Packet* pkt = (struct Packet*)cur_tx_buffer;
            std::complex<short>* tx_data_ptr = (std::complex<short>*)pkt->data;
            CommsLib::ifft2tx(ifft_buff, tx_data_ptr, config_->ofdm_ca_num(),
                config_->ofdm_tx_zero_prefix(), config_->cp_len(),
                config_->scale());
        }
    }

    rt_assert(message_queue_.enqueue(
                  *task_ptok[tid], Event_data(EventType::kIFFT, tag)),
        "Muliplexing message enqueue failed");
}

void Phy_UE::initialize_vars_from_cfg(void)
{
    dl_pilot_symbol_perframe = config_->frame().client_dl_pilot_symbols();
    ul_pilot_symbol_perframe = config_->frame().client_ul_pilot_symbols();
    ul_symbol_perframe = config_->frame().NumULSyms();
    dl_symbol_perframe = config_->frame().NumDLSyms();
    dl_data_symbol_perframe = dl_symbol_perframe - dl_pilot_symbol_perframe;
    ul_data_symbol_perframe = ul_symbol_perframe - ul_pilot_symbol_perframe;
    nCPUs = std::thread::hardware_concurrency();
    rx_thread_num = ((kUseArgos == true) && (config_->hw_framer() == false))
        ? config_->ue_num()
        : std::min(config_->ue_num(), config_->socket_thread_num());

    tx_buffer_status_size
        = (ul_symbol_perframe * config_->ue_ant_num() * kFrameWnd);
    tx_buffer_size = config_->packet_length() * tx_buffer_status_size;
    rx_buffer_status_size
        = (dl_symbol_perframe + config_->frame().NumBeaconSyms())
        * config_->ue_ant_num() * kFrameWnd;
    rx_buffer_size = config_->packet_length() * rx_buffer_status_size;
}

void Phy_UE::initialize_uplink_buffers()
{
    // initialize ul data buffer
    ul_bits_buffer_size_ = kFrameWnd * config_->mac_bytes_num_perframe();
    ul_bits_buffer_.malloc(config_->ue_ant_num(), ul_bits_buffer_size_,
        Agora_memory::Alignment_t::k64Align);
    ul_bits_buffer_status_.calloc(
        config_->ue_ant_num(), kFrameWnd, Agora_memory::Alignment_t::k64Align);
    ul_syms_buffer_size_
        = kFrameWnd * ul_data_symbol_perframe * config_->ofdm_data_num();
    ul_syms_buffer_.calloc(config_->ue_ant_num(), ul_syms_buffer_size_,
        Agora_memory::Alignment_t::k64Align);

    // initialize modulation buffer
    modul_buffer_.calloc(ul_data_symbol_perframe * kFrameWnd,
        config_->ofdm_data_num() * config_->ue_ant_num(),
        Agora_memory::Alignment_t::k64Align);

    // initialize IFFT buffer
    size_t ifft_buffer_block_num
        = config_->ue_ant_num() * ul_symbol_perframe * kFrameWnd;
    ifft_buffer_.calloc(ifft_buffer_block_num, config_->ofdm_ca_num(),
        Agora_memory::Alignment_t::k64Align);

    alloc_buffer_1d(
        &tx_buffer_, tx_buffer_size, Agora_memory::Alignment_t::k64Align, 0);
    alloc_buffer_1d(&tx_buffer_status_, tx_buffer_status_size,
        Agora_memory::Alignment_t::k64Align, 1);
}

void Phy_UE::initialize_downlink_buffers(void)
{
    // initialize rx buffer
    rx_buffer_.malloc(
        rx_thread_num, rx_buffer_size, Agora_memory::Alignment_t::k64Align);
    rx_buffer_status_.calloc(rx_thread_num, rx_buffer_status_size,
        Agora_memory::Alignment_t::k64Align);
    alloc_buffer_1d(&rx_samps_tmp, config_->samps_per_symbol(),
        Agora_memory::Alignment_t::k64Align, 1);

    // initialize FFT buffer
    size_t FFT_buffer_block_num
        = config_->ue_ant_num() * dl_symbol_perframe * kFrameWnd;
    fft_buffer_.calloc(FFT_buffer_block_num, config_->ofdm_ca_num(),
        Agora_memory::Alignment_t::k64Align);

    // initialize CSI buffer
    csi_buffer_.resize(config_->ue_ant_num() * kFrameWnd);
    for (size_t i = 0; i < csi_buffer_.size(); i++)
        csi_buffer_[i].resize(config_->ofdm_data_num());

    if (dl_data_symbol_perframe > 0) {
        // initialize equalized data buffer
        const size_t task_buffer_symbol_num_dl
            = dl_data_symbol_perframe * kFrameWnd;
        size_t buffer_size = config_->ue_ant_num() * task_buffer_symbol_num_dl;
        equal_buffer_.resize(buffer_size);
        for (size_t i = 0; i < equal_buffer_.size(); i++)
            equal_buffer_[i].resize(config_->ofdm_data_num());

        // initialize demod buffer
        dl_demod_buffer_.calloc(buffer_size,
            config_->ofdm_data_num() * kMaxModType,
            Agora_memory::Alignment_t::k64Align);

        // initialize decode buffer
        dl_decode_buffer_.resize(buffer_size);
        for (size_t i = 0; i < dl_decode_buffer_.size(); i++)
            dl_decode_buffer_[i].resize(roundup<64>(config_->num_bytes_per_cb())
                * config_->ldpc_config().num_blocks_in_symbol());
        resp_var_nodes
            = static_cast<int16_t*>(Agora_memory::padded_aligned_alloc(
                Agora_memory::Alignment_t::k64Align,
                1024 * 1024 * sizeof(int16_t)));

        decoded_bits_count_.calloc(config_->ue_ant_num(),
            task_buffer_symbol_num_dl, Agora_memory::Alignment_t::k64Align);
        bit_error_count_.calloc(config_->ue_ant_num(),
            task_buffer_symbol_num_dl, Agora_memory::Alignment_t::k64Align);

        decoded_blocks_count_.calloc(config_->ue_ant_num(),
            task_buffer_symbol_num_dl, Agora_memory::Alignment_t::k64Align);
        block_error_count_.calloc(config_->ue_ant_num(),
            task_buffer_symbol_num_dl, Agora_memory::Alignment_t::k64Align);
        decoded_symbol_count_ = new size_t[config_->ue_ant_num()];
        symbol_error_count_ = new size_t[config_->ue_ant_num()];
        std::memset(
            decoded_symbol_count_, 0, sizeof(size_t) * config_->ue_ant_num());
        std::memset(
            symbol_error_count_, 0, sizeof(size_t) * config_->ue_ant_num());
    }
}

void Phy_UE::getDemulData(long long** ptr, int* size)
{
    *ptr = (long long*)&equal_buffer_[max_equaled_frame
        * dl_data_symbol_perframe][0];
    *size = config_->ue_ant_num() * config_->ofdm_ca_num();
}

void Phy_UE::getEqualData(float** ptr, int* size, int ue_id)
{
    *ptr = (float*)&equal_buffer_[max_equaled_frame * dl_data_symbol_perframe
            * config_->ue_ant_num()
        + ue_id][0];
    *size = config_->ue_ant_num() * config_->ofdm_data_num() * 2;
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
