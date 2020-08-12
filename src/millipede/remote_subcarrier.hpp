/**
 * Author: Kevin Boos
 * Email: kevinaboos@gmail.com
 * 
 * @See `RemoteSubcarrier`.
 */

#ifndef REMOTE_SUBCARRIER_HPP
#define REMOTE_SUBCARRIER_HPP

#include "config.hpp"
#include "logger.h"
#include "phy_stats.hpp"
#include "signalHandler.hpp"
#include "stats.hpp"
#include "subcarrier_manager.hpp"
#include "Symbols.hpp"
#include <atomic>
#include <thread>

using mt_queue_t = moodycamel::ConcurrentQueue<Event_data>;


/**
 * A `RemoteSubcarrier` represents a worker process that contains 
 * one or more subcarrier doers (`DoSubcarrier` instances), which handle
 * subcarrier-parallel processing stages.
 * 
 * This is only used for running Millipede as a distributed, multi-process
 * application. This class does the following:
 *  - Sets up socket threads to receive packets (post-FFT).
 *  - Creates one or more `DoSubcarrier` instances, based on configuration. 
 *  - Creates event queues that allow the socket receiver threads to 
 *    communicate with the subcarrier doers. 
 *  - Establishes an endpoint for communicating with a health-monitoring 
 *    service and receiving re-configuration information. This only occurs
 *    when running within an environment like a kubernetes cluster. 
 *  - TODO: more features to come. 
 * 
 */
class RemoteSubcarrier {
public:

    /// Creates a new `RemoteSubcarrier`, which initializes its I/O sockets, 
    /// buffers, master thread, worker threads, and event queues.
    RemoteSubcarrier(Config* cfg)
        : running_(true)
        , cfg_(cfg)
        , freq_ghz_(measure_rdtsc_freq())
        , base_worker_core_offset(cfg_->core_offset + 1 + cfg_->socket_thread_num)
        , cur_tid(0)
    {
        MLPD_INFO("[RemoteSubcarrier]: Measured RDTSC frequency = %.2f\n",
            freq_ghz_);

        // Create queues and producer tokens for communication between
        // the master thread and each worker thread
        size_t queue_size = 512 * cfg_->data_symbol_num_perframe * 4;
        complete_task_queue_ = mt_queue_t(queue_size);
        for (size_t i = 0; i < cfg_->worker_thread_num; i++) {
            sched_info_arr[i].concurrent_q = mt_queue_t(queue_size);
            sched_info_arr[i].ptok
                = new moodycamel::ProducerToken(sched_info_arr[i].concurrent_q);
            worker_ptoks_ptr[i]
                = new moodycamel::ProducerToken(complete_task_queue_);
        }

        // Init uplink (and optionally) downlink buffers
        initialize_uplink_buffers();
        if (cfg_->dl_data_symbol_num_perframe > 0) {
            MLPD_INFO("[RemoteSubcarrier]: Initializing downlink buffers\n");
            initialize_downlink_buffers();
        }

        stats_ = new Stats(cfg, kMaxStatBreakdown, freq_ghz_);
        phy_stats_ = new PhyStats(cfg);


        // TODO: init sockets for receiving post-FFT data (into `data_buffer_`).


        // Create the subcarrier manager. This must be done before 
        // creating the worker threads, as worker threads will use information 
        // from the subcarrier manager to determine which subcarrier doers 
        // each worker thread should use and create.
        sc_mgr_ = new SubcarrierManager(this->cfg_, freq_ghz_, sched_info_arr,
            complete_task_queue_, csi_buffer_, recip_buffer_, calib_buffer_,
            dl_encoded_buffer_, data_buffer_, demod_soft_buffer_,
            dl_ifft_buffer_, phy_stats_, stats_); 


        // Spawn the master thread
        master_thread_ = std::thread([=] { master(); } );
        // Spawn the worker threads
        for (size_t tid = 0; tid < cfg_->worker_thread_num; tid++) {
            worker_threads_[tid] = std::thread([=] { worker(tid); } );
        }
    }

    
    /// The destructor does the following:
    ///  - halts all threads and waits for them to complete
    ///  - cleans up (deallocates) all buffers
    ~RemoteSubcarrier() {

        // stop master and worker threads
        running_ = false;
        MLPD_INFO("[~RemoteSubcarrier]: exiting. Joining master thread...\n");
        master_thread_.join();
        MLPD_INFO("[~RemoteSubcarrier]: exiting. Joining worker threads...\n");
        for (std::thread& wt : worker_threads_) {
            if (wt.joinable()) {
                wt.join();
            }
        }
        MLPD_INFO("[~RemoteSubcarrier]: threads have completed.\n");

    }

    /// Stops this `RemoteSubcarrier`'s master and worker threads 
    /// in an asynchronous, non-blocking way. 
    /// There is no corresponding `start()` method because all threads
    /// are created and started when creating this `RemoteSubcarrier`.
    void stop() {
        running_ = false;
    }

private: // private methods

    /// The entry point for the master thread.  
    void master() {
        std::cout << "In master thread." << std::endl;
        pin_to_core_with_offset(ThreadType::kMaster, cfg_->core_offset, 0);

        // The ID of the frame currently being processed.
        // Millipede processes a given frame only after all processing for
        // previous frames has completed. 
        size_t cur_frame_id = 0;

        /* Counters for printing summary */
        size_t demul_count = 0;
        size_t tx_count = 0;
        double demul_begin = get_time_us();
        double tx_begin = get_time_us();

        bool is_turn_to_dequeue_from_io = true;
        const size_t max_events_needed = std::max(kDequeueBulkSizeTXRX
                * (cfg_->socket_thread_num + cfg_->mac_socket_thread_num),
            kDequeueBulkSizeWorker * cfg_->worker_thread_num);
        Event_data events_list[max_events_needed];

        while (running_ && !SignalHandler::gotExitSignal()) {
            // Get a batch of events
            size_t num_events_dequeued = 0;
            // TODO: FIXME: add this back in when receiving post-FFT data.
            //
            // if (is_turn_to_dequeue_from_io) {
            //     for (size_t i = 0;
            //         i < cfg_->socket_thread_num + cfg_->mac_socket_thread_num; i++) {
            //         num_events_dequeued += message_queue_.try_dequeue_bulk_from_producer(
            //             *(rx_ptoks_ptr[i]), events_list + num_events_dequeued,
            //             kDequeueBulkSizeTXRX);
            //     }
            // } else {
                for (size_t tid = 0; tid < cfg_->worker_thread_num; tid++) {
                    num_events_dequeued
                        += complete_task_queue_.try_dequeue_bulk_from_producer(
                            *(worker_ptoks_ptr[tid]), events_list + num_events_dequeued,
                            kDequeueBulkSizeWorker);
                }
            // }
            // is_turn_to_dequeue_from_io = !is_turn_to_dequeue_from_io;

            // Handle each event that we dequeued
            for (size_t ev_i = 0; ev_i < num_events_dequeued; ev_i++) {
                Event_data& event = events_list[ev_i];

                // FFT processing is scheduled after falling through the switch
                switch (event.event_type) {
                case EventType::kPacketRX: {
                    rt_assert(false, "kPacketRX event should not occur.");
                } break;

                case EventType::kFFT: {
                    for (size_t i = 0; i < event.num_tags; i++) {
                        handle_event_fft(event.tags[i]);
                    }
                } break;

                case EventType::kRC: {
                    size_t frame_id = event.tags[0];
                    stats_->master_set_tsc(TsType::kRCDone, frame_id);
                    // print_per_frame_done(PrintType::kRC, frame_id);
                    fft_stats_.symbol_rc_count[frame_id % TASK_BUFFER_FRAME_NUM]
                        = 0;
                    rc_stats_.last_frame = frame_id;
                } break;

                // Zeroforcing is complete. For downlink, schedule demodulation.
                // For uplink,  
                case EventType::kZF: {
                    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                    // print_per_task_done(PrintType::kZF, frame_id, 0,
                    //     zf_stats_.get_symbol_count(frame_id));
                    if (zf_stats_.last_symbol(frame_id)) {
                        stats_->master_set_tsc(TsType::kZFDone, frame_id);
                        zf_stats_.coded_frame = frame_id;
                        // print_per_frame_done(PrintType::kZF, frame_id);

                        /* If all the data in a frame has arrived when ZF is done */
                        for (size_t i = 0; i < cfg_->ul_data_symbol_num_perframe;
                            i++) {
                            if (fft_stats_.cur_frame_for_symbol[i] == frame_id) {
                                sc_mgr_->schedule_subcarriers(
                                    EventType::kDemul, frame_id, i);
                            }
                        }

                        if (cfg_->dl_data_symbol_num_perframe > 0) {
                            // If downlink data transmission is enabled, schedule
                            // downlink encoding for the first data symbol
                            schedule_codeblocks(EventType::kEncode, frame_id,
                                cfg_->dl_data_symbol_start);
                        }
                    }
                } break;

                // Demodulation is complete, time to send the 
                // `demod_soft_buffer` to DoDecode or an LDPC worker.
                case EventType::kDemul: {
                    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                    size_t symbol_idx_ul = gen_tag_t(event.tags[0]).symbol_id;
                    size_t base_sc_id = gen_tag_t(event.tags[0]).sc_id;

                    // print_per_task_done(
                    //     PrintType::kDemul, frame_id, symbol_idx_ul, base_sc_id);
                    /* If this symbol is ready */
                    if (demul_stats_.last_task(frame_id, symbol_idx_ul)) {
                        schedule_codeblocks(
                            EventType::kDecode, frame_id, symbol_idx_ul);
                        // print_per_symbol_done(
                        //     PrintType::kDemul, frame_id, symbol_idx_ul);
                        if (demul_stats_.last_symbol(frame_id)) {
                            stats_->master_set_tsc(TsType::kDemulDone, frame_id);
                            // print_per_frame_done(PrintType::kDemul, frame_id);
                        }

                        demul_count++;
                        /*
                        if (demul_count == demul_stats_.max_symbol_count * 9000) {
                            demul_count = 0;
                            double diff = get_time_us() - demul_begin;
                            int samples_num_per_UE = cfg_->OFDM_DATA_NUM
                                * demul_stats_.max_symbol_count * 1000;
                            printf(
                                "Frame %zu: RX %d samples (per-client) from %zu "
                                "clients in %f secs, throughtput %f bps per-client "
                                "(16QAM), current task queue length %zu\n",
                                frame_id, samples_num_per_UE, cfg_->UE_NUM, diff,
                                samples_num_per_UE * log2(16.0f) / diff,
                                get_conq(EventType::kFFT)->size_approx());
                            demul_begin = get_time_us();
                        }
                        */
                    }
                } break;

                case EventType::kDecode: {
                    rt_assert(false, "kDecode event should not occur.");
                } break;

                case EventType::kPacketToMac: {
                    rt_assert(false, "kPacketToMac event should not occur.");
                } break;

                case EventType::kEncode: {
                    rt_assert(false, "kEncode event should not occur.");
                    // TODO: use the original logic from this case 
                    // to schedule precoding after encoded data is received.
                } break;

                case EventType::kPrecode: {
                    /* Precoding is done, schedule ifft */
                    size_t sc_id = gen_tag_t(event.tags[0]).sc_id;
                    size_t frame_id = gen_tag_t(event.tags[0]).frame_id;
                    size_t data_symbol_idx = gen_tag_t(event.tags[0]).symbol_id;
                    // print_per_task_done(
                    //     PrintType::kPrecode, frame_id, data_symbol_idx, sc_id);
                    if (precode_stats_.last_task(frame_id, data_symbol_idx)) {
                        schedule_antennas(
                            EventType::kIFFT, frame_id, data_symbol_idx);
                        if (data_symbol_idx < cfg_->dl_data_symbol_end - 1) {
                            schedule_codeblocks(
                                EventType::kEncode, frame_id, data_symbol_idx + 1);
                        }

                        // print_per_symbol_done(
                        //     PrintType::kPrecode, frame_id, data_symbol_idx);
                        if (precode_stats_.last_symbol(frame_id)) {
                            stats_->master_set_tsc(TsType::kPrecodeDone, frame_id);
                            // print_per_frame_done(PrintType::kPrecode, frame_id);
                        }
                    }
                } break;

                case EventType::kIFFT: {
                    rt_assert(false, "kIFFT event should not occur.");
                } break;

                case EventType::kPacketTX: {
                    rt_assert(false, "kPacketTX event should not occur.");
                } break;

                default:
                    MLPD_ERROR("Wrong event type %d in message queue!", 
                        static_cast<int>(event.event_type));
                    // exit(0);
                }
            } // end of for loop
        } // end of while loop

        running_ = false;
        MLPD_WARN("[~RemoteSubcarrier]: exiting master thread.\n");
    }


    /// This is invoked after an FFT event is finished, e.g.,
    /// when it's time to do uplink stages like zeroforcing or demodulation 
    /// or downlink stages like precoding.
    void handle_event_fft(size_t tag)
    {
        size_t frame_id = gen_tag_t(tag).frame_id;
        size_t symbol_id = gen_tag_t(tag).symbol_id;
        SymbolType sym_type = cfg_->get_symbol_type(frame_id, symbol_id);

        if (sym_type == SymbolType::kPilot) {
            if (fft_stats_.last_task(frame_id, symbol_id)) {
                // print_per_symbol_done(PrintType::kFFTPilots, frame_id, symbol_id);
                if (!cfg_->downlink_mode
                    || (cfg_->downlink_mode && !cfg_->recipCalEn)
                    || (cfg_->downlink_mode && cfg_->recipCalEn
                        && rc_stats_.last_frame == frame_id)) {
                    /* If CSI of all UEs is ready, schedule ZF/prediction */
                    if (fft_stats_.last_symbol(frame_id)) {
                        stats_->master_set_tsc(TsType::kFFTDone, frame_id);
                        // print_per_frame_done(PrintType::kFFTPilots, frame_id);
                        if (kPrintPhyStats)
                            phy_stats_->print_snr_stats(frame_id);
                        sc_mgr_->schedule_subcarriers(
                            EventType::kZF, frame_id, 0);
                    }
                }
            }
        } else if (sym_type == SymbolType::kUL) {
            if (fft_stats_.last_task(frame_id, symbol_id)) {
                size_t symbol_idx_ul
                    = cfg_->get_ul_symbol_idx(frame_id, symbol_id);
                fft_stats_.cur_frame_for_symbol[symbol_idx_ul] = frame_id;
                // print_per_symbol_done(PrintType::kFFTData, frame_id, symbol_id);
                /* If precoder exist, schedule demodulation */
                if (zf_stats_.coded_frame == frame_id) {
                    sc_mgr_->schedule_subcarriers(
                        EventType::kDemul, frame_id, symbol_idx_ul);
                }
            }
        } else if (sym_type == SymbolType::kCalDL
            or sym_type == SymbolType::kCalUL) 
        {
            // print_per_symbol_done(PrintType::kFFTCal, frame_id, symbol_id);
            if (++fft_stats_.symbol_rc_count[frame_id % TASK_BUFFER_FRAME_NUM]
                == fft_stats_.max_symbol_rc_count) {
                // print_per_frame_done(PrintType::kFFTCal, frame_id);
                // TODO: rc_stats_.max_task_count appears uninitalized
                // schedule_task_set(EventType::kRC, rc_stats_.max_task_count,
                // frame_id, get_conq(EventType::kRC), get_ptok(EventType::kRC));
                schedule_task_set(
                    EventType::kRC, rc_stats_.max_task_count, frame_id);
            }
        }
    }

    void schedule_codeblocks(
        EventType event_type, size_t frame_id, size_t symbol_id)
    {
        assert(event_type == EventType::kEncode 
            or event_type == EventType::kDecode);
        auto base_tag = gen_tag_t::frm_sym_cb(frame_id, symbol_id, 0);

        std::cout << "[schedule_codeblocks]: TODO: send event to LDPC worker, "
                  << ", event_type: " << static_cast<int>(event_type)
                  << ", frame_id: " << frame_id
                  << ", symbol_id: " << symbol_id
                  << std::endl;

    }


    void schedule_antennas(
        EventType event_type, size_t frame_id, size_t symbol_id)
    {
        assert(event_type == EventType::kIFFT);
        auto base_tag = gen_tag_t::frm_sym_ant(frame_id, symbol_id, 0);

        std::cout << "[schedule_codeblocks]: TODO: send event to kIFFT worker, "
                  << ", event_type: " << static_cast<int>(event_type)
                  << ", frame_id: " << frame_id
                  << ", symbol_id: " << symbol_id
                  << std::endl;

    }

    
    void schedule_task_set(
        EventType task_type, int task_set_size, int task_set_id)
    {
        Event_data task(task_type, task_set_size * task_set_id);
        for (int i = 0; i < task_set_size; i++) {
            try_enqueue_fallback(&sched_info_arr[cur_tid].concurrent_q,
                sched_info_arr[cur_tid].ptok, task);
            cur_tid = (cur_tid + 1) % cfg_->worker_thread_num;
            task.tags[0]++;
        }
    }


    /// The entry point for each worker thread.  
    void worker(size_t tid) {
        std::cout << "In worker thread " << tid << "." << std::endl;

        pin_to_core_with_offset(
            ThreadType::kWorker, base_worker_core_offset, tid);

        mt_queue_t& worker_queue_in = sched_info_arr[tid].concurrent_q;
        moodycamel::ProducerToken*& worker_ptok = worker_ptoks_ptr[tid];

        // TODO: FIXME: Reciprocity is not currently supported, 
        //              as it's not needed for simulation. TBD later.
        /// TODO: move this into the subcarrier manager/doer infrastructure.
        ///       It's currently in the SubcarrierDoer but yet used there.
        // auto* computeReciprocity = new Reciprocity(cfg_, tid, freq_ghz_,
        //     *get_conq(EventType::kRC), complete_task_queue_,
        //     worker_ptok, calib_buffer_, recip_buffer_, stats_);

        // Allocate subcarrier Doers for subcarrier ranges this worker handles
        std::vector<Doer*> sc_doers_vec(sc_mgr_->num_subcarrier_ranges());
        for (auto& range : sc_mgr_->get_subcarrier_ranges_for_worker_tid(tid)) {
            size_t doer_idx = sc_mgr_->index_for_subcarrier_id(range.start);
            sc_doers_vec[doer_idx] = sc_mgr_->create_subcarrier_doer(
                tid, worker_ptok, range);

            MLPD_INFO("Worker thread %d created DoSubcarrier for range %s at "
                    "worker Doer index %zu\n",
                tid, range.to_string().c_str(), doer_idx);
        }

        // Enter the main event handling loop for this worker thread.
        while (running_) {
            Event_data req_event;
            if (worker_queue_in.try_dequeue(req_event)) {
                Event_data resp_event;
                resp_event.num_tags = req_event.num_tags;
                Doer* doer = nullptr;

                switch (req_event.event_type) {
                case EventType::kRC:
                    rt_assert(false, "TODO: Reciprocity isn't yet supported");
                    // doer = computeReciprocity;
                    break;
                case EventType::kZF:
                case EventType::kDemul:
                case EventType::kPrecode:
                    doer = sc_doers_vec[sc_mgr_->index_for_subcarrier_id(
                        gen_tag_t(req_event.tags[0]).sc_id)];
                    break;
                default:
                    assert(false);
                }

                for (size_t i = 0; i < req_event.num_tags; i++) {
                    Event_data resp_i
                        = doer->launch(req_event.tags[i], req_event.event_type);
                    rt_assert(resp_i.num_tags == 1, "Invalid num_tags in resp");
                    resp_event.tags[i] = resp_i.tags[0];
                    resp_event.event_type = resp_i.event_type;
                }

                try_enqueue_fallback(
                    &complete_task_queue_, worker_ptok, resp_event);

                continue;
            }
        } // End of worker thread event loop

        running_ = false;
        MLPD_WARN("[RemoteSubcarrier]: exiting worker thread %zu\n", tid);
    }

    void initialize_uplink_buffers()
    {
        MLPD_INFO("[RemoteSubcarrier]: Initializing uplink buffers.\n");
        const size_t task_buffer_symbol_num_ul
            = cfg_->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

        csi_buffer_.malloc(
            cfg_->pilot_symbol_num_perframe * TASK_BUFFER_FRAME_NUM,
            cfg_->BS_ANT_NUM * cfg_->OFDM_DATA_NUM, 64);
        data_buffer_.malloc(task_buffer_symbol_num_ul,
            cfg_->OFDM_DATA_NUM * cfg_->BS_ANT_NUM, 64);
        demod_soft_buffer_.malloc(task_buffer_symbol_num_ul,
            cfg_->mod_type * cfg_->OFDM_DATA_NUM * cfg_->UE_NUM, 64);

        fft_stats_.init(cfg_->BS_ANT_NUM, cfg_->pilot_symbol_num_perframe,
            cfg_->symbol_num_perframe);
        fft_stats_.max_symbol_data_count = cfg_->ul_data_symbol_num_perframe;
        fft_stats_.symbol_rc_count.fill(0);
        fft_stats_.max_symbol_rc_count = cfg_->BS_ANT_NUM;
        fft_stats_.cur_frame_for_symbol
            = std::vector<size_t>(cfg_->ul_data_symbol_num_perframe, SIZE_MAX);

        zf_stats_.init(cfg_->zf_events_per_symbol);

        demul_stats_.init(cfg_->demul_events_per_symbol,
            cfg_->ul_data_symbol_num_perframe, cfg_->data_symbol_num_perframe);

        // TODO: FIXME: rc_stats_ is never initialized, even in Millipede.
    }

    void initialize_downlink_buffers()
    {
        const size_t task_buffer_symbol_num
            = cfg_->data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

        dl_ifft_buffer_.calloc(
            cfg_->BS_ANT_NUM * task_buffer_symbol_num, cfg_->OFDM_CA_NUM, 64);
        recip_buffer_.calloc(
            TASK_BUFFER_FRAME_NUM, cfg_->OFDM_DATA_NUM * cfg_->BS_ANT_NUM, 64);
        calib_buffer_.calloc(
            TASK_BUFFER_FRAME_NUM, cfg_->OFDM_DATA_NUM * cfg_->BS_ANT_NUM, 64);
        dl_encoded_buffer_.calloc(
            task_buffer_symbol_num, cfg_->OFDM_DATA_NUM * cfg_->UE_NUM, 64);

        precode_stats_.init(cfg_->demul_events_per_symbol,
            cfg_->dl_data_symbol_num_perframe, cfg_->data_symbol_num_perframe);
    }

    void free_uplink_buffers()
    {
        csi_buffer_.free();
        data_buffer_.free();
        demod_soft_buffer_.free();

        fft_stats_.fini();
        // note: zf_stats_ doesn't require freeing.
        demul_stats_.fini();
        // TODO: FIXME: rc_stats_ is never initialized, even in Millipede.
    }

    void free_downlink_buffers()
    {
        dl_ifft_buffer_.free();
        recip_buffer_.free();
        calib_buffer_.free();
        dl_encoded_buffer_.free();

        precode_stats_.fini();
    }


private: // private fields

    /// Whether this process is still running. 
    /// This is used as a signal to inform master & worker threads to exit. 
    std::atomic_bool running_;
    /// The master thread.
    std::thread master_thread_;
    /// The sparse list of worker threads.
    std::array<std::thread, kMaxThreads> worker_threads_;

    static const int kDequeueBulkSizeTXRX = 8;
    static const int kDequeueBulkSizeWorker = 4;

    Config* cfg_;
    Stats* stats_;
    PhyStats* phy_stats_;
    /// RDTSC frequency in GHz
    const double freq_ghz_;
    /// Worker thread `i` runs on core `base_worker_core_offset + i`.
    const size_t base_worker_core_offset;
    
    /// TODO: (junzhi): Add documentation
    size_t cur_tid; 

    FFT_stats fft_stats_;
    ZF_stats zf_stats_;
    RC_stats rc_stats_;
    Data_stats demul_stats_;
    Data_stats precode_stats_;


    sched_info_t sched_info_arr[kMaxThreads];
    /// The MPSC message queue of task completion events 
    /// from all `Doer`s on worker threads to the master thread.
    moodycamel::ConcurrentQueue<Event_data> complete_task_queue_;
    /// The array of worker threads' producer tokens, used to push 
    /// task completion events onto the `complete_task_queue_`.
    moodycamel::ProducerToken* worker_ptoks_ptr[kMaxThreads];

    /// The subcarrier manager owns buffers for subcarrier-parallel tasks,
    /// and coordinates scheduling of these tasks.
    SubcarrierManager* sc_mgr_;


    ///////////////////////////////////////////////////
    ////////////////// Input Buffers //////////////////
    ///////////////////////////////////////////////////

    /// An input buffer of channel state information (CSI),
    /// an output of the FFT stage.
    /// TODO: eventually we will move this into `DoSubcarrier`,
    ///       so it can be encapsulated entirely within the SubcarrierManager.
    Table<complex_float> csi_buffer_;

    /// TODO: add documentation here.
    ///       I'm not 100% sure what this is used for, it's an input into doZF.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM
    /// @li 2nd dimension: number of OFDM data subcarriers * number of antennas
    Table<complex_float> recip_buffer_;

    /// 1st dimension: TASK_BUFFER_FRAME_NUM
    /// 2nd dimension: number of OFDM data subcarriers * number of antennas
    Table<complex_float> calib_buffer_;

    /// The main input buffer for subcarrier processing stages,
    /// which holds the data symbols after the FFT stage.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of antennas * number of OFDM data subcarriers
    ///
    /// The 2nd dimension's data order: 32 blocks each with 32 subcarriers each:
    /// subcarrier 1 -- 32 of antennas, subcarrier 33 -- 64 of antennas, ...,
    /// subcarrier 993 -- 1024 of antennas.
    Table<complex_float> data_buffer_;

    /// An input buffer of encoded data coming from the encoder (e.g., LDPC),
    /// which is used only during downlink.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * num data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<int8_t> dl_encoded_buffer_;

    ///////////////////////////////////////////////////
    ////////////////// Output Buffers /////////////////
    ///////////////////////////////////////////////////

    /// The main output buffer, which comes from the soft demodulation stage.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<int8_t> demod_soft_buffer_;

    /// An output buffer holding data destined for IFFT, only used in downlink.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of antennas
    ///                    * number of data symbols per frame
    /// @li 2nd dimension: number of OFDM carriers (including non-data carriers)
    Table<complex_float> dl_ifft_buffer_;

};

#endif // REMOTE_SUBCARRIER_HPP
