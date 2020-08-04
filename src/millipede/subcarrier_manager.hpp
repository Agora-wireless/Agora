/// Author: Kevin Boos
/// Email: kevinaboos@gmail.com
///
/// @see SubcarrierManager

#ifndef SUBCARRIER_MANAGER_HPP
#define SUBCARRIER_MANAGER_HPP

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "dosubcarrier.hpp"
#include "gettime.h"
#include "modulation.hpp"
#include "phy_stats.hpp"
#include "stats.hpp"
#include "utils.h"
#include <iostream>
#include <mutex>
#include <numeric>
#include <vector>

using mt_queue_t = moodycamel::ConcurrentQueue<Event_data>;

/**
 * @brief The singleton manager of all `DoSubcarrier` instances.
 *
 * This class is essentially a wrapper around the `DoSubcarrier` instances, aka
 * subcarrier doers, and coordinates scheduling of tasks among them.
 * It owns the buffers shared by all `DoSubcarrier` instances and exposes
 * interfaces to communicate with them, e.g., by using event queues.
 *
 * There is at least one `DoSubcarrier` instance per subcarrier range,
 * typically just one but potentially more redundant replicates for purposes
 * of scalability and reliability.
 * This class is responsible for owning the internal buffers that are used by
 * the subcarrier workers, and serves as a wrapper interface to export
 * subcarrier-parallel functionality.
 */
class SubcarrierManager {
public:
    SubcarrierManager(Config* config, double freq_ghz,
        sched_info_t (&sched_info_arr)[kMaxThreads],
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        // input buffers
        Table<complex_float>& csi_buffer, Table<complex_float>& recip_buffer,
        Table<complex_float>& calib_buffer, Table<int8_t>& dl_encoded_buffer,
        Table<complex_float>& data_buffer,
        // output buffers
        Table<int8_t>& demod_soft_buffer, Table<complex_float>& dl_ifft_buffer,
        PhyStats* phy_stats, Stats* stats)
        : cfg(config)
        , freq_ghz_(freq_ghz)
        , phy_stats_(phy_stats)
        , stats_(stats)
        , sched_info_arr_(sched_info_arr)
        , complete_task_queue_(complete_task_queue)
        , subcarrier_block_size_(lcm(cfg->demul_block_size, cfg->zf_block_size))
        , csi_buffer_(csi_buffer)
        , recip_buffer_(recip_buffer)
        , calib_buffer_(calib_buffer)
        , data_buffer_(data_buffer)
        , dl_encoded_buffer_(dl_encoded_buffer)
        , demod_soft_buffer_(demod_soft_buffer)
        , dl_ifft_buffer_(dl_ifft_buffer)
    {
        const size_t task_buffer_symbol_num_ul
            = cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

        // Create the various buffers owned by the subcarrier manager.
        // These buffers are shared across all subcarrier doers.
        //
        // TODO: Currently, these buffers are created to be huge enough to store
        // data for *ALL* subcarrier ranges, system-wide. We may want to scale
        // them down to be sufficient for just a single subcarrier range, but
        // then we'd need to perform some kind of "gather" operation at the end
        // before handing them off to the encoder stage. Such a "gather"
        // operation will be required when

        ue_spec_pilot_buffer_.calloc(
            TASK_BUFFER_FRAME_NUM, cfg->UL_PILOT_SYMS * cfg->UE_NUM, 64);
        equal_buffer_.malloc(
            task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        ul_zf_buffer_.malloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM,
            cfg->BS_ANT_NUM * cfg->UE_NUM, 64);
        dl_zf_buffer_.calloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM,
            cfg->UE_NUM * cfg->BS_ANT_NUM, 64);

        // Fill `subcarrier_doers_` with the total amount of (uninit'd) doers
        // so that we can insert the real subcarrier doers at the correct index.
        MLPD_INFO("[SubcarrierManager]: subcarrier block size is %zu, "
                  "requires %zu subcarrier doers.",
            subcarrier_block_size_, num_subcarrier_ranges());
        subcarrier_doers_.reserve(num_subcarrier_ranges());
        for (size_t i = 0; i < num_subcarrier_ranges(); i++) {
            subcarrier_doers_.emplace_back(
                std::make_pair(sched_info_t(), nullptr));
        }

        // Calculate the mapping from worker tid to subcarrier doer instance.
        for (size_t sc_base = 0; sc_base < cfg->OFDM_DATA_NUM;
             sc_base += subcarrier_block_size_) {
            Range range_to_insert(sc_base, sc_base + subcarrier_block_size_);
            size_t tid = worker_tid_for_subcarrier(sc_base);
            worker_tid_to_subcarrier_range_[tid].emplace_back(range_to_insert);
        }

        MLPD_INFO(
            "SubcarrierManager: Map of %zu worker threads to SC ranges:\n",
            cfg->worker_thread_num);
        for (size_t tid = 0; tid < cfg->worker_thread_num; tid++) {
            std::stringstream debug_map_string;
            debug_map_string << "Worker " << tid << ": {";
            for (auto& range : worker_tid_to_subcarrier_range_[tid]) {
                debug_map_string << range.to_string() << ", ";
            }
            debug_map_string << "}";
            MLPD_INFO("%s\n", debug_map_string.str().c_str());
        }

        // NOTE: In the future (for remote subcarrier doers), we'll create the
        // subcarrier doer instances here. However, currently with dynamic
        // thread scheduling, we let the worker threads create the subcarrier
        // instances themselves.
        /*
        for (size_t sc_base = 0; sc_base < cfg->OFDM_DATA_NUM;
             sc_base += subcarrier_block_size_) {
            auto computeSubcarrier = new DoSubcarrier(cfg, subcarrier_doer_tid,
                freq_ghz, *get_conq(EventType::kSubcarrier),
                complete_task_queue_, worker_ptoks_ptr[tid], sc_base,
                subcarrier_block_size_, csi_buffer_, recip_buffer_,
                calib_buffer_, dl_encoded_buffer_, data_buffer_,
                demod_soft_buffer_, dl_ifft_buffer_, ue_spec_pilot_buffer_,
                equal_buffer_, ul_zf_buffer_, dl_zf_buffer_, phy_stats, stats);
            subcarrier_doers_.push_back(computeSubcarrier);
        }
        */
    }

    ~SubcarrierManager()
    {
        // First, free all of the buffers we own.
        equal_buffer_.free();
        ue_spec_pilot_buffer_.free();
        ul_zf_buffer_.free();
        dl_zf_buffer_.free();

        // Destroy all of the Subcarrier doers
        for (std::pair<sched_info_t, DoSubcarrier*>& dsc : subcarrier_doers_) {
            // The queue (dsc.first) is auto-freed
            delete dsc.second;
        }
    }

    /// Schedules a given subcarrier-parallel processing event or set of events
    /// by assigning those events to the proper subcarrier doer.
    void schedule_subcarriers(
        EventType event_type, size_t frame_id, size_t symbol_id)
    {
        auto base_tag = gen_tag_t::frm_sym_sc(frame_id, symbol_id, 0);
        size_t num_events = SIZE_MAX;
        size_t block_size = SIZE_MAX;

        switch (event_type) {
        case EventType::kDemul:
        case EventType::kPrecode:
            num_events = cfg->demul_events_per_symbol;
            block_size = cfg->demul_block_size;
            break;
        case EventType::kZF:
            num_events = cfg->zf_events_per_symbol;
            block_size = cfg->zf_block_size;
            break;
        default:
            assert(false);
        }

        for (size_t i = 0; i < num_events; i++) {
            mt_queue_t* destination_queue;
            moodycamel::ProducerToken* ptok;

            if (kDedicatedSubcarrierDoerQueues) {
                // Enqueue the task onto the queue for the subcarrier doer
                // that handles the `sc_id` in `base_tag`.
                sched_info_t& master_to_worker_queue
                    = subcarrier_doer_for_id(base_tag.sc_id).first;
                destination_queue = &master_to_worker_queue.concurrent_q;
                ptok = master_to_worker_queue.ptok;
            } else {
                // Enqueue the task onto the queue for the worker thread
                // that contains the subcarrier doer that handles the `sc_id`.
                size_t tid = worker_tid_for_subcarrier(base_tag.sc_id);
                destination_queue = &sched_info_arr_[tid].concurrent_q;
                ptok = sched_info_arr_[tid].ptok;

                MLPD_TRACE("Schedule subcarrier: event type %zu, base "
                           "subcarrier ID %u, enqueuing onto worker TID %zu\n",
                    static_cast<size_t>(event_type), base_tag.sc_id, tid);
            }

            try_enqueue_fallback(
                destination_queue, ptok, Event_data(event_type, base_tag._tag));
            base_tag.sc_id += block_size;
        }
    }

    /// Creates a subcarrier doer to be run on the given worker thread
    /// and returns a pointer to that new doer.
    ///
    /// This function also creates an event queue for each subcarrier doer
    /// such that this subcarrier manager can schedule tasks for those doers.
    DoSubcarrier* create_subcarrier_doer(int worker_tid,
        moodycamel::ProducerToken* worker_producer_token, Range sc_range)
    {
        rt_assert((sc_range.start) / subcarrier_block_size_
                == (sc_range.end - 1) / subcarrier_block_size_,
            "Invalid subcarrier range, all subcarrier IDs within the range "
            "do not map to the same index into the subcarrier doers list");

        std::lock_guard<std::mutex> lock(subcarrier_doers_mutex_);

        MLPD_INFO("SubcarrierManager [create_subcarrier_doer]: worker_tid %d, "
                  "range %s, worker_producer_token %p\n",
            worker_tid, sc_range.to_string().c_str(), worker_producer_token);

        // Not sure how large this queue needs to be;
        // the size was taken from Millipede::initialize_queues().
        const size_t queue_size = 512 * cfg->data_symbol_num_perframe * 4;

        // First, move the sched_info_t and its queue into `subcarrier_doers_`,
        // using a temporary nullptr in place of the real doer instance.
        const size_t dest_index = index_for_subcarrier_id(sc_range.start);
        subcarrier_doers_[dest_index]
            = std::make_pair(sched_info_t(mt_queue_t(queue_size)),
                nullptr /* This nullptr will be replaced below */);

        // Second, get that sched_info_t that we just inserted into the
        // `subcarrier_doers_` vector, then create the subcarrier doer using
        // that sched_info_t's queue and producer token, and then insert a
        // pointer to the new subcarrier doer into the vector.
        std::pair<sched_info_t, DoSubcarrier*>& inserted_sched
            = subcarrier_doers_[dest_index];
        auto computeSubcarrier = new DoSubcarrier(cfg, worker_tid, freq_ghz_,
            inserted_sched.first.concurrent_q, complete_task_queue_,
            worker_producer_token, sc_range, csi_buffer_, recip_buffer_,
            calib_buffer_, dl_encoded_buffer_, data_buffer_, demod_soft_buffer_,
            dl_ifft_buffer_, ue_spec_pilot_buffer_, equal_buffer_,
            ul_zf_buffer_, dl_zf_buffer_, phy_stats_, stats_);
        inserted_sched.second = computeSubcarrier;

        return computeSubcarrier;
    }

    /// Returns the subcarrier ranges that the given worker thread uses,
    /// with each range corresponding to an individual `DoSubcarrier` instance.
    const std::vector<Range>& get_subcarrier_ranges_for_worker_tid(
        int tid) const
    {
        return worker_tid_to_subcarrier_range_[tid];
    }

    /// Returns the worker thread ID (tid) that contains the subcarrier doer
    /// instance that handles the given subcarrier id.
    inline size_t worker_tid_for_subcarrier(size_t subcarrier_id) const
    {
        return index_for_subcarrier_id(subcarrier_id) * cfg->worker_thread_num
            / num_subcarrier_ranges();
    }

    /// Returns the index into the list of subcarrier doers for the given
    /// subcarrier id.
    inline size_t index_for_subcarrier_id(size_t subcarrier_id) const
    {
        return subcarrier_id / subcarrier_block_size_;
    }

    /// Returns the subcarrier block size, which is the least common multiple
    /// of the zeroforcing block size and the demodulation block size.
    inline size_t subcarrier_block_size() const
    {
        return subcarrier_block_size_;
    }

    /// Returns the total  number of subcarrier ranges system-wide
    inline size_t num_subcarrier_ranges() const
    {
        return cfg->OFDM_DATA_NUM / subcarrier_block_size_;
    }

    /// Return the internal equalization buffer. This is useful for debugging or
    /// GUI
    Table<complex_float>& get_equal_buffer() { return equal_buffer_; }

private:
    Config* cfg; /// Millipede-wide configuration values
    double freq_ghz_; /// RDTSC frequency in GHz
    PhyStats* phy_stats_;
    Stats* stats_;

    /// A reference to the array of worker thread queues used by Millipede's
    /// master thread to schedule tasks.
    sched_info_t (&sched_info_arr_)[kMaxThreads];

    /// The singleton task completion queue.
    /// Push events onto this queue to inform Millipede's master thread that a
    /// given task has been completed.
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue_;

    /// The range of subcarrier frequencies handled by each subcarrier doer.
    /// This block size dictates how many subcarrier doers there are, with one
    /// `DoSubcarrier` instance for each block-sized range of subcarriers.
    size_t subcarrier_block_size_;

    /// The mappings from a given worker thread ID (the index into this array)
    /// to the list of subcarrier ranges for which that worker thread
    /// creates subcarrier doers.
    /// A single worker thread may create and contain multiple subcarrier doers;
    /// this occurs when there are fewer worker threads than subcarrier ranges.
    std::array<std::vector<Range>, kMaxThreads> worker_tid_to_subcarrier_range_;

    /// The list of subcarrier doers, indexed by subcarrier range base values.
    /// The `ith` element corresponds to the `ith` subcarrier block, e.g.,
    /// if the `subcarrier_block_size` is 48, then:
    /// - element 0 will be the subcarrier doer that handles subcarriers 0-47
    /// - element 1 will be the subcarrier doer that handles subcarriers 48-95
    /// - etc, until the end of the subcarriers, given by `cfg->OFDM_DATA_NUM`.
    ///
    /// ## Usage ##
    /// - When mutating this list, the `subcarrier_doers_mutex_` should be held.
    /// - Use the `subcarrier_doer_for_base()` function to index into this list.
    ///
    /// NOTE: For convenience, we could add the subcarrier range to this vector,
    ///       e.g., using a std::pair -- `vector<pair<int, DoSubcarrier*>>`
    std::vector<std::pair<sched_info_t, DoSubcarrier*>> subcarrier_doers_;

    /// The mutex guarding the above `subcarrier_doers_` vector.
    std::mutex subcarrier_doers_mutex_;

    /// Finds the subcarrier doer instance that handles a given subcarrier id.
    /// Returns a reference to a tuple (pair) that contains the matching doer's
    /// event queue and a pointer to the doer itself.
    std::pair<sched_info_t, DoSubcarrier*>& subcarrier_doer_for_id(
        size_t subcarrier_id)
    {
        return subcarrier_doers_[index_for_subcarrier_id(subcarrier_id)];
    }

    ///////////////////////////////////////////////////
    ////////////////// Input Buffers //////////////////
    ///////////////////////////////////////////////////

    /// An input buffer of channel state information (CSI),
    /// an output of the FFT stage.
    /// TODO: eventually we will move this into `DoSubcarrier`,
    ///       so it can be encapsulated entirely within the SubcarrierManager.
    Table<complex_float>& csi_buffer_;

    /// TODO: add documentation here.
    ///       I'm not 100% sure what this is used for, it's an input into doZF.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM
    /// @li 2nd dimension: number of OFDM data subcarriers * number of antennas
    Table<complex_float>& recip_buffer_;

    /// 1st dimension: TASK_BUFFER_FRAME_NUM
    /// 2nd dimension: number of OFDM data subcarriers * number of antennas
    Table<complex_float>& calib_buffer_;

    /// The main input buffer for subcarrier processing stages,
    /// which holds the data symbols after the FFT stage.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of antennas * number of OFDM data subcarriers
    ///
    /// The 2nd dimension's data order: 32 blocks each with 32 subcarriers each:
    /// subcarrier 1 -- 32 of antennas, subcarrier 33 -- 64 of antennas, ...,
    /// subcarrier 993 -- 1024 of antennas.
    Table<complex_float>& data_buffer_;

    /// An input buffer of encoded data coming from the encoder (e.g., LDPC),
    /// which is used only during downlink.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * num data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<int8_t>& dl_encoded_buffer_;

    ///////////////////////////////////////////////////
    ////////////////// Output Buffers /////////////////
    ///////////////////////////////////////////////////

    /// The main output buffer, which comes from the soft demodulation stage.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    ///
    /// TODO: this is currently a reference to the buffer owned by `Millipede`,
    ///       but eventually it should be a buffer owned by this class.
    Table<int8_t>& demod_soft_buffer_;

    /// An output buffer holding data destined for IFFT, only used in downlink.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of antennas
    ///                    * number of data symbols per frame
    /// @li 2nd dimension: number of OFDM carriers (including non-data carriers)
    ///
    /// TODO: this is currently a reference to the buffer owned by `Millipede`,
    ///       but eventually it should be a buffer owned by this class.
    Table<complex_float>& dl_ifft_buffer_;

    ///////////////////////////////////////////////////
    ///////// Internal / Intermediate Buffers /////////
    ///////////////////////////////////////////////////

    /// An internal buffer
    /// TODO: I'm fairly certain this is used only in DoDemul,
    ///       so we can move it directly into that class.
    Table<complex_float> ue_spec_pilot_buffer_;

    /// Data after equalization
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    /// TODO: I'm fairly certain this is used only in DoDemul,
    ///       so we can move it directly into that class.
    ///       The only concern with that is that it's accessed externally
    ///       by the Python GUI, so we'd need to export it.
    Table<complex_float> equal_buffer_;

    /// Calculated uplink zeroforcing detection matrices.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * num OFDM data subcarriers
    /// @li 2nd dimension: number of antennas * number of UEs
    ///
    /// This is an internal buffer that is written to by DoZF as output
    /// and then fed into `DoDemul` as input.
    Table<complex_float> ul_zf_buffer_;

    /// Calculated zeroforcing precoders for downlink beamforming.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * num OFDM data subcarriers
    /// @li 2nd dimension: number of antennas * number of UEs
    ///
    /// This is an internal buffer that is written to by DoZF as output
    /// and then fed into `DoPrecode` as input.
    Table<complex_float> dl_zf_buffer_;
};

#endif // SUBCARRIER_MANAGER_HPP
