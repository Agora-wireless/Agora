/**
 * Author: Kevin Boos
 * Email: kevinaboos@gmail.com
 *
 * @see SubcarrierManager
 */
#ifndef SUBCARRIER_MANAGER_HPP
#define SUBCARRIER_MANAGER_HPP

#include "utils.h"
#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "modulation.hpp"
#include "stats.hpp"
#include "phy_stats.hpp"
// #include <armadillo>
#include <iostream>
#include <numeric>
#include <vector>
#include <unordered_map>
#include <mutex>
// #include "mkl_dfti.h"

// using namespace arma;
using mt_queue_t = moodycamel::ConcurrentQueue<Event_data>;


/**
 * @brief The singleton manager of all `DoSubcarrier` instances.
 * 
 * This class is essentially a wrapper around the many `DoSubcarrier` instances,
 * aka subcarrier workers.
 * It encapsulates (and owns) the buffers shared by those `DoSubcarrier` instances
 * and exposes interfaces to communicate with them, e.g., by using event queues. 
 * 
 * 
 * There is at least one `DoSubcarrier` instance per subcarrier range,
 * typically just one but potentially more redundant replicates for 
 * purposes of scalability and reliability.
 * This class is responsible for owning the internal buffers that are used
 * by the subcarrier workers, 
 * and serves as a wrapper interface to export subcarrier-parallel functionality. 
 * 
 */
class SubcarrierManager {
public:
    
    SubcarrierManager(
        Config* config,
        double freq_ghz,
        // moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        // moodycamel::ProducerToken* worker_producer_token,
        // input buffers
        Table<complex_float>& csi_buffer,
        Table<complex_float>& recip_buffer,
        Table<complex_float>& calib_buffer,
        Table<int8_t>&        dl_encoded_buffer,
        Table<complex_float>& data_buffer, 
        // output buffers
        Table<int8_t>&        demod_soft_buffer,
        Table<complex_float>& dl_ifft_buffer,
        PhyStats* phy_stats,
        Stats* stats
    ) : cfg(config), 
        freq_ghz_(freq_ghz),
        phy_stats_(phy_stats),
        stats_(stats),
        complete_task_queue_(complete_task_queue), 
        subcarrier_block_size_(lcm(cfg->demul_block_size, cfg->zf_block_size)),
        csi_buffer_(csi_buffer),
        recip_buffer_(recip_buffer),
        calib_buffer_(calib_buffer),
        data_buffer_(data_buffer),
        dl_encoded_buffer_(dl_encoded_buffer),
        demod_soft_buffer_(demod_soft_buffer),
        dl_ifft_buffer_(dl_ifft_buffer)
    {
        const size_t task_buffer_symbol_num_ul = cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

        /*
         * Create the various buffers owned by the subcarrier manager. 
         * These buffers are shared across all subcarrier doers.
         * 
         * TODO: Currently, these buffers are created to be huge enough to store data 
         *       for *ALL* subcarrier ranges, system-wide. 
         *       We may want to scale them down to be sufficient for just a single subcarrier range;
         *       although, then we'd need to perform some kind of "gather" operation at the end
         *       before handing them off to the encoder stage.
         */

        // demod_soft_buffer_   .malloc(task_buffer_symbol_num_ul, cfg->mod_type * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        ue_spec_pilot_buffer_.calloc(TASK_BUFFER_FRAME_NUM, cfg->UL_PILOT_SYMS * cfg->UE_NUM, 64);
        equal_buffer_        .malloc(task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        ul_zf_buffer_        .malloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM, 64);
        dl_zf_buffer_        .calloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, cfg->UE_NUM * cfg->BS_ANT_NUM, 64);

        // Fill the subcarrier_doers_ with the total amount of doers (uninitialized ones).
        // This allows us to easily insert the real subcarrier doers at the correct index.
        auto num_doers = cfg->OFDM_DATA_NUM / subcarrier_block_size_;
        std::cout << "[SubcarrierManager]: subcarrier block size is " << subcarrier_block_size_ 
            << ", max subcarrier is " << cfg->OFDM_DATA_NUM 
            << ", requires " << num_doers << " doer(s)." << std::endl;
        subcarrier_doers_.reserve(num_doers);
        for (size_t i = 0; i < num_doers; i++) {
            subcarrier_doers_.emplace_back(std::make_pair(sched_info_t(), nullptr));
        }

        // Calculate the mapping between subcarrier doer instances and worker threads. 
        // As in the `Millipede` constructor, worker thread IDs go from `0` to `worker_thread_num`. 
        int tid = 0;
        for (size_t sc_base = 0; sc_base < cfg->OFDM_DATA_NUM; sc_base += subcarrier_block_size_) {
            Range range_to_insert(sc_base, sc_base + subcarrier_block_size_);
            auto iter = worker_tid_to_subcarrier_range_.find(tid);
            if (iter == worker_tid_to_subcarrier_range_.end()) {
                // if empty, create a new vector. 
                worker_tid_to_subcarrier_range_[tid] = { range_to_insert };
            } else {
                // if occupied, append to the existing vector.
                worker_tid_to_subcarrier_range_[tid].emplace_back(range_to_insert);
            }

            tid = (tid + 1) % (int)cfg->worker_thread_num;
        }
    

        std::cout << "[SubcarrierManager]: map of " << cfg->worker_thread_num << " worker threads to SC ranges: " << std::endl;
        for (auto &sc_ranges : worker_tid_to_subcarrier_range_) {
            std::cout << "\t Worker " << sc_ranges.first << ": { ";
            for (auto range : sc_ranges.second) {
                std::cout << "[" << range.start << ":" << range.end << "), ";
            }
            std::cout << " }" << std::endl;
        }

        /// NOTE: In the future (for remote subcarrier doers), we'll create the subcarrier doer instances here. 
        ///       However, currently with dynamic thread scheduling, we allow the worker threads
        ///       to create the subcarrier instances themselves. 
        ///
        // for (size_t sc_base = 0; sc_base < cfg->OFDM_DATA_NUM; sc_base += subcarrier_block_size_) {
        //     auto computeSubcarrier = new DoSubcarrier(cfg, subcarrier_doer_tid, freq_ghz,
        //         *get_conq(EventType::kSubcarrier), complete_task_queue_, worker_ptoks_ptr[tid],
        //         sc_base, subcarrier_block_size_,
        //         csi_buffer_, recip_buffer_, calib_buffer_, dl_encoded_buffer_, data_buffer_,
        //         demod_soft_buffer_, dl_ifft_buffer_,
        //         ue_spec_pilot_buffer_, equal_buffer_, ul_zf_buffer_, dl_zf_buffer_,
        //         phy_stats, stats
        //     );
        //     subcarrier_doers_.push_back(computeSubcarrier);
        // }
    }


    ~SubcarrierManager() {
        // First, free all of the buffers we own.
        equal_buffer_.free();
        // demod_soft_buffer_.free();
        ue_spec_pilot_buffer_.free();
        ul_zf_buffer_.free();
        dl_zf_buffer_.free();

        // Destroy all of the Subcarrier doers
        for (auto &dsc : subcarrier_doers_) {
            // the queue (dsc.first) is auto-freed
            delete dsc.second;
        }
    }


    /// Schedules a given subcarrier-parallel processing event or set of events
    /// by assigning those events to the proper subcarrier doer.
    void schedule_subcarriers(EventType event_type, size_t frame_id, size_t symbol_id) {
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
            // get the subcarrier doer that handles the current `sc_id` in the `base_tag`.
            auto &matching_doer = subcarrier_doer_for_id(base_tag.sc_id);
            auto &queue_info = matching_doer.first;

            std::cout << "[schedule_subcarriers] enqueueing event: " << (int)event_type 
                << ", frame_id: " << frame_id 
                << ", symbol_id: " << symbol_id
                << " into DoSubcarrier[" << base_tag.sc_id 
                << "] ptr: " << matching_doer.second
                // << ", queue.size: " << queue_info.concurrent_q.size_approx() 
                << ", ptok: " << queue_info.ptok << std::endl;


            try_enqueue_fallback(&queue_info.concurrent_q, queue_info.ptok,
                Event_data(event_type, base_tag._tag)
            );
            base_tag.sc_id += block_size;
        }
    }


    /// Creates a subcarrier doer to be run on the given worker thread
    /// and returns a pointer to that new doer. 
    ///
    /// This function also creates an event queue for each subcarrier doer
    /// such that this subcarrier manager can 
    DoSubcarrier* create_subcarrier_doer(int worker_tid, Range sc_range) {
        rt_assert((sc_range.start) / subcarrier_block_size_ == 
            (sc_range.end-1) / subcarrier_block_size_ ,
            "invalid subcarrier range, all subcarrier IDs within the range "
            "do not map to the same index into the subcarrier doers list"
        );

        std::lock_guard<std::mutex> lock(subcarrier_doers_mutex_);

        std::cout << "[create_subcarrier_doer]: worker_tid: " << worker_tid 
            << ", range: [" << sc_range.start << ":" << sc_range.end << ")"
            << ", there are " << subcarrier_doers_.size() << " doers." << std::endl;

        // Not sure how large this queue needs to be; size taken from Millipede::initialize_queues().
        // sched_info_t queue_info(mt_queue_t(512 * cfg->data_symbol_num_perframe * 4));
        // sched_info_t queue_info;
        // queue_info.concurrent_q = mt_queue_t(512 * cfg->data_symbol_num_perframe * 4); 
        // queue_info.ptok = new moodycamel::ProducerToken(queue_info.concurrent_q);

        // First, move the sched_info_t and its queue into our subcarrier_doers_ vector,
        // using a temporary nullptr in place of the real subcarrier doer instance.
        auto dest_index = sc_range.start / subcarrier_block_size_;
        subcarrier_doers_[dest_index] = std::make_pair(
            // TODO: what's the right queue size? This is taken from `Millipede::initialize_queues()`.
            sched_info_t(mt_queue_t(512 * cfg->data_symbol_num_perframe * 4)),
            nullptr
        ); 
        
        // Second, get that sched_info_t that we just inserted into the `subcarrier_doers_` vector,
        // create the subcarrier doer using that sched_info_t's queue and producer token,
        // and then properly insert a pointer to the new subcarrier doer into the vector.
        auto &inserted_sched = subcarrier_doers_[dest_index];
        auto computeSubcarrier = new DoSubcarrier(cfg, worker_tid, freq_ghz_,
            inserted_sched.first.concurrent_q, complete_task_queue_, inserted_sched.first.ptok,
            sc_range,
            csi_buffer_, recip_buffer_, calib_buffer_, dl_encoded_buffer_, data_buffer_,
            demod_soft_buffer_, dl_ifft_buffer_,
            ue_spec_pilot_buffer_, equal_buffer_, ul_zf_buffer_, dl_zf_buffer_,
            phy_stats_, stats_
        );
        inserted_sched.second = computeSubcarrier;

        return computeSubcarrier;
    }


    /// Returns the subcarrier ranges that the given worker thread should handle, 
    /// with each range corresponding to a the subcarrier doers on the given worker thread should be 
    const std::vector<Range>& get_subcarrier_ranges_for_worker_tid(int worker_tid) {
        static const std::vector<Range> empty;
        auto iter = worker_tid_to_subcarrier_range_.find(worker_tid);
        if (iter != worker_tid_to_subcarrier_range_.end()) {
            return iter->second;
        }
        return empty;
    } 

    /// An accessor function to expose the internal `equal_buffer_`
    /// for debugging/GUI purposes.
    Table<complex_float>& get_equal_buffer() {
        return equal_buffer_;
    }


private:
    /// Millipede-wide configuration values.
    Config* cfg;
    /// RDTSC frequency in GHz.
    double freq_ghz_;
    PhyStats* phy_stats_;
    Stats* stats_;

    /// TODO: there should be one task_queue per SubcarrierDoer.
    /// Millipede pushes events onto this queue to indicate
    /// to worker threads that various tasks are available to work on.
    // moodycamel::ConcurrentQueue<Event_data>& task_queue_;

    /// The singleton task completion queue. 
    /// Push events onto this queue to inform Millipede's master thread
    /// that a given task has been completed. 
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue_;
    /// The range of subcarrier frequencies handled by each subcarrier doer. 
    /// This is initialized (see the constructor's initializer list) to be the 
    /// least common multiple of the demodulation block size `demod_block_size` 
    /// and the zeroforcing block size `zf_block_size`.
    /// This block size dictates how many subcarrier doers there are, 
    /// with one `DoSubcarrier` instance for each block-sized range of subcarriers.
    size_t subcarrier_block_size_;

    /// The mappings from a given worker thread ID (an `int`)
    /// to the subcarrier ranges for which that worker thread creates subcarrier doers. 
    /// A single worker thread may create and contain multiple subcarrier doers;
    /// this is common when there are fewer worker threads than subcarrier ranges. 
    std::unordered_map<int, std::vector<Range>> worker_tid_to_subcarrier_range_;

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
    /// NOTE: For convenience, we could also include the subcarrier range in this vector,
    ///       e.g., using a std::pair -- `vector<pair<int, DoSubcarrier*>>`
    std::vector<std::pair<sched_info_t, DoSubcarrier*>> subcarrier_doers_;

    /// The mutex guarding the above `subcarrier_doers_` vector. 
    /// This must be acquired/locked when mutating `subcarrier_doers_`;
    /// for this, using `std::lock_guard` is recommended.
    std::mutex subcarrier_doers_mutex_;

    /// Finds the subcarrier doer instance that handles a given subcarrier id. 
    /// Returns a reference to a tuple (pair) that contains
    /// the matching doer's event queue and a pointer to the doer itself.
    std::pair<sched_info_t, DoSubcarrier*>& subcarrier_doer_for_id(size_t subcarrier_id) {
        return subcarrier_doers_[subcarrier_id / subcarrier_block_size_];
    }

    ///////////////////////////////////////////////////
    ////////////////// Input Buffers //////////////////
    ///////////////////////////////////////////////////

    /// An input buffer of channel state information (CSI), an output of the FFT stage.
    /// TODO: eventually we will move this into `DoSubcarrier`, 
    ///       so it can be encapsulated entirely within the SubcarrierManager.
    Table<complex_float>& csi_buffer_;

    /// TODO: I'm not 100% sure what this is used for. It's an input buffer into doZF...
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
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of data symbols per frame
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
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of antennas * number of data symbols per frame
    /// @li 2nd dimension: number of OFDM carriers (including non-data carriers)
    ///
    /// TODO: this is currently a reference to the buffer owned by `Millipede`,
    ///       but eventually it should be a buffer owned by this class.
    Table<complex_float>& dl_ifft_buffer_;


    ///////////////////////////////////////////////////
    ///////// Internal / Intermediate Buffers /////////
    ///////////////////////////////////////////////////

    /// An internal buffer
    /// TODO: I'm fairly certain this is used only in DoDemul, so we can move it directly into that class.
    Table<complex_float> ue_spec_pilot_buffer_;

    /// Data after equalization
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    /// TODO: I'm fairly certain this is used only in DoDemul, so we can move it directly into that class.
    ///       The only concern with that is that it's accessed externally by the Python GUI, so we'd need to export it.
    Table<complex_float> equal_buffer_;

    /// Calculated uplink zeroforcing detection matrices.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of OFDM data subcarriers
    /// @li 2nd dimension: number of antennas * number of UEs
    ///
    /// This is an internal buffer that is written to by DoZF as output
    /// and then fed into `DoDemul` as input.
    Table<complex_float> ul_zf_buffer_;

    /// Calculated zeroforcing precoders for downlink beamforming.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of OFDM data subcarriers
    /// @li 2nd dimension: number of antennas * number of UEs
    ///
    /// This is an internal buffer that is written to by DoZF as output
    /// and then fed into `DoPrecode` as input.
    Table<complex_float> dl_zf_buffer_;
};

#endif // SUBCARRIER_MANAGER_HPP
