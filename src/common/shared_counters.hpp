#ifndef SHARED_COUNTERS_HPP
#define SHARED_COUNTERS_HPP

#include "Symbols.hpp"
#include "utils.h"
#include <mutex>
#include <sstream>
#include <vector>

// We use one RxStatus object to track packet reception status.
// This object is shared between socket threads and subcarrier workers.
class RxStatus {
public:
    RxStatus(Config* cfg)
        : num_pilot_pkts_per_frame_(
              cfg->pilot_symbol_num_perframe * cfg->BS_ANT_NUM)
        , num_pilot_symbols_per_frame_(cfg->pilot_symbol_num_perframe)
        , num_data_symbol_per_frame_(cfg->data_symbol_num_perframe)
        , num_pkts_per_symbol_(cfg->BS_ANT_NUM)
        , num_decode_tasks_per_frame_(
              cfg->get_ofdm_control_num() / cfg->subcarrier_block_size)
    {
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            num_pkts_[i] = 0;
        }
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            num_pilot_pkts_[i] = 0;
        }
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            for (size_t j = 0; j < kMaxNumSymbolsPerFrame; j++) {
                num_data_pkts_[i][j] = 0;
            }
        }
    }

    // When receive a new packet, record it here
    bool add_new_packet(size_t frame_id, size_t symbol_id)
    {
        if (frame_id >= cur_frame_ + TASK_BUFFER_FRAME_NUM) {
            std::cout << "Error: Received packet for future frame beyond "
                         "frame "
                      << "window. This can happen if Millipede is running "
                      << "slowly, e.g., in debug mode\n";
            return false;
        }
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        num_pkts_[frame_slot]++;
        if (num_pkts_[frame_slot]
            == num_pkts_per_symbol_
                * (num_pilot_symbols_per_frame_ + num_data_symbol_per_frame_)) {
            printf(
                "Main thread: received all packets in frame: %lu\n", frame_id);
        }

        if (symbol_id < num_pilot_symbols_per_frame_) {
            num_pilot_pkts_[frame_slot]++;
            if (num_pilot_pkts_[frame_slot] == num_pilot_pkts_per_frame_) {
                printf("Main thread: received all pilots in frame: %zu\n",
                    frame_id);
            }
        } else {
            num_data_pkts_[frame_slot][symbol_id]++;
        }
        if (frame_id > latest_frame_) {
            // NOTE: race condition could happen here but the impact is small
            latest_frame_ = frame_id;
        }
        return true;
    }

    // Check whether all pilot packets are received for a frame
    // used by CSI
    bool received_all_pilots(size_t frame_id)
    {
        if (frame_id < cur_frame_
            || frame_id >= cur_frame_ + TASK_BUFFER_FRAME_NUM) {
            return false;
        }
        return num_pilot_pkts_[frame_id % TASK_BUFFER_FRAME_NUM]
            == num_pilot_pkts_per_frame_;
    }

    // Check whether demodulation can proceed for a symbol in a frame
    bool is_demod_ready(size_t frame_id, size_t symbol_id)
    {
        if (frame_id < cur_frame_
            || frame_id >= cur_frame_ + TASK_BUFFER_FRAME_NUM) {
            return false;
        }
        return num_data_pkts_[frame_id % TASK_BUFFER_FRAME_NUM][symbol_id]
            == num_pkts_per_symbol_;
    }

    // When decoding is done for a frame from one decoder, call this function
    // This function will increase cur_frame when this frame is decoded so that
    // we can move on decoding the next frame and release the resources used by
    // this frame
    void decode_done(size_t frame_id)
    {
        rt_assert(frame_id == cur_frame_, "Wrong completed decode task!");
        decode_mutex_.lock();
        num_decode_tasks_completed_++;
        if (num_decode_tasks_completed_ == num_decode_tasks_per_frame_) {
            cur_frame_++;
            num_decode_tasks_completed_ = 0;
            size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
            num_pkts_[frame_slot] = 0;
            num_pilot_pkts_[frame_slot] = 0;
            for (size_t j = 0; j < kMaxNumSymbolsPerFrame; j++) {
                num_data_pkts_[frame_slot][j] = 0;
            }
            printf("Main thread: Decode done frame: %lu\n", cur_frame_ - 1);
        }
        decode_mutex_.unlock();
    }

    // num_pkts[i % TASK_BUFFER_FRAME_NUM] is the total number of packets
    // received for frame i (may be removed if not used)
    std::array<std::atomic<size_t>, TASK_BUFFER_FRAME_NUM> num_pkts_;

    // num_pilot_pkts[i % TASK_BUFFER_FRAME_NUM] is the total number of pilot
    // packets received for frame i
    std::array<std::atomic<size_t>, TASK_BUFFER_FRAME_NUM> num_pilot_pkts_;

    // num_data_pkts[i % TASK_BUFFER_FRAME_NUM][j] is the total number of data
    // packets received for frame i and symbol j
    std::array<std::array<std::atomic<size_t>, kMaxNumSymbolsPerFrame>,
        TASK_BUFFER_FRAME_NUM>
        num_data_pkts_;

    // cur_frame is the first frame for which decoding is incomplete
    size_t cur_frame_ = 0;

    // The max frame number for which socket I/O threads have received any packet
    size_t latest_frame_ = 0;

    // Atomic counter for # completed decode tasks
    // cur_frame will be incremented in all tasks are completed
    size_t num_decode_tasks_completed_;
    std::mutex decode_mutex_;

    // Copies of Config variables
    const size_t num_pilot_pkts_per_frame_;
    const size_t num_pilot_symbols_per_frame_;
    const size_t num_data_symbol_per_frame_;
    const size_t num_pkts_per_symbol_;
    const size_t num_decode_tasks_per_frame_;
};

// We use DemulStatus to track # completed demul tasks for each symbol
// This object is shared between all dosubcarriers and dodecoders
class DemulStatus {
public:
    DemulStatus(Config* cfg)
        : num_demul_tasks_required_(
              cfg->get_ofdm_control_num() / cfg->demul_block_size)
    {
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            num_demul_tasks_completed_[i].fill(0);
        }
        max_frame_ = 0;
    }

    // A dosubcarrier launch this function to notify some subcarriers
    // are completed for demul tasks in a symbol of a frame
    void demul_complete(size_t frame_id, size_t symbol_id, size_t num_tasks)
    {
        if (frame_id > max_frame_) {
            max_frame_mutex_.lock();
            if (frame_id > max_frame_) {
                max_frame_++;
                num_demul_tasks_completed_[max_frame_ % TASK_BUFFER_FRAME_NUM]
                    .fill(0);
            }
            max_frame_mutex_.unlock();
        }
        rt_assert(frame_id <= max_frame_
                && frame_id + TASK_BUFFER_FRAME_NUM > max_frame_,
            "Complete a wrong frame in demul!");
        mutex_list_[symbol_id].lock();
        num_demul_tasks_completed_[frame_id % TASK_BUFFER_FRAME_NUM][symbol_id]
            += num_tasks;
        mutex_list_[symbol_id].unlock();
    }

    // A dodecode checks whether all subcarriers in a symbol complete their
    // demul tasks so that it could start to decode
    bool ready_to_decode(size_t frame_id, size_t symbol_id)
    {
        rt_assert(
            frame_id + TASK_BUFFER_FRAME_NUM > max_frame_, "Decode too slow!");
        if (frame_id > max_frame_) {
            return false;
        }
        return num_demul_tasks_completed_[frame_id % TASK_BUFFER_FRAME_NUM]
                                         [symbol_id]
            == num_demul_tasks_required_;
    }

    // num_demul_tasks_completed[i % TASK_BUFFER_FRAME_NUM][j] is
    // the number of subcarriers completed for demul tasks in
    // frame i and symbol j
    std::array<std::array<size_t, kMaxNumSymbolsPerFrame>,
        TASK_BUFFER_FRAME_NUM>
        num_demul_tasks_completed_;

    // # subcarriers required to demul for each symbol
    const size_t num_demul_tasks_required_;

    // Create a mutex for each symbol
    // to synchronize the update for each dosubcarrier
    std::array<std::mutex, kMaxNumSymbolsPerFrame> mutex_list_;

    size_t max_frame_;
    std::mutex max_frame_mutex_;
};

class DecodeStatus {
public:
    DecodeStatus(Config* cfg)
        : cfg_(cfg)
        , num_demod_data_required_(cfg->server_addr_list.size())
    {
        cur_frame_ = new size_t[cfg->get_num_ues_to_process()];
        memset(cur_frame_, 0, sizeof(size_t) * cfg->get_num_ues_to_process());
        cur_symbol_ = new size_t[cfg->get_num_ues_to_process()];
        memset(cur_symbol_, 0, sizeof(size_t) * cfg->get_num_ues_to_process());

        num_demod_data_received_
            = new std::array<std::array<size_t, kMaxNumSymbolsPerFrame>,
                TASK_BUFFER_FRAME_NUM>[cfg->get_num_ues_to_process()];
        for (size_t i = 0; i < cfg->get_num_ues_to_process(); i++) {
            for (size_t j = 0; j < TASK_BUFFER_FRAME_NUM; j++) {
                num_demod_data_received_[i][j].fill(0);
            }
        }
    }

    void receive_demod_data(size_t ue_id, size_t frame_id, size_t symbol_id)
    {
        num_demod_data_received_[ue_id - cfg->ue_start]
                                [frame_id % TASK_BUFFER_FRAME_NUM][symbol_id]++;
    }

    bool received_all_demod_data(
        size_t ue_id, size_t frame_id, size_t symbol_id)
    {
        if (num_demod_data_received_[ue_id
                - cfg->ue_start][frame_id % TASK_BUFFER_FRAME_NUM][symbol_id]
            == num_demod_data_required_) {
            num_demod_data_received_[ue_id
                - cfg->ue_start][frame_id % TASK_BUFFER_FRAME_NUM][symbol_id]
                = 0;
            return true;
        }
        return false;
    }

private:
    Config* cfg_;
    size_t* cur_frame_;
    size_t* cur_symbol_; // symbol ID for UL data
    const size_t num_demod_data_required_;

    std::array<std::array<size_t, kMaxNumSymbolsPerFrame>,
        TASK_BUFFER_FRAME_NUM>* num_demod_data_received_;
};

#endif