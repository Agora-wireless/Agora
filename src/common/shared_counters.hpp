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
    // num_pkts[i % TASK_BUFFER_FRAME_NUM] is the total number of packets
    // received for frame i (may be removed if not used)
    std::array<std::atomic<size_t>, TASK_BUFFER_FRAME_NUM> num_pkts;

    // num_pilot_pkts[i % TASK_BUFFER_FRAME_NUM] is the total number of pilot
    // packets received for frame i
    std::array<std::atomic<size_t>, TASK_BUFFER_FRAME_NUM> num_pilot_pkts;

    // num_data_pkts[i % TASK_BUFFER_FRAME_NUM][j] is the total number of data
    // packets received for frame i and symbol j
    std::array<std::array<std::atomic<size_t>, kMaxNumSymbolsPerFrame>,
        TASK_BUFFER_FRAME_NUM>
        num_data_pkts;

    // cur_frame is the first frame for which decoding is incomplete
    size_t cur_frame = 0;

    // The max frame number for which socket I/O threads have received any packet
    size_t latest_frame = 0;

    // Atomic counter for # completed decode tasks
    // cur_frame will be incremented in all tasks are completed
    size_t num_decode_tasks_completed;
    std::mutex decode_mutex;

    // Copies of Config variables
    const size_t num_pilot_pkts_per_frame;
    const size_t num_pilot_symbols_per_frame;
    const size_t num_data_symbol_per_frame;
    const size_t num_pkts_per_symbol;
    const size_t num_decode_tasks_per_frame;

    RxStatus(size_t _num_pilot_pkts_per_frame,
        size_t _num_pilot_symbols_per_frame, size_t _num_data_symbol_per_frame,
        size_t _num_pkts_per_symbol, size_t _num_decode_tasks_per_frame)
        : num_pilot_pkts_per_frame(_num_pilot_pkts_per_frame)
        , num_pilot_symbols_per_frame(_num_pilot_symbols_per_frame)
        , num_data_symbol_per_frame(_num_data_symbol_per_frame)
        , num_pkts_per_symbol(_num_pkts_per_symbol)
        , num_decode_tasks_per_frame(_num_decode_tasks_per_frame)
    {
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            num_pkts[i] = 0;
        }
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            num_pilot_pkts[i] = 0;
        }
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            for (size_t j = 0; j < kMaxNumSymbolsPerFrame; j++) {
                num_data_pkts[i][j] = 0;
            }
        }
    }

    bool add_new_packet(size_t frame_id, size_t symbol_id)
    {
        if (frame_id >= cur_frame + TASK_BUFFER_FRAME_NUM) {
            std::cout << "Error: Received packet for future frame beyond "
                         "frame "
                      << "window. This can happen if Millipede is running "
                      << "slowly, e.g., in debug mode\n";
            return false;
        }
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        num_pkts[frame_slot]++;
        if (num_pkts[frame_slot]
            == num_pkts_per_symbol
                * (num_pilot_symbols_per_frame + num_data_symbol_per_frame)) {
            printf(
                "Main thread: received all packets in frame: %lu\n", frame_id);
        }
        // TODO
        // 1. Move those statements into functions in RxStatus
        // 2. Check invalid packets
        if (symbol_id < num_pilot_symbols_per_frame) {
            num_pilot_pkts[frame_slot]++;
            if (num_pilot_pkts[frame_slot] == num_pilot_pkts_per_frame) {
                printf("Main thread: received all pilots in frame: %zu\n",
                    frame_id);
            }
        } else {
            num_data_pkts[frame_slot][symbol_id]++;
        }
        if (frame_id > latest_frame) {
            // NOTE: race condition could happen here but the impact is small
            latest_frame = frame_id;
        }
        return true;
    }

    bool is_pilot_ready(size_t frame_id)
    {
        if (frame_id < cur_frame
            || frame_id >= cur_frame + TASK_BUFFER_FRAME_NUM) {
            return false;
        }
        return num_pilot_pkts[frame_id % TASK_BUFFER_FRAME_NUM]
            == num_pilot_pkts_per_frame;
    }

    bool is_demod_ready(size_t frame_id, size_t symbol_id)
    {
        if (frame_id < cur_frame
            || frame_id >= cur_frame + TASK_BUFFER_FRAME_NUM) {
            return false;
        }
        return num_data_pkts[frame_id % TASK_BUFFER_FRAME_NUM][symbol_id]
            == num_pkts_per_symbol;
    }

    void decode_done(size_t frame_id)
    {
        rt_assert(frame_id == cur_frame, "Wrong completed decode task!");
        decode_mutex.lock();
        num_decode_tasks_completed++;
        if (num_decode_tasks_completed == num_decode_tasks_per_frame) {
            cur_frame++;
            num_decode_tasks_completed = 0;
            size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
            num_pkts[frame_slot] = 0;
            num_pilot_pkts[frame_slot] = 0;
            for (size_t j = 0; j < kMaxNumSymbolsPerFrame; j++) {
                num_data_pkts[frame_slot][j] = 0;
            }
            printf("Main thread: Decode done frame: %lu\n", cur_frame - 1);
        }
        decode_mutex.unlock();
    }
};

// We use DecodeStatus to track # completed demul tasks for each symbol
// This object is shared between all dosubcarriers and dodecoders
class DemulStatus {
public:
    // num_demul_tasks_completed[i % TASK_BUFFER_FRAME_NUM][j] is
    // the number of subcarriers completed for demul tasks in
    // frame i and symbol j
    std::array<std::array<size_t, kMaxNumSymbolsPerFrame>,
        TASK_BUFFER_FRAME_NUM>
        num_demul_tasks_completed;

    // # subcarriers required to demul for each symbol
    const size_t num_demul_tasks_required;

    // Create a mutex for each symbol
    // to synchronize the update for each dosubcarrier
    std::array<std::mutex, kMaxNumSymbolsPerFrame> mutex_list;

    size_t max_frame;
    std::mutex max_frame_mutex;

    DemulStatus(size_t _num_demul_tasks_required)
        : num_demul_tasks_required(_num_demul_tasks_required)
    {
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            num_demul_tasks_completed[i].fill(0);
        }
        max_frame = 0;
    }

    // A dosubcarrier launch this function to notify some subcarriers
    // are completed for demul tasks in a symbol of a frame
    void demul_complete(size_t frame_id, size_t symbol_id, size_t num_tasks)
    {
        if (frame_id > max_frame) {
            max_frame_mutex.lock();
            if (frame_id > max_frame) {
                max_frame++;
                num_demul_tasks_completed[max_frame % TASK_BUFFER_FRAME_NUM]
                    .fill(0);
            }
            max_frame_mutex.unlock();
        }
        rt_assert(frame_id <= max_frame
                && frame_id + TASK_BUFFER_FRAME_NUM > max_frame,
            "Complete a wrong frame in demul!");
        mutex_list[symbol_id].lock();
        num_demul_tasks_completed[frame_id % TASK_BUFFER_FRAME_NUM][symbol_id]
            += num_tasks;
        mutex_list[symbol_id].unlock();
    }

    // A dodecode checks whether all subcarriers in a symbol complete their
    // demul tasks so that it could start to decode
    bool ready_to_decode(size_t frame_id, size_t symbol_id)
    {
        rt_assert(
            frame_id + TASK_BUFFER_FRAME_NUM > max_frame, "Decode too slow!");
        if (frame_id > max_frame) {
            return false;
        }
        return num_demul_tasks_completed[frame_id % TASK_BUFFER_FRAME_NUM]
                                        [symbol_id]
            == num_demul_tasks_required;
    }
};

#endif