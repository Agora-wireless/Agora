#pragma once

#include "Symbols.hpp"
#include "config.hpp"
#include "logger.h"
#include "utils.h"
#include "gettime.h"
#include <mutex>
#include <sstream>
#include <vector>

// We use one SharedState object to track packet reception status.
// This object is shared between socket threads and subcarrier workers.
class SharedState {
public:
    SharedState(Config* cfg)
        : num_pilot_pkts_per_frame_(
              cfg->pilot_symbol_num_perframe * cfg->BS_ANT_NUM)
        , num_pilot_symbols_per_frame_(cfg->pilot_symbol_num_perframe)
        , num_ul_data_symbol_per_frame_(cfg->ul_data_symbol_num_perframe)
        , num_pkts_per_symbol_(cfg->BS_ANT_NUM)
        , num_decode_tasks_per_frame_(cfg->decode_thread_num)
        , num_precode_tasks_per_frame_((cfg->get_num_sc_to_process() + cfg->subcarrier_block_size - 1) / cfg->subcarrier_block_size)
        , num_demul_tasks_required_(cfg->get_num_sc_to_process() / cfg->demul_block_size)
        , num_demod_pkts_per_symbol_per_ue_(cfg->bs_server_addr_list.size())
        , last_frame_cycles_(worker_rdtsc())
        , freq_ghz_(measure_rdtsc_freq())
    {
        frame_start_time_ = new uint64_t[cfg->frames_to_test];
        frame_iq_time_ = new uint64_t[cfg->frames_to_test];
        frame_sc_time_ = new uint64_t[cfg->frames_to_test];
        frame_decode_time_ = new uint64_t[cfg->frames_to_test]; 
        frame_end_time_ = new uint64_t[cfg->frames_to_test];
        memset(frame_start_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
        memset(frame_iq_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
        memset(frame_sc_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
        memset(frame_decode_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
        memset(frame_end_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
        for (size_t i = 0; i < kFrameWnd; i++) {
            for (size_t j = 0; j < kMaxSymbols; j++) {
                num_demul_tasks_completed_[i][j] = 0;
            }
        }
        for (size_t i = 0; i < cfg->UE_NUM; i++) {
            for (size_t j = 0; j < kFrameWnd; j++) {
                for (size_t k = 0; k < kMaxSymbols; k ++) {
                    num_demod_pkts_[i][j][k] = 0;
                }
            }
        }
    }


    /*********************************************
     * 
     * Functions for receiving new packets
     * 
     *********************************************/

    // When receive a new packet, record it here
    bool receive_freq_iq_pkt(const Packet* pkt)
    {
        if (unlikely(pkt->frame_id >= cur_frame_ + kFrameWnd)) {
            MLPD_ERROR(
                "SharedCounters SharedState error: Received freq iq packet for future "
                "frame %u beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
                "happen if Agora is running slowly, e.g., in debug mode. "
                "Full packet = %s.\n",
                pkt->frame_id, cur_frame_, kFrameWnd, cur_frame_, num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
                num_pkts_[cur_frame_ % kFrameWnd].load(), pkt->to_string().c_str());
            return false;
        }

        if (unlikely(frame_start_time_[pkt->frame_id] == 0)) {
            frame_start_time_[pkt->frame_id] = get_ns();
        }

        const size_t frame_slot = pkt->frame_id % kFrameWnd;
        bool full = false;
        num_pkts_[frame_slot]++;
        encode_ready_[frame_slot] = true;
        if (num_pkts_[frame_slot]
            == num_pkts_per_symbol_
                * (num_pilot_symbols_per_frame_ + num_ul_data_symbol_per_frame_)) {
            MLPD_INFO("SharedCounters: received all packets in frame: %u. "
                   "Pilot pkts = %zu of %zu\n",
                pkt->frame_id, num_pilot_pkts_[frame_slot].load(),
                num_pilot_pkts_per_frame_);
            full = true;
            frame_iq_time_[pkt->frame_id] = get_ns();
        }

        if (pkt->symbol_id < num_pilot_symbols_per_frame_) {
            num_pilot_pkts_[frame_slot]++;
            if (num_pilot_pkts_[frame_slot] == num_pilot_pkts_per_frame_) {
                MLPD_INFO("SharedCounters: received all pilots in frame: %u\n",
                    pkt->frame_id);
            }
        } else {
            num_data_pkts_[frame_slot][pkt->symbol_id - num_pilot_symbols_per_frame_]++;
        }

        return true;
    }

    bool receive_demod_pkt(size_t ue_id, size_t frame_id, size_t symbol_id_ul)
    {
        if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
            MLPD_ERROR(
                "SharedCounters SharedState error: Received demod packet for future "
                "frame %u beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
                "happen if Agora is running slowly, e.g., in debug mode. \n",
                frame_id, cur_frame_, kFrameWnd, cur_frame_, num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
                num_pkts_[cur_frame_ % kFrameWnd].load());
            return false;
        }
        num_demod_pkts_[ue_id][frame_id % kFrameWnd][symbol_id_ul]++;
        return true;
    }


    /*********************************************
     * 
     * Functions for checking whether all packets
     * of a certain type are received
     * 
     *********************************************/

    // Check whether all pilot packets are received for a frame
    // used by CSI
    bool received_all_pilots(size_t frame_id)
    {
        if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
            return false;
        }
        return num_pilot_pkts_[frame_id % kFrameWnd]
            == num_pilot_pkts_per_frame_;
    }

    // Check whether demodulation can proceed for a symbol in a frame
    bool received_all_data_pkts(size_t frame_id, size_t symbol_id_ul)
    {
        if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
            return false;
        }
        return num_data_pkts_[frame_id % kFrameWnd][symbol_id_ul]
            == num_pkts_per_symbol_;
    }

    bool received_all_demod_pkts(
        size_t ue_id, size_t frame_id, size_t symbol_id_ul)
    {
        if (num_demod_pkts_[ue_id][frame_id % kFrameWnd][symbol_id_ul]
            == num_demod_pkts_per_symbol_per_ue_) {
            if (symbol_id_ul == 0) {
                frame_decode_time_[frame_id] = get_ns();
            }
            return true;
        }
        return false;
    }

    // Check whether encoding can proceed for a frame
    bool is_encode_ready(size_t frame_id) {
        if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
            return false;
        }
        return encode_ready_[frame_id % kFrameWnd];
    }


    /*********************************************
     * 
     * Functions for recording completed tasks
     * 
     *********************************************/

    // Mark [num_tasks] demodulation tasks for this frame and symbol as complete
    void demul_done(size_t frame_id, size_t symbol_id_ul, size_t num_tasks)
    {
        rt_assert(frame_id >= cur_frame_ && frame_id < cur_frame_ + kFrameWnd,
            "Complete a wrong frame in demul!");
        num_demul_tasks_completed_[frame_id % kFrameWnd][symbol_id_ul]
            += num_tasks;
    }

    // When decoding is done for a frame from one decoder, call this function
    // This function will increase cur_frame when this frame is decoded so that
    // we can move on decoding the next frame and release the resources used by
    // this frame
    void decode_done(size_t frame_id)
    {
        rt_assert(frame_id < cur_frame_ + kFrameWnd && frame_id >= cur_frame_, "Wrong completed decode task!");
        bool cont = false;
        decode_mutex_[frame_id % kFrameWnd].lock();
        num_decode_tasks_completed_[frame_id % kFrameWnd]++;
        cont = (num_decode_tasks_completed_[frame_id % kFrameWnd] == num_decode_tasks_per_frame_);
        decode_mutex_[frame_id % kFrameWnd].unlock();

        if (unlikely(cont)) {
            cur_frame_mutex_.lock();
            while (num_decode_tasks_completed_[cur_frame_ % kFrameWnd] == num_decode_tasks_per_frame_) {
                frame_end_time_[cur_frame_] = get_ns();
                cur_frame_ ++;
                encode_ready_[(cur_frame_ - 1) % kFrameWnd] = false;
                size_t cur_cycle = worker_rdtsc();
                num_decode_tasks_completed_[(cur_frame_ - 1) % kFrameWnd] = 0;
                size_t frame_slot = (cur_frame_ - 1) % kFrameWnd;
                num_pkts_[frame_slot] = 0;
                num_pilot_pkts_[frame_slot] = 0;
                for (size_t j = 0; j < kMaxSymbols; j++) {
                    num_data_pkts_[frame_slot][j] = 0;
                }
                for (size_t i = 0; i < kMaxSymbols; i++) {
                    num_demul_tasks_completed_[frame_slot][i] = 0;
                }
                for (size_t i = 0; i < kMaxUEs; i ++) {
                    for (size_t j = 0; j < kMaxSymbols; j++) {
                        num_demod_pkts_[i][frame_slot][j] = 0;
                    }
                }
                MLPD_INFO("Main thread: Decode done frame: %lu, for %.2lfms\n", cur_frame_ - 1, cycles_to_ms(cur_cycle - last_frame_cycles_, freq_ghz_));
                last_frame_cycles_ = cur_cycle;
            }
            cur_frame_mutex_.unlock();
        }
    }

    // When precoding is done for a frame from one dosubcarrier worker, call this function
    // This function will increase cur_frame_ when this frame is precoded so that
    // we can move on precoding the next frame and release the resources used by this frame
    void precode_done(size_t frame_id)
    {
        rt_assert(frame_id == cur_frame_, "Wrong completed precode task!");
        precode_mutex_.lock();
        num_precode_tasks_completed_++;
        if (num_precode_tasks_completed_ == num_precode_tasks_per_frame_) {
            cur_frame_++;
            encode_ready_[(cur_frame_ - 1) % kFrameWnd] = false;
	        size_t cur_cycle = worker_rdtsc();
            num_precode_tasks_completed_ = 0;
            size_t frame_slot = frame_id % kFrameWnd;
            num_pkts_[frame_slot] = 0;
            num_pilot_pkts_[frame_slot] = 0;
            for (size_t j = 0; j < kMaxSymbols; j++) {
                num_data_pkts_[frame_slot][j] = 0;
            }
            MLPD_INFO("Main thread: Precode done frame: %lu, for %.2lfms\n", cur_frame_ - 1, cycles_to_ms(cur_cycle - last_frame_cycles_, freq_ghz_));
            last_frame_cycles_ = cur_cycle;
        }
        precode_mutex_.unlock();
    }


    /*********************************************
     * 
     * Functions for checking whether certain task
     * is completed
     * 
     *********************************************/

    // Return true iff we have completed demodulation for all subcarriers in
    // this symbol have
    bool is_demod_tx_ready(size_t frame_id, size_t symbol_id_ul)
    {
        if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
            return false;
        }
        if (num_demul_tasks_completed_[frame_id % kFrameWnd][symbol_id_ul]
            == num_demul_tasks_required_) {
            frame_sc_time_[frame_id] = get_ns();
            return true;
        } 
        return false;
    }


    /*********************************************
     * 
     * Packet and task completion counters
     * 
     *********************************************/

    // TODO: Instead of having all-atomic counter arrays, can we just make
    // the entire class atomic?

    // num_pkts[i % kFrameWnd] is the total number of packets
    // received for frame i (may be removed if not used)
    std::array<std::atomic<size_t>, kFrameWnd> num_pkts_ = {};

    // num_pilot_pkts[i % kFrameWnd] is the total number of pilot
    // packets received for frame i
    std::array<std::atomic<size_t>, kFrameWnd> num_pilot_pkts_ = {};

    // num_data_pkts[i % kFrameWnd][j] is the total number of data
    // packets received for frame i and symbol j
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_data_pkts_ = {};

    // num_demul_tasks_completed[i % kFrameWnd][j] is
    // the number of subcarriers completed for demul tasks in
    // frame i and symbol j
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_demul_tasks_completed_;

    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_demod_pkts_[kMaxUEs];

    // encode_ready_[i % kFrameWnd] represents whether encoding can proceed
    // for frame i
    std::array<bool, kFrameWnd> encode_ready_;

    // cur_frame is the first frame for which decoding is incomplete
    size_t cur_frame_ = 0;

    // Atomic counter for # completed decode tasks
    // cur_frame will be incremented in all tasks are completed
    std::array<size_t, kFrameWnd> num_decode_tasks_completed_ = {};
    std::array<std::mutex, kFrameWnd> decode_mutex_ = {};
    std::mutex cur_frame_mutex_;

    // Atomic counter for # completed precode tasks
    // cur_frame_ will be incremented in all tasks are completed
    size_t num_precode_tasks_completed_;
    std::mutex precode_mutex_;

    // Latency measurement counters for each frame
    uint64_t *frame_start_time_;
    uint64_t *frame_iq_time_;
    uint64_t *frame_sc_time_;
    uint64_t *frame_decode_time_;
    uint64_t *frame_end_time_;

    // The timestamp when last frame was processed (in cycles)
    size_t last_frame_cycles_;
    
    // The CPU frequency (in GHz), used to convert cycles to time
    size_t freq_ghz_;

    // Copies of Config variables
    const size_t num_pilot_pkts_per_frame_;
    const size_t num_pilot_symbols_per_frame_;
    const size_t num_ul_data_symbol_per_frame_;
    const size_t num_pkts_per_symbol_;
    const size_t num_decode_tasks_per_frame_;
    const size_t num_precode_tasks_per_frame_;
    const size_t num_demul_tasks_required_;
    const size_t num_demod_pkts_per_symbol_per_ue_;
};

class EncodeStatus
{
public:
    EncodeStatus(Config* cfg) 
        : cfg_(cfg)
        , num_encode_tasks_required_(1)
    {
        num_encode_tasks_completed_ = new 
            std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>[cfg->UE_NUM];
    }

    void encode_done(size_t ue_id, size_t frame_id, size_t symbol_id_dl) 
    {
        num_encode_tasks_completed_[ue_id][frame_id % kFrameWnd][symbol_id_dl] ++;
    }

    bool ready_to_precode(size_t ue_id, size_t frame_id, size_t symbol_id_dl)
    {
        if (num_encode_tasks_completed_[ue_id][frame_id % kFrameWnd][symbol_id_dl]
            == num_encode_tasks_required_) {
            num_encode_tasks_completed_[ue_id][frame_id % kFrameWnd][symbol_id_dl] = 0;
            return true;
        }
        return false;
    }

private:
    Config* cfg_;
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd> *num_encode_tasks_completed_; 

    const size_t num_encode_tasks_required_;
};

class PrecodeStatus
{
public:
    PrecodeStatus(Config* cfg) 
    : cfg_(cfg)
    , num_encoded_data_required_(cfg->UE_NUM) 
    {
        
    }

    void receive_encoded_data(size_t frame_id, size_t symbol_id_dl) {
        if (frame_id > max_frame_) {
            for (size_t i = 0; i < kMaxSymbols; i ++) {
                num_encoded_data_received_[frame_id % kFrameWnd][i] = 0;
            }
            max_frame_ = frame_id;
        }
        num_encoded_data_received_[frame_id % kFrameWnd][symbol_id_dl] ++;
    }

    bool received_all_encoded_data(size_t frame_id, size_t symbol_id_dl) {
        if (num_encoded_data_received_[frame_id % kFrameWnd][symbol_id_dl] == 
            num_encoded_data_required_ && frame_id <= max_frame_ && frame_id + kFrameWnd > max_frame_) {
            return true;
        }
        return false;
    }

private:
    Config* cfg_;
    const size_t num_encoded_data_required_;

    std::array<std::array<size_t, kMaxSymbols>, kFrameWnd>
        num_encoded_data_received_;
    size_t max_frame_ = 0;
};