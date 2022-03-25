#pragma once

#include "Symbols.hpp"
#include "config.hpp"
#include "logger.h"
#include "utils.h"
#include "gettime.h"
#include <mutex>
#include <sstream>
#include <vector>

class Diagnosis;

// We use one SharedState object to track packet reception status.
// This object is shared between socket threads and subcarrier workers.
class SharedState {
public:
    SharedState(Config* cfg);

    bool receive_time_iq_pkt(size_t frame_id, size_t symbol_id);
    // When receive a new freq iq packet, record it here
    bool receive_freq_iq_pkt(size_t frame_id, size_t symbol_id, size_t ant_id);
    // When receive a new demod packet, record it here
    bool receive_demod_pkt(size_t ue_id, size_t frame_id, size_t symbol_id_ul, size_t server_id);
    bool receive_demod_pkt_loss_tolerant(size_t ue_id, size_t frame_id, size_t symbol_id_ul, size_t server_id);
    // When receive a new encode packet, record it here
    bool receive_encoded_pkt(size_t frame_id, size_t symbol_id_dl, size_t ue_id);

    bool received_all_time_iq_pkts(size_t frame_id, size_t symbol_id);
    // Check whether all pilot packets are received for a frame
    // used by CSI
    bool received_all_pilots(size_t frame_id);
    // Check whether demodulation can proceed for a symbol in a frame
    bool received_all_data_pkts(size_t frame_id, size_t symbol_id_ul);
    bool received_all_demod_pkts(size_t ue_id, size_t frame_id, size_t symbol_id_ul);
    bool received_all_demod_pkts_loss_tolerant(size_t ue_id, size_t frame_id, size_t symbol_id_ul);
    bool received_all_encoded_pkts(size_t frame_id, size_t symbol_id_dl);

    // Print 
    void print_receiving_encoded_pkts(size_t frame_id, size_t symbol_id_dl);
    void print_receiving_demod_pkts(size_t ue_id, size_t frame_id, size_t symbol_id_ul);

    // Check whether encoding can proceed for a frame
    bool is_encode_ready(size_t frame_id, size_t symbol_id_dl);

    void fft_done(size_t frame_id, size_t symbol_id);
    void zf_done(size_t frame_id, size_t zf_block_id);
    // Mark [num_tasks] demodulation tasks for this frame and symbol as complete
    void demul_done(size_t frame_id, size_t symbol_id_ul, size_t num_tasks);
    // When decoding is done for a frame from one decoder, call this function
    // This function will increase cur_frame when this frame is decoded so that
    // we can move on decoding the next frame and release the resources used by
    // this frame
    void decode_done(size_t frame_id);
    // When precoding is done for a frame from one dosubcarrier worker, call this function
    // This function will increase cur_frame_ when this frame is precoded so that
    // we can move on precoding the next frame and release the resources used by this frame
    bool precode_done(size_t frame_id);
    void encode_done(size_t frame_id, size_t symbol_id);
    // void encode_done(size_t frame_id);

    bool is_fft_tx_ready(size_t frame_id, size_t symbol_id);
    // Return true iff we have completed demodulation for all subcarriers in
    // this symbol have
    bool is_demod_tx_ready(size_t frame_id, size_t symbol_id_ul);
    bool is_encode_tx_ready(size_t frame_id, size_t symbol_id_dl);
    bool is_zf_done(size_t frame_id, size_t zf_block_id);

    // Latency measurement counters for each frame
    uint64_t *frame_start_time_;
    uint64_t *frame_iq_time_;
    uint64_t *frame_sc_time_;
    uint64_t *frame_decode_time_;
    uint64_t *frame_end_time_;

    // cur_frame is the first frame for which decoding is incomplete
    size_t cur_frame_ = 0;

    size_t rru_start_ = false;

private:
    Config *cfg_;
    // TODO: Instead of having all-atomic counter arrays, can we just make
    // the entire class atomic?

    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_time_iq_pkts_ = {};

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

    std::array<size_t, kMaxAntennas> last_frame_symbol_each_ant_ = {};

    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_fft_tasks_completed_ = {};
    // num_demul_tasks_completed[i % kFrameWnd][j] is
    // the number of subcarriers completed for demul tasks in
    // frame i and symbol j
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_demul_tasks_completed_ = {};

    std::array<std::array<bool, kMaxDataSCs/kSCsPerCacheline>, kFrameWnd>
        zf_task_completed_ = {};

    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_encode_tasks_completed_ = {};

    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_demod_pkts_[kMaxUEs];
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_demod_pkts_states_[kMaxUEs];
    std::array<size_t, kMaxServers> demod_next_frame_;
    std::array<size_t, kMaxServers> demod_next_symbol_;
    std::array<size_t, kMaxServers> demod_next_ue_;

    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_encoded_pkts_ = {};
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_encoded_pkts_states_ = {};

    // encode_ready_[i % kFrameWnd] represents whether encoding can proceed
    // for frame i
    std::array<bool, kFrameWnd> encode_ready_;

    // Atomic counter for # completed decode tasks
    // cur_frame will be incremented in all tasks are completed
    std::array<size_t, kFrameWnd> num_decode_tasks_completed_ = {};
    std::array<std::mutex, kFrameWnd> decode_mutex_ = {};
    std::mutex cur_frame_mutex_;

    // Atomic counter for # completed precode tasks
    // cur_frame_ will be incremented in all tasks are completed
    std::array<size_t, kFrameWnd> num_precode_tasks_completed_;
    std::mutex precode_mutex_;

    // The timestamp when last frame was processed (in cycles)
    size_t last_frame_cycles_;
    
    // The CPU frequency (in GHz), used to convert cycles to time
    size_t freq_ghz_;

    // Copies of Config variables
    const size_t num_time_iq_pkts_per_symbol_;
    const size_t num_pilot_pkts_per_frame_;
    const size_t num_pilot_symbols_per_frame_;
    const size_t num_ul_data_symbol_per_frame_;
    const size_t num_pkts_per_symbol_;
    const size_t num_fft_tasks_per_symbol_;
    const size_t num_decode_tasks_per_frame_;
    const size_t num_precode_tasks_per_frame_;
    const size_t num_demul_tasks_required_;
    const size_t num_encode_tasks_required_;
    const size_t num_demod_pkts_per_symbol_per_ue_;
    const size_t num_encoded_pkts_per_symbol_;
    const size_t num_zf_tasks_per_frame_;
    const size_t slot_us_;
    const size_t symbol_num_per_frame_;

    friend class Diagnosis;
};

#if 0
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
#endif