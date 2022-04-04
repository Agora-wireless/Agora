#pragma once

#include "Symbols.hpp"
#include "config.hpp"
#include "logger.h"
#include "utils.h"
#include "gettime.h"
#include <mutex>
#include <sstream>
#include <vector>

class BigStationState
{
public:
    BigStationState(Config* cfg);

    // Packet receiving functions
    bool receive_time_iq_pkt(size_t frame_id, size_t symbol_id, size_t ant_id);
    bool receive_pilot_pkt(size_t frame_id, size_t ant_id);
    bool receive_ul_data_pkt(size_t frame_id, size_t symbol_id_ul, size_t ant_id);
    bool receive_zf_pkt(size_t frame_id, size_t sc_id);
    bool receive_demod_pkt(size_t frame_id, size_t symbol_id_ul, size_t ue_id, size_t sc_len);
    bool receive_encode_pkt(size_t frame_id, size_t symbol_id_dl, size_t ue_id);
    bool receive_precode_pkt(size_t frame_id, size_t symbol_id_dl, size_t ant_id, size_t sc_len);

    // Packet prepare functions
    bool prepare_freq_iq_pkt(size_t frame_id, size_t symbol_id, size_t ant_id);
    bool prepare_zf_pkt(size_t frame_id);
    bool prepare_demod_pkt(size_t frame_id, size_t symbol_id_ul, size_t sc_num);
    bool prepare_encode_pkt(size_t frame_id, size_t symbol_id_dl, size_t ue_id);
    bool prepare_precode_pkt(size_t frame_id, size_t symbol_id_dl, size_t sc_num);

    // Packet receiving checking functions
    bool received_all_time_iq_pkts(size_t frame_id, size_t symbol_id);
    bool received_all_pilot_pkts(size_t frame_id);
    bool received_all_ul_data_pkts(size_t frame_id, size_t symbol_id_ul);
    bool received_all_zf_pkts(size_t frame_id);
    bool received_all_demod_pkts(size_t frame_id, size_t symbol_id_ul);
    bool received_all_encode_pkts(size_t frame_id, size_t symbol_id_dl);
    bool received_all_precode_pkts(size_t frame_id, size_t symbol_id_dl);

    // Packet prepare checking functions
    bool prepared_all_freq_iq_pkts(size_t frame_id, size_t symbol_id);
    bool prepared_all_zf_pkts(size_t frame_id);
    bool prepared_all_demod_pkts(size_t frame_id, size_t symbol_id_ul);
    bool prepared_all_encode_pkts(size_t frame_id, size_t symbol_id_dl);
    bool prepared_all_precode_pkt(size_t frame_id, size_t symbol_id_dl);

    bool decode_done(size_t frame_id);
    bool ifft_done(size_t frame_id);

    // Latency measurement counters for each frame
    uint64_t *frame_start_time_;
    uint64_t *frame_iq_time_;
    uint64_t *frame_fft_time_;
    uint64_t *frame_zf_time_;
    uint64_t *frame_sc_time_;
    uint64_t *frame_coding_time_;
    uint64_t *frame_end_time_;

    size_t cur_frame_ = 0;

    size_t rru_start_ = false;

private:
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_time_iq_pkts_received_ = {};
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_freq_iq_pkts_prepared_ = {};
    std::array<std::atomic<size_t>, kFrameWnd>
        num_pilot_pkts_received_ = {};
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_data_pkts_received_ = {};
    std::array<std::atomic<size_t>, kFrameWnd>
        num_zf_pkts_prepared_ = {};
    std::array<std::atomic<size_t>, kFrameWnd>
        num_zf_pkts_received_ = {};
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_demod_pkts_prepared_ = {};
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_demod_pkts_received_ = {};

    std::array<size_t, kFrameWnd> num_decode_tasks_completed_ = {};
    std::array<std::mutex, kFrameWnd> decode_mutex_ = {};

    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_encode_pkts_prepared_ = {};
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_encode_pkts_received_ = {};
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_precode_pkts_prepared_ = {};
    std::array<std::array<std::atomic<size_t>, kMaxSymbols>, kFrameWnd>
        num_precode_pkts_received_ = {};

    std::array<size_t, kFrameWnd> num_ifft_tasks_completed_ = {};
    std::array<std::mutex, kFrameWnd> ifft_mutex_ = {};

    std::mutex cur_frame_mutex_;
    
    Config* cfg_;
    size_t freq_ghz_;

    const size_t num_time_iq_pkts_per_symbol_;
    const size_t num_pilot_pkts_received_per_frame_;
    const size_t num_freq_iq_pkts_prepared_per_symbol_;
    const size_t num_data_pkts_received_per_symbol_;
    const size_t num_zf_pkts_prepared_per_frame_;
    const size_t num_zf_pkts_received_per_frame_;
    const size_t num_demod_pkts_prepared_per_symbol_;
    const size_t num_demod_pkts_received_per_symbol_;
    const size_t num_decode_tasks_per_frame_;
    const size_t num_encode_pkts_prepared_per_symbol_;
    const size_t num_encode_pkts_received_per_symbol_;
    const size_t num_precode_pkts_prepared_per_symbol_;
    const size_t num_precode_pkts_received_per_symbol_;
    const size_t num_ifft_tasks_per_frame_;
};