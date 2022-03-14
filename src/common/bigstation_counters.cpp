#include "bigstation_counters.hpp"
#include "utils.h"

// BigStationState function implementations

BigStationState::BigStationState(Config *cfg)
    : freq_ghz_(measure_rdtsc_freq())
    , num_time_iq_pkts_per_symbol_(cfg->ant_end - cfg->ant_start)
    , num_pilot_pkts_received_per_frame_(cfg->BS_ANT_NUM)
    , num_freq_iq_pkts_prepared_per_symbol_(cfg->ant_end - cfg->ant_start)
    , num_data_pkts_received_per_symbol_(cfg->BS_ANT_NUM)
    , num_zf_pkts_prepared_per_frame_(cfg->zf_end - cfg->zf_start)
    , num_zf_pkts_received_per_frame_(cfg->demul_end - cfg->demul_start)
    , num_demod_pkts_prepared_per_symbol_(cfg->demul_end - cfg->demul_start)
    , num_demod_pkts_received_per_symbol_(cfg->ue_end - cfg->ue_start)
{

}

// Packet receiving functions
bool BigStationState::receive_time_iq_pkt(size_t frame_id, size_t symbol_id, size_t ant_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        MLPD_ERROR(
            "SharedState error: Received time iq packet for future "
            "frame %zu beyond frame window (%zu + %zu). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd);
        return false;
    }
    num_time_iq_pkts_received_[frame_id % kFrameWnd][symbol_id] ++;
    return true;
}

bool BigStationState::receive_pilot_pkt(size_t frame_id, size_t ant_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        MLPD_ERROR(
            "SharedState error: Received pilot packet for future "
            "frame %zu beyond frame window (%zu + %zu). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd);
        return false;
    }
    num_pilot_pkts_received_[frame_id % kFrameWnd] ++;
    return true;
}

bool BigStationState::receive_ul_data_pkt(size_t frame_id, size_t symbol_id_ul, size_t ant_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        MLPD_ERROR(
            "SharedState error: Received ul data packet for future "
            "frame %zu beyond frame window (%zu + %zu). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd);
        return false;
    }
    num_data_pkts_received_[frame_id % kFrameWnd][symbol_id_ul] ++;
    return true;
}

bool BigStationState::receive_zf_pkt(size_t frame_id, size_t ant_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        MLPD_ERROR(
            "SharedState error: Received zf packet for future "
            "frame %zu beyond frame window (%zu + %zu). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd);
        return false;
    }
    num_zf_pkts_received_[frame_id % kFrameWnd] ++;
    return true;
}

bool BigStationState::receive_demod_pkt(size_t frame_id, size_t symbol_id_ul, size_t ant_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        MLPD_ERROR(
            "SharedState error: Received demod packet for future "
            "frame %zu beyond frame window (%zu + %zu). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd);
        return false;
    }
    num_demod_pkts_received_[frame_id % kFrameWnd][symbol_id_ul] ++;
    return true;
}

// Packet prepare functions
bool BigStationState::prepare_freq_iq_pkt(size_t frame_id, size_t symbol_id_ul, size_t ant_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    num_data_pkts_prepared_[frame_id % kFrameWnd][symbol_id_ul] ++;
    return true;
}

bool BigStationState::prepare_zf_pkt(size_t frame_id, size_t ant_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    num_zf_pkts_prepared_[frame_id % kFrameWnd] ++;
    return true;
}

bool BigStationState::prepare_demod_pkt(size_t frame_id, size_t symbol_id_ul, size_t ant_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    num_demod_pkts_prepared_[frame_id % kFrameWnd][symbol_id_ul] ++;
    return true;
}

// Packet receiving checking functions
bool BigStationState::received_all_time_iq_pkts(size_t frame_id, size_t symbol_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_time_iq_pkts_received_[frame_id % kFrameWnd][symbol_id] == num_time_iq_pkts_per_symbol_;
}

bool BigStationState::received_all_pilot_pkts(size_t frame_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_pilot_pkts_received_[frame_id % kFrameWnd] == num_pilot_pkts_received_per_frame_;
}

bool BigStationState::received_all_ul_data_pkts(size_t frame_id, size_t symbol_id_ul)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_data_pkts_received_[frame_id % kFrameWnd][symbol_id_ul] == num_data_pkts_received_per_symbol_;
}

bool BigStationState::received_all_zf_pkts(size_t frame_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_zf_pkts_received_[frame_id % kFrameWnd] == num_zf_pkts_received_per_frame_;
}

bool BigStationState::received_all_demod_pkts(size_t frame_id, size_t symbol_id_ul)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_demod_pkts_received_[frame_id % kFrameWnd][symbol_id_ul] == num_demod_pkts_received_per_symbol_;
}

// Packet prepare checking functions
bool BigStationState::prepared_all_freq_iq_pkts(size_t frame_id, size_t symbol_id_ul)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_data_pkts_prepared_[frame_id % kFrameWnd][symbol_id_ul] == num_freq_iq_pkts_prepared_per_symbol_;
}

bool BigStationState::prepared_all_zf_pkts(size_t frame_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_zf_pkts_prepared_[frame_id % kFrameWnd] == num_zf_pkts_prepared_per_frame_;
}

bool BigStationState::prepared_all_demod_pkts(size_t frame_id, size_t symbol_id_ul)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_demod_pkts_prepared_[frame_id % kFrameWnd][symbol_id_ul] == num_demod_pkts_prepared_per_symbol_;
}