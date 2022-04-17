#include "bigstation_counters.hpp"
#include "utils.h"

// BigStationState function implementations

BigStationState::BigStationState(Config *cfg)
    : cfg_(cfg)
    , freq_ghz_(measure_rdtsc_freq())
    , num_time_iq_pkts_per_symbol_((cfg->ant_end - cfg->ant_start) * ceil_divide(cfg->OFDM_CA_NUM, cfg->time_iq_sc_step_size))
    , num_pilot_pkts_received_per_frame_(cfg->BS_ANT_NUM * cfg->num_zf_workers[cfg->bs_server_addr_idx])
    , num_freq_iq_pkts_prepared_per_symbol_(cfg->ant_end - cfg->ant_start)
    , num_data_pkts_received_per_symbol_(cfg->BS_ANT_NUM)
    , num_zf_pkts_prepared_per_frame_(cfg->num_zf_workers[cfg->bs_server_addr_idx])
    , num_zf_pkts_received_per_frame_(cfg->downlink_mode ? (cfg->precode_end - cfg->precode_start) * 
        ceil_divide(cfg->BS_ANT_NUM * cfg->UE_NUM * 2 * sizeof(float), cfg->post_zf_step_size) :
        (cfg->demul_end - cfg->demul_start) * 
        ceil_divide(cfg->BS_ANT_NUM * cfg->UE_NUM * 2 * sizeof(float), cfg->post_zf_step_size)) // TODO: modify this
    , num_demod_pkts_prepared_per_symbol_(cfg->demul_end - cfg->demul_start)
    , num_demod_pkts_received_per_symbol_((cfg->ue_end - cfg->ue_start) * cfg->OFDM_DATA_NUM)
    , num_decode_tasks_per_frame_(cfg->downlink_mode ? 0 : cfg->num_decode_workers[cfg->bs_server_addr_idx])
    , num_encode_pkts_prepared_per_symbol_(cfg->ue_end - cfg->ue_start)
    , num_encode_pkts_received_per_symbol_(cfg->UE_NUM)
    , num_precode_pkts_prepared_per_symbol_(cfg->precode_end - cfg->precode_start)
    , num_precode_pkts_received_per_symbol_((cfg->ant_end - cfg->ant_start) * cfg->OFDM_DATA_NUM)
    , num_ifft_tasks_per_frame_(cfg->downlink_mode ? cfg->num_ifft_workers[cfg->bs_server_addr_idx] : 0)
    , slot_us_(cfg->slot_us)
{
    frame_start_time_ = new uint64_t[cfg->frames_to_test];
    frame_iq_time_ = new uint64_t[cfg->frames_to_test];
    frame_fft_time_ = new uint64_t[cfg->frames_to_test];
    frame_zf_time_ = new uint64_t[cfg->frames_to_test];
    frame_sc_time_ = new uint64_t[cfg->frames_to_test];
    frame_coding_time_ = new uint64_t[cfg->frames_to_test]; 
    frame_end_time_ = new uint64_t[cfg->frames_to_test];
    memset(frame_start_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
    memset(frame_iq_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
    memset(frame_fft_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
    memset(frame_zf_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
    memset(frame_sc_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
    memset(frame_coding_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
    memset(frame_end_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
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

    if (unlikely(!rru_start_)) {
        rru_start_ = true;
    }

    if (unlikely(frame_start_time_[frame_id] == 0)) {
        frame_start_time_[frame_id] = get_us();
    }
    frame_iq_time_[frame_id] = get_us();

    num_time_iq_pkts_received_[frame_id % kFrameWnd][symbol_id] ++;
    if (symbol_id == 0 && num_time_iq_pkts_received_[frame_id % kFrameWnd][symbol_id] == num_time_iq_pkts_per_symbol_) {
        encode_ready_[frame_id % kFrameWnd] = true;
    }
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
    // printf("Recv pilot packets frame %zu ant %zu count %zu required %zu\n", 
    //     frame_id, ant_id, num_pilot_pkts_received_[frame_id % kFrameWnd].load(), 
    //     num_pilot_pkts_received_per_frame_);
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

bool BigStationState::receive_zf_pkt(size_t frame_id, size_t sc_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        MLPD_ERROR(
            "SharedState error: Received zf packet for future "
            "frame %zu beyond frame window (%zu + %zu). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd);
        return false;
    }

    if (cfg_->downlink_mode) {
        size_t sc_start = std::max(sc_id, cfg_->precode_start);
        size_t sc_end = std::min(sc_id + cfg_->UE_NUM, cfg_->precode_end);
        num_zf_pkts_received_[frame_id % kFrameWnd] += (sc_end - sc_start);
    } else {
        size_t sc_start = std::max(sc_id, cfg_->demul_start);
        size_t sc_end = std::min(sc_id + cfg_->UE_NUM, cfg_->demul_end);
        num_zf_pkts_received_[frame_id % kFrameWnd] += (sc_end - sc_start);
    }
    return true;
}

bool BigStationState::receive_demod_pkt(size_t frame_id, size_t symbol_id_ul, size_t ue_id, size_t sc_len)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        MLPD_ERROR(
            "SharedState error: Received demod packet for future "
            "frame %zu beyond frame window (%zu + %zu). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd);
        return false;
    }
    
    num_demod_pkts_received_[frame_id % kFrameWnd][symbol_id_ul] += sc_len;
    // printf("Recv demod packet frame %zu symbol %zu count %zu required %zu\n", frame_id,
    //     symbol_id_ul, num_demod_pkts_received_[frame_id % kFrameWnd][symbol_id_ul].load(),
    //     num_demod_pkts_received_per_symbol_);
    return true;
}

bool BigStationState::receive_encode_pkt(size_t frame_id, size_t symbol_id_dl, size_t ue_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        MLPD_ERROR(
            "SharedState error: Received encode packet for future "
            "frame %zu beyond frame window (%zu + %zu). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd);
        return false;
    }

    num_encode_pkts_received_[frame_id % kFrameWnd][symbol_id_dl] ++;
    return true;
}

bool BigStationState::receive_precode_pkt(size_t frame_id, size_t symbol_id_dl, size_t ant_id, size_t sc_len)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        MLPD_ERROR(
            "SharedState error: Received precode packet for future "
            "frame %zu beyond frame window (%zu + %zu). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd);
        return false;
    }

    num_precode_pkts_received_[frame_id % kFrameWnd][symbol_id_dl] += sc_len;
    // if (symbol_id_dl == 0) {
    //     printf("Receive precode pkt %zu %zu %zu %zu %zu\n", frame_id, symbol_id_dl, ant_id, 
    //         num_precode_pkts_received_[frame_id % kFrameWnd][symbol_id_dl].load(),
    //         num_precode_pkts_received_per_symbol_);
    // }
    return true;
}

// Packet prepare functions
bool BigStationState::prepare_freq_iq_pkt(size_t frame_id, size_t symbol_id, size_t ant_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    num_freq_iq_pkts_prepared_[frame_id % kFrameWnd][symbol_id] ++;
    frame_fft_time_[frame_id] = get_us();
    // printf("Prepare freq iq packet frame %zu symbol %zu count %zu required %zu\n", frame_id,
    //     symbol_id, num_freq_iq_pkts_prepared_[frame_id % kFrameWnd][symbol_id].load(), num_freq_iq_pkts_prepared_per_symbol_);
    return true;
}

bool BigStationState::prepare_zf_pkt(size_t frame_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    num_zf_pkts_prepared_[frame_id % kFrameWnd] ++;
    frame_zf_time_[frame_id] = get_us();
    return true;
}

bool BigStationState::prepare_demod_pkt(size_t frame_id, size_t symbol_id_ul, size_t sc_num)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    num_demod_pkts_prepared_[frame_id % kFrameWnd][symbol_id_ul] += sc_num;
    frame_sc_time_[frame_id] = get_us();
    // printf("Prepare demod packet frame %zu symbol %zu count %zu required %zu\n", frame_id,
    //     symbol_id_ul, num_demod_pkts_prepared_[frame_id % kFrameWnd][symbol_id_ul].load(),
    //     num_demod_pkts_prepared_per_symbol_);
    return true;
}

bool BigStationState::prepare_encode_pkt(size_t frame_id, size_t symbol_id_dl, size_t ue_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    num_encode_pkts_prepared_[frame_id % kFrameWnd][symbol_id_dl] ++;
    frame_coding_time_[frame_id] = get_us();
    return true;
}

bool BigStationState::prepare_precode_pkt(size_t frame_id, size_t symbol_id_dl, size_t sc_num)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    num_precode_pkts_prepared_[frame_id % kFrameWnd][symbol_id_dl] += sc_num;
    frame_sc_time_[frame_id] = get_us();
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

bool BigStationState::received_all_encode_pkts(size_t frame_id, size_t symbol_id_dl)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_encode_pkts_received_[frame_id % kFrameWnd][symbol_id_dl] == num_encode_pkts_received_per_symbol_;
}

bool BigStationState::received_all_precode_pkts(size_t frame_id, size_t symbol_id_dl)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_precode_pkts_received_[frame_id % kFrameWnd][symbol_id_dl] == num_precode_pkts_received_per_symbol_;
}

// Packet prepare checking functions
bool BigStationState::prepared_all_freq_iq_pkts(size_t frame_id, size_t symbol_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_freq_iq_pkts_prepared_[frame_id % kFrameWnd][symbol_id] == num_freq_iq_pkts_prepared_per_symbol_;
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

bool BigStationState::prepared_all_encode_pkts(size_t frame_id, size_t symbol_id_dl)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_encode_pkts_prepared_[frame_id % kFrameWnd][symbol_id_dl] == num_encode_pkts_prepared_per_symbol_;
}

bool BigStationState::prepared_all_precode_pkt(size_t frame_id, size_t symbol_id_dl)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_precode_pkts_prepared_[frame_id % kFrameWnd][symbol_id_dl] == num_precode_pkts_prepared_per_symbol_;
}

bool BigStationState::decode_done(size_t frame_id)
{
    rt_assert(frame_id < cur_frame_ + kFrameWnd && frame_id >= cur_frame_, "Wrong completed decode task!");
    bool cont = false;
    decode_mutex_[frame_id % kFrameWnd].lock();
    num_decode_tasks_completed_[frame_id % kFrameWnd]++;
    cont = (num_decode_tasks_completed_[frame_id % kFrameWnd] == num_decode_tasks_per_frame_);
    decode_mutex_[frame_id % kFrameWnd].unlock();
    frame_end_time_[frame_id] = get_us();

    if (unlikely(cont)) {
        cur_frame_mutex_.lock();
        while (num_decode_tasks_completed_[cur_frame_ % kFrameWnd] == num_decode_tasks_per_frame_) {
            num_decode_tasks_completed_[(cur_frame_) % kFrameWnd] = 0;
            size_t frame_slot = (cur_frame_) % kFrameWnd;
            for (size_t j = 0; j < kMaxSymbols; j++) {
                num_time_iq_pkts_received_[frame_slot][j] = 0;
            }
            for (size_t j = 0; j < kMaxSymbols; j++) {
                num_freq_iq_pkts_prepared_[frame_slot][j] = 0;
            }
            num_pilot_pkts_received_[frame_slot] = 0;
            for (size_t j = 0; j < kMaxSymbols; j++) {
                num_data_pkts_received_[frame_slot][j] = 0;
            }
            num_zf_pkts_prepared_[frame_slot] = 0;
            num_zf_pkts_received_[frame_slot] = 0;
            for (size_t i = 0; i < kMaxSymbols; i++) {
                num_demod_pkts_prepared_[frame_slot][i] = 0;
            }
            for (size_t i = 0; i < kMaxSymbols; i++) {
                num_demod_pkts_received_[frame_slot][i] = 0;
            }
            cur_frame_ ++;
        }
        cur_frame_mutex_.unlock();
    }

    return true;
}

bool BigStationState::ifft_done(size_t frame_id)
{
    rt_assert(frame_id < cur_frame_ + kFrameWnd && frame_id >= cur_frame_, "Wrong completed ifft task!");
    bool cont = false;
    ifft_mutex_[frame_id % kFrameWnd].lock();
    num_ifft_tasks_completed_[frame_id % kFrameWnd]++;
    cont = (num_ifft_tasks_completed_[frame_id % kFrameWnd] == num_ifft_tasks_per_frame_);
    ifft_mutex_[frame_id % kFrameWnd].unlock();
    frame_end_time_[frame_id] = get_us();

    if (unlikely(cont)) {
        cur_frame_mutex_.lock();
        while (num_ifft_tasks_completed_[cur_frame_ % kFrameWnd] == num_ifft_tasks_per_frame_) {
            num_ifft_tasks_completed_[(cur_frame_) % kFrameWnd] = 0;
            size_t frame_slot = (cur_frame_) % kFrameWnd;
            for (size_t j = 0; j < kMaxSymbols; j++) {
                num_time_iq_pkts_received_[frame_slot][j] = 0;
            }
            for (size_t j = 0; j < kMaxSymbols; j++) {
                num_freq_iq_pkts_prepared_[frame_slot][j] = 0;
            }
            num_pilot_pkts_received_[frame_slot] = 0;
            num_zf_pkts_prepared_[frame_slot] = 0;
            num_zf_pkts_received_[frame_slot] = 0;
            for (size_t i = 0; i < kMaxSymbols; i++) {
                num_precode_pkts_prepared_[frame_slot][i] = 0;
            }
            for (size_t i = 0; i < kMaxSymbols; i++) {
                num_precode_pkts_received_[frame_slot][i] = 0;
            }
            for (size_t i = 0; i < kMaxSymbols; i ++) {
                num_encode_pkts_prepared_[frame_slot][i] = 0;
            }
            for (size_t i = 0; i < kMaxSymbols; i ++) {
                num_encode_pkts_received_[frame_slot][i] = 0;
            }
            encode_ready_[frame_slot] = false;
            cur_frame_ ++;
        }
        cur_frame_mutex_.unlock();
    }

    return true;
}

bool BigStationState::is_encode_ready(size_t frame_id, size_t symbol_id_dl) {
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return encode_ready_[frame_id % kFrameWnd] && 
        (get_us() - frame_start_time_[frame_id]) >= (symbol_id_dl + 1) * slot_us_ / cfg_->symbol_num_perframe;
}