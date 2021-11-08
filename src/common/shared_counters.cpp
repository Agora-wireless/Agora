#include "shared_counters.hpp"
#include "utils.h"

SharedState::SharedState(Config* cfg)
    : last_frame_cycles_(worker_rdtsc())
    , freq_ghz_(measure_rdtsc_freq())
    , num_time_iq_pkts_per_symbol_(cfg->get_num_ant_to_process())
    , num_pilot_pkts_per_frame_(
            cfg->pilot_symbol_num_perframe * cfg->BS_ANT_NUM)
    , num_pilot_symbols_per_frame_(cfg->pilot_symbol_num_perframe)
    , num_ul_data_symbol_per_frame_(cfg->ul_data_symbol_num_perframe)
    , num_pkts_per_symbol_(cfg->BS_ANT_NUM)
    , num_fft_tasks_per_symbol_(cfg->get_num_ant_to_process())
    , num_decode_tasks_per_frame_(cfg->use_central_scheduler && cfg->use_general_worker ? 1 : cfg->decode_thread_num)
    , num_precode_tasks_per_frame_((cfg->get_num_sc_to_process() + cfg->subcarrier_block_size - 1) / cfg->subcarrier_block_size)
    , num_demul_tasks_required_(cfg->use_general_worker ? ceil_divide(cfg->get_num_sc_to_process(), cfg->demul_block_size) :
        (ceil_divide(cfg->get_num_sc_to_process(), cfg->subcarrier_block_size) - 1) * ceil_divide(cfg->subcarrier_block_size, cfg->demul_block_size) + 
        ceil_divide((cfg->get_num_sc_to_process() - 1) % cfg->subcarrier_block_size + 1, cfg->demul_block_size))
    , num_demod_pkts_per_symbol_per_ue_(cfg->bs_server_addr_list.size())
    , num_zf_tasks_per_frame_(cfg->get_num_sc_to_process() / cfg->zf_block_size)
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
    for (size_t i = 0; i < kFrameWnd; i++) {
        for (size_t j = 0; j < num_zf_tasks_per_frame_; j++) {
            zf_task_completed_[i][j] = false;
        }
    }
}

bool SharedState::receive_time_iq_pkt(size_t frame_id, size_t symbol_id)
{
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedCounters SharedState error: Received freq iq packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }

    const size_t frame_slot = frame_id % kFrameWnd;
    num_time_iq_pkts_[frame_slot][symbol_id] ++;
    if (num_time_iq_pkts_[frame_slot][symbol_id] == num_time_iq_pkts_per_symbol_) {
        MLPD_INFO("SharedCounters: received all time iq packets in frame: %u, symbol: %u\n", frame_id, symbol_id);
    }
    return true;
}

bool SharedState::receive_freq_iq_pkt(size_t frame_id, size_t symbol_id)
{
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedCounters SharedState error: Received freq iq packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }

    if (unlikely(frame_start_time_[frame_id] == 0)) {
        frame_start_time_[frame_id] = get_ns();
    }

    const size_t frame_slot = frame_id % kFrameWnd;
    num_pkts_[frame_slot]++;
    encode_ready_[frame_slot] = true;
    if (num_pkts_[frame_slot]
        == num_pkts_per_symbol_
            * (num_pilot_symbols_per_frame_ + num_ul_data_symbol_per_frame_)) {
        MLPD_INFO("SharedCounters: received all packets in frame: %u. "
                "Pilot pkts = %zu of %zu\n",
            frame_id, num_pilot_pkts_[frame_slot].load(),
            num_pilot_pkts_per_frame_);
        frame_iq_time_[frame_id] = get_ns();
    }

    if (symbol_id < num_pilot_symbols_per_frame_) {
        num_pilot_pkts_[frame_slot]++;
        if (num_pilot_pkts_[frame_slot] == num_pilot_pkts_per_frame_) {
            MLPD_INFO("SharedCounters: received all pilots in frame: %u\n", frame_id);
        }
    } else {
        num_data_pkts_[frame_slot][symbol_id - num_pilot_symbols_per_frame_]++;
    }

    return true;
}

bool SharedState::receive_demod_pkt(size_t ue_id, size_t frame_id, size_t symbol_id_ul)
{
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedCounters SharedState error: Received demod packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }
    num_demod_pkts_[ue_id][frame_id % kFrameWnd][symbol_id_ul]++;
    if (num_demod_pkts_[ue_id][frame_id % kFrameWnd][symbol_id_ul] == num_demod_pkts_per_symbol_per_ue_) {
        MLPD_INFO("SharedCounters: received all demod packets in frame: %u, ue: %u, symbol: %u\n", frame_id, ue_id, symbol_id_ul);
    }
    return true;
}

bool SharedState::received_all_time_iq_pkts(size_t frame_id, size_t symbol_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_time_iq_pkts_[frame_id % kFrameWnd][symbol_id]
        == num_time_iq_pkts_per_symbol_;
}

bool SharedState::received_all_pilots(size_t frame_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_pilot_pkts_[frame_id % kFrameWnd]
        == num_pilot_pkts_per_frame_;
}

bool SharedState::received_all_data_pkts(size_t frame_id, size_t symbol_id_ul)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return num_data_pkts_[frame_id % kFrameWnd][symbol_id_ul]
        == num_pkts_per_symbol_;
}

bool SharedState::received_all_demod_pkts(
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

bool SharedState::is_encode_ready(size_t frame_id) {
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return encode_ready_[frame_id % kFrameWnd];
}

void SharedState::fft_done(size_t frame_id, size_t symbol_id)
{
    rt_assert(frame_id >= cur_frame_ && frame_id < cur_frame_ + kFrameWnd,
        "Complete a wrong frame in fft!");
    num_fft_tasks_completed_[frame_id % kFrameWnd][symbol_id] ++;
    if (num_fft_tasks_completed_[frame_id % kFrameWnd][symbol_id] == num_fft_tasks_per_symbol_) {
        MLPD_INFO("SharedCounters: FFT done frame: %u, symbol: %u\n", frame_id, symbol_id);
    }
}

void SharedState::zf_done(size_t frame_id, size_t zf_block_id)
{
    rt_assert(frame_id >= cur_frame_ && frame_id < cur_frame_ + kFrameWnd,
        "Complete a wrong frame in fft!");
    zf_task_completed_[frame_id % kFrameWnd][zf_block_id] = true;
}

void SharedState::demul_done(size_t frame_id, size_t symbol_id_ul, size_t num_tasks)
{
    rt_assert(frame_id >= cur_frame_ && frame_id < cur_frame_ + kFrameWnd,
        "Complete a wrong frame in demul!");
    num_demul_tasks_completed_[frame_id % kFrameWnd][symbol_id_ul]
        += num_tasks;
    if (num_demul_tasks_completed_[frame_id % kFrameWnd][symbol_id_ul] == num_demul_tasks_required_) {
        MLPD_INFO("SharedCounters: Demul done frame: %u, symbol: %u\n", frame_id, symbol_id_ul);
    }
}

// When decoding is done for a frame from one decoder, call this function
// This function will increase cur_frame when this frame is decoded so that
// we can move on decoding the next frame and release the resources used by
// this frame
void SharedState::decode_done(size_t frame_id)
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
            for (size_t i = 0; i < kMaxSymbols; i++) {
                num_time_iq_pkts_[frame_slot][i] = 0;
            }
            for (size_t i = 0; i < kMaxSymbols; i++) {
                num_fft_tasks_completed_[frame_slot][i] = 0;
            }
            for (size_t i = 0; i < num_zf_tasks_per_frame_; i++) {
                zf_task_completed_[frame_slot][i] = false;
            }
            MLPD_INFO("Main thread: Decode done frame: %lu, for %.2lfms\n", cur_frame_ - 1, cycles_to_ms(cur_cycle - last_frame_cycles_, freq_ghz_));
            last_frame_cycles_ = cur_cycle;
        }
        cur_frame_mutex_.unlock();
    }
}

void SharedState::precode_done(size_t frame_id)
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

bool SharedState::is_fft_tx_ready(size_t frame_id, size_t symbol_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    if (num_fft_tasks_completed_[frame_id % kFrameWnd][symbol_id]
        == num_fft_tasks_per_symbol_) {
        return true;
    } 
    return false;
}

bool SharedState::is_demod_tx_ready(size_t frame_id, size_t symbol_id_ul)
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

bool SharedState::is_zf_done(size_t frame_id, size_t zf_block_id)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return zf_task_completed_[frame_id % kFrameWnd][zf_block_id];
}