#include "shared_counters.hpp"
#include "utils.h"

SharedState::SharedState(Config* cfg)
    : cfg_(cfg)
    , last_frame_cycles_(worker_rdtsc())
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
        cfg->get_num_sc_to_process())
    , num_encode_tasks_required_(cfg->get_num_ues_to_process())
    , num_demod_pkts_per_symbol_per_ue_(cfg->bs_server_addr_list.size())
    , num_encoded_pkts_per_symbol_(cfg->UE_NUM)
    , num_zf_tasks_per_frame_(cfg->get_num_sc_to_process() / cfg->zf_block_size)
    , slot_us_(cfg->slot_us)
    , symbol_num_per_frame_(cfg->symbol_num_perframe)
{
    frame_start_time_ = new uint64_t[cfg->frames_to_test];
    frame_iq_time_ = new uint64_t[cfg->frames_to_test];
    frame_sc_time_ = new uint64_t[cfg->frames_to_test];
    frame_coding_time_ = new uint64_t[cfg->frames_to_test]; 
    frame_end_time_ = new uint64_t[cfg->frames_to_test];
    memset(frame_start_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
    memset(frame_iq_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
    memset(frame_sc_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
    memset(frame_coding_time_, 0, sizeof(uint64_t) * cfg->frames_to_test);
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
    for (size_t i = 0; i < cfg->bs_server_addr_list.size(); i ++) {
        demod_next_ue_[i] = cfg->ue_start;
    }
}

bool SharedState::receive_time_iq_pkt(size_t frame_id, size_t symbol_id)
{
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedState error: Received time iq packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }

    if (unlikely(!rru_start_)) {
        rru_start_ = true;
    }

    const size_t frame_slot = frame_id % kFrameWnd;
    num_time_iq_pkts_[frame_slot][symbol_id] ++;
    if (num_time_iq_pkts_[frame_slot][symbol_id] == num_time_iq_pkts_per_symbol_) {
        MLPD_INFO("SharedCounters: received all time iq packets in frame: %u, symbol: %u\n", frame_id, symbol_id);
    }
    return true;
}

bool SharedState::receive_freq_iq_pkt(size_t frame_id, size_t symbol_id, size_t ant_id)
{
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedState error: Received freq iq packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }

    if (unlikely(!rru_start_)) {
        rru_start_ = true;
    }

    if (unlikely(frame_start_time_[frame_id] == 0)) {
        frame_start_time_[frame_id] = get_us();
    }

    const size_t frame_slot = frame_id % kFrameWnd;
    num_pkts_[frame_slot]++;
    if (num_pkts_[frame_slot]
        == num_pkts_per_symbol_
            * (num_pilot_symbols_per_frame_ + num_ul_data_symbol_per_frame_)) {
        MLPD_INFO("SharedCounters: received all packets in frame: %u. "
                "Pilot pkts = %zu of %zu\n",
            frame_id, num_pilot_pkts_[frame_slot].load(),
            num_pilot_pkts_per_frame_);
        frame_iq_time_[frame_id] = get_us();
    }

    if (symbol_id < num_pilot_symbols_per_frame_) {
        num_pilot_pkts_[frame_slot]++;
        if (num_pilot_pkts_[frame_slot] == num_pilot_pkts_per_frame_) {
            MLPD_INFO("SharedCounters: received all pilots in frame: %u\n", frame_id);
            encode_ready_[frame_slot] = true;
        }
    } else {
        num_data_pkts_[frame_slot][symbol_id - num_pilot_symbols_per_frame_]++;
    }

    return true;
}

bool SharedState::receive_freq_iq_pkt_loss_tolerant(size_t frame_id, size_t symbol_id, size_t ant_id)
{
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedState error: Received freq iq packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }

    if (unlikely(!rru_start_)) {
        rru_start_ = true;
    }

    if (unlikely(frame_start_time_[frame_id] == 0)) {
        frame_start_time_[frame_id] = get_us();
    }

    // Handle pkt loss for freq iq packets
    size_t recvd_symbol_num = num_pilot_symbols_per_frame_ + num_ul_data_symbol_per_frame_;
    if (frame_id != freq_iq_next_frame_[ant_id]) {
        if (frame_id > freq_iq_next_frame_[ant_id]) {
            for (size_t sid = freq_iq_next_symbol_[ant_id]; sid < recvd_symbol_num; sid ++) {
                num_pkts_[freq_iq_next_frame_[ant_id] % kFrameWnd] ++;
                if (sid < cfg_->pilot_symbol_num_perframe) {
                    num_pilot_pkts_[freq_iq_next_frame_[ant_id] % kFrameWnd] ++;
                    if (num_pilot_pkts_[freq_iq_next_frame_[ant_id] % kFrameWnd] == num_pilot_pkts_per_frame_) {
                        MLPD_INFO("SharedCounters: received all pilots in frame: %u\n", freq_iq_next_frame_[ant_id]);
                        encode_ready_[freq_iq_next_frame_[ant_id] % kFrameWnd] = true;
                    }
                } else {
                    num_data_pkts_[freq_iq_next_frame_[ant_id] % kFrameWnd][sid - cfg_->pilot_symbol_num_perframe] ++;
                }
            }
            if (num_pkts_[freq_iq_next_frame_[ant_id] % kFrameWnd] == num_pkts_per_symbol_
                    * (num_pilot_symbols_per_frame_ + num_ul_data_symbol_per_frame_)) {
                MLPD_INFO("SharedCounters: received all packets in frame: %u. "
                        "Pilot pkts = %zu of %zu\n",
                    freq_iq_next_frame_[ant_id], num_pilot_pkts_[freq_iq_next_frame_[ant_id] % kFrameWnd].load(),
                    num_pilot_pkts_per_frame_);
                frame_iq_time_[freq_iq_next_frame_[ant_id]] = get_us();
            }
            for (size_t fid = freq_iq_next_frame_[ant_id] + 1; fid < frame_id; fid ++) {
               for (size_t sid = 0; sid < recvd_symbol_num; sid ++) {
                    num_pkts_[fid % kFrameWnd] ++;
                    if (sid < cfg_->pilot_symbol_num_perframe) {
                        num_pilot_pkts_[fid % kFrameWnd] ++;
                        if (num_pilot_pkts_[fid % kFrameWnd] == num_pilot_pkts_per_frame_) {
                            MLPD_INFO("SharedCounters: received all pilots in frame: %u\n", fid);
                            encode_ready_[fid % kFrameWnd] = true;
                        }
                    } else {
                        num_data_pkts_[fid % kFrameWnd][sid - cfg_->pilot_symbol_num_perframe] ++;
                    }
                }
                if (num_pkts_[fid % kFrameWnd] == num_pkts_per_symbol_
                        * (num_pilot_symbols_per_frame_ + num_ul_data_symbol_per_frame_)) {
                    MLPD_INFO("SharedCounters: received all packets in frame: %u. "
                            "Pilot pkts = %zu of %zu\n",
                        fid, num_pilot_pkts_[fid % kFrameWnd].load(),
                        num_pilot_pkts_per_frame_);
                    frame_iq_time_[fid] = get_us();
                }
            }
            for (size_t sid = 0; sid < symbol_id; sid ++) {
                num_pkts_[frame_id % kFrameWnd] ++;
                if (sid < cfg_->pilot_symbol_num_perframe) {
                    num_pilot_pkts_[frame_id % kFrameWnd] ++;
                    if (num_pilot_pkts_[frame_id % kFrameWnd] == num_pilot_pkts_per_frame_) {
                        MLPD_INFO("SharedCounters: received all pilots in frame: %u\n", frame_id);
                        encode_ready_[frame_id % kFrameWnd] = true;
                    }
                } else {
                    num_data_pkts_[frame_id % kFrameWnd][sid - cfg_->pilot_symbol_num_perframe] ++;
                }
            }
            if (num_pkts_[frame_id % kFrameWnd] == num_pkts_per_symbol_
                    * (num_pilot_symbols_per_frame_ + num_ul_data_symbol_per_frame_)) {
                MLPD_INFO("SharedCounters: received all packets in frame: %u. "
                        "Pilot pkts = %zu of %zu\n",
                    frame_id, num_pilot_pkts_[frame_id % kFrameWnd].load(),
                    num_pilot_pkts_per_frame_);
                frame_iq_time_[frame_id] = get_us();
            }
        }
    } else if (symbol_id != freq_iq_next_symbol_[ant_id]) {
        if (symbol_id > freq_iq_next_symbol_[ant_id]) {
            for (size_t sid = freq_iq_next_symbol_[ant_id]; sid < symbol_id; sid ++) {
                num_pkts_[frame_id % kFrameWnd] ++;
                if (sid < cfg_->pilot_symbol_num_perframe) {
                    num_pilot_pkts_[frame_id % kFrameWnd] ++;
                    if (num_pilot_pkts_[frame_id % kFrameWnd] == num_pilot_pkts_per_frame_) {
                        MLPD_INFO("SharedCounters: received all pilots in frame: %u\n", frame_id);
                        encode_ready_[frame_id % kFrameWnd] = true;
                    }
                } else {
                    num_data_pkts_[frame_id % kFrameWnd][sid - cfg_->pilot_symbol_num_perframe] ++;
                }
            }
            if (num_pkts_[frame_id % kFrameWnd] == num_pkts_per_symbol_
                    * (num_pilot_symbols_per_frame_ + num_ul_data_symbol_per_frame_)) {
                MLPD_INFO("SharedCounters: received all packets in frame: %u. "
                        "Pilot pkts = %zu of %zu\n",
                    frame_id, num_pilot_pkts_[frame_id % kFrameWnd].load(),
                    num_pilot_pkts_per_frame_);
                frame_iq_time_[frame_id] = get_us();
            }
        }
    }

    const size_t frame_slot = frame_id % kFrameWnd;
    num_pkts_[frame_slot]++;
    if (num_pkts_[frame_slot]
        == num_pkts_per_symbol_
            * (num_pilot_symbols_per_frame_ + num_ul_data_symbol_per_frame_)) {
        MLPD_INFO("SharedCounters: received all packets in frame: %u. "
                "Pilot pkts = %zu of %zu\n",
            frame_id, num_pilot_pkts_[frame_slot].load(),
            num_pilot_pkts_per_frame_);
        frame_iq_time_[frame_id] = get_us();
    }

    if (symbol_id < num_pilot_symbols_per_frame_) {
        num_pilot_pkts_[frame_slot]++;
        if (num_pilot_pkts_[frame_slot] == num_pilot_pkts_per_frame_) {
            MLPD_INFO("SharedCounters: received all pilots in frame: %u\n", frame_id);
            encode_ready_[frame_slot] = true;
        }
    } else {
        num_data_pkts_[frame_slot][symbol_id - num_pilot_symbols_per_frame_]++;
    }

    size_t next_symbol = symbol_id + 1;
    size_t next_frame = frame_id;
    if (next_symbol == recvd_symbol_num) {
        next_symbol = 0;
        next_frame ++;
    }
    freq_iq_next_frame_[ant_id] = next_frame;
    freq_iq_next_symbol_[ant_id] = next_symbol;
    
    return true;
}

bool SharedState::receive_demod_pkt(size_t ue_id, size_t frame_id, size_t symbol_id_ul, size_t server_id)
{
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedState error: Received demod packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }

    num_demod_pkts_[ue_id][frame_id % kFrameWnd][symbol_id_ul]++;
    if (unlikely(((num_demod_pkts_states_[ue_id][frame_id % kFrameWnd][symbol_id_ul] >> server_id) & 1) == 1)) {
        printf("ERROR! Duplicate demod packet for frame %zu symbol %zu ue %zu server %zu\n", frame_id, symbol_id_ul, ue_id, server_id);
        exit(0);
    }
    if (unlikely(server_id >= num_demod_pkts_per_symbol_per_ue_)) {
        printf("ERROR! Server ID is larger than limit (%zu >= %zu)\n", server_id, num_demod_pkts_per_symbol_per_ue_);
        exit(0);
    }
    num_demod_pkts_states_[ue_id][frame_id % kFrameWnd][symbol_id_ul] ^= (1 << server_id);
    if (num_demod_pkts_[ue_id][frame_id % kFrameWnd][symbol_id_ul] == num_demod_pkts_per_symbol_per_ue_) {
        MLPD_INFO("SharedCounters: received all demod packets in frame: %u, ue: %u, symbol: %u\n", frame_id, ue_id, symbol_id_ul);
    }
    return true;
}

bool SharedState::receive_demod_pkt_loss_tolerant(size_t ue_id, size_t frame_id, size_t symbol_id_ul, size_t server_id) {
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedState error: Received demod packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }

    // Handle pkt loss for demod pkts
    if (frame_id != demod_next_frame_[server_id]) {
        if (frame_id > demod_next_frame_[server_id]) {
            for (size_t uid = demod_next_ue_[server_id]; uid < cfg_->ue_end; uid ++) {
                num_demod_pkts_states_[uid][demod_next_frame_[server_id] % kFrameWnd][demod_next_symbol_[server_id]] ^= (1 << server_id);
            }
            for (size_t sid = demod_next_symbol_[server_id] + 1; sid < cfg_->ul_data_symbol_num_perframe; sid ++) {
                for (size_t uid = cfg_->ue_start; uid < cfg_->ue_end; uid ++) {
                    num_demod_pkts_states_[uid][demod_next_frame_[server_id] % kFrameWnd][sid] ^= (1 << server_id);
                }
            }
            for (size_t fid = demod_next_frame_[server_id] + 1; fid < frame_id; fid ++) {
                for (size_t sid = 0; sid < cfg_->ul_data_symbol_num_perframe; sid ++) {
                    for (size_t uid = cfg_->ue_start; uid < cfg_->ue_end; uid ++) {
                        num_demod_pkts_states_[uid][fid % kFrameWnd][sid] ^= (1 << server_id);
                    }
                } 
            }
            for (size_t sid = 0; sid < symbol_id_ul; sid ++) {
                for (size_t uid = cfg_->ue_start; uid < cfg_->ue_end; uid ++) {
                    num_demod_pkts_states_[uid][frame_id % kFrameWnd][sid] ^= (1 << server_id);
                }
            }
            for (size_t uid = cfg_->ue_start; uid < ue_id; uid ++) {
                num_demod_pkts_states_[uid][frame_id % kFrameWnd][symbol_id_ul] ^= (1 << server_id);
            }
        }
    } else if (symbol_id_ul != demod_next_symbol_[server_id]) {
        if (symbol_id_ul > demod_next_symbol_[server_id]) {
            for (size_t uid = demod_next_ue_[server_id]; uid < cfg_->ue_end; uid ++) {
                num_demod_pkts_states_[uid][frame_id % kFrameWnd][demod_next_symbol_[server_id]] ^= (1 << server_id);
            }
            for (size_t sid = demod_next_symbol_[server_id] + 1; sid < symbol_id_ul; sid ++) {
                for (size_t uid = cfg_->ue_start; uid < cfg_->ue_end; uid ++) {
                    num_demod_pkts_states_[uid][frame_id % kFrameWnd][sid] ^= (1 << server_id);
                }
            }
            for (size_t uid = cfg_->ue_start; uid < ue_id; uid ++) {
                num_demod_pkts_states_[uid][frame_id % kFrameWnd][symbol_id_ul] ^= (1 << server_id);
            }
        }
    } else if (ue_id != demod_next_ue_[server_id]) {
        if (ue_id > demod_next_ue_[server_id]) {
            for (size_t uid = demod_next_ue_[server_id]; uid < ue_id; uid ++) {
                num_demod_pkts_states_[uid][frame_id % kFrameWnd][symbol_id_ul] ^= (1 << server_id);
            }
        }
    }
    num_demod_pkts_states_[ue_id][frame_id % kFrameWnd][symbol_id_ul] ^= (1 << server_id);

    size_t next_ue = ue_id + 1;
    size_t next_symbol = symbol_id_ul;
    size_t next_frame = frame_id;
    if (next_ue == cfg_->ue_end) {
        next_ue = cfg_->ue_start;
        next_symbol ++;
        if (next_symbol == cfg_->ul_data_symbol_num_perframe) {
            next_symbol = 0;
            next_frame ++;
        }
    }
    demod_next_frame_[server_id] = next_frame;
    demod_next_symbol_[server_id] = next_symbol;
    demod_next_ue_[server_id] = next_ue;
    
    return true;
}

bool SharedState::receive_encoded_pkt(size_t frame_id, size_t symbol_id_dl, size_t ue_id)
{
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedState error: Received encoded packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }
    num_encoded_pkts_[frame_id % kFrameWnd][symbol_id_dl]++;
    if (unlikely(((num_encoded_pkts_states_[frame_id % kFrameWnd][symbol_id_dl] >> ue_id) & 1) == 1)) {
        printf("ERROR! Duplicate encoded packet for frame %zu symbol %zu ue %zu\n", frame_id, symbol_id_dl, ue_id);
        exit(0);
    }
    if (unlikely(ue_id >= num_encoded_pkts_per_symbol_)) {
        printf("ERROR! UE ID is larger than limit (%zu >= %zu)\n", ue_id, num_encoded_pkts_per_symbol_);
        exit(0);
    }
    num_encoded_pkts_states_[frame_id % kFrameWnd][symbol_id_dl] ^= (1 << ue_id);
    if (num_encoded_pkts_[frame_id % kFrameWnd][symbol_id_dl] == num_encoded_pkts_per_symbol_) {
        MLPD_INFO("SharedCounters: received all encoded packets in frame: %u, symbol: %u, packets: %zu\n", frame_id, symbol_id_dl, num_encoded_pkts_per_symbol_);
    } else if (num_encoded_pkts_[frame_id % kFrameWnd][symbol_id_dl] > num_encoded_pkts_per_symbol_) {
        printf("ERROR!\n");
        exit(0);
    }
    return true;
}

bool SharedState::receive_encoded_pkt_loss_tolerant(size_t frame_id, size_t symbol_id_dl, size_t ue_id)
{
    if (unlikely(frame_id >= cur_frame_ + kFrameWnd)) {
        MLPD_ERROR(
            "SharedState error: Received encoded packet for future "
            "frame %zu beyond frame window (%zu + %zu) (Pilot pkt num for frame %zu is %u, pkt num %u). This can "
            "happen if Agora is running slowly, e.g., in debug mode. \n",
            frame_id, cur_frame_, kFrameWnd, cur_frame_, (unsigned int)num_pilot_pkts_[cur_frame_ % kFrameWnd].load(), 
            (unsigned int)num_pkts_[cur_frame_ % kFrameWnd].load());
        return false;
    }

    // Handle packet loss for encode packets
    if (frame_id != encode_next_frame_[ue_id]) {
        if (frame_id > encode_next_frame_[ue_id]) {
            for (size_t sid = encode_next_symbol_[ue_id]; sid < cfg_->dl_data_symbol_num_perframe; sid ++) {
                num_encoded_pkts_[encode_next_frame_[ue_id] % kFrameWnd][sid] ++;
                num_encoded_pkts_states_[encode_next_frame_[ue_id] % kFrameWnd][sid] ^= (1 << ue_id);
            }
            for (size_t fid = encode_next_frame_[ue_id] + 1; fid < frame_id; fid ++) {
                for (size_t sid = 0; sid < cfg_->dl_data_symbol_num_perframe; sid ++) {
                    num_encoded_pkts_[fid % kFrameWnd][sid] ++;
                    num_encoded_pkts_states_[fid % kFrameWnd][sid] ^= (1 << ue_id);
                }
            }
            for (size_t sid = 0; sid < symbol_id_dl; sid ++) {
                num_encoded_pkts_[frame_id % kFrameWnd][sid] ++;
                num_encoded_pkts_states_[frame_id % kFrameWnd][sid] ^= (1 << ue_id);
            }
        }
    } else if (symbol_id_dl != encode_next_symbol_[ue_id]) {
        if (symbol_id_dl > encode_next_symbol_[ue_id]) {
            for (size_t sid = encode_next_symbol_[ue_id]; sid < symbol_id_dl; sid ++) {
                num_encoded_pkts_[frame_id % kFrameWnd][sid] ++;
                num_encoded_pkts_states_[frame_id % kFrameWnd][sid] ^= (1 << ue_id);
            }
        }
    }
    num_encoded_pkts_[frame_id % kFrameWnd][symbol_id_dl]++;
    num_encoded_pkts_states_[frame_id % kFrameWnd][symbol_id_dl] ^= (1 << ue_id);

    size_t next_symbol = symbol_id_dl + 1;
    size_t next_frame = frame_id;
    if (next_symbol == cfg_->dl_data_symbol_num_perframe) {
        next_symbol = 0;
        next_frame ++;
    }
    encode_next_frame_[ue_id] = next_frame;
    encode_next_symbol_[ue_id] = next_symbol;

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
            frame_coding_time_[frame_id] = get_us();
        }
        return true;
    }
    return false;
}

bool SharedState::received_all_demod_pkts_loss_tolerant(size_t ue_id, size_t frame_id, size_t symbol_id_ul)
{
    if (num_demod_pkts_states_[ue_id][frame_id % kFrameWnd][symbol_id_ul] == (1UL << cfg_->bs_server_addr_list.size()) - 1) {
        if (symbol_id_ul == 0) {
            frame_coding_time_[frame_id] = get_us();
        }
        return true;
    }
    return false;
}

bool SharedState::received_all_encoded_pkts(size_t frame_id, size_t symbol_id_dl)
{
    // static size_t frame_tag = 0;
    // if (frame_tag == frame_id) {
    //     printf("Recvd packets: %zu, required: %zu\n", num_encoded_pkts_[frame_id % kFrameWnd][symbol_id_dl].load(), num_encoded_pkts_per_symbol_);
    //     frame_tag ++;
    // }
    if (num_encoded_pkts_[frame_id % kFrameWnd][symbol_id_dl]
        == num_encoded_pkts_per_symbol_) {
        if (symbol_id_dl == 0) {
            frame_sc_time_[frame_id] = get_us();
        }
        return true;
    }
    return false;
}

void num_to_binary(char* res, size_t num, size_t n_bits) {
    for (size_t i = 0; i < n_bits; i ++) {
        res[i] = '0' + ((num >> i) & 1);
    }
    res[n_bits] = '\0';
}

void SharedState::print_receiving_encoded_pkts(size_t frame_id, size_t symbol_id_dl) 
{
    char res[64];
    num_to_binary(res, num_encoded_pkts_states_[frame_id % kFrameWnd][symbol_id_dl].load(), num_encoded_pkts_per_symbol_);
    printf("[Shared State] Frame %zu symbol %zu: received %zu encoded packets, required %zu, state %s\n",
        frame_id, symbol_id_dl, num_encoded_pkts_[frame_id % kFrameWnd][symbol_id_dl].load(), num_encoded_pkts_per_symbol_, res);
}

void SharedState::print_receiving_demod_pkts(size_t ue_id, size_t frame_id, size_t symbol_id_ul)
{
    char res[64];
    num_to_binary(res, num_demod_pkts_states_[ue_id][frame_id % kFrameWnd][symbol_id_ul].load(), num_demod_pkts_per_symbol_per_ue_);
    printf("[Shared State] Frame %zu symbol %zu ue %zu: received %zu demod packets, required %zu, state %s\n",
        frame_id, symbol_id_ul, ue_id, num_demod_pkts_[ue_id][frame_id % kFrameWnd][symbol_id_ul].load(), num_demod_pkts_per_symbol_per_ue_, res);
}

bool SharedState::is_encode_ready(size_t frame_id, size_t symbol_id_dl) {
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    return encode_ready_[frame_id % kFrameWnd] && 
        (get_us() - frame_start_time_[frame_id]) >= (symbol_id_dl + 1) * slot_us_ / symbol_num_per_frame_;
    // return encode_ready_[frame_id % kFrameWnd];
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
    // bool cont = false;
    decode_mutex_[frame_id % kFrameWnd].lock();
    num_decode_tasks_completed_[frame_id % kFrameWnd]++;
    // cont = (num_decode_tasks_completed_[frame_id % kFrameWnd] == num_decode_tasks_per_frame_);
    decode_mutex_[frame_id % kFrameWnd].unlock();

    // if (unlikely(cont)) {
    //     cur_frame_mutex_.lock();
    //     while (num_decode_tasks_completed_[cur_frame_ % kFrameWnd] == num_decode_tasks_per_frame_) {
    //         frame_end_time_[cur_frame_] = get_us();
    //         encode_ready_[(cur_frame_) % kFrameWnd] = false;
    //         size_t cur_cycle = worker_rdtsc();
    //         num_decode_tasks_completed_[(cur_frame_) % kFrameWnd] = 0;
    //         size_t frame_slot = (cur_frame_) % kFrameWnd;
    //         num_pkts_[frame_slot] = 0;
    //         num_pilot_pkts_[frame_slot] = 0;
    //         for (size_t j = 0; j < kMaxSymbols; j++) {
    //             num_data_pkts_[frame_slot][j] = 0;
    //         }
    //         for (size_t i = 0; i < kMaxSymbols; i++) {
    //             num_demul_tasks_completed_[frame_slot][i] = 0;
    //         }
    //         for (size_t i = 0; i < kMaxUEs; i ++) {
    //             for (size_t j = 0; j < kMaxSymbols; j++) {
    //                 num_demod_pkts_[i][frame_slot][j] = 0;
    //                 num_demod_pkts_states_[i][frame_slot][j] = 0;
    //             }
    //         }
    //         for (size_t i = 0; i < kMaxSymbols; i++) {
    //             num_time_iq_pkts_[frame_slot][i] = 0;
    //         }
    //         for (size_t i = 0; i < kMaxSymbols; i++) {
    //             num_fft_tasks_completed_[frame_slot][i] = 0;
    //         }
    //         for (size_t i = 0; i < num_zf_tasks_per_frame_; i++) {
    //             zf_task_completed_[frame_slot][i] = false;
    //         }
    //         MLPD_INFO("Main thread: Decode done frame: %lu, for %.2lfms\n", cur_frame_, cycles_to_ms(cur_cycle - last_frame_cycles_, freq_ghz_));
    //         last_frame_cycles_ = cur_cycle;
    //         cur_frame_ ++;
    //     }
    //     cur_frame_mutex_.unlock();
    // }
}

bool SharedState::move_forward_and_clean_up_ul()
{
    bool cont = (num_decode_tasks_completed_[cur_frame_ % kFrameWnd] == num_decode_tasks_per_frame_);
    if (unlikely(cont)) {
        cur_frame_mutex_.lock();
        while (num_decode_tasks_completed_[cur_frame_ % kFrameWnd] == num_decode_tasks_per_frame_) {
            frame_end_time_[cur_frame_] = get_us();
            encode_ready_[(cur_frame_) % kFrameWnd] = false;
            size_t cur_cycle = worker_rdtsc();
            num_decode_tasks_completed_[(cur_frame_) % kFrameWnd] = 0;
            size_t frame_slot = (cur_frame_) % kFrameWnd;
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
                    num_demod_pkts_states_[i][frame_slot][j] = 0;
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
            MLPD_INFO("Main thread: Decode done frame: %lu, for %.2lfms\n", cur_frame_, cycles_to_ms(cur_cycle - last_frame_cycles_, freq_ghz_));
            last_frame_cycles_ = cur_cycle;
            cur_frame_ ++;
        }
        cur_frame_mutex_.unlock();
    }
    return true;
}

bool SharedState::precode_done(size_t frame_id)
{
    // rt_assert(frame_id == cur_frame_, "Wrong completed precode task!");
    // if (unlikely(frame_id != cur_frame_)) {
    //     MLPD_ERROR("SharedState error: commit a wrong precode frame id (committed: %zu, expected: %u)!\n",
    //         frame_id, cur_frame_);
    //     return false;
    // }
    precode_mutex_.lock();
    num_precode_tasks_completed_[frame_id % kFrameWnd]++;
    while (frame_id == cur_frame_ && num_precode_tasks_completed_[frame_id % kFrameWnd] == num_precode_tasks_per_frame_) {
        frame_end_time_[cur_frame_] = get_us();
        encode_ready_[frame_id % kFrameWnd] = false;
        size_t cur_cycle = worker_rdtsc();
        num_precode_tasks_completed_[frame_id % kFrameWnd] = 0;
        size_t frame_slot = frame_id % kFrameWnd;
        num_pkts_[frame_slot] = 0;
        num_pilot_pkts_[frame_slot] = 0;
        for (size_t j = 0; j < kMaxSymbols; j++) {
            num_data_pkts_[frame_slot][j] = 0;
        }
        for (size_t j = 0; j < kMaxSymbols; j++) {
            num_encode_tasks_completed_[frame_slot][j] = 0;
        }
        // num_encode_tasks_completed_[frame_slot] = 0;
        for (size_t j = 0; j < kMaxSymbols; j++) {
            num_encoded_pkts_[frame_slot][j] = 0;
            num_encoded_pkts_states_[frame_slot][j] = 0;
        }
        MLPD_INFO("Main thread: Precode done frame: %lu, for %.2lfms\n", cur_frame_, cycles_to_ms(cur_cycle - last_frame_cycles_, freq_ghz_));
        last_frame_cycles_ = cur_cycle;
        cur_frame_ ++;
        frame_id ++;
    }
    precode_mutex_.unlock();
    return true;
}

void SharedState::encode_done(size_t frame_id, size_t symbol_id_dl)
{   
    rt_assert(frame_id >= cur_frame_ && frame_id < cur_frame_ + kFrameWnd,
        "Complete a wrong frame in encode!");
    num_encode_tasks_completed_[frame_id % kFrameWnd][symbol_id_dl] ++;
    if (num_encode_tasks_completed_[frame_id % kFrameWnd][symbol_id_dl] == num_encode_tasks_required_) {
        MLPD_INFO("SharedCounters: Encode done frame: %u, symbol: %u\n", frame_id, symbol_id_dl);
    }
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
        frame_sc_time_[frame_id] = get_us();
        return true;
    } 
    return false;
}

bool SharedState::is_encode_tx_ready(size_t frame_id, size_t symbol_id_dl)
{
    if (frame_id < cur_frame_ || frame_id >= cur_frame_ + kFrameWnd) {
        return false;
    }
    if (num_encode_tasks_completed_[frame_id % kFrameWnd][symbol_id_dl]
        == num_encode_tasks_required_) {
        frame_coding_time_[frame_id] = get_us();
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
