#ifndef DYCODING
#define DYCODING

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "control.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.hpp"
#include "phy_stats.hpp"
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "encoder.hpp"
#include "iobuffer.hpp"
#include "shared_counters.hpp"
#include "utils_ldpc.hpp"

#if 0
class DyEncode : public Doer {
public:
    DyEncode(Config* in_config, int in_tid, double freq_ghz,
        Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
        Stats* in_stats_manager, SharedState* shared_state_,
        EncodeStatus* encode_status);
    ~DyEncode();

    Event_data launch(size_t tag);

    void start_work();

private:
    Table<int8_t>& raw_data_buffer_;
    int8_t* parity_buffer; // Intermediate buffer to hold LDPC encoding parity

    // Intermediate buffer to hold LDPC encoding output
    int8_t* encoded_buffer_temp;
    Table<int8_t>& encoded_buffer_;
    DurationStat* duration_stat;

    SharedState* shared_state_;
    EncodeStatus* encode_status_;

    size_t ue_id_;

    size_t cur_frame_ = 0;
    size_t cur_symbol_ = 0;
    size_t cur_cb_ = 0;
    moodycamel::ConcurrentQueue<Event_data> dummy_conq_;
};
#endif

class DyDecode : public Doer {
public:
    DyDecode(Config* in_config, int in_tid, double freq_ghz,
        Table<int8_t> demod_buffer_to_decode,
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
        std::vector<std::vector<ControlInfo>>& control_info_table_,
        std::vector<size_t>& control_idx_list_,
        SharedState* shared_state_ = nullptr);

    ~DyDecode();

    Event_data launch(size_t tag);

    void start_work();

private:
    inline bool should_sleep(size_t ue_num) { 
        return ue_num <= cfg->ue_start;
    }

    int16_t* resp_var_nodes;
    Table<int8_t> demod_buffer_to_decode_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers_;
    SharedState* shared_state_;

    size_t total_ue_num_;
    size_t total_dycode_num_;

    // decoder process one code block at a time
    size_t cur_frame_ = 0; // Current frame to decode
    size_t cur_symbol_ = 0; // Current symbol to decode
    size_t cur_ue_ = 0;
    size_t cur_cb_ = 0; // Current code block id to decode
    size_t cur_idx_ = 0;

    // Control info
    std::vector<std::vector<ControlInfo>>& control_info_table_;
    std::vector<size_t>& control_idx_list_;

    size_t decode_count_ = 0;
    size_t decode_max_ = 0;
    size_t decode_tsc_ = 0;
};

#endif
