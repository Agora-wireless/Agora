#ifndef DYCODING
#define DYCODING

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "control.hpp"
#include "doer.hpp"
#include "diagnosis.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "encoder.hpp"
#include "iobuffer.hpp"
#include "shared_counters.hpp"
#include "utils_ldpc.hpp"

class DyEncode : public Doer {
public:
    DyEncode(Config* in_config, int in_tid, double freq_ghz,
        Table<int8_t>& dl_bits_buffer,
        Table<int8_t>& dl_encoded_buffer,
        std::vector<std::vector<ControlInfo>>& control_info_table,
        std::vector<size_t>& control_idx_list,
        SharedState* shared_state,
        BottleneckEncode& bottleneck_encode);

    ~DyEncode();

    void Launch(size_t frame_id, size_t symbol_id_dl, size_t ue_id);
    void LaunchStatic(size_t frame_id, size_t symbol_id_dl, size_t ue_id);

    void StartWork();

private:
    Table<int8_t>& dl_bits_buffer_;
    Table<int8_t>& dl_encoded_buffer_;

    // Intermediate buffer to hold LDPC encoding parity
    int8_t* parity_buffer; 
    // Intermediate buffer to hold LDPC encoding output
    int8_t* encoded_buffer_temp;

    SharedState* shared_state_;

    size_t total_ue_num_;
    size_t total_dycode_num_;

    size_t cur_frame_ = 0;
    size_t cur_symbol_ = 0;
    size_t cur_ue_ = 0;
    size_t cur_idx_ = 0;

    // Control info
    std::vector<std::vector<ControlInfo>>& control_info_table_;
    std::vector<size_t>& control_idx_list_;

    size_t encode_count_ = 0;
    size_t encode_max_ = 0;
    size_t encode_tsc_ = 0;

    BottleneckEncode& bottleneck_encode_;
};

class DyDecode : public Doer {
public:
    DyDecode(Config* in_config, int in_tid, double freq_ghz,
        Table<int8_t> demod_buffer_to_decode,
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
        std::vector<std::vector<ControlInfo>>& control_info_table,
        std::vector<size_t>& control_idx_list,
        SharedState* shared_state,
        BottleneckDecode& bottleneck_decode);

    ~DyDecode();

    void Launch(size_t frame_id, size_t symbol_id_ul, size_t ue_id);
    void LaunchStatic(size_t frame_id, size_t symbol_id_ul, size_t ue_id);

    void StartWork();
    void StartWorkCentral();

private:
    inline bool shouldSleep(size_t ue_num) { 
        return ue_num <= cfg_->ue_start;
    }

    int16_t* resp_var_nodes_;
    Table<int8_t> demod_buffer_to_decode_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers_;
    SharedState* shared_state_;

    size_t total_ue_num_;
    size_t total_dycode_num_;

    // decoder process one code block at a time
    size_t cur_frame_ = 0; // Current frame to decode
    size_t cur_symbol_ = 0; // Current symbol to decode
    size_t cur_ue_ = 0;
    size_t cur_idx_ = 0;

    // Control info
    std::vector<std::vector<ControlInfo>>& control_info_table_;
    std::vector<size_t>& control_idx_list_;

    size_t decode_count_ = 0;
    size_t decode_max_ = 0;
    size_t decode_tsc_ = 0;

    BottleneckDecode& bottleneck_decode_;
};

#endif
