#include "dycoding.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "signalHandler.hpp"
#include <malloc.h>

static constexpr bool kPrintEncodedData = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

DyEncode::DyEncode(Config* in_config, int in_tid, double freq_ghz,
    Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
    Stats* in_stats_manager, RxStatus* rx_status,
    EncodeStatus* encode_status)
    : Doer(in_config, in_tid, freq_ghz, dummy_conq_, complete_task_queue,
          worker_producer_token)
    , raw_data_buffer_(in_raw_data_buffer)
    , encoded_buffer_(in_encoded_buffer)
    , rx_status_(rx_status)
    , encode_status_(encode_status)
    , ue_id_(in_tid + in_config->ue_start)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kEncode, in_tid);
    parity_buffer = (int8_t*)memalign(64,
        ldpc_encoding_parity_buf_size(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
    encoded_buffer_temp = (int8_t*)memalign(64,
        ldpc_encoding_encoded_buf_size(
            cfg->LDPC_config.Bg, cfg->LDPC_config.Zc));
}

DyEncode::~DyEncode()
{
    free(parity_buffer);
    free(encoded_buffer_temp);
}

Event_data DyEncode::launch(size_t tag)
{
    LDPCconfig LDPC_config = cfg->LDPC_config;
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t cb_id = gen_tag_t(tag).cb_id;
    size_t cur_cb_id = cb_id % cfg->LDPC_config.nblocksInSymbol;
    size_t ue_id = cb_id / cfg->LDPC_config.nblocksInSymbol;
    if (kDebugPrintInTask) {
        printf(
            "In doEncode thread %d: frame: %zu, symbol: %zu, code block %zu\n",
            tid, frame_id, symbol_id, cur_cb_id);
    }
    // printf("DoEncode frame %u symbol %u cb %u\n", frame_id, symbol_id, cur_cb_id);

    size_t start_tsc = worker_rdtsc();

    // size_t symbol_idx_dl = cfg->get_dl_symbol_idx(frame_id, symbol_id);
    size_t symbol_idx_dl = symbol_id;
    int8_t* input_ptr
        = cfg->get_info_bits(raw_data_buffer_, symbol_idx_dl, ue_id, cur_cb_id);

    ldpc_encode_helper(LDPC_config.Bg, LDPC_config.Zc, LDPC_config.nRows,
        encoded_buffer_temp, parity_buffer, input_ptr);
    // Start Debug
    // if (ue_id == 2) {
    //     printf("frame id: %u, symbol id: %u, symbol idx dl: %u\n", frame_id, symbol_id, symbol_idx_dl);
    //     printf("Raw data:\n");
    //     for (size_t i = 0; i < ldpc_encoding_input_buf_size(LDPC_config.Bg, LDPC_config.Zc); i ++) {
    //         printf("%02x ", (uint8_t)input_ptr[i]);
    //     }
    //     printf("\nEncoded data:\n");
    //     for (size_t i = 0; i < ldpc_encoding_encoded_buf_size(LDPC_config.Bg, LDPC_config.Zc); i ++) {
    //         printf("%02x ", (uint8_t)encoded_buffer_temp[i]);
    //     }
    //     printf("\n");
    // }
    // End Debug
    int8_t* final_output_ptr = cfg->get_encoded_buf(
        encoded_buffer_, frame_id, symbol_idx_dl, ue_id, cur_cb_id);
    adapt_bits_for_mod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
        reinterpret_cast<uint8_t*>(final_output_ptr),
        bits_to_bytes(LDPC_config.cbCodewLen), cfg->mod_order_bits);

    // if (ue_id == 2) {
    //     complex_float tf = mod_single_uint8(final_output_ptr[1], cfg->mod_table);
    //     printf("Mod data: (%lf %lf)\n", tf.re, tf.im);
    // }

    // printf("Encoded data\n");
    // int num_mod = LDPC_config.cbCodewLen / cfg->mod_order_bits;
    // for(int i = 0; i < num_mod; i++) {
    //     printf("%u ", *(final_output_ptr + i));
    // }
    // printf("\n");

    size_t duration = worker_rdtsc() - start_tsc;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (cycles_to_us(duration, freq_ghz) > 500) {
        printf("Thread %d Encode takes %.2f\n", tid,
            cycles_to_us(duration, freq_ghz));
    }

    return Event_data(EventType::kEncode, tag);
}

void DyEncode::start_work() 
{
    while (cfg->running && !SignalHandler::gotExitSignal()) {
        if (cur_cb_ > 0
            || rx_status_->is_encode_ready(cur_frame_)) {
            // printf("Start to encode user %lu frame %lu symbol %lu cb %u\n", ue_id_, cur_frame_, cur_symbol_, cur_cb_);
            launch(gen_tag_t::frm_sym_cb(cur_frame_, cur_symbol_,
                cur_cb_ + ue_id_ * cfg->LDPC_config.nblocksInSymbol)
                       ._tag);
            cur_cb_++;
            if (cur_cb_ == cfg->LDPC_config.nblocksInSymbol) {
                // printf("Encode is done??? ue %u\n", ue_id_);
                cur_cb_ = 0;
                encode_status_->encode_done(ue_id_, cur_frame_, cur_symbol_);
                cur_symbol_++;
                if (cur_symbol_ == cfg->dl_data_symbol_num_perframe) {
                    cur_symbol_ = 0;
                    cur_frame_++;
                }
            }
        }
    }
}

DyDecode::DyDecode(Config* in_config, int in_tid, double freq_ghz,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
    Table<int8_t> demod_soft_buffer_to_decode,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
    std::vector<std::vector<ControlInfo>>& control_info_table,
    std::vector<size_t>& control_idx_list,
    PhyStats* in_phy_stats, Stats* in_stats_manager, RxStatus* rx_status,
    DecodeStatus* decode_status)
    : Doer(in_config, in_tid, freq_ghz, dummy_conq_, complete_task_queue,
          worker_producer_token)
    , demod_buffers_(demod_buffers)
    , demod_soft_buffer_to_decode_(demod_soft_buffer_to_decode)
    , decoded_buffers_(decoded_buffers)
    , control_info_table_(control_info_table)
    , control_idx_list_(control_idx_list)
    , phy_stats(in_phy_stats)
    , rx_status_(rx_status)
    , decode_status_(decode_status)
    // , ue_id_(in_tid / cfg->decode_thread_num_per_ue + in_config->ue_start)
    // , tid_in_ue_(in_tid % cfg->decode_thread_num_per_ue)
    , total_dycode_num_(cfg->decode_thread_num)
    , total_ue_num_(cfg->ue_end - cfg->ue_start)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kDecode, in_tid);
    resp_var_nodes = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
}

DyDecode::~DyDecode() { free(resp_var_nodes); }

Event_data DyDecode::launch(size_t tag)
{
    LDPCconfig LDPC_config = cfg->LDPC_config;
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t symbol_idx_ul = gen_tag_t(tag).symbol_id;
    const size_t cb_id = gen_tag_t(tag).cb_id;
    const size_t symbol_offset
        = cfg->get_total_data_symbol_idx_ul(frame_id, symbol_idx_ul);
    const size_t cur_cb_id = cb_id % cfg->LDPC_config.nblocksInSymbol;
    const size_t ue_id = cb_id / cfg->LDPC_config.nblocksInSymbol;
    const size_t frame_slot = frame_id % kFrameWnd;
    if (kDebugPrintInTask) {
        printf("In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
               "%zu, ue: %zu\n",
            tid, frame_id, symbol_idx_ul, cur_cb_id, ue_id);
    }

    std::vector<ControlInfo>& info_list = control_info_table_[control_idx_list_[frame_id]];
    if (ue_id >= info_list.size()) {
        return Event_data(EventType::kDecode, tag);
    }
    ControlInfo& info = info_list[ue_id];

    size_t start_tsc = worker_rdtsc();

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };

    // Decoder setup
    int16_t numFillerBits = 0;
    size_t nRows = info.Bg == 1 ? 46 : 42;
    uint32_t cbCodewLen = ldpc_num_encoded_bits(info.Bg, info.Zc, nRows);
    uint32_t cbLen = ldpc_num_input_bits(info.Bg, info.Zc);
    // int16_t numChannelLlrs = LDPC_config.cbCodewLen;
    int16_t numChannelLlrs = cbCodewLen;

    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = LDPC_config.decoderIter;
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = LDPC_config.earlyTermination;
    // ldpc_decoder_5gnr_request.Zc = LDPC_config.Zc;
    ldpc_decoder_5gnr_request.Zc = info.Zc;
    // ldpc_decoder_5gnr_request.baseGraph = LDPC_config.Bg;
    ldpc_decoder_5gnr_request.baseGraph = info.Bg;
    // ldpc_decoder_5gnr_request.nRows = LDPC_config.nRows;
    ldpc_decoder_5gnr_request.nRows = nRows;

    // int numMsgBits = LDPC_config.cbLen - numFillerBits;
    int numMsgBits = cbLen - numFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

    // auto* llr_buffer_ptr
    //     = cfg->get_demod_buf_to_decode(demod_soft_buffer_to_decode_, frame_id,
    //         symbol_idx_ul, ue_id, LDPC_config.cbCodewLen * cur_cb_id);
    auto* llr_buffer_ptr
        = cfg->get_demod_buf_to_decode(demod_soft_buffer_to_decode_, frame_id,
            symbol_idx_ul, ue_id, info.sc_start);

    // uint8_t* decoded_buffer_ptr
    //     = decoded_buffers_[frame_slot][symbol_idx_ul][ue_id]
    //     + (cur_cb_id * roundup<64>(cfg->num_bytes_per_cb));
    uint8_t* decoded_buffer_ptr
        = decoded_buffers_[frame_slot][symbol_idx_ul][ue_id];

    ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

    size_t start_tsc1 = worker_rdtsc();
    duration_stat->task_duration[1] += start_tsc1 - start_tsc;

    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

    size_t start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;
    decode_count_ ++;
    decode_max_ = decode_max_ < start_tsc2 - start_tsc1 ? start_tsc2 - start_tsc1 : decode_max_;

    if (kPrintLLRData) {
        printf("LLR data, symbol_offset: %zu\n", symbol_offset);
        for (size_t i = 0; i < LDPC_config.cbCodewLen; i++) {
            printf("%d ", *(llr_buffer_ptr + i));
        }
        printf("\n");
    }

    if (kPrintDecodedData) {
        printf("Decoded data: ");
        for (size_t i = 0; i < (LDPC_config.cbLen >> 3); i++) {
            printf("%u ", *(decoded_buffer_ptr + i));
        }
        printf("\n");
    }

    if (!kEnableMac && kPrintPhyStats && symbol_idx_ul == cfg->UL_PILOT_SYMS) {
        phy_stats->update_decoded_bits(
            ue_id, symbol_offset, cfg->num_bytes_per_cb * 8);
        phy_stats->increment_decoded_blocks(ue_id, symbol_offset);
        size_t block_error(0);
        for (size_t i = 0; i < cfg->num_bytes_per_cb; i++) {
            uint8_t rx_byte = decoded_buffer_ptr[i];
            uint8_t tx_byte = (uint8_t)cfg->get_info_bits(
                cfg->ul_bits, symbol_idx_ul, ue_id, cur_cb_id)[i];
            phy_stats->update_bit_errors(
                ue_id, symbol_offset, tx_byte, rx_byte);
            if (rx_byte != tx_byte)
                block_error++;
        }
        phy_stats->update_block_errors(ue_id, symbol_offset, block_error);
    }

    double duration = worker_rdtsc() - start_tsc;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (cycles_to_us(duration, freq_ghz) > 500) {
        printf("Thread %d Decode takes %.2f\n", tid,
            cycles_to_us(duration, freq_ghz));
    }

    return Event_data(EventType::kDecode, tag);
}

void DyDecode::start_work()
{
    // printf("Decode for ue %u tid %u starts to work!\n", ue_id_, tid_in_ue_);
    printf("Decode tid %u starts to work!\n", tid);
    // cur_symbol_ = tid_in_ue_;
    cur_idx_ = tid;
    cur_symbol_ = cur_idx_ / total_ue_num_;
    cur_ue_ = cur_idx_ % total_ue_num_ + cfg->ue_start;

    size_t start_tsc = 0;
    size_t work_tsc_duration = 0;
    size_t decode_tsc_duration = 0;
    size_t state_operation_duration = 0;
    size_t loop_count = 0;
    size_t work_count = 0;
    size_t decode_count = 0;
    size_t decode_start_tsc;
    bool state_trigger = false;

    size_t decode_max = 0;

    while (cfg->running && !SignalHandler::gotExitSignal()) {

        if (cfg->test_mode > 0) {
            continue;
        }

        if (likely(state_trigger)) {
            loop_count ++;
        }
        size_t work_start_tsc, state_start_tsc;

        if (cur_cb_ > 0) {

            if (unlikely(!state_trigger && cur_frame_ >= 200)) {
                start_tsc = rdtsc();
                state_trigger = true;
            }

            if (likely(state_trigger)) {
                work_start_tsc = rdtsc();
                work_count ++;
            }
            
            // printf("Start to decode user %lu frame %lu symbol %lu\n", ue_id_, cur_frame_, cur_symbol_);

            if (likely(state_trigger)) {
                decode_start_tsc = rdtsc();
            }
            launch(gen_tag_t::frm_sym_cb(cur_frame_, cur_symbol_,
                cur_cb_ + cur_ue_ * cfg->LDPC_config.nblocksInSymbol)
                       ._tag);
            if (likely(state_trigger)) {
                size_t decode_tmp_tsc = rdtsc() - decode_start_tsc;
                decode_tsc_duration += decode_tmp_tsc;
                decode_max = decode_max < decode_tmp_tsc ? decode_tmp_tsc : decode_max;
                decode_count ++;
            }

            // printf("Start to decode user %lu frame %lu symbol %lu end\n", ue_id_, cur_frame_, cur_symbol_);
            if (likely(state_trigger)) {
                decode_start_tsc = rdtsc();
            }
            cur_cb_++;
            if (cur_cb_ == cfg->LDPC_config.nblocksInSymbol) {
                cur_cb_ = 0;
                // cur_symbol_ += cfg->decode_thread_num_per_ue;
                cur_idx_ += total_dycode_num_;
                cur_symbol_ = cur_idx_ / total_ue_num_;
                cur_ue_ = cur_idx_ % total_ue_num_ + cfg->ue_start;
                if (cur_symbol_ >= cfg->ul_data_symbol_num_perframe) {
                    // cur_symbol_ = tid_in_ue_;
                    cur_idx_ = tid;
                    cur_symbol_ = cur_idx_ / total_ue_num_;
                    cur_ue_ = cur_idx_ % total_ue_num_ + cfg->ue_start;

                    // decode_start_tsc = rdtsc();
                    rx_status_->decode_done(cur_frame_);
                    // state_operation_duration += rdtsc() - decode_start_tsc;

                    cur_frame_++;
                    if (unlikely(cur_frame_ == cfg->frames_to_test)) {
                        if (likely(state_trigger)) {
                            state_operation_duration += rdtsc() - decode_start_tsc;
                            work_tsc_duration += rdtsc() - work_start_tsc;
                        }
                        break;
                    }
                }
            }
            if (likely(state_trigger)) {
                state_operation_duration += rdtsc() - decode_start_tsc;
                work_tsc_duration += rdtsc() - work_start_tsc;
            }

        } else {

            if (likely(state_trigger)) {
                work_start_tsc = rdtsc();
                state_start_tsc = rdtsc();
            }
            bool ret = decode_status_->received_all_demod_data(
                   cur_ue_, cur_frame_, cur_symbol_);
            if (likely(state_trigger)) {
                state_operation_duration += rdtsc() - state_start_tsc;
                work_tsc_duration += rdtsc() - work_start_tsc;
            }

            if (ret) {

                if (unlikely(!state_trigger && cur_frame_ >= 200)) {
                    loop_count ++;
                    start_tsc = rdtsc();
                    state_trigger = true;
                }

                if (likely(state_trigger)) {
                    work_start_tsc = rdtsc();
                    work_count ++;
                }
            
                // printf("Start to decode user %lu frame %lu symbol %lu\n", ue_id_, cur_frame_, cur_symbol_);

                if (likely(state_trigger)) {
                    decode_start_tsc = rdtsc();
                }
                launch(gen_tag_t::frm_sym_cb(cur_frame_, cur_symbol_,
                    cur_cb_ + cur_ue_ * cfg->LDPC_config.nblocksInSymbol)
                        ._tag);
                if (likely(state_trigger)) {
                    size_t decode_tmp_tsc = rdtsc() - decode_start_tsc;
                    decode_tsc_duration += decode_tmp_tsc;
                    decode_max = decode_max < decode_tmp_tsc ? decode_tmp_tsc : decode_max;
                    decode_count ++;
                }

                // printf("Start to decode user %lu frame %lu symbol %lu end\n", ue_id_, cur_frame_, cur_symbol_);
                if (likely(state_trigger)) {
                    state_start_tsc = rdtsc();
                }
                cur_cb_++;
                if (cur_cb_ == cfg->LDPC_config.nblocksInSymbol) {
                    cur_cb_ = 0;
                    // cur_symbol_ += cfg->decode_thread_num_per_ue;
                    cur_idx_ += total_dycode_num_;
                    cur_symbol_ = cur_idx_ / total_ue_num_;
                    cur_ue_ = cur_idx_ % total_ue_num_ + cfg->ue_start;
                    if (cur_symbol_ >= cfg->ul_data_symbol_num_perframe) {
                        // cur_symbol_ = tid_in_ue_;
                        cur_idx_ = tid;
                        cur_symbol_ = cur_idx_ / total_ue_num_;
                        cur_ue_ = cur_idx_ % total_ue_num_ + cfg->ue_start;

                        // state_start_tsc = rdtsc();
                        rx_status_->decode_done(cur_frame_);
                        // state_operation_duration += rdtsc() - state_start_tsc;

                        cur_frame_++;
                        if (unlikely(cur_frame_ == cfg->frames_to_test)) {
                            if (likely(state_trigger)) {
                                state_operation_duration += rdtsc() - state_start_tsc;
                                work_tsc_duration += rdtsc() - work_start_tsc;
                            }
                            break;
                        }

                        if (should_sleep(control_info_table_[control_idx_list_[cur_frame_]].size())) {
                        //     cur_frame_ ++;
                        //     if (unlikely(cur_frame_ == cfg->frames_to_test)) {
                        //         if (likely(state_trigger)) {
                        //             state_operation_duration += rdtsc() - state_start_tsc;
                        //             work_tsc_duration += rdtsc() - work_start_tsc;
                        //         }
                        //         break;
                        //     }
                        //     usleep(800); // TODO: Adjust this number
                            std::this_thread::sleep_for(std::chrono::microseconds(900));
                        }
                    }
                }
                if (likely(state_trigger)) {
                    state_operation_duration += rdtsc() - state_start_tsc;
                
                    work_tsc_duration += rdtsc() - work_start_tsc;
                }

            }
            
        }

    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("DoDecode Thread %u duration stats: total time used %.2lfms, "
        "decode %.2lfms (%u, %.2lf\%, %.2lfus, %u, %.2lfus), stating %.2lfms (%.2lf\%), idle %.2lfms (%.2lf\%), "
        "working proportions (%u/%u: %.2lf\%)\n",
        tid, cycles_to_ms(whole_duration, freq_ghz),
        cycles_to_ms(decode_tsc_duration, freq_ghz), decode_count, decode_tsc_duration * 100.0f / whole_duration, cycles_to_us(decode_max, freq_ghz), decode_count_, cycles_to_us(decode_max_, freq_ghz),
        cycles_to_ms(state_operation_duration, freq_ghz), state_operation_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, work_count * 100.0f / loop_count);
}
