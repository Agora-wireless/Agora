#include "dycoding.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "signalHandler.hpp"
#include <malloc.h>

#define TRIGGER_TIMER(stmt) if(likely(state_trigger)){stmt;}

static constexpr bool kPrintEncodedData = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

#if 0
DyEncode::DyEncode(Config* in_config, int in_tid, double freq_ghz,
    Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
    Stats* in_stats_manager, SharedState* shared_state_,
    EncodeStatus* encode_status)
    : Doer(in_config, in_tid, freq_ghz)
    , raw_data_buffer_(in_raw_data_buffer)
    , encoded_buffer_(in_encoded_buffer)
    , shared_state_(shared_state_)
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

EventData DyEncode::launch(size_t tag)
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

    size_t start_tsc = worker_rdtsc();

    size_t symbol_idx_dl = symbol_id;
    int8_t* input_ptr
        = cfg->get_info_bits(raw_data_buffer_, symbol_idx_dl, ue_id, cur_cb_id);

    ldpc_encode_helper(LDPC_config.Bg, LDPC_config.Zc, LDPC_config.nRows,
        encoded_buffer_temp, parity_buffer, input_ptr);
    
    int8_t* final_output_ptr = cfg->get_encoded_buf(
        encoded_buffer_, frame_id, symbol_idx_dl, ue_id, cur_cb_id);
    adapt_bits_for_mod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
        reinterpret_cast<uint8_t*>(final_output_ptr),
        bits_to_bytes(LDPC_config.cbCodewLen), cfg->mod_order_bits);

    size_t duration = worker_rdtsc() - start_tsc;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (cycles_to_us(duration, freq_ghz) > 500) {
        printf("Thread %d Encode takes %.2f\n", tid,
            cycles_to_us(duration, freq_ghz));
    }

    return EventData(EventType::kEncode, tag);
}

void DyEncode::start_work() 
{
    while (cfg->running && !SignalHandler::gotExitSignal()) {
        if (cur_cb_ > 0
            || shared_state_->is_encode_ready(cur_frame_)) {
            launch(gen_tag_t::frm_sym_cb(cur_frame_, cur_symbol_,
                cur_cb_ + ue_id_ * cfg->LDPC_config.nblocksInSymbol)
                       ._tag);
            cur_cb_++;
            if (cur_cb_ == cfg->LDPC_config.nblocksInSymbol) {
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
#endif

DyDecode::DyDecode(Config* in_config, int in_tid, double freq_ghz,
    Table<int8_t> demod_buffer_to_decode,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
    std::vector<std::vector<ControlInfo>>& control_info_table,
    std::vector<size_t>& control_idx_list,
    SharedState* shared_state)
    : Doer(in_config, in_tid, freq_ghz)
    , demod_buffer_to_decode_(demod_buffer_to_decode)
    , decoded_buffers_(decoded_buffers)
    , shared_state_(shared_state)
    , total_ue_num_(cfg_->ue_end - cfg_->ue_start)
    , total_dycode_num_(cfg_->decode_thread_num)
    , control_info_table_(control_info_table)
    , control_idx_list_(control_idx_list)
{
    resp_var_nodes_ = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
}

DyDecode::~DyDecode() { free(resp_var_nodes_); }

EventData DyDecode::Launch(size_t tag)
{
    LDPCconfig LDPC_config = cfg_->LDPC_config;
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t symbol_idx_ul = gen_tag_t(tag).symbol_id;
    const size_t cb_id = gen_tag_t(tag).cb_id;
    const size_t symbol_offset
        = cfg_->get_total_data_symbol_idx_ul(frame_id, symbol_idx_ul);
    const size_t cur_cb_id = cb_id % cfg_->LDPC_config.nblocksInSymbol;
    const size_t ue_id = cb_id / cfg_->LDPC_config.nblocksInSymbol;
    const size_t frame_slot = frame_id % kFrameWnd;
    if (kDebugPrintInTask) {
        printf("In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
               "%zu, ue: %zu\n",
            tid_, frame_id, symbol_idx_ul, cur_cb_id, ue_id);
    }

    std::vector<ControlInfo>& info_list = control_info_table_[control_idx_list_[frame_id]];
    if (ue_id >= info_list.size()) {
        return EventData(EventType::kDecode, tag);
    }
    ControlInfo& info = info_list[ue_id];

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };

    // Decoder setup
    int16_t numFillerBits = 0;
    size_t nRows = info.Bg == 1 ? 46 : 42;
    uint32_t cbCodewLen = ldpc_num_encoded_bits(info.Bg, info.Zc, nRows);
    uint32_t cbLen = ldpc_num_input_bits(info.Bg, info.Zc);
    int16_t numChannelLlrs = cbCodewLen;

    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = LDPC_config.decoderIter;
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = LDPC_config.earlyTermination;
    ldpc_decoder_5gnr_request.Zc = info.Zc;
    ldpc_decoder_5gnr_request.baseGraph = info.Bg;
    ldpc_decoder_5gnr_request.nRows = nRows;

    int numMsgBits = cbLen - numFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes_;

    auto* llr_buffer_ptr
        = cfg_->get_demod_buf_to_decode(demod_buffer_to_decode_, frame_id,
            symbol_idx_ul, ue_id, info.sc_start);

    uint8_t* decoded_buffer_ptr
        = decoded_buffers_[frame_slot][symbol_idx_ul][ue_id];

    ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

    if (frame_id == 0 && symbol_idx_ul == 0 && ue_id == 0) {
        FILE* file = fopen("data/tmp_decode.bin", "wb");
        fwrite(llr_buffer_ptr, 1, cbCodewLen, file);
        fclose(file);

        file = fopen("data/first_decode.txt", "w");
        for (size_t i = 0; i < cbCodewLen; i ++) {
            fprintf(file, "%d ", llr_buffer_ptr[i]);
        }
        fprintf(file, "\n");
        fclose(file);
    }

    size_t start_tsc1 = worker_rdtsc();

    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

    size_t start_tsc2 = worker_rdtsc();
    decode_count_ ++;
    decode_max_ = decode_max_ < start_tsc2 - start_tsc1 ? start_tsc2 - start_tsc1 : decode_max_;
    decode_tsc_ += start_tsc2 - start_tsc1;

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

    return EventData(EventType::kDecode, tag);
}

void DyDecode::StartWork()
{
    printf("Decode tid %u starts to work!\n", tid_);
    cur_idx_ = tid_;
    cur_symbol_ = cur_idx_ / total_ue_num_;
    cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;

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

    while (cfg_->running && !SignalHandler::gotExitSignal()) {
        TRIGGER_TIMER(loop_count ++);
        size_t work_start_tsc, state_start_tsc;

        if (cur_cb_ > 0) {
            if (unlikely(!state_trigger && cur_frame_ >= 200)) {
                start_tsc = rdtsc();
                state_trigger = true;
            }

            TRIGGER_TIMER({
                work_start_tsc = rdtsc();
                work_count ++;
                decode_start_tsc = rdtsc();
            });

            Launch(gen_tag_t::frm_sym_cb(cur_frame_, cur_symbol_,
                cur_cb_ + cur_ue_ * cfg_->LDPC_config.nblocksInSymbol)
                       ._tag);
            
            TRIGGER_TIMER({
                size_t decode_tmp_tsc = rdtsc() - decode_start_tsc;
                decode_tsc_duration += decode_tmp_tsc;
                decode_max = decode_max < decode_tmp_tsc ? decode_tmp_tsc : decode_max;
                decode_count ++;
                decode_start_tsc = rdtsc();
            });

            cur_cb_++;
            if (cur_cb_ == cfg_->LDPC_config.nblocksInSymbol) {
                cur_cb_ = 0;
                cur_idx_ += total_dycode_num_;
                cur_symbol_ = cur_idx_ / total_ue_num_;
                cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;
                if (cur_symbol_ >= cfg_->ul_data_symbol_num_perframe) {
                    cur_idx_ = tid_;
                    cur_symbol_ = cur_idx_ / total_ue_num_;
                    cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;

                    shared_state_->decode_done(cur_frame_);

                    cur_frame_++;
                    if (unlikely(cur_frame_ == cfg_->frames_to_test)) {
                        TRIGGER_TIMER({
                            state_operation_duration += rdtsc() - decode_start_tsc;
                            work_tsc_duration += rdtsc() - work_start_tsc;
                        });
                        break;
                    }
                }
            }

            TRIGGER_TIMER({
                state_operation_duration += rdtsc() - decode_start_tsc;
                work_tsc_duration += rdtsc() - work_start_tsc;
            });
        } else {
            TRIGGER_TIMER({
                work_start_tsc = rdtsc();
                state_start_tsc = rdtsc();
            });

            bool ret = shared_state_->received_all_demod_pkts(
                   cur_ue_, cur_frame_, cur_symbol_);
            
            TRIGGER_TIMER({
                state_operation_duration += rdtsc() - state_start_tsc;
                work_tsc_duration += rdtsc() - work_start_tsc;
            });

            if (ret) {
                if (unlikely(!state_trigger && cur_frame_ >= 200)) {
                    loop_count ++;
                    start_tsc = rdtsc();
                    state_trigger = true;
                }

                TRIGGER_TIMER({
                    work_start_tsc = rdtsc();
                    work_count ++;
                    decode_start_tsc = rdtsc();
                });
                
                Launch(gen_tag_t::frm_sym_cb(cur_frame_, cur_symbol_,
                    cur_cb_ + cur_ue_ * cfg_->LDPC_config.nblocksInSymbol)
                        ._tag);

                TRIGGER_TIMER({
                    size_t decode_tmp_tsc = rdtsc() - decode_start_tsc;
                    decode_tsc_duration += decode_tmp_tsc;
                    decode_max = decode_max < decode_tmp_tsc ? decode_tmp_tsc : decode_max;
                    decode_count ++;
                    state_start_tsc = rdtsc();
                });

                cur_cb_++;
                if (cur_cb_ == cfg_->LDPC_config.nblocksInSymbol) {
                    cur_cb_ = 0;
                    cur_idx_ += total_dycode_num_;
                    cur_symbol_ = cur_idx_ / total_ue_num_;
                    cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;
                    if (cur_symbol_ >= cfg_->ul_data_symbol_num_perframe) {
                        cur_idx_ = tid_;
                        cur_symbol_ = cur_idx_ / total_ue_num_;
                        cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;

                        shared_state_->decode_done(cur_frame_);

                        cur_frame_++;
                        if (unlikely(cur_frame_ == cfg_->frames_to_test)) {
                            TRIGGER_TIMER({
                                state_operation_duration += rdtsc() - state_start_tsc;
                                work_tsc_duration += rdtsc() - work_start_tsc;
                            });
                            break;
                        }

                        if (shouldSleep(control_info_table_[control_idx_list_[cur_frame_]].size())) {
                            std::this_thread::sleep_for(std::chrono::microseconds(900));
                        }
                    }
                }
                
                TRIGGER_TIMER({
                    state_operation_duration += rdtsc() - state_start_tsc;
                    work_tsc_duration += rdtsc() - work_start_tsc;
                });
            }
        }
    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("DoDecode Thread %u duration stats: total time used %.2lfms, "
        "decode %.2lfms (%zu, %.2lf%%, %.2lfus, %zu, %.2lfus, %.2lfus), stating %.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), "
        "working proportions (%zu/%zu: %.2lf%%)\n",
        tid_, cycles_to_ms(whole_duration, freq_ghz_),
        cycles_to_ms(decode_tsc_duration, freq_ghz_), decode_count, decode_tsc_duration * 100.0f / whole_duration, cycles_to_us(decode_max, freq_ghz_), decode_count_, cycles_to_us(decode_max_, freq_ghz_), cycles_to_us(decode_tsc_ / decode_count_, freq_ghz_),
        cycles_to_ms(state_operation_duration, freq_ghz_), state_operation_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz_), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, work_count * 100.0f / loop_count);
}
