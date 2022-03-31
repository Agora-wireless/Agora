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

DyEncode::DyEncode(Config* in_config, int in_tid, double freq_ghz,
    Table<int8_t>& dl_bits_buffer,
    Table<int8_t>& dl_encoded_buffer,
    std::vector<std::vector<ControlInfo>>& control_info_table,
    std::vector<size_t>& control_idx_list,
    SharedState* shared_state,
    BottleneckEncode& bottleneck_encode)
    : Doer(in_config, in_tid, freq_ghz)
    , dl_bits_buffer_(dl_bits_buffer)
    , dl_encoded_buffer_(dl_encoded_buffer)
    , shared_state_(shared_state)
    , total_ue_num_(cfg_->ue_end - cfg_->ue_start)
    , total_dycode_num_(cfg_->encode_thread_num)
    , control_info_table_(control_info_table)
    , control_idx_list_(control_idx_list)
    , bottleneck_encode_(bottleneck_encode)
{
    parity_buffer = (int8_t*)memalign(64,
        32768);
    encoded_buffer_temp = (int8_t*)memalign(64,
        32768);
}

DyEncode::~DyEncode()
{
    free(parity_buffer);
    free(encoded_buffer_temp);
}

void DyEncode::Launch(size_t frame_id, size_t symbol_id_dl, size_t ue_id)
{
    if (kDebugPrintInTask) {
        printf(
            "In doEncode thread %d: frame: %zu, symbol: %zu, ue %zu\n",
            tid_, frame_id, symbol_id_dl, ue_id);
    }

    std::vector<ControlInfo>& info_list = cfg_->fixed_control != -1 ? control_info_table_[cfg_->fixed_control] : control_info_table_[control_idx_list_[frame_id]];
    if (ue_id >= info_list.size()) {
        return;
    }
    ControlInfo& info = info_list[ue_id];
    size_t nRows = info.Bg == 1 ? 46 : 42;
    uint32_t cbCodewLen = ldpc_num_encoded_bits(info.Bg, info.Zc, nRows);

    int8_t* input_ptr
        = cfg_->get_info_bits(dl_bits_buffer_, symbol_id_dl, ue_id);

    ldpc_encode_helper(info.Bg, info.Zc, nRows,
        encoded_buffer_temp, parity_buffer, input_ptr);
    
    int8_t* final_output_ptr = cfg_->get_encoded_buf(
        dl_encoded_buffer_, frame_id, symbol_id_dl, ue_id);
    adapt_bits_for_mod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
        reinterpret_cast<uint8_t*>(final_output_ptr),
        bits_to_bytes(cbCodewLen), cfg_->mod_order_bits);
}

void DyEncode::LaunchStatic(size_t frame_id, size_t symbol_id_dl, size_t ue_id)
{
    LDPCconfig LDPC_config = cfg_->LDPC_config;
    if (kDebugPrintInTask) {
        printf(
            "In doEncode thread %d: frame: %zu, symbol: %zu, ue %zu\n",
            tid_, frame_id, symbol_id_dl, ue_id);
    }

    size_t nRows = LDPC_config.Bg == 1 ? 46 : 42;
    uint32_t cbCodewLen = ldpc_num_encoded_bits(LDPC_config.Bg, LDPC_config.Zc, nRows);

    int8_t* input_ptr
        = cfg_->get_info_bits(dl_bits_buffer_, symbol_id_dl, ue_id);

    ldpc_encode_helper(LDPC_config.Bg, LDPC_config.Zc, nRows,
        encoded_buffer_temp, parity_buffer, input_ptr);
    
    int8_t* final_output_ptr = cfg_->get_encoded_buf(
        dl_encoded_buffer_, frame_id, symbol_id_dl, ue_id);
    adapt_bits_for_mod(reinterpret_cast<uint8_t*>(encoded_buffer_temp),
        reinterpret_cast<uint8_t*>(final_output_ptr),
        bits_to_bytes(cbCodewLen), cfg_->mod_order_bits);
}

void DyEncode::StartWork() 
{
    cur_idx_ = tid_;
    cur_symbol_ = cur_idx_ / total_ue_num_;
    cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;

    size_t start_tsc = 0;
    size_t work_tsc_duration = 0;
    size_t encode_tsc_duration = 0;
    size_t state_operation_duration = 0;
    size_t loop_count = 0;
    size_t work_count = 0;
    size_t encode_count = 0;
    size_t encode_start_tsc;
    bool state_trigger = false;

    size_t encode_max = 0;

    size_t last_sleep_tsc = 0;

    while (cfg_->running && !SignalHandler::gotExitSignal()) {
        TRIGGER_TIMER(loop_count ++);
        size_t work_start_tsc;

        if (shared_state_->is_encode_ready(cur_frame_, cur_symbol_)) {
            if (unlikely(!state_trigger && cur_frame_ >= 200)) {
                start_tsc = rdtsc();
                state_trigger = true;
            }

            TRIGGER_TIMER({
                work_start_tsc = rdtsc();
                work_count ++;
                encode_start_tsc = rdtsc();
            });

            Launch(cur_frame_, cur_symbol_, cur_ue_);

            TRIGGER_TIMER({
                size_t encode_tmp_tsc = rdtsc() - encode_start_tsc;
                encode_tsc_duration += encode_tmp_tsc;
                encode_max = encode_max < encode_tmp_tsc ? encode_tmp_tsc : encode_max;
                encode_count ++;
                encode_start_tsc = rdtsc();
            });

            shared_state_->encode_done(cur_frame_, cur_symbol_);
            cur_idx_ += total_dycode_num_;
            cur_symbol_ = cur_idx_ / total_ue_num_;
            cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;
            if (cur_symbol_ >= cfg_->dl_data_symbol_num_perframe) {
                cur_idx_ = tid_;
                cur_symbol_ = cur_idx_ / total_ue_num_;
                cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;

                // shared_state_->encode_done(cur_frame_);
                cur_frame_++;
            }

            TRIGGER_TIMER({
                state_operation_duration += rdtsc() - encode_start_tsc;
                work_tsc_duration += rdtsc() - work_start_tsc;
            });
        }

        size_t cur_sleep_tsc = rdtsc();
        if (cur_sleep_tsc - last_sleep_tsc > freq_ghz_ / 1000) {
            last_sleep_tsc = cur_sleep_tsc;
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }

    if (cfg_->error) {
        printf("DyEncode Thread %d error traceback: encode (frame %zu, symbol %zu, ue %zu) "
            "total ue num %zu, total dyencode num %zu\n",
            tid_, cur_frame_, cur_symbol_, cur_ue_, total_ue_num_, total_dycode_num_);
    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("DyEncode Thread %u duration stats: total time used %.2lfms, "
        "encode %.2lfms (%zu, %.2lf%%, %.2lfus, %zu, %.2lfus, %.2lfus), stating %.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), "
        "working proportions (%zu/%zu: %.2lf%%)\n",
        tid_, cycles_to_ms(whole_duration, freq_ghz_),
        cycles_to_ms(encode_tsc_duration, freq_ghz_), encode_count, encode_tsc_duration * 100.0f / whole_duration, cycles_to_us(encode_max, freq_ghz_), encode_count_, cycles_to_us(encode_max_, freq_ghz_), cycles_to_us(encode_count_ == 0 ? 0 : encode_tsc_ / encode_count_, freq_ghz_),
        cycles_to_ms(state_operation_duration, freq_ghz_), state_operation_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz_), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, loop_count == 0 ? 0 : work_count * 100.0f / loop_count);

    bottleneck_encode_.encode = encode_tsc_duration * 100.0f / whole_duration;
    bottleneck_encode_.idle = idle_duration * 100.0f / whole_duration;
}

DyDecode::DyDecode(Config* in_config, int in_tid, double freq_ghz,
    Table<int8_t> demod_buffer_to_decode,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
    std::vector<std::vector<ControlInfo>>& control_info_table,
    std::vector<size_t>& control_idx_list,
    SharedState* shared_state,
    BottleneckDecode& bottleneck_decode)
    : Doer(in_config, in_tid, freq_ghz)
    , demod_buffer_to_decode_(demod_buffer_to_decode)
    , decoded_buffers_(decoded_buffers)
    , shared_state_(shared_state)
    , total_ue_num_(cfg_->ue_end - cfg_->ue_start)
    , total_dycode_num_(cfg_->decode_thread_num)
    , control_info_table_(control_info_table)
    , control_idx_list_(control_idx_list)
    , bottleneck_decode_(bottleneck_decode)
{
    resp_var_nodes_ = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
}

DyDecode::~DyDecode() { free(resp_var_nodes_); }

void DyDecode::Launch(size_t frame_id, size_t symbol_id_ul, size_t ue_id)
{
    LDPCconfig LDPC_config = cfg_->LDPC_config;
    const size_t frame_slot = frame_id % kFrameWnd;
    if (kDebugPrintInTask) {
        printf("In doDecode thread %d: frame: %zu, symbol: %zu, ue: %zu\n",
            tid_, frame_id, symbol_id_ul, ue_id);
    }

    std::vector<ControlInfo>& info_list = cfg_->fixed_control != -1 ? control_info_table_[cfg_->fixed_control] : control_info_table_[control_idx_list_[frame_id]];
    if (ue_id >= info_list.size()) {
        return;
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
            symbol_id_ul, ue_id, info.sc_start);

    uint8_t* decoded_buffer_ptr
        = decoded_buffers_[frame_slot][symbol_id_ul][ue_id];

    ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

    size_t start_tsc1 = worker_rdtsc();

    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

    size_t start_tsc2 = worker_rdtsc();
    decode_count_ ++;
    decode_max_ = decode_max_ < start_tsc2 - start_tsc1 ? start_tsc2 - start_tsc1 : decode_max_;
    decode_tsc_ += start_tsc2 - start_tsc1;
}

void DyDecode::LaunchStatic(size_t frame_id, size_t symbol_id_ul, size_t ue_id)
{
    LDPCconfig LDPC_config = cfg_->LDPC_config;
    const size_t frame_slot = frame_id % kFrameWnd;
    if (kDebugPrintInTask) {
        printf("In doDecode thread %d: frame: %zu, symbol: %zu, ue: %zu\n",
            tid_, frame_id, symbol_id_ul, ue_id);
    }

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };

    // Decoder setup
    int16_t numFillerBits = 0;
    size_t nRows = LDPC_config.Bg == 1 ? 46 : 42;
    uint32_t cbCodewLen = ldpc_num_encoded_bits(LDPC_config.Bg, LDPC_config.Zc, nRows);
    uint32_t cbLen = ldpc_num_input_bits(LDPC_config.Bg, LDPC_config.Zc);
    int16_t numChannelLlrs = cbCodewLen;

    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = LDPC_config.decoderIter;
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = LDPC_config.earlyTermination;
    ldpc_decoder_5gnr_request.Zc = LDPC_config.Zc;
    ldpc_decoder_5gnr_request.baseGraph = LDPC_config.Bg;
    ldpc_decoder_5gnr_request.nRows = nRows;

    int numMsgBits = cbLen - numFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes_;

    auto* llr_buffer_ptr
        = cfg_->get_demod_buf_to_decode(demod_buffer_to_decode_, frame_id,
            symbol_id_ul, ue_id, 0);

    uint8_t* decoded_buffer_ptr
        = decoded_buffers_[frame_slot][symbol_id_ul][ue_id];

    ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

    size_t start_tsc1 = worker_rdtsc();

    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

    size_t start_tsc2 = worker_rdtsc();
    decode_count_ ++;
    decode_max_ = decode_max_ < start_tsc2 - start_tsc1 ? start_tsc2 - start_tsc1 : decode_max_;
    decode_tsc_ += start_tsc2 - start_tsc1;
}

void DyDecode::StartWork()
{
    size_t last_frame = 0;
    cur_idx_ = tid_;
    cur_frame_ = cur_idx_ / (total_ue_num_ * cfg_->ul_data_symbol_num_perframe);
    cur_symbol_ = (cur_idx_ % (total_ue_num_* cfg_->ul_data_symbol_num_perframe)) / total_ue_num_;
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

    size_t last_sleep_tsc = 0;

    while (cfg_->running && !SignalHandler::gotExitSignal()) {
        TRIGGER_TIMER(loop_count ++);
        size_t work_start_tsc, state_start_tsc;

        TRIGGER_TIMER({
            work_start_tsc = rdtsc();
            state_start_tsc = rdtsc();
        });

        bool ret = shared_state_->received_all_demod_pkts_loss_tolerant(
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
            
            Launch(cur_frame_, cur_symbol_, cur_ue_);

            TRIGGER_TIMER({
                size_t decode_tmp_tsc = rdtsc() - decode_start_tsc;
                decode_tsc_duration += decode_tmp_tsc;
                decode_max = decode_max < decode_tmp_tsc ? decode_tmp_tsc : decode_max;
                decode_count ++;
                state_start_tsc = rdtsc();
            });

            cur_idx_ += total_dycode_num_;
            cur_frame_ = cur_idx_ / (total_ue_num_ * cfg_->ul_data_symbol_num_perframe);
            cur_symbol_ = (cur_idx_ % (total_ue_num_* cfg_->ul_data_symbol_num_perframe)) / total_ue_num_;
            cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;
            if (cur_frame_ > last_frame) {
                rt_assert(cur_frame_ == last_frame + 1, "Run cur frame calculation!");
                shared_state_->decode_done(last_frame);
                last_frame ++;
                if (unlikely(cur_frame_ == cfg_->frames_to_test)) {
                    TRIGGER_TIMER({
                        state_operation_duration += rdtsc() - state_start_tsc;
                        work_tsc_duration += rdtsc() - work_start_tsc;
                    });
                    break;
                }

                if (cfg_->fixed_control == -1 && shouldSleep(control_info_table_[control_idx_list_[cur_frame_]].size())) {
                    std::this_thread::sleep_for(std::chrono::microseconds(900));
                }
            }
            // if (cur_symbol_ >= cfg_->ul_data_symbol_num_perframe) {
            //     cur_idx_ = tid_;
            //     cur_symbol_ = cur_idx_ / total_ue_num_;
            //     cur_ue_ = cur_idx_ % total_ue_num_ + cfg_->ue_start;

            //     shared_state_->decode_done(cur_frame_);

            //     cur_frame_++;
            //     if (unlikely(cur_frame_ == cfg_->frames_to_test)) {
            //         TRIGGER_TIMER({
            //             state_operation_duration += rdtsc() - state_start_tsc;
            //             work_tsc_duration += rdtsc() - work_start_tsc;
            //         });
            //         break;
            //     }

            //     if (shouldSleep(control_info_table_[control_idx_list_[cur_frame_]].size())) {
            //         std::this_thread::sleep_for(std::chrono::microseconds(900));
            //     }
            // }
            
            TRIGGER_TIMER({
                state_operation_duration += rdtsc() - state_start_tsc;
                work_tsc_duration += rdtsc() - work_start_tsc;
            });
        }

        size_t cur_sleep_tsc = rdtsc();
        if (cur_sleep_tsc - last_sleep_tsc > freq_ghz_ / 1000) {
            last_sleep_tsc = cur_sleep_tsc;
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }

    if (cfg_->error) {
        printf("DeDecode Thread %d error traceback: decode (frame %zu, symbol %zu, ue %zu), recvd packets done %d\n", tid_, 
            cur_frame_, cur_symbol_, cur_ue_, shared_state_->received_all_demod_pkts_loss_tolerant(
                    cur_ue_, cur_frame_, cur_symbol_));
        shared_state_->print_receiving_demod_pkts(cur_ue_, cur_frame_, cur_symbol_);
    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("DoDecode Thread %u duration stats: total time used %.2lfms, "
        "decode %.2lfms (%zu, %.2lf%%, %.2lfus, %zu, %.2lfus, %.2lfus), stating %.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), "
        "working proportions (%zu/%zu: %.2lf%%)\n",
        tid_, cycles_to_ms(whole_duration, freq_ghz_),
        cycles_to_ms(decode_tsc_duration, freq_ghz_), decode_count, decode_tsc_duration * 100.0f / whole_duration, cycles_to_us(decode_max, freq_ghz_), decode_count_, cycles_to_us(decode_max_, freq_ghz_), cycles_to_us(decode_count_ == 0 ? 0 : decode_tsc_ / decode_count_, freq_ghz_),
        cycles_to_ms(state_operation_duration, freq_ghz_), state_operation_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz_), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, loop_count == 0 ? 0 : work_count * 100.0f / loop_count);

    bottleneck_decode_.decode = decode_tsc_duration * 100.0f / whole_duration;
    bottleneck_decode_.idle = idle_duration * 100.0f / whole_duration;
}

void DyDecode::StartWorkCentral() 
{
    size_t start_tsc = 0;
    size_t work_tsc_duration = 0;
    size_t decode_tsc_duration = 0;
    size_t state_operation_duration = 0;
    size_t loop_count = 0;
    size_t work_count = 0;
    size_t decode_start_tsc;
    bool state_trigger = false;

    size_t last_sleep_tsc = 0;

    while (cfg_->running && !SignalHandler::gotExitSignal()) {
        if (likely(state_trigger)) {
            loop_count ++;
        }
        size_t work_start_tsc, state_start_tsc;
        EventData event;
        if (task_queue_->try_dequeue(event)) {
            size_t tag, slot_id, symbol_id, ue_id;
            EventData resp;
            switch (event.event_type_) {
            case EventType::kDecode:
                tag = event.tags_[0];
                slot_id = gen_tag_t(tag).frame_id;
                symbol_id = gen_tag_t(tag).symbol_id;
                ue_id = gen_tag_t(tag).ue_id;
                if (unlikely(!state_trigger && slot_id >= 200)) {
                    start_tsc = rdtsc();
                    state_trigger = true;
                }
                if (likely(state_trigger)) {
                    work_start_tsc = rdtsc();
                    work_count ++;
                }
                if (likely(state_trigger)) {
                    decode_start_tsc = rdtsc();
                }
                Launch(slot_id, symbol_id, ue_id);
                if (likely(state_trigger)) {
                    decode_tsc_duration += rdtsc() - decode_start_tsc;
                }
                if (likely(state_trigger)) {
                    state_start_tsc = rdtsc();
                }
                resp = EventData(EventType::kDecode);
                TryEnqueueFallback(complete_queue_, complete_queue_token_, resp);
                if (likely(state_trigger)) {
                    state_operation_duration += rdtsc() - state_start_tsc;
                    work_tsc_duration += rdtsc() - work_start_tsc;
                }
                break;
            default:
                perror("Wrong type of task in Dycoding!");
                exit(1);
                break;
            }
        }

        size_t cur_sleep_tsc = rdtsc();
        if (cur_sleep_tsc - last_sleep_tsc > freq_ghz_ / 1000) {
            last_sleep_tsc = cur_sleep_tsc;
            std::this_thread::sleep_for(std::chrono::microseconds(1));
        }
    }

    size_t whole_duration = rdtsc() - start_tsc;
    size_t idle_duration = whole_duration - work_tsc_duration;
    printf("DoDecode Thread %d duration stats: total time used %.2lfms, "
        "decode %.2lfms (%.2lf%%), stating %.2lfms (%.2lf%%), idle %.2lfms (%.2lf%%), "
        "working proportions (%zu/%zu: %.2lf%%)\n",
        tid_, cycles_to_ms(whole_duration, freq_ghz_),
        cycles_to_ms(decode_tsc_duration, freq_ghz_), decode_tsc_duration * 100.0f / whole_duration,
        cycles_to_ms(state_operation_duration, freq_ghz_), state_operation_duration * 100.0f / whole_duration,
        cycles_to_ms(idle_duration, freq_ghz_), idle_duration * 100.0f / whole_duration,
        work_count, loop_count, loop_count == 0 ? 0 : work_count * 100.0f / loop_count);
}