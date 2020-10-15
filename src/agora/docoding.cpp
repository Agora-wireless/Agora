#include "docoding.hpp"
#include "concurrent_queue_wrapper.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include <malloc.h>

static constexpr bool kPrintEncodedData = false;
static constexpr bool kPrintLLRData = false;
static constexpr bool kPrintDecodedData = false;

DoEncode::DoEncode(Config* in_config, int in_tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<int8_t>& raw_data_buffer,
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& encoded_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , raw_data_buffer_(raw_data_buffer)
    , encoded_buffer_(encoded_buffer)
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

DoEncode::~DoEncode()
{
    free(parity_buffer);
    free(encoded_buffer_temp);
}

Event_data DoEncode::launch(size_t tag)
{
    LDPCconfig LDPC_config = cfg->LDPC_config;
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t cb_id = gen_tag_t(tag).cb_id;
    size_t cur_cb_id = cb_id % cfg->LDPC_config_dl.nCb;
    size_t ue_id = cb_id / cfg->LDPC_config_dl.nCb;
    if (kDebugPrintInTask) {
        printf("In doEncode thread %d: frame: %zu, symbol: %zu, code block "
               "%zu, ue: %zu\n",
            tid, frame_id, symbol_id, cur_cb_id, ue_id);
    }

    size_t start_tsc = worker_rdtsc();

    size_t symbol_idx_dl = cfg->get_dl_symbol_idx(frame_id, symbol_id);
    int8_t* input_ptr
        = cfg->get_info_bits(raw_data_buffer_, symbol_idx_dl, ue_id, cur_cb_id);

    ldpc_encode_helper(cfg->LDPC_config_dl.Bg, cfg->LDPC_config_dl.Zc,
        cfg->LDPC_config_dl.nRows, encoded_buffer_temp, parity_buffer,
        input_ptr);
    auto* encoded_temp_ptr = reinterpret_cast<uint8_t*>(encoded_buffer_temp);
    if (kPrintEncodedData) {
        printf("Encoded data\n");
        for (int i = 0; i < cfg->LDPC_config_dl.num_encoded_bytes(); i++) {
            printf("%u ", *(encoded_temp_ptr + i));
        }
        printf("\n");
    }
    for (size_t i = 0;
         i < cfg->LDPC_config_dl.lut_cb_to_symbol[cur_cb_id].size(); i++) {
        int8_t* final_output_ptr
            = encoded_buffer_[frame_id][symbol_idx_dl][ue_id]
            + LDPC_config_dl.get_chunk_start_sc(cur_cb_id, i);
        adapt_bits_for_mod(encoded_temp_ptr,
            reinterpret_cast<uint8_t*>(final_output_ptr),
            cfg->LDPC_config_dl.lut_cb_chunks_bytes[cur_cb_id][i],
            cfg->mod_order_bits);
        ul_encoded_ptr += cfg->LDPC_config_dl.lut_cb_chunks_bytes[cur_cb_id][i];
    }

    size_t duration = worker_rdtsc() - start_tsc;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (cycles_to_us(duration, freq_ghz) > 500) {
        printf("Thread %d Encode takes %.2f\n", tid,
            cycles_to_us(duration, freq_ghz));
    }

    return Event_data(EventType::kEncode, tag);
}

DoDecode::DoDecode(Config* in_config, int in_tid, double freq_ghz,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    PtrGrid<kFrameWnd, kMaxUEs, int8_t>& demod_buffers,
    PtrGrid<kFrameWnd, kMaxUEs, uint8_t>& decoded_buffers,
    PhyStats* in_phy_stats, Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , demod_buffers_(demod_buffers)
    , decoded_buffers_(decoded_buffers)
    , phy_stats(in_phy_stats)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kDecode, in_tid);
    resp_var_nodes = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
}

DoDecode::~DoDecode() { free(resp_var_nodes); }

Event_data DoDecode::launch(size_t tag)
{
    const size_t frame_id = gen_tag_t(tag).frame_id;
    const size_t symbol_idx_ul = gen_tag_t(tag).symbol_id;
    const size_t cb_id = gen_tag_t(tag).cb_id;
    const size_t symbol_offset
        = cfg->get_total_data_symbol_idx_ul(frame_id, symbol_idx_ul);
    const size_t cur_cb_id = cb_id % cfg->LDPC_config_ul.nCb;
    const size_t ue_id = cb_id / cfg->LDPC_config_ul.nCb;
    const size_t frame_slot = frame_id % kFrameWnd;
    if (kDebugPrintInTask) {
        printf("In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
               "%zu, ue: %zu\n",
            tid, frame_id, symbol_idx_ul, cur_cb_id, ue_id);
    }

    size_t start_tsc = worker_rdtsc();

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };

    ldpc_decoder_5gnr_request.numChannelLlrs = cfg->LDPC_config_ul.cbCodewLen;
    ldpc_decoder_5gnr_request.numFillerBits = 0;
    ldpc_decoder_5gnr_request.maxIterations = cfg->LDPC_config_ul.decoderIter;
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = cfg->LDPC_config_ul.earlyTermination;
    ldpc_decoder_5gnr_request.Zc = cfg->LDPC_config_ul.Zc;
    ldpc_decoder_5gnr_request.baseGraph = cfg->LDPC_config_ul.Bg;
    ldpc_decoder_5gnr_request.nRows = cfg->LDPC_config_ul.nRows;

    int numMsgBits = LDPC_config.cbLen - numFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits
        = cfg->LDPC_config_ul.cbLen - ldpc_decoder_5gnr_request.numFillerBits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

    size_t start_symbol = cfg->LDPC_config_ul.lut_cb_to_symbol[cur_cb_id][0];
    size_t start_sc = cfg->LDPC_config_ul.get_chunk_start_sc[cur_cb_id][0];
    int8_t* llr_buffer_ptr = demod_buffers_[frame_slot][ue_id]
        + (cfg->mod_order_bits
              * (start_symbol * cfg->OFDM_DATA_NUM + start_sc));

    uint8_t* decoded_buffer_ptr = decoded_buffers_[frame_slot][ue_id]
        + (cur_cb_id * roundup<64>(cfg->num_bytes_per_cb));

    ldpc_decoder_5gnr_request.varNodes = llr_buffer_ptr;
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer_ptr;

    size_t start_tsc1 = worker_rdtsc();
    duration_stat->task_duration[1] += start_tsc1 - start_tsc;

    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

    size_t start_tsc2 = worker_rdtsc();
    duration_stat->task_duration[2] += start_tsc2 - start_tsc1;

    if (kPrintLLRData) {
        printf("LLR data, symbol_offset: %zu\n", symbol_offset);
        for (size_t i = 0; i < LDPC_config.cbCodewLen; i++) {
            printf("%d ", *(llr_buffer_ptr + i));
        }
        printf("\n");
    }

    if (kPrintDecodedData) {
        printf("Decoded data\n");
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
