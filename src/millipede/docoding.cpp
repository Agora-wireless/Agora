/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
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
    Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , raw_data_buffer_(in_raw_data_buffer)
    , encoded_buffer_(in_encoded_buffer)
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
    size_t symbol_offset = cfg->get_total_data_symbol_idx(frame_id, symbol_id);
    size_t cur_cb_id = cb_id % cfg->LDPC_config.nblocksInSymbol;
    size_t ue_id = cb_id / cfg->LDPC_config.nblocksInSymbol;
    if (kDebugPrintInTask) {
        printf(
            "In doEncode thread %d: frame: %zu, symbol: %zu, code block %zu\n",
            tid, frame_id, symbol_id, cur_cb_id);
    }

    size_t start_tsc = worker_rdtsc();

    int OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    int cbLenBytes = (LDPC_config.cbLen + 7) >> 3;
    int input_offset = cbLenBytes * cfg->LDPC_config.nblocksInSymbol * ue_id
        + cbLenBytes * cur_cb_id;
    int symbol_id_in_buffer = symbol_id - cfg->dl_data_symbol_start;
    int8_t* input_ptr
        = (int8_t*)raw_data_buffer_[symbol_id_in_buffer] + input_offset;
    int8_t* output_ptr = encoded_buffer_temp;

    ldpc_encode_helper(LDPC_config.Bg, LDPC_config.Zc, LDPC_config.nRows,
        output_ptr, parity_buffer, input_ptr);
    int cbCodedBytes = LDPC_config.cbCodewLen / cfg->mod_type;
    int output_offset = OFDM_DATA_NUM * ue_id + cbCodedBytes * cur_cb_id;
    int8_t* final_output_ptr
        = (int8_t*)encoded_buffer_[symbol_offset] + output_offset;
    adapt_bits_for_mod(output_ptr, final_output_ptr,
        (LDPC_config.cbCodewLen + 7) >> 3, cfg->mod_type);

    // printf("Encoded data\n");
    // int mod_type = cfg->mod_type;
    // int num_mod = LDPC_config.cbCodewLen / mod_type;
    // for(int i = 0; i < num_mod; i++) {
    //     printf("%u ", *(final_output_ptr + i));
    // }
    // printf("\n");

    double duration = worker_rdtsc() - start_tsc;
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
    Table<int8_t>& in_demod_buffer, Table<uint8_t>& in_decoded_buffer,
    //Table<int>& in_decoded_bits_count, Table<int>& in_error_bits_count,
    PhyStats* in_phy_stats, Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , llr_buffer_(in_demod_buffer)
    , decoded_buffer_(in_decoded_buffer)
    //, decoded_bits_count_(in_decoded_bits_count)
    //, error_bits_count_(in_error_bits_count)
    , phy_stats(in_phy_stats)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kDecode, in_tid);
    resp_var_nodes = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
}

DoDecode::~DoDecode() { free(resp_var_nodes); }

Event_data DoDecode::launch(size_t tag)
{
    LDPCconfig LDPC_config = cfg->LDPC_config;
    size_t frame_id = gen_tag_t(tag).frame_id;
    size_t symbol_id = gen_tag_t(tag).symbol_id;
    size_t cb_id = gen_tag_t(tag).cb_id;
    size_t symbol_offset
        = cfg->get_total_data_symbol_idx_ul(frame_id, symbol_id);
    size_t cur_cb_id = cb_id % cfg->LDPC_config.nblocksInSymbol;
    size_t ue_id = cb_id / cfg->LDPC_config.nblocksInSymbol;
    if (kDebugPrintInTask) {
        printf("In doDecode thread %d: frame: %zu, symbol: %zu, code block: "
               "%zu ue:  "
               "%zu\n",
            tid, frame_id, symbol_id, cur_cb_id, ue_id);
    }

    size_t start_tsc = worker_rdtsc();

    struct bblib_ldpc_decoder_5gnr_request ldpc_decoder_5gnr_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_decoder_5gnr_response {
    };

    // Decoder setup
    int16_t numFillerBits = 0;
    int16_t numChannelLlrs = LDPC_config.cbCodewLen;

    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = LDPC_config.decoderIter;
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = LDPC_config.earlyTermination;
    ldpc_decoder_5gnr_request.Zc = LDPC_config.Zc;
    ldpc_decoder_5gnr_request.baseGraph = LDPC_config.Bg;
    ldpc_decoder_5gnr_request.nRows = LDPC_config.nRows;

    int numMsgBits = LDPC_config.cbLen - numFillerBits;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    ldpc_decoder_5gnr_response.varNodes = resp_var_nodes;

    size_t input_offset
        = cfg->OFDM_DATA_NUM * ue_id + LDPC_config.cbCodewLen * cur_cb_id;
    size_t llr_buffer_offset = input_offset * cfg->mod_type;
    ldpc_decoder_5gnr_request.varNodes
        = (int8_t*)llr_buffer_[symbol_offset] + llr_buffer_offset;
    size_t cbLenBytes = (LDPC_config.cbLen + 7) >> 3;
    size_t output_offset = cbLenBytes * cfg->LDPC_config.nblocksInSymbol * ue_id
        + cbLenBytes * cur_cb_id;
    ldpc_decoder_5gnr_response.compactedMessageBytes
        = (uint8_t*)decoded_buffer_[symbol_offset] + output_offset;

    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

    if (kPrintLLRData) {
        printf("LLR data, symbol_offset: %zu, input offset: %zu\n",
            symbol_offset, input_offset);
        for (size_t i = 0; i < LDPC_config.cbCodewLen; i++) {
            printf(
                "%d ", *(llr_buffer_[symbol_offset] + llr_buffer_offset + i));
        }
        printf("\n");
    }

    if (kPrintDecodedData) {
        printf("Decoded data\n");
        for (size_t i = 0; i < (LDPC_config.cbLen >> 3); i++) {
            printf(
                "%u ", *(decoded_buffer_[symbol_offset] + output_offset + i));
        }
        printf("\n");
    }

    if (!kEnableMac && kPrintPhyStats && symbol_id == cfg->UL_PILOT_SYMS) {
        phy_stats->update_decoded_bits(ue_id, symbol_offset, cbLenBytes * 8);
        phy_stats->increment_decoded_blocks(ue_id, symbol_offset);
        size_t block_error(0);
        for (size_t i = 0; i < (LDPC_config.cbLen >> 3); i++) {
            uint8_t rx_byte = decoded_buffer_[symbol_offset][output_offset + i];
            uint8_t tx_byte = cfg->ul_bits[symbol_id][output_offset + i];
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
