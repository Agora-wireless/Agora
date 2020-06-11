/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "docoding.hpp"
#include "concurrent_queue_wrapper.hpp"

using namespace arma;

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
    int OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    alloc_buffer_1d(&encoded_buffer_temp, OFDM_DATA_NUM * 16, 32, 1);
    LDPCconfig LDPC_config = cfg->LDPC_config;
    uint16_t Zc = LDPC_config.Zc;
    i_LS = select_base_matrix_entry(Zc);

    if (LDPC_config.Bg == 1) {
        pShiftMatrix = Bg1HShiftMatrix + i_LS * BG1_NONZERO_NUM;
        pMatrixNumPerCol = Bg1MatrixNumPerCol;
        pAddr = Bg1Address;
    } else {
        pShiftMatrix = Bg2HShiftMatrix + i_LS * BG2_NONZERO_NUM;
        pMatrixNumPerCol = Bg2MatrixNumPerCol;
        pAddr = Bg2Address;
    }

    ldpc_adapter_func = ldpc_select_adapter_func(LDPC_config.Zc);
    ldpc_encoder_func = ldpc_select_encoder_func(LDPC_config.Bg);
}

DoEncode::~DoEncode()
{
    free_buffer_1d(&encoded_buffer_temp);
    free_buffer_1d(&ldpc_decoder_5gnr_response.varNodes);
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

    ldpc_adapter_func(
        input_ptr, internalBuffer0, LDPC_config.Zc, LDPC_config.cbLen, 1);
    ldpc_encoder_func(internalBuffer0, internalBuffer1, pMatrixNumPerCol, pAddr,
        pShiftMatrix, (int16_t)LDPC_config.Zc, i_LS);
    /* scatter the output back to compacted
     * combine the input sequence and the parity bits into codeword outputs */
    memcpy(internalBuffer2, internalBuffer0 + 2 * PROC_BYTES,
        (LDPC_config.cbLen / LDPC_config.Zc - 2) * PROC_BYTES);
    memcpy(
        internalBuffer2 + (LDPC_config.cbLen / LDPC_config.Zc - 2) * PROC_BYTES,
        internalBuffer1, LDPC_config.cbEncLen / LDPC_config.Zc * PROC_BYTES);

    ldpc_adapter_func(
        output_ptr, internalBuffer2, LDPC_config.Zc, LDPC_config.cbCodewLen, 0);
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
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, freq_ghz, in_task_queue, complete_task_queue,
          worker_producer_token)
    , llr_buffer_(in_demod_buffer)
    , decoded_buffer_(in_decoded_buffer)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kDecode, in_tid);
    // decoder setup
    // --------------------------------------------------------------
    int16_t numFillerBits = 0;
    LDPCconfig LDPC_config = cfg->LDPC_config;
    int16_t numChannelLlrs = LDPC_config.cbCodewLen;
    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;
    ldpc_decoder_5gnr_request.maxIterations = LDPC_config.decoderIter;
    ldpc_decoder_5gnr_request.enableEarlyTermination
        = LDPC_config.earlyTermination;
    ldpc_decoder_5gnr_request.Zc = LDPC_config.Zc;
    ldpc_decoder_5gnr_request.baseGraph = LDPC_config.Bg;
    ldpc_decoder_5gnr_request.nRows = LDPC_config.nRows;

    const long int buffer_len = 1024 * 1024;
    int numMsgBits = LDPC_config.cbLen - numFillerBits;
    // int numMsgBytes = (numMsgBits + 7) / 8;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    alloc_buffer_1d(&(ldpc_decoder_5gnr_response.varNodes), buffer_len, 32, 1);
    // ldpc_decoder_5gnr_response.varNodes =
    // aligned_alloc()aligned_malloc<int16_t>(buffer_len, 32);
    // memset(ldpc_decoder_5gnr_response.varNodes, 0, numMsgBytes);
}

DoDecode::~DoDecode() {}

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

    // printf("LLR data, symbol_offset: %zu, input offset: %zu\n", symbol_offset,
    //     input_offset);
    // for (size_t i = 0; i < LDPC_config.cbCodewLen; i++) {
    //     // printf("%u ", *(ldpc_decoder_5gnr_response.compactedMessageBytes + i));
    //     printf("%d ", *(llr_buffer_[symbol_offset] + llr_buffer_offset + i));
    // }
    // printf("\n");
    // printf("Decode data\n");
    // for (int i = 0; i<LDPC_config.cbLen>> 3; i++) {
    //     // printf("%u ", *(ldpc_decoder_5gnr_response.compactedMessageBytes + i));
    //     printf("%u ", *(decoded_buffer_[symbol_offset] + output_offset + i));
    // }
    // printf("\n");

    double duration = worker_rdtsc() - start_tsc;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (cycles_to_us(duration, freq_ghz) > 500) {
        printf("Thread %d Decode takes %.2f\n", tid,
            cycles_to_us(duration, freq_ghz));
    }

    return Event_data(EventType::kDecode, tag);
}
