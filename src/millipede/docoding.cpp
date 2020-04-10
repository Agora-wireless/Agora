/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "docoding.hpp"
#include "concurrent_queue_wrapper.hpp"

using namespace arma;

#ifndef __has_builtin
#define __has_builtin(x) 0
#endif

static inline uint8_t bitreverse8(uint8_t x)
{
#if __has_builtin(__builtin_bireverse8)
    return (__builtin_bitreverse8(x));
#else
    x = (x << 4) | (x >> 4);
    x = ((x & 0x33) << 2) | ((x >> 2) & 0x33);
    x = ((x & 0x55) << 1) | ((x >> 1) & 0x55);
    return (x);
#endif
}

/*
 * Copy packed, bit-reversed m-bit fields (m == mod_order) stored in
 * vec_in[0..len-1] into unpacked vec_out.  Storage at vec_out must be
 * at least 8*len/m bytes.
 */
static void adapt_bits_for_mod(
    int8_t* vec_in, int8_t* vec_out, int len, int mod_order)
{
    int bits_avail = 0;
    uint16_t bits = 0;
    for (int i = 0; i < len; i++) {
        bits |= bitreverse8(vec_in[i]) << 8 - bits_avail;
        bits_avail += 8;
        while (bits_avail >= mod_order) {
            *vec_out++ = bits >> (16 - mod_order);
            bits <<= mod_order;
            bits_avail -= mod_order;
        }
    }
}

DoEncode::DoEncode(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<int8_t>& in_raw_data_buffer, Table<int8_t>& in_encoded_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, complete_task_queue,
          worker_producer_token)
    , raw_data_buffer_(in_raw_data_buffer)
    , encoded_buffer_(in_encoded_buffer)
{
    duration_stat
        = in_stats_manager->get_duration_stat(DoerType::kEncode, in_tid);
    int OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    alloc_buffer_1d(&encoded_buffer_temp, OFDM_DATA_NUM * 16, 32, 1);
    LDPCconfig LDPC_config = cfg->LDPC_config;
    int Zc = LDPC_config.Zc;
    if ((Zc % 15) == 0)
        i_LS = 7;
    else if ((Zc % 13) == 0)
        i_LS = 6;
    else if ((Zc % 11) == 0)
        i_LS = 5;
    else if ((Zc % 9) == 0)
        i_LS = 4;
    else if ((Zc % 7) == 0)
        i_LS = 3;
    else if ((Zc % 5) == 0)
        i_LS = 2;
    else if ((Zc % 3) == 0)
        i_LS = 1;
    else
        i_LS = 0;

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

Event_data DoEncode::launch(int offset)
{
    LDPCconfig LDPC_config = cfg->LDPC_config;
    int nblocksInSymbol = LDPC_config.nblocksInSymbol;
    int cur_cb_id = offset % nblocksInSymbol;
    int UE_NUM = cfg->UE_NUM;
    int ue_id = (offset / nblocksInSymbol) % UE_NUM;
    int symbol_offset = offset / (UE_NUM * LDPC_config.nblocksInSymbol);
    int data_subframe_num_perframe = cfg->data_symbol_num_perframe;
    int symbol_id = symbol_offset % data_subframe_num_perframe;
#if DEBUG_PRINT_IN_TASK
    int frame_id = symbol_offset / data_subframe_num_perframe;
    printf("In doEncode thread %d: frame: %d, symbol: %d, code block %d\n", tid,
        frame_id, symbol_id, cur_cb_id);
#endif

#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif

    int OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    int cbLenBytes = (LDPC_config.cbLen + 7) >> 3;
    int input_offset
        = cbLenBytes * nblocksInSymbol * ue_id + cbLenBytes * cur_cb_id;
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

    // int frame_id = symbol_offset / data_subframe_num_perframe;
    // printf("In doEncode thread %d: frame: %d, symbol: %d, ue: %d, code block
    // %d\n",
    //     tid, frame_id, symbol_id, ue_id, cur_cb_id);
    // printf("Encoded data\n");
    // int mod_type = cfg->mod_type;
    // int num_mod = LDPC_config.cbCodewLen / mod_type;
    // for(int i = 0; i < num_mod; i++) {
    //     printf("%u ", *(final_output_ptr + i));
    // }
    // printf("\n");

#if DEBUG_UPDATE_STATS
    double duration = get_time() - start_time;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (duration > 500) {
        printf("Thread %d Encode takes %.2f\n", tid, duration);
    }
#endif

    /* Inform main thread */
    Event_data encode_finish_event(EventType::kEncode, offset);
    // consumer_.handle(encode_finish_event);
    return encode_finish_event;
}

DoDecode::DoDecode(Config* in_config, int in_tid,
    moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
    moodycamel::ProducerToken* worker_producer_token,
    Table<int8_t>& in_demod_buffer, Table<uint8_t>& in_decoded_buffer,
    Stats* in_stats_manager)
    : Doer(in_config, in_tid, in_task_queue, complete_task_queue,
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

Event_data DoDecode::launch(int offset)
{
    LDPCconfig LDPC_config = cfg->LDPC_config;
    int nblocksInSymbol = LDPC_config.nblocksInSymbol;
    int cur_cb_id = offset % nblocksInSymbol;
    int UE_NUM = cfg->UE_NUM;
    int ue_id = (offset / nblocksInSymbol) % UE_NUM;
    int symbol_offset = offset / (UE_NUM * LDPC_config.nblocksInSymbol);
    int data_subframe_num_perframe = cfg->ul_data_symbol_num_perframe;
    int symbol_id = symbol_offset % data_subframe_num_perframe;
#if DEBUG_PRINT_IN_TASK
    int frame_id = symbol_offset / data_subframe_num_perframe;
    printf("In doDecode thread %d: frame: %d, symbol: %d, code block %d\n", tid,
        frame_id, symbol_id, cur_cb_id);
#endif

#if DEBUG_UPDATE_STATS
    double start_time = get_time();
#endif

    int OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    int input_offset
        = OFDM_DATA_NUM * ue_id + LDPC_config.cbCodewLen * cur_cb_id;
    int llr_buffer_offset = input_offset * cfg->mod_type;
    ldpc_decoder_5gnr_request.varNodes
        = (int8_t*)llr_buffer_[symbol_offset] + llr_buffer_offset;
    int cbLenBytes = (LDPC_config.cbLen + 7) >> 3;
    int output_offset
        = cbLenBytes * nblocksInSymbol * ue_id + cbLenBytes * cur_cb_id;
    ldpc_decoder_5gnr_response.compactedMessageBytes
        = (uint8_t*)decoded_buffer_[symbol_offset] + output_offset;

    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

    // int frame_id = symbol_offset / data_subframe_num_perframe;
    // printf("In doDecode thread %d: frame: %d, symbol: %d, code block %d\n",
    // tid,
    //     frame_id, symbol_id, cur_cb_id);
    // printf("Decode data\n");
    // for(int i = 0; i < LDPC_config.cbLen >> 3; i++) {
    //     // printf("%u ", *(ldpc_decoder_5gnr_response.compactedMessageBytes +
    //     i)); printf("%u ", *(decoded_buffer_[symbol_offset] + output_offset +
    //     i));
    // }
    // printf("\n");

#if DEBUG_UPDATE_STATS
    double duration = get_time() - start_time;
    duration_stat->task_duration[0] += duration;
    duration_stat->task_count++;
    if (duration > 500) {
        printf("Thread %d Decode takes %.2f\n", tid, duration);
    }
#endif

    /* Inform main thread */
    return Event_data(EventType::kDecode, offset);
}
