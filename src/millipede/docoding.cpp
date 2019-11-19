/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "docoding.hpp"

using namespace arma;
DoCoding::DoCoding(Config *cfg, int in_tid, 
    moodycamel::ConcurrentQueue<Event_data> *in_complete_task_queue, moodycamel::ProducerToken *in_task_ptok,
    int8_t **in_raw_data_buffer, int8_t **in_encoded_buffer, int8_t **in_demod_buffer, uint8_t **in_decoded_buffer, 
    Stats *in_stats_manager) 
{
    config_ = cfg;
    LDPC_config = cfg->LDPC_config;
    // BS_ANT_NUM = cfg->BS_ANT_NUM;
    UE_NUM = cfg->UE_NUM;
    // OFDM_CA_NUM = cfg->OFDM_CA_NUM;
    OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;

    tid = in_tid;
    complete_task_queue_ = in_complete_task_queue;
    task_ptok = in_task_ptok;

    encoded_buffer = in_encoded_buffer;
    decoded_buffer = in_decoded_buffer;
    raw_data_buffer = in_raw_data_buffer;
    llr_buffer = in_demod_buffer;

    Encode_task_duration = in_stats_manager->encode_stats_worker.task_duration;
    Encode_task_count = in_stats_manager->encode_stats_worker.task_count;
    Decode_task_duration = in_stats_manager->decode_stats_worker.task_duration;
    Decode_task_count = in_stats_manager->decode_stats_worker.task_count;

    int16_t numChannelLlrs = LDPC_config.cbCodewLen;
    
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


    // decoder setup --------------------------------------------------------------
    int16_t numFillerBits = 0;
    ldpc_decoder_5gnr_request.numChannelLlrs = numChannelLlrs;                      
    ldpc_decoder_5gnr_request.numFillerBits = numFillerBits;                    
    ldpc_decoder_5gnr_request.maxIterations = LDPC_config.decoderIter;              
    ldpc_decoder_5gnr_request.enableEarlyTermination = LDPC_config.earlyTermination;   
    const long int buffer_len = 1024 * 1024;
    ldpc_decoder_5gnr_request.Zc = LDPC_config.Zc;                           
    ldpc_decoder_5gnr_request.baseGraph = LDPC_config.Bg;                    
    ldpc_decoder_5gnr_request.nRows = LDPC_config.nRows;                   
    
    int numMsgBits = LDPC_config.cbLen - numFillerBits;
    int numMsgBytes = (numMsgBits + 7) / 8;
    ldpc_decoder_5gnr_response.numMsgBits = numMsgBits;
    alloc_buffer_1d(&(ldpc_decoder_5gnr_response.varNodes), buffer_len, 32, 1); 
    alloc_buffer_1d(&encoded_buffer_temp, OFDM_DATA_NUM * 16, 32, 1);
    // ldpc_decoder_5gnr_response.varNodes = aligned_alloc()aligned_malloc<int16_t>(buffer_len, 32);
    // memset(ldpc_decoder_5gnr_response.varNodes, 0, numMsgBytes);

}


DoCoding::~DoCoding()
{

}


int8_t cvt_to_int8(int8_t x, int start, int end) 
{
    int8_t re = 0;
    int index = end - start -1;
    for (int i = start; i < end; i++) {
        re |= ((x >> i) & 0x1) << index;
        index--;
    }
    return re;
}

void adapt_bits_for_mod(int8_t *vec_in, int8_t *vec_out, int len, int mod_order)
{
    int start_bit = 0, end_bit = 0;
    int out_idx = 0;
    for (int i = 0; i < len; i++) {
        end_bit = start_bit + mod_order;
        while (end_bit <= 8) {
            vec_out[out_idx] = cvt_to_int8(vec_in[i], start_bit, end_bit);
            out_idx++;
            start_bit = end_bit;
            end_bit = start_bit + mod_order;
        } 
        if (i + 1 < len) {
            int nremaining_bits = 8 - start_bit;
            int nbits_in_next = mod_order - (nremaining_bits);
            int nremaining_bits_next = 8 - nbits_in_next;
            vec_out[out_idx] = (cvt_to_int8(vec_in[i], start_bit, 8) << nremaining_bits ) + (cvt_to_int8(vec_in[i + 1], 0, nbits_in_next));
            out_idx++;
            start_bit = nbits_in_next;
        }
    }
}

void DoCoding::Encode(int offset)
{
    int frame_id, symbol_id, cb_id;
    interpreteOffset3d(offset, &frame_id, &symbol_id, &cb_id);

    #if DEBUG_PRINT_IN_TASK
    printf("In doEncode thread %d: frame: %d, symbol: %d, code block %d\n", tid, frame_id, symbol_id, cb_id);
    #endif

    #if DEBUG_UPDATE_STATS    
    double start_time = get_time();
    #endif

    int ue_id = cb_id / LDPC_config.nblocksInSymbol;
    int cur_cb_id = cb_id % LDPC_config.nblocksInSymbol;
    int input_offset = OFDM_DATA_NUM * ue_id + LDPC_config.cbLen * cur_cb_id;
    int output_offset = OFDM_DATA_NUM * ue_id + LDPC_config.cbCodewLen * cur_cb_id;
    int symbol_offset = config_->data_symbol_num_perframe * frame_id + symbol_id;
    int8_t *input_ptr = raw_data_buffer[symbol_id] + input_offset;
    int8_t *output_ptr = encoded_buffer_temp;
    // int8_t *output_ptr = encoded_buffer[symbol_offset] + output_offset;
    ldpc_adapter_func(input_ptr, internalBuffer0, LDPC_config.Zc, LDPC_config.cbLen, 1);
    // encode
    ldpc_encoder_func(internalBuffer0, internalBuffer1, pMatrixNumPerCol, pAddr, pShiftMatrix, (int16_t) LDPC_config.Zc, i_LS);
    // scatter the output back to compacted 
    // combine the input sequence and the parity bits into codeword outputs
    memcpy(internalBuffer2, internalBuffer0 + 2 * PROC_BYTES, (LDPC_config.cbLen / LDPC_config.Zc - 2) * PROC_BYTES);
    memcpy(internalBuffer2+(LDPC_config.cbLen / LDPC_config.Zc - 2) * PROC_BYTES, internalBuffer1, LDPC_config.cbEncLen / LDPC_config.Zc * PROC_BYTES);

    ldpc_adapter_func(output_ptr, internalBuffer2, LDPC_config.Zc, LDPC_config.cbCodewLen, 0);
    int8_t *final_output_ptr = encoded_buffer[symbol_offset] + output_offset;
    adapt_bits_for_mod(output_ptr, final_output_ptr, LDPC_config.cbCodewLen / 8, config_->mod_type);

    #if DEBUG_UPDATE_STATS   
    double duration = get_time() - start_time;
    Encode_task_count[tid * 16] = Encode_task_count[tid * 16] + 1;
    Encode_task_duration[tid * 8][0] += duration;
    if (duration > 500) {
            printf("Thread %d Encode takes %.2f\n", tid, duration);
    }
    #endif

    // inform main thread
    Event_data Encode_finish_event;
    Encode_finish_event.event_type = EVENT_ENCODE;
    Encode_finish_event.data = offset;
    

    if ( !complete_task_queue_->enqueue(*task_ptok, Encode_finish_event ) ) {
        printf("Encode message enqueue failed\n");
        exit(0);
    }


}


void DoCoding::Decode(int offset) 
{
    int frame_id, symbol_id, cb_id;
    interpreteOffset3d(offset, &frame_id, &symbol_id, &cb_id);
    #if DEBUG_PRINT_IN_TASK
    printf("In doDecode thread %d: frame: %d, symbol: %d, code block %d\n", tid, frame_id, symbol_id, cb_id);
    #endif

    #if DEBUG_UPDATE_STATS    
    double start_time = get_time();
    #endif

    int ue_id = cb_id / LDPC_config.nblocksInSymbol;
    int cur_cb_id = cb_id % LDPC_config.nblocksInSymbol;
    int llr_buffer_offset = (OFDM_DATA_NUM * ue_id + LDPC_config.cbCodewLen * cur_cb_id) * config_->mod_type;
    int decoded_buffer_offset = OFDM_DATA_NUM * ue_id + LDPC_config.cbLen * cur_cb_id;
    int symbol_offset = config_->data_symbol_num_perframe * frame_id + symbol_id;
    ldpc_decoder_5gnr_request.varNodes = llr_buffer[symbol_offset] + llr_buffer_offset;
    ldpc_decoder_5gnr_response.compactedMessageBytes = decoded_buffer[symbol_offset] + decoded_buffer_offset;
    // printf("In doDecode thread %d: frame: %d, symbol: %d, code block %d, llr offset %d, decode offset: %d, request_addr: %lx, response_addr: %lx\n", 
    //     tid, frame_id, symbol_id, cb_id, llr_buffer_offset, decoded_buffer_offset, llr_buffer[symbol_offset] + llr_buffer_offset,decoded_buffer[0]);
    bblib_ldpc_decoder_5gnr(&ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);
    // inform main thread


    #if DEBUG_UPDATE_STATS   
    double duration = get_time() - start_time;
    Decode_task_count[tid * 16] = Decode_task_count[tid * 16] + 1;
    Decode_task_duration[tid * 8][0] += duration;
    if (duration > 500) {
            printf("Thread %d Decode takes %.2f\n", tid, duration);
    }
    #endif

    Event_data Decode_finish_event;
    Decode_finish_event.event_type = EVENT_DECODE;
    Decode_finish_event.data = offset;

    if ( !complete_task_queue_->enqueue(*task_ptok, Decode_finish_event ) ) {
        printf("Decode message enqueue failed\n");
        exit(0);
    }
}
