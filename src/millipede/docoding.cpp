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
    double **in_Encode_task_duration, int *in_Encode_task_count, double **in_Decode_task_duration, int *in_Decode_task_count) 
{
    config_ = cfg;
    LDPC_config = cfg->LDPC_config;
    // BS_ANT_NUM = cfg->BS_ANT_NUM;
    UE_NUM = cfg->UE_NUM;
    // OFDM_CA_NUM = cfg->OFDM_CA_NUM;
    // OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;

    tid = in_tid;
    complete_task_queue_ = in_complete_task_queue;
    task_ptok = in_task_ptok;

    encoded_buffer = in_encoded_buffer;
    decoded_buffer = in_decoded_buffer;
    raw_data_buffer = in_raw_data_buffer;
    llr_buffer = in_demod_buffer;

    // buffers for encoders
    __attribute__ ((aligned (64))) int8_t internalBuffer0[BG1_ROW_TOTAL * PROC_BYTES] = {0};
    __attribute__ ((aligned (64))) int8_t internalBuffer1[BG1_ROW_TOTAL * PROC_BYTES] = {0};
    __attribute__ ((aligned (64))) int8_t internalBuffer2[BG1_COL_TOTAL * PROC_BYTES] = {0};

    Encode_task_duration = in_Encode_task_duration;
    Encode_task_count = in_Encode_task_count;
    Decode_task_duration = in_Decode_task_duration;
    Decode_task_count = in_Decode_task_count;

    int16_t numChannelLlrs = LDPC_config.cbCodewLen;
    const int16_t *pShiftMatrix;
    const int16_t *pMatrixNumPerCol;
    const int16_t *pAddr;

    uint8_t i_LS;       // i_Ls decides the base matrix entries
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

    LDPC_ADAPTER_P ldpc_adapter_func = ldpc_select_adapter_func(LDPC_config.Zc);
    LDPC_ENCODER ldpc_encoder_func = ldpc_select_encoder_func(LDPC_config.Bg);


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
    // ldpc_decoder_5gnr_response.varNodes = aligned_alloc()aligned_malloc<int16_t>(buffer_len, 32);
    // memset(ldpc_decoder_5gnr_response.varNodes, 0, numMsgBytes);

}


DoCoding::~DoCoding()
{

}


void DoCoding::Encode(int offset)
{
    int frame_id, symbol_id, cb_id;
    interpreteOffset3d(offset, &frame_id, &symbol_id, &cb_id);

    #if DEBUG_PRINT_IN_TASK
    printf("In doEncode thread %d: frame: %d, symbol: %d, code block %d\n", tid, frame_id, symbol_id, cb_id);
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
    printf("In doEncode thread %d: frame: %d, symbol: %d, code block %d\n", tid, frame_id, symbol_id, cb_id);
    #endif

    // inform main thread
    Event_data Decode_finish_event;
    Decode_finish_event.event_type = EVENT_DECODE;
    Decode_finish_event.data = offset;

    if ( !complete_task_queue_->enqueue(*task_ptok, Decode_finish_event ) ) {
        printf("Decode message enqueue failed\n");
        exit(0);
    }
}
