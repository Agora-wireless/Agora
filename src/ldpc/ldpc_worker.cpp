#include "ldpc_worker.hpp"
#include "utils_ldpc.hpp"
#include <malloc.h>
#include <string>

void basic_sm_handler(int session_num, erpc::SmEventType sm_event_type,
    erpc::SmErrType sm_err_type, void* _context)
{
    printf("Connected session: %d\n", session_num);
}

LDPCWorker::LDPCWorker(LDPCconfig LDPC_config, int tid, erpc::Nexus* nexus)
    : LDPC_config(LDPC_config)
    , tid(tid)
    , decoded_bits(ldpc_encoding_input_buf_size(LDPC_config.Bg, LDPC_config.Zc))
{
    resp_var_nodes = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));
    rpc = new erpc::Rpc<erpc::CTransport>(
        nexus, static_cast<void*>(this), tid, basic_sm_handler);
}

LDPCWorker::~LDPCWorker() { free(resp_var_nodes); }

void ldpc_req_handler(erpc::ReqHandle* req_handle, void* _context)
{
    auto* worker = static_cast<LDPCWorker*>(_context);
    auto* in_buf = reinterpret_cast<int8_t*>(req_handle->get_req_msgbuf()->buf);

    req_handle->dyn_resp_msgbuf
        = worker->rpc->alloc_msg_buffer(worker->decoded_bits);
    auto* out_buf = reinterpret_cast<uint8_t*>(req_handle->dyn_resp_msgbuf.buf);
    worker->decode(in_buf, out_buf);

    worker->rpc->enqueue_response(req_handle, &req_handle->dyn_resp_msgbuf);
}

void LDPCWorker::run_erpc_event_loop_forever()
{
    while (true) {
        rpc->run_event_loop_once();
    }
}

void LDPCWorker::decode(int8_t* in_buffer, uint8_t* out_buffer)
{
    struct bblib_ldpc_decoder_5gnr_request ldpc_request {
    };
    struct bblib_ldpc_decoder_5gnr_response ldpc_response {
    };

    // Decoder setup
    int16_t numFillerBits = 0;
    int16_t numChannelLlrs = LDPC_config.cbCodewLen;

    ldpc_request.numChannelLlrs = numChannelLlrs;
    ldpc_request.numFillerBits = numFillerBits;
    ldpc_request.maxIterations = LDPC_config.decoderIter;
    ldpc_request.enableEarlyTermination = LDPC_config.earlyTermination;
    ldpc_request.Zc = LDPC_config.Zc;
    ldpc_request.baseGraph = LDPC_config.Bg;
    ldpc_request.nRows = LDPC_config.nRows;

    int numMsgBits = LDPC_config.cbLen - numFillerBits;
    ldpc_response.numMsgBits = numMsgBits;
    ldpc_response.varNodes = resp_var_nodes;

    ldpc_request.varNodes = in_buffer;
    ldpc_response.compactedMessageBytes = out_buffer;

    bblib_ldpc_decoder_5gnr(&ldpc_request, &ldpc_response);
}