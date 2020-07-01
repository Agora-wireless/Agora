#include "ldpc_worker.hpp"
#include "utils_ldpc.hpp"
#include <malloc.h>
#include <string>

void ldpc_req_handler(erpc::ReqHandle *req_handle, void * _context) {
    auto *context = static_cast<RPCContext *>(_context);
    auto *worker = static_cast<LDPCWorker *>(context->get_info());
    auto *in_buf = reinterpret_cast<int8_t *>(req_handle->get_req_msgbuf()->buf);

    req_handle->dyn_resp_msgbuf = context->alloc_msg_buffer(worker->get_decoded_bits());
    auto *out_buf = reinterpret_cast<uint8_t *>(req_handle->dyn_resp_msgbuf.buf);

    worker->decode(in_buf, out_buf);

    context->respond_without_copy(req_handle, &req_handle->dyn_resp_msgbuf);
}

LDPCWorker::LDPCWorker(Config *config, int _tid, erpc::Nexus *nexus) {
    cfg = config;
    tid = _tid;
    resp_var_nodes = (int16_t*)memalign(64, 1024 * 1024 * sizeof(int16_t));

    ctx = new RPCContext(nexus, tid, static_cast<void *>(this));

    decoded_bits = ldpc_encoding_input_buf_size(config->LDPC_config.Bg, config->LDPC_config.Zc);
}

LDPCWorker::~LDPCWorker() {
    free(resp_var_nodes);
}

size_t LDPCWorker::get_decoded_bits() {
    return decoded_bits;
}

void LDPCWorker::serve() {
    while (true) {
        ctx->poll_event();
    }
}

bool LDPCWorker::decode(int8_t *in_buffer, uint8_t *out_buffer) {
    LDPCconfig LDPC_config = cfg->LDPC_config;

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

    ldpc_decoder_5gnr_request.varNodes = in_buffer;
    ldpc_decoder_5gnr_response.compactedMessageBytes = out_buffer;

    bblib_ldpc_decoder_5gnr(
        &ldpc_decoder_5gnr_request, &ldpc_decoder_5gnr_response);

    return true;
}