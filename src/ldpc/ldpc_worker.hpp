#ifndef LDPC_WORKER
#define LDPC_WORKER

#include "rpc_context.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "config.hpp"
#include <cstdint>

#define LDPC_MAX_BUFFER_SIZE (65536)

void ldpc_req_handler(erpc::ReqHandle *req_handle, void * _context);

class LDPCWorker {
public:
    LDPCWorker(Config *config, int tid, erpc::Nexus *nexus);
    ~LDPCWorker();

    void serve();
    bool decode(int8_t *in_buffer, uint8_t *out_buffer);
    size_t get_decoded_bits();

private:
    Config *cfg;
    int tid;
    int16_t *resp_var_nodes;
    RPCContext *ctx;
    size_t decoded_bits;
};

#endif