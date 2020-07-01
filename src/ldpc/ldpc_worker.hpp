// XXX Some documentation about the file

#ifndef LDPC_WORKER
#define LDPC_WORKER

#include "config.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "rpc_context.hpp"
#include <cstdint>

// XXX: Need a comment for global variables
// static constexpr kLPDCMaxBufferSize = 65536;
#define LDPC_MAX_BUFFER_SIZE (65536)

void ldpc_req_handler(erpc::ReqHandle* req_handle, void* _context);

// XXX doc for the class & public members
class LDPCWorker {
public:
    LDPCWorker(Config* config, int tid, erpc::Nexus* nexus);
    ~LDPCWorker();

    void serve();
    bool decode(int8_t* in_buffer, uint8_t* out_buffer);
    size_t get_decoded_bits();

private:
    // XXX: Short doc if needed for private variables
    Config* cfg;
    int tid; // Thread ID as defined by eRPC
    int16_t* resp_var_nodes;
    RPCContext* ctx;
    size_t decoded_bits;
};

#endif