// XXX Some documentation about the file

#ifndef LDPC_WORKER
#define LDPC_WORKER

#include "config.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "rpc.h"
#include <cstdint>

void ldpc_req_handler(erpc::ReqHandle* req_handle, void* _context);

// XXX doc for the class & public members
class LDPCWorker {
public:
    LDPCWorker(Config* config, int tid, erpc::Nexus* nexus);
    ~LDPCWorker();

    void serve();
    bool decode(int8_t* in_buffer, uint8_t* out_buffer);
    size_t get_decoded_bits();

    friend void ldpc_req_handler(erpc::ReqHandle* req_handle, void* _context);

private:
    // XXX: Short doc if needed for private variables
    Config* cfg;
    int tid; // Thread ID as defined by eRPC
    int16_t* resp_var_nodes;
    erpc::Rpc<erpc::CTransport>* rpc;
    size_t decoded_bits;
};

#endif