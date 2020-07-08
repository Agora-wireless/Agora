/**
 * This file declares the LDPCWorker class
 * and the ldpc request handler function
 */

#ifndef LDPC_WORKER
#define LDPC_WORKER

#include "config.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "rpc.h"
#include <cstdint>

// TODO: Add documentation
void ldpc_req_handler(erpc::ReqHandle* req_handle, void* _context);

/**
 * @brief This class is used to process LDPC decoding tasks 
 * from remote Millipede servers
 */
class LDPCWorker {
public:
    /**
     * @brief Create an LDPCWorker class and initialize the eRPC object
     * 
     * @param config The config file used to configure LDPC worker
     * @param tid The eRPC thread ID to help identify the eRPC object
     * @param nexus The nexus object used in eRPC
     */
    LDPCWorker(Config* config, int tid, erpc::Nexus* nexus);
    ~LDPCWorker();

    /// Continuosly poll for incoming RPC requests and process them
    void serve(); // TODO: Better name run_erpc_event_loop_forever?

    /// LDPC-decode data from in_buffer and place it in out_buffer
    void decode(int8_t* in_buffer, uint8_t* out_buffer);

    /// Return the number of decoded bits for each decoding round
    size_t get_decoded_bits();

    friend void ldpc_req_handler(erpc::ReqHandle* req_handle, void* _context);

private:
    const int tid; // Thread ID as defined by eRPC
    const size_t decoded_bits; // TODO: DOC
    Config* cfg; // TODO: Do we just need cfg->LDPC_config?
    int16_t* resp_var_nodes; // TODO: DOC
    erpc::Rpc<erpc::CTransport>* rpc;
};

#endif