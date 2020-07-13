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

// The eRPC request handler function to process remote LDPC decode tasks
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
     * @param LDPC_config The configurations for the LDPC worker
     * @param tid The eRPC thread ID to help identify the eRPC object
     * @param nexus The nexus object used in eRPC
     */
    LDPCWorker(LDPCconfig LDPC_config, int tid, erpc::Nexus* nexus);
    ~LDPCWorker();

    /// Continuosly poll for incoming RPC requests and process them
    void run_erpc_event_loop_forever();

    /// LDPC-decode data from in_buffer and place it in out_buffer
    void decode(int8_t* in_buffer, uint8_t* out_buffer);

    void stop_loop() { stop = true; }

    friend void ldpc_req_handler(erpc::ReqHandle* req_handle, void* _context);

    friend void ldpc_req_handler_test(
        erpc::ReqHandle* req_handle, void* _context);

private:
    const int tid; // Thread ID as defined by eRPC
    const size_t decoded_bits; // Number of decoded bits for each decoding round
    LDPCconfig LDPC_config; // Configurations for LDPC worker
    erpc::Rpc<erpc::CTransport>* rpc;

    // The buffer used to store the code word 16-bit LLR outputs for FlexRAN lib
    int16_t* resp_var_nodes;

    bool stop;
};

#endif