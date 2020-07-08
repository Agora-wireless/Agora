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
     * @param tid The thread ID to help identify the eRPC object
     * @param nexus The nexus object used in eRPC object
     */
    LDPCWorker(Config* config, int tid, erpc::Nexus* nexus);
    ~LDPCWorker();

    /**
     * @brief This function is used to poll for remote requests 
     * and process them
     */
    void serve();

    /**
     * @brief This function is used to decode the data
     * 
     * @param in_buffer The buffer storing the data to be decoded
     * @param out_buffer The buffer to store the data decoded
     */
    void decode(int8_t* in_buffer, uint8_t* out_buffer);

    /**
     * @brief This function is used to get the number of decoded bits 
     * for each decoding round
     */
    size_t get_decoded_bits();

    friend void ldpc_req_handler(erpc::ReqHandle* req_handle, void* _context);

private:
    Config* cfg;
    int tid; // Thread ID as defined by eRPC
    int16_t* resp_var_nodes;
    erpc::Rpc<erpc::CTransport>* rpc;
    size_t decoded_bits;
};

#endif