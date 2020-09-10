/**
 * This file declares the RemoteLDPC class
 * and the ldpc request handler function
 */

#ifndef REMOTE_LDPC
#define REMOTE_LDPC

#include "config.hpp"
#include "encoder.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "udp_client.h"
#include "udp_server.h"
#include "docoding.hpp"
#include <cstdint>

/**
 * @brief This class is used to process LDPC decoding tasks 
 * from remote Agora servers. 
 * 
 * There is one instance of this class per remote worker thread,
 * .
 */
class RemoteLDPC {
public:
    /**
     * @brief Initialize a RemoteLDPC worker and spawn a thread for it.
     * 
     * @param cfg The main Agora-wide configuration info.
     * @param tid The thread ID on which this LDPC worker runs. 
     *            This will also determine which UDP port to listen on.
     * @param rx_buf_size The size of this worker's rx socket buffer, in bytes.
     * 
     */
    RemoteLDPC(Config* cfg, int tid, size_t rx_buf_size);
    ~RemoteLDPC();

    /// LDPC-decode data from in_buffer and place it in out_buffer
    void decode(int8_t* in_buffer, uint8_t* out_buffer);

    void worker_loop(size_t tid);

    /// Join this LDPC worker's thread, i.e. wait for it to complete.
    void join()
    {
        if (worker_thread.joinable()) {
            worker_thread.join();
        }
    }

    friend void ldpc_req_handler_test();

private:
    /// main Agora-wide configuration info.
    Config* cfg;
    /// Configurations for RemoteLDPC
    LDPCconfig LDPC_config;
    /// The ID of the thread on which this `RemoteLDPC` instance runs on
    const int tid;
    /// The worker thread
    std::thread worker_thread;
    /// Number of decoded bits for each decoding round
    const size_t decoded_bits;
    /// The UDP socket on which we receive LDPC requests
    UDPServer udp_server;
    /// The UDP socket on which we send back LDPC responses.
    UDPClient udp_client;

    // The buffer used to store the code word 16-bit LLR outputs for FlexRAN lib
    int16_t* resp_var_nodes;

    size_t num_reqs_recvd = 0;
};

#endif