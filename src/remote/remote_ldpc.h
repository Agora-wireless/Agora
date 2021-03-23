/**
 * @file remote_ldpc.h
 * @brief This file declares the RemoteLDPC class and the ldpc request handler
 * function
 */
#ifndef REMOTE_LDPC_H_
#define REMOTE_LDPC_H_

#include <cstdint>

#include "config.h"
#include "docoding.h"
#include "encoder.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "udp_client.h"
#include "udp_server.h"

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
  RemoteLDPC(Config* cfg, size_t tid, size_t rx_buf_size);
  ~RemoteLDPC();

  /// LDPC-decode data from in_buffer and place it in out_buffer
  void Decode(int8_t* in_buffer, uint8_t* out_buffer);

  void WorkerLoop(size_t tid);

  /// Join this LDPC worker's thread, i.e. wait for it to complete.
  void Join() {
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

  friend void LdpcReqHandlerTest();

 private:
  /// main Agora-wide configuration info.
  Config* cfg_;
  /// Configurations for RemoteLDPC
  LDPCconfig ldpc_config_;
  /// The ID of the thread on which this `RemoteLDPC` instance runs on
  const size_t tid_;
  /// The worker thread
  std::thread worker_thread_;
  /// Number of decoded bits for each decoding round
  const size_t decoded_bits_;
  /// The UDP socket on which we receive LDPC requests
  UDPServer udp_server_;
  /// The UDP socket on which we send back LDPC responses.
  UDPClient udp_client_;

  // The buffer used to store the code word 16-bit LLR outputs for FlexRAN lib
  int16_t* resp_var_nodes_;

  size_t num_reqs_recvd_ = 0;
};

#endif  // REMOTE_LDPC_H_