/**
 * @file mac_receiver.h
 * @brief Declaration file for the simple mac receiver class
 */
#ifndef MAC_RECEIVER_H_
#define MAC_RECEIVER_H_

#include <thread>

#include "config.h"

class MacReceiver {
 public:
  explicit MacReceiver(Config* const cfg, size_t num_frame_data_bytes,
                       std::string server_address, size_t tx_port,
                       size_t rx_thread_num = 1, size_t core_offset = 1);

  ~MacReceiver() = default;

  std::vector<std::thread> StartRecv();
  void* LoopRecv(size_t tid);

 private:
  const size_t data_bytes_;
  const std::string server_address_;
  const size_t server_tx_port_;

  size_t rx_thread_num_;
  size_t core_id_;
  Config* const cfg_;
};

#endif  // MAC_RECEIVER_H_
