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
                       std::string phy_server_address, size_t phy_port,
                       size_t rx_thread_num = 1, size_t core_offset = 1);

  explicit MacReceiver(Config* const cfg, size_t num_frame_data_bytes,
                       std::string phy_server_address, size_t phy_port,
                       std::string fwd_data_udp_address, size_t fwd_port,
                       size_t rx_thread_num = 1, size_t core_offset = 1);

  ~MacReceiver() = default;

  std::vector<std::thread> StartRecv();
  void* LoopRecv(size_t tid);

 private:
  const size_t data_bytes_;
  const std::string phy_address_;
  const size_t phy_port_;

  const bool enable_udp_output_;
  const size_t udp_dest_port_;
  const std::string udp_dest_address_;

  const size_t rx_thread_num_;
  const size_t core_id_;
  Config* const cfg_;
};

#endif  // MAC_RECEIVER_H_
