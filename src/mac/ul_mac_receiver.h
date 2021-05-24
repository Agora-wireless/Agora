/**
 * @file ul_mac_receiver.h
 * @brief Declaration file for the simple uplink mac receiver class (interfaces
 * with the basestation)
 */
#ifndef UL_MAC_RECEIVER_H_
#define UL_MAC_RECEIVER_H_

#include <thread>

#include "config.h"

class UlMacReceiver {
 public:
  explicit UlMacReceiver(Config* const cfg, size_t rx_thread_num = 1,
                         size_t core_offset = 1);

  ~UlMacReceiver() = default;

  std::vector<std::thread> StartRecv();
  void* LoopRecv(size_t tid);

 private:
  size_t rx_thread_num_;
  size_t tx_thread_num_;

  size_t core_id_;
  Config* const cfg_;
};

#endif  // UL_MAC_RECEIVER_H_
