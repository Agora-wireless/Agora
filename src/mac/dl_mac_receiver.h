/**
 * @file dl_mac_receiver.h
 * @brief Declaration file for the simple downlink mac receiver class
 * (interfaces with the client)
 */
#ifndef DL_MAC_RECEIVER_H_
#define DL_MAC_RECEIVER_H_

#include <thread>

#include "config.h"

class DlMacReceiver {
 public:
  explicit DlMacReceiver(Config* const cfg, size_t rx_thread_num = 1,
                         size_t core_offset = 1);

  ~DlMacReceiver() = default;

  std::vector<std::thread> StartRecv();
  void* LoopRecv(size_t tid);

 private:
  size_t rx_thread_num_;
  size_t tx_thread_num_;

  size_t core_id_;
  Config* const cfg_;
};

#endif  // DL_MAC_RECEIVER_H_
