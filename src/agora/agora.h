/**
 * @file agora.h
 * @brief Declaration file for the main agora class
 */

#ifndef AGORA_H_
#define AGORA_H_

#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <queue>
#include <system_error>
#include <vector>

#include "buffer.h"
#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "dodecode.h"
#include "dodemul.h"
#include "doencode.h"
#include "dofft.h"
#include "doifft.h"
#include "doprecode.h"
#include "dozf.h"
#include "mac_thread_basestation.h"
#include "mat_logger.h"
#include "memory_manage.h"
#include "packet_txrx.h"
#include "phy_stats.h"
#include "signal_handler.h"
#include "stats.h"
#include "utils.h"

class Agora {
 public:
  explicit Agora(
      Config* /*cfg*/);  /// Create an Agora object and start the worker threads
  ~Agora();

  void Start();  /// The main Agora event loop
  void Stop();
  void GetEqualData(float** ptr, int* size);

  // Flags that allow developer control over Agora internals
  struct {
    //     void getEqualData(float** ptr, int* size);Before exiting, save
    //     LDPC-decoded or demodulated data to a file
    bool enable_save_decode_data_to_file_ = false;

    // Before exiting, save data sent on downlink to a file
    bool enable_save_tx_data_to_file_ = false;
  } flags_;

 private:
  void WorkerFft(int tid);
  void WorkerZf(int tid);
  void WorkerDemul(int tid);
  void WorkerDecode(int tid);
  void Worker(int tid);
  void CreateThreads();

  std::vector<std::thread> workers_;
};

#endif  // AGORA_H_
