/**
 * @file manager.h
 * @brief Declaration file for the main manager class
 */

#ifndef MANAGER_H_
#define MANAGER_H_

#include <vector>

#include "agora_helper.h"
#include "buffer.h"
#include "concurrent_queue_wrapper.h"
#include "concurrentqueue.h"
#include "config.h"
#include "mat_logger.h"
#include "phy_stats.h"
#include "signal_handler.h"
#include "stats.h"
#include "symbols.h"
#include "utils.h"

class Manager {
 public:
  explicit Manager(Config*, Stats*, PhyStats*, MessageInfo*, Buffer*,
                   FrameInfo*);
  ~Manager();
  void Start();

 private:
  size_t FetchEvent(EventData events_list[], bool is_turn_to_dequeue_from_io);
  void HandleEventPacketRx(size_t tag);
  void HandleEventFft(size_t tag);
  void HandleEventZf(size_t tag);
  void HandleEventDemul(size_t tag);
  void HandleEventDecode(size_t tag);
  void HandleEventRanUpdate(size_t tag1, size_t tag2, size_t tag3);
  void HandleEventPacketToMac(size_t tag);
  void HandleEventPacketFromMac(size_t tag);
  void HandleEventEncode(size_t tag);
  void HandleEventPrecode(size_t tag);
  void HandleEventIfft(size_t tag);
  void HandleEventPacketTx(size_t tag);

  Config* const config_;
  Stats* stats_;
  PhyStats* phy_stats_;
  std::vector<std::thread> workers_;
  size_t base_worker_core_offset_;

  MessageInfo* queues_;
  SchedInfo* sched_info_arr_[kScheduleQueues][kNumEventTypes];
  Buffer* buffers_;
  Counter* counters_;
  FrameInfo* frame_info_;

  uint8_t schedule_process_flags_;
  std::array<std::queue<fft_req_tag_t>, kFrameWnd> fft_queue_arr_;
};

#endif  // MANAGER_H_
