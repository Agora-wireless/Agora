/*
 Copyright (c) 2018-2020
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
Event based message queue thread class for the recorder worker
---------------------------------------------------------------------
*/
#ifndef AGORA_RECORDER_THREAD_H_
#define AGORA_RECORDER_THREAD_H_

#include <condition_variable>
#include <mutex>
#include <vector>

#include "concurrentqueue.h"
#include "recorder_worker.h"

namespace Agora_recorder {

class RecorderThread {
 public:
  RecorderThread(const Config *in_cfg, size_t thread_id, int core,
                 size_t queue_size, size_t antenna_offset, size_t num_antennas,
                 size_t interval, Direction rx_direction,
                 const std::vector<RecorderWorker::RecorderWorkerTypes> &types,
                 bool wait_signal = true);
  ~RecorderThread();

  void Start();
  void Stop();
  bool DispatchWork(const EventData &event);

 private:
  /*Main threading loop */
  void DoRecording();
  void HandleEvent(const EventData &event);
  void Finalize();

  // 1 - Producer (dispatcher), 1 - Consumer
  moodycamel::ConcurrentQueue<EventData> event_queue_;
  moodycamel::ProducerToken producer_token_;
  std::vector<std::unique_ptr<RecorderWorker>> workers_;
  std::thread thread_;

  size_t id_;

  /* >= 0 to assign a core to the thread
   * <0   to disable thread core assignment */
  int core_alloc_;

  /* Synchronization for startup and sleeping */
  /* Setting wait signal to false will disable the thread waiting on new message
   * may cause excessive CPU load for infrequent messages.
   * However, when the message processing time ~= queue posting time the mutex
   * could become unnecessary work
   */
  bool wait_signal_;
  std::mutex sync_;
  std::condition_variable condition_;
  bool running_;
};
};  // namespace Agora_recorder

#endif /* AGORA_RECORDER_THREAD_H_ */
