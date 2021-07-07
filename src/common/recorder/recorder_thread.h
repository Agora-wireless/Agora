/*
 Copyright (c) 2018-2020
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
Event based message queue thread class for the recorder worker
---------------------------------------------------------------------
*/
#ifndef AGORA_RECORDER_THREAD_H_
#define AGORA_RECORDER_THREAD_H_

#include <vector>
#include <unordered_map>
#include <condition_variable>
#include <mutex>

#include "concurrentqueue.h"
#include "recorder_worker.h"

namespace Recorder {
class RecorderThread {
 public:
  RecorderThread(Config *in_cfg, std::vector<RecorderWorkerFactory *> &factories,
                  std::string file_name, size_t thread_id, int core,
                  size_t queue_size, size_t antenna_offset, size_t num_antennas,
                  bool wait_signal = true);
  ~RecorderThread();

  void Start();
  void Stop();
  bool DispatchWork(const EventData &event);

  inline size_t GetNumAntennas() { return num_antennas_; }
  inline size_t GetAntennaOffset() { return antenna_offset_; }

 private:
  /*Main threading loop */
  void DoRecording();
  void HandleEvent(const EventData &event);
  void Finalize();

  // 1 - Producer (dispatcher), 1 - Consumer
  moodycamel::ConcurrentQueue<EventData> event_queue_;
  moodycamel::ProducerToken producer_token_;
  std::thread thread_;

  size_t id_;
  size_t packet_length_;

  /* >= 0 to assign a core to the thread
   * <0   to disable thread core assignment */
  int core_alloc_;

  std::mutex sync_;
  std::condition_variable condition_;

  /* Synchronization for startup and sleeping */
  /* Setting wait signal to false will disable the thread waiting on new message
   * may cause excessive CPU load for infrequent messages.
   * However, when the message processing time ~= queue posting time the mutex
   * could become unnecessary work
   */
  volatile bool wait_signal_; // Could be operated in multi-core environment
  volatile bool running_; // The recorder thread might not be pinned to a single core

  size_t antenna_offset_;
  size_t num_antennas_;

  /* Mapping of concrete recorders to events */
  std::unordered_map<EventType, RecorderWorker *> worker_mapping_;
};
};  // namespace Recorder

#endif /* AGORA_RECORDER_THREAD_H_ */
