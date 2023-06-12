/**
 * @file  recorder.h
 * @brief Record received frames from massive-mimo base station in HDF5 format
 */
#ifndef AGORA_RECORDER_H_
#define AGORA_RECORDER_H_

#include <memory>

#include "concurrentqueue.h"
#include "config.h"
#include "recorder_thread.h"

namespace Agora_recorder {
class Recorder {
 public:
  explicit Recorder(const Config* in_cfg, size_t core_start = 0u);
  ~Recorder();

  void DoIt();
  inline moodycamel::ConcurrentQueue<EventData>& GetRecorderQueue() {
    return message_queue_;
  }

  //const std::string& GetTraceFileName() { return this->cfg_->TraceFile(); }

 private:
  void Gc();

  // buffer length of each rx thread
  static const int kSampleBufferFrameNum;
  // dequeue bulk size, used to reduce the overhead of dequeue in main thread
  static const int kDequeueBulkSize;

  const Config* cfg_;
  size_t rx_thread_buff_size_;

  std::vector<std::unique_ptr<RecorderThread>> recorders_;
  moodycamel::ConcurrentQueue<EventData> message_queue_;

  /* Core assignment start variables */
  const size_t main_dispatch_core_;
  const size_t recorder_core_;

  size_t num_writter_threads_;
};     /* class Recorder */
};     // namespace Agora_recorder
#endif /* AGORA_RECORDER_H_ */
