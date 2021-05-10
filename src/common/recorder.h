/**
 * @file  recorder.h
 * @brief Record received frames from massive-mimo base station in HDF5 format
 */
#ifndef AGORA_RECORDER_H_
#define AGORA_RECORDER_H_

#include "concurrentqueue.h"
#include "config.h"
#include "recorder_thread.h"

namespace Agora_recorder {
class Recorder {
 public:
  explicit Recorder(Config* in_cfg, unsigned int core_start = 0u);
  ~Recorder();

  void DoIt();
  inline moodycamel::ConcurrentQueue<EventData>& GetRecorderQueue() {
    return message_queue_;
  }

  size_t GetRecordedFrameNum();
  // std::string GetTraceFileName() { return this->cfg_->trace_file(); }

 private:
  void Gc();

  // buffer length of each rx thread
  static const int kSampleBufferFrameNum;
  // dequeue bulk size, used to reduce the overhead of dequeue in main thread
  static const int KDequeueBulkSize;

  Config* cfg_;
  SampleBuffer* rx_buffer_;
  size_t rx_thread_buff_size_;

  std::vector<Agora_recorder::RecorderThread*> recorders_;
  size_t max_frame_number_;

  moodycamel::ConcurrentQueue<EventData> message_queue_;

  /* Core assignment start variables */
  const unsigned int kMainDispatchCore;
  const unsigned int kRecorderCore;
  const unsigned int kRecvCore;
};     /* class Recorder */
};     // namespace Agora_recorder
#endif /* AGORA_RECORDER_H_ */
