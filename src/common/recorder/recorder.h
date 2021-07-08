/**
 * @file  recorder.h
 * @brief Record received frames from massive-mimo base station in HDF5 format
 */
#ifndef AGORA_RECORDER_H_
#define AGORA_RECORDER_H_

#include <vector>

#include "concurrentqueue.h"
#include "config.h"
#include "recorder_thread.h"

namespace Recorder {
class Recorder {
  public:
  Recorder() = delete;
  ~Recorder();

  inline static Recorder &GetInstance(Config *cfg = nullptr, unsigned int core_start = 0) {
    static Recorder instance(cfg, core_start);
    return instance;
  }

  inline static void Record() {
    GetInstance().RecordInternal();
  }

  inline static void DoIt(std::vector<RecorderWorkerFactory *> &);

  inline static const std::string &GetTraceFileName() {
    return GetInstance().cfg_->TraceFile();
  }

  // buffer length of each rx thread
  static const int kSampleBufferFrameNum;
  // dequeue bulk size, used to reduce the overhead of dequeue in main thread
  static const int KDequeueBulkSize;

  private:
  explicit Recorder(Config *in_cfg, unsigned int core_start);

  // Internal non-static impl
  void RecordInternal();
  void DoItInternal(std::vector<RecorderWorkerFactory *> &);

  // Garbage Collect
  void Gc();

  // Manage HDF5 File
  herr_t InitHDF5(H5std_string);

  // Constructor Args
  Config *cfg_;
  unsigned int core_start_;

  size_t rx_thread_buff_size_;

  std::vector<std::unique_ptr<RecorderThread>> recorders_;

  /*
    TODO: Check analog of MAX_FRAME_INC
  size_t max_frame_number_;
  inline static size_t GetRecordedFrameNum() {
    return GetInstance().max_frame_number_;
  }
  */

  moodycamel::ConcurrentQueue<EventData> message_queue_;

  /* Core assignment start variables */
  const unsigned int kMainDispatchCore;
  const unsigned int kRecorderCore;
  const unsigned int kRecvCore;

  size_t num_writter_threads_;

  H5::H5File *h5_file_;
};

};     // namespace Recorder
#endif /* AGORA_RECORDER_H_ */
