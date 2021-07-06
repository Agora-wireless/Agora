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

namespace Agora_recorder {
class Recorder {
  public:
  Recorder() = delete;
  ~Recorder();

  inline static Recorder &GetInstance(Config *cfg = nullptr, unsigned int core_start = 0) {
    static Recorder instance(cfg, core_start);
    return instance;
  }

  inline static void Record() {
    GetInstance().Record_();
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
  void Record_();
  void DoIt_(std::vector<RecorderWorkerFactory *> &);

  // Garbage Collect
  void Gc();

  // Manage HDF5 File
  herr_t InitHDF5();
  void OpenHDF5();
  void CloseHDF5();
  void FinishHDF5();

  // Constructor Args
  Config *cfg_;
  unsigned int core_start_;

  size_t rx_thread_buff_size_;

  std::vector<Agora_recorder::RecorderThread *> recorders_;

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

  H5std_string file_name_;
  H5::H5File *file_;
};

};     // namespace Agora_recorder
#endif /* AGORA_RECORDER_H_ */
