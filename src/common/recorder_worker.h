/*
 Copyright (c) 2018-2020
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Class to handle writting data to an hdf5 file
---------------------------------------------------------------------
*/
#ifndef AGORA_RECORDER_WORKER_H_
#define AGORA_RECORDER_WORKER_H_

#include "H5Cpp.h"
#include "config.h"

namespace Agora_recorder {

// each thread has a SampleBuffer
struct SampleBuffer {
  std::vector<char> buffer;
  std::atomic_int* pkg_buf_inuse;
};

/*
  An Interface on how to implement concrete recorders
*/
template<class T>
class RecorderWorker {
  public:
  RecorderWorker(Config* in_cfg, H5::H5File *h5_file)
  : file_(h5_file),
    cfg_(in_cfg) { }

  ~RecorderWorker() = default;

  virtual herr_t Record(int, T *) = 0;

  private:
  H5::H5File *file_;

  Config* cfg_;
};

/*
  Concrete recorder implementation
*/
class RxPacketRecorder : public RecorderWorker<Packet> {
  public:
  RxPacketRecorder(Config *in_cfg, H5::H5File *h5_file):
    RecorderWorker(in_cfg, h5_file) {}

  ~RxPacketRecorder() = default;

  herr_t Record(int thread_id, Packet *data) override {
    throw std::runtime_error("Unimplemented!");
  }
};

}; /* End namespace Agora_recorder */

#endif /* AGORA_RECORDER_WORKER_H_ */
