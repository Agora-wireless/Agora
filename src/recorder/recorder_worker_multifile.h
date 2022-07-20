/**
 * @file recorder_worker_multifile.h
 * @brief Recorder worker to write to multiple bin files per rx symbol
 */
#ifndef AGORA_RECORDER_WORKER_MULTIFILE_H_
#define AGORA_RECORDER_WORKER_MULTIFILE_H_

#include "recorder_worker.h"

namespace Agora_recorder {

class RecorderWorkerMultiFile : public RecorderWorker {
 public:
  explicit RecorderWorkerMultiFile(const Config* in_cfg, size_t antenna_offset,
                                   size_t num_antennas);
  virtual ~RecorderWorkerMultiFile();

  void Init() final;
  void Finalize() final;
  int Record(const Packet* pkg) final;

  inline size_t NumAntennas() const final { return num_antennas_; }
  inline size_t AntennaOffset() const final { return antenna_offset_; }

 private:
  void Open();
  void Close();
  void Gc();

  const Config* cfg_;

  size_t antenna_offset_;
  size_t num_antennas_;
};
}; /* End namespace Agora_recorder */

#endif /* AGORA_RECORDER_WORKER_MULTIFILE_H_ */
