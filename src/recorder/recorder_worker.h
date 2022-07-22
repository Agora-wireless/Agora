/**
 * @file recorder_worker.h
 * @brief Recorder worker interface definition
 */
#ifndef AGORA_RECORDER_WORKER_H_
#define AGORA_RECORDER_WORKER_H_

#include "buffer.h"
#include "config.h"

namespace Agora_recorder {

class RecorderWorker {
 public:
  RecorderWorker([[maybe_unused]] const Config* in_cfg,
                 [[maybe_unused]] size_t antenna_offset,
                 [[maybe_unused]] size_t num_antennas,
                 [[maybe_unused]] size_t record_interval){};
  virtual ~RecorderWorker() = default;

  virtual void Init() = 0;
  virtual int Record(const Packet* pkg) = 0;
  virtual void Finalize() = 0;

  virtual size_t NumAntennas() const { return 0; }
  virtual size_t AntennaOffset() const { return 0; }
};
}; /* End namespace Agora_recorder */

#endif /* AGORA_RECORDER_WORKER_H_ */
