/**
 * @file recorder_worker.h
 * @brief Recorder worker interface definition
 */
#ifndef AGORA_RECORDER_WORKER_H_
#define AGORA_RECORDER_WORKER_H_

#include <memory>

#include "config.h"
#include "message.h"
#include "symbols.h"

namespace Agora_recorder {

static const std::string kOutputFilePath = "files/experiment/";

class RecorderWorker {
 public:
  enum RecorderWorkerTypes { kRecorderWorkerMultiFile, kRecorderWorkerHdf5 };
  RecorderWorker([[maybe_unused]] const Config* in_cfg,
                 [[maybe_unused]] size_t antenna_offset,
                 [[maybe_unused]] size_t num_antennas,
                 [[maybe_unused]] size_t record_interval,
                 [[maybe_unused]] Direction rx_direction) {}
  virtual ~RecorderWorker() = default;

  virtual void Init() = 0;
  virtual int Record(const Packet* pkg) = 0;
  virtual void Finalize() = 0;

  virtual size_t NumAntennas() const { return 0; }
  virtual size_t AntennaOffset() const { return 0; }

  ///Factory function to make concrete worker
  static std::unique_ptr<RecorderWorker> Create(
      RecorderWorkerTypes type, const Config* in_cfg, size_t antenna_offset,
      size_t num_antennas, size_t record_interval, Direction rx_direction);
};
}; /* End namespace Agora_recorder */

#endif /* AGORA_RECORDER_WORKER_H_ */
