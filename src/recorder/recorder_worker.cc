/**
 * @file recorder_worker.h
 * @brief Recorder worker interface factory
 */
#include "recorder_worker.h"

#if defined(ENABLE_HDF5)
#include "recorder_worker_hdf5.h"
#endif
#include "recorder_worker_multifile.h"

namespace Agora_recorder {

std::unique_ptr<RecorderWorker> RecorderWorker::Create(
    RecorderWorker::RecorderWorkerTypes type, const Config* in_cfg,
    size_t antenna_offset, size_t num_antennas, size_t record_interval,
    Direction rx_direction) {
  switch (type) {
    case RecorderWorker::RecorderWorkerTypes::kRecorderWorkerMultiFile: {
      return std::make_unique<RecorderWorkerMultiFile>(
          in_cfg, antenna_offset, num_antennas, record_interval, rx_direction);
    }
#if defined(ENABLE_HDF5)
    case RecorderWorker::RecorderWorkerTypes::kRecorderWorkerHdf5: {
      return std::make_unique<RecorderWorkerHDF5>(
          in_cfg, antenna_offset, num_antennas, record_interval, rx_direction);
    }
#endif
    default: {
      throw std::runtime_error("RecorderWorker::Create() type not supported");
    }
  }
}

}; /* End namespace Agora_recorder */