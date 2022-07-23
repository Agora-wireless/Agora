/**
 * @file recorder_worker_multifile.h
 * @brief Recorder worker to write to multiple bin files per rx symbol
 */
#ifndef AGORA_RECORDER_WORKER_HDF5_H_
#define AGORA_RECORDER_WORKER_HDF5_H_

#include <memory>
#include <string>

#include "hdf5_lib.h"
#include "recorder_worker.h"

namespace Agora_recorder {

class RecorderWorkerHDF5 : public RecorderWorker {
 public:
  explicit RecorderWorkerHDF5(const Config* in_cfg, size_t antenna_offset,
                              size_t num_antennas, size_t record_interval);
  ~RecorderWorkerHDF5() override;

  void Init() final;
  void Finalize() final;
  int Record(const Packet* pkt) final;

  inline size_t NumAntennas() const final { return num_antennas_; }
  inline size_t AntennaOffset() const final { return antenna_offset_; }

 private:
  void Open();
  void Close();

  const Config* cfg_;

  size_t antenna_offset_;
  size_t num_antennas_;
  size_t interval_;

  std::unique_ptr<Hdf5Lib> hdf5_;
  size_t max_frame_number_;
  std::vector<std::pair<std::string, std::array<hsize_t, kDsDimsNum>>>
      datasets_;
  const std::array<hsize_t, kDsDimsNum> data_chunk_dims_;
};
}; /* End namespace Agora_recorder */

#endif /* AGORA_RECORDER_WORKER_HDF5_H_ */
