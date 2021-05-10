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

struct Package {
  uint32_t frame_id;
  uint32_t symbol_id;
  uint32_t cell_id;
  uint32_t ant_id;
  short data[];
  Package(int f, int s, int c, int a)
      : frame_id(f), symbol_id(s), cell_id(c), ant_id(a) {}
};

class RecorderWorker {
 public:
  RecorderWorker(Config* in_cfg, size_t antenna_offset, size_t num_antennas);
  ~RecorderWorker();

  void init();
  void finalize();
  herr_t record(int tid, Package* pkg);

  inline size_t num_antennas() { return num_antennas_; }
  inline size_t antenna_offset() { return antenna_offset_; }

 private:
  // pilot dataset size increment
  static const int kConfigPilotExtentStep;
  // data dataset size increment
  static const int kConfigDataExtentStep;

  void gc();
  herr_t initHDF5();
  void openHDF5();
  void closeHDF5();
  void finishHDF5();

  Config* cfg_;
  H5std_string hdf5_name_;

  H5::H5File* file_;
  // Group* group;
  H5::DSetCreatPropList pilot_prop_;
  H5::DSetCreatPropList noise_prop_;
  H5::DSetCreatPropList data_prop_;

  H5::DataSet* pilot_dataset_;
  H5::DataSet* noise_dataset_;
  H5::DataSet* data_dataset_;

  size_t frame_number_pilot_;
  size_t frame_number_noise_;
  size_t frame_number_data_;

  size_t max_frame_number_;

  size_t antenna_offset_;
  size_t num_antennas_;
};
}; /* End namespace Agora_recorder */

#endif /* AGORA_RECORDER_WORKER_H_ */
