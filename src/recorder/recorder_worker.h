/*
 Copyright (c) 2018-2020
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

----------------------------------------------------------------------
 Class to handle writting data to an hdf5 file
---------------------------------------------------------------------
*/
#ifndef AGORA_RECORDER_WORKER_H_
#define AGORA_RECORDER_WORKER_H_

#include "config.h"

namespace Agora_recorder {

class RecorderWorker {
 public:
  RecorderWorker(const Config* in_cfg, size_t antenna_offset,
                 size_t num_antennas);
  ~RecorderWorker();

  void Init();
  void Finalize();
  int Record(Packet* pkg);

  inline size_t NumAntennas() const { return num_antennas_; }
  inline size_t AntennaOffset() const { return antenna_offset_; }

 private:
  void Open();
  void Close();
  void Gc();

  const Config* cfg_;

  size_t antenna_offset_;
  size_t num_antennas_;
};
}; /* End namespace Agora_recorder */

#endif /* AGORA_RECORDER_WORKER_H_ */
