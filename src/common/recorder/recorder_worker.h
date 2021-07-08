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

namespace Recorder {
const H5std_string dataset_root_prefix = "/Data";
/*
  An Interface for recorders
*/
class RecorderWorker {
  public:
  RecorderWorker() = delete;
  RecorderWorker(EventType type, Config *in_cfg, H5::H5File *h5_file)
  : dataset_name_(H5std_string(dataset_root_prefix + "/" + EventTypeString(type))),
    h5_file_(h5_file),
    cfg_(in_cfg) { }
  virtual ~RecorderWorker() = default;

  virtual herr_t Record(void *) = 0;

  EventType GetEventType() const {
    return event_type_;
  }

  std::string GetDataSetName() const {
    return dataset_name_;
  }

  private:
  const H5std_string dataset_name_;

  H5::H5File *h5_file_;

  Config *cfg_;
  EventType event_type_;
};

/*
  An Interface for worker factory
*/
class RecorderWorkerFactory {
  public:
  virtual ~RecorderWorkerFactory() = default;

  virtual RecorderWorker *GenWorker(Config *, H5::H5File *) = 0;
};
}; /* End namespace Recorder */

#endif /* AGORA_RECORDER_WORKER_H_ */
