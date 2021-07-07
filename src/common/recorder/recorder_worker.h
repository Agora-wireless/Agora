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
  RecorderWorker(EventType type, Config *in_cfg, std::string file_name)
  : file_name_(file_name),
    dataset_name_(H5std_string(dataset_root_prefix + "/" + EventTypeString(type))),
    cfg_(in_cfg) { }
  virtual ~RecorderWorker() = default;

  virtual herr_t Record(int, void *) = 0;

  EventType GetEventType() {
    return event_type_;
  }

  private:
  const H5std_string file_name_;
  const H5std_string dataset_name_;

  Config *cfg_;
  EventType event_type_;
};

/*
  An Interface for worker factory
*/
class RecorderWorkerFactory {
  public:
  virtual ~RecorderWorkerFactory() = default;

  virtual RecorderWorker *GenWorker(Config *, std::string) = 0;
};
}; /* End namespace Recorder */

#endif /* AGORA_RECORDER_WORKER_H_ */
