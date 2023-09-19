/**
 * @file scheduler_model.h
 * @brief Declaration file for the scheduler model API
*/
#ifndef SCHEDULER_MODEL_H_
#define SCHEDULER_MODEL_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "armadillo"
#include "config.h"

class SchedulerModel {
 public:
  SchedulerModel(Config* const cfg);
  virtual ~SchedulerModel();

  virtual void Update(size_t frame_id, arma::cx_fmat csi_,
                      std::vector<float> snr_per_ue_) {}

  virtual bool IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) {
    return false;
  }
  virtual arma::uvec ScheduledUeList(size_t frame_id, size_t sc_id) {
    return arma::uvec();
  }
  virtual arma::uvec ScheduledUeMap(size_t frame_id, size_t sc_id) {
    return arma::uvec();
  }

  static std::unique_ptr<SchedulerModel> CreateSchedulerModel(
      Config* const cfg);

  inline const size_t SelectedGroup() const { return selected_group_; }
  inline const size_t NumGroups() const { return num_groups_; }

 protected:
  Config* const cfg_;

  Table<size_t> schedule_buffer_index_;
  Table<int> schedule_buffer_;

  size_t selected_group_;
  size_t num_groups_;
};

#endif  //SCHEDULER_MODEL_H_
