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
#include "memory_manage.h"

class SchedulerModel {
 public:
  explicit SchedulerModel(Config* const cfg);
  virtual ~SchedulerModel();

  virtual void Update([[maybe_unused]] size_t frame_id) {}

  virtual void Update([[maybe_unused]] size_t frame_id,
                      [[maybe_unused]] const std::vector<arma::cx_fmat>& csi,
                      [[maybe_unused]] const std::vector<float>& snr_per_ue) {}

  virtual void Update([[maybe_unused]] size_t frame_id,
                      [[maybe_unused]] const std::vector<size_t> prb_map,
                      [[maybe_unused]] const std::vector<size_t> ul_mcs,
                      [[maybe_unused]] const std::vector<size_t> dl_mcs) {}

  virtual bool IsUeScheduled([[maybe_unused]] size_t frame_id,
                             [[maybe_unused]] size_t prb_id,
                             [[maybe_unused]] size_t ue_id) {
    return false;
  }
  virtual arma::uvec ScheduledUeList([[maybe_unused]] size_t frame_id,
                                     [[maybe_unused]] size_t prb_id) {
    return {};
  }
  virtual arma::uvec ScheduledUeMap([[maybe_unused]] size_t frame_id,
                                    [[maybe_unused]] size_t prb_id) {
    return {};
  }
  virtual arma::uvec SchedulePrbList(size_t frame_id, size_t ue_id) {
    return {};
  }
  arma::uvec ScheduledPrbMap(size_t frame_id, size_t ue_id) {
    const size_t gp = this->GetGroup(frame_id);
    auto sched_mat =
        arma::umat(reinterpret_cast<unsigned long long*>(schedule_buffer_[gp]),
                   cfg_->UeAntNum(), num_prbs_, false);
    return sched_mat.row(ue_id).st();
  }
  size_t NumScheduledUes(size_t frame_id) {
    const size_t gp = this->GetGroup(frame_id);
    auto sched_mat =
        arma::umat(reinterpret_cast<unsigned long long*>(schedule_buffer_[gp]),
                   cfg_->UeAntNum(), num_prbs_, false);
    arma::uvec scheduled_ues = arma::find(arma::sum(sched_mat, 1) > 0);
    return scheduled_ues.n_elem;
    /*
     * num_sched_ues = function NumScheduledUes(num_ues, num_prbs)
     * A = randn(num_ues, num_prbs);
     * num_sched_ues = length(find(sum(A, 2) > 0));
     * end
     */
  }
  virtual size_t UeScheduleIndex([[maybe_unused]] size_t sched_id) {
    return {};
  }
  virtual size_t GetGroup(size_t frame_id) { return {}; }

  virtual size_t SelectedUlMcs(size_t frame_id, size_t ue_id) { return 0; }
  virtual size_t SelectedDlMcs(size_t frame_id, size_t ue_id) { return 0; }

  static std::unique_ptr<SchedulerModel> CreateSchedulerModel(Config* const cfg,
                                                              bool client);

  inline size_t SelectedGroup() const { return selected_group_; }
  inline size_t NumGroups() const { return num_groups_; }

 protected:
  Config* const cfg_;

  Table<size_t> schedule_buffer_index_;
  Table<size_t> schedule_buffer_;

  Table<size_t> ul_mcs_buffer_;
  Table<size_t> dl_mcs_buffer_;

  size_t selected_group_{0};
  size_t num_groups_;
  size_t num_prbs_;
};

#endif  //SCHEDULER_MODEL_H_
