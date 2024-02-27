/**
 * @file mac_scheduler.h
 * @brief Declaration file for the simple MAC scheduler class
 */
#ifndef MAC_SCHEDULER_H_
#define MAC_SCHEDULER_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "armadillo"
#include "config.h"
#include "memory_manage.h"
#include "scheduler_model.h"

class MacScheduler {
 public:
  explicit MacScheduler(Config* const cfg);
  ~MacScheduler();

  bool IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id);
  size_t ScheduledUeIndex(size_t frame_id, size_t sc_id, size_t sched_ue_id);
  arma::uvec ScheduledUeList(size_t frame_id, size_t sc_id);
  arma::uvec ScheduledUeMap(size_t frame_id, size_t sc_id);
  size_t UeScheduleIndex(size_t sched_d);
  size_t ScheduledUeUlMcs(size_t frame_id, size_t ue_id) const;
  size_t ScheduledUeDlMcs(size_t frame_id, size_t ue_id) const;

  //Used for Proportional Fairness Algorithm
  void UpdateCSI(size_t cur_sc_id, const arma::cx_fmat& csi_in);
  void UpdateSNR(std::vector<float> snr_per_ue);
  void UpdateScheduler(size_t frame_id);

 private:
  Table<size_t> ul_mcs_buffer_;
  Table<size_t> dl_mcs_buffer_;
  Config* const cfg_;

  std::vector<float> snr_per_ue_;
  arma::cx_fmat csi_;

  std::unique_ptr<SchedulerModel> scheduler_model_;
};

#endif  // MAC_SCHEDULER_H_
