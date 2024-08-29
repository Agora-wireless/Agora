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
#include "mac_utils.h"
#include "memory_manage.h"
#include "scheduler_model.h"

class MacScheduler {
 public:
  explicit MacScheduler(Config* const cfg, bool client = false);
  ~MacScheduler();

  bool IsUeScheduled(size_t frame_id, size_t ue_id);
  bool IsUeScheduled(size_t frame_id, size_t prb_id, size_t ue_id);
  size_t ScheduledUeIndex(size_t frame_id, size_t prb_id, size_t sched_ue_id);
  arma::uvec ScheduledUeList(size_t frame_id, size_t prb_id);
  arma::uvec ScheduledUeMap(size_t frame_id, size_t prb_id);
  arma::uvec SchedulePrbList(size_t frame_id, size_t ue_id);
  arma::uvec ScheduledPrbMap(size_t frame_id, size_t ue_id);
  size_t NumScheduledPrbs(size_t frame_id, size_t ue_id);
  size_t NumScheduledUes(size_t frame_id);
  size_t UeScheduleIndex(size_t sched_id);
  size_t SelectedUlMcs(size_t frame_id, size_t ue_id);
  size_t SelectedDlMcs(size_t frame_id, size_t ue_id);
  size_t MacPacketLength(Direction dir, size_t frame_id, size_t ue_id);

  //Used for Proportional Fairness Algorithm
  //void UpdateCSI(size_t cur_sc_id, const arma::cx_fmat& csi_in);
  //void UpdateSNR(std::vector<float> snr_per_ue);
  void UpdateScheduler(size_t frame_id);
  void UpdateScheduler(size_t frame_id, std::vector<size_t> prb_map,
                       std::vector<size_t> ul_mcs, std::vector<size_t> dl_mcs);
  void UpdateScheduler(size_t frame_id, std::vector<arma::cx_fmat>& csi_mat,
                       std::vector<float> snr_per_ue);

  void UpdateMcsParams(size_t frame_id);
  inline MacUtils& Params() { return this->params_; }
  size_t NumGroups();
  size_t SelectedGroup();

 private:
  Config* const cfg_;

  /*std::vector<float> snr_per_ue_;
  arma::cx_fmat csi_;*/

  std::unique_ptr<SchedulerModel> scheduler_model_;
  MacUtils params_;
};

#endif  // MAC_SCHEDULER_H_
