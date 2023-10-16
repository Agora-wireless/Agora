/**
 * @file proportional_fairness.h
 * @brief Declaration file for the Proportional Fairness scheduling algorithm 
 */

#ifndef PROPORTIONAL_FAIRNESS_H_
#define PROPORTIONAL_FAIRNESS_H_

#include <cstddef>
#include <vector>

#include "armadillo"
#include "config.h"
#include "scheduler_model.h"

class ProportionalFairness : public SchedulerModel {
 public:
  explicit ProportionalFairness(Config* const cfg);
  ~ProportionalFairness() = default;

  size_t UpdateScheduler(size_t frame_id, std::vector<float> ues_capacity);

  void Update(size_t frame_id, arma::cx_fmat csi,
              std::vector<float> snr_per_ue) final;

  bool IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) final;
  arma::uvec ScheduledUeList(size_t frame_id, size_t sc_id) final;
  arma::uvec ScheduledUeMap(size_t frame_id, size_t sc_id) final;

 protected:
  arma::vec last_SE_;

  //Vector of size 1 x UEs that records the sum of UEs max capacity
  arma::vec pf_UEs_history_;

  //Matrix of size Frames x UEs, records the UEs max capacity at specific frame
  std::vector<arma::vec> pf_UEs_history_frames_;

  size_t current_frame = 0;

  std::vector<bool> ues_flags_;
  std::vector<float> pf_ues_history;

  //Vector of possible scheduling options
  std::vector<std::vector<size_t> > groups_vector;
  std::vector<size_t> ues_vector;
  std::vector<size_t> combination;

  std::vector<float> snr_per_ue_;
  arma::cx_fmat csi_;

  std::vector<float> capacities_per_ue_;
  arma::cx_fcube channel_covariance_matrices_;

  //Returns the maximum capacity per ue
  std::vector<float> UEsCapacity(arma::cx_fmat csi,
                                 std::vector<float> snr_per_ue);

  void Schedule(size_t frame, std::vector<float> ues_capacity);
  void UpdatePF(size_t frame, std::vector<float> ues_capacity);

  void Combination(int k, int offset);
};

#endif  //PROPORTIONAL_FAIRNESS_H_
