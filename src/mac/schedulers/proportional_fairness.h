/**
 * @file proportional_fairness.h
 * @brief Declaration file for the Proportional Fairness scheduling algorithm 
 */

#ifndef PROPORTIONAL_FAIRNESS_H_
#define PROPORTIONAL_FAIRNESS_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "armadillo"
#include "config.h"

class ProportionalFairness {
 public:
  ProportionalFairness(const size_t spatial_streams, const size_t bss_num,
                       const size_t ues_num, size_t ofdm_data_num_);
  ~ProportionalFairness() = default;

  size_t UpdateScheduler(size_t frame_id, std::vector<float> ues_capacity);

  Table<int> schedule_buffer_;
  Table<size_t> schedule_buffer_index_;

  size_t num_groups_;
  size_t selected_group_;

  void Update(size_t frame_id, arma::cx_fmat csi,
              std::vector<float> snr_per_ue);

  //Returns the maximum capacity per ue
  std::vector<float> UEsCapacity(arma::cx_fmat csi,
                                 std::vector<float> snr_per_ue);

 protected:
  //Number of scheduled UEs each frame
  const size_t spatial_streams_;
  const size_t bss_num_;
  const size_t ues_num_;

  //Data rate weight, range[0,1]
  const float lamda_;

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

  void Schedule(size_t frame, std::vector<float> ues_capacity);
  void UpdatePF(size_t frame, std::vector<float> ues_capacity);

  void Combination(int k, int offset);
};

#endif  //PROPORTIONAL_FAIRNESS_H_
