/**
 * @file rb_share.h
 * @brief Declaration file for beam_slice rb sharing scheduling algorithm 
 */

#ifndef RB_SHARING_H_
#define RB_SHARING_H_

#include <iostream>
#include <complex>
// #include "H5Cpp.h"
#include <cmath>
#include <iomanip>
#include <armadillo>
#include <vector>
#include <algorithm> // For std::max_element and std::distance
#include <numeric>   // For std::accumulate
#include <chrono>

#include <cstddef>
#include <vector>

#include "config.h"
#include "scheduler_model.h"


class RB_Share : public SchedulerModel {
 public:
  explicit RB_Share(Config* const cfg);
  ~RB_Share() override = default;

//   size_t UpdateScheduler(size_t frame_id, std::vector<float> ues_capacity);

  // void Update(size_t frame_id, const std::vector<arma::cx_fmat>& csi,
  //             const std::vector<float>& snr_per_ue) final;

  bool IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) final;
  arma::uvec ScheduledUeList(size_t frame_id, size_t sc_id) final;
  arma::uvec ScheduledUeMap(size_t frame_id, size_t sc_id) final;
  size_t SelectedUlMcs(size_t frame_id, size_t ue_id) final;
  size_t SelectedDlMcs(size_t frame_id, size_t ue_id) final;
  void Update(size_t frame_id) final;
  size_t GetGroup(size_t frame_id) final;

 private:

  // const int Num_RBG = 25;
  // const int Num_BS = 64;
  const double corr_th = 0.5;
  // const int total_tti = 10;
  // const int Num_UE = 192;
  const int Num_slice = 8;

  const std::vector<int> Num_UE_ps = {9, 11, 17, 19, 24, 32, 44, 36};
  // const int SEL_UE = 16;
  const std::vector<double> SLAs = {135, 120, 130, 140, 45, 110, 50, 125};

  const int new_rb_para = 0; // If RB Parallel

  std::vector<double> total_slice_tp;
  std::vector<double> est_slice_tp;
  size_t avg_rb;
  size_t total_rb_allocated;

  bool any_positive(const std::vector<double>& data);
  std::pair<std::vector<int>, arma::vec> alloc_rb(std::vector<std::vector<int>>& group_list, const arma::vec& cg_ue_vec, const arma::cx_mat& H, int ue_index, std::vector<int>& large_ue_list, std::vector<int>& small_ue_list);
  void do_alloc(const std::vector<arma::cx_fmat>& csi, size_t frame, const std::vector<float>& last_throughput);
};

#endif  //RB_SHARING_H_
