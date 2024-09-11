/**
 * @file helix_orth.h
 * @brief Declaration file for beam_slice rb sharing scheduling algorithm 
 */

#ifndef RB_ORTH_H_
#define RB_ORTH_H_

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


class RB_Orth : public SchedulerModel {
 public:
  explicit RB_Orth(Config* const cfg);
  ~RB_Orth() override = default;

//   size_t UpdateScheduler(size_t frame_id, std::vector<float> ues_capacity);

  // void Update(size_t frame_id, const std::vector<arma::cx_fmat>& csi,
  //             const std::vector<float>& snr_per_ue) final;

  bool IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) final;
  arma::uvec ScheduledUeList(size_t frame_id, size_t sc_id) final;
  arma::uvec ScheduledUeMap(size_t frame_id, size_t sc_id) final;
  size_t SelectedUlMcs(size_t frame_id, size_t ue_id) final;
  size_t SelectedDlMcs(size_t frame_id, size_t ue_id) final;
  void Update(size_t frame_id) final;
  void Update(size_t frame_id, const std::vector<arma::cx_fmat>& csi,
            [[maybe_unused]] const std::vector<float>& snr_per_ue,
            const std::vector<float>& last_throughout) final;
  size_t GetGroup(size_t frame_id) final;

 private:

  std::pair<std::vector<int>, float> alloc_rb(
      std::vector<std::vector<int>>& group_list, int sel_sl,
      const arma::cx_fmat& H, int ue_index,
      std::vector<int>& sl_ue_list, arma::vec& cg_ue_vec);
  inline bool any_positive(const std::vector<float>& data) {
    return std::any_of(data.begin(), data.end(), [](float x) { return x > 1; });
  }

  // const int Num_RBG = 25;
  // const int Num_BS = 64;
  const double corr_th = 0.5;
  // const int total_tti = 10;
  // const int Num_UE = 192;
  const size_t Num_slice = 8;

  const std::vector<size_t> Num_UE_ps = {9, 11, 17, 19, 24, 32, 44, 36};
  // const int SEL_UE = 16;
  const std::vector<double> SLAs = {135, 120, 130, 140, 45, 110, 50, 125};

  const int new_rb_para = 0; // If RB Parallel

  std::vector<size_t> selected_group_vec_;
  std::vector<float> total_slice_tp_;
  std::vector<float> est_slice_tp_;
  // Store group in a new vector
  std::vector<std::vector<int>> group_vector_;

  arma::vec group_len_;
  arma::vec cg_tti_rb_ue_;
  float avg_rb_;
  size_t total_rb_allocated_;
};

#endif  //RB_ORTH_H_
