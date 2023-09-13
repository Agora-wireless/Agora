/**
 * @file proportional_fairness.h
 * @brief Declaration file for the Proportional Fairness scheduling algorithm 
 */

#ifndef PROPORTIONAL_FAIRNESS_H_
#define PROPORTIONAL_FAIRNESS_H_

#include <cstddef>
#include <memory>
#include <vector>
#include "config.h"

#include "armadillo"

class ProportionalFairness {
 public:
  ProportionalFairness( const size_t spatial_streams, const size_t bss_num, const size_t ues_num, size_t ofdm_data_num_ );
  ~ProportionalFairness() = default;

  size_t UpdateScheduler( size_t frame, std::vector<float> csi );

  //Possible Proportional Fairness actions N_Combination_R
  size_t actions_num_;
  Table<int> schedule_buffer_;
  Table<size_t> schedule_buffer_index_;
  
  //Action index range[0,N_Combination_R]
  size_t selected_action_;

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

  //Vector of size 1 x UEs that records the UE Schedule, 1 = Scheduled, 0 = Non Scheduled
  arma::vec last_flag_;
  //Vector of size 1 x UEs that records the sum of UEs max capacity

  size_t current_frame = 0;

  std::vector<bool> ues_flags_;
  std::vector<float> pf_ues_history;
  
  std::vector< std::vector<size_t> > combination_vector;
  std::vector<size_t> ues_vector;
  std::vector<size_t> combination;

  void Schedule( size_t frame, std::vector<float> ues_capacity );
  void UpdatePF( size_t frame, std::vector<float> ues_capacity );

  void Combination( int k, int offset);
  
};

#endif  //PROPORTIONAL_FAIRNESS_H_
