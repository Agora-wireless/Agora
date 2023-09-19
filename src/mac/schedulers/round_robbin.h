/**
 * @file round_robbin.h
 * @brief Declaration file for the Round Robbin scheduling algorithm 
 */

#ifndef ROUND_ROBBIN_H_
#define ROUND_ROBBIN_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "armadillo"
#include "config.h"

class RoundRobbin {
 public:
  RoundRobbin(const size_t spatial_streams, const size_t bss_num,
                       const size_t ues_num, size_t ofdm_data_num_);
  ~RoundRobbin() = default;

  arma::uvec GetScheduledUeList(size_t frame_id, size_t sc_id);

  Table<size_t> schedule_buffer_index_;
  Table<int> schedule_buffer_;
  
  size_t selected_group_;
  size_t num_groups_;

 protected: 
  const size_t spatial_streams_;
  const size_t bss_num_;
  const size_t ues_num_;
};

#endif //ROUND_ROBBIN_H_