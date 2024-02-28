/**
 * @file round_robbin.h
 * @brief Declaration file for the Round Robbin scheduling algorithm 
 */

#ifndef ROUND_ROBBIN_H_
#define ROUND_ROBBIN_H_

#include <cstddef>

#include "armadillo"
#include "config.h"
#include "scheduler_model.h"

class RoundRobbin : public SchedulerModel {
 public:
  explicit RoundRobbin(Config* const cfg);
  ~RoundRobbin() override = default;

  bool IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) final;
  arma::uvec ScheduledUeList(size_t frame_id, size_t sc_id) final;
  arma::uvec ScheduledUeMap(size_t frame_id, size_t sc_id) final;
};

#endif  //ROUND_ROBBIN_H_