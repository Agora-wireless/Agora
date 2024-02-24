/**
 * @file custom_schedule.h
 * @brief Declaration file for the Custom scheduling algorithm 
 */

#ifndef CUSTOM_SCHEDULE_H_
#define CUSTOM_SCHEDULE_H_

#include <cstddef>

#include "armadillo"
#include "config.h"
#include "scheduler_model.h"

class CustomSchedule : public SchedulerModel {
 public:
  explicit CustomSchedule(Config* const cfg);
  ~CustomSchedule() override = default;

  bool IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) final;
  arma::uvec ScheduledUeList(size_t frame_id, size_t sc_id) final;
  arma::uvec ScheduledUeMap(size_t frame_id, size_t sc_id) final;

 private:
  std::vector<uint8_t> ue_map_array_;
  std::vector<uint8_t> ue_num_array_;
};

#endif  //CUSTOM_SCHEDULE_H_
