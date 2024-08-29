/**
 * @file ue_scheduler.h
 * @brief Declaration file for class handling the 
 * schedules sent by Agora at the UE side
 */

#ifndef UE_SCHEDULER_H_
#define UE_SCHEDULER_H_

#include <cstddef>
#include <mutex>

#include "armadillo"
#include "config.h"
#include "scheduler_model.h"

class UeScheduler : public SchedulerModel {
 public:
  explicit UeScheduler(Config* const cfg);
  ~UeScheduler() override = default;

  bool IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) final;
  arma::uvec ScheduledUeList(size_t frame_id, size_t prb_id) final;
  arma::uvec ScheduledUeMap(size_t frame_id, size_t prb_id) final;
  size_t SelectedUlMcs(size_t frame_id, size_t ue_id) final;
  size_t SelectedDlMcs(size_t frame_id, size_t ue_id) final;
  void Update(size_t frame_id) final;
  void Update(size_t frame_id, const std::vector<size_t> prb_map,
              const std::vector<size_t> ul_mcs,
              const std::vector<size_t> dl_mcs) final;
  size_t GetGroup(size_t frame_id) final;

 private:
  std::vector<size_t> ue_num_array_;
  std::mutex mtx;  // mutex for critical section
};

#endif  //UE_SCHEDULER_H_
