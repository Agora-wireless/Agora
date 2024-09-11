/**
 * @file scheduler_model.cc
 * @brief Defination file for the generic scheduler model.
 */
#include "scheduler_model.h"

//scheduler models
#include "custom_schedule.h"
#include "helix_orth.h"
#include "helix_share.h"
#include "logger.h"
#include "proportional_fairness.h"
#include "round_robbin.h"
#include "ue_scheduler.h"

///Factory function
std::unique_ptr<SchedulerModel> SchedulerModel::CreateSchedulerModel(
    Config* const cfg, bool client) {
  std::string scheduler_type = cfg->SchedulerType();
  if (scheduler_type == "rr") {
    return std::make_unique<RoundRobbin>(cfg);
  } else {  // all the dynamic schedulers
    if (client == true) {
      return std::make_unique<UeScheduler>(cfg);
    } else {
      if (scheduler_type == "pf") {
        return std::make_unique<ProportionalFairness>(cfg);
      } else if (scheduler_type == "helix_share") {
        return std::make_unique<RB_Share>(cfg);
      } else if (scheduler_type == "helix_orth") {
        return std::make_unique<RB_Orth>(cfg);
      } else if (scheduler_type == "custom") {
        return std::make_unique<CustomSchedule>(cfg);
      } else {
        AGORA_LOG_WARN("Invalid scheduler type (%s), using Round Robbin... \n",
                       scheduler_type.c_str());
        return std::make_unique<RoundRobbin>(cfg);
      }
    }
  }
}

SchedulerModel::SchedulerModel(Config* const cfg) : cfg_(cfg) {
  // num_prbs are currently the same for uplink and downlink
  num_prbs_ =
      cfg_->MacParams().LdpcConfig(Direction::kUplink).NumBlocksInSymbol();
}

SchedulerModel::~SchedulerModel() {
  schedule_buffer_index_.Free();
  schedule_buffer_.Free();
  ul_mcs_buffer_.Free();
  dl_mcs_buffer_.Free();
}
