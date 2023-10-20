/**
 * @file scheduler_model.cc
 * @brief Defination file for the generic scheduler model.
 */
#include "scheduler_model.h"

//scheduler models
#include "logger.h"
#include "proportional_fairness.h"
#include "round_robbin.h"

///Factory function
std::unique_ptr<SchedulerModel> SchedulerModel::CreateSchedulerModel(
    Config* const cfg) {
  std::string scheduler_type = cfg->SchedulerType();
  if (scheduler_type == "round_robbin") {
    return std::make_unique<RoundRobbin>(cfg);
  } else if (scheduler_type == "proportional_fairness") {
    return std::make_unique<ProportionalFairness>(cfg);
  } else {
    AGORA_LOG_WARN("Invalid scheduler type (%s), using Round Robbin... \n",
                   scheduler_type.c_str());
    return std::make_unique<RoundRobbin>(cfg);
  }
}

SchedulerModel::SchedulerModel(Config* const cfg) : cfg_(cfg) {}

SchedulerModel::~SchedulerModel() {
  schedule_buffer_index_.Free();
  schedule_buffer_.Free();
}