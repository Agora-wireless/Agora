/**
 * @file  mac_scheduler.cc
 * @brief Declaration file for the simple MAC scheduler
 */
#include "mac_scheduler.h"

#include <utility>

#include "logger.h"

static constexpr size_t kCsiSubcarrierIdx = 0;

MacScheduler::MacScheduler(Config* const cfg)
    : cfg_(cfg), params_(cfg->MacParams()) {
  scheduler_model_ = std::move(SchedulerModel::CreateSchedulerModel(cfg_));
}

MacScheduler::~MacScheduler() = default;

size_t MacScheduler::ScheduledUeIndex(size_t frame_id, size_t sc_id,
                                      size_t sched_ue_id) {
  return (size_t)scheduler_model_->ScheduledUeList(frame_id,
                                                   sc_id)[sched_ue_id];
}

bool MacScheduler::IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) {
  return scheduler_model_->IsUeScheduled(frame_id, sc_id, ue_id);
}

arma::uvec MacScheduler::ScheduledUeMap(size_t frame_id, size_t sc_id) {
  return scheduler_model_->ScheduledUeMap(frame_id, sc_id);
}

arma::uvec MacScheduler::ScheduledUeList(size_t frame_id, size_t sc_id) {
  return scheduler_model_->ScheduledUeList(frame_id, sc_id);
}

size_t MacScheduler::UeScheduleIndex(size_t sched_id) {
  return scheduler_model_->UeScheduleIndex(sched_id);
}

size_t MacScheduler::SelectedUlMcs(size_t frame_id, size_t ue_id) {
  return scheduler_model_->SelectedUlMcs(frame_id, ue_id);
}

size_t MacScheduler::SelectedDlMcs(size_t frame_id, size_t ue_id) {
  return scheduler_model_->SelectedDlMcs(frame_id, ue_id);
}

void MacScheduler::UpdateScheduler(size_t frame_id) {
  scheduler_model_->Update(frame_id, csi_, snr_per_ue_);
}

size_t MacScheduler::NumGroups() { return scheduler_model_->NumGroups(); }

size_t MacScheduler::SelectedGroup() {
  return scheduler_model_->SelectedGroup();
}

void MacScheduler::UpdateSNR(std::vector<float> snr_per_ue) {
  snr_per_ue_ = std::move(snr_per_ue);
}

void MacScheduler::UpdateCSI(size_t cur_sc_id, const arma::cx_fmat& csi_in) {
  if (cur_sc_id == kCsiSubcarrierIdx) {
    csi_ = csi_in;
  }
}

void MacScheduler::UpdateMcsParams(size_t frame_id) {
  size_t ul_mcs = this->SelectedUlMcs(frame_id, 0u);
  size_t dl_mcs = this->SelectedDlMcs(frame_id, 0u);
  AGORA_LOG_INFO("Frame %zu: updating UL MCS: %zu, DL MCS %zu\n", frame_id,
                 ul_mcs, dl_mcs);
  this->Params().UpdateUlMcsParams(ul_mcs);
  this->Params().UpdateDlMcsParams(dl_mcs);
}
