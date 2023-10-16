/**
 * @file  mac_scheduler.cc
 * @brief Declaration file for the simple MAC scheduler
 */
#include "mac_scheduler.h"

#include "logger.h"

static constexpr size_t kCSI_SubcarrierIdx = 0;

MacScheduler::MacScheduler(Config* const cfg) : cfg_(cfg) {
  scheduler_model_ = std::move(SchedulerModel::CreateSchedulerModel(cfg_));
  const size_t num_groups = scheduler_model_->NumGroups();

  ul_mcs_buffer_.Calloc(num_groups, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  dl_mcs_buffer_.Calloc(num_groups, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  for (size_t gp = 0u; gp < num_groups; gp++) {
    for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
      ul_mcs_buffer_[gp][ue] = cfg_->McsIndex(Direction::kUplink);
      dl_mcs_buffer_[gp][ue] = cfg_->McsIndex(Direction::kDownlink);
    }
  }
}

MacScheduler::~MacScheduler() {
  ul_mcs_buffer_.Free();
  dl_mcs_buffer_.Free();
}

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

void MacScheduler::UpdateScheduler(size_t frame_id) {
  scheduler_model_->Update(frame_id, csi_, snr_per_ue_);
}

void MacScheduler::UpdateSNR(std::vector<float> snr_per_ue) {
  snr_per_ue_ = snr_per_ue;
}

void MacScheduler::UpdateCSI(size_t cur_sc_id, const arma::cx_fmat& csi_in) {
  if (cur_sc_id == kCSI_SubcarrierIdx) {
    csi_ = csi_in;
  }
}

size_t MacScheduler::ScheduledUeUlMcs(size_t frame_id, size_t ue_id) const {
  const size_t gp = scheduler_model_->SelectedGroup();
  return ul_mcs_buffer_.At(gp)[ue_id];
}

size_t MacScheduler::ScheduledUeDlMcs(size_t frame_id, size_t ue_id) const {
  const size_t gp = scheduler_model_->SelectedGroup();
  return dl_mcs_buffer_.At(gp)[ue_id];
}