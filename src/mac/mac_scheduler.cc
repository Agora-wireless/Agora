/**
 * @file  mac_scheduler.cc
 * @brief Declaration file for the simple MAC scheduler
 */
#include "mac_scheduler.h"
#include "logger.h"

static constexpr size_t kCSI_SubcarrierIdx = 0;

MacScheduler::MacScheduler(Config* const cfg) : cfg_(cfg) {
  
  proportional_fairness_ = std::make_unique<ProportionalFairness>(
      cfg_->SpatialStreamsNum(), cfg_->BsAntNum(), cfg_->UeAntNum(),
      cfg_->OfdmDataNum());

  round_robbin_ = std::make_unique<RoundRobbin>(
      cfg_->SpatialStreamsNum(), cfg_->BsAntNum(), cfg_->UeAntNum(),
      cfg_->OfdmDataNum());
  
  num_groups_ =
      (cfg_->SpatialStreamsNum() == cfg_->UeAntNum()) ? 1 : cfg_->UeAntNum();
  schedule_buffer_.Calloc(num_groups_, cfg_->UeAntNum() * cfg_->OfdmDataNum(),
                          Agora_memory::Alignment_t::kAlign64);
  schedule_buffer_index_.Calloc(num_groups_,
                                cfg_->SpatialStreamsNum() * cfg_->OfdmDataNum(),
                                Agora_memory::Alignment_t::kAlign64);
  // Create round-robin schedule
  for (size_t gp = 0u; gp < num_groups_; gp++) {
    for (size_t sc = 0; sc < cfg_->OfdmDataNum(); sc++) {
      for (size_t ue = gp; ue < gp + cfg_->SpatialStreamsNum(); ue++) {
        size_t cur_ue = ue % cfg_->UeAntNum();
        // for now all SCs are allocated to scheduled UEs
        schedule_buffer_[gp][cur_ue + cfg_->UeAntNum() * sc] = 1;
        schedule_buffer_index_[gp][(ue - gp) + cfg_->SpatialStreamsNum() * sc] =
            cur_ue;
      }
    }
  }

  ul_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  dl_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  for (size_t gp = 0u; gp < num_groups_; gp++) {
    for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
      ul_mcs_buffer_[gp][ue] = cfg_->McsIndex(Direction::kUplink);
      dl_mcs_buffer_[gp][ue] = cfg_->McsIndex(Direction::kDownlink);
    }
  }
}

MacScheduler::~MacScheduler() {
  schedule_buffer_.Free();
  schedule_buffer_index_.Free();
  ul_mcs_buffer_.Free();
  dl_mcs_buffer_.Free();
}

void MacScheduler::UpdateScheduler(size_t frame_id) {
  proportional_fairness_->Update(frame_id, csi_, snr_per_ue_);
}

void MacScheduler::UpdateSNR(std::vector<float> snr_per_ue) {
  snr_per_ue_ = snr_per_ue;
}

void MacScheduler::UpdateCSI(size_t cur_sc_id, const arma::cx_fmat& csi_in) {
  if (cur_sc_id == kCSI_SubcarrierIdx) {
    csi_ = csi_in;
  }
}

bool MacScheduler::IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) {
  size_t gp = frame_id % num_groups_;
  return (schedule_buffer_[gp][ue_id + cfg_->UeAntNum() * sc_id] != 0);
}

size_t MacScheduler::ScheduledUeIndex(size_t frame_id, size_t sc_id,
                                      size_t sched_ue_id) {
  return (size_t)this->ScheduledUeList(frame_id, sc_id)[sched_ue_id];
}

arma::uvec MacScheduler::ScheduledUeMap(size_t frame_id, size_t sc_id) {
  size_t gp = frame_id % num_groups_;
  return arma::uvec(reinterpret_cast<unsigned long long*>(
                        &schedule_buffer_[gp][cfg_->UeAntNum() * sc_id]),
                    cfg_->UeAntNum(), false);
}

arma::uvec MacScheduler::ScheduledUeList(size_t frame_id, size_t sc_id) {
  size_t gp = frame_id % num_groups_;
  return sort(arma::uvec(
      reinterpret_cast<unsigned long long*>(
          &schedule_buffer_index_[gp][cfg_->SpatialStreamsNum() * sc_id]),
      cfg_->SpatialStreamsNum(), false));
}

size_t MacScheduler::ScheduledUeUlMcs(size_t frame_id, size_t ue_id) {
  size_t gp = frame_id % num_groups_;
  return ul_mcs_buffer_[gp][ue_id];
}

size_t MacScheduler::ScheduledUeDlMcs(size_t frame_id, size_t ue_id) {
  size_t gp = frame_id % num_groups_;
  return dl_mcs_buffer_[gp][ue_id];
}
