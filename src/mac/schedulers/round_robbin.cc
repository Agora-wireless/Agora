#include "round_robbin.h"

#include "logger.h"

RoundRobbin::RoundRobbin(Config* const cfg) : SchedulerModel(cfg) {
  num_groups_ =
      (cfg_->SpatialStreamsNum() == cfg_->UeAntNum()) ? 1 : cfg_->UeAntNum();
  // num_prbs are currently the same for uplink and downlink
  num_prbs_ =
      cfg_->MacParams().LdpcConfig(Direction::kUplink).NumBlocksInSymbol();
  schedule_buffer_.Calloc(num_groups_, cfg_->UeAntNum() * num_prbs_,
                          Agora_memory::Alignment_t::kAlign64);
  schedule_buffer_index_.Calloc(num_groups_,
                                cfg_->SpatialStreamsNum() * num_prbs_,
                                Agora_memory::Alignment_t::kAlign64);
  ul_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  dl_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  //Round Robbin Schedule Buffer Process
  for (size_t gp = 0u; gp < num_groups_; gp++) {
    for (size_t ue = gp; ue < gp + cfg_->SpatialStreamsNum(); ue++) {
      for (size_t prb = 0; prb < num_prbs_; prb++) {
        size_t cur_ue = ue % cfg_->UeAntNum();
        schedule_buffer_[gp][cur_ue + cfg_->UeAntNum() * prb] = 1;
        schedule_buffer_index_[gp][(ue - gp) +
                                   cfg_->SpatialStreamsNum() * prb] = cur_ue;
      }
    }
    for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
      ul_mcs_buffer_[gp][ue] = cfg->MacParams().McsIndex(Direction::kUplink);
      dl_mcs_buffer_[gp][ue] = cfg->MacParams().McsIndex(Direction::kDownlink);
      AGORA_LOG_INFO("UL MCS Init: gp %zu, ue %zu, mcs %zu\n", gp, ue,
                     ul_mcs_buffer_[gp][ue]);
    }
  }
}

bool RoundRobbin::IsUeScheduled(size_t frame_id, size_t prb_id, size_t ue_id) {
  const size_t gp = frame_id % num_groups_;
  return (schedule_buffer_[gp][ue_id + cfg_->UeAntNum() * prb_id] != 0);
}

arma::uvec RoundRobbin::ScheduledUeMap(size_t frame_id, size_t prb_id) {
  const size_t gp = frame_id % num_groups_;
  return arma::uvec(reinterpret_cast<unsigned long long*>(
                        &schedule_buffer_[gp][cfg_->UeAntNum() * prb_id]),
                    cfg_->UeAntNum(), false);
}

arma::uvec RoundRobbin::ScheduledUeList(size_t frame_id, size_t prb_id) {
  const size_t gp = frame_id % num_groups_;
  return sort(arma::uvec(
      reinterpret_cast<unsigned long long*>(
          &schedule_buffer_index_[gp][cfg_->SpatialStreamsNum() * prb_id]),
      cfg_->SpatialStreamsNum(), false));
}

size_t RoundRobbin::SelectedUlMcs(size_t frame_id, size_t ue_id) {
  return ul_mcs_buffer_[frame_id % num_groups_][ue_id];
}

size_t RoundRobbin::SelectedDlMcs(size_t frame_id, size_t ue_id) {
  return dl_mcs_buffer_[frame_id % num_groups_][ue_id];
}
