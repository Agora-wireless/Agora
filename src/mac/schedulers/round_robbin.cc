#include "round_robbin.h"

#include "logger.h"

RoundRobbin::RoundRobbin(Config* const cfg) : SchedulerModel(cfg) {
  num_groups_ =
      (cfg_->SpatialStreamsNum() == cfg_->UeAntNum()) ? 1 : cfg_->UeAntNum();
  schedule_buffer_.Calloc(num_groups_, cfg_->UeAntNum() * cfg_->OfdmDataNum(),
                          Agora_memory::Alignment_t::kAlign64);
  schedule_buffer_index_.Calloc(num_groups_,
                                cfg_->SpatialStreamsNum() * cfg_->OfdmDataNum(),
                                Agora_memory::Alignment_t::kAlign64);
  //Round Robbin Schedule Buffer Process
  for (size_t gp = 0u; gp < num_groups_; gp++) {
    for (size_t sc = 0; sc < cfg_->OfdmDataNum(); sc++) {
      for (size_t ue = gp; ue < gp + cfg_->SpatialStreamsNum(); ue++) {
        size_t cur_ue = ue % cfg_->UeAntNum();
        schedule_buffer_[gp][cur_ue + cfg_->UeAntNum() * sc] = 1;
        schedule_buffer_index_[gp][(ue - gp) + cfg_->SpatialStreamsNum() * sc] =
            cur_ue;
      }
    }
  }
}

bool RoundRobbin::IsUeScheduled(size_t frame_id, size_t sc_id, size_t ue_id) {
  size_t gp = frame_id % num_groups_;
  return (schedule_buffer_[gp][ue_id + cfg_->UeAntNum() * sc_id] != 0);
}

arma::uvec RoundRobbin::ScheduledUeMap(size_t frame_id, size_t sc_id) {
  size_t gp = frame_id % num_groups_;
  return arma::uvec(reinterpret_cast<unsigned long long*>(
                        &schedule_buffer_[gp][cfg_->UeAntNum() * sc_id]),
                    cfg_->UeAntNum(), false);
}

arma::uvec RoundRobbin::ScheduledUeList(size_t frame_id, size_t sc_id) {
  size_t gp = frame_id % num_groups_;
  return sort(arma::uvec(
      reinterpret_cast<unsigned long long*>(
          &schedule_buffer_index_[gp][cfg_->SpatialStreamsNum() * sc_id]),
      cfg_->SpatialStreamsNum(), false));
}