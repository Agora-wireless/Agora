#include "ue_scheduler.h"

#include "logger.h"

static constexpr bool kPrintUeSchedule = false;

UeScheduler::UeScheduler(Config* const cfg) : SchedulerModel(cfg) {
  num_groups_ = kFrameWnd;
  schedule_buffer_.Calloc(num_groups_, cfg_->UeAntNum() * num_prbs_,
                          Agora_memory::Alignment_t::kAlign64);
  schedule_buffer_index_.Calloc(num_groups_, cfg_->UeAntNum() * num_prbs_,
                                Agora_memory::Alignment_t::kAlign64);
  ul_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  dl_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  ue_num_array_.resize(num_groups_ * num_prbs_, cfg_->SpatialStreamsNum());
  AGORA_LOG_INFO("Initializing UE MAC Scheduler\n");
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
      AGORA_LOG_TRACE("UL MCS Init: gp %zu, ue %zu, mcs %zu\n", gp, ue,
                      ul_mcs_buffer_[gp][ue]);
    }
  }

  if (kPrintUeSchedule) {
    std::stringstream dataprint;
    for (size_t gp = 0u; gp < num_groups_; gp++) {
      dataprint << "Group " << gp << ":\n";
      for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
        for (size_t prb = 0; prb < num_prbs_; prb++) {
          dataprint << schedule_buffer_[gp][ue + prb * cfg_->UeAntNum()] << " ";
        }
        dataprint << "\n";
      }
    }
    AGORA_LOG_INFO("%s", dataprint.str());
  }
}

bool UeScheduler::IsUeScheduled(size_t frame_id, size_t prb_id, size_t ue_id) {
  const size_t gp = frame_id % num_groups_;
  return (schedule_buffer_[gp][ue_id + cfg_->UeAntNum() * prb_id] != 0);
}

arma::uvec UeScheduler::ScheduledUeMap(size_t frame_id, size_t prb_id) {
  const size_t gp = frame_id % num_groups_;
  return arma::uvec(reinterpret_cast<unsigned long long*>(
                        &schedule_buffer_[gp][cfg_->UeAntNum() * prb_id]),
                    cfg_->UeAntNum(), false);
}

arma::uvec UeScheduler::ScheduledUeList(size_t frame_id, size_t prb_id) {
  const size_t gp = frame_id % num_groups_;
  return sort(
      arma::uvec(reinterpret_cast<unsigned long long*>(
                     &schedule_buffer_index_[gp][cfg_->UeAntNum() * prb_id]),
                 ue_num_array_.at(gp * num_prbs_ + prb_id), false));
}

void UeScheduler::Update(size_t frame_id) {
  selected_group_ = this->GetGroup(frame_id);
}

void UeScheduler::Update(size_t frame_id, const std::vector<size_t> prb_ue_map,
                         const std::vector<size_t> ul_mcs,
                         const std::vector<size_t> dl_mcs) {
  mtx.lock();
  AGORA_LOG_INFO("UeScheduler Update: Frame %zu\n", frame_id);
  if (prb_ue_map.size() != cfg_->UeAntNum()) {
    AGORA_LOG_ERROR("UeScheduler Update: Invalid PRB map size %zu/%zu\n",
                    prb_ue_map.size(), cfg_->UeAntNum());
    return;
  }
  if (ul_mcs.size() != cfg_->UeAntNum() || dl_mcs.size() != cfg_->UeAntNum()) {
    AGORA_LOG_ERROR(
        "UeScheduler Update: Invalid MCS vector size UL: %zu, DL %zu, expected "
        "%zu\n",
        ul_mcs.size(), dl_mcs.size(), cfg_->UeAntNum());
    return;
  }
  const size_t gp = this->GetGroup(frame_id);
  arma::umat sched_mat(num_prbs_, cfg_->UeAntNum(), arma::fill::zeros);
  for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
    auto ue_map = Utils::Int2BitVector(prb_ue_map.at(ue), num_prbs_);
    sched_mat.col(ue) = ue_map;
  }
  for (size_t prb = 0; prb < num_prbs_; prb++) {
    size_t cnt = 0;
    for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
      schedule_buffer_[gp][ue + cfg_->UeAntNum() * prb] = sched_mat.at(prb, ue);
      if (sched_mat.at(prb, ue) == 1) {
        schedule_buffer_index_[gp][cnt + cfg_->UeAntNum() * prb] = ue;
        cnt++;
      }
    }
    ue_num_array_.at(gp * num_prbs_ + prb) = cnt;
  }
  for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
    ul_mcs_buffer_[gp][ue] = ul_mcs.at(ue);
    dl_mcs_buffer_[gp][ue] = dl_mcs.at(ue);
  }
  if (kPrintUeSchedule) {
    std::stringstream dataprint;
    dataprint << "Group " << gp << ":\n";
    for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
      for (size_t prb = 0; prb < num_prbs_; prb++) {
        dataprint << schedule_buffer_[gp][ue + prb * cfg_->UeAntNum()] << " ";
      }
      dataprint << "\n";
    }
    AGORA_LOG_INFO("%s", dataprint.str());
  }

  mtx.unlock();
}

size_t UeScheduler::GetGroup(size_t frame_id) { return frame_id % num_groups_; }

size_t UeScheduler::SelectedUlMcs(size_t frame_id, size_t ue_id) {
  return ul_mcs_buffer_[frame_id % num_groups_][ue_id];
}

size_t UeScheduler::SelectedDlMcs(size_t frame_id, size_t ue_id) {
  return dl_mcs_buffer_[frame_id % num_groups_][ue_id];
}
