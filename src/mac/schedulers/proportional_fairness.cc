/**
 * @file proportional_fairness.cc
 * @brief Implementation file for the Proportional Fairness scheduling algorithm 
 */
#include "proportional_fairness.h"

#include "logger.h"

static constexpr bool kPrintSchedulingBuffers = true;
static constexpr bool kPrintSchedulingGroups = true;
static constexpr bool kPrintSelectedGroup = true;
static constexpr size_t kCalibFrames = 50;
static constexpr float kLamda = 0.5;

ProportionalFairness::ProportionalFairness(Config* const cfg)
    : SchedulerModel(cfg),
      last_se_(arma::zeros(cfg->UeAntNum())),
      capacities_per_ue_(cfg->UeAntNum() * num_prbs_),
      channel_covariance_matrices_(cfg->BsAntNum(), cfg->BsAntNum(),
                                   cfg->UeAntNum() * num_prbs_,
                                   arma::fill::zeros) {
  //Define UE Index vector
  for (size_t ue_idx = 0; ue_idx < cfg_->UeAntNum(); ue_idx++) {
    ues_vector_.push_back(ue_idx);
    ues_flags_.push_back(false);
    pf_ues_history_.push_back(0.01F);
  }

  //Possible scheduling options
  Combination(cfg_->SpatialStreamsNum());
  num_groups_ = groups_vector_.size();
  schedule_buffer_.Calloc(num_groups_, cfg_->UeAntNum() * num_prbs_,
                          Agora_memory::Alignment_t::kAlign64);
  schedule_buffer_index_.Calloc(num_groups_,
                                cfg_->SpatialStreamsNum() * num_prbs_,
                                Agora_memory::Alignment_t::kAlign64);

  //Proportional Fairness Schedule Buffer Process
  for (size_t gp = 0; gp < num_groups_; gp++) {
    for (size_t sc = 0; sc < num_prbs_; sc++) {
      std::vector<size_t> u_es_idx = groups_vector_[gp];
      for (size_t ue_idx = 0; ue_idx < cfg_->SpatialStreamsNum(); ue_idx++) {
        schedule_buffer_[gp][u_es_idx[ue_idx] + cfg_->UeAntNum() * sc] = 1;
      }
      for (size_t ue = gp; ue < gp + cfg_->SpatialStreamsNum(); ue++) {
        schedule_buffer_index_[gp][(ue - gp) + cfg_->SpatialStreamsNum() * sc] =
            u_es_idx[ue - gp];
      }
    }
  }

  std::stringstream ss;
  if (kPrintSchedulingBuffers) {
    for (size_t row = 0; row < num_groups_; row++) {
      ss << "schedule_PF_index_buffer_" << row << " \n\n";
      for (size_t col = 0; col < cfg_->SpatialStreamsNum() * num_prbs_; col++) {
        ss << schedule_buffer_index_[row][col] << " ";
      }
      ss << "\n\n";
      ss << "schedule_PF_SC_buffer_" << row << " \n\n";
      for (size_t col = 0; col < cfg_->SpatialStreamsNum() * num_prbs_; col++) {
        ss << schedule_buffer_[row][col] << " ";
      }
      ss << "\n\n";
    }
  }
  if (kPrintSchedulingGroups) {
    ss << "Proportional Fairness Scheduling Groups \n\n";
    for (size_t gp = 0; gp < num_groups_; gp++) {
      ss << "Group " << gp << " = [ ";
      for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
        ss << schedule_buffer_[gp][ue] << " ";
      }
      ss << "]\n\n";
    }
  }
  AGORA_LOG_INFO(ss.str());
}

void ProportionalFairness::Combination(int k, int offset) {
  if (k == 0) {
    groups_vector_.push_back(combination_);
    return;
  }
  for (size_t i = offset; i <= ues_vector_.size() - k; ++i) {
    combination_.push_back(ues_vector_[i]);
    Combination(k - 1, i + 1);
    combination_.pop_back();
  }
}

void ProportionalFairness::Update(size_t frame_id,
                                  const std::vector<arma::cx_fmat>& csi,
                                  const std::vector<float>& snr_per_ue) {
  std::vector<float> ues_capacity = UEsCapacity(csi, snr_per_ue);

  if (frame_id < kCalibFrames) {
    selected_group_ = frame_id % num_groups_;
    if (kPrintSelectedGroup) {
      std::stringstream s;
      s << "Proportional Fairness Calibration " << frame_id << "/"
        << kCalibFrames << "\n";
      AGORA_LOG_INFO(s.str());
    }
  } else if (current_frame_ != frame_id) {
    current_frame_ = frame_id;
    Schedule(frame_id, ues_capacity);
    UpdatePF(frame_id, ues_capacity);
    if (kPrintSelectedGroup) {
      std::stringstream s;
      s << "Proportional Fairness Scheduled Frame " << frame_id << " Group "
        << selected_group_ << " = [ ";
      for (size_t i = 0; i < cfg_->UeAntNum(); i++) {
        std::string a = (ues_flags_[i]) ? "1 " : "0 ";
        s << a;
      }
      s << "] \n ";
      AGORA_LOG_INFO(s.str());
    }
  }
}

std::vector<float> ProportionalFairness::UEsCapacity(
    const std::vector<arma::cx_fmat>& csi,
    const std::vector<float>& snr_per_ue) {
  // Calculate the channel covariance matrices for each UE
  std::vector<size_t> selected_ues = groups_vector_[selected_group_];
  for (size_t ue = 0; ue < cfg_->SpatialStreamsNum(); ue++) {
    size_t ue_idx = selected_ues[ue];
    arma::cx_fmat csi_mat = csi.at(ue_idx);
    for (size_t prb = 0; prb < num_prbs_; prb++) {
      channel_covariance_matrices_.slice(ue_idx * num_prbs_ + prb) =
          csi_mat.col(prb) * csi_mat.col(prb).t();
      arma::cx_fmat capacity_matrix =
          arma::eye<arma::cx_fmat>(cfg_->BsAntNum(), cfg_->BsAntNum()) +
          std::pow(10, snr_per_ue[ue_idx] / 10.f) *
              channel_covariance_matrices_.slice(ue_idx * num_prbs_ + prb);
      capacities_per_ue_[ue_idx * num_prbs_ + prb] =
          std::log2f(std::abs(arma::det(capacity_matrix)));
    }
  }
  return capacities_per_ue_;
}

void ProportionalFairness::Schedule(size_t frame,
                                    const std::vector<float>& ues_capacity) {
  size_t gp = frame % num_groups_;
  arma::vec pf(num_groups_ * num_prbs_, arma::fill::zeros);
  float max_pf = 0;
  std::memset(schedule_buffer_[gp], 0,
              num_prbs_ * cfg_->UeAntNum() * sizeof(size_t));
  for (size_t prb = 0; prb < num_prbs_; prb++) {
    for (size_t action = 0; action < num_groups_; action++) {
      std::vector<size_t> selected_ues = groups_vector_[action];
      if (frame > 0) {
        for (const auto& ue_idx : selected_ues) {
          float tp_history = 0;
          if (ues_flags_[ue_idx]) {
            tp_history = kLamda * pf_ues_history_[ue_idx] / frame +
                         (1 - kLamda) * last_se_[ue_idx];
          } else {
            tp_history = kLamda * pf_ues_history_[ue_idx] / frame;
          }
          pf[action + prb * num_groups_] +=
              ues_capacity[ue_idx * num_prbs_ + prb] / tp_history;
          if (pf[action + prb * num_groups_] >= max_pf) {
            max_pf = pf[action * prb * num_groups_];
            selected_group_vec_.at(prb) = action;
          }
        }
      }
    }
    auto selected_ues = groups_vector_[selected_group_vec_.at(prb)];
    for (size_t i = 0; i < selected_ues.size(); i++) {
      schedule_buffer_index_[gp][i + cfg_->SpatialStreamsNum() + prb] =
          selected_ues.at(i);
      schedule_buffer_[gp][selected_ues.at(i) + cfg_->UeAntNum() + prb] = 1;
    }
  }
}

void ProportionalFairness::UpdatePF(size_t /*frame*/,
                                    const std::vector<float>& ues_capacity) {
  for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
    ues_flags_[ue] = false;
    last_se_[ue] = 0.0F;
    for (size_t prb = 0; prb < num_prbs_; prb++) {
      auto selected_ues = groups_vector_[selected_group_vec_.at(prb)];
      if (std::find(selected_ues.begin(), selected_ues.end(), ue) !=
          selected_ues.end()) {
        ues_flags_[ue] = true;
        last_se_[ue] += ues_capacity[ue * num_prbs_ + prb];
        // TODO: history should be based on actualy received rate
        pf_ues_history_[ue] += ues_capacity[ue * num_prbs_ + prb];
      }
    }
  }
}

bool ProportionalFairness::IsUeScheduled(size_t frame_id, size_t prb_id,
                                         size_t ue_id) {
  size_t gp = frame_id % num_groups_;
  return (schedule_buffer_[gp][ue_id + cfg_->UeAntNum() * prb_id] != 0);
}

arma::uvec ProportionalFairness::ScheduledUeMap(size_t frame_id,
                                                size_t prb_id) {
  size_t gp = frame_id % num_groups_;
  return arma::uvec(reinterpret_cast<unsigned long long*>(
                        &schedule_buffer_[gp][cfg_->UeAntNum() * prb_id]),
                    cfg_->UeAntNum(), false);
}

arma::uvec ProportionalFairness::ScheduledUeList(size_t frame_id,
                                                 size_t prb_id) {
  size_t gp = frame_id % num_groups_;
  return sort(arma::uvec(
      reinterpret_cast<unsigned long long*>(
          &schedule_buffer_index_[gp][cfg_->SpatialStreamsNum() * prb_id]),
      cfg_->SpatialStreamsNum(), false));
}
