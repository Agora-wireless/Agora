/**
 * @file proportional_fairness.cc
 * @brief Implementation file for the Proportional Fairness scheduling algorithm 
 */
#include "proportional_fairness.h"

#include "logger.h"

static constexpr bool kPrintSchedulingBuffers = false;
static constexpr bool kPrintSchedulingGroups = true;
static constexpr bool kPrintSelectedGroup = true;

static constexpr size_t kCalibFrames = 50;

void ProportionalFairness::Combination(int k, int offset = 0) {
  if (k == 0) {
    groups_vector.push_back(combination);
    return;
  }
  for (size_t i = offset; i <= ues_vector.size() - k; ++i) {
    combination.push_back(ues_vector[i]);
    Combination(k - 1, i + 1);
    combination.pop_back();
  }
}

ProportionalFairness::ProportionalFairness(const size_t spatial_streams,
                                           const size_t bss_num,
                                           const size_t ues_num,
                                           size_t ofdm_data_num_)
    : selected_group_(0),
      spatial_streams_(spatial_streams),
      bss_num_(bss_num),
      ues_num_(ues_num),
      lamda_(0.5F),
      last_SE_(arma::zeros(ues_num)),
      capacities_per_ue_(ues_num_),
      channel_covariance_matrices_(bss_num_, bss_num_, ues_num_,
                                   arma::fill::zeros) {
  std::stringstream ss;

  //Define UE Index vector
  for (size_t ue_idx = 0; ue_idx < ues_num_; ue_idx++) {
    ues_vector.push_back(ue_idx);
    ues_flags_.push_back(false);
    pf_ues_history.push_back(0.01F);
  }

  //Possible scheduling options
  Combination(spatial_streams_);
  num_groups_ = groups_vector.size();

  schedule_buffer_.Calloc(num_groups_, ues_num_ * ofdm_data_num_,
                          Agora_memory::Alignment_t::kAlign64);
  schedule_buffer_index_.Calloc(num_groups_, spatial_streams_ * ofdm_data_num_,
                                Agora_memory::Alignment_t::kAlign64);

  //Schedule Buffer Process
  for (size_t gp = 0; gp < num_groups_; gp++) {
    for (size_t sc = 0; sc < ofdm_data_num_; sc++) {
      std::vector<size_t> UEs_idx = groups_vector[gp];
      for (size_t ue_idx = 0; ue_idx < spatial_streams_; ue_idx++) {
        schedule_buffer_[gp][UEs_idx[ue_idx] + ues_num_ * sc] = 1;
      }
      for (size_t ue = gp; ue < gp + spatial_streams_; ue++) {
        schedule_buffer_index_[gp][(ue - gp) + spatial_streams_ * sc] =
            UEs_idx[ue - gp];
      }
    }
  }

  if (kPrintSchedulingBuffers) {
    for (size_t row = 0; row < num_groups_; row++) {
      ss << "schedule_PF_index_buffer_" << row << " \n\n";
      for (size_t col = 0; col < spatial_streams_ * ofdm_data_num_; col++) {
        ss << schedule_buffer_index_[row][col] << " ";
      }
      ss << "\n\n";
      ss << "schedule_PF_SC_buffer_" << row << " \n\n";
      for (size_t col = 0; col < spatial_streams_ * ofdm_data_num_; col++) {
        ss << schedule_buffer_[row][col] << " ";
      }
      ss << "\n\n";
    }
  }
  if (kPrintSchedulingGroups) {
    ss << "Proportional Fairness Scheduling Groups \n\n";
    for (size_t gp = 0; gp < num_groups_; gp++) {
      ss << "Group " << gp << " = [ ";
      for (size_t ue = 0; ue < ues_num_; ue++) {
        ss << schedule_buffer_[gp][ue] << " ";
      }
      ss << "]\n\n";
    }
  }
  AGORA_LOG_INFO(ss.str());
}

void ProportionalFairness::Update(size_t frame_id, arma::cx_fmat csi,
                                  std::vector<float> snr_per_ue) {
  std::vector<float> ues_capacity = UEsCapacity(csi, snr_per_ue);
  UpdateScheduler(frame_id, ues_capacity);
}

size_t ProportionalFairness::UpdateScheduler(size_t frame_id,
                                             std::vector<float> ues_capacity) {
  if (frame_id < kCalibFrames) {
    selected_group_ = frame_id % num_groups_;
    if (kPrintSelectedGroup) {
      std::stringstream s_;
      s_ << "Scheduled PF Frame " << frame_id << " Calibration " << frame_id
         << "/" << kCalibFrames << "\n";
      AGORA_LOG_INFO(s_.str());
    }
  } else if (current_frame != frame_id) {
    current_frame = frame_id;
    Schedule(frame_id, ues_capacity);
    UpdatePF(frame_id, ues_capacity);
    if (kPrintSelectedGroup) {
      std::stringstream s_;
      s_ << "Scheduled PF Frame " << frame_id << " Group " << selected_group_
         << " = [ ";
      for (size_t i = 0; i < ues_num_; i++) {
        std::string a = (ues_flags_[i]) ? "1 " : "0 ";
        s_ << a;
      }
      s_ << "] \n ";
      AGORA_LOG_INFO(s_.str());
    }
  }
  return selected_group_;
}

std::vector<float> ProportionalFairness::UEsCapacity(
    arma::cx_fmat csi, std::vector<float> snr_per_ue) {
  // Calculate the channel covariance matrices for each UE
  std::vector<size_t> selected_ues = groups_vector[selected_group_];
  for (size_t ue = 0; ue < spatial_streams_; ue++) {
    size_t ue_idx = selected_ues[ue];
    channel_covariance_matrices_.slice(ue_idx) = csi.col(ue) * csi.col(ue).t();
  }
  for (size_t ue = 0; ue < spatial_streams_; ue++) {
    size_t ue_idx = selected_ues[ue];
    arma::cx_fmat capacity_matrix =
        arma::eye<arma::cx_fmat>(bss_num_, bss_num_) +
        snr_per_ue[ue_idx] * channel_covariance_matrices_.slice(ue_idx);
    capacities_per_ue_[ue_idx] =
        std::log2f(std::abs(arma::det(capacity_matrix)));
  }
  return capacities_per_ue_;
}

void ProportionalFairness::Schedule(size_t frame,
                                    std::vector<float> ues_capacity) {
  arma::vec pf_(num_groups_, arma::fill::zeros);
  float max_pf = 0;
  for (size_t action = 0; action < num_groups_; action++) {
    std::vector<size_t> selected_ues = groups_vector[action];
    if (frame > 0) {
      for (size_t i = 0; i < selected_ues.size(); i++) {
        float tp_history = 0;
        size_t ue_idx = selected_ues[i];
        if (ues_flags_[ue_idx]) {
          tp_history = lamda_ * pf_ues_history[ue_idx] / frame +
                       (1 - lamda_) * last_SE_[ue_idx];
        } else {
          tp_history = lamda_ * pf_ues_history[ue_idx] / frame;
        }
        pf_[action] += ues_capacity[ue_idx] / tp_history;
        if (pf_[action] >= max_pf) {
          max_pf = pf_[action];
          selected_group_ = action;
        }
      }
    }
  }
}

void ProportionalFairness::UpdatePF(size_t frame,
                                    std::vector<float> ues_capacity) {
  for (size_t i = 0; i < groups_vector[selected_group_].size(); i++) {
    size_t idx_ = groups_vector[selected_group_][i];
    pf_ues_history[idx_] += ues_capacity[idx_];
  }
  for (size_t ue = 0; ue < ues_num_; ue++) {
    if (std::find(groups_vector[selected_group_].begin(),
                  groups_vector[selected_group_].end(),
                  ue) != groups_vector[selected_group_].end()) {
      ues_flags_[ue] = true;
      last_SE_[ue] = ues_capacity[ue];
    } else {
      ues_flags_[ue] = false;
      last_SE_[ue] = 0.0F;
    }
  }
}