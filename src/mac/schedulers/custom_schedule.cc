#include "custom_schedule.h"

#include "data_generator.h"
#include "logger.h"
#include "utils.h"

CustomSchedule::CustomSchedule(Config* const cfg) : SchedulerModel(cfg) {
  size_t n_items = cfg->FramesToTest() * cfg_->UeAntNum();
  ue_map_array_.resize(n_items);
  ue_num_array_.resize(cfg->FramesToTest());
  const std::string directory =
      TOSTRING(PROJECT_DIRECTORY) "/files/experiment/";
  static const std::string kFilename =
      directory + kUeSchedulePrefix + std::to_string(cfg_->UeAntNum());
  AGORA_LOG_INFO(
      "Custom MAC Scheduler: Reading scheduled map of UEs across frames "
      "from %s\n",
      std::string(kFilename + "ue.bin").c_str());
  Utils::ReadBinaryFile(kFilename + "ue.bin", sizeof(uint8_t), n_items, 0,
                        ue_map_array_.data());
  std::vector<uint8_t> ul_mcs(cfg->FramesToTest(), 0);
  Utils::ReadBinaryFile(kFilename + "ue_ul_mcs.bin", sizeof(uint8_t),
                        cfg->FramesToTest(), 0, ul_mcs.data());
  std::vector<uint8_t> dl_mcs(cfg->FramesToTest(), 0);
  Utils::ReadBinaryFile(kFilename + "ue_dl_mcs.bin", sizeof(uint8_t),
                        cfg->FramesToTest(), 0, dl_mcs.data());
  std::vector<size_t> sched_id_vec;
  for (size_t fn = 0u; fn < cfg->FramesToTest(); fn++) {
    size_t ue_sched_id = 0;
    for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
      uint8_t sched_bit = ue_map_array_.at(fn * cfg_->UeAntNum() + ue);
      if (sched_bit == 1) {
        ue_num_array_.at(fn)++;  // count no. of scheduled UEs in this frame
      }
      ue_sched_id += static_cast<size_t>(sched_bit * std::pow(2, ue));
    }
    sched_id_vec.push_back(ue_sched_id);
    if (ue_sched_set_.empty()) {
      ue_sched_set_.push_back(ue_sched_id);
    } else {
      std::vector<size_t>::iterator it;
      for (it = ue_sched_set_.begin(); it < ue_sched_set_.end(); it++) {
        if (ue_sched_id == *it) {  // dont's push this to keep vector unique
          break;
        } else if (ue_sched_id > *it && (it + 1) == ue_sched_set_.end()) {
          ue_sched_set_.push_back(ue_sched_id);
          break;
        } else if (ue_sched_id < *it && it == ue_sched_set_.begin()) {
          ue_sched_set_.insert(it, ue_sched_id);
          break;
        } else if (ue_sched_id > *it && ue_sched_id < *(it + 1)) {
          ue_sched_set_.insert(it + 1, ue_sched_id);
          break;
        }
      }
    }
  }
  num_groups_ = ue_sched_set_.size();
  for (size_t fn = 0u; fn < cfg->FramesToTest(); fn++) {
    sched_id_array_.push_back(this->UeScheduleIndex(sched_id_vec.at(fn)));
  }
  schedule_buffer_.Calloc(num_groups_, cfg_->UeAntNum() * cfg_->OfdmDataNum(),
                          Agora_memory::Alignment_t::kAlign64);
  schedule_buffer_index_.Calloc(num_groups_,
                                cfg_->UeAntNum() * cfg_->OfdmDataNum(),
                                Agora_memory::Alignment_t::kAlign64);
  ul_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  dl_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
  for (size_t gp = 0u; gp < num_groups_; gp++) {
    size_t cnt = 0;
    for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
      uint8_t sched_bit = (ue_sched_set_.at(gp) >> ue) & 1;
      for (size_t sc = 0; sc < cfg_->OfdmDataNum(); sc++) {
        schedule_buffer_[gp][ue + cfg_->UeAntNum() * sc] = sched_bit;
        if (sched_bit == 1) {
          schedule_buffer_index_[gp][cnt + cfg_->UeAntNum() * sc] = ue;
        }
      }
      cnt += sched_bit;
      ul_mcs_buffer_[gp][ue] = ul_mcs.at(gp);
      dl_mcs_buffer_[gp][ue] = dl_mcs.at(gp);
    }
  }
}

bool CustomSchedule::IsUeScheduled(size_t frame_id, size_t sc_id,
                                   size_t ue_id) {
  size_t sched_id = sched_id_array_.at(frame_id);
  return (schedule_buffer_[sched_id][ue_id + cfg_->UeAntNum() * sc_id] != 0);
}

arma::uvec CustomSchedule::ScheduledUeMap(size_t frame_id, size_t sc_id) {
  size_t sched_id = sched_id_array_.at(frame_id);
  return arma::uvec(reinterpret_cast<unsigned long long*>(
                        &schedule_buffer_[sched_id][cfg_->UeAntNum() * sc_id]),
                    cfg_->UeAntNum(), false);
}

arma::uvec CustomSchedule::ScheduledUeList(size_t frame_id, size_t sc_id) {
  size_t sched_id = sched_id_array_.at(frame_id);
  return arma::uvec(
      reinterpret_cast<unsigned long long*>(
          &schedule_buffer_index_[sched_id][cfg_->UeAntNum() * sc_id]),
      ue_num_array_.at(frame_id), false);
}

size_t CustomSchedule::UeScheduleIndex(size_t sched_id) {
  auto it = std::find(ue_sched_set_.begin(), ue_sched_set_.end(), sched_id);
  if (it == ue_sched_set_.end()) {
    return -1;
  }
  return it - ue_sched_set_.begin();
}

void CustomSchedule::Update(size_t frame_id, const arma::cx_fmat& /*csi*/,
                            const std::vector<float>& /*snr_per_ue*/) {
  selected_group_ = sched_id_array_.at(frame_id);
}

size_t CustomSchedule::SelectedUlMcs(size_t frame_id, size_t ue_id) {
  size_t sched_id = sched_id_array_.at(frame_id);
  return ul_mcs_buffer_[sched_id][ue_id];
}

size_t CustomSchedule::SelectedDlMcs(size_t frame_id, size_t ue_id) {
  size_t sched_id = sched_id_array_.at(frame_id);
  return dl_mcs_buffer_[sched_id][ue_id];
}
