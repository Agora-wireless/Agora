#include "custom_schedule.h"

#include "logger.h"

CustomSchedule::CustomSchedule(Config* const cfg) : SchedulerModel(cfg) {
  num_groups_ = cfg_->FramesToTest();
  adapt_ues_array_.resize(num_groups_);
  const std::string directory =
      TOSTRING(PROJECT_DIRECTORY) "/files/experiment/";
  static const std::string kFilename =
      directory + "adapt_ueant" + std::to_string(cfg_->UeAntNum()) + ".bin";
  AGORA_LOG_INFO(
      "Agora: Reading adaptable number of UEs across frames from %s\n",
      kFilename.c_str());

  FILE* fp = std::fopen(kFilename.c_str(), "rb");
  RtAssert(fp != nullptr, "Failed to open adapt UEs file");

  const size_t expected_count = cfg_->FramesToTest();
  const size_t actual_count =
      std::fread(&adapt_ues_array_.at(0), sizeof(uint8_t), expected_count, fp);

  if (expected_count != actual_count) {
    std::fprintf(stderr,
                 "Agora: Failed to read adapt UEs file %s. expected "
                 "%zu number of UE entries but read %zu. Errno %s\n",
                 kFilename.c_str(), expected_count, actual_count,
                 strerror(errno));
    throw std::runtime_error("Agora: Failed to read adapt UEs file");
  }
  schedule_buffer_.Calloc(num_groups_, cfg_->UeAntNum() * cfg_->OfdmDataNum(),
                          Agora_memory::Alignment_t::kAlign64);
  schedule_buffer_index_.Calloc(num_groups_,
                                cfg_->UeAntNum() * cfg_->OfdmDataNum(),
                                Agora_memory::Alignment_t::kAlign64);
  for (size_t gp = 0u; gp < num_groups_; gp++) {
    for (size_t sc = 0; sc < cfg_->OfdmDataNum(); sc++) {
      for (size_t ue = 0; ue < adapt_ues_array_.at(gp); ue++) {
        schedule_buffer_[gp][ue + cfg_->UeAntNum() * sc] = 1;
        schedule_buffer_index_[gp][ue + cfg_->UeAntNum() * sc] = ue;
      }
    }
  }
}

bool CustomSchedule::IsUeScheduled(size_t frame_id, size_t sc_id,
                                   size_t ue_id) {
  return (schedule_buffer_[frame_id][ue_id + cfg_->UeAntNum() * sc_id] != 0);
}

arma::uvec CustomSchedule::ScheduledUeMap(size_t frame_id, size_t sc_id) {
  return arma::uvec(reinterpret_cast<unsigned long long*>(
                        &schedule_buffer_[frame_id][cfg_->UeAntNum() * sc_id]),
                    cfg_->UeAntNum(), false);
}

arma::uvec CustomSchedule::ScheduledUeList(size_t frame_id, size_t sc_id) {
  return arma::uvec(
      reinterpret_cast<unsigned long long*>(
          &schedule_buffer_index_[frame_id][cfg_->UeAntNum() * sc_id]),
      adapt_ues_array_.at(frame_id), false);
}
