/**
 * @file channel_model.cc
 * @brief Defination file for the generic channel model.
 */
#include "channel_model.h"

//All the concrete channel models
#include "awgn_model.h"
#include "channel_model.h"
#include "dataset_model.h"
#include "logger.h"
#include "rayleigh_model.h"

///Factory function
std::unique_ptr<ChannelModel> ChannelModel::CreateChannelModel(
    const Config* const config, std::string& channel_type,
    std::string& dataset_path) {
  if (channel_type == "AWGN") {
    return std::make_unique<AwgnModel>(config->BsAntNum(), config->UeAntNum(),
                                       config->SampsPerSymbol());
  } else if (channel_type == "RAYLEIGH") {
    return std::make_unique<RayleighModel>(
        config->BsAntNum(), config->UeAntNum(), config->SampsPerSymbol());
#if defined(ENABLE_HDF5)
  } else if (channel_type == "DATASET") {
    return std::make_unique<DatasetModel>(
        config->BsAntNum(), config->UeAntNum(), config->SampsPerSymbol(),
        dataset_path);
#endif
  } else {
    AGORA_LOG_WARN(
        "Invalid channel model (%s) at CHSim, assuming AWGN Model... \n",
        channel_type.c_str());
    return std::make_unique<AwgnModel>(config->BsAntNum(), config->UeAntNum(),
                                       config->SampsPerSymbol());
  }
}

ChannelModel::ChannelModel(size_t bs_ant_num, size_t ue_ant_num,
                           size_t samples_per_sym, FadingType fading_type)
    : bss_num_(bs_ant_num),
      ues_num_(ue_ant_num),
      n_samps_(samples_per_sym),
      h_flat_(),
      h_selective_(),
      fading_type_(fading_type) {}

/*
  * Returns H Matrix, if selective fading apply h_slice_index for each subcarrier
  * @param h_matrix_index  -1 if flat fading, subcarrier index if selective fading
  */
arma::cx_fmat ChannelModel::GetMatrix(bool is_downlink, int h_matrix_index) {
  //Check if its flat fading
  if (h_matrix_index == -1) {
    return GetMatrixByPathway(is_downlink, h_flat_);
  } else {
    return GetMatrixByPathway(is_downlink, h_selective_[h_matrix_index]);
  }
}

//Returns Simple Transposed Target Matrix if Downlink or Target Matrix if Uplink.
arma::cx_fmat ChannelModel::GetMatrixByPathway(
    bool is_downlink, const arma::cx_fmat& matrix_target) {
  if (is_downlink) {
    return matrix_target.st();
  } else {
    return matrix_target;
  }
}