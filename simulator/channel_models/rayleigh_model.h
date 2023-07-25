/**
 * @file rayleigh_model.h
 * @brief Declaration file for the rayleigh channel model
 */
#ifndef RAYLEIGH_MODEL_H_
#define RAYLEIGH_MODEL_H_

#include "armadillo"
#include "channel_model.h"

//Flat fading Rayleigh channel model
class RayleighModel : public ChannelModel {
 public:
  RayleighModel(size_t bs_ant_num, size_t ue_ant_num, size_t samples_per_sym)
      : ChannelModel(bs_ant_num, ue_ant_num, samples_per_sym,
                     ChannelModel::kFlat) {}

  void UpdateModel() final {
    arma::fmat rmat(ues_num_, bss_num_, arma::fill::randn);
    arma::fmat imat(ues_num_, bss_num_, arma::fill::randn);

    h_flat_ = arma::cx_fmat(rmat, imat);
  }
};

#endif  // RAYLEIGH_MODEL_H_