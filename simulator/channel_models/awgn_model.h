/**
 * @file awgn_model.h
 * @brief Declaration file for the channel simulator class
 */
#ifndef AWGN_MODEL_H_
#define AWGN_MODEL_H_

#include "armadillo"
#include "channel_model.h"

class AwgnModel : public ChannelModel {
 public:
  AwgnModel(size_t bs_ant_num, size_t ue_ant_num, size_t samples_per_sym)
      : ChannelModel(bs_ant_num, ue_ant_num, samples_per_sym,
                     ChannelModel::kFlat) {}

  void UpdateModel() final {
    arma::fmat rmat(ues_num_, bss_num_, arma::fill::ones);
    arma::fmat imat(ues_num_, bss_num_, arma::fill::ones);

    h_flat_ = arma::cx_fmat(rmat, imat);
  }
};

#endif  //AWGN_MODEL_H_