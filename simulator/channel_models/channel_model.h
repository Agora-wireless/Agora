#ifndef CHANNEL_MODEL_H
#define CHANNEL_MODEL_H

#include "armadillo"
#include "config.h"

class ChannelModel {
 protected:
  size_t bss_num_;
  size_t ues_num_;
  size_t n_samps_;

 public:
  enum FadingType { kFlat, kSelective };

  ChannelModel(const Config* config, FadingType fading_type_) {
    bss_num_ = config->BsAntNum();
    ues_num_ = config->UeAntNum();
    n_samps_ = config->SampsPerSymbol();

    fading_type = fading_type_;
  }
  virtual ~ChannelModel() = default;

  FadingType fading_type;

  //H Matrix, MUST be of size UEs x BSs
  arma::cx_fmat h_flat_ = arma::cx_fmat();

  // Vector MUST be of size NSubcarriers or OFDMSamples
  std::vector<arma::cx_fmat> h_selective_ = std::vector<arma::cx_fmat>();

  //Function called every frame
  virtual void UpdateModel() {}

  /*
  * Returns H Matrix, if selective fading apply h_slice_index for each subcarrier
  * @param h_matrix_index  -1 if flat fading, subcarrier index if selective fading
  */
  virtual arma::cx_fmat GetMatrix(bool is_downlink, int h_matrix_index = -1) {
    //Check if its flat fading
    if (h_matrix_index == -1) {
      return GetMatrixByPathway(is_downlink, h_flat_);
    }

    return GetMatrixByPathway(is_downlink, h_selective_[h_matrix_index]);
  }

  //Returns Simple Transposed Target Matrix if Downlink or Target Matrix if Uplink.
  arma::cx_fmat GetMatrixByPathway(bool is_downlink,
                                   const arma::cx_fmat& matrix_target) {
    if (is_downlink) {
      return matrix_target.st();
    }

    return matrix_target;
  }
};

#endif
