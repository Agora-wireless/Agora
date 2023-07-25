/**
 * @file channel_model.h
 * @brief Declaration file for the channel model API
 */
#ifndef CHANNEL_MODEL_H_
#define CHANNEL_MODEL_H_

#include <cstddef>
#include <memory>
#include <vector>

#include "armadillo"
#include "config.h"

class ChannelModel {
 public:
  enum FadingType { kFlat, kSelective };

  ChannelModel(size_t bs_ant_num, size_t ue_ant_num, size_t samples_per_sym,
               FadingType fading_type);
  virtual ~ChannelModel() = default;

  //Function called every frame
  virtual void UpdateModel() = 0;

  /*
  * Returns H Matrix, if selective fading apply h_slice_index for each subcarrier
  * @param h_matrix_index  -1 if flat fading, subcarrier index if selective fading
  */
  virtual arma::cx_fmat GetMatrix(bool is_downlink, int h_matrix_index = -1);

  //Returns Simple Transposed Target Matrix if Downlink or Target Matrix if Uplink.
  arma::cx_fmat GetMatrixByPathway(bool is_downlink,
                                   const arma::cx_fmat& matrix_target);

  inline FadingType GetFadingType() const { return fading_type_; }
  //Factory function
  static std::unique_ptr<ChannelModel> CreateChannelModel(
      const Config* const config, std::string& channel_type,
      std::string& dataset_path);

 protected:
  const size_t bss_num_;
  const size_t ues_num_;
  const size_t n_samps_;

  //H Matrix, MUST be of size UEs x BSs
  arma::cx_fmat h_flat_;
  // Vector MUST be of size NSubcarriers or OFDMSamples
  //Not sure this makes sense for all models?
  std::vector<arma::cx_fmat> h_selective_;

 private:
  FadingType fading_type_;
};

#endif  //CHANNEL_MODEL_H_
