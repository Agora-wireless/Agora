/**
 * @file channel.h
 * @brief Declaration file for the channel class
 */
#ifndef CHANNEL_H_
#define CHANNEL_H_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <ctime>
#include <iomanip>
#include <numeric>

#include "armadillo"
#include "channel_models/awgn_model.h"
#include "channel_models/channel_model.h"
#include "channel_models/dataset_model.h"
#include "channel_models/rayleigh_model.h"
#include "config.h"
#include "gettime.h"
#include "memory_manage.h"
#include "message.h"
#include "signal_handler.h"
#include "symbols.h"
#include "utils.h"

class Channel {
 public:
  Channel(const Config* const config, std::string& channel_type,
          double channel_snr, std::string& dataset_path);
  ~Channel();

  // Dimensions of fmat_src: ( bscfg->sampsPerSymbol, uecfg->UE_ANT_NUM )
  void ApplyChan(const arma::cx_fmat& fmat_src, arma::cx_fmat& mat_dst,
                 const bool is_downlink, const bool is_newChan);

  // Additive White Gaussian Noise. Dimensions of src: ( bscfg->sampsPerSymbol,
  // uecfg->UE_ANT_NUM )
  void Awgn(const arma::cx_fmat& fmat_src, arma::cx_fmat& fmat_dst) const;

 private:
  const Config* const cfg_;

  Channel* channel_;

  std::string sim_chan_model_;
  double channel_snr_db_;
  double noise_samp_std_;

  std::string dataset_path_;

  ChannelModel* channel_model;
  ChannelModel* GetChannelModel();
};

#endif  // CHANNEL_H_