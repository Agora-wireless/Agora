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
          double channel_snr);
  ~Channel();

  // Dimensions of fmat_src: ( bscfg->sampsPerSymbol, uecfg->UE_ANT_NUM )
  void ApplyChan(const arma::cx_fmat& fmat_src, arma::cx_fmat& mat_dst,
                 const bool is_downlink, const bool is_newChan);

  // Additive White Gaussian Noise. Dimensions of src: ( bscfg->sampsPerSymbol,
  // uecfg->UE_ANT_NUM )
  void Awgn(const arma::cx_fmat& fmat_src, arma::cx_fmat& fmat_dst) const;

  /*
   * From "Study on 3D-channel model for Elevation Beamforming
   *       and FD-MIMO studies for LTE"
   *       3GPP TR 36.873 V12.7.0 (2017-12)
   * Note: FD-MIMO Stands for Full-Dimension MIMO
   * Currenlty implements the 3D-UMa scenario only. This
   * corresponds to an Urban Macro cell with high UE density
   * indoor and outdoor. This scenario assumes Base Stations
   * are above surrounding buildings.
   *
   */
  void Lte3gpp(const arma::cx_fmat& fmat_src, arma::cx_fmat& fmat_dst);

 private:
  const Config* const cfg_;

  Channel* channel_;
  size_t bs_ant_;
  size_t ue_ant_;
  size_t n_samps_;

  std::string sim_chan_model_;
  double channel_snr_db_;
  double noise_samp_std_;
  enum ChanModel { kAwgn, kRayleigh, kRan3Gpp } chan_model_;

  arma::cx_fmat h_;
};

#endif  // CHANNEL_H_
