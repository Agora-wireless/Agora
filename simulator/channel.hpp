#ifndef SIM_CHAN_MODEL
#define SIM_CHAN_MODEL

#include <assert.h>
#include <math.h>

#include <algorithm>
#include <armadillo>
#include <ctime>
#include <iomanip>
#include <numeric>

#include "Symbols.hpp"
#include "buffer.hpp"
#include "config.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "signalHandler.hpp"
#include "utils.h"

using namespace arma;

class Channel {
 public:
  Channel(Config* bscfg, Config* uecfg, std::string channel_type,
          double channel_snr);
  ~Channel();

  // Dimensions of fmat_src: ( bscfg->sampsPerSymbol, uecfg->UE_ANT_NUM )
  void apply_chan(const cx_fmat& fmat_src, cx_fmat& mat_dst,
                  const bool is_downlink, const bool is_newChan);

  // Additive White Gaussian Noise. Dimensions of src: ( bscfg->sampsPerSymbol,
  // uecfg->UE_ANT_NUM )
  void awgn(const cx_fmat& fmat_src, cx_fmat& fmat_dst);

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
  void lte_3gpp(const cx_fmat& fmat_src, cx_fmat& fmat_dst);

 private:
  Config* bscfg;
  Config* uecfg;

  Channel* channel;
  size_t bs_ant;
  size_t ue_ant;
  size_t n_samps;

  std::string sim_chan_model;
  double channel_snr_db;
  enum ChanModel { AWGN, RAYLEIGH, RAN_3GPP } chan_model;

  cx_fmat H;
};

#endif /* SIM_CHAN_MODEL */
