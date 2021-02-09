#include "channel.h"

static constexpr bool kPrintChannelOutput = false;
using namespace arma;

Channel::Channel(Config* config_bs, Config* config_ue,
                 std::string in_channel_type, double in_channel_snr)
    : bscfg_(config_bs),
      uecfg_(config_ue),
      sim_chan_model_(in_channel_type),
      channel_snr_db_(in_channel_snr) {
  bs_ant_ = bscfg_->bs_ant_num_;
  ue_ant_ = uecfg_->ue_ant_num_;
  n_samps_ = bscfg_->samps_per_symbol_;

  if (sim_chan_model_ == "AWGN") {
    chan_model_ = kAwgn;
  } else if (sim_chan_model_ == "RAYLEIGH") {
    chan_model_ = kRayleigh;
  } else if (sim_chan_model_ == "RAN_3GPP") {
    chan_model_ = kRan3Gpp;
    printf("3GPP Model in progress, setting to RAYLEIGH channel \n");
    chan_model_ = kRayleigh;
  } else {
    chan_model_ = kAwgn;
  }
}

Channel::~Channel() {}

void Channel::ApplyChan(const cx_fmat& fmat_src, cx_fmat& fmat_dst,
                        const bool is_downlink, const bool is_newChan) {
  cx_fmat fmat_h;

  if (is_newChan) {
    switch (chan_model_) {
      case kAwgn: {
        fmat rmat(ue_ant_, bs_ant_, fill::ones);
        fmat imat(ue_ant_, bs_ant_, fill::zeros);
        h_ = cx_fmat(rmat, imat);
        // H = H / abs(H).max();
      } break;

      case kRayleigh:
        // Simple Uncorrelated Rayleigh Channel - Flat fading (single tap)
        {
          fmat rmat(ue_ant_, bs_ant_, fill::randn);
          fmat imat(ue_ant_, bs_ant_, fill::randn);
          h_ = cx_fmat(rmat, imat);
          h_ = (1 / sqrt(2)) * h_;
          // H = H / abs(H).max();
        }
        break;

      case kRan3Gpp:
        Lte3gpp(fmat_src, fmat_dst);
        break;
    }
  }
  if (is_downlink) {
    fmat_h = fmat_src * h_.st() / std::sqrt(bscfg_->bs_ant_num_);
  } else {
    fmat_h = fmat_src * h_;
  }

  // Add noise
  Awgn(fmat_h, fmat_dst);

  if (kPrintChannelOutput) {
    Utils::PrintMat(h_, "H");
  }
}

void Channel::Awgn(const cx_fmat& src, cx_fmat& dst) const {
  int n_row = src.n_rows;
  int n_col = src.n_cols;
  float snr_lin = pow(10, channel_snr_db_ / 10);

  // Power spectral density of noise
  fmat src_sq = square(abs(src));
  // pwr = sum(abs(samps)ˆ2)/length(samps)
  frowvec pwr_vec = mean(src_sq, 0);
  frowvec n0 = pwr_vec / snr_lin;
  frowvec n = sqrt(n0 / 2);

  // Generate noise
  cx_fmat noise(randn<fmat>(n_row, n_col), randn<fmat>(n_row, n_col));
  // Supposed to be faster
  // fmat x(n_row, n_col, fill::randn);
  // fmat y(n_row, n_col, fill::randn);
  // cx_fmat noise = cx_fmat(x, y);

  fmat n_mat = repmat(n, n_row, 1);
  // Element-wise multiplication
  noise = noise % n_mat;
  // Add noise to signal
  dst = src + noise;

  // Check SNR
  /*
  fmat noise_sq = square(abs(noise));
  frowvec noise_vec = mean(noise_sq, 0);
  frowvec snr = 10 * log10(pwr_vec / noise_vec);
  printf("SNR: ");
  std::cout << snr << std::endl;
  */
}

void Channel::Lte3gpp(const cx_fmat& fmat_src, cx_fmat& fmat_dst) {
  // TODO - In progress (Use Rayleigh for now...)
  cx_fmat h(randn<fmat>(uecfg_->ue_ant_num_, bscfg_->bs_ant_num_),
            randn<fmat>(uecfg_->ue_ant_num_, bscfg_->bs_ant_num_));
  h = (1 / sqrt(2)) * h;
  fmat_dst = fmat_src * h;
}
