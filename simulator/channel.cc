/**
 * @file channel.h
 * @brief Implementation file for the channel class
 */
#include "channel.h"

#include <utility>

static constexpr bool kPrintChannelOutput = false;
static constexpr bool kPrintSNRCheck = false;

Channel::Channel(const Config* const config, std::string& in_channel_type,
                 double in_channel_snr)
    : cfg_(config),
      sim_chan_model_(std::move(in_channel_type)),
      channel_snr_db_(in_channel_snr) {
  bs_ant_ = cfg_->BsAntNum();
  ue_ant_ = cfg_->UeAntNum();
  n_samps_ = cfg_->SampsPerSymbol();

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

Channel::~Channel() = default;

void Channel::ApplyChan(const arma::cx_fmat& fmat_src, arma::cx_fmat& fmat_dst,
                        const bool is_downlink, const bool is_newChan) {
  arma::cx_fmat fmat_h;

  if (is_newChan) {
    switch (chan_model_) {
      case kAwgn: {
        arma::fmat rmat(ue_ant_, bs_ant_, arma::fill::ones);
        arma::fmat imat(ue_ant_, bs_ant_, arma::fill::zeros);
        h_ = arma::cx_fmat(rmat, imat);
        // H = H / abs(H).max();
      } break;

      case kRayleigh:
        // Simple Uncorrelated Rayleigh Channel - Flat fading (single tap)
        {
          arma::fmat rmat(ue_ant_, bs_ant_, arma::fill::randn);
          arma::fmat imat(ue_ant_, bs_ant_, arma::fill::randn);
          h_ = arma::cx_fmat(rmat, imat);
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
    fmat_h = fmat_src * h_.st() /
             (kDebugDownlink == true ? 1 : std::sqrt(cfg_->BsAntNum()));
  } else {
    fmat_h = fmat_src * h_;
  }

  // Add noise
  Awgn(fmat_h, fmat_dst);

  if (kPrintChannelOutput) {
    Utils::PrintMat(h_, "H");
  }
}

void Channel::Awgn(const arma::cx_fmat& src, arma::cx_fmat& dst) const {
  if (channel_snr_db_ < 120.0f) {
    const int n_row = src.n_rows;
    const int n_col = src.n_cols;
    float snr_lin = pow(10, channel_snr_db_ / 10);

    // Power spectral density of noise
    arma::fmat src_sq = square(abs(src));
    // pwr = sum(abs(samps)ˆ2)/length(samps)
    arma::frowvec pwr_vec = mean(src_sq, 0);
    arma::frowvec n0 = pwr_vec / snr_lin;
    arma::frowvec n = sqrt(n0 / 2);

    // Generate noise
    arma::cx_fmat noise(arma::randn<arma::fmat>(n_row, n_col),
                        arma::randn<arma::fmat>(n_row, n_col));
    // Supposed to be faster
    // arma::fmat x(n_row, n_col, arma::fill::arma::randn);
    // arma::fmat y(n_row, n_col, arma::fill::arma::randn);
    // arma::cx_fmat noise = arma::cx_fmat(x, y);

    arma::fmat n_mat = repmat(n, n_row, 1);
    // Element-wise multiplication
    noise = noise % n_mat;
    // Add noise to signal
    dst = src + noise;

    // Check SNR
    if (kPrintSNRCheck) {
      arma::fmat noise_sq = square(abs(noise));
      arma::frowvec noise_vec = mean(noise_sq, 0);
      arma::frowvec snr = 10 * log10(pwr_vec / noise_vec);
      std::cout << "SNR: " << snr;
    }
  } else {
    dst = src;
  }
}

void Channel::Lte3gpp(const arma::cx_fmat& fmat_src, arma::cx_fmat& fmat_dst) {
  // TODO - In progress (Use Rayleigh for now...)
  arma::cx_fmat h(arma::randn<arma::fmat>(cfg_->UeAntNum(), cfg_->BsAntNum()),
                  arma::randn<arma::fmat>(cfg_->UeAntNum(), cfg_->BsAntNum()));
  h = (1 / sqrt(2)) * h;
  fmat_dst = fmat_src * h;
}
