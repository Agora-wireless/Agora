/**
 * @file channel.h
 * @brief Implementation file for the channel class
 */
#include "channel.h"

#include "armadillo"
#include "awgn_model.h"
#include "channel_model.h"
#include "dataset_model.h"
#include "logger.h"
#include "rayleigh_model.h"
#include "utils.h"

static constexpr bool kPrintChannelOutput = false;
static constexpr bool kPrintSNRCheck = false;
static constexpr double kMeanChannelGain = 0.1f;

Channel::~Channel() = default;

Channel::Channel(const Config* const config, std::string& in_channel_type,
                 double in_channel_snr, std::string& dataset_path)
    : cfg_(config),
      sim_chan_model_(std::move(in_channel_type)),
      channel_snr_db_(in_channel_snr),
      dataset_path_(dataset_path) {
  channel_model = GetChannelModel();

  const float snr_lin = std::pow(10, channel_snr_db_ / 10.0f);
  noise_samp_std_ = std::sqrt(kMeanChannelGain / (snr_lin * 2.0f));
  std::cout << "Noise level to be used is: " << std::fixed << std::setw(5)
            << std::setprecision(2) << noise_samp_std_ << std::endl;
}

ChannelModel* Channel::GetChannelModel() {
  if (sim_chan_model_ == "AWGN") {
    return new AwgnModel(cfg_->BsAntNum(), cfg_->UeAntNum(),
                         cfg_->SampsPerSymbol());
  } else if (sim_chan_model_ == "RAYLEIGH") {
    return new RayleighModel(cfg_->BsAntNum(), cfg_->UeAntNum(),
                             cfg_->SampsPerSymbol());
#if defined(ENABLE_HDF5)
  } else if (sim_chan_model_ == "DATASET") {
    return new DatasetModel(cfg_->BsAntNum(), cfg_->UeAntNum(),
                            cfg_->SampsPerSymbol(), dataset_path_);
#endif
  } else {
    AGORA_LOG_WARN(
        "Invalid channel model (%s) at CHSim, assuming AWGN Model... \n",
        sim_chan_model_.c_str());
    return new AwgnModel(cfg_->BsAntNum(), cfg_->UeAntNum(),
                         cfg_->SampsPerSymbol());
  }
}

void Channel::ApplyChan(const arma::cx_fmat& fmat_src, arma::cx_fmat& fmat_dst,
                        const bool is_downlink, const bool is_newChan) {
  arma::cx_fmat fmat_h;

  if (is_newChan) {
    channel_model->UpdateModel();
  }

  switch (channel_model->GetFadingType()) {
    case ChannelModel::kFlat: {
      fmat_h = fmat_src * channel_model->GetMatrix(is_downlink);
      break;
    }

    case ChannelModel::kSelective: {
      //For each Subcarrier or OFDMSample input, multiply H Matrix slice
      for (int h_index = 0; h_index < (int)fmat_src.n_rows; h_index++) {
        arma::cx_fmat y_ = fmat_src.row(h_index) *
                           channel_model->GetMatrix(is_downlink, h_index);
        fmat_h.insert_rows(h_index, y_);
      }
      break;
    }

    default: {
      AGORA_LOG_ERROR("Invalid Channel model fading type \n");
      break;
    }
  }

  // Add noise
  Awgn(fmat_h, fmat_dst);

  if (kPrintChannelOutput) {
    Utils::PrintMat(fmat_dst, "H");
  }
}

void Channel::Awgn(const arma::cx_fmat& src, arma::cx_fmat& dst) const {
  if (channel_snr_db_ < 120.0f) {
    const int n_row = src.n_rows;
    const int n_col = src.n_cols;

    // Generate noise
    arma::cx_fmat noise(arma::randn<arma::fmat>(n_row, n_col),
                        arma::randn<arma::fmat>(n_row, n_col));

    // Supposed to be faster
    // arma::fmat x(n_row, n_col, arma::fill::arma::randn);
    // arma::fmat y(n_row, n_col, arma::fill::arma::randn);
    // arma::cx_fmat noise = arma::cx_fmat(x, y);

    // Add noise to signal
    //noise *= noise_samp_std_;
    //dst = src + noise;
    dst = src;
    
    // Check SNR
    if (kPrintSNRCheck) {
      arma::fmat noise_sq = arma::square(abs(noise));
      arma::frowvec noise_vec = arma::mean(noise_sq, 0);
      arma::fmat src_sq = arma::square(abs(src));
      arma::frowvec pwr_vec = arma::mean(src_sq, 0);
      arma::frowvec snr = 10.0f * arma::log10(pwr_vec / noise_vec);
      std::cout << "SNR: " << snr;
    }
  } else {
    dst = src;
  }
}