/**
 * @file channel.cc
 * @brief Implementation file for the channel class
 */
#include "channel.h"

#include "armadillo"
#include "logger.h"
#include "utils.h"

static constexpr bool kPrintChannelOutput = false;
static constexpr bool kPrintSNRCheck = false;

Channel::~Channel() = default;

Channel::Channel(const Config* const config, std::string& in_channel_type,
                 std::string& dataset_path)
    : cfg_(config), sim_chan_model_(std::move(in_channel_type)) {
  channel_model_ = std::move(
      ChannelModel::CreateChannelModel(cfg_, sim_chan_model_, dataset_path));

  noise_samp_std_ = config->NoiseLevel() / std::sqrt(2);

  std::cout << "Noise level to be used is: " << std::fixed << std::setw(5)
            << std::setprecision(3) << noise_samp_std_ << std::endl;
}

void Channel::ApplyChan(const arma::cx_fmat& fmat_src, arma::cx_fmat& fmat_dst,
                        const bool is_downlink, const bool is_newChan) {
  arma::cx_fmat fmat_h;

  if (is_newChan) {
    channel_model_->UpdateModel();
  }

  switch (channel_model_->GetFadingType()) {
    case ChannelModel::kFlat: {
      fmat_h = fmat_src * channel_model_->GetMatrix(is_downlink);
      break;
    }

    case ChannelModel::kSelective: {
      const size_t n_rows = (cfg_->FreqDomainChannel())
                                ? cfg_->OfdmCaNum()
                                : cfg_->SampsPerSymbol();
      const size_t n_cols = (is_downlink) ? cfg_->UeAntNum() : cfg_->BsAntNum();

      fmat_h.zeros(n_rows, n_cols);
      for (size_t h_index = 0; h_index < n_rows; h_index++) {
        arma::cx_fmat H;
        if (cfg_->FreqDomainChannel()) {
          if (h_index < cfg_->OfdmDataStart() ||
              h_index >= cfg_->OfdmDataStop()) {
            const size_t new_dim =
                (is_downlink) ? cfg_->BsAntNum() : cfg_->UeAntNum();
            //H = arma::zeros<arma::cx_fmat>(new_dim, n_cols);
            H = channel_model_->GetMatrix(is_downlink, 0u);
          } else {
            H = channel_model_->GetMatrix(is_downlink,
                                          h_index - cfg_->OfdmDataStart());
          }
        } else {
          H = channel_model_->GetMatrix(is_downlink, h_index);
        }
        fmat_h.row(h_index) = fmat_src.row(h_index) * H;
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
  if (cfg_->NoiseLevel() < 0.0001f) {
    const int n_row = src.n_rows;
    const int n_col = src.n_cols;

    // Generate noise
    arma::cx_fmat noise(arma::randn<arma::fmat>(n_row, n_col),
                        arma::randn<arma::fmat>(n_row, n_col));

    noise *= noise_samp_std_;
    dst = src + noise;

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
