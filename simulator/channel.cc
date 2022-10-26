/**
 * @file channel.h
 * @brief Implementation file for the channel class
 */
#include "channel.h"

#include <utility>

#include "logger.h"

static constexpr bool kPrintChannelOutput = false;
static constexpr bool kPrintSNRCheck = false;
static constexpr bool kPrintCSIFromFile = false;
static constexpr double kMeanChannelGain = 0.1f;


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
    csi_matrices_ = arma::cx_fcube(ue_ant_, bs_ant_, kNumStatsFrames);
    const std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    const std::string filename_csi = cur_directory + "/data/H_" +
                                     std::to_string(bs_ant_) + "x" +
                                     std::to_string(ue_ant_) + ".bin";
    FILE* fp = std::fopen(filename_csi.c_str(), "rb");
    const size_t expected_count = bs_ant_ * ue_ant_ * 2;
    RtAssert(fp != nullptr, "Failed to open CSI data file");
    for (size_t i = 0; i < kNumStatsFrames; i++) {
      const size_t actual_count = std::fread(csi_matrices_.slice(i).memptr(),
                                             sizeof(float), expected_count, fp);
      if (expected_count != actual_count) {
        std::fprintf(stderr,
                     "ChannelSim: Failed to read CSI data file %s. Time slice "
                     "%zu: expected "
                     "%zu I/Q samples but read %zu. Errno %s\n",
                     filename_csi.c_str(), i, expected_count, actual_count,
                     strerror(errno));
        throw std::runtime_error("Sender: Failed to read IQ data file");
      }
      if (kPrintCSIFromFile) {
        std::cout << "CSI slice: " << i << std::endl;
        csi_matrices_.slice(i).print();
      }
    }
  } else {
    chan_model_ = kAwgn;
  }
  float snr_lin = std::pow(10, channel_snr_db_ / 10.0f);
  noise_samp_std_ = std::sqrt(kMeanChannelGain / (snr_lin * 2.0f));
  std::cout << "Noise level to be used is: " << std::fixed << std::setw(5)
            << std::setprecision(2) << noise_samp_std_ << std::endl;
}

Channel::~Channel() = default;

void Channel::ApplyChan(const arma::cx_fmat& fmat_src, arma::cx_fmat& fmat_dst,
                        const bool is_downlink, const bool is_newChan) {
  arma::cx_fmat fmat_h;
  bool gen_new_channel = is_newChan or h_.is_empty();
  if (gen_new_channel) {
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
          h_ = sqrt(kMeanChannelGain / 2.0f) * h_;
          // h_ = h_ / abs(h_).max();
          // H = H / abs(H).max();
        }
        break;

      case kRan3Gpp:
        h_ = csi_matrices_.slice(time_slice_id_);
        time_slice_id_++;
        if (time_slice_id_ == kNumStatsFrames) time_slice_id_ = 0;
        // Lte3gpp(fmat_src, fmat_dst);
        break;
    }
  }
  if (is_downlink) {
    fmat_h = fmat_src * h_.st();
  } else {
    fmat_h = fmat_src * h_;
  }

  // Add noise
  // fmat_dst = fmat_h;
  AwgnNoise(fmat_h, fmat_dst);

  if (kPrintChannelOutput && gen_new_channel) {
    Utils::PrintMat(h_, "H");
  }
}

void Channel::AwgnNoise(const arma::cx_fmat& src, arma::cx_fmat& dst) const {
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

void Channel::Lte3gpp(const arma::cx_fmat& fmat_src, arma::cx_fmat& fmat_dst) {
  // TODO - In progress (Use Rayleigh for now...)
  arma::cx_fmat h(arma::randn<arma::fmat>(cfg_->UeAntNum(), cfg_->BsAntNum()),
                  arma::randn<arma::fmat>(cfg_->UeAntNum(), cfg_->BsAntNum()));
  h = (1 / sqrt(2)) * h;
  fmat_dst = fmat_src * h;
}
