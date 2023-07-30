// Copyright (c) 2018-2023, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file fivegconfig.cc
 * @brief Implementation file for the 5G configuration class which which imports
 * json configuration values into class variables and verifies that the 
 * specified configuration is compatible with 5G standards
 */

#include "fivegconfig.h"

#include "logger.h"
#include "utils.h"

static constexpr size_t kSubframesPerFrame = 10;
static constexpr size_t kFlexibleSlotFormatIdx = 2;
static constexpr bool kDebug = false;

static constexpr size_t kNumerology = 0;

FiveGConfig::FiveGConfig(const nlohmann::json& tdd_conf, size_t user_num)
    : valid_ffts_({512, 1024, 1536, 2048}),
      supported_channel_bandwidths_({5, 10, 15, 20}),
      supported_formats_({0, 1, 2, 3, 4, 27, 28, 34, 39}),
      user_num_(user_num) {
  tdd_conf_ = tdd_conf;
  subcarrier_spacing_ = 15e3 * pow(2, kNumerology);
  /*
  Maximum number of ofdm subcarriers that can be supported in each
  hannel bandwidth while representing whole reasource blocks
  and satisfying AVX512 requirements. 
  */
  channel_bandwidth_to_ofdm_data_num_[5] = 288;
  channel_bandwidth_to_ofdm_data_num_[10] = 624;
  channel_bandwidth_to_ofdm_data_num_[15] = 912;
  channel_bandwidth_to_ofdm_data_num_[20] = 1200;
  format_table_.at(0) = "DDDDDDDDDDDDDD";
  format_table_.at(1) = "UUUUUUUUUUUUUU";
  format_table_.at(2) = "FFFFFFFFFFFFFF";
  format_table_.at(3) = "DDDDDDDDDDDDDG";
  format_table_.at(4) = "DDDDDDDDDDDDGG";
  format_table_.at(5) = "DDDDDDDDDDDGGG";
  format_table_.at(6) = "DDDDDDDDDDGGGG";
  format_table_.at(7) = "DDDDDDDDDGGGGG";
  format_table_.at(8) = "GGGGGGGGGGGGGU";
  format_table_.at(9) = "GGGGGGGGGGGGUU";
  format_table_.at(10) = "GUUUUUUUUUUUUU";
  format_table_.at(11) = "GGUUUUUUUUUUUU";
  format_table_.at(12) = "GGGUUUUUUUUUUU";
  format_table_.at(13) = "GGGGUUUUUUUUUU";
  format_table_.at(14) = "GGGGGUUUUUUUUU";
  format_table_.at(15) = "GGGGGGUUUUUUUU";
  format_table_.at(16) = "DGGGGGGGGGGGGGG";
  format_table_.at(17) = "DDGGGGGGGGGGGGG";
  format_table_.at(18) = "DDDGGGGGGGGGGGG";
  format_table_.at(19) = "DGGGGGGGGGGGGGU";
  format_table_.at(20) = "DDGGGGGGGGGGGU";
  format_table_.at(21) = "DDDGGGGGGGGGGU";
  format_table_.at(22) = "DGGGGGGGGGGGUU";
  format_table_.at(23) = "DDGGGGGGGGGGUU";
  format_table_.at(24) = "DDDGGGGGGGGGUU";
  format_table_.at(25) = "DGGGGGGGGGGUUU";
  format_table_.at(26) = "DDGGGGGGGGGUUU";
  format_table_.at(27) = "DDDGGGGGGGGUUU";
  format_table_.at(28) = "DDDDDDDDDDDDGU";
  format_table_.at(29) = "DDDDDDDDDDDGGU";
  format_table_.at(30) = "DDDDDDDDDDGGGU";
  format_table_.at(31) = "DDDDDDDDDDDGUU";
  format_table_.at(32) = "DDDDDDDDDDGGUU";
  format_table_.at(33) = "DDDDDDDDDGGGUU";
  format_table_.at(34) = "DGUUUUUUUUUUUU";
  format_table_.at(35) = "DDGUUUUUUUUUUU";
  format_table_.at(36) = "DDDGUUUUUUUUUU";
  format_table_.at(37) = "DGGUUUUUUUUUUU";
  format_table_.at(38) = "DDGGUUUUUUUUUU";
  format_table_.at(39) = "DDDGGUUUUUUUUU";
  format_table_.at(40) = "DGGGUUUUUUUUUU";
  format_table_.at(41) = "DDGGGUUUUUUUUU";
  format_table_.at(42) = "DDDGGGUUUUUUUU";
  format_table_.at(43) = "DDDDDDDDDGGGGU";
  format_table_.at(44) = "DDDDDDGGGGGGUU";
  format_table_.at(45) = "DDDDDDGGUUUUUU";
  format_table_.at(46) = "DDDDDGUDDDDDGU";
  format_table_.at(47) = "DDGUUUUDDGUUUU";
  format_table_.at(48) = "DGUUUUUDGUUUUU";
  format_table_.at(49) = "DDDDGGUDDDDGGU";
  format_table_.at(50) = "DDGGUUUDDGGUUU";
  format_table_.at(51) = "DGGUUUUDFFUUUU";
  format_table_.at(52) = "DGGGGGUDGGGGGU";
  format_table_.at(53) = "DDGGGGUDDGGGGU";
  format_table_.at(54) = "GGGGGGGDDDDDDD";
  format_table_.at(55) = "DDGGGUUUDDDDDD";
}

FiveGConfig::~FiveGConfig() = default;

void FiveGConfig::ReadAndVerifyValues() {
  double guard_band;
  double transmission_bandwidth;
  const double num_slots = pow(2, kNumerology);
  const size_t num_symbols = kSubframesPerFrame * num_slots * 14;
  bool fft_is_valid = false;
  //ofdm_data_start and sampling rate should be calculated, not specified.
  RtAssert(!tdd_conf_.contains("ofdm_data_start"),
           "Ofdm data start is calculated using fft_size and ofdm_data_num and "
           "should not be specified by the user in a 5G schema.");
  RtAssert(!tdd_conf_.contains("sample_rate"),
           "The sampling rate is calculated using the fft_size and the "
           "subcarrier spacing which is a result of the numerology and should "
           "not be specified by the user in a 5G schema.");
  nlohmann::json jframes =
      tdd_conf_.value("frame_schedule", nlohmann::json::array());
  assert(jframes.size() == 1);
  frame_schedule_ = jframes.at(0);
  flex_formats_ = tdd_conf_.value("flex_formats", nlohmann::json::array());
  if (tdd_conf_.contains("channel_bandwidth")) {
    channel_bandwidth_ = tdd_conf_.value("channel_bandwidth", 0);
    RtAssert(channel_bandwidth_ <= supported_channel_bandwidths_.back(),
             "Specified channel bandwidth is larger than the max supported "
             "channel bandwidth.");
    RtAssert(
        !tdd_conf_.contains("ofdm_data_num") && !tdd_conf_.contains("fft_size"),
        "The channel bandwidth is not compatible with ofdm_data_num and "
        "fft_size. Either do not specify a channel bandwidth or do not "
        "specify the ofdm_data_num and fft_size.");
    //Calculate ofdm_data_num and fft_size from the channel bandwidth.
    auto iterator =
        std::find(supported_channel_bandwidths_.begin(),
                  supported_channel_bandwidths_.end(), channel_bandwidth_);
    RtAssert(*iterator == channel_bandwidth_,
             "Specified channel bandwidth is not supported.");
    ofdm_data_num_ = channel_bandwidth_to_ofdm_data_num_.at(channel_bandwidth_);
    for (size_t i = 0; i < valid_ffts_.size(); i++) {
      if (valid_ffts_.at(i) > ofdm_data_num_) {
        fft_size_ = valid_ffts_.at(i);
        break;
      }
    }
  } else {
    RtAssert(
        tdd_conf_.contains("ofdm_data_num") && tdd_conf_.contains("fft_size"),
        "ofdm_data_num and fft_size must both be specified for a 5G "
        "configuration.");
    ofdm_data_num_ = tdd_conf_.value("ofdm_data_num", 0);
    fft_size_ = tdd_conf_.value("fft_size", 0);
    RtAssert((ofdm_data_num_ % 12 == 0),
             "The given number of ofdm data subcarriers is not divisible by "
             "12. Non integer number of reasource blocks.\n");
    RtAssert(fft_size_ > ofdm_data_num_,
             "The fft_size is smaller than the number of subcarriers.\n");
    RtAssert(SetChannelBandwidth(),
             "No supported channel bandwidth compatible with given fft_size "
             "and ofdm_data_num parameters.");
    transmission_bandwidth = ofdm_data_num_ * subcarrier_spacing_;
    //channel bandwidth must be in Mhz and subcarrier spacing must be in Khz
    guard_band = (1e3) *
                 (1000 * (channel_bandwidth_) -
                  (ofdm_data_num_ + 1) * (subcarrier_spacing_ / 1e3)) /
                 2;
    RtAssert(
        transmission_bandwidth + 2 * guard_band <= channel_bandwidth_ * 1e6,
        "The channel bandwidth calculated from the specified parameters "
        "is larger than the selected channel bandwidth. Try using "
        "smaller values.");
    for (size_t i = 0; i < valid_ffts_.size(); i++) {
      if (fft_size_ == valid_ffts_.at(i)) {
        fft_is_valid = true;
        break;
      }
    }
    RtAssert(fft_is_valid, "Specified fft_size is not a valid fft size,\n");
  }
  ofdm_data_start_ = (fft_size_ - ofdm_data_num_) / 2;
  sampling_rate_ = subcarrier_spacing_ * (fft_size_);
  RtAssert(num_symbols <= kMaxSymbols, "Number of symbols exceeded " +
                                           std::to_string(kMaxSymbols) +
                                           " symbols.\n");

  if (kDebug == true) {
    AGORA_LOG_INFO("Selected channel bandwidth: %zu Mhz\n", channel_bandwidth_);
    AGORA_LOG_INFO("Calculated CBW: %f\n",
                   transmission_bandwidth + 2 * guard_band);
  }
}
/** 
 * Effects: Verifies that the passed specs are 5G compliant and compatible
 *          with eachother and returns a 5G formated frame.
*/
std::string FiveGConfig::FiveGFormat() {
  ReadAndVerifyValues();
  return FormFrame(frame_schedule_, user_num_, flex_formats_);
}
/**
 * Effects: Generates a subframe that transmits a beacon symbol and as many
 * pilot symbols as there are users.
*/
std::string FiveGConfig::FormBeaconSubframe(int format_num, size_t user_num) {
  std::string subframe = format_table_.at(format_num);
  size_t pilot_num = 0;
  RtAssert(subframe.at(0) == 'D',
           "First symbol of selected format doesn't start with a downlink "
           "symbol.");
  RtAssert(user_num < 12, "Number of users exceeds pilot symbol limit of 12.");
  //Replace the first symbol with a beacon symbol.
  subframe.replace(0, 1, "B");
  //Add in the pilot symbols.
  for (size_t i = 1; i < subframe.size(); i++) {
    // Break once user_num many pilot_nums have been put in the beacon subframe.
    if (pilot_num >= user_num) {
      break;
    }
    if (subframe.at(i) == 'U') {
      subframe.replace(i, 1, "P");
      pilot_num++;
    }
  }
  RtAssert(pilot_num == user_num,
           "More users specified than the "
           "chosen slot format can support.");
  /*
  If the last symbol of the first slot is a D and this D is not overwritten
  by a pilot and the first symbol of the next slot is a U we might get a DU 
  pair in the frame which could cause a problem.
  */
  return subframe;
}
/**
 * Effects: Builds a symbol based frame which Agora is built to handle from the 
 *          slot format based frame given in the frame schedule.
*/
std::string FiveGConfig::FormFrame(std::string frame_schedule, size_t user_num,
                                   std::vector<std::string> flex_formats) {
  std::string frame;
  std::string temp = "";
  size_t subframes[kSubframesPerFrame];
  size_t subframe_idx = 0;
  size_t flex_format_idx = 0;

  for (size_t i = 0; i < frame_schedule.size(); i++) {
    RtAssert(subframe_idx < 10,
             "Entered frame_schedule has more than 10 subframes.");

    if (frame_schedule.at(i) == ',') {
      subframes[subframe_idx] = std::stoi(temp);
      RtAssert(IsSupported(subframes[subframe_idx]),
               "Format " + std::to_string(subframes[subframe_idx]) +
                   " isn't supported.");
      subframe_idx++;
      temp.clear();
    } else {
      temp += std::to_string(frame_schedule.at(i) - 48);
    }
    if (i == frame_schedule.size() - 1) {
      subframes[subframe_idx] = std::stoi(temp);
    }
  }
  RtAssert(subframe_idx == 9,
           "Entered frame_schedule has less than 10 subframes.");
  // Create the frame based on the format nums in the subframe array.
  frame += FormBeaconSubframe(subframes[0], user_num);
  for (size_t i = 1; i < kSubframesPerFrame; i++) {
    if (subframes[i] == kFlexibleSlotFormatIdx) {
      frame += flex_formats.at(flex_format_idx);
      flex_format_idx++;
    } else {
      frame += format_table_.at(subframes[i]);
    }
  }
  return frame;
}
/**
 * Effects: Checks that the passed format is in the list of supported formats.
*/
bool FiveGConfig::IsSupported(size_t format_num) const {
  for (size_t i = 0; i < supported_formats_.size(); i++) {
    if (format_num == supported_formats_[i]) {
      return true;
    }
  }
  std::string error_message =
      "User specified a non supported subframe format.\nCurrently "
      "supported subframe formats are:\n";
  for (auto format = supported_formats_.begin();
       format != supported_formats_.end(); format++) {
    error_message +=
        std::to_string(*format) + " " + format_table_.at(*format) + ".\n";
  }
  AGORA_LOG_ERROR(error_message);
  return false;
}
/**
 * Effects: Sets the channel bandwidth based on the ofdm_data_num.
*/
bool FiveGConfig::SetChannelBandwidth() {
  for (auto iterator = channel_bandwidth_to_ofdm_data_num_.begin();
       iterator != channel_bandwidth_to_ofdm_data_num_.end(); ++iterator) {
    if (iterator->second >= ofdm_data_num_) {
      channel_bandwidth_ = iterator->first;
      return true;
    }
  }
  return false;
}
//Accessors for sampling rate and ofdm data start.
double FiveGConfig::SamplingRate() const { return sampling_rate_; }
size_t FiveGConfig::FftSize() const { return fft_size_; }
size_t FiveGConfig::OfdmDataNum() const { return ofdm_data_num_; }
size_t FiveGConfig::OfdmDataStart() const { return ofdm_data_start_; }
