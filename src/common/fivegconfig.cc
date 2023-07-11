// Copyright (c) 2018-2022, Rice University
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

using json = nlohmann::json;

static constexpr size_t kSubframesPerFrame = 10;
static constexpr size_t kFlexibleSlotFormatIdx = 2;
static constexpr size_t kMaxSlotFormat = 55;


std::map<size_t, std::string> FiveGConfig::format_table {};
std::vector<size_t> FiveGConfig::supported_formats{};

FiveGConfig::FiveGConfig (json tdd_conf) {
  tdd_conf_ = tdd_conf;
  max_supported_channel_bandwidth_ = 20e6;
  subcarrier_spacing_ = 15e3;
  numerology_ = 0;
  valid_ffts_= {512, 1024, 1536, 2048};
  supported_channel_bandwidths_ = {5, 10, 15, 20};
  channel_bandwidth_to_ofdm_data_num_[5] = 288;
  channel_bandwidth_to_ofdm_data_num_[10] = 624;
  channel_bandwidth_to_ofdm_data_num_[15] = 912;
  channel_bandwidth_to_ofdm_data_num_[20] = 1200;
  format_table[0] = "DDDDDDDDDDDDDD";
  format_table[1] = "UUUUUUUUUUUUUU";
  format_table[3] = "DDDDDDDDDDDDDG";
  format_table[4] = "DDDDDDDDDDDDGG";
  format_table[5] = "DDDDDDDDDDDGGG";
  format_table[6] = "DDDDDDDDDDGGGG";
  format_table[7] = "DDDDDDDDDGGGGG";
  format_table[8] = "GGGGGGGGGGGGGU";
  format_table[9] = "GGGGGGGGGGGGUU";
  format_table[10] = "GUUUUUUUUUUUUU";
  format_table[11] = "GGUUUUUUUUUUUU";
  format_table[12] = "GGGUUUUUUUUUUU";
  format_table[13] = "GGGGUUUUUUUUUU";
  format_table[14] = "GGGGGUUUUUUUUU";
  format_table[15] = "GGGGGGUUUUUUUU";
  format_table[16] = "DGGGGGGGGGGGGGG";
  format_table[17] = "DDGGGGGGGGGGGGG";
  format_table[18] = "DDDGGGGGGGGGGGG";
  format_table[19] = "DGGGGGGGGGGGGGU";
  format_table[20] = "DDGGGGGGGGGGGU";
  format_table[21] = "DDDGGGGGGGGGGU";
  format_table[22] = "DGGGGGGGGGGGUU";
  format_table[23] = "DDGGGGGGGGGGUU";
  format_table[24] = "DDDGGGGGGGGGUU";
  format_table[25] = "DGGGGGGGGGGUUU";
  format_table[26] = "DDGGGGGGGGGUUU";
  format_table[27] = "DDDGGGGGGGGUUU";
  format_table[28] = "DDDDDDDDDDDDGU";
  format_table[29] = "DDDDDDDDDDDGGU";
  format_table[30] = "DDDDDDDDDDGGGU";
  format_table[31] = "DDDDDDDDDDDGUU";
  format_table[32] = "DDDDDDDDDDGGUU";
  format_table[33] = "DDDDDDDDDGGGUU";
  format_table[34] = "DGUUUUUUUUUUUU";
  format_table[35] = "DDGUUUUUUUUUUU";
  format_table[36] = "DDDGUUUUUUUUUU";
  format_table[37] = "DGGUUUUUUUUUUU";
  format_table[38] = "DDGGUUUUUUUUUU";
  format_table[39] = "DDDGGUUUUUUUUU";
  format_table[40] = "DGGGUUUUUUUUUU";
  format_table[41] = "DDGGGUUUUUUUUU";
  format_table[42] = "DDDGGGUUUUUUUU";
  format_table[43] = "DDDDDDDDDGGGGU";
  format_table[44] = "DDDDDDGGGGGGUU";
  format_table[45] = "DDDDDDGGUUUUUU";
  format_table[46] = "DDDDDGUDDDDDGU";
  format_table[47] = "DDGUUUUDDGUUUU";
  format_table[48] = "DGUUUUUDGUUUUU";
  format_table[49] = "DDDDGGUDDDDGGU";
  format_table[50] = "DDGGUUUDDGGUUU";
  format_table[51] = "DGGUUUUDFFUUUU";
  format_table[52] = "DGGGGGUDGGGGGU";
  format_table[53] = "DDGGGGUDDGGGGU";
  format_table[54] = "GGGGGGGDDDDDDD";
  format_table[55] = "DDGGGUUUDDDDDD";
  supported_formats.push_back(0);
  supported_formats.push_back(1);
  supported_formats.push_back(2);
  supported_formats.push_back(3);
  supported_formats.push_back(4);
  supported_formats.push_back(27);
  supported_formats.push_back(28);
  supported_formats.push_back(34);
  supported_formats.push_back(39);
}

FiveGConfig::~FiveGConfig() = default;

void FiveGConfig::ReadAndVerifyValues() {
  double guard_band; 
  double transmission_bandwidth; 
  double min_sampling_rate;
  double num_slots = pow(2, numerology_);
  size_t num_symbols = kSubframesPerFrame*num_slots*14;
  bool fft_is_valid = false;

  user_num_ = tdd_conf_.value("ue_radio_num", 0);
  json jframes = tdd_conf_.value("frame_schedule", json::array());
  assert(jframes.size() == 1);
  frame_schedule_ = jframes.at(0);
  flex_formats_ = tdd_conf_.value("flex_formats", json::array());

  if (tdd_conf_.contains("channel_bandwidth")) {
    channel_bandwidth_ = tdd_conf_.value("channel_bandwidth", -1);
    if (tdd_conf_.contains("sample_rate") || 
      tdd_conf_.contains("num_ofdm_data_sub") ||
      tdd_conf_.contains("fft_size")) {
      throw std::runtime_error("The Channel Bandwidth variable is not "
        "compatible with sample rate, num_ofdm_data_sub, and fft_size. "
        "Either do not specify a channel bandwidth or do not specify the "
        "sample rate, num_ofdm_data_sub, and fft_size.");
    }

    auto iterator = std::find(supported_channel_bandwidths_.begin(), 
                              supported_channel_bandwidths_.end(), 
                              channel_bandwidth_);
    RtAssert(*iterator == channel_bandwidth_,
            "Specified Channel Bandwidth is not supported.");
    ofdm_data_num_ = channel_bandwidth_to_ofdm_data_num_.at(channel_bandwidth_);

    for (size_t i = 0; i < valid_ffts_.size(); i++) {
      if (valid_ffts_.at(i) > 2*ofdm_data_num_) {
        fft_size_ = valid_ffts_.at(i); // Need to check this to make sure
        //it produces a 10 ms frame.
    }
  }
    //Set the sample_rate, num_ofdm_data_sub, and fft_size given the
    //channel bandwidth
    sampling_rate_ = subcarrier_spacing_ * fft_size_;
  } else {
    RtAssert(tdd_conf_.contains("sample_rate") && 
            tdd_conf_.contains("ofdm_data_num") &&
            tdd_conf_.contains("fft_size"), "Sample rate, "
            "num_ofdm_data_subcarriers and fft_size must all be specified for "
            "a 5G configuration.");

    sampling_rate_ = tdd_conf_.value("sample_rate", -1);
    ofdm_data_num_ = tdd_conf_.value("ofdm_data_num", 0);
    fft_size_ = tdd_conf_.value("fft_size", -1);

    RtAssert(SetChannelBandwidth());

    transmission_bandwidth = ofdm_data_num_ * subcarrier_spacing_;
    //channel bandwidth must be in Mhz and subcarrier spacing must be in Khz
    guard_band = (1e3) * (1000 * (channel_bandwidth_) - 
      (ofdm_data_num_ + 1) * (subcarrier_spacing_ / 1e3)) / 2;

    std::cout<<"CBW: " << std::to_string(transmission_bandwidth + 2* guard_band) << std::endl;
    std::cout<<"TBW: " << std::to_string(transmission_bandwidth) << std::endl;
    std::cout<<"Channel_bandwidth: " << std::to_string(channel_bandwidth_) << std::endl;
    std::cout<<"sampling_rate_: " << std::to_string(sampling_rate_) << std::endl;
    std::cout<<"ofdm_data_num: " << std::to_string(ofdm_data_num_) << std::endl;
    std::cout<<"fft_size_: " << std::to_string(fft_size_) << std::endl;


    std::cout<<"max_supported_channel_bandwidh: " << std::to_string(max_supported_channel_bandwidth_) << std::endl<<std::flush;

    RtAssert(transmission_bandwidth+2*guard_band <= max_supported_channel_bandwidth_,
    "The channel bandwidth required to transmit given the specified parameters"
    "is larger than Agora currently supports. Try using smaller values.");

    RtAssert(!(ofdm_data_num_ % 16), "The given number of ofdm data "
    "subcarriers is not divisible by 16. Agora's memory management requires"
    " that the number of ofdm subcarriers be divisible by 16.\n");
    RtAssert(!(ofdm_data_num_ % 12), "The given number of ofdm data "
    "subcarriers is not divisible by 12. Non integer number of reasource blocks.\n");
    RtAssert(fft_size_ > ofdm_data_num_, "The fft_size is smaller than the "
    "number of subcarriers.\n");

    for (size_t i = 0; i < valid_ffts_.size(); i++) {
            if (fft_size_ == valid_ffts_.at(i)) {
                    fft_is_valid = true;
            }
    }
    RtAssert(fft_is_valid, "Specified fft_size is not a valid fft size,\n");
  }

  min_sampling_rate = subcarrier_spacing_*(fft_size_); 
  
  if (min_sampling_rate < sampling_rate_) {
    AGORA_LOG_WARN("Specified sampling rate %f is larger than the " 
    "minimum required sampling rate of %f.\n", sampling_rate_, 
    min_sampling_rate);
  }
  RtAssert(num_symbols <= kMaxSymbols, "Number of symbols exceeded " +
        std::to_string(kMaxSymbols) + " symbols.\n");
}

/** 
 * Effects: Verifies that the passed specs are 5G compliant and compatible
 *          with eachother and returns a 5G formated frame.
*/
std::string FiveGConfig::FiveGFormat() {
  ReadAndVerifyValues();
  std::cout<<"Here.\n" << std::flush;
  return FormFrame(frame_schedule_, user_num_, flex_formats_);
}

/**
 * Effects: Generates a subframe that transmits a beacon symbol and as many
 * pilot symbols as there are users.
*/
std::string FiveGConfig::FormBeaconSubframe(int format_num, size_t user_num) {
  std::string subframe = format_table[format_num];
  size_t pilot_num = 0;

  RtAssert(subframe.at(0) == 'D', "First symbol of selected format doesn't start with a downlink symbol.");
  RtAssert(user_num_ < 12, "Number of users exceeds pilot symbol limit of 12.");
  //Replace the first symbol with a beacon symbol.
  subframe.replace(0, 1, "B");

  std::cout<< subframe << std::endl << std::flush;
  std::cout<< "format num: " << std::to_string(format_num) << std::endl << std::flush;

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

  std::cout<<"Pilot num: " << std::to_string(pilot_num) << std::flush;
  RtAssert(pilot_num == user_num_, "More users specified than the " 
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

    std::cout<<"subframe index: " << subframe_idx << std::endl<<std::flush;
    std::cout<< "inex: " << i << std::endl<<std::flush;
    RtAssert(subframe_idx < 10, "Entered frame_schedule has more than 10 subframes.");

    if (frame_schedule.at(i) == ',') {   
            subframes[subframe_idx] = std::stoi(temp); 
            RtAssert(IsSupported(subframes[subframe_idx]), "Format " + std::to_string(subframes[subframe_idx]) + " isn't supported.");
            subframe_idx++;
            temp.clear();
    } else {
            temp += std::to_string(frame_schedule.at(i) - 48);
    } 
    if (i == frame_schedule.size()-1) {
            subframes[subframe_idx] = std::stoi(temp);
    }
  }

  RtAssert(subframe_idx == 9, "Entered frame_schedule has less than 10 subframes.");



  std::cout<<"Midway"<<std::endl<<std::flush;

  // Create the frame based on the format nums in the subframe array.
  frame += FormBeaconSubframe(subframes[0], user_num_);
  for (size_t i = 1; i < kSubframesPerFrame; i++) {  

    if (subframes[i] < 0 || subframes[i] > kMaxSlotFormat) {
      std::string error_message = "User specified a non supported subframe "
      "format.\nCurrently supported subframe formats are:";

      for (auto format = format_table.begin(); format != format_table.end(); format++) {
            error_message += std::to_string(format->first) + " " + format->second + ".\n";
      }
      throw std::runtime_error(error_message); 
    } else {
      if (subframes[i] == kFlexibleSlotFormatIdx) {
              frame += flex_formats.at(flex_format_idx);
              flex_format_idx++;
      } else {
              frame += format_table.at(subframes[i]);
      }
    }
  }
  return frame;
}

bool FiveGConfig::IsSupported(size_t format_num) {
  for (size_t i = 0; i < supported_formats.size(); i++) { 
    if (format_num == supported_formats[i]) {
      return true;
    }
  }
  return false;
}

bool FiveGConfig::SetChannelBandwidth(){
    for (auto iterator = channel_bandwidth_to_ofdm_data_num_.begin(); iterator != channel_bandwidth_to_ofdm_data_num_.end(); ++iterator) {
      if (iterator->second > ofdm_data_num_) {
        channel_bandwidth_ = iterator->first;
        return true;
      }
    }
    return false;
}
