// Copyright (c) 2018-2023, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file fivegconfig.h
 * @brief Declaration file for the 5G configuration class which imports
 * json configuration values into class variables and verifies that the 
 * specified configuration is compatible with 5G standards
 */

#ifndef FIVEGCONFIG_H_
#define FIVEGCONFIG_H_

#include <array>
#include <map>
#include <string>
#include <vector>

#include "nlohmann/json.hpp"

class FiveGConfig {
  static constexpr size_t kFormatTableSize = 56;

 public:
  explicit FiveGConfig(const nlohmann::json &tdd_conf);
  ~FiveGConfig();

  std::string FiveGFormat();
  double SamplingRate() const;
  size_t OfdmDataStart() const;

 private:
  nlohmann::json tdd_conf_;
  double sampling_rate_;
  float subcarrier_spacing_;
  size_t channel_bandwidth_;
  size_t ofdm_data_num_;
  size_t ofdm_data_start_;
  size_t fft_size_;
  size_t user_num_;
  std::string frame_schedule_;
  std::vector<std::string> flex_formats_;
  const std::vector<size_t> valid_ffts_;
  const std::vector<size_t> supported_channel_bandwidths_;
  //only valid for numerology 0.
  std::map<size_t, size_t> channel_bandwidth_to_ofdm_data_num_;
  std::array<std::string, kFormatTableSize> format_table_;
  const std::vector<size_t> supported_formats_;
  //Checks if the specified format number is supported by agora.
  bool IsSupported(size_t format_num) const;
  //Sets the config's channel bandwidth based on the specified ofdm data number.
  bool SetChannelBandwidth();
  //Reads user inputs related to a 5G config and verifies that they
  // are in spec.
  void ReadAndVerifyValues();
  //Creates a 140 symbol long frame in compliance with 5G configuration
  //using a user specified frame schedule.
  std::string FormFrame(std::string frame_schedule, size_t user_num,
                        std::vector<std::string> flex_formats);
  //Puts the beacon and pilot symbols in the first subframe of
  //the frame.
  std::string FormBeaconSubframe(int format_num, size_t user_num);
};

#endif /* FIVEGCONFIG_HPP_ */