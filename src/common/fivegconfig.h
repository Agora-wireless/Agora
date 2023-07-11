// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file fivegconfig.h
 * @brief Declaration file for the 5G configuration class which imports
 * json configuration values into class variables and verifies that the 
 * specified configuration is compatible with 5G standards
 */

#ifndef FIVEGCONFIG_H_
#define FIVEGCONFIG_H_

#include <string>
#include <map>
#include <vector>
#include "nlohmann/json.hpp"

class FiveGConfig {
  public:
    
    static std::map<size_t, std::string> format_table;
    static std::vector<size_t> supported_formats;

    explicit FiveGConfig(nlohmann::json tdd_conf);
    ~FiveGConfig();

    std::string FiveGFormat();

    std::string FormBeaconSubframe(int format_num, size_t user_num);

    void ReadAndVerifyValues();

    std::string FormFrame(std::string frame_schedule, size_t user_num, 
                                    std::vector<std::string> flex_formats);
  private:
    nlohmann::json tdd_conf_;
    double channel_bandwidth_;
    double sampling_rate_;
    double max_supported_channel_bandwidth_;
    float subcarrier_spacing_;
    size_t ofdm_data_num_;
    size_t fft_size_;
    size_t user_num_;
    size_t numerology_;
    std::string frame_schedule_;
    std::vector<std::string> flex_formats_;
    std::vector<size_t> valid_ffts_;
    std::vector<size_t> supported_channel_bandwidths_;
    std::map<size_t, size_t> channel_bandwidth_to_ofdm_data_num_;

    bool IsSupported(size_t format_num);
    bool SetChannelBandwidth();
};

#endif /* FIVEGCONFIG_H_ */