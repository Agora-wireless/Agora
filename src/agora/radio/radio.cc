/** @file radio.h
  * @brief Defination file for the Radio class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#include "radio.h"

#include "logger.h"
#include "radio_soapysdr.h"

std::unique_ptr<Radio> Radio::Create(Radio::RadioType type) {
  switch (type) {
    case kSoapySdrStream: {
      return std::make_unique<RadioSoapySdr>(RadioDataPlane::kSoapyStream);
    }
    case kSoapySdrSocket: {
      return std::make_unique<RadioSoapySdr>(RadioDataPlane::kLinuxSocket);
    }
    default: {
      AGORA_LOG_ERROR("Unknown or unsupported radio type\n");
      return std::unique_ptr<Radio>();
    }
  }
}

Radio::Radio() : serial_number_("") {
  AGORA_LOG_TRACE("Create Generic Null Radio\n");
}

Radio::~Radio() {
  AGORA_LOG_TRACE("Destroy Radio %s(%zu)\n", serial_number_.c_str(), id_);
  Radio::Close();
}

void Radio::Close() {
  if ((serial_number_.empty()) && (cfg_ == nullptr)) {
    AGORA_LOG_TRACE("Close Radio %s(%zu)\n", serial_number_.c_str(), id_);
    id_ = 0;
    serial_number_ = "";
    enabled_channels_.clear();
    hw_framer_ = false;
    cfg_ = nullptr;
  }
}

void Radio::Init(const Config* cfg, size_t id, const std::string& serial,
                 const std::vector<size_t>& enabled_channels, bool hw_framer) {
  AGORA_LOG_TRACE("Init Radio %s(%zu)\n", serial.c_str(), id);

  id_ = id;
  cfg_ = cfg;
  serial_number_ = serial;
  enabled_channels_ = enabled_channels;
  hw_framer_ = hw_framer;
}

void Radio::Setup([[maybe_unused]] const std::vector<double>& tx_gains,
                  [[maybe_unused]] const std::vector<double>& rx_gains) {
  AGORA_LOG_TRACE("Setup Radio %s(%zu)\n", serial_number_.c_str(), id_);
}

void Radio::Activate([[maybe_unused]] Radio::ActivationTypes type,
                     [[maybe_unused]] long long act_time_ns,
                     [[maybe_unused]] size_t samples) {
  AGORA_LOG_TRACE(
      "Activate Radio %s(%zu) with type %d at time %lld for %zu samples\n",
      serial_number_.c_str(), id_, static_cast<int>(type), act_time_ns,
      samples);
}

void Radio::Deactivate() {
  AGORA_LOG_TRACE("Deactivate Radio %s(%zu)\n", serial_number_.c_str(), id_);
}