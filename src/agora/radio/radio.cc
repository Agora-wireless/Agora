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
    case SoapySdr: {
      return std::make_unique<RadioSoapySdr>();
    }
    default: {
      AGORA_LOG_ERROR("Unknown or unsupported radio type\n");
      return std::unique_ptr<Radio>();
    }
  }
}

Radio::Radio() : cfg_(nullptr), id_(0), serial_number_("") {
  AGORA_LOG_INFO("Create Generic Null Radio\n");
}

Radio::~Radio() {
  AGORA_LOG_INFO("Destroy Radio %s(%zu)\n", serial_number_.c_str(), id_);
  Radio::Close();
}

void Radio::Close() {
  AGORA_LOG_INFO("Close Radio %s(%zu)\n", serial_number_.c_str(), id_);
  id_ = 0;
  serial_number_ = "";
  enabled_channels_.clear();
  cfg_ = nullptr;
}

void Radio::Init(const Config* cfg, size_t id, const std::string& serial,
                 const std::vector<size_t>& enabled_channels) {
  AGORA_LOG_INFO("Init Radio %s(%zu)\n", serial.c_str(), id);

  id_ = id;
  cfg_ = cfg;
  serial_number_ = serial;
  enabled_channels_ = enabled_channels;
}

void Radio::Setup([[maybe_unused]] const std::vector<double>& tx_gains,
                  [[maybe_unused]] const std::vector<double>& rx_gains) {
  AGORA_LOG_INFO("Setup Radio %s(%zu)\n", serial_number_.c_str(), id_);
}

void Radio::Activate() {
  AGORA_LOG_INFO("Activate Radio %s(%zu)\n", serial_number_.c_str(), id_);
}

void Radio::Deactivate() {
  AGORA_LOG_INFO("Deactivate Radio %s(%zu)\n", serial_number_.c_str(), id_);
}