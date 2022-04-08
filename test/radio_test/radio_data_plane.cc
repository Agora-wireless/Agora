/**
 * @file radio_data_plane.cc
 * @brief Defination file for the RadioDataPlane Class
 */

#include "radio_data_plane.h"

#include "SoapySDR/Formats.h"
#include "SoapySDR/Time.hpp"
#include "logger.h"
#include "utils.h"

RadioDataPlane::RadioDataPlane() : RadioDataPlane(nullptr, nullptr, 0) {}

RadioDataPlane::RadioDataPlane(const Config* cfg, SoapySDR::Device* device,
                               size_t id)
    : mode_(kModeUninit) {
  RadioDataPlane::Init(cfg, device, id);
}

RadioDataPlane::~RadioDataPlane() { RadioDataPlane::Close(); }

void RadioDataPlane::Init(const Config* cfg, SoapySDR::Device* device,
                          size_t id) {
  if (mode_ == kModeUninit) {
    if ((cfg != nullptr) && (device != nullptr)) {
      cfg_ = cfg;
      device_ = device;
      radio_id_ = id;
      mode_ = kModeShutdown;
    } else {
      AGORA_LOG_WARN(
          "Attempted to init the data plane with null cfg or device pointer\n");
    }
  } else {
    AGORA_LOG_ERROR(
        "Attempted to init the data plane while in the wrong mode %d\n",
        static_cast<int>(mode_));
  }
}

void RadioDataPlane::Activate() {
  if (mode_ == kModeDeactive) {
    if ((device_ == nullptr) || (rx_stream_ == nullptr)) {
      AGORA_LOG_ERROR(
          "Device pointer or rx_stream is null in RadioDataPlane Activate");
    } else {
      device_->activateStream(rx_stream_);
      mode_ = kModeActive;
    }
  } else {
    AGORA_LOG_WARN("Attempting to Activate data plane when in wrong state %d\n",
                   static_cast<int>(mode_));
  }
}

void RadioDataPlane::Deactivate() {
  if (mode_ == kModeActive) {
    if ((device_ == nullptr) || (rx_stream_ == nullptr)) {
      AGORA_LOG_ERROR(
          "Device pointer or rx_stream is null in RadioDataPlane Deactivate");
    } else {
      device_->deactivateStream(rx_stream_);
      mode_ = kModeDeactive;
    }
  } else {
    AGORA_LOG_WARN(
        "Attempting to Deactivate data plane when in wrong state %d\n",
        static_cast<int>(mode_));
  }
}

void RadioDataPlane::Close() {
  if (device_ == nullptr) {
    AGORA_LOG_ERROR("Device pointer is null in RadioDataPlane Close");
    return;
  }

  if (mode_ == kModeActive) {
    Deactivate();
  }

  if (mode_ == kModeDeactive) {
    device_->closeStream(rx_stream_);
    rx_stream_ = nullptr;
  }
  mode_ = kModeShutdown;
}

void RadioDataPlane::Setup() {
  if (mode_ == kModeShutdown) {
    SoapySDR::Kwargs sargs;
    auto channels = Utils::StrToChannels(cfg_->Channel());
    rx_stream_ =
        device_->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    mode_ = kModeDeactive;
  } else {
    AGORA_LOG_WARN(
        "Attempting to Setup a previously configured data plane "
        "connection\n");
  }
}
