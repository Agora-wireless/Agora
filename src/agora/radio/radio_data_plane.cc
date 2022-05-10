/**
 * @file radio_data_plane.cc
 * @brief Defination file for the RadioDataPlane Class
 */

#include "radio_data_plane.h"

#include "SoapySDR/Formats.h"
#include "SoapySDR/Time.hpp"
#include "logger.h"
#include "radio_data_plane_soapy.h"
#include "radio_data_plane_socket.h"
#include "radio_soapysdr.h"
#include "utils.h"

std::unique_ptr<RadioDataPlane> RadioDataPlane::Create(
    RadioDataPlane::DataPlaneType type) {
  switch (type) {
    case SoapyStream: {
      return std::make_unique<RadioDataPlaneSoapy>();
    }
    case LinuxSocket: {
      return std::make_unique<RadioDataPlaneSocket>();
    }
    default: {
      AGORA_LOG_ERROR("Unknown or unsupported radio type\n");
      return std::unique_ptr<RadioDataPlane>();
    }
  }
}

RadioDataPlane::RadioDataPlane()
    : radio_(nullptr), remote_stream_(nullptr), mode_(kModeUninit) {}

RadioDataPlane::~RadioDataPlane() { RadioDataPlane::Close(); }

void RadioDataPlane::Init(Radio* radio, const Config* cfg) {
  if (mode_ == kModeUninit) {
    //This data plane class only works with RadioSoapySdr class
    auto soapy_radio = dynamic_cast<RadioSoapySdr*>(radio);
    if ((soapy_radio != nullptr) && (radio != nullptr)) {
      cfg_ = cfg;
      radio_ = radio;
      mode_ = kModeShutdown;
    } else {
      AGORA_LOG_WARN(
          "Attempted to init the data plane with an invalid radio pointer, "
          "requires radio to be of type RadioSoapySdr\n");
    }
  } else {
    AGORA_LOG_ERROR(
        "Attempted to init the data plane while in the wrong mode %d\n",
        static_cast<int>(mode_));
  }
}

void RadioDataPlane::Activate(Radio::ActivationTypes type) {
  if (mode_ == kModeDeactive) {
    if ((radio_ == nullptr) || (remote_stream_ == nullptr)) {
      AGORA_LOG_ERROR(
          "Device pointer or remote stream is null in RadioDataPlane "
          "Activate");
    } else {
      auto device = dynamic_cast<RadioSoapySdr*>(radio_)->SoapyDevice();
      int soapy_flags = 0;
      if (type == Radio::ActivationTypes::kActivateWaitTrigger) {
        soapy_flags = SOAPY_SDR_WAIT_TRIGGER;
      }
      device->activateStream(remote_stream_, soapy_flags);
      mode_ = kModeActive;
    }
  } else {
    AGORA_LOG_WARN("Attempting to Activate data plane when in wrong state %d\n",
                   static_cast<int>(mode_));
  }
}

void RadioDataPlane::Deactivate() {
  if (mode_ == kModeActive) {
    if ((radio_ == nullptr) || (remote_stream_ == nullptr)) {
      AGORA_LOG_ERROR(
          "Device pointer or remote stream is null in RadioDataPlane "
          "Deactivate");
    } else {
      auto device = dynamic_cast<RadioSoapySdr*>(radio_)->SoapyDevice();
      device->deactivateStream(remote_stream_);
      mode_ = kModeDeactive;
    }
  } else {
    AGORA_LOG_WARN(
        "Attempting to Deactivate data plane when in wrong state %d\n",
        static_cast<int>(mode_));
  }
}

void RadioDataPlane::Close() {
  if (radio_ == nullptr) {
    AGORA_LOG_ERROR("Device pointer is null in RadioDataPlane Close");
    return;
  }

  if (mode_ == kModeActive) {
    Deactivate();
  }

  if (mode_ == kModeDeactive) {
    auto device = dynamic_cast<RadioSoapySdr*>(radio_)->SoapyDevice();
    device->closeStream(remote_stream_);
    remote_stream_ = nullptr;
  }
  mode_ = kModeShutdown;
}

void RadioDataPlane::Setup(const SoapySDR::Kwargs& args) {
  if (mode_ == kModeShutdown) {
    auto radio = dynamic_cast<RadioSoapySdr*>(radio_);
    auto device = radio->SoapyDevice();
    remote_stream_ = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16,
                                         radio->EnabledChannels(), args);
    mode_ = kModeDeactive;
  } else {
    AGORA_LOG_WARN(
        "Attempting to Setup a previously configured data plane "
        "connection\n");
  }
}

void RadioDataPlane::Setup() {
  SoapySDR::Kwargs args;
  return RadioDataPlane::Setup(args);
}