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
    case kSoapyStream: {
      return std::make_unique<RadioDataPlaneSoapy>();
    }
    case kLinuxSocket: {
      return std::make_unique<RadioDataPlaneSocket>();
    }
    default: {
      AGORA_LOG_ERROR("Unknown or unsupported radio type\n");
      return std::unique_ptr<RadioDataPlane>();
    }
  }
}

RadioDataPlane::RadioDataPlane() = default;

RadioDataPlane::~RadioDataPlane() { RadioDataPlane::Close(); }

void RadioDataPlane::Init(Radio *radio, const Config *cfg, bool hw_framer) {
  if (mode_ == kModeUninit) {
    //This data plane class only works with RadioSoapySdr class
    auto *soapy_radio = dynamic_cast<RadioSoapySdr *>(radio);
    if ((soapy_radio != nullptr) && (radio != nullptr)) {
      cfg_ = cfg;
      radio_ = radio;
      hw_framer_ = hw_framer;
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

void RadioDataPlane::Activate(Radio::ActivationTypes type,
                              long long act_time_ns, size_t samples) {
  if (mode_ == kModeDeactive) {
    if ((radio_ == nullptr) || (remote_stream_ == nullptr)) {
      AGORA_LOG_ERROR(
          "Device pointer or remote stream is null in RadioDataPlane "
          "Activate");
    } else {
      auto *device = dynamic_cast<RadioSoapySdr *>(radio_)->SoapyDevice();
      int soapy_flags = 0;
      if (type == Radio::ActivationTypes::kActivateWaitTrigger) {
        soapy_flags |= SOAPY_SDR_WAIT_TRIGGER;
      } else if (act_time_ns != 0) {
        soapy_flags |= SOAPY_SDR_HAS_TIME;
      }

      // End burst if passing a number of samples
      if (samples > 0) {
        soapy_flags |= SOAPY_SDR_END_BURST;
      }
      const auto status = device->activateStream(remote_stream_, soapy_flags,
                                                 act_time_ns, samples);
      if (status < 0) {
        AGORA_LOG_WARN("Activate soapy stream with error status %d %s\n",
                       status, SoapySDR_errToStr(status));
      }
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
      auto *device = dynamic_cast<RadioSoapySdr *>(radio_)->SoapyDevice();
      const auto status = device->deactivateStream(remote_stream_);
      if (status < 0) {
        AGORA_LOG_WARN("Deactivate soapy rx stream with error status %d %s\n",
                       status, SoapySDR_errToStr(status));
      }
    }
    mode_ = kModeDeactive;
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
    AGORA_LOG_TRACE("RadioDataPlane::Closing stream for radio %s(%zu)\n",
                    radio_->SerialNumber().c_str(), radio_->Id());
    auto *device = dynamic_cast<RadioSoapySdr *>(radio_)->SoapyDevice();
    device->closeStream(remote_stream_);
    remote_stream_ = nullptr;
  }
  mode_ = kModeShutdown;
}

void RadioDataPlane::Setup(const SoapySDR::Kwargs &args) {
  if (mode_ == kModeShutdown) {
    auto *radio = dynamic_cast<RadioSoapySdr *>(radio_);
    auto *device = radio->SoapyDevice();
    remote_stream_ = device->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16,
                                         radio->EnabledChannels(), args);
    if (remote_stream_ == nullptr) {
      AGORA_LOG_ERROR(
          "RadioDataPlane::Setup - SetupStream returned an error\n");
      throw std::runtime_error(
          "RadioDataPlane::Setup - SetupStream returned an error");
    }
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