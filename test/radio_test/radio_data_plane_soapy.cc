/**
 * @file radio_data_plane_soapy.cc
 * @brief Defination file for the RadioDataPlaneSoapy Class
 */

#include "radio_data_plane_soapy.h"

#include "SoapySDR/Formats.h"
#include "SoapySDR/Time.hpp"
#include "logger.h"
#include "utils.h"

RadioDataPlaneSoapy::RadioDataPlaneSoapy()
    : RadioDataPlaneSoapy(nullptr, nullptr, 0) {}

RadioDataPlaneSoapy::RadioDataPlaneSoapy(const Config* cfg,
                                         SoapySDR::Device* device, size_t id)
    : radio_id_(id),
      mode_(kModeUninit),
      cfg_(cfg),
      device_(device),
      rx_stream_(nullptr) {
  if ((cfg != nullptr) && (device != nullptr)) {
    mode_ = kModeShutdown;
  }
}

RadioDataPlaneSoapy::~RadioDataPlaneSoapy() { Close(); }

void RadioDataPlaneSoapy::Init(const Config* cfg, SoapySDR::Device* device,
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

void RadioDataPlaneSoapy::Activate() {
  if (mode_ == kModeDeactive) {
    if ((device_ == nullptr) || (rx_stream_ == nullptr)) {
      AGORA_LOG_ERROR(
          "Device pointer or rx_stream is null in RadioDataPlaneSoapy "
          "Activate");
    } else {
      device_->activateStream(rx_stream_);
      mode_ = kModeActive;
    }
  } else {
    AGORA_LOG_WARN("Attempting to Activate data plane when in wrong state %d\n",
                   static_cast<int>(mode_));
  }
}

void RadioDataPlaneSoapy::Deactivate() {
  if (mode_ == kModeActive) {
    if ((device_ == nullptr) || (rx_stream_ == nullptr)) {
      AGORA_LOG_ERROR(
          "Device pointer or rx_stream is null in RadioDataPlaneSoapy "
          "Deactivate");
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

void RadioDataPlaneSoapy::Close() {
  if (device_ == nullptr) {
    AGORA_LOG_ERROR("Device pointer is null in RadioDataPlaneSoapy Close");
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

void RadioDataPlaneSoapy::Setup() {
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

// For now, radio rx will return 1 symbol
int RadioDataPlaneSoapy::Rx(
    std::vector<std::vector<std::complex<int16_t>>>& rx_data,
    long long& rx_time_ns) {
  constexpr long kRxTimeout = 1;  // 1uS
  // SOAPY_SDR_ONE_PACKET; SOAPY_SDR_END_BURST
  int rx_flags = SOAPY_SDR_END_BURST;
  int rx_status = 0;
  long long frame_time_ns(0);

  std::vector<void*> buffs;
  for (auto& ch_buff : rx_data) {
    buffs.emplace_back(ch_buff.data());
  }
  rx_status =
      device_->readStream(rx_stream_, buffs.data(), cfg_->SampsPerSymbol(),
                          rx_flags, frame_time_ns, kRxTimeout);

  if (cfg_->HwFramer() == true) {
    rx_time_ns = frame_time_ns;
  } else {
    // for UHD device recv using ticks
    rx_time_ns = SoapySDR::timeNsToTicks(frame_time_ns, cfg_->Rate());
  }

  if (kDebugRadioRX) {
    if (rx_status == static_cast<int>(cfg_->SampsPerSymbol())) {
      std::cout << "Radio " << radio_id_ << " received " << rx_status
                << " flags: " << rx_flags << " MTU "
                << device_->getStreamMTU(rx_stream_) << std::endl;
    } else {
      if (!((rx_status == SOAPY_SDR_TIMEOUT) && (rx_flags == 0))) {
        std::cout << "Unexpected RadioRx return value " << rx_status
                  << " from radio " << radio_id_ << " flags: " << rx_flags
                  << std::endl;
      }
    }
  }

  /// If a timeout occurs tell the user you received 0 bytes
  if (rx_status == SOAPY_SDR_TIMEOUT) {
    rx_status = 0;
  }
  return rx_status;
}

void RadioDataPlaneSoapy::Flush() {
  const long timeout_us(0);
  int flags = 0;
  long long frame_time(0);
  int r = 0;

  std::vector<std::vector<std::complex<int16_t>>> samples(
      cfg_->NumChannels(),
      std::vector<std::complex<int16_t>>(cfg_->SampsPerSymbol(),
                                         std::complex<int16_t>(0, 0)));
  std::vector<void*> ignore;
  for (auto& ch_buff : samples) {
    ignore.emplace_back(ch_buff.data());
  }

  while (r > 0) {
    r = device_->readStream(rx_stream_, ignore.data(), cfg_->SampsPerSymbol(),
                            flags, frame_time, timeout_us);
  }
}
