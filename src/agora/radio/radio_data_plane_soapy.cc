/**
 * @file radio_data_plane_soapy.cc
 * @brief Defination file for the RadioDataPlaneSoapy Class
 */

#include "radio_data_plane_soapy.h"

#include "SoapySDR/Time.hpp"

RadioDataPlaneSoapy::RadioDataPlaneSoapy()
    : RadioDataPlaneSoapy(nullptr, nullptr, 0) {}

RadioDataPlaneSoapy::RadioDataPlaneSoapy(const Config* cfg,
                                         SoapySDR::Device* device, size_t id)
    : RadioDataPlane(cfg, device, id) {}

void RadioDataPlaneSoapy::Init(const Config* cfg, SoapySDR::Device* device,
                               size_t id) {
  return RadioDataPlane::Init(cfg, device, id);
}

inline void RadioDataPlaneSoapy::Activate() {
  return RadioDataPlane::Activate();
}

inline void RadioDataPlaneSoapy::Deactivate() {
  return RadioDataPlane::Deactivate();
}

inline void RadioDataPlaneSoapy::Close() { return RadioDataPlane::Close(); }

inline void RadioDataPlaneSoapy::Setup() {
  SoapySDR::Kwargs sargs;
  return RadioDataPlane::Setup(sargs);
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
  buffs.reserve(rx_data.size());
  for (auto& ch_buff : rx_data) {
    buffs.emplace_back(ch_buff.data());
  }
  rx_status = device_->readStream(rx_stream_, buffs.data(),
                                  Configuration()->SampsPerSymbol(), rx_flags,
                                  frame_time_ns, kRxTimeout);

  if (Configuration()->HwFramer() == true) {
    rx_time_ns = frame_time_ns;
  } else {
    // for UHD device recv using ticks
    rx_time_ns =
        SoapySDR::timeNsToTicks(frame_time_ns, Configuration()->Rate());
  }

  if (kDebugRadioRX) {
    if (rx_status == static_cast<int>(Configuration()->SampsPerSymbol())) {
      std::cout << "Radio " << Id() << " received " << rx_status
                << " flags: " << rx_flags << " MTU "
                << device_->getStreamMTU(rx_stream_) << std::endl;
    } else {
      if (!((rx_status == SOAPY_SDR_TIMEOUT) && (rx_flags == 0))) {
        std::cout << "Unexpected RadioRx return value " << rx_status
                  << " from radio " << Id() << " flags: " << rx_flags
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
      Configuration()->NumChannels(),
      std::vector<std::complex<int16_t>>(Configuration()->SampsPerSymbol(),
                                         std::complex<int16_t>(0, 0)));
  std::vector<void*> ignore;
  ignore.reserve(samples.size());
  for (auto& ch_buff : samples) {
    ignore.emplace_back(ch_buff.data());
  }

  while (r > 0) {
    r = device_->readStream(rx_stream_, ignore.data(),
                            Configuration()->SampsPerSymbol(), flags,
                            frame_time, timeout_us);
  }
}
