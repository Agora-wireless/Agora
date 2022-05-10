/**
 * @file radio_data_plane_soapy.cc
 * @brief Defination file for the RadioDataPlaneSoapy Class
 */

#include "radio_data_plane_soapy.h"

#include "SoapySDR/Time.hpp"
#include "radio_soapysdr.h"

constexpr bool kDebugRx = false;

RadioDataPlaneSoapy::RadioDataPlaneSoapy() : RadioDataPlane() {}

void RadioDataPlaneSoapy::Init(Radio* radio, const Config* cfg) {
  return RadioDataPlane::Init(radio, cfg);
}

inline void RadioDataPlaneSoapy::Activate(Radio::ActivationTypes type) {
  return RadioDataPlane::Activate(type);
}

inline void RadioDataPlaneSoapy::Deactivate() {
  return RadioDataPlane::Deactivate();
}

inline void RadioDataPlaneSoapy::Close() { return RadioDataPlane::Close(); }

inline void RadioDataPlaneSoapy::Setup() {
  SoapySDR::Kwargs sargs;
  return RadioDataPlane::Setup(sargs);
}

int RadioDataPlaneSoapy::Rx(
    std::vector<std::vector<std::complex<int16_t>>>& rx_data, size_t rx_size,
    Radio::RxFlags rx_flags, long long& rx_time_ns) {
  std::vector<void*> rx_locations;
  rx_locations.reserve(rx_data.size());

  for (auto& buff : rx_data) {
    rx_locations.emplace_back(buff.data());
  }
  return RadioDataPlaneSoapy::Rx(rx_locations, rx_size, rx_flags, rx_time_ns);
}

int RadioDataPlaneSoapy::Rx(
    std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs, size_t rx_size,
    Radio::RxFlags rx_flags, long long& rx_time_ns) {
  std::vector<void*> rx_locations;
  rx_locations.reserve(rx_buffs.size());

  for (auto& buff : rx_buffs) {
    rx_locations.emplace_back(buff->data());
  }
  return RadioDataPlaneSoapy::Rx(rx_locations, rx_size, rx_flags, rx_time_ns);
}

int RadioDataPlaneSoapy::Rx(std::vector<void*>& rx_locations, size_t rx_size,
                            Radio::RxFlags rx_flags, long long& rx_time_ns) {
  constexpr long kRxTimeout = 1;  // 1uS
  //constexpr long kRxTimeout = 1000000;  // 1uS
  // SOAPY_SDR_ONE_PACKET; SOAPY_SDR_END_BURST
  int soapy_rx_flags = 0;
  if (rx_flags == Radio::RxFlagCompleteSymbol) {
    //Done rxing for a while?
    soapy_rx_flags = SOAPY_SDR_END_BURST;
  }
  if (rx_flags == Radio::RxFlagNone) {
    soapy_rx_flags = 0;
  }

  int rx_status = 0;
  long long frame_time_ns(0);
  auto device = dynamic_cast<RadioSoapySdr*>(radio_)->SoapyDevice();

  rx_status = device->readStream(remote_stream_, rx_locations.data(), rx_size,
                                 soapy_rx_flags, frame_time_ns, kRxTimeout);

  if (kDebugRx) {
    if ((rx_status > 0) || (soapy_rx_flags != 0)) {
      std::printf(
          "Soapy RX return flags: %d - HAS TIME: %d | END BURST: %d\n",
          soapy_rx_flags,
          (soapy_rx_flags & SOAPY_SDR_HAS_TIME) == SOAPY_SDR_HAS_TIME,
          (soapy_rx_flags & SOAPY_SDR_END_BURST) == SOAPY_SDR_END_BURST);
    }
  }

  if (Configuration()->HwFramer() == true) {
    rx_time_ns = frame_time_ns;
  } else {
    // for UHD device recv using ticks
    rx_time_ns =
        SoapySDR::timeNsToTicks(frame_time_ns, Configuration()->Rate());
  }

  if (kDebugRadioRX) {
    if (rx_status == static_cast<int>(Configuration()->SampsPerSymbol())) {
      std::cout << "Radio " << radio_->SerialNumber() << "(" << radio_->Id()
                << ") received " << rx_status << " flags: " << rx_flags
                << " MTU " << device->getStreamMTU(remote_stream_) << std::endl;
    } else {
      if (!((rx_status == SOAPY_SDR_TIMEOUT) && (rx_flags == 0))) {
        std::cout << "Unexpected RadioRx return value " << rx_status
                  << " from radio " << radio_->SerialNumber() << "("
                  << radio_->Id() << ") flags: " << rx_flags << std::endl;
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
  auto device = dynamic_cast<RadioSoapySdr*>(radio_)->SoapyDevice();

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
    r = device->readStream(remote_stream_, ignore.data(),
                           Configuration()->SampsPerSymbol(), flags, frame_time,
                           timeout_us);
  }
}
