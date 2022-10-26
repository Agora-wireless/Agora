/**
 * @file radio_data_plane_soapy.cc
 * @brief Defination file for the RadioDataPlaneSoapy Class
 */

#include "radio_data_plane_soapy.h"

#include "SoapySDR/Time.hpp"
#include "logger.h"
#include "radio_soapysdr.h"

static constexpr bool kDebugPrintRx = false;

RadioDataPlaneSoapy::RadioDataPlaneSoapy() = default;

void RadioDataPlaneSoapy::Init(Radio* radio, const Config* cfg,
                               bool hw_framer) {
  return RadioDataPlane::Init(radio, cfg, hw_framer);
}

inline void RadioDataPlaneSoapy::Activate(Radio::ActivationTypes type,
                                          long long act_time_ns,
                                          size_t samples) {
  return RadioDataPlane::Activate(type, act_time_ns, samples);
}

inline void RadioDataPlaneSoapy::Deactivate() {
  return RadioDataPlane::Deactivate();
}

inline void RadioDataPlaneSoapy::Close() { return RadioDataPlane::Close(); }

inline void RadioDataPlaneSoapy::Setup() {
  SoapySDR::Kwargs sargs;
  //Helps with the disable stream, errors (-2)
  sargs["SYNC_ACTIVATE"] = "false";
  return RadioDataPlane::Setup(sargs);
}

int RadioDataPlaneSoapy::Rx(
    std::vector<std::vector<std::complex<int16_t>>>& rx_data, size_t rx_size,
    Radio::RxFlags& out_flags, long long& rx_time_ns) {
  std::vector<void*> rx_locations;
  rx_locations.reserve(rx_data.size());

  for (auto& buff : rx_data) {
    rx_locations.emplace_back(buff.data());
  }
  return RadioDataPlaneSoapy::Rx(rx_locations, rx_size, out_flags, rx_time_ns);
}

int RadioDataPlaneSoapy::Rx(
    std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs, size_t rx_size,
    Radio::RxFlags& out_flags, long long& rx_time_ns) {
  std::vector<void*> rx_locations;
  rx_locations.reserve(rx_buffs.size());

  for (auto& buff : rx_buffs) {
    rx_locations.emplace_back(buff->data());
  }
  return RadioDataPlaneSoapy::Rx(rx_locations, rx_size, out_flags, rx_time_ns);
}

int RadioDataPlaneSoapy::Rx(std::vector<void*>& rx_locations, size_t rx_size,
                            Radio::RxFlags& out_flags, long long& rx_time_ns) {
  static constexpr long kRxTimeout = 1;  // 1uS
  out_flags = Radio::RxFlags::kRxFlagNone;
  // SOAPY_SDR_ONE_PACKET; SOAPY_SDR_END_BURST
  //Magic number for soapy driver code to ignore tdd framing logic
  int soapy_rx_flags = (1 << 29);

  int rx_status = 0;
  long long frame_time_ns(0);
  auto* device = dynamic_cast<RadioSoapySdr*>(radio_)->SoapyDevice();

  rx_status = device->readStream(remote_stream_, rx_locations.data(), rx_size,
                                 soapy_rx_flags, frame_time_ns, kRxTimeout);

  if (rx_status > 0) {
    const size_t rx_samples = static_cast<size_t>(rx_status);

    if ((soapy_rx_flags & SOAPY_SDR_HAS_TIME) == 0) {
      AGORA_LOG_WARN("RadioDataPlaneSoapy::Rx %s(%zu) - does not have time",
                     radio_->SerialNumber().c_str(), radio_->Id());
    }
    //if end burst flag is not set, then we have partial data (hw_framer mode only)
    if (HwFramer()) {
      if (rx_samples != rx_size) {
        if ((soapy_rx_flags & SOAPY_SDR_END_BURST) == 0) {
          AGORA_LOG_TRACE(
              "RadioDataPlaneSoapy::Rx %s(%zu) - short rx call %zu:%zu, more "
              "data could be available? %d\n",
              radio_->SerialNumber().c_str(), radio_->Id(), rx_samples, rx_size,
              soapy_rx_flags);
          //Soapy could print a 'D' if this happens. But this would be acceptable
        } else if ((soapy_rx_flags & SOAPY_SDR_END_BURST) ==
                   SOAPY_SDR_END_BURST) {
          AGORA_LOG_WARN(
              "RadioDataPlaneSoapy::Rx %s(%zu) - short rx call %zu:%zu end "
              "of the receive data status %d\n",
              radio_->SerialNumber().c_str(), radio_->Id(), rx_samples, rx_size,
              soapy_rx_flags);
          out_flags = Radio::RxFlags::kEndReceive;
        } else if (((frame_time_ns & 0xFFFF) + rx_samples) >=
                   Configuration()->SampsPerSymbol()) {
          //Hackish way to determine if we should proceed
          //The first symbol bug appears to periodically cut off a few samples from the first symbol
          AGORA_LOG_WARN(
              "RadioDataPlaneSoapy::Rx %s(%zu) - short rx call %zu:%zu "
              "samples contained are the last of a symbol %lld:%zu with flags "
              "%d\n",
              radio_->SerialNumber().c_str(), radio_->Id(), rx_samples, rx_size,
              ((frame_time_ns & 0xFFFF) + rx_samples),
              Configuration()->SampsPerSymbol(), soapy_rx_flags);
          out_flags = Radio::RxFlags::kEndReceive;
        }
      } else {
        /// rx_samples == rx_size
        if ((soapy_rx_flags & SOAPY_SDR_END_BURST) == 0) {
          //This usually happens when the timeout is not long enough to wait for multiple packets for a given requested rx length
          AGORA_LOG_WARN(
              "RadioDataPlaneSoapy::Rx - expected SOAPY_SDR_END_BURST but "
              "didn't happen samples count %zu requested %zu symbols with "
              "flags %d at time %lld\n",
              rx_samples, rx_size, soapy_rx_flags, frame_time_ns);
        } else {
          AGORA_LOG_TRACE(
              "RadioDataPlaneSoapy::Rx - good rx samples count %zu requested "
              "%zu symbols with flags %d at time %lld\n",
              rx_samples, rx_size, soapy_rx_flags, frame_time_ns);
        }
      }

      if ((soapy_rx_flags & SOAPY_SDR_MORE_FRAGMENTS) ==
          SOAPY_SDR_MORE_FRAGMENTS) {
        AGORA_LOG_WARN(
            "RadioDataPlaneSoapy::Rx - fragments remaining on rx call for "
            "sample count %zu requested %zu symbols with flags %d at time "
            "%lld\n",
            rx_samples, rx_size, soapy_rx_flags, frame_time_ns);
      }
      rx_time_ns = frame_time_ns;
    } else {
      // for UHD device (or software framer) recv using ticks
      rx_time_ns =
          SoapySDR::timeNsToTicks(frame_time_ns, Configuration()->Rate());
    }

    if (kDebugPrintRx) {
      AGORA_LOG_INFO(
          "Rx Radio %s(%zu) RadioSoapy RX return count %d out of "
          "requested %zu - flags: %d - HAS TIME: %d | END BURST: %d | MORE "
          "FRAGS: %d | END_ABRUPT: %d | SINGLE PKT: %d | received at "
          "%lld:%lld (Frame %zu, Symbol %zu)\n",
          radio_->SerialNumber().c_str(), radio_->Id(), rx_status, rx_size,
          soapy_rx_flags,
          static_cast<int>((soapy_rx_flags & SOAPY_SDR_HAS_TIME) ==
                           SOAPY_SDR_HAS_TIME),
          static_cast<int>((soapy_rx_flags & SOAPY_SDR_END_BURST) ==
                           SOAPY_SDR_END_BURST),
          static_cast<int>((soapy_rx_flags & SOAPY_SDR_MORE_FRAGMENTS) ==
                           SOAPY_SDR_MORE_FRAGMENTS),
          static_cast<int>((soapy_rx_flags & SOAPY_SDR_END_ABRUPT) ==
                           SOAPY_SDR_END_ABRUPT),
          static_cast<int>((soapy_rx_flags & SOAPY_SDR_ONE_PACKET) ==
                           SOAPY_SDR_ONE_PACKET),
          rx_time_ns, frame_time_ns, static_cast<size_t>(frame_time_ns >> 32),
          static_cast<size_t>((frame_time_ns >> 16) & 0xFFFF));
    }

    if (kDebugRadioRX) {
      if (rx_status == static_cast<int>(rx_size)) {
        AGORA_LOG_INFO("Radio %s(%zu) received %d:%zu flags: %d MTU %zu\n",
                       radio_->SerialNumber().c_str(), radio_->Id(), rx_status,
                       Configuration()->SampsPerSymbol(), (int)out_flags,
                       device->getStreamMTU(remote_stream_));
      } else {
        if ((rx_status != SOAPY_SDR_TIMEOUT) &&
            (out_flags == Radio::RxFlags::kEndReceive)) {
          AGORA_LOG_INFO(
              "Unexpected RadioRx return value %d from radio %s(%zu) flags: "
              "%d\n",
              rx_status, radio_->SerialNumber().c_str(), radio_->Id(),
              out_flags);
        }
      }
    }
  } else if (rx_status == SOAPY_SDR_TIMEOUT) {
    /// If a timeout occurs tell the requester there are 0 bytes
    rx_status = 0;
  }
  return rx_status;
}

void RadioDataPlaneSoapy::Flush() {
  const long timeout_us(0);
  int flags = 0;
  long long frame_time(0);
  int r = 0;
  auto* device = dynamic_cast<RadioSoapySdr*>(radio_)->SoapyDevice();

  std::vector<std::vector<std::complex<int16_t>>> samples(
      kMaxChannels,
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