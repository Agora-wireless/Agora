/** @file radio_uhdsdr.h
  * @brief Defination file for the RadioSoapySdr class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#include "radio_uhdsdr.h"

#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Logger.hpp"
#include "SoapySDR/Time.hpp"
#include "SoapySDR/Version.hpp"
#include "comms-lib.h"
#include "logger.h"
// #include "radio_data_plane_uhd.h"
#include "symbols.h"

#include "uhd/usrp/multi_usrp.hpp"
#include <uhd/utils/log_add.hpp>
#include <uhd/property_tree.hpp>
#include <uhd/version.hpp>

static constexpr size_t kSoapyMakeMaxAttempts = 3;
static constexpr bool kPrintRadioSettings = false;
static constexpr double kAttnMax = -18.0f;

//Soapy sdr version
static constexpr int kSoapyMajorMinAPI = 0;
static constexpr int kSoapyMinorMinAPI = 8;
static constexpr int kSoapyBuildMinAPI = 0;

//Iris driver version
static constexpr int kIrisDriverYearMinAPI = 2022;
static constexpr int kIrisDriverMonthMinAPI = 5;
static constexpr int kIrisDriverDayMinAPI = 0;
static constexpr int kIrisDriverRelMinAPI = 0;

// radio init time for UHD devices
static constexpr size_t kUhdInitTimeSec = 3;

RadioUHDSdr::RadioUHDSdr()
    : dev_(nullptr),
      rxs_(nullptr),
      txs_(nullptr) {

  std::cout<<"calling pure uhd version of radio constructor" << std::endl;
  AGORA_LOG_FRAME(
      "Create RadioSoapySdr Radio with SoapySDR - Lib Version: %s - API "
      "Version: %s - ABI Version: %s\n",
      soapy_version.c_str(), soapy_api.c_str(),
      SoapySDR::getABIVersion().c_str());

  //Reduce the soapy log level
  SoapySDR::setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_NOTICE);
  //SoapySDR::setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_SSI);
  //SoapySDR::setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_DEBUG);
}

RadioUHDSdr::~RadioUHDSdr() {
  AGORA_LOG_TRACE("Destroy RadioUHDSdr %s(%zu)\n", SerialNumber().c_str(),
                  Id());
  if ((dev_ != nullptr) && (rxs_ != nullptr) && (txs_ != nullptr)) {
    Close();
  }
}

void RadioUHDSdr::Close() {
  if (dev_ != nullptr) {
    AGORA_LOG_TRACE("Close RadioUHDSdr %s(%zu)\n", SerialNumber().c_str(),
                    Id());
    if (rxs_ != nullptr) {
      rxs_ = nullptr;
    }
    // as from Sounder, no need to delete the txs_, since it is deconstructed by default
    if (txs_ != nullptr) {
      txs_ = nullptr;
    }
    dev_ = nullptr;
    Radio::Close();
  }
}

void RadioUHDSdr::Init(const Config* cfg, size_t id,
                         const std::string& serial,
                         const std::vector<size_t>& enabled_channels,
                         bool hw_framer) {
  if (dev_ == nullptr) {
    AGORA_LOG_TRACE("Init RadioUHDSdr %s(%zu)\n", serial.c_str(), id);
    Radio::Init(cfg, id, serial, enabled_channels, hw_framer);

    // changed args from SoapySDR to two maps
    std::map<std::string, std::string> args;
    std::map<std::string, std::string> sargs;
    
    args["timeout"] = "1000000";
    args["driver"] = "uhd";
    args["addr"] = SerialNumber();

    for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
      try {
        dev_ = uhd::usrp::multi_usrp::make(args);
        break;
      } catch (const std::runtime_error& e) {
        const auto* message = e.what();
        AGORA_LOG_ERROR(
            "RadioUHDSdr::Init[%zu] - UHD error try %zu -- %s\n", id, tries,
            message);
      }
    }
    if (dev_ == nullptr) {
      AGORA_LOG_ERROR(
          "UHDSDR failed to locate the radio %s in %zu attempts\n",
          SerialNumber().c_str(), kSoapyMakeMaxAttempts);
      throw std::runtime_error("UHDSDR failed to locate the radio with id " +
                               SerialNumber());
    }

    std::stringstream device_info;
    // device_info << "Driver   = " << dev_->getDriverKey() << std::endl;
    // The UHD does not have appropriate function to do getDriverKey(), which the 
    // SoapyUHD way of doing it is to directly get info from args, so decided to get rid of this for now

    device_info << "Hardware = " << dev_->get_mboard_name() << std::endl;
    // commented out for now as not main funciton
      //Get the Ip address from the driver

    if (ip_address_.empty()) {
      AGORA_LOG_ERROR("UHD Device IP address not found");
      throw std::runtime_error("Device IP address not found");
    }

    if (kPrintRadioSettings) {
      auto clock_sources = dev_->get_clock_source(0);
      for ([[maybe_unused]] const auto& source : clock_sources) {
        AGORA_LOG_TRACE("Clock source %s\n", source.c_str());
      }

      auto time_sources = dev_->get_time_sources(0);
      for ([[maybe_unused]] const auto& source : time_sources) {
        AGORA_LOG_INFO("Time source %s\n", source.c_str());
      }

      // TODO
      // Did not consider so far, will add when main function is done

      // auto register_interfaces = dev_->listRegisterInterfaces();
      AGORA_LOG_INFO("%s\n", device_info.str().c_str());
    }

    if (rxs_ == nullptr) {
      AGORA_LOG_ERROR(
          "RadioUHDSdr::Init[%zu] - Radio rx control plane could not be "
          "configured\n",
          id);
    }

    auto clk_status = dev_->get_mboard_sensor("CLKBUFF_LOCKED").value;
    if (clk_status.compare("false") == 0) {
      AGORA_LOG_WARN("RadioUHDSdr::Init[%zu] clk_status %s\n", id,
                     clk_status.c_str());
    }

    // this is a UHD version of setting up stream:

    uhd::stream_args_t stream_args("sc16");
    stream_args.channels = enabled_channels;
    stream_args.args = sargs;

    rxs_ = dev_->get_rx_stream(stream_args);

    txs_ = dev_->get_tx_stream(stream_args);
  
    if (txs_ == nullptr || rxs_ == nullptr) {
      AGORA_LOG_ERROR(
          "RadioUHDSdr::Init[%zu] - Radio tx control plane could not be "
          "configured\n",
          id);
    }
  } else {
    AGORA_LOG_ERROR("RadioUHDSdr::Init[%zu] - Radio already has been init\n",
                    id);
  }
}

void RadioUHDSdr::Setup(const std::vector<double>& tx_gains,
                          const std::vector<double>& rx_gains) {
  AGORA_LOG_TRACE("Setup RadioUHD Sdr %s(%zu)\n", SerialNumber().c_str(),
                  Id());
  for (auto ch : EnabledChannels()) {
    dev_->set_rx_rate(cfg_->Rate(), ch);
    dev_->set_tx_rate(cfg_->Rate(), ch);
  }

  // use the TRX antenna port for both tx and rx
  for (auto ch : EnabledChannels()) {
    dev_->set_rx_antenna("RX2", ch);
    dev_->set_tx_antenna("TX/RX", ch);
  }

  for (const auto& ch : EnabledChannels()) {
    ///\todo Check to see if this is correct for kUseUhd == false (BwFilter)
    uhd::tune_request_t tr(cfg_->RadioRfFreq());
    tr.rf_freq_policy = uhd::tune_request_t::POLICY_NONE;
    tr.dsp_freq_policy = uhd::tune_request_t::POLICY_NONE;

    tr.rf_freq = cfg_->RadioRfFreq();
    tr.rf_freq_policy = uhd::tune_request_t::POLICY_MANUAL;

    uhd::tune_request_t tr1(0);
    tr1.rf_freq_policy = uhd::tune_request_t::POLICY_NONE;
    tr1.dsp_freq_policy = uhd::tune_request_t::POLICY_NONE;

    tr1.dsp_freq = 0;
    tr1.dsp_freq_policy = uhd::tune_request_t::POLICY_MANUAL;

    dev_->set_rx_freq(tr, ch);
    dev_->set_rx_freq(tr1, ch);
    dev_->set_tx_freq(tr, ch);
    dev_->set_tx_freq(tr1, ch);

      // kUseUHD
    dev_->set_rx_gain(rx_gains.at(ch), "PGA0", ch);
    dev_->set_tx_gain(tx_gains.at(ch), "PGA0", ch);
  }    // end for (const auto& ch : EnabledChannels())

  // for (auto ch : EnabledChannels()) {
  //   // Clients??
  //   // dev_->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
  //   // dev_->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
  //   if (kUseUHD == false) {
  //     dev_->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
  //   }
  // }

  auto clk_status = dev_->get_mboard_sensor("CLKBUFF_LOCKED").value;
  if (clk_status.compare("false") == 0) {
    AGORA_LOG_WARN("Radio - clk_status %s\n", clk_status.c_str());
  }
}

void RadioUHDSdr::Activate(Radio::ActivationTypes type, long long act_time_ns,
                             size_t samples) {
  (void)type;
  (void)act_time_ns;
  (void)samples;
  AGORA_LOG_TRACE(
      "Activate RadioUHDSdr %s(%zu) at time %lld with samples %zu and type "
      "%d\n",
      SerialNumber().c_str(), Id(), act_time_ns, samples,
      static_cast<int>(type));
  const bool is_ue = false;
  if (is_ue) {
    dev_->set_time_source("internal");
    dev_->set_clock_source("internal");
    uhd::time_spec_t time1 = uhd::time_spec_t::from_ticks(0, 1e9);
    dev_->set_time_unknown_pps(time1);
  } else {
    dev_->set_clock_source("external");
    dev_->set_time_source("external");
    uhd::time_spec_t time2 = uhd::time_spec_t::from_ticks(0, 1e9);
    dev_->set_time_next_pps(time2);
  }
  // Wait for pps sync pulse ??`
  //std::this_thread::sleep_for(std::chrono::seconds(kUhdInitTimeSec -1));
  //dev_->activateStream(rxs_, SOAPY_SDR_HAS_TIME, kUhdInitTimeSec * 1e9, 0);
  uhd::stream_cmd_t::stream_mode_t mode;
  const size_t numElems = 0;
  if (numElems == 0) {
    mode = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
  } else if ((SOAPY_SDR_HAS_TIME & SOAPY_SDR_END_BURST) != 0) {
    mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE;
  } else {
    mode = uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_MORE;
  }

  uhd::stream_cmd_t cmd(mode);
  cmd.stream_now = (SOAPY_SDR_HAS_TIME & SOAPY_SDR_HAS_TIME) == 0;
  cmd.time_spec = uhd::time_spec_t::from_ticks(kUhdInitTimeSec * 1e9, 1e9);
  cmd.num_samps = numElems;

  rxs_->issue_stream_cmd(cmd);

  // there is not issue_stream_cmd in uhd::tx_streamer
  const auto status = 0;

  if (status < 0) {
    AGORA_LOG_WARN("Activate UHD tx stream with status % d %s\n", status,
                    SoapySDR_errToStr(status));
  }
  
}

void RadioUHDSdr::Deactivate() {
  AGORA_LOG_TRACE("Deactivate RadioUHDSdr %s(%zu)\n", SerialNumber().c_str(),
                  Id());
  const std::string tdd_conf_str = "{\"tdd_enabled\":false}";

  uhd::stream_cmd_t::stream_mode_t mode;
  mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
  uhd::stream_cmd_t cmd(mode);
  int flags = 0; 
  cmd.stream_now = (flags & SOAPY_SDR_HAS_TIME) == 0;
  const long long timeNs = 0;
  cmd.time_spec = uhd::time_spec_t::from_ticks(timeNs, 1e9);
  rxs_->issue_stream_cmd(cmd);
  
  // there is not issue_stream_cmd in uhd::tx_streamer
  const auto status = 0;
  if (status < 0) {
    AGORA_LOG_WARN("Deactivate UHD tx stream with status %d %s\n", status,
                   SoapySDR_errToStr(status));
  }
  // Correlator(false);
}

int RadioUHDSdr::Tx(const void* const* tx_buffs, size_t tx_size,
                      Radio::TxFlags flags, long long& tx_time_ns) {
  constexpr size_t kTxTimeoutUs = 1000000;

  int soapy_flags;
  if (flags == Radio::TxFlags::kTxFlagNone) {
    soapy_flags = SOAPY_SDR_HAS_TIME;
  } else if (flags == Radio::TxFlags::kEndTransmit) {
    soapy_flags = (SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST);
  } else if (flags == Radio::TxFlags::kTxWaitTrigger) {
    soapy_flags = SOAPY_SDR_END_BURST | SOAPY_SDR_WAIT_TRIGGER;
  } else {
    AGORA_LOG_ERROR("Unsupported radio tx flag %d\n", static_cast<int>(flags));
    soapy_flags = 0;
  }

  if (HwFramer() == false) {
    // For UHD device xmit from host using frameTimeNs
    tx_time_ns = SoapySDR::ticksToTimeNs(tx_time_ns, cfg_->Rate());
  }

  uhd::tx_streamer::sptr stream = txs_;
  uhd::tx_metadata_t md;
  md.has_time_spec = (soapy_flags & SOAPY_SDR_HAS_TIME) != 0;
  md.end_of_burst = (soapy_flags & SOAPY_SDR_END_BURST) != 0;
  md.time_spec = uhd::time_spec_t::from_ticks(tx_time_ns, 1e9);

  uhd::tx_streamer::buffs_type stream_buffs(tx_buffs, stream->get_num_channels());
  int ret = stream->send(stream_buffs, tx_size, md,kTxTimeoutUs/1e6);
  soapy_flags = 0;
  // const int write_status = dev_->writeStream(
  //     txs_, tx_buffs, tx_size, soapy_flags, tx_time_ns, kTxTimeoutUs);  
  const int write_status = ret;

  if (kDebugRadioTX) {
    // size_t chan_mask;
    // long timeout_us(0);
    int status_flag = 0;
    
    uhd::async_metadata_t md_debug;
    // int s;
    // const int s = dev_->readStreamStatus(txs_, chan_mask, status_flag,
    //                                      tx_time_ns, timeout_us);
    // if (not stream->recv_async_msg(md_debug, timeout_us/1e6)){
    //   s = SOAPY_SDR_TIMEOUT
    // }
    // chan_mask = (1 << md_debug.channel);
    status_flag = 0;
    if (md.has_time_spec) status_flag |= SOAPY_SDR_HAS_TIME;
    tx_time_ns = md_debug.time_spec.to_ticks(1e9);

    // switch (md_debug.event_code)
    //     {
    //     case uhd::async_metadata_t::EVENT_CODE_BURST_ACK: status_flag |= SOAPY_SDR_END_BURST; break;
    //     case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW: s = -7;
    //     case uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR: s = -3;
    //     case uhd::async_metadata_t::EVENT_CODE_TIME_ERROR: s = -6;
    //     case uhd::async_metadata_t::EVENT_CODE_UNDERFLOW_IN_PACKET: s = -7;
    //     case uhd::async_metadata_t::EVENT_CODE_SEQ_ERROR_IN_BURST: s = -3;
    //     case uhd::async_metadata_t::EVENT_CODE_USER_PAYLOAD: break;
    //     }

    // std::cout << "radio " << SerialNumber() << "(" << Id() << ") tx returned "
    //           << write_status << " and status " << s << " when flags was "
    //           << flags << std::endl;
  }
  return write_status;
}

int RadioUHDSdr::Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
                      size_t rx_size, Radio::RxFlags& out_flags,
                      long long& rx_time_ns) {
  std::vector<void*> rx_locations;
  rx_locations.reserve(rx_data.size());

  for (auto& buff : rx_data) {
    rx_locations.emplace_back(buff.data());
    }
  return RadioUHDSdr::Rx(rx_locations, rx_size, out_flags, rx_time_ns);
}

int RadioUHDSdr::Rx(
    std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs, size_t rx_size,
    Radio::RxFlags& out_flags, long long& rx_time_ns) {
  std::vector<void*> rx_locations;
  rx_locations.reserve(rx_buffs.size());

  for (auto& buff : rx_buffs) {
    rx_locations.emplace_back(buff->data());
  }
  return RadioUHDSdr::Rx(rx_locations, rx_size, out_flags, rx_time_ns);
}

int RadioUHDSdr::Rx(std::vector<void*>& rx_locs, size_t rx_size,
                      Radio::RxFlags& out_flags, long long& rx_time_ns) {
  static constexpr long kRxTimeout = 1;  // 1uS
  out_flags = Radio::RxFlags::kRxFlagNone;
  int soapy_rx_flags = (1 << 29);

  int rx_status = 0;
  long long frame_time_ns(0);
  // uhd::usrp::multi_usrp::sptr device = dynamic_cast<RadioUHDSdr*>(radio_)->UHDDevice();

  uhd::rx_metadata_t md;
  uhd::rx_streamer::buffs_type stream_buffs(rx_locs.data(), rxs_->get_num_channels());
  rx_status = rxs_->recv(stream_buffs, rx_size, md, kRxTimeout/1e6, (soapy_rx_flags & SOAPY_SDR_ONE_PACKET) != 0);

  soapy_rx_flags = 0;
  if (md.has_time_spec) soapy_rx_flags |= SOAPY_SDR_HAS_TIME;
  if (md.end_of_burst) soapy_rx_flags |= SOAPY_SDR_END_BURST;
  if (md.more_fragments) soapy_rx_flags |= SOAPY_SDR_MORE_FRAGMENTS;
  frame_time_ns = md.time_spec.to_ticks(1e9);

  if (rx_status > 0) {
    const size_t rx_samples = static_cast<size_t>(rx_status);

    if ((soapy_rx_flags & SOAPY_SDR_HAS_TIME) == 0) {
      AGORA_LOG_WARN("RadioUHDSdr::Rx - does not have time");
    }
    //if end burst flag is not set, then we have partial data (hw_framer mode only)
    if (HwFramer()) {
      if (rx_samples != rx_size) {
        if ((soapy_rx_flags & SOAPY_SDR_END_BURST) == 0) {
          AGORA_LOG_TRACE(
              "RadioUHDSdr::Rx - short rx call");
          //Soapy could print a 'D' if this happens. But this would be acceptable
        } else if ((soapy_rx_flags & SOAPY_SDR_END_BURST) ==
                   SOAPY_SDR_END_BURST) {
          AGORA_LOG_WARN(
              "RadioDataPlane_UHD::Rx");
          out_flags = Radio::RxFlags::kEndReceive;
        } else if (((frame_time_ns & 0xFFFF) + rx_samples) >=
                   cfg_->SampsPerSymbol()) {
          //Hackish way to determine if we should proceed
          //The first symbol bug appears to periodically cut off a few samples from the first symbol
          AGORA_LOG_WARN(
              "RadioDataPlane_UHD::Rx");
          out_flags = Radio::RxFlags::kEndReceive;
        }
      } else {
        /// rx_samples == rx_size
        if ((soapy_rx_flags & SOAPY_SDR_END_BURST) == 0) {
          //This usually happens when the timeout is not long enough to wait for multiple packets for a given requested rx length
          AGORA_LOG_WARN(
              "RadioDataPlane_UHD::Rx - expected SOAPY_SDR_END_BURST but "
              "didn't happen samples count %zu requested %zu symbols with "
              "flags %d\n",
              rx_samples, rx_size, soapy_rx_flags);
        }
      }

      if ((soapy_rx_flags & SOAPY_SDR_MORE_FRAGMENTS) ==
          SOAPY_SDR_MORE_FRAGMENTS) {
        AGORA_LOG_WARN(
            "RadioDataPlane_UHD::Rx - fragments remaining on rx call for "
            "sample count %zu requested %zu symbols with flags %d\n",
            rx_samples, rx_size, soapy_rx_flags);
      }
      rx_time_ns = frame_time_ns;
    } else {
      // for UHD device (or software framer) recv using ticks
      rx_time_ns =
          SoapySDR::timeNsToTicks(frame_time_ns, cfg_->Rate());
    }

    // if (kDebugPrintRx) {
    //   std::printf(
    //       "Rx Radio UHD %s(%zu) RadioUHD RX return count %d out of "
    //       "requested %zu - flags: %d - HAS TIME: %d | END BURST: %d | MORE "
    //       "FRAGS: %d | SINGLE PKT: %d\n",
    //       radio_->SerialNumber().c_str(), radio_->Id(), rx_status, rx_size,
    //       soapy_rx_flags,
    //       static_cast<int>((soapy_rx_flags & SOAPY_SDR_HAS_TIME) ==
    //                        SOAPY_SDR_HAS_TIME),
    //       static_cast<int>((soapy_rx_flags & SOAPY_SDR_END_BURST) ==
    //                        SOAPY_SDR_END_BURST),
    //       static_cast<int>((soapy_rx_flags & SOAPY_SDR_MORE_FRAGMENTS) ==
    //                        SOAPY_SDR_MORE_FRAGMENTS),
    //       static_cast<int>((soapy_rx_flags & SOAPY_SDR_ONE_PACKET) ==
    //                        SOAPY_SDR_ONE_PACKET));
    // }

    // if (kDebugRadioRX) {
    //   if (rx_status == static_cast<int>(cfg_->SampsPerSymbol())) {
    //     std::cout << "Radio UHD " << radio_->SerialNumber() << "(" << radio_->Id()
    //               << ") received " << rx_status << " flags: " << out_flags
    //               << " MTU " << rxs_>get_max_num_samps()
    //               << std::endl;
    //   } else {
    //     if (!((rx_status == SOAPY_SDR_TIMEOUT) && (out_flags == 0))) {
    //       std::cout << "Unexpected RadioRx return value " << rx_status
    //                 << " from radio " << radio_->SerialNumber() << "("
    //                 << radio_->Id() << ") flags: " << out_flags << std::endl;
    //     }
    //   }
    // }
  } else if (rx_status == SOAPY_SDR_TIMEOUT) {
    /// If a timeout occurs tell the requester there are 0 bytes
    rx_status = 0;
  }
  return rx_status;
}

void RadioUHDSdr::Trigger() {}

void RadioUHDSdr::ReadSensor() const {
  std::stringstream print_message;
  print_message << "TEMPs on Iris " << SerialNumber() << "(" << Id() << ")"
                << std::endl;
  print_message << "ZYNQ_TEMP  : " << dev_->get_mboard_sensor("ZYNQ_TEMP").value
                << std::endl;
  print_message << "LMS7_TEMP  : " << dev_->get_mboard_sensor("LMS7_TEMP").value
                << std::endl;
  print_message << "FE_TEMP  : " << dev_->get_mboard_sensor("FE_TEMP").value << std::endl;
  print_message << "TX0 TEMP  : " << dev_->get_tx_sensor("TEMP", 0).value
                << std::endl;
  print_message << "TX1 TEMP  : " << dev_->get_tx_sensor("TEMP", 1).value
                << std::endl;
  print_message << "RX0 TEMP  : " << dev_->get_rx_sensor("TEMP", 0).value
                << std::endl;
  print_message << "RX1 TEMP  : " << dev_->get_rx_sensor("TEMP", 1).value
                << std::endl;
  AGORA_LOG_INFO("%s\n", print_message.str().c_str());
}

void RadioUHDSdr::SetTimeAtTrigger(long long time_ns) {
  uhd::time_spec_t time = uhd::time_spec_t::from_ticks(0, 1e9);
  dev_->set_time_now(time);

  auto time_dev = dev_->get_time_now().to_ticks(1e9);
  if (time_dev != time_ns) {
    AGORA_LOG_WARN(
        "RadioUHDSdr::SetTimeAtTrigger[%zu] the hardware trigger time is "
        "not the expected value %lld : %lld\n",
        Id(), time_dev, time_ns);
  }
  auto clk_status = dev_->get_mboard_sensor("CLKBUFF_LOCKED").value;
  if (clk_status.compare("false") == 0) {
    AGORA_LOG_WARN("RadioUHDSdr::Activate[%zu] clk_status %s\n", Id(),
                   clk_status.c_str());
  }
}

long long RadioUHDSdr::GetTimeNs() {
  auto clk_status = dev_->get_mboard_sensor("CLKBUFF_LOCKED").value;
  if (clk_status.compare("false") == 0) {
    AGORA_LOG_WARN("RadioUHDSdr::Activate[%zu] clk_status %s\n", Id(),
                   clk_status.c_str());
  }

  auto time_dev = dev_->get_time_now().to_ticks(1e9);
  AGORA_LOG_TRACE("RadioUHDSdr::GetTimeNs[%zu] the hardware time is %lld\n",
                  Id(), time_dev);
  return time_dev;
}

void RadioUHDSdr::PrintSettings() const {
  std::stringstream print_message;

  SoapySDR::Kwargs out;
  for (size_t i = 0; i < dev_->get_tx_num_channels(); i++)
  {
    const uhd::dict<std::string, std::string> info = dev_->get_usrp_tx_info(i);
    for (const std::string &key : info.keys())
    {
        if (key.size() > 3 and key.substr(0, 3) == "tx_")
            out[str(boost::format("tx%d_%s") % i % key.substr(3))] = info[key];
        else out[key] = info[key];
    }
  }
  for (size_t i = 0; i < dev_->get_rx_num_channels(); i++)
  {
    const uhd::dict<std::string, std::string> info = dev_->get_usrp_rx_info(i);
    for (const std::string &key : info.keys())
    {
        if (key.size() > 3 and key.substr(0, 3) == "rx_")
            out[str(boost::format("rx%d_%s") % i % key.substr(3))] = info[key];
        else out[key] = info[key];
    }
  }

  // uhd::property_tree::sptr tree = _get_tree();
  // if (tree->exists("/mboards/0/fw_version")) out["fw_version"] = tree->access<std::string>("/mboards/0/fw_version").get();
  // if (tree->exists("/mboards/0/fpga_version")) out["fpga_version"] = tree->access<std::string>("/mboards/0/fpga_version").get();

  SoapySDR::Kwargs hw_info = out;

  std::string label = hw_info["revision"];
  std::string frontend = "";
  if (hw_info.find("frontend") != hw_info.end()) {
    frontend = hw_info["frontend"];
  }

  print_message << SerialNumber() << " (" << Id() << ") - " << label << " - "
                << frontend << std::endl;

  if (kPrintRadioSettings) {
    for (const auto& c : EnabledChannels()) {
      if (c < dev_->get_rx_num_channels()) {
        print_message << "RX Channel " << c << std::endl
                      << "Actual RX sample rate: "
                      << (dev_->get_rx_rate(c) / 1e6)
                      << "MSps..." << std::endl
                      << "Actual RX frequency: "
                      << (dev_->get_rx_freq(c) / 1e9) << "GHz..."
                      << std::endl
                      << "Actual RX gain: " << (dev_->get_rx_gain(c))
                      << "..." << std::endl;
        print_message << "Actual RX bandwidth: "
                      << (dev_->get_rx_bandwidth(c) / 1e6) << "M..."
                      << std::endl
                      << "Actual RX antenna: "
                      << (dev_->get_rx_antenna(c)) << "..."
                      << std::endl;
      }
    }

    for (auto c : EnabledChannels()) {
      if (c < dev_->get_tx_num_channels()) {
        print_message << "TX Channel " << c << std::endl
                      << "Actual TX sample rate: "
                      << (dev_->get_tx_rate(c) / 1e6)
                      << "MSps..." << std::endl
                      << "Actual TX frequency: "
                      << (dev_->get_tx_freq(c) / 1e9) << "GHz..."
                      << std::endl
                      << "Actual TX gain: " << (dev_->get_tx_gain(c))
                      << "..." << std::endl;
        print_message << "Actual TX bandwidth: "
                      << (dev_->get_tx_bandwidth(c) / 1e6) << "M..."
                      << std::endl
                      << "Actual TX antenna: "
                      << (dev_->get_tx_antenna(c)) << "... "
                      << std::endl;
      }
    }
    AGORA_LOG_INFO("%s\n", print_message.str().c_str());
  }
}

void RadioUHDSdr::ClearSyncDelay() {}

void RadioUHDSdr::ConfigureTddModeBs(bool is_ref_radio, size_t radio_id) {
  nlohmann::json conf;
  conf["tdd_enabled"] = true;
  conf["frame_mode"] = "free_running";
  conf["max_frame"] = 0;
  conf["symbol_size"] = cfg_->SampsPerSymbol();
  conf["beacon_start"] = cfg_->OfdmTxZeroPrefix();
  conf["beacon_stop"] = cfg_->OfdmTxZeroPrefix() + cfg_->BeaconLen();

  // experimentally good value for dev front-end
  std::vector<std::string> tdd_sched;

  std::string sched = cfg_->Frame().FrameIdentifier();
  size_t sched_size = sched.length();
  for (size_t s = 0; s < sched_size; s++) {
    char c = cfg_->Frame().FrameIdentifier().at(s);
    if (c == 'C') {
      sched.replace(s, 1, is_ref_radio ? "R" : "T");
    } else if (c == 'L') {
      sched.replace(s, 1, is_ref_radio ? "T" : "R");
    } else if (c == 'P') {
      sched.replace(s, 1, "R");
    } else if (c == 'U') {
      sched.replace(s, 1, "R");
    } else if (c == 'D') {
      sched.replace(s, 1, "T");
    } else if (c != 'B') {
      sched.replace(s, 1, "G");
    }
  }
  std::cout << "Radio " << Id() << " Frame 1: " << sched << std::endl;
  tdd_sched.push_back(sched);

  conf["frames"] = tdd_sched;
  std::string conf_string = conf.dump();

  size_t ndx = 0;
  for (const auto& channel : EnabledChannels()) {
    const bool is_beacon_antenna =
        !cfg_->Beamsweep() &&
        radio_id * cfg_->NumChannels() + ndx == cfg_->BeaconAnt();
    std::vector<unsigned> beacon_weights(
        cfg_->NumRadios() * cfg_->NumChannels(), is_beacon_antenna ? 1 : 0);
    if (cfg_->Beamsweep()) {
      for (size_t j = 0; j < beacon_weights.size(); j++) {
        beacon_weights.at(j) = CommsLib::Hadamard2(ndx, j);
      }
    }

    char channel_letter;
    if (channel == 0) {
      channel_letter = 'A';
    } else if (channel == 1) {
      channel_letter = 'B';
    } else {
      AGORA_LOG_ERROR("Unsupported channel %zu\n", channel);
      throw std::runtime_error("Unsupported channel");
    }
    std::string prog_reg = "BEACON_RAM_WGT_";
    prog_reg.push_back(channel_letter);
    ++ndx;
  }
}

void RadioUHDSdr::ConfigureTddModeUe() {
  nlohmann::json conf;
  conf["tdd_enabled"] = true;
  conf["frame_mode"] = "continuous_resync";
  const int max_frame =
      (int)(2.0f / ((cfg_->SampsPerSymbol() * cfg_->Frame().NumTotalSyms()) /
                    cfg_->Rate()));
  conf["max_frame"] = max_frame;
  conf["dual_pilot"] = (cfg_->NumUeChannels() == 2);
  auto tdd_sched = cfg_->Frame().FrameIdentifier();
  for (size_t s = 0; s < cfg_->Frame().FrameIdentifier().length(); s++) {
    char c = cfg_->Frame().FrameIdentifier().at(s);
    if (c == 'B') {
      tdd_sched.replace(s, 1, "R");  // Dummy RX used in PHY scheduler
    } else if (c == 'P' and ((cfg_->NumUeChannels() == 1 and
                              cfg_->Frame().GetPilotSymbol(Id()) != s) or
                             (cfg_->NumUeChannels() == 2 and
                              (cfg_->Frame().GetPilotSymbol(2 * Id()) != s and
                               cfg_->Frame().GetPilotSymbol(Id() * 2 + 1) !=
                                   s)))) {  // TODO: change this for
      // orthogonal pilots
      tdd_sched.replace(s, 1, "G");
    } else if (c == 'U') {
      tdd_sched.replace(s, 1, "T");
    } else if (c == 'D') {
      tdd_sched.replace(s, 1, "R");
    } else if (c != 'P') {
      tdd_sched.replace(s, 1, "G");
    }
  }
  std::cout << "UE " << SerialNumber() << "(" << Id()
            << ") Frame: " << tdd_sched << std::endl;
  std::vector<std::string> jframes;
  jframes.push_back(tdd_sched);
  conf["frames"] = jframes;
  conf["symbol_size"] = cfg_->SampsPerSymbol();
  std::string conf_string = conf.dump();
  // beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) +
  // 82 (Client FE Delay)
  const int cl_trig_offset = cfg_->BeaconLen() + 249;
  const int sf_start = cl_trig_offset / cfg_->SampsPerSymbol();
  const int sp_start = cl_trig_offset % cfg_->SampsPerSymbol();
  // dev_->setHardwareTime(
  //     SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, cfg_->Rate()),
  //     "TRIGGER");
  long long timeinput = SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, cfg_->Rate());
  uhd::time_spec_t time = uhd::time_spec_t::from_ticks(timeinput, 1e9);
  dev_->set_time_now(time);


  // experimentally good value for dev front-end
  for (const auto& channel : EnabledChannels()) {
    char channel_letter;
    if (channel == 0) {
      channel_letter = 'A';
    } else if (channel == 1) {
      channel_letter = 'B';
    } else {
      AGORA_LOG_ERROR("Unsupported channel %zu\n", channel);
      throw std::runtime_error("Unsupported channel");
    }
    std::string prog_reg = "TX_RAM_";
    prog_reg.push_back(channel_letter);
  }
  // Correlator(true);
}

// void RadioUHDSdr::Correlator(bool enable) {
//   if (enable) {
//     if (correlator_enabled_ == false) {
//       correlator_enabled_ = true;
//       const std::string corr_conf =
//           R"({"corr_enabled":true,"corr_threshold":)" + std::to_string(1) + "}";
//     }
//   } else if (correlator_enabled_ == true) {
//     correlator_enabled_ = false;
//     const std::string corr_conf = "{\"corr_enabled\":false}";
//   }
// }

void RadioUHDSdr::Flush() {
  AGORA_LOG_INFO("Flushing radio %zu rx data plane\n", Id());
}
