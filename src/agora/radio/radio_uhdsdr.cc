/** @file radio_uhdsdr.h
  * @brief Defination file for the RadioSoapySdr class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#include "radio_uhdsdr.h"

#include <uhd/property_tree.hpp>
#include <uhd/utils/log_add.hpp>
#include <uhd/version.hpp>

#include "comms-lib.h"
#include "logger.h"
#include "symbols.h"
#include "uhd/usrp/multi_usrp.hpp"

static constexpr size_t kMakeMaxAttempts = 3;
static constexpr bool kPrintRadioSettings = true;

// radio init time for UHD devices
// increase the wait time for radio init to get rid of the late packet issue
static constexpr double kUhdInitTimeSec = 1.0;

RadioUHDSdr::RadioUHDSdr() : dev_(nullptr), rxs_(nullptr), txs_(nullptr) {
  AGORA_LOG_INFO("calling pure uhd version of radio constructor.\n");
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

void RadioUHDSdr::Init(const Config* cfg, size_t id, const std::string& serial,
                       const std::vector<size_t>& enabled_channels,
                       bool hw_framer) {
  if (dev_ == nullptr) {
    AGORA_LOG_TRACE("Init RadioUHDSdr %s(%zu)\n", serial.c_str(), id);
    Radio::Init(cfg, id, serial, enabled_channels, hw_framer);

    std::map<std::string, std::string> args;
    std::map<std::string, std::string> sargs;

    args["timeout"] = "1000000";
    args["driver"] = "uhd";
    args["addr"] = SerialNumber();
    //Need to make sure MTU is acceptable of this (assume 32 bit/4 byte samples)
    const std::string frame_size = std::to_string(cfg->SampsPerSymbol() * 4);
    args["send_frame_size"] = frame_size;
    args["recv_frame_size"] = frame_size;

    for (size_t tries = 0; tries < kMakeMaxAttempts; tries++) {
      try {
        dev_ = uhd::usrp::multi_usrp::make(args);
        break;
      } catch (const std::runtime_error& e) {
        const auto* message = e.what();
        AGORA_LOG_ERROR("RadioUHDSdr::Init[%zu] - UHD error try %zu -- %s\n",
                        id, tries, message);
      }
    }
    if (dev_ == nullptr) {
      AGORA_LOG_ERROR("UHDSDR failed to locate the radio %s in %zu attempts\n",
                      SerialNumber().c_str(), kMakeMaxAttempts);
      throw std::runtime_error("UHDSDR failed to locate the radio with id " +
                               SerialNumber());
    }

    std::stringstream device_info;
    device_info << "Hardware = " << dev_->get_mboard_name() << std::endl;

    if (kPrintRadioSettings) {
      auto clock_sources = dev_->get_clock_sources(0);
      for ([[maybe_unused]] const auto& source : clock_sources) {
        AGORA_LOG_TRACE("Clock source %s\n", source.c_str());
      }
      auto time_sources = dev_->get_time_sources(0);
      for ([[maybe_unused]] const auto& source : time_sources) {
        AGORA_LOG_TRACE("Time source %s\n", source.c_str());
      }
      AGORA_LOG_INFO("%s\n", device_info.str().c_str());
    }

    // this is a UHD version of setting up stream:
    uhd::stream_args_t stream_args("sc16");
    stream_args.channels = enabled_channels;
    stream_args.args = sargs;

    rxs_ = dev_->get_rx_stream(stream_args);
    txs_ = dev_->get_tx_stream(stream_args);

    if (txs_ == nullptr || rxs_ == nullptr) {
      AGORA_LOG_ERROR(
          "RadioUHDSdr::Init[%zu] - Radio tx/rx control plane could not be "
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
  AGORA_LOG_TRACE("Setup RadioUHD Sdr %s(%zu)\n", SerialNumber().c_str(), Id());
  dev_->set_rx_rate(cfg_->Rate());
  dev_->set_tx_rate(cfg_->Rate());

  // use the TRX antenna port for both tx and rx
  for (auto ch : EnabledChannels()) {
    dev_->set_rx_antenna("TX/RX", ch);
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
  }  // end for (const auto& ch : EnabledChannels())
}

void RadioUHDSdr::Activate(Radio::ActivationTypes type, long long act_time_ns,
                           size_t samples) {
  AGORA_LOG_INFO(
      "Activate RadioUHDSdr %s(%zu) at time %lld with samples %zu and type "
      "%d\n",
      SerialNumber().c_str(), Id(), act_time_ns, samples,
      static_cast<int>(type));
  const bool is_ue = false;
  if (is_ue) {
    AGORA_LOG_INFO("setting sources to internal \n");
    dev_->set_clock_source("internal");
    dev_->set_time_source("internal");
    uhd::time_spec_t time_zero = uhd::time_spec_t(0.0);
    dev_->set_time_unknown_pps(time_zero);
  } else {
    AGORA_LOG_INFO("setting sources to external \n");
    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    dev_->set_time_source("external");
    uhd::time_spec_t time_zero = uhd::time_spec_t(0.0);
    dev_->set_time_unknown_pps(time_zero);
    // Wait for pps sync pulse
    // std::this_thread::sleep_for(std::chrono::seconds(0.1));

    std::vector<std::string> sensor_names;
    sensor_names = dev_->get_tx_sensor_names(0);
    if (std::find(sensor_names.begin(), sensor_names.end(), "lo_locked") !=
        sensor_names.end()) {
      uhd::sensor_value_t lo_locked = dev_->get_tx_sensor("lo_locked", 0);
      std::cout << boost::format("Checking TXRX: %s ...") %
                       lo_locked.to_pp_string()
                << std::endl;
      UHD_ASSERT_THROW(lo_locked.to_bool());
    }
    dev_->set_clock_source("external");
    const size_t mboard_sensor_idx = 0;
    sensor_names = dev_->get_mboard_sensor_names(mboard_sensor_idx);
    uhd::sensor_value_t ref_locked =
        dev_->get_mboard_sensor("ref_locked", mboard_sensor_idx);
    std::cout << boost::format("Checking TXRX: %s ...") %
                     ref_locked.to_pp_string()
              << std::endl;
    UHD_ASSERT_THROW(ref_locked.to_bool());
    dev_->set_time_next_pps(time_zero);
    //dev_->set_time_unknown_pps(uhd::time_spec_t(0.0));
    // Wait for pps sync pulse (per manual)
    std::this_thread::sleep_for(std::chrono::seconds(1));
    auto time_check = dev_->get_time_last_pps();
    RtAssert(time_check == time_zero, "Time check did not match");
  }
  AGORA_LOG_INFO("in/external clock set \n");

  //Enable the rx stream in 1 second from now
  uhd::stream_cmd_t::stream_mode_t mode;
  mode = uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS;
  uhd::stream_cmd_t cmd(mode);
  cmd.stream_now = false;
  cmd.time_spec = dev_->get_time_now() + uhd::time_spec_t(kUhdInitTimeSec);
  cmd.num_samps = samples;

  AGORA_LOG_INFO("RadioUHDSdr::Activated\n");
  AGORA_LOG_INFO("before issue stream \n");
  rxs_->issue_stream_cmd(cmd);
  AGORA_LOG_INFO("activation success \n");
}

void RadioUHDSdr::Deactivate() {
  AGORA_LOG_TRACE("Deactivate RadioUHDSdr %s(%zu)\n", SerialNumber().c_str(),
                  Id());
  uhd::stream_cmd_t cmd(uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS);
  //Stop now
  cmd.stream_now = true;
  rxs_->issue_stream_cmd(cmd);
}

int RadioUHDSdr::Tx(const void* const* tx_buffs, size_t tx_size,
                    Radio::TxFlags flags, long long& tx_time_ns) {
  constexpr double kTxTimeoutSec = 1.0f;
  //tx_time_ns is a sample counter / tick value

  uhd::tx_metadata_t md;
  //Should handle the start of burst flag...
  //md.start_of_burst = true;
  //md.end_of_burst = (flags == kEndTransmit);
  md.start_of_burst = true;
  md.end_of_burst = true;
  md.has_time_spec = true;
  md.time_spec = uhd::time_spec_t::from_ticks(tx_time_ns, cfg_->Rate());

  uhd::tx_streamer::buffs_type stream_buffs(tx_buffs, txs_->get_num_channels());
  const int write_status = txs_->send(stream_buffs, tx_size, md, kTxTimeoutSec);
  if (kDebugRadioTX) {
    //Should check the async status here
  }
  return write_status;
}

int RadioUHDSdr::Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
                    size_t rx_size, Radio::RxFlags& out_flags,
                    long long& rx_time_ns) {
  rx_time_ns = 0;
  std::vector<void*> rx_locations;
  rx_locations.reserve(rx_data.size());

  for (auto& buff : rx_data) {
    rx_locations.emplace_back(buff.data());
  }
  return RadioUHDSdr::Rx(rx_locations, rx_size, out_flags, rx_time_ns);
}

int RadioUHDSdr::Rx(std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
                    size_t rx_size, Radio::RxFlags& out_flags,
                    long long& rx_time_ns) {
  rx_time_ns = 0;
  std::vector<void*> rx_locations;
  rx_locations.reserve(rx_buffs.size());

  for (auto& buff : rx_buffs) {
    rx_locations.emplace_back(buff->data());
  }
  return RadioUHDSdr::Rx(rx_locations, rx_size, out_flags, rx_time_ns);
}

int RadioUHDSdr::Rx(std::vector<void*>& rx_locs, size_t rx_size,
                    Radio::RxFlags& out_flags, long long& rx_time_ns) {
  constexpr double kRxTimeoutSec = 1.0f;
  //Init the output values
  out_flags = Radio::RxFlags::kRxFlagNone;
  rx_time_ns = 0;

  uhd::rx_metadata_t md;
  uhd::rx_streamer::buffs_type stream_buffs(rx_locs.data(),
                                            rxs_->get_num_channels());
  const size_t rx_status =
      rxs_->recv(stream_buffs, rx_size, md, kRxTimeoutSec, false);

  const bool has_time = md.has_time_spec;
  const bool start_burst = md.start_of_burst;
  const bool end_burst = md.end_of_burst;
  const bool more_frags = md.more_fragments;
  AGORA_LOG_TRACE(
      "Rx Debug | HAS TIME:%d | START BURST:%d | END BURST:%d | FRAGS :%d "
      "| FRAG OFFSET:%zu |\n",
      has_time, start_burst, end_burst, more_frags, md.fragment_offset);

  if (has_time) {
    rx_time_ns = md.time_spec.to_ticks(cfg_->Rate());
  } else {
    AGORA_LOG_WARN("RadioUHDSdr::Rx - does not have time \n");
  }

  //If status == 0 check why
  if (rx_status == 0) {
    AGORA_LOG_WARN("RadioUHDSdr::Rx - returned error code %d(%s)\n",
                   static_cast<int>(md.error_code), md.strerror().c_str());
    /*
    switch (md.error_code) {
      case uhd::rx_metadata_t::ERROR_CODE_NONE:
      case uhd::rx_metadata_t::ERROR_CODE_OVERFLOW:
      case uhd::rx_metadata_t::ERROR_CODE_TIMEOUT:
      case uhd::rx_metadata_t::ERROR_CODE_BAD_PACKET:
      case uhd::rx_metadata_t::ERROR_CODE_ALIGNMENT:
      case uhd::rx_metadata_t::ERROR_CODE_LATE_COMMAND:
      case uhd::rx_metadata_t::ERROR_CODE_BROKEN_CHAIN:
        break;
    }
    */
  } else if (rx_status == rx_size) {
    if (end_burst) {
      //This usually happens when the timeout is not long enough to wait for multiple packets for a given requested rx length
      AGORA_LOG_WARN("RadioUHDSdr::Rx - unexpected end burst %zu:%zu\n",
                     rx_size, rx_status);
    }

    if (more_frags) {
      AGORA_LOG_WARN(
          "RadioUHDSdr::Rx - fragments remaining on rx call requested %zu "
          "received %zu\n",
          rx_size, rx_status);
    }
    out_flags = Radio::RxFlags::kEndReceive;
  } else {
    AGORA_LOG_WARN(
        "RadioUHDSdr::Rx - returned less than requested samples %zu:%zu\n",
        rx_size, rx_status);
  }
  return rx_status;
}

void RadioUHDSdr::Trigger() {}

void RadioUHDSdr::ReadSensor() const {
  for (size_t i = 0; i < dev_->get_num_mboards(); i++) {
    std::cout << dev_->get_mboard_sensor_names(i).at(0) << std::endl;
  }
}

void RadioUHDSdr::SetTimeAtTrigger(long long time_ns) {
  uhd::time_spec_t time_new =
      uhd::time_spec_t::from_ticks(time_ns, cfg_->Rate());
  dev_->set_time_now(time_new);
  // there is no time setting function as set time last pps
  // dev_->set_time_last_pps(time_new);
  //dev_->set_time_next_pps(time_new);
  //Wait a second?

  auto time_dev = dev_->get_time_last_pps().to_ticks(cfg_->Rate());
  if (time_dev != time_ns) {
    AGORA_LOG_WARN(
        "RadioUHDSdr::SetTimeAtTrigger[%zu] the hardware trigger time is "
        "not the expected value %lld : %lld\n",
        Id(), time_dev, time_ns);
  }
}

long long RadioUHDSdr::GetTimeNs() {
  auto time_dev = dev_->get_time_now().to_ticks(cfg_->Rate());
  AGORA_LOG_TRACE("RadioUHDSdr::GetTimeNs[%zu] the hardware time is %lld\n",
                  Id(), time_dev);
  return time_dev;
}

void RadioUHDSdr::PrintSettings() const {
  std::stringstream print_message;
  std::map<std::string, std::string> out;
  for (size_t i = 0; i < dev_->get_tx_num_channels(); i++) {
    const uhd::dict<std::string, std::string> info = dev_->get_usrp_tx_info(i);
    for (const std::string& key : info.keys()) {
      if (key.size() > 3 && key.substr(0, 3) == "tx_") {
        out[str(boost::format("tx%d_%s") % i % key.substr(3))] = info[key];
      } else {
        out[key] = info[key];
      }
    }
  }
  for (size_t i = 0; i < dev_->get_rx_num_channels(); i++) {
    const uhd::dict<std::string, std::string> info = dev_->get_usrp_rx_info(i);
    for (const std::string& key : info.keys()) {
      if (key.size() > 3 && key.substr(0, 3) == "rx_") {
        out[str(boost::format("rx%d_%s") % i % key.substr(3))] = info[key];
      } else {
        out[key] = info[key];
      }
    }
  }

  print_message << SerialNumber() << " (" << Id() << ")" << std::endl;

  if (kPrintRadioSettings) {
    for (size_t i = 0; i < EnabledChannels().size(); i++) {
      print_message << "channels enalbed are: " << EnabledChannels()[i]
                    << std::endl;
    }
    for (const auto& c : EnabledChannels()) {
      if (c < dev_->get_rx_num_channels()) {
        print_message << "RX Channel " << c << std::endl
                      << "Actual RX sample rate: "
                      << (dev_->get_rx_rate(c) / 1e6) << "MSps..." << std::endl
                      << "Actual RX frequency: " << (dev_->get_rx_freq(c) / 1e9)
                      << "GHz..." << std::endl
                      << "Actual RX gain: " << (dev_->get_rx_gain(c)) << "..."
                      << std::endl;
        print_message << "Actual RX bandwidth: "
                      << (dev_->get_rx_bandwidth(c) / 1e6) << "M..."
                      << std::endl
                      << "Actual RX antenna: " << (dev_->get_rx_antenna(c))
                      << "..." << std::endl;
      }
    }

    for (auto c : EnabledChannels()) {
      if (c < dev_->get_tx_num_channels()) {
        print_message << "TX Channel " << c << std::endl
                      << "Actual TX sample rate: "
                      << (dev_->get_tx_rate(c) / 1e6) << "MSps..." << std::endl
                      << "Actual TX frequency: " << (dev_->get_tx_freq(c) / 1e9)
                      << "GHz..." << std::endl
                      << "Actual TX gain: " << (dev_->get_tx_gain(c)) << "..."
                      << std::endl;
        print_message << "Actual TX bandwidth: "
                      << (dev_->get_tx_bandwidth(c) / 1e6) << "M..."
                      << std::endl
                      << "Actual TX antenna: " << (dev_->get_tx_antenna(c))
                      << "... " << std::endl;
      }
    }
    AGORA_LOG_INFO("%s\n", print_message.str().c_str());
  }
}

void RadioUHDSdr::ClearSyncDelay() {}
void RadioUHDSdr::ConfigureTddModeBs(bool is_ref_radio) {}
void RadioUHDSdr::ConfigureTddModeUe() {}

void RadioUHDSdr::Flush() {
  AGORA_LOG_INFO("Flushing radio %zu rx data plane\n", Id());
}
