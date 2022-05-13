/** @file radio_soapysdr.h
  * @brief Defination file for the RadioSoapySdr class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#include "radio_soapysdr.h"

#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Logger.hpp"
#include "SoapySDR/Time.hpp"
#include "logger.h"
#include "radio_data_plane.h"
#include "symbols.h"

constexpr size_t kSoapyMakeMaxAttempts = 3;
constexpr bool kPrintRadioSettings = false;
constexpr double kAttnMax = -18.0f;

// radio init time for UHD devices
constexpr size_t kUhdInitTimeSec = 3;

RadioSoapySdr::RadioSoapySdr(RadioDataPlane::DataPlaneType rx_dp_type)
    : Radio(),
      dev_(nullptr),
      rxp_(RadioDataPlane::Create(rx_dp_type)),
      txs_(nullptr),
      correlator_enabled_(false) {
  //Reduce the soapy log level
  AGORA_LOG_INFO("Create RadioSoapySdr Radio\n");
  //SoapySDR::setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_NOTICE);
  SoapySDR::setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_DEBUG);
}

RadioSoapySdr::~RadioSoapySdr() {
  AGORA_LOG_INFO("Destroy RadioSoapySdr %s(%zu)\n", SerialNumber().c_str(),
                 Id());
  Close();
}

void RadioSoapySdr::Close() {
  AGORA_LOG_INFO("Close RadioSoapySdr %s(%zu)\n", SerialNumber().c_str(), Id());
  rxp_.reset();
  if (txs_ != nullptr) {
    dev_->closeStream(txs_);
  }
  txs_ = nullptr;
  if (dev_ != nullptr) {
    SoapySDR::Device::unmake(dev_);
  }
  dev_ = nullptr;
  Radio::Close();
}

void RadioSoapySdr::Init(const Config* cfg, size_t id,
                         const std::string& serial,
                         const std::vector<size_t>& enabled_channels,
                         bool hw_framer) {
  AGORA_LOG_INFO("Init RadioSoapySdr %s(%zu)\n", serial.c_str(), id);
  if (dev_ == nullptr) {
    Radio::Init(cfg, id, serial, enabled_channels, hw_framer);

    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["timeout"] = "1000000";
    if (kUseUHD == false) {
      args["driver"] = "iris";
      args["serial"] = SerialNumber();
    } else {
      args["driver"] = "uhd";
      args["addr"] = SerialNumber();
    }

    for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
      try {
        dev_ = SoapySDR::Device::make(args);
        break;
      } catch (const std::runtime_error& e) {
        const auto* message = e.what();
        AGORA_LOG_ERROR(
            "RadioSoapySdr::Init[%zu] - Soapy error try %zu -- %s\n", id, tries,
            message);
      }
    }
    if (dev_ == nullptr) {
      AGORA_LOG_ERROR(
          "SoapySDR failed to locate the radio %s in %zu attempts\n",
          SerialNumber().c_str(), kSoapyMakeMaxAttempts);
      throw std::runtime_error("SoapySDR failed to locate the radio with id " +
                               SerialNumber());
    }
    // resets the DATA_clk domain logic.
    SoapySDR::Kwargs hw_info = dev_->getHardwareInfo();
    if (hw_info["revision"].find("Iris") != std::string::npos) {
      dev_->writeSetting("RESET_DATA_LOGIC", "");
    }

    if (rxp_ == nullptr) {
      AGORA_LOG_ERROR(
          "RadioSoapySdr::Init[%zu] - Radio rx control plane could not be "
          "configured\n",
          id);
    }
    rxp_->Init(this, cfg_, hw_framer);
    //rxs_ = dev_->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, enabled_channels,
    //                         sargs);
    rxp_->Setup();

    txs_ = dev_->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, enabled_channels,
                             sargs);
    if (txs_ == nullptr) {
      AGORA_LOG_ERROR(
          "RadioSoapySdr::Init[%zu] - Radio tx control plane could not be "
          "configured\n",
          id);
    }
  } else {
    AGORA_LOG_ERROR("RadioSoapySdr::Init[%zu] - Radio already has been init\n",
                    id);
  }
}

void RadioSoapySdr::Setup(const std::vector<double>& tx_gains,
                          const std::vector<double>& rx_gains) {
  AGORA_LOG_INFO("Setup RadioSoapySdr %s(%zu)\n", SerialNumber().c_str(), Id());
  for (auto ch : EnabledChannels()) {
    dev_->setSampleRate(SOAPY_SDR_RX, ch, cfg_->Rate());
    dev_->setSampleRate(SOAPY_SDR_TX, ch, cfg_->Rate());
  }

  // use the TRX antenna port for both tx and rx
  for (auto ch : EnabledChannels()) {
    if (kUseUHD == false) {
      dev_->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    } else {
      dev_->setAntenna(SOAPY_SDR_RX, ch, "RX2");
      dev_->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
    }
  }

  SoapySDR::Kwargs hw_info = dev_->getHardwareInfo();
  for (const auto& ch : EnabledChannels()) {
    ///\todo Check to see if this is correct for kUseUhd == false (BwFilter)
    if (kUseUHD == false) {
      dev_->setBandwidth(SOAPY_SDR_RX, ch, cfg_->BwFilter());
      dev_->setBandwidth(SOAPY_SDR_TX, ch, cfg_->BwFilter());
    }

    dev_->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg_->RadioRfFreq());
    dev_->setFrequency(SOAPY_SDR_RX, ch, "BB", kUseUHD ? 0 : cfg_->Nco());
    dev_->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg_->RadioRfFreq());
    dev_->setFrequency(SOAPY_SDR_TX, ch, "BB", kUseUHD ? 0 : cfg_->Nco());

    if (kUseUHD == false) {
      std::string sdr_label = hw_info["revision"];
      std::string sdr_fe = "";
      if (hw_info.find("frontend") != hw_info.end()) {
        sdr_fe = hw_info["frontend"];
      }
      const bool is_cbrs = (sdr_fe.find("CBRS") != std::string::npos);
      const bool is_iris = (sdr_label.find("Iris") != std::string::npos);
      const bool is_villager = (sdr_label.find("VGER") != std::string::npos);

      if (is_iris) {
        // Unified gains are only available for CBRS FE for now
        if (is_cbrs && cfg_->SingleGain()) {
          // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
          dev_->setGain(SOAPY_SDR_RX, ch, rx_gains.at(ch));
          // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
          dev_->setGain(SOAPY_SDR_TX, ch, tx_gains.at(ch));
        } else {
          if (is_cbrs) {
            if (cfg_->Freq() > 3e9) {
              //[-18,0]
              dev_->setGain(SOAPY_SDR_RX, ch, "ATTN", -6);
            } else if ((cfg_->Freq() > 2e9) && (cfg_->Freq() < 3e9)) {
              //[-18,0]
              dev_->setGain(SOAPY_SDR_RX, ch, "ATTN", -18);
            } else {
              //[-18,0]
              dev_->setGain(SOAPY_SDR_RX, ch, "ATTN", -12);
            }
            //[0,17]
            dev_->setGain(SOAPY_SDR_RX, ch, "LNA2", 17);
          }

          //[0,30]
          dev_->setGain(SOAPY_SDR_RX, ch, "LNA", rx_gains.at(ch));
          //[0,12]
          dev_->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
          //[-12,19]
          dev_->setGain(SOAPY_SDR_RX, ch, "PGA", 0);

          if (is_cbrs) {
            //[-18,0] by 3
            dev_->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);
            //[0|15]
            dev_->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
          }
          //[-12,12]
          dev_->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
          //[0,30]
          dev_->setGain(SOAPY_SDR_TX, ch, "PAD", tx_gains.at(ch));
        }  // end single gain
      } else if (is_villager) {
        dev_->setGain(SOAPY_SDR_RX, ch, "LNA",
                      std::min(30.0, rx_gains.at(ch)));  //[0,30]
        dev_->setGain(SOAPY_SDR_TX, ch, "PAD",
                      std::min(30.0, tx_gains.at(ch)));  //[0,52]
      } else {
        AGORA_LOG_ERROR("SDR Type %s is not supported!\n", sdr_label.c_str());
      }
    } else {
      // kUseUHD
      dev_->setGain(SOAPY_SDR_RX, ch, "PGA0", rx_gains.at(ch));
      dev_->setGain(SOAPY_SDR_TX, ch, "PGA0", tx_gains.at(ch));
    }  // end kUseUHD == false
  }    // end for (const auto& ch : EnabledChannels())

  for (auto ch : EnabledChannels()) {
    // Clients??
    // dev_->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
    // dev_->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
    if (kUseUHD == false) {
      dev_->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }
  }
}

void RadioSoapySdr::Activate(Radio::ActivationTypes type) {
  AGORA_LOG_INFO("Activate RadioSoapySdr %s(%zu)\n", SerialNumber().c_str(),
                 Id());
  const bool is_ue = false;
  if (kUseUHD == false) {
    dev_->setHardwareTime(0, "TRIGGER");
    //SOAPY_SDR_WAIT_TRIGGER for UE
    //**********************************************************
    //dev_->activateStream(rxs_, SOAPY_SDR_WAIT_TRIGGER);
    //dev_->activateStream(txs_, SOAPY_SDR_WAIT_TRIGGER);
    //**********************************************************
    rxp_->Activate(type);
    int soapy_flags = 0;
    if (type == Radio::ActivationTypes::kActivateWaitTrigger) {
      soapy_flags = SOAPY_SDR_WAIT_TRIGGER;
    }
    dev_->activateStream(txs_, soapy_flags);
  } else {
    if (is_ue) {
      dev_->setTimeSource("internal");
      dev_->setClockSource("internal");
      dev_->setHardwareTime(0, "UNKNOWN_PPS");
    } else {
      dev_->setClockSource("external");
      dev_->setTimeSource("external");
      dev_->setHardwareTime(0, "PPS");
    }
    // Wait for pps sync pulse ??
    //std::this_thread::sleep_for(std::chrono::seconds(kUhdInitTimeSec -1));
    //dev_->activateStream(rxs_, SOAPY_SDR_HAS_TIME, kUhdInitTimeSec * 1e9, 0);
    rxp_->Activate(type);
    dev_->activateStream(txs_, SOAPY_SDR_HAS_TIME, kUhdInitTimeSec * 1e9, 0);
  }
}

void RadioSoapySdr::Deactivate() {
  AGORA_LOG_INFO("Deactivate RadioSoapySdr %s(%zu)\n", SerialNumber().c_str(),
                 Id());
  const std::string tdd_conf_str = "{\"tdd_enabled\":false}";
  rxp_->Deactivate();
  dev_->deactivateStream(txs_);
  dev_->writeSetting("TDD_MODE", "false");
  dev_->writeSetting("TDD_CONFIG", tdd_conf_str);
  Correlator(false);
}

int RadioSoapySdr::Tx(const void* const* tx_buffs, size_t tx_size, int flags,
                      long long& tx_time_ns) {
  constexpr size_t kTxTimeoutUs = 1000000;
  int tx_flags = 0;
  if (flags == 1) {
    tx_flags = SOAPY_SDR_HAS_TIME;
  } else if (flags == 2) {
    tx_flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
  }

  int w;
  if (cfg_->HwFramer() == true) {
    w = dev_->writeStream(txs_, tx_buffs, tx_size, tx_flags, tx_time_ns,
                          kTxTimeoutUs);
  } else {
    // For UHD device xmit from host using frameTimeNs
    long long frame_time_ns = SoapySDR::ticksToTimeNs(tx_time_ns, cfg_->Rate());
    w = dev_->writeStream(txs_, tx_buffs, tx_size, tx_flags, frame_time_ns,
                          kTxTimeoutUs);
  }
  if (kDebugRadioTX) {
    size_t chan_mask;
    long timeout_us(0);
    int status_flag = 0;
    const int s = dev_->readStreamStatus(txs_, chan_mask, status_flag,
                                         tx_time_ns, timeout_us);
    std::cout << "radio " << SerialNumber() << "(" << Id() << ") tx returned "
              << w << " and status " << s << " when flags was " << flags
              << std::endl;
  }
  return w;
}

int RadioSoapySdr::Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
                      size_t rx_size, Radio::RxFlags& out_flags,
                      long long& rx_time_ns) {
  rx_time_ns = 0;

  const int rx_return = rxp_->Rx(rx_data, rx_size, out_flags, rx_time_ns);
  if (rx_return < 0) {
    throw std::runtime_error("Error in RadioRx!");
  }
  return rx_return;
}

int RadioSoapySdr::Rx(
    std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs, size_t rx_size,
    Radio::RxFlags& out_flags, long long& rx_time_ns) {
  rx_time_ns = 0;

  const int rx_return = rxp_->Rx(rx_buffs, rx_size, out_flags, rx_time_ns);
  if (rx_return < 0) {
    throw std::runtime_error("Error in RadioRx!");
  }
  return rx_return;
}

int RadioSoapySdr::Rx(std::vector<void*>& rx_locs, size_t rx_size,
                      Radio::RxFlags& out_flags, long long& rx_time_ns) {
  rx_time_ns = 0;

  const int rx_return = rxp_->Rx(rx_locs, rx_size, out_flags, rx_time_ns);
  if (rx_return < 0) {
    throw std::runtime_error("Error in RadioRx!");
  }
  return rx_return;
}

void RadioSoapySdr::Trigger() { dev_->writeSetting("TRIGGER_GEN", ""); }

void RadioSoapySdr::ReadSensor() const {
  std::stringstream print_message;
  print_message << "TEMPs on Iris " << SerialNumber() << "(" << Id() << ")"
                << std::endl;
  print_message << "ZYNQ_TEMP  : " << dev_->readSensor("ZYNQ_TEMP")
                << std::endl;
  print_message << "LMS7_TEMP  : " << dev_->readSensor("LMS7_TEMP")
                << std::endl;
  print_message << "FE_TEMP  : " << dev_->readSensor("FE_TEMP") << std::endl;
  print_message << "TX0 TEMP  : " << dev_->readSensor(SOAPY_SDR_TX, 0, "TEMP")
                << std::endl;
  print_message << "TX1 TEMP  : " << dev_->readSensor(SOAPY_SDR_TX, 1, "TEMP")
                << std::endl;
  print_message << "RX0 TEMP  : " << dev_->readSensor(SOAPY_SDR_RX, 0, "TEMP")
                << std::endl;
  print_message << "RX1 TEMP  : " << dev_->readSensor(SOAPY_SDR_RX, 1, "TEMP")
                << std::endl;
  AGORA_LOG_INFO("%s\n", print_message.str().c_str());
}

void RadioSoapySdr::PrintSettings() const {
  std::stringstream print_message;

  SoapySDR::Kwargs hw_info = dev_->getHardwareInfo();
  std::string label = hw_info["revision"];
  std::string frontend = "";
  if (hw_info.find("frontend") != hw_info.end()) {
    frontend = hw_info["frontend"];
  }

  print_message << SerialNumber() << " (" << Id() << ") - " << label << " - "
                << frontend << std::endl;

  if (kPrintRadioSettings) {
    for (const auto& c : EnabledChannels()) {
      if (c < dev_->getNumChannels(SOAPY_SDR_RX)) {
        print_message << "RX Channel " << c << std::endl
                      << "Actual RX sample rate: "
                      << (dev_->getSampleRate(SOAPY_SDR_RX, c) / 1e6)
                      << "MSps..." << std::endl
                      << "Actual RX frequency: "
                      << (dev_->getFrequency(SOAPY_SDR_RX, c) / 1e9) << "GHz..."
                      << std::endl
                      << "Actual RX gain: " << (dev_->getGain(SOAPY_SDR_RX, c))
                      << "..." << std::endl;
        if (kUseUHD == false) {
          print_message << "Actual RX LNA gain: "
                        << (dev_->getGain(SOAPY_SDR_RX, c, "LNA")) << "..."
                        << std::endl
                        << "Actual RX PGA gain: "
                        << (dev_->getGain(SOAPY_SDR_RX, c, "PGA")) << "..."
                        << std::endl
                        << "Actual RX TIA gain: "
                        << (dev_->getGain(SOAPY_SDR_RX, c, "TIA")) << "..."
                        << std::endl;
          if (dev_->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            print_message << "Actual RX LNA1 gain: "
                          << (dev_->getGain(SOAPY_SDR_RX, c, "LNA1")) << "..."
                          << std::endl
                          << "Actual RX LNA2 gain: "
                          << (dev_->getGain(SOAPY_SDR_RX, c, "LNA2")) << "..."
                          << std::endl;
          }
        }
        print_message << "Actual RX bandwidth: "
                      << (dev_->getBandwidth(SOAPY_SDR_RX, c) / 1e6) << "M..."
                      << std::endl
                      << "Actual RX antenna: "
                      << (dev_->getAntenna(SOAPY_SDR_RX, c)) << "..."
                      << std::endl;
      }
    }

    for (auto c : EnabledChannels()) {
      if (c < dev_->getNumChannels(SOAPY_SDR_TX)) {
        print_message << "TX Channel " << c << std::endl
                      << "Actual TX sample rate: "
                      << (dev_->getSampleRate(SOAPY_SDR_TX, c) / 1e6)
                      << "MSps..." << std::endl
                      << "Actual TX frequency: "
                      << (dev_->getFrequency(SOAPY_SDR_TX, c) / 1e9) << "GHz..."
                      << std::endl
                      << "Actual TX gain: " << (dev_->getGain(SOAPY_SDR_TX, c))
                      << "..." << std::endl;
        if (kUseUHD == false) {
          print_message << "Actual TX PAD gain: "
                        << (dev_->getGain(SOAPY_SDR_TX, c, "PAD")) << "..."
                        << std::endl
                        << "Actual TX IAMP gain: "
                        << (dev_->getGain(SOAPY_SDR_TX, c, "IAMP")) << "..."
                        << std::endl;
          if (dev_->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            print_message << "Actual TX PA1 gain: "
                          << (dev_->getGain(SOAPY_SDR_TX, c, "PA1")) << "..."
                          << std::endl
                          << "Actual TX PA2 gain: "
                          << (dev_->getGain(SOAPY_SDR_TX, c, "PA2")) << "..."
                          << std::endl
                          << "Actual TX PA3 gain: "
                          << (dev_->getGain(SOAPY_SDR_TX, c, "PA3")) << "..."
                          << std::endl;
          }
        }
        print_message << "Actual TX bandwidth: "
                      << (dev_->getBandwidth(SOAPY_SDR_TX, c) / 1e6) << "M..."
                      << std::endl
                      << "Actual TX antenna: "
                      << (dev_->getAntenna(SOAPY_SDR_TX, c)) << "... "
                      << std::endl;
      }
    }
    AGORA_LOG_INFO("%s\n", print_message.str().c_str());
  }
}

void RadioSoapySdr::ClearSyncDelay() { dev_->writeSetting("SYNC_DELAYS", ""); }

void RadioSoapySdr::InitRefTx(size_t channel, double freq) {
  dev_->setFrequency(SOAPY_SDR_TX, channel, "RF", freq);
  dev_->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
}

void RadioSoapySdr::InitCalRx(size_t channel, double center_freq) {
  // must set TX "RF" Freq to make sure, we continue using the same LO for
  // rx cal
  dev_->setFrequency(SOAPY_SDR_TX, channel, "RF", center_freq);
  dev_->setFrequency(SOAPY_SDR_RX, channel, "RF", center_freq);
  dev_->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
  dev_->setDCOffsetMode(SOAPY_SDR_RX, channel, false);
}

void RadioSoapySdr::ResetTxGains() {
  ///\todo should this do all channels or enabled channels?
  for (const auto& channel : EnabledChannels()) {
    dev_->setGain(SOAPY_SDR_TX, channel, "PA2", 0);
    dev_->setGain(SOAPY_SDR_TX, channel, "PAD", 0);
    dev_->setGain(SOAPY_SDR_TX, channel, "IAMP", 0);
    dev_->setGain(SOAPY_SDR_TX, channel, "ATTN", kAttnMax);
  }
}

void RadioSoapySdr::ResetRxGains() {
  for (const auto& channel : EnabledChannels()) {
    dev_->setGain(SOAPY_SDR_RX, channel, "LNA", 0);
    dev_->setGain(SOAPY_SDR_RX, channel, "PGA", 0);
    dev_->setGain(SOAPY_SDR_RX, channel, "TIA", 0);
    dev_->setGain(SOAPY_SDR_RX, channel, "ATTN", kAttnMax);
    dev_->setGain(SOAPY_SDR_RX, channel, "LNA2", 14.0);
  }
}

void RadioSoapySdr::SetTxCalGain(size_t channel) {
  dev_->setGain(SOAPY_SDR_TX, channel, "PAD", 40);
}

void RadioSoapySdr::SetTxGain(size_t channel, const std::string& gain_stage,
                              double value) {
  dev_->setGain(SOAPY_SDR_TX, channel, gain_stage, value);
}

void RadioSoapySdr::SetRxGain(size_t channel, const std::string& gain_stage,
                              double value) {
  dev_->setGain(SOAPY_SDR_RX, channel, gain_stage, value);
}

std::vector<std::complex<float>> RadioSoapySdr::SnoopSamples(size_t channel,
                                                             size_t read_size) {
  std::vector<uint32_t> samps_int =
      dev_->readRegisters("RX_SNOOPER", channel, read_size);
  std::vector<std::complex<float>> samps =
      Utils::Uint32tocfloat(samps_int, "IQ");
  return samps;
}

void RadioSoapySdr::StartRefTx(size_t channel) {
  //Can we move this code to InitRefTx????
  dev_->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                     std::to_string(1 << 14));
  dev_->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");
}

void RadioSoapySdr::StopRefTx(size_t channel) {
  dev_->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
  dev_->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");
}

void RadioSoapySdr::SetDcOffset(int direction, size_t channel,
                                std::complex<double> dc_corr) {
  dev_->setDCOffset(direction, channel, dc_corr);
}

void RadioSoapySdr::SetIQBalance(int direction, size_t channel,
                                 std::complex<double> i_qcorr) {
  dev_->setIQBalance(direction, channel, i_qcorr);
}

void RadioSoapySdr::AdjustDelay(const std::string& delay) {
  dev_->writeSetting("ADJUST_DELAYS", delay);
}

void RadioSoapySdr::SetFreqBb(size_t channel, double freq) {
  dev_->setFrequency(SOAPY_SDR_TX, channel, "BB", freq);
  dev_->setFrequency(SOAPY_SDR_RX, channel, "BB", freq);
}

void RadioSoapySdr::SetFreqRf(size_t channel, double freq) {
  dev_->setFrequency(SOAPY_SDR_TX, channel, "RF", freq);
  dev_->setFrequency(SOAPY_SDR_RX, channel, "RF", freq);
}

void RadioSoapySdr::ConfigureTddModeBs(bool is_ref_radio,
                                       size_t beacon_radio_id) {
  nlohmann::json conf;
  conf["tdd_enabled"] = true;
  conf["frame_mode"] = "free_running";
  conf["max_frame"] = 0;
  conf["symbol_size"] = cfg_->SampsPerSymbol();
  conf["beacon_start"] = cfg_->OfdmTxZeroPrefix();
  conf["beacon_stop"] = cfg_->OfdmTxZeroPrefix() + cfg_->BeaconLen();

  // experimentally good value for dev front-end
  dev_->writeSetting("TX_SW_DELAY", "30");
  dev_->writeSetting("TDD_MODE", "true");
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
  dev_->writeSetting("TDD_CONFIG", conf_string);
  dev_->writeRegisters("BEACON_RAM", 0, cfg_->Beacon());

  size_t ndx = 0;
  for (const auto& channel : EnabledChannels()) {
    const bool is_beacon_antenna =
        !cfg_->Beamsweep() && ndx == cfg_->BeaconAnt();
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
    dev_->writeRegisters(prog_reg, 0, beacon_weights);
    ++ndx;
  }
  dev_->writeSetting("BEACON_START", std::to_string(beacon_radio_id));
}

void RadioSoapySdr::ConfigureTddModeUe() {
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
  dev_->writeSetting("TDD_CONFIG", conf_string);
  // beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) +
  // 82 (Client FE Delay)
  const int cl_trig_offset = cfg_->BeaconLen() + 249;
  const int sf_start = cl_trig_offset / cfg_->SampsPerSymbol();
  const int sp_start = cl_trig_offset % cfg_->SampsPerSymbol();
  dev_->setHardwareTime(
      SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, cfg_->Rate()),
      "TRIGGER");

  // experimentally good value for dev front-end
  dev_->writeSetting("TX_SW_DELAY", "30");
  dev_->writeSetting("TDD_MODE", "true");
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
    dev_->writeRegisters(prog_reg, 0, cfg_->Pilot());
  }
  Correlator(true);
}

void RadioSoapySdr::InitAgc(bool enabled, size_t setting) {
  nlohmann::json agc_conf;
  agc_conf["agc_enabled"] = enabled;
  // 0 to 108
  agc_conf["agc_gain_init"] = setting;
  const std::string agc_confStr = agc_conf.dump();

  dev_->writeSetting("AGC_CONFIG", agc_confStr);
}

void RadioSoapySdr::Correlator(bool enable) {
  if (enable) {
    if (correlator_enabled_ == false) {
      correlator_enabled_ = true;
      const std::string corr_conf =
          "{\"corr_enabled\":true,\"corr_threshold\":" + std::to_string(1) +
          "}";
      dev_->writeSetting("CORR_CONFIG", corr_conf);
      dev_->writeRegisters("CORR_COE", 0, cfg_->Coeffs());
      dev_->writeSetting("CORR_START", (cfg_->UeChannel() == "B") ? "B" : "A");
    }
  } else if (correlator_enabled_ == true) {
    correlator_enabled_ = false;
    const std::string corr_conf = "{\"corr_enabled\":false}";
    dev_->writeSetting("CORR_CONFIG", corr_conf);
  }
}

void RadioSoapySdr::Flush() {
  AGORA_LOG_INFO("Flushing radio rx data plane%zu\n", Id());
  rxp_->Flush();
}