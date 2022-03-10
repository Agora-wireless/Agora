/**
 * @file client_radio.cc
 * @brief Implementation file for the client radio config class
 */

#include "client_radio.h"

#include <SoapySDR/Logger.hpp>

#include "comms-lib.h"
#include "logger.h"
#include "nlohmann/json.hpp"

static constexpr size_t kSoapyMakeMaxAttempts = 2;

ClientRadioConfig::ClientRadioConfig(const Config* const cfg) : cfg_(cfg) {
  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  // load channels
  auto channels = Utils::StrToChannels(cfg_->UeChannel());
  /// Reduce the soapy log level
  SoapySDR::setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_NOTICE);

  this->radio_num_ = cfg_->UeNum();
  this->antenna_num_ = cfg_->UeAntNum();
  std::cout << "radio num is " << this->radio_num_ << std::endl;

  cl_stn_.resize(radio_num_);
  tx_streams_.resize(radio_num_);
  rx_streams_.resize(radio_num_);

  std::vector<std::thread> radio_threads;
  num_client_radios_initialized_ = 0;
  for (size_t i = 0; i < this->radio_num_; i++) {
#ifdef THREADED_INIT
    radio_threads.emplace_back(&ClientRadioConfig::InitClientRadio, this, i);
#else
    InitClientRadio(tid);
#endif
  }  // end for (size_t i = 0; i < this->radio_num_; i++)

#ifdef THREADED_INIT
  size_t num_checks = 0;

  size_t num_client_radios_init = num_client_radios_initialized_.load();
  while (num_client_radios_init != this->radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      AGORA_LOG_INFO(
          "RadioConfig: Waiting for radio initialization, %zu of %zu "
          "ready\n",
          num_client_radios_init, this->radio_num_);
      num_checks = 0;
    }
    num_client_radios_init = num_client_radios_initialized_.load();
  }

  for (auto& init_thread : radio_threads) {
    init_thread.join();
  }
#endif

  for (size_t i = 0; i < this->radio_num_; i++) {
    SoapySDR::Kwargs hw_info = cl_stn_.at(i)->getHardwareInfo();
    std::string label = hw_info["revision"];
    std::string frontend = "";
    if (hw_info.find("frontend") != hw_info.end()) {
      frontend = hw_info["frontend"];
    }
    std::cout << cfg_->UeRadioId().at(i) << " - " << label << " - " << frontend
              << std::endl;
    for (auto c : channels) {
      if (c < cl_stn_.at(i)->getNumChannels(SOAPY_SDR_RX)) {
        AGORA_LOG_INFO("RX Channel %zu\n", c);
        AGORA_LOG_INFO("Actual RX sample rate: %fMSps...\n",
                       (cl_stn_.at(i)->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
        AGORA_LOG_INFO("Actual RX frequency: %fGHz...\n",
                       (cl_stn_.at(i)->getFrequency(SOAPY_SDR_RX, c) / 1e9));
        AGORA_LOG_INFO("Actual RX gain: %f...\n",
                       (cl_stn_.at(i)->getGain(SOAPY_SDR_RX, c)));
        if (!kUseUHD) {
          AGORA_LOG_INFO("Actual RX LNA gain: %f...\n",
                         (cl_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "LNA")));
          AGORA_LOG_INFO("Actual RX PGA gain: %f...\n",
                         (cl_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "PGA")));
          AGORA_LOG_INFO("Actual RX TIA gain: %f...\n",
                         (cl_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "TIA")));
          if (cl_stn_.at(i)->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            AGORA_LOG_INFO("Actual RX LNA1 gain: %f...\n",
                           (cl_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "LNA1")));
            AGORA_LOG_INFO("Actual RX LNA2 gain: %f...\n",
                           (cl_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "LNA2")));
          }
        }
        AGORA_LOG_INFO("Actual RX bandwidth: %fM...\n",
                       (cl_stn_.at(i)->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
        AGORA_LOG_INFO("Actual RX antenna: %s...\n",
                       (cl_stn_.at(i)->getAntenna(SOAPY_SDR_RX, c).c_str()));
      }
    }

    for (auto c : channels) {
      if (c < cl_stn_.at(i)->getNumChannels(SOAPY_SDR_TX)) {
        AGORA_LOG_INFO("TX Channel %zu\n", c);
        AGORA_LOG_INFO("Actual TX sample rate: %fMSps...\n",
                       (cl_stn_.at(i)->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
        AGORA_LOG_INFO("Actual TX frequency: %fGHz...\n",
                       (cl_stn_.at(i)->getFrequency(SOAPY_SDR_TX, c) / 1e9));
        AGORA_LOG_INFO("Actual TX gain: %f...\n",
                       (cl_stn_.at(i)->getGain(SOAPY_SDR_TX, c)));
        if (!kUseUHD) {
          AGORA_LOG_INFO("Actual TX PAD gain: %f...\n",
                         (cl_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PAD")));
          AGORA_LOG_INFO("Actual TX IAMP gain: %f...\n",
                         (cl_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "IAMP")));
          if (cl_stn_.at(i)->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            AGORA_LOG_INFO("Actual TX PA1 gain: %f...\n",
                           (cl_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PA1")));
            AGORA_LOG_INFO("Actual TX PA2 gain: %f...\n",
                           (cl_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PA2")));
            AGORA_LOG_INFO("Actual TX PA3 gain: %f...\n",
                           (cl_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PA3")));
          }
        }
        AGORA_LOG_INFO("Actual TX bandwidth: %fM...\n",
                       (cl_stn_.at(i)->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
        AGORA_LOG_INFO("Actual TX antenna: %s...\n",
                       (cl_stn_.at(i)->getAntenna(SOAPY_SDR_TX, c).c_str()));
      }
    }
    std::cout << std::endl;
  }
  std::cout << "radio init done!" << std::endl;
}

void ClientRadioConfig::InitClientRadio(size_t tid) {
  // load channels
  auto channels = Utils::StrToChannels(cfg_->UeChannel());

  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  args["timeout"] = "1000000";
  if (!kUseUHD) {
    args["driver"] = "iris";
    args["serial"] = cfg_->UeRadioId().at(tid);
  } else {
    args["driver"] = "uhd";
    args["addr"] = cfg_->UeRadioId().at(tid);
  }

  SoapySDR::Device* ue_device = nullptr;
  for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
    try {
      ue_device = SoapySDR::Device::make(args);
      break;
    } catch (const std::runtime_error& e) {
      const auto* message = e.what();
      AGORA_LOG_INFO("InitClientRadio[%zu] - Soapy error try %zu -- %s\n", tid,
                     tries, message);
    }
  }
  if (ue_device == nullptr) {
    AGORA_LOG_INFO(
        "SoapySDR failed to locate the Ue device %s in %zu attempts\n",
        cfg_->UeRadioId().at(tid).c_str(), kSoapyMakeMaxAttempts);
    throw std::runtime_error("SoapySDR failed to locate the Ue device");
  }
  cl_stn_.at(tid) = ue_device;

  for (auto ch : channels) {
    cl_stn_.at(tid)->setSampleRate(SOAPY_SDR_RX, ch, cfg_->Rate());
    cl_stn_.at(tid)->setSampleRate(SOAPY_SDR_TX, ch, cfg_->Rate());
  }
  rx_streams_.at(tid) = cl_stn_.at(tid)->setupStream(
      SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
  tx_streams_.at(tid) = cl_stn_.at(tid)->setupStream(
      SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);

  // resets the DATA_clk domain logic.
  SoapySDR::Kwargs hw_info = cl_stn_.at(tid)->getHardwareInfo();
  if (kUseUHD == false) {
    if (hw_info["revision"].find("Iris") != std::string::npos) {
      cl_stn_.at(tid)->writeSetting("RESET_DATA_LOGIC", "");
    }
  }

  // use the TRX antenna port for both tx and rx
  for (auto ch : channels) {
    if (kUseUHD == false) {
      cl_stn_.at(tid)->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    } else {
      cl_stn_.at(tid)->setAntenna(SOAPY_SDR_RX, ch, "RX2");
      cl_stn_.at(tid)->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
    }
  }

  for (auto ch : channels) {
    cl_stn_.at(tid)->setBandwidth(SOAPY_SDR_RX, ch, cfg_->BwFilter());
    cl_stn_.at(tid)->setBandwidth(SOAPY_SDR_TX, ch, cfg_->BwFilter());

    cl_stn_.at(tid)->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg_->RadioRfFreq());
    cl_stn_.at(tid)->setFrequency(SOAPY_SDR_RX, ch, "BB",
                                  kUseUHD ? 0 : cfg_->Nco());
    cl_stn_.at(tid)->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg_->RadioRfFreq());
    cl_stn_.at(tid)->setFrequency(SOAPY_SDR_TX, ch, "BB",
                                  kUseUHD ? 0 : cfg_->Nco());

    if (kUseUHD == false) {
      std::string sdr_label = hw_info["revision"];
      std::string sdr_fe = "";
      if (hw_info.find("frontend") != hw_info.end()) {
        sdr_fe = hw_info["frontend"];
      }
      bool is_cbrs = (sdr_fe.find("CBRS") != std::string::npos);
      // Check if the hardware is Iris-030
      if (sdr_label.find("Iris") != std::string::npos) {
        // Unified gains are only available for CBRS FE for now
        if (is_cbrs && cfg_->SingleGain() == true) {
          // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
          cl_stn_.at(tid)->setGain(
              SOAPY_SDR_RX, ch,
              ch != 0u ? cfg_->ClientRxGainB(tid) : cfg_->ClientRxGainA(tid));
          // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
          cl_stn_.at(tid)->setGain(
              SOAPY_SDR_TX, ch,
              ch != 0u ? cfg_->ClientTxGainB(tid) : cfg_->ClientTxGainA(tid));
          AGORA_LOG_INFO("Tx gain: %lf, Rx gain: %lf\n",
                         cl_stn_.at(tid)->getGain(SOAPY_SDR_TX, ch),
                         cl_stn_.at(tid)->getGain(SOAPY_SDR_RX, ch));
        } else {
          if (is_cbrs) {
            if (cfg_->Freq() > 3e9) {
              cl_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "ATTN", -6);  //[-18,0]
            } else if ((cfg_->Freq() > 2e9) && (cfg_->Freq() < 3e9)) {
              cl_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "ATTN",
                                       -18);  //[-18,0]
            } else {
              cl_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "ATTN",
                                       -12);  //[-18,0]
            }
            cl_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "LNA2", 17);  //[0,17]
          }

          cl_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "LNA",
                                   ch != 0u
                                       ? cfg_->ClientRxGainB(tid)
                                       : cfg_->ClientRxGainA(tid));  //[0,30]
          cl_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "TIA", 0);      //[0,12]
          cl_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "PGA", 0);      //[-12,19]

          if (is_cbrs) {
            cl_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "ATTN",
                                     -6);  //[-18,0] by 3
            cl_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "PA2", 0);  //[0|15]
          }
          cl_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);  //[0,12]
          cl_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "PAD",
                                   ch != 0u
                                       ? cfg_->ClientTxGainB(tid)
                                       : cfg_->ClientTxGainA(tid));  //[0,30]
        }
      } else if (sdr_label.find("VGER") != std::string::npos) {
        double rxgain_a = std::min(30.0, cfg_->ClientRxGainA(tid));
        double rxgain_b = std::min(30.0, cfg_->ClientRxGainB(tid));
        double txgain_a = std::min(50.0, cfg_->ClientTxGainA(tid));
        double txgain_b = std::min(50.0, cfg_->ClientTxGainB(tid));
        cl_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "LNA",
                                 ch != 0u ? rxgain_a : rxgain_b);  //[0,30]
        cl_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "PAD",
                                 ch != 0u ? txgain_a : txgain_b);  //[0,52]
      } else {
        AGORA_LOG_ERROR("SDR is not supported!");
      }
    } else {
      cl_stn_.at(tid)->setGain(
          SOAPY_SDR_RX, ch, "PGA0",
          ch != 0u ? cfg_->ClientRxGainB(tid) : cfg_->ClientRxGainA(tid));
      cl_stn_.at(tid)->setGain(
          SOAPY_SDR_TX, ch, "PGA0",
          ch != 0u ? cfg_->ClientTxGainB(tid) : cfg_->ClientTxGainA(tid));
    }
  }

  for (auto ch : channels) {
    // cl_stn_.at(tid)->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
    // cl_stn_.at(tid)->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
    if (kUseUHD == false) {
      cl_stn_.at(tid)->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }
  }
  this->num_client_radios_initialized_.fetch_add(1);
}

bool ClientRadioConfig::RadioStart() {
  // send through the first radio for now
  // int beacon_ant = 1;
  int flags = SOAPY_SDR_WAIT_TRIGGER;

  for (size_t i = 0; i < this->radio_num_; i++) {
    if (cfg_->UeHwFramer()) {
      nlohmann::json conf;
      conf["tdd_enabled"] = true;
      conf["frame_mode"] = "continuous_resync";
      int max_frame =
          (int)(2.0f /
                ((cfg_->SampsPerSymbol() * cfg_->Frame().NumTotalSyms()) /
                 cfg_->Rate()));
      conf["max_frame"] = max_frame;
      conf["dual_pilot"] = (cfg_->NumUeChannels() == 2);
      auto tdd_sched = cfg_->Frame().FrameIdentifier();
      for (size_t s = 0; s < cfg_->Frame().FrameIdentifier().length(); s++) {
        char c = cfg_->Frame().FrameIdentifier().at(s);
        if (c == 'B') {
          tdd_sched.replace(s, 1, "R");  // Dummy RX used in PHY scheduler
        } else if (c == 'P' and ((cfg_->NumUeChannels() == 1 and
                                  cfg_->Frame().GetPilotSymbol(i) != s) or
                                 (cfg_->NumUeChannels() == 2 and
                                  (cfg_->Frame().GetPilotSymbol(2 * i) != s and
                                   cfg_->Frame().GetPilotSymbol(i * 2 + 1) !=
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
      std::cout << "Client " << i << " Frame: " << tdd_sched << std::endl;
      std::vector<std::string> jframes;
      jframes.push_back(tdd_sched);
      conf["frames"] = jframes;
      conf["symbol_size"] = cfg_->SampsPerSymbol();
      std::string conf_string = conf.dump();
      cl_stn_.at(i)->writeSetting("TDD_CONFIG", conf_string);
      // beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) +
      // 82 (Client FE Delay)
      int cl_trig_offset = cfg_->BeaconLen() + 249;
      int sf_start = cl_trig_offset / cfg_->SampsPerSymbol();
      int sp_start = cl_trig_offset % cfg_->SampsPerSymbol();
      cl_stn_.at(i)->setHardwareTime(
          SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, cfg_->Rate()),
          "TRIGGER");
      cl_stn_.at(i)->writeSetting(
          "TX_SW_DELAY",
          "30");  // experimentally good value for dev front-end
      cl_stn_.at(i)->writeSetting("TDD_MODE", "true");
      for (char const& c : cfg_->UeChannel()) {
        std::string tx_ram = "TX_RAM_";
        cl_stn_.at(i)->writeRegisters(tx_ram + c, 0, cfg_->Pilot());
      }
      cl_stn_.at(i)->activateStream(rx_streams_.at(i), flags);
      cl_stn_.at(i)->activateStream(tx_streams_.at(i), flags);

      std::string corr_conf_string =
          R"({"corr_enabled":true,"corr_threshold":)" + std::to_string(1) + "}";
      cl_stn_.at(i)->writeSetting("CORR_CONFIG", corr_conf_string);
      cl_stn_.at(i)->writeRegisters("CORR_COE", 0, cfg_->Coeffs());

      cl_stn_.at(i)->writeSetting("CORR_START",
                                  (cfg_->UeChannel() == "B") ? "B" : "A");
    } else {
      if (!kUseUHD) {
        cl_stn_.at(i)->setHardwareTime(0, "TRIGGER");
        cl_stn_.at(i)->activateStream(rx_streams_.at(i), flags);
        cl_stn_.at(i)->activateStream(tx_streams_.at(i), flags);
      } else {
        cl_stn_.at(i)->setHardwareTime(0, "UNKNOWN_PPS");
        cl_stn_.at(i)->activateStream(this->rx_streams_.at(i),
                                      SOAPY_SDR_HAS_TIME, 1e9, 0);
        cl_stn_.at(i)->activateStream(this->tx_streams_.at(i),
                                      SOAPY_SDR_HAS_TIME, 1e9, 0);
      }
    }
  }
  std::cout << "radio start done!" << std::endl;
  return true;
}

void ClientRadioConfig::Go() {
  if (!kUseUHD) {
    for (size_t i = 0; i < this->radio_num_; i++) {
      //std::cout << "triggering Iris ..." << i << std::endl;
      cl_stn_.at(i)->writeSetting("TRIGGER_GEN", "");
    }
  }
}

int ClientRadioConfig::RadioTx(size_t r /*radio id*/, void** buffs,
                               size_t num_samps, int flags,
                               long long& frameTime) {
  int tx_flags = 0;
  if (flags == 1) {
    tx_flags = SOAPY_SDR_HAS_TIME;
  } else if (flags == 2) {
    tx_flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
  }
  int w(0);
  if (cfg_->UeHwFramer()) {
    w = cl_stn_.at(r)->writeStream(this->tx_streams_.at(r), buffs, num_samps,
                                   tx_flags, frameTime, 1000000);
  } else {
    long long frame_time_ns = SoapySDR::ticksToTimeNs(frameTime, cfg_->Rate());
    w = cl_stn_.at(r)->writeStream(this->tx_streams_.at(r), buffs, num_samps,
                                   tx_flags, frame_time_ns, 1000000);
  }
  if (kDebugRadioTX) {
    size_t chan_mask;
    long timeout_us(0);
    int status_flag = 0;
    int s = cl_stn_.at(r)->readStreamStatus(this->tx_streams_.at(r), chan_mask,
                                            status_flag, frameTime, timeout_us);
    std::cout << "radio " << r << " tx returned " << w << " and status " << s
              << " when flags was " << flags << std::endl;
  }
  return w;
}

int ClientRadioConfig::RadioRx(size_t r /*radio id*/, void** buffs,
                               size_t num_samps, long long& frameTime) {
  static constexpr size_t kRxTimeoutuS = 1000000;
  int rx_flags = SOAPY_SDR_END_BURST;
  if (r < radio_num_) {
    int ret(0);
    if (cfg_->UeHwFramer()) {
      ret = cl_stn_.at(r)->readStream(this->rx_streams_.at(r), buffs, num_samps,
                                      rx_flags, frameTime, kRxTimeoutuS);
    } else {
      long long frame_time_ns = 0;
      ret = cl_stn_.at(r)->readStream(this->rx_streams_.at(r), buffs, num_samps,
                                      rx_flags, frame_time_ns, kRxTimeoutuS);
      frameTime = SoapySDR::timeNsToTicks(frame_time_ns, cfg_->Rate());
    }
    if (kDebugRadioRX) {
      if (ret != static_cast<int>(num_samps)) {
        std::cout << "invalid return " << ret << " from radio " << r
                  << std::endl;
      } else {
        std::cout << "radio " << r << "received " << ret << std::endl;
      }
    }
    return ret;
  }
  std::cout << "invalid radio id " << r << std::endl;
  return 0;
}

void ClientRadioConfig::DrainBuffers() {
  std::vector<std::vector<std::complex<int16_t>>> sample_storage(
      cfg_->NumUeChannels(),
      std::vector<std::complex<int16_t>>(cfg_->SampsPerSymbol(),
                                         std::complex<int16_t>(0, 0)));
  std::vector<void*> rx_buffs;
  rx_buffs.reserve(sample_storage.size());
  for (auto& buff : sample_storage) {
    rx_buffs.push_back(buff.data());
  }
  for (size_t i = 0; i < cfg_->UeNum(); i++) {
    ClientRadioConfig::DrainRxBuffer(cl_stn_.at(i), rx_streams_.at(i), rx_buffs,
                                     cfg_->SampsPerSymbol());
  }
}

void ClientRadioConfig::DrainRxBuffer(SoapySDR::Device* dev,
                                      SoapySDR::Stream* istream,
                                      std::vector<void*> buffs,
                                      size_t symSamp) {
  long long frame_time = 0;
  int flags = 0;
  int r = 0;
  int i = 0;
  long timeout_us(0);
  while (r != -1) {
    r = dev->readStream(istream, buffs.data(), symSamp, flags, frame_time,
                        timeout_us);
    i++;
  }
  // std::cout << "Number of reads needed to drain: " << i << std::endl;
}

int ClientRadioConfig::Triggers(int i) {
  return std::stoi(cl_stn_.at(i)->readSetting("TRIGGER_COUNT"));
}

void ClientRadioConfig::ReadSensors() {
  for (size_t i = 0; i < this->radio_num_; i++) {
    std::cout << "TEMPs on Iris " << i << std::endl;
    std::cout << "ZYNQ_TEMP: " << cl_stn_.at(i)->readSensor("ZYNQ_TEMP")
              << std::endl;
    std::cout << "LMS7_TEMP  : " << cl_stn_.at(i)->readSensor("LMS7_TEMP")
              << std::endl;
    std::cout << "FE_TEMP  : " << cl_stn_.at(i)->readSensor("FE_TEMP")
              << std::endl;
    std::cout << "TX0 TEMP  : "
              << cl_stn_.at(i)->readSensor(SOAPY_SDR_TX, 0, "TEMP")
              << std::endl;
    std::cout << "TX1 TEMP  : "
              << cl_stn_.at(i)->readSensor(SOAPY_SDR_TX, 1, "TEMP")
              << std::endl;
    std::cout << "RX0 TEMP  : "
              << cl_stn_.at(i)->readSensor(SOAPY_SDR_RX, 0, "TEMP")
              << std::endl;
    std::cout << "RX1 TEMP  : "
              << cl_stn_.at(i)->readSensor(SOAPY_SDR_RX, 1, "TEMP")
              << std::endl;
    std::cout << std::endl;
  }
}

void ClientRadioConfig::RadioStop() {
  std::string corr_conf_str = "{\"corr_enabled\":false}";
  std::string tdd_conf_str = "{\"tdd_enabled\":false}";
  for (size_t i = 0; i < radio_num_; i++) {
    cl_stn_.at(i)->deactivateStream(rx_streams_.at(i));
    cl_stn_.at(i)->deactivateStream(tx_streams_.at(i));
    if (cfg_->UeHwFramer()) {
      cl_stn_.at(i)->writeSetting("TDD_MODE", "false");
      cl_stn_.at(i)->writeSetting("TDD_CONFIG", tdd_conf_str);
      cl_stn_.at(i)->writeSetting("CORR_CONFIG", corr_conf_str);
    }
  }
}

ClientRadioConfig::~ClientRadioConfig() {
  for (size_t i = 0; i < radio_num_; i++) {
    cl_stn_.at(i)->closeStream(rx_streams_.at(i));
    cl_stn_.at(i)->closeStream(tx_streams_.at(i));
    SoapySDR::Device::unmake(cl_stn_.at(i));
  }
}
