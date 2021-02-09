#include "client_radio.h"

#include "comms-lib.h"

std::atomic<size_t> num_client_radios_initialized;

ClientRadioConfig::ClientRadioConfig(Config* cfg) : cfg_(cfg) {
  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  // load channels
  auto channels = Utils::StrToChannels(cfg_->channel_);

  this->radio_num_ = cfg_->n_radios_;
  this->antenna_num_ = radio_num_ * cfg_->n_channels_;
  std::cout << "radio num is " << this->radio_num_ << std::endl;

  cl_stn_.resize(radio_num_);
  tx_streams_.resize(radio_num_);
  rx_streams_.resize(radio_num_);

  std::vector<ClientRadioConfigContext> client_radio_config_ctx(
      this->radio_num_);
  for (size_t i = 0; i < this->radio_num_; i++) {
    auto* context = &client_radio_config_ctx[i];
    context->ptr_ = this;
    context->tid_ = i;
#ifdef THREADED_INIT
    pthread_t init_thread;
    if (pthread_create(&init_thread, NULL, InitClientRadioLaunch, context) !=
        0) {
      perror("init thread create failed");
      std::exit(0);
    }
#else
    initClientRadio(context);
#endif
  }

#ifdef THREADED_INIT
  size_t num_checks = 0;
  while (num_client_radios_initialized != this->radio_num_) {
    size_t num_client_radios_initialized = num_client_radios_initialized;
    num_checks++;
    if (num_checks > 1e9) {
      std::printf(
          "RadioConfig: Waiting for radio initialization, %zu of %zu "
          "ready\n",
          num_client_radios_initialized, this->radio_num_);
      num_checks = 0;
    }
  }
#endif

  for (size_t i = 0; i < this->radio_num_; i++) {
    std::cout << cfg_->radio_ids_.at(i) << ": Front end "
              << cl_stn_[i]->getHardwareInfo()["frontend"] << std::endl;
    for (auto c : channels) {
      if (c < cl_stn_[i]->getNumChannels(SOAPY_SDR_RX)) {
        std::printf("RX Channel %zu\n", c);
        std::printf("Actual RX sample rate: %fMSps...\n",
                    (cl_stn_[i]->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
        std::printf("Actual RX frequency: %fGHz...\n",
                    (cl_stn_[i]->getFrequency(SOAPY_SDR_RX, c) / 1e9));
        std::printf("Actual RX gain: %f...\n",
                    (cl_stn_[i]->getGain(SOAPY_SDR_RX, c)));
        if (!kUseUHD) {
          std::printf("Actual RX LNA gain: %f...\n",
                      (cl_stn_[i]->getGain(SOAPY_SDR_RX, c, "LNA")));
          std::printf("Actual RX PGA gain: %f...\n",
                      (cl_stn_[i]->getGain(SOAPY_SDR_RX, c, "PGA")));
          std::printf("Actual RX TIA gain: %f...\n",
                      (cl_stn_[i]->getGain(SOAPY_SDR_RX, c, "TIA")));
          if (cl_stn_[i]->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            std::printf("Actual RX LNA1 gain: %f...\n",
                        (cl_stn_[i]->getGain(SOAPY_SDR_RX, c, "LNA1")));
            std::printf("Actual RX LNA2 gain: %f...\n",
                        (cl_stn_[i]->getGain(SOAPY_SDR_RX, c, "LNA2")));
          }
        }
        std::printf("Actual RX bandwidth: %fM...\n",
                    (cl_stn_[i]->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
        std::printf("Actual RX antenna: %s...\n",
                    (cl_stn_[i]->getAntenna(SOAPY_SDR_RX, c).c_str()));
      }
    }

    for (auto c : channels) {
      if (c < cl_stn_[i]->getNumChannels(SOAPY_SDR_TX)) {
        std::printf("TX Channel %zu\n", c);
        std::printf("Actual TX sample rate: %fMSps...\n",
                    (cl_stn_[i]->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
        std::printf("Actual TX frequency: %fGHz...\n",
                    (cl_stn_[i]->getFrequency(SOAPY_SDR_TX, c) / 1e9));
        std::printf("Actual TX gain: %f...\n",
                    (cl_stn_[i]->getGain(SOAPY_SDR_TX, c)));
        if (!kUseUHD) {
          std::printf("Actual TX PAD gain: %f...\n",
                      (cl_stn_[i]->getGain(SOAPY_SDR_TX, c, "PAD")));
          std::printf("Actual TX IAMP gain: %f...\n",
                      (cl_stn_[i]->getGain(SOAPY_SDR_TX, c, "IAMP")));
          if (cl_stn_[i]->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            std::printf("Actual TX PA1 gain: %f...\n",
                        (cl_stn_[i]->getGain(SOAPY_SDR_TX, c, "PA1")));
            std::printf("Actual TX PA2 gain: %f...\n",
                        (cl_stn_[i]->getGain(SOAPY_SDR_TX, c, "PA2")));
            std::printf("Actual TX PA3 gain: %f...\n",
                        (cl_stn_[i]->getGain(SOAPY_SDR_TX, c, "PA3")));
          }
        }
        std::printf("Actual TX bandwidth: %fM...\n",
                    (cl_stn_[i]->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
        std::printf("Actual TX antenna: %s...\n",
                    (cl_stn_[i]->getAntenna(SOAPY_SDR_TX, c).c_str()));
      }
    }
    std::cout << std::endl;
  }

  std::cout << "radio init done!" << std::endl;
}

void* ClientRadioConfig::InitClientRadioLaunch(void* in_context) {
  auto* context = (ClientRadioConfigContext*)in_context;
  context->ptr_->InitClientRadio(context);
  return 0;
}

void ClientRadioConfig::InitClientRadio(ClientRadioConfigContext* in_context) {
  size_t i = in_context->tid_;
  Config* cfg = cfg_;

  // load channels
  auto channels = Utils::StrToChannels(cfg->channel_);

  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  args["timeout"] = "1000000";
  if (!kUseUHD) {
    args["driver"] = "iris";
    args["serial"] = cfg->radio_ids_.at(i);
  } else {
    args["driver"] = "uhd";
    args["addr"] = cfg->radio_ids_.at(i);
  }
  cl_stn_[i] = SoapySDR::Device::make(args);
  for (auto ch : channels) {
    cl_stn_[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate_);
    cl_stn_[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate_);
  }
  rx_streams_[i] =
      cl_stn_[i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
  tx_streams_[i] =
      cl_stn_[i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);

  // resets the DATA_clk domain logic.
  if (!kUseUHD) {
    cl_stn_[i]->writeSetting("RESET_DATA_LOGIC", "");
  }

  // use the TRX antenna port for both tx and rx
  for (auto ch : channels) {
    if (!kUseUHD) {
      cl_stn_[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    } else {
      cl_stn_[i]->setAntenna(SOAPY_SDR_RX, ch, "RX2");
      cl_stn_[i]->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
    }
  }

  SoapySDR::Kwargs info = cl_stn_[i]->getHardwareInfo();
  for (auto ch : channels) {
    cl_stn_[i]->setBandwidth(SOAPY_SDR_RX, ch, cfg->bw_filter_);
    cl_stn_[i]->setBandwidth(SOAPY_SDR_TX, ch, cfg->bw_filter_);

    // clStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
    // clStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

    cl_stn_[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg->radio_rf_freq_);
    cl_stn_[i]->setFrequency(SOAPY_SDR_RX, ch, "BB", kUseUHD ? 0 : cfg->nco_);
    cl_stn_[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg->radio_rf_freq_);
    cl_stn_[i]->setFrequency(SOAPY_SDR_TX, ch, "BB", kUseUHD ? 0 : cfg->nco_);

    if (!kUseUHD) {
      // Unified gains for both lime and frontend
      if (cfg_->SingleGain()) {
        // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
        cl_stn_[i]->setGain(SOAPY_SDR_RX, ch,
                            ch != 0u ? cfg_->rx_gain_b_ : cfg_->rx_gain_a_);
        // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
        cl_stn_[i]->setGain(
            SOAPY_SDR_TX, ch,
            ch != 0u ? cfg_->tx_gain_b_ + cfg->client_gain_adj_b_[i]
                     : cfg_->tx_gain_a_ + cfg->client_gain_adj_a_[i]);
      } else {
        if (info["frontend"].find("CBRS") != std::string::npos) {
          if (cfg->freq_ > 3e9) {
            cl_stn_[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -6);  //[-18,0]
          } else if (cfg->freq_ > 2e9 && cfg->freq_ < 3e9) {
            cl_stn_[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -18);  //[-18,0]
          } else {
            cl_stn_[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -12);  //[-18,0]
          }
          cl_stn_[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17);  //[0,17]
        }

        cl_stn_[i]->setGain(
            SOAPY_SDR_RX, ch, "LNA",
            ch != 0u ? cfg->rx_gain_b_ : cfg->rx_gain_a_);  //[0,30]
        cl_stn_[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0);    //[0,12]
        cl_stn_[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0);    //[-12,19]

        if (info["frontend"].find("CBRS") != std::string::npos) {
          cl_stn_[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);  //[-18,0] by 3
          cl_stn_[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0);    //[0|15]
        }
        cl_stn_[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);  //[0,12]
        cl_stn_[i]->setGain(
            SOAPY_SDR_TX, ch, "PAD",
            ch != 0u ? cfg->tx_gain_b_ + cfg->client_gain_adj_b_[i]
                     : cfg->tx_gain_a_ + cfg->client_gain_adj_a_[i]);  //[0,30]
      }
    } else {
      cl_stn_[i]->setGain(SOAPY_SDR_RX, ch, "PGA0",
                          ch != 0u ? cfg->rx_gain_b_ : cfg->rx_gain_a_);
      cl_stn_[i]->setGain(SOAPY_SDR_TX, ch, "PGA0",
                          ch != 0u
                              ? cfg->tx_gain_b_ + cfg->client_gain_adj_b_[i]
                              : cfg->tx_gain_a_ + cfg->client_gain_adj_a_[i]);
    }
  }

  for (auto ch : channels) {
    // clStn[i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
    // clStn[i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
    if (!kUseUHD) {
      cl_stn_[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }
  }

  num_client_radios_initialized++;
}

bool ClientRadioConfig::RadioStart() {
  // send through the first radio for now
  // int beacon_ant = 1;
  int flags(0);  // = SOAPY_SDR_WAIT_TRIGGER;
  std::vector<unsigned> zeros(cfg_->samps_per_symbol_, 0);
  std::vector<uint32_t> beacon = cfg_->beacon_;
  std::vector<unsigned> beacon_weights(cfg_->n_antennas_);

  std::vector<uint32_t> pilot = cfg_->pilot_;

  std::vector<std::string> tdd_sched;
  tdd_sched.resize(this->radio_num_);
  for (size_t r = 0; r < radio_num_; r++) {
    tdd_sched[r] = cfg_->frames_[0];
    for (size_t s = 0; s < cfg_->frames_[0].size(); s++) {
      char c = cfg_->frames_[0].at(s);
      if (c == 'P' and
          ((cfg_->n_channels_ == 1 and cfg_->pilot_symbols_[0][r] != s) or
           (cfg_->n_channels_ == 2 and (cfg_->pilot_symbols_[0][2 * r] != s and
                                        cfg_->pilot_symbols_[0][r * 2 + 1] !=
                                            s)))) {  // TODO: change this for
        // orthogonal pilots
        tdd_sched[r].replace(s, 1, "G");
      } else if (c == 'U') {
        tdd_sched[r].replace(s, 1, "T");
      } else if (c == 'D') {
        tdd_sched[r].replace(s, 1, "R");
      } else if (c != 'P') {
        tdd_sched[r].replace(s, 1, "G");
      }
    }
    std::cout << tdd_sched[r] << std::endl;
  }
  // beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) +
  // 82 (Client FE Delay)
  int cl_trig_offset = cfg_->beacon_len_ + 249;
  int sf_start = cl_trig_offset / cfg_->samps_per_symbol_;
  int sp_start = cl_trig_offset % cfg_->samps_per_symbol_;
  for (size_t i = 0; i < this->radio_num_; i++) {
    if (cfg_->hw_framer_) {
      json conf;
      conf["tdd_enabled"] = true;
      conf["frame_mode"] = "continuous_resync";
      int max_frame =
          (int)(2.0 / ((cfg_->samps_per_symbol_ * cfg_->symbol_num_perframe_) /
                       cfg_->rate_));
      conf["max_frame"] = max_frame;
      conf["dual_pilot"] = (cfg_->n_channels_ == 2);
      std::vector<std::string> jframes;
      jframes.push_back(tdd_sched[i]);
      conf["frames"] = jframes;
      conf["symbol_size"] = cfg_->samps_per_symbol_;
      std::string conf_string = conf.dump();
      cl_stn_[i]->writeSetting("TDD_CONFIG", conf_string);
      cl_stn_[i]->setHardwareTime(
          SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, cfg_->rate_),
          "TRIGGER");
      cl_stn_[i]->writeSetting(
          "TX_SW_DELAY",
          "30");  // experimentally good value for dev front-end
      cl_stn_[i]->writeSetting("TDD_MODE", "true");
      for (char const& c : cfg_->channel_) {
        std::string tx_ram = "TX_RAM_";
        cl_stn_[i]->writeRegisters(tx_ram + c, 0, pilot);
      }
      cl_stn_[i]->activateStream(this->rx_streams_[i], flags, 0);
      cl_stn_[i]->activateStream(this->tx_streams_[i]);

      std::string corr_conf_string =
          "{\"corr_enabled\":true,\"corr_threshold\":" + std::to_string(1) +
          "}";
      cl_stn_[i]->writeSetting("CORR_CONFIG", corr_conf_string);
      cl_stn_[i]->writeRegisters("CORR_COE", 0, cfg_->coeffs_);

      cl_stn_[i]->writeSetting("CORR_START",
                               (cfg_->channel_ == "B") ? "B" : "A");
    } else {
      if (!kUseUHD) {
        cl_stn_[i]->setHardwareTime(0, "TRIGGER");
        cl_stn_[i]->activateStream(this->rx_streams_[i], flags, 0);
        cl_stn_[i]->activateStream(this->tx_streams_[i]);
        cl_stn_[i]->writeSetting("TRIGGER_GEN", "");
      } else {
        cl_stn_[i]->setHardwareTime(0, "UNKNOWN_PPS");
        cl_stn_[i]->activateStream(this->rx_streams_[i], SOAPY_SDR_HAS_TIME,
                                   1e9, 0);
        cl_stn_[i]->activateStream(this->tx_streams_[i], SOAPY_SDR_HAS_TIME,
                                   1e9, 0);
      }
    }
  }

  std::cout << "radio start done!" << std::endl;
  return true;
}

void ClientRadioConfig::Go() {
  if (!kUseUHD) {
    if (hubs_.empty()) {
      // std::cout << "triggering first Iris ..." << std::endl;
      cl_stn_[0]->writeSetting("TRIGGER_GEN", "");
    } else {
      // std::cout << "triggering Hub ..." << std::endl;
      hubs_[0]->writeSetting("TRIGGER_GEN", "");
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
  if (cfg_->hw_framer_) {
    w = cl_stn_[r]->writeStream(this->tx_streams_[r], buffs, num_samps,
                                tx_flags, frameTime, 1000000);
  } else {
    long long frame_time_ns = SoapySDR::ticksToTimeNs(frameTime, cfg_->rate_);
    w = cl_stn_[r]->writeStream(this->tx_streams_[r], buffs, num_samps,
                                tx_flags, frame_time_ns, 1000000);
  }
  if (kDebugRadioTX) {
    size_t chan_mask;
    long timeout_us(0);
    int status_flag = 0;
    int s = cl_stn_[r]->readStreamStatus(this->tx_streams_[r], chan_mask,
                                         status_flag, frameTime, timeout_us);
    std::cout << "radio " << r << " tx returned " << w << " and status " << s
              << " when flags was " << flags << std::endl;
  }
  return w;
}

int ClientRadioConfig::RadioRx(size_t r /*radio id*/, void** buffs,
                               size_t num_samps, long long& frameTime) {
  int flags(0);
  if (r < this->radio_num_) {
    int ret(0);
    if (cfg_->hw_framer_) {
      ret = cl_stn_[r]->readStream(this->rx_streams_[r], buffs, num_samps,
                                   flags, frameTime, 1000000);
    } else {
      long long frame_time_ns = 0;
      ret = cl_stn_[r]->readStream(this->rx_streams_[r], buffs, num_samps,
                                   flags, frame_time_ns, 1000000);
      frameTime = SoapySDR::timeNsToTicks(frame_time_ns, cfg_->rate_);
    }
    if (kDebugRadioRX) {
      if (ret != (int)num_samps) {
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
  std::vector<std::complex<int16_t>> dummy_buff0(cfg_->samps_per_symbol_);
  std::vector<std::complex<int16_t>> dummy_buff1(cfg_->samps_per_symbol_);
  std::vector<void*> dummybuffs(2);
  dummybuffs[0] = dummy_buff0.data();
  dummybuffs[1] = dummy_buff1.data();
  for (size_t i = 0; i < cfg_->n_radios_; i++) {
    ClientRadioConfig::DrainRxBuffer(cl_stn_[i], rx_streams_[i], dummybuffs,
                                     cfg_->samps_per_symbol_);
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
  return std::stoi(cl_stn_[i]->readSetting("TRIGGER_COUNT"));
}

void ClientRadioConfig::ReadSensors() {
  for (size_t i = 0; i < this->radio_num_; i++) {
    std::cout << "TEMPs on Iris " << i << std::endl;
    std::cout << "ZYNQ_TEMP: " << cl_stn_[i]->readSensor("ZYNQ_TEMP")
              << std::endl;
    std::cout << "LMS7_TEMP  : " << cl_stn_[i]->readSensor("LMS7_TEMP")
              << std::endl;
    std::cout << "FE_TEMP  : " << cl_stn_[i]->readSensor("FE_TEMP")
              << std::endl;
    std::cout << "TX0 TEMP  : "
              << cl_stn_[i]->readSensor(SOAPY_SDR_TX, 0, "TEMP") << std::endl;
    std::cout << "TX1 TEMP  : "
              << cl_stn_[i]->readSensor(SOAPY_SDR_TX, 1, "TEMP") << std::endl;
    std::cout << "RX0 TEMP  : "
              << cl_stn_[i]->readSensor(SOAPY_SDR_RX, 0, "TEMP") << std::endl;
    std::cout << "RX1 TEMP  : "
              << cl_stn_[i]->readSensor(SOAPY_SDR_RX, 1, "TEMP") << std::endl;
    std::cout << std::endl;
  }
}

void ClientRadioConfig::RadioStop() {
  std::string corr_conf_str = "{\"corr_enabled\":false}";
  std::string tdd_conf_str = "{\"tdd_enabled\":false}";
  for (size_t i = 0; i < this->radio_num_; i++) {
    cl_stn_[i]->deactivateStream(this->rx_streams_[i]);
    cl_stn_[i]->deactivateStream(this->tx_streams_[i]);
    if (cfg_->hw_framer_) {
      cl_stn_[i]->writeSetting("TDD_MODE", "false");
      cl_stn_[i]->writeSetting("TDD_CONFIG", tdd_conf_str);
      cl_stn_[i]->writeSetting("CORR_CONFIG", corr_conf_str);
    }
  }
}

ClientRadioConfig::~ClientRadioConfig() {
  for (size_t i = 0; i < this->radio_num_; i++) {
    cl_stn_[i]->closeStream(this->rx_streams_[i]);
    cl_stn_[i]->closeStream(this->tx_streams_[i]);
    SoapySDR::Device::unmake(cl_stn_[i]);
  }
}
