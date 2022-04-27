/** @file Radio.h
  * @brief Defination file for the Radio class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#include "radio.h"

#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Time.hpp"
#include "logger.h"
#include "symbols.h"

constexpr size_t kSoapyMakeMaxAttempts = 3;
constexpr bool kPrintRadioSettings = false;
constexpr double kAttnMax = -18.0f;

Radio::Radio()
    : id_(0),
      num_channels_(2),
      dev_(nullptr),
      rxs_(nullptr),
      txs_(nullptr),
      cfg_(nullptr) {}

Radio::~Radio() { Close(); }

void Radio::Close() {
  id_ = 0;
  if (rxs_ != nullptr) {
    dev_->closeStream(rxs_);
  }
  rxs_ = nullptr;
  if (txs_ != nullptr) {
    dev_->closeStream(txs_);
  }
  txs_ = nullptr;
  if (dev_ != nullptr) {
    SoapySDR::Device::unmake(dev_);
  }
  dev_ = nullptr;
}

void Radio::Init(const Config* cfg, size_t id) {
  if (dev_ == nullptr) {
    id_ = id;
    cfg_ = cfg;

    auto channels = Utils::StrToChannels(cfg->Channel());
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["timeout"] = "1000000";
    if (kUseUHD == false) {
      args["driver"] = "iris";
      args["serial"] = cfg->RadioId().at(id);
    } else {
      args["driver"] = "uhd";
      args["addr"] = cfg->RadioId().at(id);
    }

    for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
      try {
        dev_ = SoapySDR::Device::make(args);
        break;
      } catch (const std::runtime_error& e) {
        const auto* message = e.what();
        AGORA_LOG_ERROR("Radio::Init[%zu] - Soapy error try %zu -- %s\n", id,
                        tries, message);
      }
    }
    if (dev_ == nullptr) {
      AGORA_LOG_ERROR(
          "SoapySDR failed to locate the radio %s in %zu attempts\n",
          cfg->RadioId().at(id).c_str(), kSoapyMakeMaxAttempts);
      throw std::runtime_error("SoapySDR failed to locate the radio with id " +
                               cfg->RadioId().at(id));
    }
    // resets the DATA_clk domain logic.
    dev_->writeSetting("RESET_DATA_LOGIC", "");
    rxs_ = dev_->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    if (rxs_ == nullptr) {
      AGORA_LOG_ERROR(
          "Radio::Init[%zu] - Radio rx control plane could not be configured\n",
          id);
    }
    txs_ = dev_->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
    if (rxs_ == nullptr) {
      AGORA_LOG_ERROR(
          "Radio::Init[%zu] - Radio tx control plane could not be configured\n",
          id);
    }
  } else {
    AGORA_LOG_ERROR("Radio::Init[%zu] - Radio already has been init\n", id);
  }
}

void Radio::Setup() {
  auto channels = Utils::StrToChannels(cfg_->Channel());
  for (auto ch : channels) {
    dev_->setSampleRate(SOAPY_SDR_RX, ch, cfg_->Rate());
    dev_->setSampleRate(SOAPY_SDR_TX, ch, cfg_->Rate());
  }

  // use the TRX antenna port for both tx and rx
  for (auto ch : channels) {
    if (kUseUHD == false) {
      dev_->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    } else {
      dev_->setAntenna(SOAPY_SDR_RX, ch, "RX2");
      dev_->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
    }
  }

  SoapySDR::Kwargs info = dev_->getHardwareInfo();
  for (auto ch : channels) {
    if (kUseUHD == false) {
      dev_->setBandwidth(SOAPY_SDR_RX, ch, cfg_->BwFilter());
      dev_->setBandwidth(SOAPY_SDR_TX, ch, cfg_->BwFilter());
    }

    // dev_->setSampleRate(SOAPY_SDR_RX, ch, cfg->Rate());
    // dev_->setSampleRate(SOAPY_SDR_TX, ch, cfg->Rate());

    dev_->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg_->RadioRfFreq());
    dev_->setFrequency(SOAPY_SDR_RX, ch, "BB", kUseUHD ? 0 : cfg_->Nco());
    dev_->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg_->RadioRfFreq());
    dev_->setFrequency(SOAPY_SDR_TX, ch, "BB", kUseUHD ? 0 : cfg_->Nco());

    if (kUseUHD == false) {
      // Unified gains for both lime and frontend
      if (cfg_->SingleGain()) {
        // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
        dev_->setGain(SOAPY_SDR_RX, ch,
                      ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());
        // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
        dev_->setGain(SOAPY_SDR_TX, ch,
                      ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());
      } else {
        if (info["frontend"].find("CBRS") != std::string::npos) {
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
        dev_->setGain(SOAPY_SDR_RX, ch, "LNA",
                      ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());
        //[0,12]
        dev_->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
        //[-12,19]
        dev_->setGain(SOAPY_SDR_RX, ch, "PGA", 0);

        if (info["frontend"].find("CBRS") != std::string::npos) {
          //[-18,0] by 3
          dev_->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);
          //[0|15]
          dev_->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
        }
        //[-12,12]
        dev_->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
        //[0,30]
        dev_->setGain(SOAPY_SDR_TX, ch, "PAD",
                      ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());
      }
    } else {
      dev_->setGain(SOAPY_SDR_RX, ch, "PGA0",
                    ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());
      dev_->setGain(SOAPY_SDR_TX, ch, "PGA0",
                    ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());
    }
  }

  for (auto ch : channels) {
    dev_->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
  }
}

void Radio::Activate() {
  if (kUseUHD == false) {
    dev_->setHardwareTime(0, "TRIGGER");
    if (cfg_->HwFramer() == true) {
      dev_->activateStream(rxs_);
      dev_->activateStream(txs_);
    }
  } else {
    dev_->setHardwareTime(0, "UNKNOWN_PPS");
    dev_->activateStream(rxs_, SOAPY_SDR_HAS_TIME, 1e9, 0);
    dev_->activateStream(txs_, SOAPY_SDR_HAS_TIME, 1e9, 0);
  }
}

void Radio::Deactivate() {
  const std::string corr_conf_str = "{\"corr_enabled\":false}";
  const std::string tdd_conf_str = "{\"tdd_enabled\":false}";
  dev_->deactivateStream(rxs_);
  dev_->deactivateStream(txs_);
  dev_->writeSetting("TDD_MODE", "false");
  dev_->writeSetting("TDD_CONFIG", tdd_conf_str);
}

int Radio::Tx(const void* const* tx_buffs, size_t tx_size, int flags,
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
    int s = dev_->readStreamStatus(txs_, chan_mask, status_flag, tx_time_ns,
                                   timeout_us);
    std::cout << "radio " << id_ << " tx returned " << w << " and status " << s
              << " when flags was " << flags << std::endl;
  }
  return w;
}

int Radio::Rx(void** rx_buffs, size_t rx_size, int rx_flags,
              long long& rx_time_ns) {
  int rx_status = 0;
  constexpr long kRxTimeout = 1;  // 1uS
  long long frame_time_ns = 0;
  rx_status = dev_->readStream(rxs_, rx_buffs, rx_size, rx_flags, frame_time_ns,
                               kRxTimeout);

  if (cfg_->HwFramer() == true) {
    rx_time_ns = frame_time_ns;
  } else {
    // for UHD device recv using ticks
    rx_time_ns = SoapySDR::timeNsToTicks(frame_time_ns, cfg_->Rate());
  }

  if (kDebugRadioRX) {
    if (rx_status == static_cast<int>(rx_size)) {
      std::cout << "Radio " << id_ << " received " << rx_status
                << " flags: " << rx_flags << " MTU " << dev_->getStreamMTU(rxs_)
                << std::endl;
    } else {
      if (!((rx_status == SOAPY_SDR_TIMEOUT) && (rx_flags == 0))) {
        std::cout << "Unexpected RadioRx return value " << rx_status
                  << " from radio " << id_ << " flags: " << rx_flags
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

int Radio::Rx(void** rx_buffs, long long& rx_time_ns) {
  return Rx(rx_buffs, cfg_->SampsPerSymbol(), SOAPY_SDR_END_BURST, rx_time_ns);
}

void Radio::Trigger() { dev_->writeSetting("TRIGGER_GEN", ""); }

void Radio::ReadSensor() const {
  std::cout << "TEMPs on Iris " << id_ << std::endl;
  std::cout << "ZYNQ_TEMP  : " << dev_->readSensor("ZYNQ_TEMP") << std::endl;
  std::cout << "LMS7_TEMP  : " << dev_->readSensor("LMS7_TEMP") << std::endl;
  std::cout << "FE_TEMP  : " << dev_->readSensor("FE_TEMP") << std::endl;
  std::cout << "TX0 TEMP  : " << dev_->readSensor(SOAPY_SDR_TX, 0, "TEMP")
            << std::endl;
  std::cout << "TX1 TEMP  : " << dev_->readSensor(SOAPY_SDR_TX, 1, "TEMP")
            << std::endl;
  std::cout << "RX0 TEMP  : " << dev_->readSensor(SOAPY_SDR_RX, 0, "TEMP")
            << std::endl;
  std::cout << "RX1 TEMP  : " << dev_->readSensor(SOAPY_SDR_RX, 1, "TEMP")
            << std::endl;
  std::cout << std::endl;
}

void Radio::PrintSettings() const {
  std::cout << id_ << ": Front end " << dev_->getHardwareInfo()["frontend"]
            << std::endl;

  auto channels = Utils::StrToChannels(cfg_->Channel());

  if (kPrintRadioSettings) {
    for (auto c : channels) {
      if (c < dev_->getNumChannels(SOAPY_SDR_RX)) {
        std::printf("RX Channel %zu\n", c);
        std::printf("Actual RX sample rate: %fMSps...\n",
                    (dev_->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
        std::printf("Actual RX frequency: %fGHz...\n",
                    (dev_->getFrequency(SOAPY_SDR_RX, c) / 1e9));
        std::printf("Actual RX gain: %f...\n",
                    (dev_->getGain(SOAPY_SDR_RX, c)));
        if (kUseUHD == false) {
          std::printf("Actual RX LNA gain: %f...\n",
                      (dev_->getGain(SOAPY_SDR_RX, c, "LNA")));
          std::printf("Actual RX PGA gain: %f...\n",
                      (dev_->getGain(SOAPY_SDR_RX, c, "PGA")));
          std::printf("Actual RX TIA gain: %f...\n",
                      (dev_->getGain(SOAPY_SDR_RX, c, "TIA")));
          if (dev_->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            std::printf("Actual RX LNA1 gain: %f...\n",
                        (dev_->getGain(SOAPY_SDR_RX, c, "LNA1")));
            std::printf("Actual RX LNA2 gain: %f...\n",
                        (dev_->getGain(SOAPY_SDR_RX, c, "LNA2")));
          }
        }
        std::printf("Actual RX bandwidth: %fM...\n",
                    (dev_->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
        std::printf("Actual RX antenna: %s...\n",
                    (dev_->getAntenna(SOAPY_SDR_RX, c).c_str()));
      }
    }

    for (auto c : channels) {
      if (c < dev_->getNumChannels(SOAPY_SDR_TX)) {
        std::printf("TX Channel %zu\n", c);
        std::printf("Actual TX sample rate: %fMSps...\n",
                    (dev_->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
        std::printf("Actual TX frequency: %fGHz...\n",
                    (dev_->getFrequency(SOAPY_SDR_TX, c) / 1e9));
        std::printf("Actual TX gain: %f...\n",
                    (dev_->getGain(SOAPY_SDR_TX, c)));
        if (kUseUHD == false) {
          std::printf("Actual TX PAD gain: %f...\n",
                      (dev_->getGain(SOAPY_SDR_TX, c, "PAD")));
          std::printf("Actual TX IAMP gain: %f...\n",
                      (dev_->getGain(SOAPY_SDR_TX, c, "IAMP")));
          if (dev_->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            std::printf("Actual TX PA1 gain: %f...\n",
                        (dev_->getGain(SOAPY_SDR_TX, c, "PA1")));
            std::printf("Actual TX PA2 gain: %f...\n",
                        (dev_->getGain(SOAPY_SDR_TX, c, "PA2")));
            std::printf("Actual TX PA3 gain: %f...\n",
                        (dev_->getGain(SOAPY_SDR_TX, c, "PA3")));
          }
        }
        std::printf("Actual TX bandwidth: %fM...\n",
                    (dev_->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
        std::printf("Actual TX antenna: %s...\n",
                    (dev_->getAntenna(SOAPY_SDR_TX, c).c_str()));
      }
    }
    std::cout << std::endl;
  }
}

void Radio::ClearSyncDelay() { dev_->writeSetting("SYNC_DELAYS", ""); }

void Radio::InitRefTx(size_t channel, double freq) {
  dev_->setFrequency(SOAPY_SDR_TX, channel, "RF", freq);
  dev_->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
}

void Radio::InitCalRx(size_t channel, double center_freq) {
  // must set TX "RF" Freq to make sure, we continue using the same LO for
  // rx cal
  dev_->setFrequency(SOAPY_SDR_TX, channel, "RF", center_freq);
  dev_->setFrequency(SOAPY_SDR_RX, channel, "RF", center_freq);
  dev_->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
  dev_->setDCOffsetMode(SOAPY_SDR_RX, channel, false);
}

void Radio::ResetTxGains() {
  for (size_t channel = 0; channel < num_channels_; channel++) {
    dev_->setGain(SOAPY_SDR_TX, channel, "PA2", 0);
    dev_->setGain(SOAPY_SDR_TX, channel, "PAD", 0);
    dev_->setGain(SOAPY_SDR_TX, channel, "IAMP", 0);
    dev_->setGain(SOAPY_SDR_TX, channel, "ATTN", kAttnMax);
  }
}

void Radio::ResetRxGains() {
  for (size_t channel = 0; channel < num_channels_; channel++) {
    dev_->setGain(SOAPY_SDR_RX, channel, "LNA", 0);
    dev_->setGain(SOAPY_SDR_RX, channel, "PGA", 0);
    dev_->setGain(SOAPY_SDR_RX, channel, "TIA", 0);
    dev_->setGain(SOAPY_SDR_RX, channel, "ATTN", kAttnMax);
    dev_->setGain(SOAPY_SDR_RX, channel, "LNA2", 14.0);
  }
}

void Radio::SetTxCalGain(size_t channel) {
  dev_->setGain(SOAPY_SDR_TX, channel, "PAD", 40);
}

void Radio::SetTxGain(size_t channel, const std::string& gain_stage,
                      double value) {
  dev_->setGain(SOAPY_SDR_TX, channel, gain_stage, value);
}

void Radio::SetRxGain(size_t channel, const std::string& gain_stage,
                      double value) {
  dev_->setGain(SOAPY_SDR_RX, channel, gain_stage, value);
}

std::vector<std::complex<float>> Radio::SnoopSamples(size_t channel,
                                                     size_t read_size) {
  std::vector<uint32_t> samps_int =
      dev_->readRegisters("RX_SNOOPER", channel, read_size);
  std::vector<std::complex<float>> samps =
      Utils::Uint32tocfloat(samps_int, "IQ");
  return samps;
}

void Radio::StartRefTx(size_t channel) {
  //Can we move this code to InitRefTx????
  dev_->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST",
                     std::to_string(1 << 14));
  dev_->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");
}

void Radio::StopRefTx(size_t channel) {
  dev_->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
  dev_->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");
}

void Radio::SetDcOffset(int direction, size_t channel,
                        std::complex<double> dc_corr) {
  dev_->setDCOffset(direction, channel, dc_corr);
}

void Radio::SetIQBalance(int direction, size_t channel,
                         std::complex<double> i_qcorr) {
  dev_->setIQBalance(direction, channel, i_qcorr);
}

void Radio::AdjustDelay(const std::string& delay) {
  dev_->writeSetting("ADJUST_DELAYS", delay);
}

void Radio::SetFreqBb(size_t channel, double freq) {
  dev_->setFrequency(SOAPY_SDR_TX, channel, "BB", freq);
  dev_->setFrequency(SOAPY_SDR_RX, channel, "BB", freq);
}

void Radio::SetFreqRf(size_t channel, double freq) {
  dev_->setFrequency(SOAPY_SDR_TX, channel, "RF", freq);
  dev_->setFrequency(SOAPY_SDR_RX, channel, "RF", freq);
}

void Radio::ConfigureTddMode(bool is_ref_radio, size_t beacon_radio_id) {
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
  std::cout << "Radio " << id_ << " Frame 1: " << sched << std::endl;
  tdd_sched.push_back(sched);

  conf["frames"] = tdd_sched;
  std::string conf_string = conf.dump();
  dev_->writeSetting("TDD_CONFIG", conf_string);

  dev_->writeRegisters("BEACON_RAM", 0, cfg_->Beacon());

  size_t ndx = 0;
  for (char const& c : cfg_->Channel()) {
    const bool is_beacon_antenna =
        !cfg_->Beamsweep() && ndx == cfg_->BeaconAnt();
    std::vector<unsigned> beacon_weights(
        cfg_->NumRadios() * cfg_->NumChannels(), is_beacon_antenna ? 1 : 0);
    std::string tx_ram_wgt = "BEACON_RAM_WGT_";
    if (cfg_->Beamsweep()) {
      for (size_t j = 0; j < beacon_weights.size(); j++) {
        beacon_weights.at(j) = CommsLib::Hadamard2(ndx, j);
      }
    }
    dev_->writeRegisters(tx_ram_wgt + c, 0, beacon_weights);
    ++ndx;
  }
  dev_->writeSetting("BEACON_START", std::to_string(beacon_radio_id));
}