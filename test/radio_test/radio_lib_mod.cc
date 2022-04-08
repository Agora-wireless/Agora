/**
 * @file radio_lib_mod.cc
 * @brief Implementation file for the RadioConfigNoRxStream class.
 */
#include "radio_lib_mod.h"

#include "SoapySDR/Formats.hpp"
#include "SoapySDR/Logger.hpp"
#include "comms-lib.h"
#include "nlohmann/json.hpp"
#include "radio_data_plane_soapy.h"

static constexpr bool kPrintCalibrationMats = false;
static constexpr bool kPrintRadioSettings = false;

static constexpr size_t kSoapyMakeMaxAttempts = 3;
static constexpr size_t kHubMissingWaitMs = 100;

RadioConfigNoRxStream::RadioConfigNoRxStream(Config* cfg)
    : cfg_(cfg), num_radios_initialized_(0), num_radios_configured_(0) {
  SoapySDR::Kwargs args;
  // load channels
  auto channels = Utils::StrToChannels(cfg_->Channel());
  ///Reduce the soapy log level
  SoapySDR::setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_SSI);

  radio_num_ = cfg_->NumRadios();
  antenna_num_ = cfg_->BsAntNum();

  std::cout << "BS Radio num is " << radio_num_
            << ", Antenna num: " << antenna_num_ << std::endl;

  //Build hub devices
  for (size_t i = 0; i < cfg_->NumCells(); i++) {
    SoapySDR::Device* hub_device = nullptr;
    if (!cfg_->HubId().at(i).empty()) {
      args["driver"] = "remote";
      args["timeout"] = "100000";
      args["serial"] = cfg_->HubId().at(i);
      args["remote:type"] = "faros";
      args["remote:driver"] = "faros";
      args["remote:mtu"] = "1500";
      args["remote:ipver"] = "6";
      args["remote:prot"] = "udp";

      for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
        try {
          hub_device = SoapySDR::Device::make(args);
          break;
        } catch (const std::runtime_error& e) {
          const auto* message = e.what();
          std::printf("RadioConfigNoRxStream::Soapy error[%zu] -- %s\n", tries,
                      message);
          std::this_thread::sleep_for(
              std::chrono::milliseconds(kHubMissingWaitMs));
        }
      }
      if (hub_device == nullptr) {
        std::printf(
            "SoapySDR failed to locate the hub device %s in %zu tries\n",
            cfg_->HubId().at(i).c_str(), kSoapyMakeMaxAttempts);
        throw std::runtime_error("SoapySDR failed to locate the hub device");
      }
    }
    hubs_.push_back(hub_device);
  }

  ba_stn_.resize(radio_num_);
  tx_streams_.resize(radio_num_);
  std::vector<std::thread> init_bs_threads;

  for (size_t i = 0; i < radio_num_; i++) {
    //init_bs_threads.emplace_back(&RadioConfigNoRxStream::InitBsRadio, this, i);
    InitBsRadio(i);
  }

  // Block until all radios are initialized
  size_t num_checks = 0;
  size_t num_radios_init = num_radios_initialized_.load();
  while (num_radios_init != radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      std::printf(
          "RadioConfigNoRxStream: Waiting for radio initialization, %zu of %zu "
          "ready\n",
          num_radios_init, radio_num_);
      num_checks = 0;
    }
    num_radios_init = num_radios_initialized_.load();
  }

  for (auto& join_thread : init_bs_threads) {
    join_thread.join();
  }

  std::vector<std::thread> config_bs_threads;
  for (size_t i = 0; i < radio_num_; i++) {
    //config_bs_threads.emplace_back(&RadioConfigNoRxStream::ConfigureBsRadio,
    //                               this, i);
    ConfigureBsRadio(i);
  }

  num_checks = 0;
  // Block until all radios are configured
  size_t num_radios_config = num_radios_configured_.load();
  while (num_radios_config != radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      std::printf(
          "RadioConfigNoRxStream: Waiting for radio initialization, %zu of %zu "
          "ready\n",
          num_radios_config, radio_num_);
      num_checks = 0;
    }
    num_radios_config = num_radios_configured_.load();
  }

  for (auto& join_thread : config_bs_threads) {
    join_thread.join();
  }

  for (size_t i = 0; i < radio_num_; i++) {
    std::cout << cfg_->RadioId().at(i) << ": Front end "
              << ba_stn_.at(i)->getHardwareInfo()["frontend"] << std::endl;

    if (kPrintRadioSettings) {
      for (auto c : channels) {
        if (c < ba_stn_.at(i)->getNumChannels(SOAPY_SDR_RX)) {
          std::printf("RX Channel %zu\n", c);
          std::printf("Actual RX sample rate: %fMSps...\n",
                      (ba_stn_.at(i)->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
          std::printf("Actual RX frequency: %fGHz...\n",
                      (ba_stn_.at(i)->getFrequency(SOAPY_SDR_RX, c) / 1e9));
          std::printf("Actual RX gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c)));

          std::printf("Actual RX LNA gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "LNA")));
          std::printf("Actual RX PGA gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "PGA")));
          std::printf("Actual RX TIA gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "TIA")));
          if (ba_stn_.at(i)->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            std::printf("Actual RX LNA1 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "LNA1")));
            std::printf("Actual RX LNA2 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "LNA2")));
          }
          std::printf("Actual RX bandwidth: %fM...\n",
                      (ba_stn_.at(i)->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
          std::printf("Actual RX antenna: %s...\n",
                      (ba_stn_.at(i)->getAntenna(SOAPY_SDR_RX, c).c_str()));
        }
      }

      for (auto c : channels) {
        if (c < ba_stn_.at(i)->getNumChannels(SOAPY_SDR_TX)) {
          std::printf("TX Channel %zu\n", c);
          std::printf("Actual TX sample rate: %fMSps...\n",
                      (ba_stn_.at(i)->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
          std::printf("Actual TX frequency: %fGHz...\n",
                      (ba_stn_.at(i)->getFrequency(SOAPY_SDR_TX, c) / 1e9));
          std::printf("Actual TX gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c)));

          std::printf("Actual TX PAD gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PAD")));
          std::printf("Actual TX IAMP gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "IAMP")));
          if (ba_stn_.at(i)->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            std::printf("Actual TX PA1 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PA1")));
            std::printf("Actual TX PA2 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PA2")));
            std::printf("Actual TX PA3 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PA3")));
          }
          std::printf("Actual TX bandwidth: %fM...\n",
                      (ba_stn_.at(i)->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
          std::printf("Actual TX antenna: %s...\n",
                      (ba_stn_.at(i)->getAntenna(SOAPY_SDR_TX, c).c_str()));
        }
      }
      std::cout << std::endl;
    }
  }

  // TODO: For multi-cell, this procedure needs modification
  for (size_t i = 0; i < cfg_->NumCells(); i++) {
    if (hubs_.at(i) == nullptr) {
      ba_stn_.at(i)->writeSetting("SYNC_DELAYS", "");
    } else {
      hubs_[i]->writeSetting("SYNC_DELAYS", "");
    }
  }
  std::cout << "radio init done!" << std::endl;
}

RadioConfigNoRxStream::~RadioConfigNoRxStream() {
  if (radio_num_ != ba_stn_.size()) {
    std::printf(
        "**************************** BAD NEWS ****************************");
    std::printf("radio_num_ != the size of the radio array!");
  }

  for (size_t i = 0; i < radio_num_; i++) {
    rx_data_plane_.at(i)->Close();
    ba_stn_.at(i)->closeStream(tx_streams_.at(i));
    SoapySDR::Device::unmake(ba_stn_.at(i));
    ba_stn_.at(i) = nullptr;
  }
  ba_stn_.clear();

  for (auto* hub : hubs_) {
    SoapySDR::Device::unmake(hub);
  }
  hubs_.clear();
}

void RadioConfigNoRxStream::InitBsRadio(size_t radio) {
  auto channels = Utils::StrToChannels(cfg_->Channel());
  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  args["timeout"] = "1000000";
  args["driver"] = "iris";
  args["serial"] = cfg_->RadioId().at(radio);

  SoapySDR::Device* bs_device = nullptr;
  for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
    try {
      bs_device = SoapySDR::Device::make(args);
      break;
    } catch (const std::runtime_error& e) {
      const auto* message = e.what();
      std::printf("InitBsRadio[%zu] - Soapy error try %zu -- %s\n", radio,
                  tries, message);
    }
  }
  if (bs_device == nullptr) {
    std::printf("SoapySDR failed to locate the Bs radio %s in %zu attempts\n",
                cfg_->RadioId().at(radio).c_str(), kSoapyMakeMaxAttempts);
    throw std::runtime_error("SoapySDR failed to locate the Bs radio");
  }
  ba_stn_.at(radio) = bs_device;
  for (auto ch : {0, 1}) {
    ba_stn_.at(radio)->setSampleRate(SOAPY_SDR_RX, ch, cfg_->Rate());
    ba_stn_.at(radio)->setSampleRate(SOAPY_SDR_TX, ch, cfg_->Rate());
  }

  // resets the DATA_clk domain logic.
  ba_stn_.at(radio)->writeSetting("RESET_DATA_LOGIC", "");
  auto& dp = rx_data_plane_.emplace_back(
      std::make_unique<RadioDataPlaneSoapy>(cfg_, ba_stn_.at(radio), radio));
  dp->Setup();
  tx_streams_.at(radio) = ba_stn_.at(radio)->setupStream(
      SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
  num_radios_initialized_.fetch_add(1);
}

void RadioConfigNoRxStream::ConfigureBsRadio(size_t radio) {
  // load channels
  auto channels = Utils::StrToChannels(cfg_->Channel());

  // use the TRX antenna port for both tx and rx
  for (auto ch : channels) {
    ba_stn_.at(radio)->setAntenna(SOAPY_SDR_RX, ch, "TRX");
  }

  SoapySDR::Kwargs info = ba_stn_.at(radio)->getHardwareInfo();
  for (auto ch : channels) {
    ba_stn_.at(radio)->setBandwidth(SOAPY_SDR_RX, ch, cfg_->BwFilter());
    ba_stn_.at(radio)->setBandwidth(SOAPY_SDR_TX, ch, cfg_->BwFilter());

    // ba_stn_.at(radio)->setSampleRate(SOAPY_SDR_RX, ch, cfg->Rate());
    // ba_stn_.at(radio)->setSampleRate(SOAPY_SDR_TX, ch, cfg->Rate());

    ba_stn_.at(radio)->setFrequency(SOAPY_SDR_RX, ch, "RF",
                                    cfg_->RadioRfFreq());
    ba_stn_.at(radio)->setFrequency(SOAPY_SDR_RX, ch, "BB",
                                    kUseUHD ? 0 : cfg_->Nco());
    ba_stn_.at(radio)->setFrequency(SOAPY_SDR_TX, ch, "RF",
                                    cfg_->RadioRfFreq());
    ba_stn_.at(radio)->setFrequency(SOAPY_SDR_TX, ch, "BB",
                                    kUseUHD ? 0 : cfg_->Nco());

    // Unified gains for both lime and frontend
    if (cfg_->SingleGain()) {
      // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      ba_stn_.at(radio)->setGain(SOAPY_SDR_RX, ch,
                                 ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());
      // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
      ba_stn_.at(radio)->setGain(SOAPY_SDR_TX, ch,
                                 ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());
    } else {
      if (info["frontend"].find("CBRS") != std::string::npos) {
        if (cfg_->Freq() > 3e9) {
          ba_stn_.at(radio)->setGain(SOAPY_SDR_RX, ch, "ATTN", -6);  //[-18,0]
        } else if ((cfg_->Freq() > 2e9) && (cfg_->Freq() < 3e9)) {
          ba_stn_.at(radio)->setGain(SOAPY_SDR_RX, ch, "ATTN", -18);  //[-18,0]
        } else {
          ba_stn_.at(radio)->setGain(SOAPY_SDR_RX, ch, "ATTN", -12);  //[-18,0]
        }
        ba_stn_.at(radio)->setGain(SOAPY_SDR_RX, ch, "LNA2", 17);  //[0,17]
      }

      ba_stn_.at(radio)->setGain(
          SOAPY_SDR_RX, ch, "LNA",
          ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());       //[0,30]
      ba_stn_.at(radio)->setGain(SOAPY_SDR_RX, ch, "TIA", 0);  //[0,12]
      ba_stn_.at(radio)->setGain(SOAPY_SDR_RX, ch, "PGA", 0);  //[-12,19]

      if (info["frontend"].find("CBRS") != std::string::npos) {
        ba_stn_.at(radio)->setGain(SOAPY_SDR_TX, ch, "ATTN",
                                   -6);                          //[-18,0] by 3
        ba_stn_.at(radio)->setGain(SOAPY_SDR_TX, ch, "PA2", 0);  //[0|15]
      }
      ba_stn_.at(radio)->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);  //[-12,12]
      ba_stn_.at(radio)->setGain(
          SOAPY_SDR_TX, ch, "PAD",
          ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());  //[0,30]
    }
  }

  for (auto ch : channels) {
    ba_stn_.at(radio)->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
  }
  num_radios_configured_.fetch_add(1);
}

bool RadioConfigNoRxStream::RadioStart() {
  DrainBuffers();
  nlohmann::json conf;
  conf["tdd_enabled"] = true;
  conf["frame_mode"] = "free_running";
  conf["max_frame"] = 0;
  conf["symbol_size"] = cfg_->SampsPerSymbol();
  conf["beacon_start"] = cfg_->OfdmTxZeroPrefix();
  conf["beacon_stop"] = cfg_->OfdmTxZeroPrefix() + cfg_->BeaconLen();

  size_t ndx = 0;
  for (size_t i = 0; i < radio_num_; i++) {
    size_t cell_id = cfg_->CellId().at(i);
    bool is_ref_radio = (i == cfg_->RefRadio(cell_id));
    if (cfg_->HwFramer() == true) {
      ba_stn_.at(i)->writeSetting(
          "TX_SW_DELAY",
          "30");  // experimentally good value for dev front-end
      ba_stn_.at(i)->writeSetting("TDD_MODE", "true");
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
      std::cout << "Radio " << i << " Frame 1: " << sched << std::endl;
      tdd_sched.push_back(sched);

      conf["frames"] = tdd_sched;
      std::string conf_string = conf.dump();
      ba_stn_.at(i)->writeSetting("TDD_CONFIG", conf_string);

      ba_stn_.at(i)->writeRegisters("BEACON_RAM", 0, cfg_->Beacon());
      for (char const& c : cfg_->Channel()) {
        bool is_beacon_antenna = !cfg_->Beamsweep() && ndx == cfg_->BeaconAnt();
        std::vector<unsigned> beacon_weights(
            cfg_->NumRadios() * cfg_->NumChannels(), is_beacon_antenna ? 1 : 0);
        std::string tx_ram_wgt = "BEACON_RAM_WGT_";
        if (cfg_->Beamsweep()) {
          for (size_t j = 0; j < beacon_weights.size(); j++) {
            beacon_weights.at(j) = CommsLib::Hadamard2(ndx, j);
          }
        }
        ba_stn_.at(i)->writeRegisters(tx_ram_wgt + c, 0, beacon_weights);
        ++ndx;
      }
      ba_stn_.at(i)->writeSetting("BEACON_START", std::to_string(radio_num_));
    }
    ba_stn_.at(i)->setHardwareTime(0, "TRIGGER");
    rx_data_plane_.at(i)->Activate();
    ba_stn_.at(i)->activateStream(tx_streams_.at(i));
  }
  std::cout << "radio start done!" << std::endl;
  return true;
}

void RadioConfigNoRxStream::Go() {
  // TODO: For multi-cell trigger process needs modification
  for (size_t i = 0; i < cfg_->NumCells(); i++) {
    if (hubs_.at(i) == nullptr) {
      ba_stn_.at(i)->writeSetting("TRIGGER_GEN", "");
    } else {
      hubs_.at(i)->writeSetting("TRIGGER_GEN", "");
    }
  }
}

int RadioConfigNoRxStream::RadioTx(size_t radio_id, const void* const* buffs,
                                   int flags, long long& frameTime) {
  return 0;
}

int RadioConfigNoRxStream::RadioTx(
    size_t radio_id,
    const std::vector<std::vector<std::complex<int16_t>>>& tx_data, int flags,
    long long& frameTime)

{
  std::vector<const void*> buffs(tx_data.size());
  for (size_t i = 0; i < tx_data.size(); i++) {
    buffs.at(i) = tx_data.at(i).data();
  }
  return RadioTx(radio_id, buffs.data(), flags, frameTime);
}

int RadioConfigNoRxStream::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>>& rx_data,
    long long& rx_time_ns) {
  // For now, radio rx will return 1 symbol
  int rx_return = 0;
  rx_return = rx_data_plane_.at(radio_id)->Rx(rx_data, rx_time_ns);
  if (rx_return > 0) {
    std::printf("Rx'd sample count %d\n", rx_return);
  } else if (rx_return < 0) {
    throw std::runtime_error("Error in RadioRx!");
  }
  return rx_return;
}

void RadioConfigNoRxStream::DrainBuffers() {
  std::vector<std::vector<std::complex<int16_t>>> sample_storage(
      cfg_->NumChannels(),
      std::vector<std::complex<int16_t>>(cfg_->SampsPerSymbol(),
                                         std::complex<int16_t>(0, 0)));
}

void RadioConfigNoRxStream::ReadSensors() {
  for (size_t i = 0; i < radio_num_; i++) {
    std::cout << "TEMPs on Iris " << i << std::endl;
    std::cout << "ZYNQ_TEMP: " << ba_stn_.at(i)->readSensor("ZYNQ_TEMP")
              << std::endl;
    std::cout << "LMS7_TEMP  : " << ba_stn_.at(i)->readSensor("LMS7_TEMP")
              << std::endl;
    std::cout << "FE_TEMP  : " << ba_stn_.at(i)->readSensor("FE_TEMP")
              << std::endl;
    std::cout << "TX0 TEMP  : "
              << ba_stn_.at(i)->readSensor(SOAPY_SDR_TX, 0, "TEMP")
              << std::endl;
    std::cout << "TX1 TEMP  : "
              << ba_stn_.at(i)->readSensor(SOAPY_SDR_TX, 1, "TEMP")
              << std::endl;
    std::cout << "RX0 TEMP  : "
              << ba_stn_.at(i)->readSensor(SOAPY_SDR_RX, 0, "TEMP")
              << std::endl;
    std::cout << "RX1 TEMP  : "
              << ba_stn_.at(i)->readSensor(SOAPY_SDR_RX, 1, "TEMP")
              << std::endl;
    std::cout << std::endl;
  }
}

void RadioConfigNoRxStream::RadioStop() {
  std::vector<uint32_t> zeros(4096, 0);
  std::string corr_conf_str = "{\"corr_enabled\":false}";
  std::string tdd_conf_str = "{\"tdd_enabled\":false}";
  for (size_t i = 0; i < radio_num_; i++) {
    rx_data_plane_.at(i)->Deactivate();
    ba_stn_.at(i)->deactivateStream(tx_streams_.at(i));
    ba_stn_.at(i)->writeSetting("TDD_MODE", "false");
    ba_stn_.at(i)->writeSetting("TDD_CONFIG", tdd_conf_str);
  }
}
