/**
 * @file radio_lib.cc
 * @brief Implementation file for the RadioConfig class.
 */
#include "radio_lib.h"

#include <SoapySDR/Logger.hpp>

#include "comms-lib.h"
#include "nlohmann/json.hpp"

static constexpr bool kPrintCalibrationMats = false;
static constexpr bool kPrintRadioSettings = false;

static constexpr size_t kSoapyMakeMaxAttempts = 3;
static constexpr size_t kHubMissingWaitMs = 100;

RadioConfig::RadioConfig(Config* cfg)
    : cfg_(cfg), num_radios_initialized_(0), num_radios_configured_(0) {
  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  // load channels
  auto channels = Utils::StrToChannels(cfg_->Channel());
  ///Reduce the soapy log level
  SoapySDR::setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_NOTICE);

  this->radio_num_ = cfg_->NumRadios();
  this->antenna_num_ = cfg_->BsAntNum();
  std::cout << "BS Radio num is " << this->radio_num_
            << ", Antenna num: " << antenna_num_ << std::endl;
  if (kUseUHD == false) {
    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      SoapySDR::Device* hub_device = nullptr;
      if (!cfg_->HubId().at(i).empty()) {
        args["driver"] = "remote";
        args["timeout"] = "1000000";
        args["serial"] = cfg_->HubId().at(i);

        for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
          try {
            hub_device = SoapySDR::Device::make(args);
            break;
          } catch (const std::runtime_error& e) {
            const auto* message = e.what();
            std::printf("RadioConfig::Soapy error[%zu] -- %s\n", tries,
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
  }

  ba_stn_.resize(radio_num_);
  tx_streams_.resize(radio_num_);
  rx_streams_.resize(radio_num_);

  std::vector<std::thread> init_bs_threads;

  for (size_t i = 0; i < this->radio_num_; i++) {
#ifdef THREADED_INIT
    init_bs_threads.emplace_back(&RadioConfig::InitBsRadio, this, i);
#else
    InitBsRadio(i);
#endif
  }

  // Block until all radios are initialized
  size_t num_checks = 0;
  size_t num_radios_init = num_radios_initialized_.load();
  while (num_radios_init != this->radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      std::printf(
          "RadioConfig: Waiting for radio initialization, %zu of %zu ready\n",
          num_radios_init, this->radio_num_);
      num_checks = 0;
    }
    num_radios_init = num_radios_initialized_.load();
  }

  for (auto& join_thread : init_bs_threads) {
    join_thread.join();
  }

  // Perform DC Offset & IQ Imbalance Calibration
  if (cfg_->ImbalanceCalEn()) {
    if (cfg_->Channel().find('A') != std::string::npos) {
      DciqCalibrationProc(0);
    }
    if (cfg_->Channel().find('B') != std::string::npos) {
      DciqCalibrationProc(1);
    }
  }

  std::vector<std::thread> config_bs_threads;
  for (size_t i = 0; i < this->radio_num_; i++) {
#ifdef THREADED_INIT
    config_bs_threads.emplace_back(&RadioConfig::ConfigureBsRadio, this, i);
#else
    ConfigureBsRadio(i);
#endif
  }

  num_checks = 0;
  // Block until all radios are configured
  size_t num_radios_config = num_radios_configured_.load();
  while (num_radios_config != this->radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      std::printf(
          "RadioConfig: Waiting for radio initialization, %zu of %zu ready\n",
          num_radios_config, this->radio_num_);
      num_checks = 0;
    }
    num_radios_config = num_radios_configured_.load();
  }

  for (auto& join_thread : config_bs_threads) {
    join_thread.join();
  }

  for (size_t i = 0; i < this->radio_num_; i++) {
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
          if (!kUseUHD) {
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
          if (!kUseUHD) {
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
  if (kUseUHD == false) {
    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      if (hubs_.at(i) == nullptr) {
        ba_stn_.at(i)->writeSetting("SYNC_DELAYS", "");
      } else {
        hubs_[i]->writeSetting("SYNC_DELAYS", "");
      }
    }
  }
  std::cout << "radio init done!" << std::endl;
}

void RadioConfig::InitBsRadio(size_t tid) {
  size_t i = tid;
  auto channels = Utils::StrToChannels(cfg_->Channel());
  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  args["timeout"] = "1000000";
  if (!kUseUHD) {
    args["driver"] = "iris";
    args["serial"] = cfg_->RadioId().at(i);
  } else {
    args["driver"] = "uhd";
    args["addr"] = cfg_->RadioId().at(i);
  }

  SoapySDR::Device* bs_device = nullptr;
  for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
    try {
      bs_device = SoapySDR::Device::make(args);
      break;
    } catch (const std::runtime_error& e) {
      const auto* message = e.what();
      std::printf("InitBsRadio[%zu] - Soapy error try %zu -- %s\n", tid, tries,
                  message);
    }
  }
  if (bs_device == nullptr) {
    std::printf("SoapySDR failed to locate the Bs radio %s in %zu attempts\n",
                cfg_->RadioId().at(tid).c_str(), kSoapyMakeMaxAttempts);
    throw std::runtime_error("SoapySDR failed to locate the Bs radio");
  }
  ba_stn_.at(i) = bs_device;

  // resets the DATA_clk domain logic.
  ba_stn_.at(i)->writeSetting("RESET_DATA_LOGIC", "");

  rx_streams_.at(i) =
      ba_stn_.at(i)->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
  tx_streams_.at(i) =
      ba_stn_.at(i)->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
  this->num_radios_initialized_.fetch_add(1);
}

void RadioConfig::ConfigureBsRadio(size_t tid) {
  // load channels
  auto channels = Utils::StrToChannels(cfg_->Channel());
  for (auto ch : channels) {
    ba_stn_.at(tid)->setSampleRate(SOAPY_SDR_RX, ch, cfg_->Rate());
    ba_stn_.at(tid)->setSampleRate(SOAPY_SDR_TX, ch, cfg_->Rate());
  }

  // use the TRX antenna port for both tx and rx
  for (auto ch : channels) {
    if (kUseUHD == false) {
      ba_stn_.at(tid)->setAntenna(SOAPY_SDR_RX, ch, "TRX");
    } else {
      ba_stn_.at(tid)->setAntenna(SOAPY_SDR_RX, ch, "RX2");
      ba_stn_.at(tid)->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
    }
  }

  SoapySDR::Kwargs info = ba_stn_.at(tid)->getHardwareInfo();
  for (auto ch : channels) {
    if (kUseUHD == false) {
      ba_stn_.at(tid)->setBandwidth(SOAPY_SDR_RX, ch, cfg_->BwFilter());
      ba_stn_.at(tid)->setBandwidth(SOAPY_SDR_TX, ch, cfg_->BwFilter());
    }

    // ba_stn_.at(tid)->setSampleRate(SOAPY_SDR_RX, ch, cfg->Rate());
    // ba_stn_.at(tid)->setSampleRate(SOAPY_SDR_TX, ch, cfg->Rate());

    ba_stn_.at(tid)->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg_->RadioRfFreq());
    ba_stn_.at(tid)->setFrequency(SOAPY_SDR_RX, ch, "BB",
                                  kUseUHD ? 0 : cfg_->Nco());
    ba_stn_.at(tid)->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg_->RadioRfFreq());
    ba_stn_.at(tid)->setFrequency(SOAPY_SDR_TX, ch, "BB",
                                  kUseUHD ? 0 : cfg_->Nco());

    if (kUseUHD == false) {
      // Unified gains for both lime and frontend
      if (cfg_->SingleGain()) {
        // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
        ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch,
                                 ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());
        // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
        ba_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch,
                                 ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());
      } else {
        if (info["frontend"].find("CBRS") != std::string::npos) {
          if (cfg_->Freq() > 3e9) {
            ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "ATTN", -6);  //[-18,0]
          } else if ((cfg_->Freq() > 2e9) && (cfg_->Freq() < 3e9)) {
            ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "ATTN", -18);  //[-18,0]
          } else {
            ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "ATTN", -12);  //[-18,0]
          }
          ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "LNA2", 17);  //[0,17]
        }

        ba_stn_.at(tid)->setGain(
            SOAPY_SDR_RX, ch, "LNA",
            ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());     //[0,30]
        ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "TIA", 0);  //[0,12]
        ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "PGA", 0);  //[-12,19]

        if (info["frontend"].find("CBRS") != std::string::npos) {
          ba_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "ATTN",
                                   -6);                          //[-18,0] by 3
          ba_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "PA2", 0);  //[0|15]
        }
        ba_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);  //[-12,12]
        ba_stn_.at(tid)->setGain(
            SOAPY_SDR_TX, ch, "PAD",
            ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());  //[0,30]
      }
    } else {
      ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "PGA0",
                               ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());
      ba_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "PGA0",
                               ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());
    }
  }

  for (auto ch : channels) {
    ba_stn_.at(tid)->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
  }
  this->num_radios_configured_.fetch_add(1);
}

bool RadioConfig::RadioStart() {
  if (cfg_->SampleCalEn()) {
    CalibrateSampleOffset();
  }
  bool good_calib = false;
  AllocBuffer1d(&init_calib_dl_processed_,
                cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float),
                Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&init_calib_ul_processed_,
                cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float),
                Agora_memory::Alignment_t::kAlign64, 1);
  // initialize init_calib to a matrix of zeros
  for (size_t i = 0; i < cfg_->OfdmDataNum() * cfg_->BfAntNum(); i++) {
    init_calib_dl_processed_[i] = 0;
    init_calib_ul_processed_[i] = 0;
  }

  calib_meas_num_ = cfg_->InitCalibRepeat();
  if (calib_meas_num_ != 0u) {
    init_calib_ul_.Calloc(calib_meas_num_,
                          cfg_->OfdmDataNum() * cfg_->BfAntNum(),
                          Agora_memory::Alignment_t::kAlign64);
    init_calib_dl_.Calloc(calib_meas_num_,
                          cfg_->OfdmDataNum() * cfg_->BfAntNum(),
                          Agora_memory::Alignment_t::kAlign64);
    if (cfg_->Frame().NumDLSyms() > 0) {
      int iter = 0;
      int max_iter = 1;
      std::cout << "Start initial reciprocity calibration..." << std::endl;
      while (good_calib == false) {
        good_calib = InitialCalib();
        iter++;
        if ((iter == max_iter) && (good_calib == false)) {
          std::cout << "attempted " << max_iter
                    << " unsucessful calibration, stopping ..." << std::endl;
          break;
        }
      }
      if (good_calib == false) {
        return good_calib;
      } else {
        std::cout << "initial calibration successful!" << std::endl;
      }
      // process initial measurements
      arma::cx_fcube calib_dl_cube(cfg_->OfdmDataNum(), cfg_->BfAntNum(),
                                   calib_meas_num_, arma::fill::zeros);
      arma::cx_fcube calib_ul_cube(cfg_->OfdmDataNum(), cfg_->BfAntNum(),
                                   calib_meas_num_, arma::fill::zeros);
      for (size_t i = 0; i < calib_meas_num_; i++) {
        arma::cx_fmat calib_dl_mat(init_calib_dl_[i], cfg_->OfdmDataNum(),
                                   cfg_->BfAntNum(), false);
        arma::cx_fmat calib_ul_mat(init_calib_ul_[i], cfg_->OfdmDataNum(),
                                   cfg_->BfAntNum(), false);
        calib_dl_cube.slice(i) = calib_dl_mat;
        calib_ul_cube.slice(i) = calib_ul_mat;
        if (kPrintCalibrationMats) {
          Utils::PrintMat(calib_dl_mat, "calib_dl_mat" + std::to_string(i));
          Utils::PrintMat(calib_ul_mat, "calib_ul_mat" + std::to_string(i));
          Utils::PrintMat(calib_dl_mat / calib_ul_mat,
                          "calib_mat" + std::to_string(i));
        }
        if (kRecordCalibrationMats == true) {
          Utils::SaveMat(calib_dl_mat, "calib_dl_mat.m",
                         "init_calib_dl_mat" + std::to_string(i),
                         i > 0 /*append*/);
          Utils::SaveMat(calib_ul_mat, "calib_ul_mat.m",
                         "init_calib_ul_mat" + std::to_string(i),
                         i > 0 /*append*/);
        }
      }
      arma::cx_fmat calib_dl_mean_mat(init_calib_dl_processed_,
                                      cfg_->OfdmDataNum(), cfg_->BfAntNum(),
                                      false);
      arma::cx_fmat calib_ul_mean_mat(init_calib_ul_processed_,
                                      cfg_->OfdmDataNum(), cfg_->BfAntNum(),
                                      false);
      calib_dl_mean_mat = arma::mean(calib_dl_cube, 2);  // mean along dim 2
      calib_ul_mean_mat = arma::mean(calib_ul_cube, 2);  // mean along dim 2
      if (kPrintCalibrationMats) {
        Utils::PrintMat(calib_dl_mean_mat, "calib_dl_mat");
        Utils::PrintMat(calib_ul_mean_mat, "calib_ul_mat");
        Utils::PrintMat(calib_dl_mean_mat / calib_ul_mean_mat, "calib_mat");
      }
      if (kRecordCalibrationMats == true) {
        Utils::SaveMat(calib_dl_mean_mat, "calib_dl_mat.m",
                       "init_calib_dl_mat_mean", true /*append*/);
        Utils::SaveMat(calib_ul_mean_mat, "calib_ul_mat.m",
                       "init_calib_ul_mat_mean", true /*append*/);
      }
    }
    init_calib_dl_.Free();
    init_calib_ul_.Free();
  }

  DrainBuffers();
  nlohmann::json conf;
  conf["tdd_enabled"] = true;
  conf["frame_mode"] = "free_running";
  conf["max_frame"] = 0;
  conf["symbol_size"] = cfg_->SampsPerSymbol();
  conf["beacon_start"] = cfg_->OfdmTxZeroPrefix();
  conf["beacon_stop"] = cfg_->OfdmTxZeroPrefix() + cfg_->BeaconLen();

  size_t ndx = 0;
  for (size_t i = 0; i < this->radio_num_; i++) {
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

    if (!kUseUHD) {
      ba_stn_.at(i)->setHardwareTime(0, "TRIGGER");
      if (cfg_->HwFramer() == true) {
        ba_stn_.at(i)->activateStream(this->rx_streams_.at(i));
        ba_stn_.at(i)->activateStream(this->tx_streams_.at(i));
      }
    } else {
      ba_stn_.at(i)->setHardwareTime(0, "UNKNOWN_PPS");
      ba_stn_.at(i)->activateStream(this->rx_streams_.at(i), SOAPY_SDR_HAS_TIME,
                                    1e9, 0);
      ba_stn_.at(i)->activateStream(this->tx_streams_.at(i), SOAPY_SDR_HAS_TIME,
                                    1e9, 0);
    }
  }

  //Stream is already activated?
  if (!kUseUHD && cfg_->HwFramer() == false) {
    this->Go();  // to set all radio timestamps to zero
    int flags = SOAPY_SDR_HAS_TIME;
    for (size_t i = 0; i < this->radio_num_; i++) {
      ba_stn_.at(i)->activateStream(this->rx_streams_.at(i), flags, 1e9, 0);
      ba_stn_.at(i)->activateStream(this->tx_streams_.at(i), flags, 1e9, 0);
    }
  }
  std::cout << "radio start done!" << std::endl;
  return true;
}

void RadioConfig::Go() {
  // TODO: For multi-cell trigger process needs modification
  if (kUseUHD == false) {
    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      if (hubs_.at(i) == nullptr) {
        ba_stn_.at(i)->writeSetting("TRIGGER_GEN", "");
      } else {
        hubs_[i]->writeSetting("TRIGGER_GEN", "");
      }
    }
  }
}

void RadioConfig::RadioTx(void** buffs) {
  static constexpr size_t kTxTimeoutUs = 1000000;
  int flags = 0;
  long long frame_time(0);
  for (size_t i = 0; i < this->radio_num_; i++) {
    ba_stn_.at(i)->writeStream(this->tx_streams_.at(i), buffs,
                               cfg_->SampsPerSymbol(), flags, frame_time,
                               kTxTimeoutUs);
  }
}

int RadioConfig::RadioTx(size_t radio_id, const void* const* buffs, int flags,
                         long long& frameTime) {
  static constexpr size_t kTxTimeoutUs = 1000000;
  int tx_flags = 0;
  if (flags == 1) {
    tx_flags = SOAPY_SDR_HAS_TIME;
  } else if (flags == 2) {
    tx_flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
  }
  // long long frameTime(0);

  int w;
  if (cfg_->HwFramer() == true) {
    w = ba_stn_.at(radio_id)->writeStream(tx_streams_.at(radio_id), buffs,
                                          cfg_->SampsPerSymbol(), tx_flags,
                                          frameTime, kTxTimeoutUs);
  } else {
    // For UHD device xmit from host using frameTimeNs
    long long frame_time_ns = SoapySDR::ticksToTimeNs(frameTime, cfg_->Rate());
    w = ba_stn_.at(radio_id)->writeStream(tx_streams_.at(radio_id), buffs,
                                          cfg_->SampsPerSymbol(), tx_flags,
                                          frame_time_ns, kTxTimeoutUs);
  }
  if (kDebugRadioTX) {
    size_t chan_mask;
    long timeout_us(0);
    int status_flag = 0;
    int s = ba_stn_.at(radio_id)->readStreamStatus(tx_streams_.at(radio_id),
                                                   chan_mask, status_flag,
                                                   frameTime, timeout_us);
    std::cout << "radio " << radio_id << " tx returned " << w << " and status "
              << s << " when flags was " << flags << std::endl;
  }
  return w;
}

int RadioConfig::RadioTx(
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

void RadioConfig::RadioRx(void** buffs) {
  static constexpr size_t kTxTimeoutUs = 1000000;
  long long frame_time(0);
  int rx_flags = SOAPY_SDR_END_BURST;
  for (size_t i = 0; i < this->radio_num_; i++) {
    void** buff = buffs + (i * 2);
    ba_stn_.at(i)->readStream(this->rx_streams_.at(i), buff,
                              cfg_->SampsPerSymbol(), rx_flags, frame_time,
                              kTxTimeoutUs);
  }
}

int RadioConfig::RadioRx(size_t radio_id, void** buffs, long long& rx_time_ns) {
  int rx_status = 0;
  // SOAPY_SDR_ONE_PACKET; SOAPY_SDR_END_BURST
  int rx_flags = SOAPY_SDR_END_BURST;
  constexpr long kRxTimeout = 1;  // 1uS
  //constexpr long kRxTimeout = 1000000;  // 1S

  if (radio_id < radio_num_) {
    long long frame_time_ns = 0;
    rx_status = ba_stn_.at(radio_id)->readStream(
        rx_streams_.at(radio_id), buffs, cfg_->SampsPerSymbol(), rx_flags,
        frame_time_ns, kRxTimeout);

    if (cfg_->HwFramer() == true) {
      rx_time_ns = frame_time_ns;
    } else {
      // for UHD device recv using ticks
      rx_time_ns = SoapySDR::timeNsToTicks(frame_time_ns, cfg_->Rate());
    }

    if (kDebugRadioRX) {
      if (rx_status == static_cast<int>(cfg_->SampsPerSymbol())) {
        std::cout << "Radio " << radio_id << " received " << rx_status
                  << " flags: " << rx_flags << " MTU "
                  << ba_stn_.at(radio_id)->getStreamMTU(
                         rx_streams_.at(radio_id))
                  << std::endl;
      } else {
        if (!((rx_status == SOAPY_SDR_TIMEOUT) && (rx_flags == 0))) {
          std::cout << "Unexpected RadioRx return value " << rx_status
                    << " from radio " << radio_id << " flags: " << rx_flags
                    << std::endl;
        }
      }
    }
  } else {
    std::cout << "Invalid radio id " << radio_id << std::endl;
  }

  /// If a timeout occurs tell the user you received 0 bytes
  if (rx_status == SOAPY_SDR_TIMEOUT) {
    rx_status = 0;
  }
  return rx_status;
}

int RadioConfig::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>>& rx_data,
    long long& rx_time_ns) {
  std::vector<void*> buffs(rx_data.size());
  for (size_t i = 0u; i < rx_data.size(); i++) {
    buffs.at(i) = rx_data.at(i).data();
  }
  return RadioRx(radio_id, buffs.data(), rx_time_ns);
}

void RadioConfig::DrainBuffers() {
  std::vector<std::vector<std::complex<int16_t>>> sample_storage(
      cfg_->NumChannels(),
      std::vector<std::complex<int16_t>>(cfg_->SampsPerSymbol(),
                                         std::complex<int16_t>(0, 0)));
  std::vector<void*> rx_buffs;
  rx_buffs.reserve(sample_storage.size());
  for (auto& buff : sample_storage) {
    rx_buffs.push_back(buff.data());
  }

  for (size_t i = 0; i < cfg_->NumRadios(); i++) {
    RadioConfig::DrainRxBuffer(ba_stn_.at(i), rx_streams_.at(i), rx_buffs,
                               cfg_->SampsPerSymbol());
  }
}

void RadioConfig::DrainRxBuffer(SoapySDR::Device* ibsSdrs,
                                SoapySDR::Stream* istream,
                                std::vector<void*> buffs, size_t symSamp) {
  long long frame_time = 0;
  int flags = 0;
  int r = 0;
  int i = 0;
  long timeout_us(0);
  while (r != -1) {
    r = ibsSdrs->readStream(istream, buffs.data(), symSamp, flags, frame_time,
                            timeout_us);
    i++;
  }
  //std::cout << "Number of reads needed to drain: " << i << std::endl;
}

void RadioConfig::ReadSensors() {
  for (size_t i = 0; i < this->radio_num_; i++) {
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

void RadioConfig::RadioStop() {
  std::vector<uint32_t> zeros(4096, 0);
  std::string corr_conf_str = "{\"corr_enabled\":false}";
  std::string tdd_conf_str = "{\"tdd_enabled\":false}";
  for (size_t i = 0; i < this->radio_num_; i++) {
    ba_stn_.at(i)->deactivateStream(this->rx_streams_.at(i));
    ba_stn_.at(i)->deactivateStream(this->tx_streams_.at(i));
    ba_stn_.at(i)->writeSetting("TDD_MODE", "false");
    ba_stn_.at(i)->writeSetting("TDD_CONFIG", tdd_conf_str);
  }
}

RadioConfig::~RadioConfig() {
  FreeBuffer1d(&init_calib_dl_processed_);
  FreeBuffer1d(&init_calib_ul_processed_);

  if ((radio_num_ != ba_stn_.size()) || (rx_streams_.size() != radio_num_) ||
      (tx_streams_.size() != radio_num_)) {
    std::printf(
        "**************************** BAD NEWS ****************************");
    std::printf("radio_num_ != the size of the radio array!");
  }

  for (size_t i = 0; i < radio_num_; i++) {
    ba_stn_.at(i)->closeStream(rx_streams_.at(i));
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
