/**
 * @file radio_lib.cc
 * @brief Implementation file for the RadioConfig class.
 */
#include "radio_lib.h"

#include "SoapySDR/Formats.h"
#include "SoapySDR/Logger.hpp"
#include "logger.h"

constexpr bool kPrintCalibrationMats = false;

constexpr size_t kSoapyMakeMaxAttempts = 3;
constexpr size_t kHubMissingWaitMs = 100;

RadioConfig::RadioConfig(Config* cfg, Radio::RadioType radio_type)
    : cfg_(cfg), num_radios_initialized_(0), num_radios_configured_(0) {
  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  // load channels
  auto channels = Utils::StrToChannels(cfg_->Channel());

  radio_num_ = cfg_->NumRadios();
  antenna_num_ = cfg_->BsAntNum();
  std::cout << "BS Radio num is " << radio_num_
            << ", Antenna num: " << antenna_num_ << std::endl;
  if (kUseUHD == false) {
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

  for (size_t i = 0; i < radio_num_; i++) {
    radios_.emplace_back(Radio::Create(radio_type));
  }

  std::vector<std::thread> init_bs_threads;

  for (size_t i = 0; i < radio_num_; i++) {
#ifdef THREADED_INIT
    init_bs_threads.emplace_back(&RadioConfig::InitBsRadio, this, i);
#else
    InitBsRadio(i);
#endif
  }

  // Block until all radios are initialized
  size_t num_checks = 0;
  size_t num_radios_init = num_radios_initialized_.load();
  while (num_radios_init != radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      std::printf(
          "RadioConfig: Waiting for radio initialization, %zu of %zu ready\n",
          num_radios_init, radio_num_);
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
  for (size_t i = 0; i < radio_num_; i++) {
#ifdef THREADED_INIT
    config_bs_threads.emplace_back(&RadioConfig::ConfigureBsRadio, this, i);
#else
    ConfigureBsRadio(i);
#endif
  }

  num_checks = 0;
  // Block until all radios are configured
  size_t num_radios_config = num_radios_configured_.load();
  while (num_radios_config != radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      AGORA_LOG_WARN(
          "RadioConfig: Waiting for radio initialization, %zu of %zu ready\n",
          num_radios_config, radio_num_);
      num_checks = 0;
    }
    num_radios_config = num_radios_configured_.load();
  }

  for (auto& join_thread : config_bs_threads) {
    join_thread.join();
  }

  for (const auto& radio : radios_) {
    radio->PrintSettings();
  }

  // TODO: For multi-cell, this procedure needs modification
  if (kUseUHD == false) {
    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      if (hubs_.at(i) == nullptr) {
        radios_.at(i)->ClearSyncDelay();
      } else {
        hubs_.at(i)->writeSetting("SYNC_DELAYS", "");
      }
    }
  }
  AGORA_LOG_INFO("RadioConfig init complete!\n");
}

void RadioConfig::InitBsRadio(size_t radio_id) {
  radios_.at(radio_id)->Init(cfg_, radio_id, cfg_->RadioId().at(radio_id),
                             Utils::StrToChannels(cfg_->Channel()));
  num_radios_initialized_.fetch_add(1);
}

void RadioConfig::ConfigureBsRadio(size_t radio_id) {
  std::vector<double> tx_gains;
  tx_gains.emplace_back(cfg_->TxGainA());
  tx_gains.emplace_back(cfg_->TxGainB());

  std::vector<double> rx_gains;
  rx_gains.emplace_back(cfg_->RxGainA());
  rx_gains.emplace_back(cfg_->RxGainB());

  radios_.at(radio_id)->Setup(tx_gains, rx_gains);
  num_radios_configured_.fetch_add(1);
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

  const size_t beacon_radio = radio_num_;
  for (size_t i = 0; i < radio_num_; i++) {
    if (cfg_->HwFramer()) {
      const size_t cell_id = cfg_->CellId().at(i);
      const bool is_ref_radio = (i == cfg_->RefRadio(cell_id));
      radios_.at(i)->ConfigureTddModeBs(is_ref_radio, beacon_radio);
    }
    radios_.at(i)->Activate();
  }
  AGORA_LOG_INFO("RadioConfig::RadioStart complete!\n");
  return true;
}

void RadioConfig::Go() {
  // TODO: For multi-cell trigger process needs modification
  if (kUseUHD == false) {
    for (size_t i = 0; i < cfg_->NumCells(); i++) {
      if (hubs_.at(i) == nullptr) {
        radios_.at(i)->Trigger();
      } else {
        hubs_.at(i)->writeSetting("TRIGGER_GEN", "");
      }
    }
  }
}

int RadioConfig::RadioTx(size_t radio_id, const void* const* buffs, int flags,
                         long long& tx_time) {
  return radios_.at(radio_id)->Tx(buffs, cfg_->SampsPerSymbol(), flags,
                                  tx_time);
}

int RadioConfig::RadioTx(
    size_t radio_id,
    const std::vector<std::vector<std::complex<int16_t>>>& tx_data, int flags,
    long long& tx_time)

{
  std::vector<const void*> buffs(tx_data.size());
  for (size_t i = 0; i < tx_data.size(); i++) {
    buffs.at(i) = tx_data.at(i).data();
  }
  return radios_.at(radio_id)->Tx(buffs.data(), cfg_->SampsPerSymbol(), flags,
                                  tx_time);
}

int RadioConfig::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>>& rx_data,
    size_t rx_size, int rx_flags, long long& rx_time_ns) {
  return radios_.at(radio_id)->Rx(rx_data, rx_size, rx_flags, rx_time_ns);
}

int RadioConfig::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
    size_t rx_size, int rx_flags, long long& rx_time_ns) {
  return radios_.at(radio_id)->Rx(rx_buffs, rx_size, rx_flags, rx_time_ns);
}

int RadioConfig::RadioRx(size_t radio_id, std::vector<void*>& rx_locs,
                         size_t rx_size, int rx_flags, long long& rx_time_ns) {
  return radios_.at(radio_id)->Rx(rx_locs, rx_size, rx_flags, rx_time_ns);
}

void RadioConfig::ReadSensors() {
  for (const auto& radio : radios_) {
    radio->ReadSensor();
  }
}

void RadioConfig::RadioStop() {
  for (auto& radio : radios_) {
    radio->Deactivate();
  }
}

RadioConfig::~RadioConfig() {
  FreeBuffer1d(&init_calib_dl_processed_);
  FreeBuffer1d(&init_calib_ul_processed_);

  for (auto& radio : radios_) {
    radio->Close();
  }

  for (auto* hub : hubs_) {
    SoapySDR::Device::unmake(hub);
  }
  hubs_.clear();
}
