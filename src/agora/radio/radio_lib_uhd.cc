/**
 * @file radio_lib_uhd.cc
 * @brief Implementation file for the RadioUHDConfig class.
 */
#include "radio_lib_uhd.h"

#include <thread>

#include "SoapySDR/Formats.h"
#include "SoapySDR/Logger.hpp"
#include "logger.h"

static constexpr bool kPrintCalibrationMats = false;
static constexpr size_t kSoapyMakeMaxAttempts = 3;
static constexpr size_t kHubMissingWaitMs = 100;

// only one BS radio object, since, no emplace_back is needed, and the thread number for BS is also set to be 1
RadioUHDConfig::RadioUHDConfig(Config* cfg, Radio::RadioType radio_type)
    : cfg_(cfg), num_radios_initialized_(0), num_radios_configured_(0) {
  std::map<std::string, std::string> args;
  std::map<std::string, std::string> sargs;
  // load channels
  auto channels = Utils::StrToChannels(cfg_->Channel());

  radio_num_ = cfg_->NumRadios();
  antenna_num_ = cfg_->BsAntNum();
  std::cout << "BS Radio num is " << radio_num_
            << ", Antenna num: " << antenna_num_ << std::endl;

  radios_ = Radio::Create(radio_type);
  std::cout << "radio UHD created here" << std::endl;
  std::vector<std::thread> init_bs_threads;

  for (size_t i = 0; i < radio_num_; i++) {
#if defined(THREADED_INIT)
    init_bs_threads.emplace_back(&RadioUHDConfig::InitBsRadio, this, i);
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
          "RadioUHDConfig: Waiting for radio initialization, %zu of %zu "
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
#if defined(THREADED_INIT)
    config_bs_threads.emplace_back(&RadioUHDConfig::ConfigureBsRadio, this, i);
#else
    ConfigureBsRadio(i);
#endif
  }
  std::cout << "radio UHD configured here" << std::endl;

  num_checks = 0;
  // Block until all radios are configured
  size_t num_radios_config = num_radios_configured_.load();
  while (num_radios_config != radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      AGORA_LOG_WARN(
          "RadioUHDConfig: Waiting for radio initialization, %zu of %zu "
          "ready\n",
          num_radios_config, radio_num_);
      num_checks = 0;
    }
    num_radios_config = num_radios_configured_.load();
  }
  for (auto& join_thread : config_bs_threads) {
    join_thread.join();
  }
  radios_->PrintSettings();
  AGORA_LOG_INFO("RadioUHDConfig init complete!\n");
}

void RadioUHDConfig::InitBsRadio(size_t radio_id) {
  radios_->Init(cfg_, radio_id, cfg_->RadioId().at(radio_id),
                Utils::StrToChannels(cfg_->Channel()), cfg_->HwFramer());
  num_radios_initialized_.fetch_add(1);
}

void RadioUHDConfig::ConfigureBsRadio(size_t radio_id) {
  (void)radio_id;
  std::vector<double> tx_gains;
  tx_gains.emplace_back(cfg_->TxGainA());
  tx_gains.emplace_back(cfg_->TxGainB());

  std::vector<double> rx_gains;
  rx_gains.emplace_back(cfg_->RxGainA());
  rx_gains.emplace_back(cfg_->RxGainB());

  radios_->Setup(tx_gains, rx_gains);
  num_radios_configured_.fetch_add(1);
}

bool RadioUHDConfig::RadioStart() {
  bool good_calib = false;
  AllocBuffer1d(&init_calib_dl_processed_,
                cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float),
                Agora_memory::Alignment_t::kAlign64, 1);
  AllocBuffer1d(&init_calib_ul_processed_,
                cfg_->OfdmDataNum() * cfg_->BfAntNum() * sizeof(arma::cx_float),
                Agora_memory::Alignment_t::kAlign64, 1);
  // initialize init_calib to a matrix of zeros
  for (size_t i = 0; i < (cfg_->OfdmDataNum() * cfg_->BfAntNum()); i++) {
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
      // int iter = 0;
      // int max_iter = 1;
      std::cout << "Start initial reciprocity calibration..." << std::endl;
      // while (good_calib == false) {
      //   good_calib = InitialCalib();
      //   iter++;
      //   if ((iter == max_iter) && (good_calib == false)) {
      //     std::cout << "attempted " << max_iter
      //               << " unsucessful calibration, stopping ..." << std::endl;
      //     break;
      //   }
      // }
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

  //Speed up the activations (could have a flush)
  std::vector<std::thread> activate_radio_threads;
  for (size_t i = 0; i < radio_num_; i++) {
    if (cfg_->HwFramer()) {
      const size_t cell_id = cfg_->CellId().at(i);
      const bool is_ref_radio = (i == cfg_->RefRadio(cell_id));
      radios_->ConfigureTddModeBs(is_ref_radio);
    }
    radios_->SetTimeAtTrigger(0);
    activate_radio_threads.emplace_back(&Radio::Activate, radios_.get(),
                                        Radio::ActivationTypes::kActivate, 0,
                                        0);
  }

  AGORA_LOG_INFO("RadioStart waiting for activation\n");
  for (auto& join_thread : activate_radio_threads) {
    join_thread.join();
  }
  AGORA_LOG_INFO("RadioUHDConfig::RadioUHDStart complete!\n");
  return true;
}

void RadioUHDConfig::Go() {}

int RadioUHDConfig::RadioTx(size_t radio_id, const void* const* buffs,
                            Radio::TxFlags flags, long long& tx_time) {
  (void)radio_id;
  return radios_->Tx(buffs, cfg_->SampsPerSymbol(), flags, tx_time);
}

int RadioUHDConfig::RadioTx(
    size_t radio_id,
    const std::vector<std::vector<std::complex<int16_t>>>& tx_data,
    Radio::TxFlags flags, long long& tx_time)

{
  (void)radio_id;
  std::vector<const void*> buffs(tx_data.size());
  for (size_t i = 0; i < tx_data.size(); i++) {
    buffs.at(i) = tx_data.at(i).data();
  }
  return radios_->Tx(buffs.data(), cfg_->SampsPerSymbol(), flags, tx_time);
}

int RadioUHDConfig::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>>& rx_data,
    size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns) {
  (void)radio_id;
  return radios_->Rx(rx_data, rx_size, out_flags, rx_time_ns);
}

int RadioUHDConfig::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
    size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns) {
  (void)radio_id;
  return radios_->Rx(rx_buffs, rx_size, out_flags, rx_time_ns);
}

int RadioUHDConfig::RadioRx(size_t radio_id, std::vector<void*>& rx_locs,
                            size_t rx_size, Radio::RxFlags& out_flags,
                            long long& rx_time_ns) {
  (void)radio_id;
  return radios_->Rx(rx_locs, rx_size, out_flags, rx_time_ns);
}

void RadioUHDConfig::ReadSensors() { radios_->ReadSensor(); }

void RadioUHDConfig::RadioStop() {
  //Could add a threaded deactivate if it speeds things up.
  std::vector<std::thread> deactivate_radio_threads;
  deactivate_radio_threads.emplace_back(&Radio::Deactivate, radios_.get());

  AGORA_LOG_INFO("RadioStop waiting for deactivation\n");
  for (auto& join_thread : deactivate_radio_threads) {
    join_thread.join();
  }
  AGORA_LOG_INFO("RadioStop deactivated\n");
}

long long RadioUHDConfig::SyncArrayTime() {
  //1ms
  // constexpr long long kTimeSyncMaxLimit = 1000000;
  //Use the trigger to sync the array
  AGORA_LOG_TRACE("SyncArrayTime: Setting trigger time\n");
  radios_->SetTimeAtTrigger();
  AGORA_LOG_TRACE("SyncArrayTime: Triggering!\n");
  Go();

  ///Wait for enough time for the boards to update
  auto wait_time = std::chrono::milliseconds(500);
  AGORA_LOG_TRACE("SyncArrayTime: Waiting for %ld ms\n", wait_time.count());
  std::this_thread::sleep_for(wait_time);
  AGORA_LOG_TRACE("SyncArrayTime: Time Check!\n");

  //Get first time
  auto radio_time = radios_->GetTimeNs();
  //Verify all times are within 1ms (typical .35ms - .85 between calls) - since only one object in UHD setting, this is being deleted
  // for (size_t i = 1; i < radios_.size(); i++) {
  //   auto time_now = radios_->GetTimeNs();
  //   auto time_diff_ns = time_now - radio_time;
  //   if (time_diff_ns > kTimeSyncMaxLimit) {
  //     AGORA_LOG_WARN(
  //         "SyncArrayTime: Radio UHD time out of bounds during alignment.  Radio UHD "
  //         "%zu, time %lld, reference %lld, difference %lld, tolerance %lld\n",
  //         i, time_now, radio_time, time_diff_ns, kTimeSyncMaxLimit);
  //   }
  //   //Update the check time with the current time
  //   radio_time = time_now;
  // }
  return radio_time;
}

RadioUHDConfig::~RadioUHDConfig() {
  FreeBuffer1d(&init_calib_dl_processed_);
  FreeBuffer1d(&init_calib_ul_processed_);

  std::vector<std::thread> close_radio_threads;
  close_radio_threads.emplace_back(&Radio::Close, radios_.get());

  AGORA_LOG_INFO("~RadioUHDConfig waiting for close\n");
  for (auto& join_thread : close_radio_threads) {
    join_thread.join();
  }
  AGORA_LOG_INFO("RadioStop destructed\n");
}
