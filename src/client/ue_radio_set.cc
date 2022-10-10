/**
 * @file ue_radio_set.cc
 * @brief Implementation file for the UeRadioSet class
 */

#include "ue_radio_set.h"

#include "logger.h"

UeRadioSet::UeRadioSet(const Config* const cfg, Radio::RadioType radio_type)
    : RadioSet(cfg->SampsPerSymbol()), cfg_(cfg) {
  if (radio_type == Radio::RadioType::kUhdNative) {
    total_radios_ = 1;
  } else {
    total_radios_ = cfg_->UeNum();
  }
  total_antennas_ = cfg_->UeAntNum();
  std::cout << "Total Number of Client Radios " << total_radios_ << " with "
            << total_antennas_ << " antennas" << std::endl;

  for (size_t i = 0; i < total_radios_; i++) {
    radios_.emplace_back(Radio::Create(radio_type));
  }

  std::vector<std::thread> radio_threads;
  num_client_radios_initialized_ = 0;
  for (size_t i = 0; i < total_radios_; i++) {
#if defined(THREADED_INIT)
    radio_threads.emplace_back(&UeRadioSet::InitRadio, this, i);
#else
    InitRadio(i);
#endif
  }  // end for (size_t i = 0; i < total_radios_; i++)

#if defined(THREADED_INIT)
  size_t num_checks = 0;
  size_t num_client_radios_init = num_client_radios_initialized_.load();
  while (num_client_radios_init != total_radios_) {
    num_checks++;
    if (num_checks > 1e9) {
      AGORA_LOG_INFO(
          "UeRadioSet: Waiting for radio initialization, %zu of %zu "
          "ready\n",
          num_client_radios_init, total_radios_);
      num_checks = 0;
    }
    num_client_radios_init = num_client_radios_initialized_.load();
  }

  for (auto& init_thread : radio_threads) {
    init_thread.join();
  }
#endif

  for (const auto& radio : radios_) {
    radio->PrintSettings();
  }
  AGORA_LOG_INFO("UeRadioSet: Radio init complete\n");
}

void UeRadioSet::InitRadio(size_t radio_id) {
  radios_.at(radio_id)->Init(cfg_, radio_id, cfg_->UeRadioId().at(radio_id),
                             Utils::StrToChannels(cfg_->UeChannel()),
                             cfg_->UeHwFramer());

  std::vector<double> tx_gains;
  tx_gains.emplace_back(cfg_->ClientTxGainA(radio_id));
  tx_gains.emplace_back(cfg_->ClientTxGainB(radio_id));

  std::vector<double> rx_gains;
  rx_gains.emplace_back(cfg_->ClientRxGainA(radio_id));
  rx_gains.emplace_back(cfg_->ClientRxGainB(radio_id));

  radios_.at(radio_id)->Setup(tx_gains, rx_gains);
  this->num_client_radios_initialized_.fetch_add(1);
}

bool UeRadioSet::RadioStart() {
  // send through the first radio for now
  for (size_t i = 0; i < total_radios_; i++) {
    if (cfg_->UeHwFramer()) {
      radios_.at(i)->ConfigureTddModeUe();
    }
  }
  RadioSet::RadioStart(Radio::kActivateWaitTrigger);
  AGORA_LOG_INFO("UeRadioSet: Radio start complete!\n");
  return true;
}

void UeRadioSet::Go() {
  if ((kUseUHD == false) || (kUsePureUHD == false)) {
    for (size_t i = 0; i < total_radios_; i++) {
      radios_.at(i)->Trigger();
    }
  }
}