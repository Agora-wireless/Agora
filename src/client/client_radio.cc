/**
 * @file client_radio.cc
 * @brief Implementation file for the client radio config class
 */

#include "client_radio.h"

#include "logger.h"

ClientRadioConfig::ClientRadioConfig(const Config* const cfg,
                                     Radio::RadioType radio_type)
    : cfg_(cfg) {
  total_radios_ = cfg_->UeNum();
  total_antennas_ = cfg_->UeAntNum();
  std::cout << "Total Number of Client Radios " << total_radios_ << " with "
            << total_antennas_ << " antennas" << std::endl;

  for (size_t i = 0; i < total_radios_; i++) {
    radios_.emplace_back(Radio::Create(radio_type));
  }

  std::vector<std::thread> radio_threads;
  num_client_radios_initialized_ = 0;
  for (size_t i = 0; i < total_radios_; i++) {
#ifdef THREADED_INIT
    radio_threads.emplace_back(&ClientRadioConfig::InitClientRadio, this, i);
#else
    InitClientRadio(i);
#endif
  }  // end for (size_t i = 0; i < total_radios_; i++)

#ifdef THREADED_INIT
  size_t num_checks = 0;

  size_t num_client_radios_init = num_client_radios_initialized_.load();
  while (num_client_radios_init != total_radios_) {
    num_checks++;
    if (num_checks > 1e9) {
      AGORA_LOG_INFO(
          "ClientRadioConfig: Waiting for radio initialization, %zu of %zu "
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
  AGORA_LOG_INFO("ClientRadioConfig: Radio init complete\n");
}

void ClientRadioConfig::InitClientRadio(size_t radio_id) {
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

bool ClientRadioConfig::RadioStart() {
  // send through the first radio for now
  for (size_t i = 0; i < total_radios_; i++) {
    if (cfg_->UeHwFramer()) {
      radios_.at(i)->ConfigureTddModeUe();
    }
    radios_.at(i)->Activate(Radio::ActivationTypes::kActivateWaitTrigger);
  }
  AGORA_LOG_INFO("ClientRadioConfig: Radio start complete!\n");
  return true;
}

void ClientRadioConfig::Go() const {
  if (kUseUHD == false) {
    for (size_t i = 0; i < total_radios_; i++) {
      radios_.at(i)->Trigger();
    }
  }
}

int ClientRadioConfig::RadioTx(size_t radio_id, void** buffs, size_t num_samps,
                               Radio::TxFlags flags, long long& tx_time) {
  return radios_.at(radio_id)->Tx(buffs, num_samps, flags, tx_time);
}

int ClientRadioConfig::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>>& rx_data,
    size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns) {
  return radios_.at(radio_id)->Rx(rx_data, rx_size, out_flags, rx_time_ns);
}

int ClientRadioConfig::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
    size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns) {
  return radios_.at(radio_id)->Rx(rx_buffs, rx_size, out_flags, rx_time_ns);
}

int ClientRadioConfig::RadioRx(size_t radio_id, std::vector<void*>& rx_locs,
                               size_t rx_size, Radio::RxFlags& out_flags,
                               long long& rx_time_ns) {
  return radios_.at(radio_id)->Rx(rx_locs, rx_size, out_flags, rx_time_ns);
}

void ClientRadioConfig::ReadSensors() {
  for (const auto& radio : radios_) {
    radio->ReadSensor();
  }
}

void ClientRadioConfig::RadioStop() {
  for (auto& radio : radios_) {
    radio->Deactivate();
  }
}

ClientRadioConfig::~ClientRadioConfig() {
  for (auto& radio : radios_) {
    radio->Close();
  }
}
