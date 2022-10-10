/**
 * @file radio_set.cc
 * @brief Implementation file for the RadioSet class.
 */

#include "radio_set.h"

#include <thread>
#include <vector>

#include "logger.h"

RadioSet::RadioSet(size_t samples_per_symbol)
    : samples_per_symbol_(samples_per_symbol) {}

RadioSet::~RadioSet() {
  for (auto& radio : radios_) {
    radio->Close();
  }
}

int RadioSet::RadioTx(size_t radio_id, const void* const* buffs,
                      size_t num_samps, Radio::TxFlags flags,
                      long long& tx_time) {
  return radios_.at(radio_id)->Tx(buffs, num_samps, flags, tx_time);
}

int RadioSet::RadioTx(size_t radio_id, const void* const* buffs,
                      Radio::TxFlags flags, long long& tx_time) {
  return radios_.at(radio_id)->Tx(buffs, samples_per_symbol_, flags, tx_time);
}

int RadioSet::RadioTx(
    size_t radio_id,
    const std::vector<std::vector<std::complex<int16_t>>>& tx_data,
    Radio::TxFlags flags, long long& tx_time)

{
  std::vector<const void*> buffs(tx_data.size());
  for (size_t i = 0; i < tx_data.size(); i++) {
    buffs.at(i) = tx_data.at(i).data();
  }
  return radios_.at(radio_id)->Tx(buffs.data(), samples_per_symbol_, flags,
                                  tx_time);
}

int RadioSet::RadioRx(size_t radio_id,
                      std::vector<std::vector<std::complex<int16_t>>>& rx_data,
                      size_t rx_size, Radio::RxFlags& out_flags,
                      long long& rx_time_ns) {
  return radios_.at(radio_id)->Rx(rx_data, rx_size, out_flags, rx_time_ns);
}

int RadioSet::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
    size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns) {
  return radios_.at(radio_id)->Rx(rx_buffs, rx_size, out_flags, rx_time_ns);
}

int RadioSet::RadioRx(size_t radio_id, std::vector<void*>& rx_locs,
                      size_t rx_size, Radio::RxFlags& out_flags,
                      long long& rx_time_ns) {
  return radios_.at(radio_id)->Rx(rx_locs, rx_size, out_flags, rx_time_ns);
}

void RadioSet::ReadSensors() {
  for (const auto& radio : radios_) {
    radio->ReadSensor();
  }
}

void RadioSet::RadioStop() {
  //Threaded deactivate to speed things up
  std::vector<std::thread> deactivate_radio_threads;
  for (auto& radio : radios_) {
    deactivate_radio_threads.emplace_back(&Radio::Deactivate, radio.get());
  }

  AGORA_LOG_INFO("RadioStop waiting for deactivation\n");
  for (auto& join_thread : deactivate_radio_threads) {
    join_thread.join();
  }
  AGORA_LOG_INFO("RadioStop deactivated\n");
}