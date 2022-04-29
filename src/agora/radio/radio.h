/** @file Radio.h
  * @brief Declaration file for the Radio class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#ifndef RADIO_H_
#define RADIO_H_

#include <string>

#include "SoapySDR/Device.hpp"
#include "config.h"

class Radio {
 public:
  Radio();
  ~Radio();
  inline size_t Id() const { return id_; }
  inline const std::string& SerialNumber() const { return serial_number_; }

  void Init(const Config* cfg, size_t id, const std::string& serial,
            const std::vector<size_t>& enabled_channels);
  void Setup(const std::vector<double>& tx_gains,
             const std::vector<double>& rx_gains);
  void Activate();    // Start?
  void Deactivate();  // Stop?
  void Close();

  int Tx(const void* const* tx_buffs, size_t tx_size, int tx_flags,
         long long& tx_time_ns);
  int Rx(void** rx_buffs, long long& rx_time_ns);
  int Rx(void** rx_buffs, size_t rx_size, int rx_flags, long long& rx_time_ns);
  std::vector<std::complex<float>> SnoopSamples(size_t channel,
                                                size_t read_size);

  void ConfigureTddModeBs(bool is_ref_radio, size_t beacon_radio_id);
  void ConfigureTddModeUe();

  // Calibration helper functions
  void InitRefTx(size_t channel, double freq);
  void InitCalRx(size_t channel, double center_freq);
  void ResetTxGains();
  void ResetRxGains();
  void SetTxCalGain(size_t channel);
  void SetRxGain(size_t channel, const std::string& gain_stage, double value);
  void SetTxGain(size_t channel, const std::string& gain_stage, double value);
  void StartRefTx(size_t channel);
  void StopRefTx(size_t channel);
  void SetDcOffset(int direction, size_t channel, std::complex<double> dc_corr);
  void SetIQBalance(int direction, size_t channel,
                    std::complex<double> i_qcorr);
  void AdjustDelay(const std::string& delay);
  //Sets both tx and rx, is this ok?
  void SetFreqBb(size_t channel, double freq);
  void SetFreqRf(size_t channel, double freq);
  //End Calibration Routines
  void InitAgc(bool enabled, size_t setting);

  void ClearSyncDelay();
  void PrintSettings() const;
  void Trigger();
  void ReadSensor() const;

 private:
  void InitCorr();

  size_t id_;
  std::string serial_number_;
  std::vector<size_t> enabled_channels_;
  SoapySDR::Device* dev_;
  SoapySDR::Stream* rxs_;
  SoapySDR::Stream* txs_;
  //Should remove this.
  const Config* cfg_;
  bool correlator_enabled_;
};

#endif  // RADIO_H_