/** @file radio_soapysdr.h
  * @brief Declaration file for the RadioSoapySdr class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#ifndef RADIO_SOAPYSDR_H_
#define RADIO_SOAPYSDR_H_

#include "SoapySDR/Device.hpp"
#include "config.h"
#include "radio.h"

class RadioSoapySdr : public Radio {
 public:
  RadioSoapySdr();
  ~RadioSoapySdr() final;

  void Init(const Config* cfg, size_t id, const std::string& serial,
            const std::vector<size_t>& enabled_channels) final;
  void Setup(const std::vector<double>& tx_gains,
             const std::vector<double>& rx_gains) final;
  void Activate() final;
  void Deactivate() final;
  void Close() final;

  int Tx(const void* const* tx_buffs, size_t tx_size, int tx_flags,
         long long& tx_time_ns) final;
  int Rx(void** rx_buffs, long long& rx_time_ns) final;
  int Rx(void** rx_buffs, size_t rx_size, int rx_flags,
         long long& rx_time_ns) final;
  //End of generic interface

  void ConfigureTddModeBs(bool is_ref_radio, size_t beacon_radio_id) final;
  void ConfigureTddModeUe() final;
  void ClearSyncDelay() final;
  void PrintSettings() const final;
  void Trigger() final;
  void ReadSensor() const final;
  void AdjustDelay(const std::string& delay) final;

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
  //Sets both tx and rx, is this ok?
  void SetFreqBb(size_t channel, double freq);
  void SetFreqRf(size_t channel, double freq);
  //End Calibration Routines
  void InitAgc(bool enabled, size_t setting);

  std::vector<std::complex<float>> SnoopSamples(size_t channel,
                                                size_t read_size);

 private:
  void Correlator(bool enable);

  SoapySDR::Device* dev_;
  SoapySDR::Stream* rxs_;
  SoapySDR::Stream* txs_;
  bool correlator_enabled_;
};

#endif  // RADIO_SOAPYSDR_H_