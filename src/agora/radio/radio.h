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
  //inline SoapySDR::Device* RawDev() const { return dev_; };
  inline size_t Id() const { return id_; }

  void Init(const Config* cfg, size_t id);
  void Setup();
  void Activate();    // Start?
  void Deactivate();  // Stop?
  void Close();

  int Tx(const void* const* tx_buffs, size_t tx_size, int tx_flags,
         long long& tx_time_ns);
  int Rx(void** rx_buffs, long long& rx_time_ns);
  int Rx(void** rx_buffs, size_t rx_size, int rx_flags, long long& rx_time_ns);
  std::vector<std::complex<float>> SnoopSamples(size_t channel,
                                                size_t read_size);

  void ConfigureTddMode(bool is_ref_radio, size_t beacon_radio_id);

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

  void ClearSyncDelay();
  void PrintSettings() const;
  void Trigger();
  void ReadSensor() const;

 private:
  size_t id_;
  size_t num_channels_;
  SoapySDR::Device* dev_;
  SoapySDR::Stream* rxs_;
  SoapySDR::Stream* txs_;
  //Should remove this.
  const Config* cfg_;
};

#endif  // RADIO_H_