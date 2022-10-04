/** @file radio_soapysdr.h
  * @brief Declaration file for the RadioUHDSdr class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#ifndef RADIO_UHDSDR_H_
#define RADIO_UHDSDR_H_

#include <memory>

#include "SoapySDR/Device.hpp"
#include "config.h"
#include "radio.h"
#include "uhd/usrp/multi_usrp.hpp"

class RadioUHDSdr : public Radio {
 public:
  explicit RadioUHDSdr();  // NO need for DataPlaneType for UHD Radios,
  ~RadioUHDSdr() final;

  void Init(const Config* cfg, size_t id, const std::string& serial,
            const std::vector<size_t>& enabled_channels, bool hw_framer) final;
  void Setup(const std::vector<double>& tx_gains,
             const std::vector<double>& rx_gains) final;

  // change to more generic version of Radio::ActivateTypes, if needed, add new act types
  void Activate(Radio::ActivationTypes type = Radio::ActivationTypes::kActivate,
                long long act_time_ns = 0, size_t samples = 0);
  void Deactivate() final;
  void Close() final;
  void Flush() final;

  // change to Radio::TxFlags, if new flags are needed, add them to Radio
  int Tx(const void* const* tx_buffs, size_t tx_size, Radio::TxFlags tx_flags,
         long long& tx_time_ns);

  int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
         size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns);

  int Rx(std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
         size_t rx_size, Radio::RxFlags& out_flags, long long& rx_time_ns);

  int Rx(std::vector<void*>& rx_locs, size_t rx_size, Radio::RxFlags& out_flags,
         long long& rx_time_ns);

  void SetTimeAtTrigger(long long time_ns = 0) final;
  long long GetTimeNs() final;
  //End of generic interface

  void ConfigureTddModeBs(bool is_ref_radio);
  void ConfigureTddModeUe() final;
  void ClearSyncDelay() final;
  void PrintSettings() const final;
  void Trigger() final;
  void ReadSensor() const final;
  // void AdjustDelay(const std::string& delay) {};

  //   // Calibration helper functions -

  //   //Sets both tx and rx, is this ok?
  //   void SetFreqBb(size_t channel, double freq);
  //   //End Calibration Routines
  //   std::vector<std::complex<float>> SnoopSamples(size_t channel,
  //                                                 size_t read_size);

  inline uhd::usrp::multi_usrp::sptr UHDDevice() const {
    return dev_;
  }  // TODO: check which higher layer functions needs this?
  inline const std::string& IpAddress() const { return ip_address_; }

 private:
  //   void Correlator(bool enable);

  uhd::usrp::multi_usrp::sptr dev_;
  std::string ip_address_;

  //   std::unique_ptr<RadioDataPlaneUHD> rxp_;

  uhd::rx_streamer::sptr rxs_;  // init in RadioUHDSdr::Init()
  uhd::tx_streamer::sptr txs_;
  //   bool correlator_enabled_;
};

#endif  // RADIO_UHDSDR_H_
