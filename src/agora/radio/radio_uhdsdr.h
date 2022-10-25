/** @file radio_soapysdr.h
  * @brief Declaration file for the RadioUHDSdr class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#ifndef RADIO_UHDSDR_H_
#define RADIO_UHDSDR_H_

#include <memory>

#include "config.h"
#include "radio.h"
#include "uhd/usrp/multi_usrp.hpp"

class RadioUHDSdr : public Radio {
 public:
  explicit RadioUHDSdr();  // NO need for DataPlaneType for UHD Radios,
  virtual ~RadioUHDSdr() final;

  virtual void Init(const Config* cfg, size_t id, const std::string& serial,
                    const std::vector<size_t>& enabled_channels,
                    bool hw_framer, bool isUE) final;
  virtual void Setup(const std::vector<double>& tx_gains,
                     const std::vector<double>& rx_gains) final;

  // change to more generic version of Radio::ActivateTypes, if needed, add new act types
  virtual void Activate(
      Radio::ActivationTypes type = Radio::ActivationTypes::kActivate,
      long long act_time_ns = 0, size_t samples = 0) final;
  virtual void Deactivate() final;
  virtual void Close() final;
  virtual void Flush() final;

  // change to Radio::TxFlags, if new flags are needed, add them to Radio
  virtual int Tx(const void* const* tx_buffs, size_t tx_size,
                 Radio::TxFlags tx_flags, long long& tx_time_ns) final;

  virtual int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
                 size_t rx_size, Radio::RxFlags& out_flags,
                 long long& rx_time_ns) final;

  virtual int Rx(std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
                 size_t rx_size, Radio::RxFlags& out_flags,
                 long long& rx_time_ns) final;

  virtual int Rx(std::vector<void*>& rx_locs, size_t rx_size,
                 Radio::RxFlags& out_flags, long long& rx_time_ns) final;

  virtual void SetTimeAtTrigger(long long time_ns = 0) final;
  virtual long long GetTimeNs() final;
  //End of generic interface

  virtual void ConfigureTddModeBs(bool is_ref_radio) final;
  virtual void ConfigureTddModeUe() final;
  virtual void ClearSyncDelay() final;
  virtual void PrintSettings() const final;
  virtual void Trigger() final;
  virtual void ReadSensor() const final;

 private:
  uhd::usrp::multi_usrp::sptr dev_;
  uhd::rx_streamer::sptr rxs_;  // init in RadioUHDSdr::Init()
  uhd::tx_streamer::sptr txs_;
};

#endif  // RADIO_UHDSDR_H_
