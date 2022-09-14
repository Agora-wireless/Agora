/** @file radio.h
  * @brief Declaration file for the Radio class.
  * 
  * This class defines functions specific to different radio implementations 
*/
#ifndef RADIO_H_
#define RADIO_H_

#include <memory>
#include <string>
#include <vector>

#include "config.h"

class Radio {
 public:
  enum RadioType { kSoapySdrStream, kSoapySdrSocket };
  //EndReceive is set when the samples returned are the last of a contiguous set
  enum RxFlags { kRxFlagNone = 0, kEndReceive = 1 };
  enum TxFlags { kTxFlagNone = 0, kEndTransmit = 1, kTxWaitTrigger = 2 };
  static std::unique_ptr<Radio> Create(RadioType type);

  enum ActivationTypes { kActivate, kActivateWaitTrigger };
  virtual ~Radio();

  //Accessors
  inline size_t Id() const { return id_; }
  inline const std::string& SerialNumber() const { return serial_number_; }
  inline bool HwFramer() const { return hw_framer_; }

  virtual void Init(const Config* cfg, size_t id, const std::string& serial,
                    const std::vector<size_t>& enabled_channels,
                    bool hw_framer) = 0;
  virtual void Setup(const std::vector<double>& tx_gains,
                     const std::vector<double>& rx_gains) = 0;

  // Start?
  virtual void Activate(
      Radio::ActivationTypes type = Radio::ActivationTypes::kActivate,
      long long act_time_ns = 0, size_t samples = 0) = 0;
  // Stop?
  virtual void Deactivate() = 0;
  virtual void Close() = 0;
  virtual void Flush() = 0;

  virtual int Tx(const void* const* tx_buffs, size_t tx_size, TxFlags tx_flags,
                 long long& tx_time_ns) = 0;

  virtual int Rx(std::vector<std::vector<std::complex<int16_t>>>& rx_data,
                 size_t rx_size, RxFlags& out_flags, long long& rx_time_ns) = 0;

  virtual int Rx(std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs,
                 size_t rx_size, RxFlags& out_flags, long long& rx_time_ns) = 0;

  virtual int Rx(std::vector<void*>& rx_locs, size_t rx_size,
                 RxFlags& out_flags, long long& rx_time_ns) = 0;

  inline virtual void ConfigureTddModeBs([[maybe_unused]] bool is_ref_radio) {}
  inline virtual void ConfigureTddModeUe() {}
  inline virtual void ClearSyncDelay() {}
  inline virtual void PrintSettings() const {}
  inline virtual void Trigger() {}
  inline virtual void ReadSensor() const {}

  inline virtual void SetTimeAtTrigger(long long time_ns = 0) { (void)time_ns; }
  inline virtual long long GetTimeNs() { return 0; }

  //For digital cal
  inline virtual void AdjustDelay([[maybe_unused]] const std::string& delay) {}

  inline const std::vector<size_t>& EnabledChannels() const {
    return enabled_channels_;
  }

 protected:
  Radio();
  //Should remove this.
  const Config* cfg_{nullptr};

 private:
  size_t id_{0};
  std::string serial_number_;
  std::vector<size_t> enabled_channels_;
  bool hw_framer_{false};
};

#endif  // RADIO_H_