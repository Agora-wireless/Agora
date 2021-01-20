#ifndef CLIENT_RADIO_LIB
#define CLIENT_RADIO_LIB

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Time.hpp>
#include <chrono>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

#include "config.hpp"

class ClientRadioConfig {
 public:
  ClientRadioConfig(Config* cfg);
  bool radioStart();
  void radioStop();
  void readSensors();
  int radioTx(size_t, void** buffs, size_t num_samps, int flags,
              long long& frameTime);
  int radioRx(size_t, void** buffs, size_t num_samps, long long& frameTime);
  static void drain_rx_buffer(SoapySDR::Device* dev, SoapySDR::Stream* istream,
                              std::vector<void*> buffs, size_t symSamp);
  void drain_buffers();
  void go();
  int triggers(int i);
  // static void dciqMinimize(SoapySDR::Device*, SoapySDR::Device*, int,
  // size_t, double, double); static void setIQBalance(SoapySDR::Device*, int,
  // size_t, int, int); static void
  // adjustCalibrationGains(std::vector<SoapySDR::Device*>, SoapySDR::Device*,
  // size_t, double, bool plot = false); static
  // std::vector<std::complex<float>> snoopSamples(SoapySDR::Device*, size_t,
  // size_t); void dciqCalibrationProc(size_t);
  ~ClientRadioConfig();

 private:
  struct ClientRadioConfigContext {
    ClientRadioConfig* ptr;
    size_t tid;
  };
  static void* initClientRadio_launch(void* context);
  void initClientRadio(ClientRadioConfigContext* context);

  Config* _cfg;
  std::vector<SoapySDR::Device*> hubs;
  std::vector<SoapySDR::Device*> clStn;
  SoapySDR::Device* ref;
  SoapySDR::Stream* refRxStream;
  std::vector<SoapySDR::Stream*> txStreams;
  std::vector<SoapySDR::Stream*> rxStreams;
  size_t _radioNum;
  size_t _antennaNum;
  ClientRadioConfigContext* context;
};
#endif
