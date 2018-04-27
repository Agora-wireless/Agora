#include <iostream>
#include <SoapySDR/Device.hpp>
#include <SoapySDR/Formats.hpp>
#include <SoapySDR/Errors.hpp>
#include <SoapySDR/Time.hpp>
#include <cstdlib>
#include <cstddef>
#include <chrono>
#include <string>
#include <cstdint>
#include <complex>
#include <csignal>

enum RadioType
{
 eNB=0,
 UE=1
};

class RadioConfig
{
public:
    RadioConfig(std::vector<std::string>, int, double, double, double, double, int, int, int, std::vector<int>);
    void radioStart(std::vector<void *> buffs);
    void radioStop();
    void radioTx(std::vector<void *> buffs);
    void radioRx(std::vector<void *> buffs);
    void radioSched(std::vector<int> sched);
    ~RadioConfig();
private:
    std::vector<std::string> _radioId;
    std::vector<SoapySDR::Device *> baStn;
    std::vector<SoapySDR::Stream *> txStreams;
    std::vector<SoapySDR::Stream *> rxStreams;
    // beacon
    std::vector<std::complex<int16_t>> buff;
    double _rate;
    double _freq;
    double _rxgain;
    double _txgain; 
    int _radioNum;
    int _numCh;
    int _symlen; //size of ofdm symbols as well as burst length
    int _symnum; // max value 256
    int _maxFrame;
    std::vector<int> _tddSched; // max len 256*16
    RadioType _radioType; 
};
