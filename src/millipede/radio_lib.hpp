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
#include "config.hpp"
enum RadioType
{
 eNB=0,
 UE=1
};

class RadioConfig
{
public:
    RadioConfig(Config *cfg);
    static void *initBSRadio(void * context);
    bool radioStart();
    void radioStop();
    void readSensors();
    void radioTx(void ** buffs);
    void radioRx(void ** buffs);
    void radioTx(int, void ** buffs, int flags, long long & frameTime);
    int radioRx(int, void ** buffs, long long & frameTime);
    bool doCalib() { return calib; }
    bool calib_proc(int, bool);
    static void drain_rx_buffer(SoapySDR::Device * ibsSdrs, SoapySDR::Stream * istream, std::vector<void *> buffs, int symSamp);
    void drain_buffers();
    void adjustDelays(std::vector<int>, int);
    void go();
    std::vector<std::vector<std::complex<float>>> get_calib_mat() { return calib_mat; }
    ~RadioConfig();
    struct RadioConfigContext
    {
        RadioConfig *ptr;
        int tid;
    };

private:
    Config *_cfg;
    std::vector<SoapySDR::Device *> hubs;
    std::vector<SoapySDR::Device *> baStn;
    SoapySDR::Device *ref; 
    SoapySDR::Stream * refRxStream; 
    std::vector<SoapySDR::Stream *> txStreams;
    std::vector<SoapySDR::Stream *> rxStreams;
    std::vector<std::vector<std::complex<float>>> calib_mat;
    int _radioNum;
    int _antennaNum;
    bool isUE;
    bool calib;
    RadioType _radioType; 
    std::atomic<int> remainingJobs;
    RadioConfigContext *context;
};
