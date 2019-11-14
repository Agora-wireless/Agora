#ifndef RADIO_LIB
#define RADIO_LIB

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
    static void *configureBSRadio(void * context);
    bool radioStart();
    void radioStop();
    void readSensors();
    void radioTx(void ** buffs);
    void radioRx(void ** buffs);
    int radioTx(size_t, void ** buffs, int flags, long long & frameTime);
    int radioRx(size_t, void ** buffs, long long & frameTime);
    bool doCalib() { return calib; }
    bool correctSampleOffset(size_t, bool);
    static void drain_rx_buffer(SoapySDR::Device * ibsSdrs, SoapySDR::Stream * istream, std::vector<void *> buffs, size_t symSamp);
    void drain_buffers();
    void adjustDelays(std::vector<int>, size_t);
    void go();
    std::vector<std::vector<std::complex<float>>> get_calib_mat() { return calib_mat; }
    static void dciqMinimize(SoapySDR::Device*, SoapySDR::Device*, int, size_t, double, double);
    static void setIQBalance(SoapySDR::Device*, int, size_t, int, int);
    static void adjustCalibrationGains(std::vector<SoapySDR::Device*>, SoapySDR::Device*, size_t, double, bool plot = false);
    static std::vector<std::complex<float>> snoopSamples(SoapySDR::Device*, size_t, size_t);
    void dciqCalibrationProc(size_t);
    ~RadioConfig();
    struct RadioConfigContext
    {
        RadioConfig *ptr;
        size_t tid;
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
    size_t _radioNum;
    size_t _antennaNum;
    bool isUE;
    bool calib;
    RadioType _radioType; 
    std::atomic<size_t> remainingJobs;
    RadioConfigContext *context;
};
#endif
