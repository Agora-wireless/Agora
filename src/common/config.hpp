
#ifndef CONFIG_HEADER
#define CONFIG_HEADER

#include <boost/range/algorithm/count.hpp>
#include <iostream>
#include <complex.h>
#include <stdio.h>  /* for fprintf */
#include <string.h> /* for memcpy */
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <fstream>      // std::ifstream
#define JSON
#ifdef JSON
#include <nlohmann/json.hpp>
#include "Symbols.hpp"
#include "buffer.hpp"
#include "utils.h"
//#include <itpp/itbase.h>
//using namespace itpp;
using json = nlohmann::json;
#endif
typedef unsigned char uchar;
typedef unsigned short ushort;

class Config
{
public:
    int sampsPerSymbol;
    int prefix;
    int postfix;
    int symbolsPerFrame;
    int pilotSymsPerFrame;
    int ulSymsPerFrame;
    int dlSymsPerFrame;
    std::string modulation;
    
    std::string conf;
    //std::string beacon_file;
    //std::string coeffs_file;
    std::string pilot_file;
    std::string serial_file;
    std::string hub_file;
    std::string ref_ant;
    std::vector<std::string> radio_ids;
    std::vector<std::string> hub_ids;
    std::vector<std::string> frames;
    std::vector<std::vector<size_t>> pilotSymbols;
    std::vector<std::vector<size_t>> ULSymbols;
    std::vector<std::vector<size_t>> DLSymbols;
    std::vector<std::complex<int16_t>> beacon_ci16;
    std::vector<std::vector<uint32_t>> beacon_weights;
    std::vector<uint32_t> coeffs; 
    std::vector<std::complex<int16_t>> pilot_ci16;
    std::vector<uint32_t> pilot;
    std::vector<uint32_t> beacon;
    float *pilots_;
    int beacon_ant;
    int beacon_len;
    std::string beacon_mode;
    double freq;
    double txgainA;
    double rxgainA;
    double txgainB;
    double rxgainB;
    double calTxGainA;
    double calTxGainB;
    bool sampleCalEn;
    double rate;
    int framePeriod;
    int nCells;
    int nRadios;
    int nAntennas;
    int nUEs;
    int nChannels;
    int core_offset;
    bool isUE;
    const int maxFrame = 1 << 31;
    const int data_offset = sizeof(int) * 4;
    int dl_data_symbol_perframe;
    std::atomic<bool> running;
    int rx_port;
    int tx_port;
    std::string rx_addr;
    std::string tx_addr;
    int hdr_size = 8;
    // header 4 int for: frame_id, symbol_id, cell_id, ant_id
    // ushort for: I/Q samples
    int getRxPackageLength() { return sizeof(int) * hdr_size + sizeof(float) * sampsPerSymbol * 2; }
    // header 4 int for: frame_id, symbol_id, rsvd, ue_id
    // ushort for: I/Q samples
    int getTxPackageLength() { return sizeof(float) * sampsPerSymbol * 2; }
    //int getL2PackageLength() { return sizeof(int) * 4 + sizeof(mac_dtype) * FFT_LEN * nUEs; }
    int getNumAntennas() { return nRadios*nChannels; }
    int getDlSFIndex(int, int);
    int getUlSFIndex(int, int);
    int getDownlinkPilotId(int, int);
    int getPilotSFIndex(int, int);
    bool isPilot(int, int);
    bool isDownlink(int, int);
    bool isUplink(int, int);
    Config(std::string);
    ~Config();
};
#endif
