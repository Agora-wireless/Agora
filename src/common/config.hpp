
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
#include <immintrin.h>
#include <emmintrin.h>
#define JSON
#ifdef JSON
#include <nlohmann/json.hpp>
#include "Symbols.hpp"
#include "memory_manage.h"
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
    int dl_prefix;
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
    size_t ref_ant;
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
    std::vector<std::complex<float>> pilot_cf32;
    std::vector<uint32_t> pilot;
    std::vector<uint32_t> beacon;
    float *pilots_;
    int **dl_IQ_data;
    int **ul_IQ_data;
    complex_float **ul_IQ_modul;
    
    int beacon_ant;
    int beacon_len;
    std::string beacon_mode;
    double freq;
    double bbf_ratio;
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
    int worker_thread_num;
    int socket_thread_num;
    int fft_thread_num;
    int demul_thread_num;
    int zf_thread_num;
    int demul_block_size;
    int zf_block_size;


    int BS_ANT_NUM;
    int UE_NUM;
    int OFDM_CA_NUM;
    int OFDM_DATA_NUM;
    int OFDM_DATA_START;
    int TX_PREFIX_LEN;
    int CP_LEN;
    int OFDM_PREFIX_LEN;
    int OFDM_FRAME_LEN;

    int symbol_num_perframe, pilot_symbol_num_perframe, data_symbol_num_perframe;
    int ul_data_symbol_num_perframe, dl_data_symbol_num_perframe;
    int dl_data_symbol_start, dl_data_symbol_end;
    bool downlink_mode;

    int package_header_offset;
    int package_length;

    std::string rx_addr;
    std::string tx_addr;
    int rx_port;
    int tx_port;

    bool isUE;
    const int maxFrame = 1 << 31;
    const int data_offset = sizeof(int) * 16;
    // int dl_data_symbol_perframe;
    std::atomic<bool> running;
    
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
