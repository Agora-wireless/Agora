
#ifndef CONFIG_HEADER
#define CONFIG_HEADER

#include <boost/range/algorithm/count.hpp>
#include <complex.h>
#include <emmintrin.h>
#include <fstream> // std::ifstream
#include <immintrin.h>
#include <iostream>
#include <stdio.h> /* for fprintf */
#include <stdlib.h>
#include <string.h> /* for memcpy */
#include <unistd.h>
#include <vector>
#define JSON
#ifdef JSON
#include "Symbols.hpp"
#include "buffer.hpp"
#include "comms-lib.h"
#include "memory_manage.h"
#include "utils.h"
#include <nlohmann/json.hpp>
//#include <itpp/itbase.h>
//using namespace itpp;
using json = nlohmann::json;
#endif
typedef unsigned char uchar;
typedef unsigned short ushort;

class Config {
public:
    int transpose_block_size;
    size_t sampsPerSymbol;
    size_t dl_prefix;
    size_t prefix;
    size_t postfix;
    std::string modulation;
    size_t mod_type;
    size_t mod_order;

    std::string conf;
    //std::string beacon_file;
    //std::string coeffs_file;
    std::string pilot_file;
    std::string serial_file;
    std::string hub_file;
    std::vector<std::string> radio_ids;
    std::vector<std::string> hub_ids;
    std::vector<std::string> frames;
    std::vector<std::vector<size_t>> pilotSymbols;
    std::vector<std::vector<size_t>> ULSymbols;
    std::vector<std::vector<size_t>> DLSymbols;
    std::vector<std::vector<size_t>> ULCalSymbols;
    std::vector<std::vector<size_t>> DLCalSymbols;
    std::vector<std::complex<int16_t>> beacon_ci16;
    std::vector<std::vector<uint32_t>> beacon_weights;
    std::vector<uint32_t> coeffs;
    std::vector<std::complex<int16_t>> pilot_ci16;
    std::vector<std::complex<float>> pilot_cf32;
    std::vector<std::complex<double>> pilot_cd64;
    std::vector<uint32_t> pilot;
    std::vector<uint32_t> beacon;
    float* pilots_;
    Table<int8_t> dl_IQ_data;
    Table<int8_t> ul_IQ_data;
    Table<complex_float> ul_IQ_modul;
    Table<complex_float> dl_IQ_modul;
    Table<std::complex<int16_t>> dl_IQ_symbol;
    std::vector<std::complex<float>> pilotsF;

    double freq;
    double txgainA;
    double rxgainA;
    double txgainB;
    double rxgainB;
    double calTxGainA;
    double calTxGainB;
    double rate;
    double nco;
    double radioRfFreq;
    double bwFilter;
    size_t framePeriod;
    size_t nCells;
    size_t nRadios;
    size_t nAntennas;
    size_t nChannels;
    size_t ref_ant;
    size_t beacon_ant;
    size_t beacon_len;
    bool beamsweep;
    bool sampleCalEn;
    bool imbalanceCalEn;
    bool recipCalEn;
    std::string channel;

    size_t core_offset;
    size_t worker_thread_num;
    size_t socket_thread_num;
    size_t fft_thread_num;
    size_t demul_thread_num;
    size_t zf_thread_num;
    size_t demul_block_size;
    int demul_block_num;
    size_t zf_block_size;
    int zf_block_num;

    bool freq_orthogonal_pilot;
    size_t BS_ANT_NUM;
    size_t UE_NUM;
    size_t OFDM_CA_NUM;
    size_t OFDM_DATA_NUM;
    size_t OFDM_DATA_START;
    size_t TX_PREFIX_LEN;
    size_t CP_LEN;
    size_t OFDM_PREFIX_LEN;
    size_t OFDM_FRAME_LEN;
    size_t OFDM_SYM_LEN;

    size_t symbol_num_perframe, pilot_symbol_num_perframe, data_symbol_num_perframe;
    size_t ul_data_symbol_num_perframe, dl_data_symbol_num_perframe;
    size_t dl_data_symbol_start, dl_data_symbol_end;
    bool downlink_mode;

    size_t packet_header_offset;
    size_t packet_length;

    std::string rx_addr;
    std::string tx_addr;
    int rx_port;
    int tx_port;

    /* LDPC parameters */
    LDPCconfig LDPC_config;

    bool isUE;
    const size_t maxFrame = 1 << 30;
    const size_t data_offset = sizeof(int) * 16;
    // int dl_data_symbol_perframe;
    std::atomic<bool> running;

    size_t getNumAntennas() { return nRadios * nChannels; }
    int getDlSFIndex(size_t, size_t);
    int getUlSFIndex(size_t, size_t);
    int getDownlinkPilotId(size_t, size_t);
    int getPilotSFIndex(size_t, size_t);
    bool isPilot(size_t, size_t);
    bool isCalDlPilot(size_t, size_t);
    bool isCalUlPilot(size_t, size_t);
    bool isDownlink(size_t, size_t);
    bool isUplink(size_t, size_t);
    Config(std::string);
    ~Config();
};
#endif
