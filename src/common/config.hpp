
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
#include "modulation.hpp"
#include "utils.h"
#include <nlohmann/json.hpp>
//#include <itpp/itbase.h>
// using namespace itpp;
using json = nlohmann::json;
#endif
typedef unsigned char uchar;
typedef unsigned short ushort;

class Config {
public:
    size_t sampsPerSymbol;
    size_t dl_prefix;
    size_t prefix;
    size_t postfix;
    std::string modulation;
    size_t mod_type;
    size_t mod_order;

    std::string conf;
    std::string serial_file;
    std::string hub_file;
    std::vector<std::string> radio_ids;
    std::vector<std::string> hub_ids;

    // A string in \p frames contains letters representing the symbol types in
    // the frame (e.g., 'P' for pilot symbols, 'U' for uplink data symbols)
    std::vector<std::string> frames;

    // pilotSymbols[i] contains indices of pilot symbols in frames[i]
    std::vector<std::vector<size_t>> pilotSymbols;

    // ULSymbols[i] contains indices of uplink data symbols in frames[i]
    std::vector<std::vector<size_t>> ULSymbols;

    // DLSymbols[i] contains indices of downlink data symbols in frames[i]
    std::vector<std::vector<size_t>> DLSymbols;

    // ULCalSymbols[i] contains indices of uplink calibration symbols in
    // frames[i]
    std::vector<std::vector<size_t>> ULCalSymbols;

    // ULCalSymbols[i] contains indices of downlink calibration symbols in
    // frames[i]
    std::vector<std::vector<size_t>> DLCalSymbols;

    std::vector<std::complex<int16_t>> beacon_ci16;
    std::vector<std::vector<uint32_t>> beacon_weights;
    std::vector<uint32_t> coeffs;
    std::vector<std::complex<int16_t>> pilot_ci16;
    std::vector<std::complex<float>> pilot_cf32;
    std::vector<std::complex<double>> pilot_cd64;
    std::vector<uint32_t> pilot;
    std::vector<uint32_t> beacon;
    complex_float* pilots_;
    complex_float* pilots_sgn_;
    Table<int8_t> dl_bits;
    Table<int8_t> ul_bits;
    Table<complex_float> dl_iq_f;
    Table<complex_float> ul_iq_f;
    Table<std::complex<int16_t>> dl_iq_t;
    Table<std::complex<int16_t>> ul_iq_t;
    Table<complex_float> ue_specific_pilot;
    Table<std::complex<int16_t>> ue_specific_pilot_t;
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

    // Number of OFDM data subcarriers handled in one demodulation event
    size_t demul_block_size;
    size_t demul_events_per_symbol; // Derived from demul_block_size

    // Number of OFDM data subcarriers handled in one zeroforcing event
    size_t zf_block_size;
    size_t zf_events_per_symbol; // Derived from zf_block_size

    size_t fft_block_size;

    bool freq_orthogonal_pilot;
    size_t BS_ANT_NUM;
    size_t UE_NUM;
    size_t UE_ANT_NUM;
    size_t OFDM_CA_NUM;
    size_t OFDM_DATA_NUM;
    size_t OFDM_DATA_START;
    size_t OFDM_DATA_STOP;
    size_t TX_PREFIX_LEN;
    size_t CP_LEN;
    size_t OFDM_PREFIX_LEN;
    size_t OFDM_FRAME_LEN;
    size_t OFDM_SYM_LEN;
    size_t DL_PILOT_SYMS;
    size_t UL_PILOT_SYMS;

    size_t symbol_num_perframe, pilot_symbol_num_perframe,
        data_symbol_num_perframe;
    size_t ul_data_symbol_num_perframe, dl_data_symbol_num_perframe;
    size_t dl_data_symbol_start, dl_data_symbol_end;
    bool downlink_mode;
    bool bigstation_mode;
    bool correct_phase_shift;

    size_t packet_length;

    std::string rx_addr;
    std::string tx_addr;
    int bs_port;
    int ue_rx_port;
    int ue_tx_port;

    // Number of frames sent by sender during testing = number of frames
    // processed by Millipede before exiting.
    size_t frames_to_test;

    /* LDPC parameters */
    LDPCconfig LDPC_config;

    bool isUE;
    const size_t maxFrame = 1 << 30;
    const size_t data_offset = sizeof(int) * 16;
    // int dl_data_symbol_perframe;
    std::atomic<bool> running;

    size_t getNumAntennas() { return nRadios * nChannels; }
    int getSymbolId(size_t symbol_id);

    // Get the index of this downlink symbol among this frame's downlink symbols
    size_t get_dl_symbol_idx(size_t frame_id, size_t symbol_id) const;

    // Get the index of this uplink symbol among this frame's uplink symbols
    size_t get_ul_symbol_idx(size_t frame_id, size_t symbol_id) const;

    // Get the index of this pilot symbol among this frame's pilot symbols
    size_t get_pilot_symbol_idx(size_t frame_id, size_t symbol_id) const;

    bool isPilot(size_t, size_t);
    bool isCalDlPilot(size_t, size_t);
    bool isCalUlPilot(size_t, size_t);
    bool isDownlink(size_t, size_t);
    bool isUplink(size_t, size_t);

    /// Return the symbol type of this symbol in this frame
    SymbolType get_symbol_type(size_t frame_id, size_t symbol_id);

    // TODO: Documentation
    inline size_t get_total_data_symbol_idx(
        size_t frame_id, size_t symbol_id) const
    {
        return ((frame_id % TASK_BUFFER_FRAME_NUM) * data_symbol_num_perframe)
            + symbol_id;
    }

    // TODO: Documentation
    inline size_t get_total_data_symbol_idx_ul(
        size_t frame_id, size_t symbol_idx_ul) const
    {
        return ((frame_id % TASK_BUFFER_FRAME_NUM)
                   * ul_data_symbol_num_perframe)
            + symbol_idx_ul;
    }

    /// Fetch the channel state information buffer for this frame and symbol ID.
    /// The symbol must be a pilot symbol.
    inline complex_float* get_csi_buf(Table<complex_float>& csi_buffers,
        size_t frame_id, size_t symbol_id) const
    {
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        size_t symbol_offset = (frame_slot * pilot_symbol_num_perframe)
            + get_pilot_symbol_idx(frame_id, symbol_id);
        return csi_buffers[symbol_offset];
    }

    /// Fetch the data buffer for this frame and symbol ID. The symbol must
    /// be an uplink symbol.
    inline complex_float* get_data_buf(Table<complex_float>& data_buffers,
        size_t frame_id, size_t symbol_id) const
    {
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        size_t symbol_offset = (frame_slot * ul_data_symbol_num_perframe)
            + get_ul_symbol_idx(frame_id, symbol_id);
        return data_buffers[symbol_offset];
    }

    /// Get the precoder buffer for this frame and subcarrier ID
    inline complex_float* get_precoder_buf(
        Table<complex_float>& precoder_buffers, size_t frame_id,
        size_t sc_id) const
    {
        if (freq_orthogonal_pilot)
            sc_id -= (sc_id % UE_NUM);
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        return precoder_buffers[(frame_slot * OFDM_DATA_NUM) + sc_id];
    }

    /// Get the calibration buffer for this frame and subcarrier ID
    inline complex_float* get_calib_buffer(
        Table<complex_float>& calib_buffer, size_t frame_id, size_t sc_id) const
    {
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        return &calib_buffer[frame_slot][sc_id * BS_ANT_NUM];
    }

    Config(std::string);
    void genData();
    ~Config();
};
#endif
