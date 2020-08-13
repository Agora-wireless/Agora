
#ifndef CONFIG_HEADER
#define CONFIG_HEADER

#include <boost/range/algorithm/count.hpp>
#include <complex>
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


/// Metadata that describes a remote Subcarrier handler endpoint.
/// The remote subcarrier endpoint handles the given range of subcarriers
/// and is listening at the given IP address on the given port.
struct SubcarrierEndpoint {
    Range sc_range;
    std::string ip_addr;
    int port;
};


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

    bool hw_framer;

    std::vector<std::complex<float>> gold_cf32;
    std::vector<std::complex<int16_t>> beacon_ci16;
    std::vector<std::vector<uint32_t>> beacon_weights;
    std::vector<uint32_t> coeffs;
    std::vector<std::complex<int16_t>> pilot_ci16;
    std::vector<std::complex<float>> pilot_cf32;
    std::vector<uint32_t> pilot;
    std::vector<uint32_t> beacon;
    complex_float* pilots_;
    complex_float* pilots_sgn_;
    Table<int8_t> dl_bits;
    Table<int8_t> ul_bits;
    Table<int8_t> ul_encoded_bits;
    Table<uint8_t> ul_mod_input;
    Table<uint8_t> dl_mod_input;
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
    size_t mac_socket_thread_num;
    size_t fft_thread_num;
    size_t demul_thread_num;
    size_t decode_thread_num;
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
    size_t OFDM_DATA_NUM_pad; // Pad for cache line size (64 bytes)
    size_t OFDM_PILOT_NUM;
    size_t OFDM_PILOT_SPACING;
    size_t OFDM_DATA_START;
    size_t OFDM_DATA_STOP;
    size_t TX_PREFIX_LEN;
    size_t CP_LEN;
    size_t OFDM_PREFIX_LEN;
    size_t OFDM_FRAME_LEN;
    size_t OFDM_SYM_LEN;
    size_t DL_PILOT_SYMS;
    size_t UL_PILOT_SYMS;
    int cl_tx_advance;
    // Indicates all UEs that are in this experiment,
    // including the ones instantiated on other runs/machines.
    size_t total_ue_ant_num;
    // Indicates the (pilot) offset of the UEs in this instance,
    // with respect to all UEs used in the same experiment
    size_t ue_ant_offset;
    float scale; // scaling factor for all transmit symbols

    size_t symbol_num_perframe, pilot_symbol_num_perframe,
        data_symbol_num_perframe;
    size_t ul_data_symbol_num_perframe, dl_data_symbol_num_perframe;
    size_t dl_data_symbol_start, dl_data_symbol_end;
    bool downlink_mode;
    bool bigstation_mode;
    bool correct_phase_shift;

    size_t packet_length;
    size_t data_bytes_num_persymbol;
    size_t data_bytes_num_perframe;
    size_t mac_data_bytes_num_perframe;
    size_t mac_packet_length;
    size_t num_frames_per_mac_packet;
    size_t sym_packet_length;

    std::string server_addr; // IP address of the Millipede server
    std::string sender_addr; // IP address of the simulator sender
    std::string tx_addr_to_mac;
    // Port ID at Millipede side
    int bs_port;
    int ue_rx_port; // UDP port used by UEs to receive data
    int ue_tx_port; // UDP port used by UEs to transmit data

    // Port ID at MAC layer side
    int mac_rx_port;
    int mac_tx_port;
    bool init_mac_running;

    // Number of frames sent by sender during testing = number of frames
    // processed by Millipede before exiting.
    size_t frames_to_test;

    /* LDPC parameters */
    LDPCconfig LDPC_config;
    // Number of bytes per code block
    size_t num_bytes_per_cb;
    // Number of bytes per code block padded for cache line size (64 bytes)
    size_t num_bytes_per_cb_pad;

    bool fft_in_rru; // If true, the RRU does FFT instead of Millipede

    /// The list of remote subcarrier handler endpoints and the 
    /// ranges of subcarriers that they handle. 
    /// If this list is not empty, then the sender will use it
    /// to send subcarrier-specific packets to the proper handler endpoint.
    ///
    /// Below is an example json entry for two subcarrier endpoints.
    /// Note that "ip_addr" and "port" are optional.
    /// ------------------------------------------------------------
    ///     "subcarrier_endpoints": [
    ///        {
    ///            "sc_start": 0,
    ///            "sc_end": 600,
    ///            "ip_addr": "127.0.0.1",
    ///            "port": 8800
    ///        }, 
    ///        {
    ///            "sc_start": 600,
    ///            "sc_end": 1200,
    ///            "ip_addr": "127.0.0.1",
    ///            "port": 8810
    ///        }
    ///     ]
    ///
    std::vector<SubcarrierEndpoint> subcarrier_endpoints;

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

    inline size_t get_total_data_symbol_idx_dl(
        size_t frame_id, size_t symbol_idx_dl) const
    {
        return ((frame_id % TASK_BUFFER_FRAME_NUM)
                   * dl_data_symbol_num_perframe)
            + symbol_idx_dl;
    }

    /// Fetch the channel state information matrix for this frame and symbol ID.
    /// The symbol must be a pilot symbol.
    inline complex_float* get_csi_mat(Table<complex_float>& csi_buffers,
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

    /// Return a pointer to the downlink zeroforcing precoding matrix for this
    /// frame and subcarrier ID
    inline complex_float* get_dl_zf_mat(Table<complex_float>& dl_zf_buffers,
        size_t frame_id, size_t sc_id) const
    {
        if (freq_orthogonal_pilot)
            sc_id -= (sc_id % UE_NUM);
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        return dl_zf_buffers[(frame_slot * OFDM_DATA_NUM) + sc_id];
    }

    /// Return a pointer to the uplink zeroforcing detection matrix for this
    /// frame and subcarrier ID
    inline complex_float* get_ul_zf_mat(Table<complex_float>& ul_zf_buffers,
        size_t frame_id, size_t sc_id) const
    {
        if (freq_orthogonal_pilot)
            sc_id -= (sc_id % UE_NUM);
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        return ul_zf_buffers[(frame_slot * OFDM_DATA_NUM) + sc_id];
    }

    /// Get the calibration buffer for this frame and subcarrier ID
    inline complex_float* get_calib_buffer(
        Table<complex_float>& calib_buffer, size_t frame_id, size_t sc_id) const
    {
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        return &calib_buffer[frame_slot][sc_id * BS_ANT_NUM];
    }

    /// Get the soft demodulation buffer for this frame, symbol,
    /// user and subcarrier ID
    inline int8_t* get_demod_buf(Table<int8_t>& demod_buffer, size_t frame_id,
        size_t symbol_id, size_t ue_id, size_t sc_id) const
    {
        size_t total_data_symbol_id
            = get_total_data_symbol_idx_ul(frame_id, symbol_id);
        return &demod_buffer[total_data_symbol_id]
                            [OFDM_DATA_NUM * 8 * ue_id + sc_id * mod_type];
    }

    /// Get the decode buffer for this frame, symbol,
    /// user and code block ID
    inline uint8_t* get_decode_buf(Table<uint8_t>& decoded_buffer,
        size_t frame_id, size_t symbol_id, size_t ue_id, size_t cb_id) const
    {
        size_t total_data_symbol_id
            = get_total_data_symbol_idx_ul(frame_id, symbol_id);
        return &decoded_buffer[total_data_symbol_id][num_bytes_per_cb_pad
            * (LDPC_config.nblocksInSymbol * ue_id + cb_id)];
    }

    /// Get ul_bits for this symbol, user and code block ID
    inline int8_t* get_info_bits(Table<int8_t>& info_bits, size_t symbol_id,
        size_t ue_id, size_t cb_id) const
    {
        return &info_bits[symbol_id][num_bytes_per_cb_pad
            * (LDPC_config.nblocksInSymbol * ue_id + cb_id)];
    }

    /// Get encoded_buffer for this frame, symbol, user and code block ID
    inline int8_t* get_encoded_buf(Table<int8_t>& encoded_buffer,
        size_t frame_id, size_t symbol_id, size_t ue_id, size_t cb_id) const
    {
        size_t total_data_symbol_id
            = get_total_data_symbol_idx(frame_id, symbol_id);
        size_t num_encoded_bytes_per_cb = LDPC_config.cbCodewLen / mod_type;
        return &encoded_buffer[total_data_symbol_id][OFDM_DATA_NUM_pad * ue_id
            + num_encoded_bytes_per_cb * cb_id];
    }

    Config(std::string);
    void genData();
    ~Config();
};
#endif
