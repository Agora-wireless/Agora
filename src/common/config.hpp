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
#include "utils_ldpc.hpp"
#include <nlohmann/json.hpp>
//#include <itpp/itbase.h>
// using namespace itpp;
using json = nlohmann::json;
#endif
typedef unsigned char uchar;
typedef unsigned short ushort;

class LDPCconfig {
public:
    uint16_t Bg; /// The 5G NR LDPC base graph (one or two)
    uint16_t Zc; /// The 5G NR LDPC expansion factor
    int16_t decoderIter; /// Maximum number of decoder iterations per codeblock

    /// Allow the LDPC decoder to terminate without completing all iterations
    /// if it decodes the codeblock eariler
    bool earlyTermination;

    size_t nRows; /// Number of rows in the LDPC base graph to use
    uint32_t cbLen; /// Number of information bits input to LDPC encoding
    uint32_t cbCodewLen; /// Number of codeword bits output from LDPC encoding
    size_t nblocksInSymbol;

    // Return the number of bytes in the information bit sequence for LDPC
    // encoding of one code block
    size_t num_input_bytes() const
    {
        return bits_to_bytes(ldpc_num_input_bits(Bg, Zc));
    }

    // Return the number of bytes in the encoded LDPC code word
    size_t num_encoded_bytes() const
    {
        return bits_to_bytes(ldpc_num_encoded_bits(Bg, Zc, nRows));
    }
};

class Config {
public:
    std::string modulation; // Modulation order as a string, e.g., "16QAM"
    size_t mod_order; // Modulation order (e.g., 4: QPSK, 16: 16QAM, 64: 64QAM)
    size_t mod_order_bits; // Number of binary bits used for a modulation order

    // Modulation lookup table for mapping binary bits to constellation points
    Table<float> mod_table;

    std::vector<std::string> radio_ids;
    std::vector<std::string> hub_ids;

    // A string in \p frames contains letters representing the symbol types in
    // the frame (e.g., 'P' for pilot symbols, 'U' for uplink data symbols)
    std::vector<std::string> frames;

    // beaconSymbols[i] contains IDs of beacon symbols in frames[i]
    std::vector<std::vector<size_t>> beaconSymbols;

    // pilotSymbols[i] contains IDs of pilot symbols in frames[i]
    std::vector<std::vector<size_t>> pilotSymbols;

    // ULSymbols[i] contains IDs of uplink data symbols in frames[i]
    std::vector<std::vector<size_t>> ULSymbols;

    // DLSymbols[i] contains IDs of downlink data symbols in frames[i]
    std::vector<std::vector<size_t>> DLSymbols;

    // ULCalSymbols[i] contains IDs of uplink calibration symbols in
    // frames[i]
    std::vector<std::vector<size_t>> ULCalSymbols;

    // DLCalSymbols[i] contains IDs of downlink calibration symbols in
    // frames[i]
    std::vector<std::vector<size_t>> DLCalSymbols;

    // Controls whether the synchronization and frame time keeping is done
    // in hardware or software
    // true: use hardware correlator; false: use software corrleator
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
    size_t rx_thread_num;
    size_t decode_thread_num;

    // Number of OFDM data subcarriers handled in one demodulation event
    size_t demul_block_size;
    size_t demul_events_per_symbol; // Derived from demul_block_size

    // Number of OFDM data subcarriers handled in one zeroforcing event
    size_t zf_block_size;
    size_t zf_events_per_symbol; // Derived from zf_block_size

    // Number of antennas handled in one FFT event
    size_t fft_block_size;

    bool freq_orthogonal_pilot;
    size_t BS_ANT_NUM;
    size_t UE_NUM;
    size_t UE_ANT_NUM;

    // The total number of OFDM subcarriers, which is a power of two
    size_t OFDM_CA_NUM;

    // The number of cyclic prefix IQ samples. These are taken from the tail of
    // the time-domain OFDM samples and prepended to the beginning.
    size_t CP_LEN;

    // The number of OFDM subcarriers that are non-zero in the frequency domain
    size_t OFDM_DATA_NUM;

    // The index of the first non-zero OFDM subcarrier (in the frequency domain)
    // in block of OFDM_CA_NUM subcarriers.
    size_t OFDM_DATA_START;

    // The index of the last non-zero OFDM subcarrier (in the frequency domain)
    // in block of OFDM_CA_NUM subcarriers.
    size_t OFDM_DATA_STOP;

    // The number of zero IQ samples prepended to a time-domain symbol (i.e.,
    // before the cyclic prefix) before transmission. Its value depends on
    // over-the-air and RF delays, and is currently calculated by manual tuning.
    size_t ofdm_tx_zero_prefix_;

    // The number of zero IQ samples appended to a time-domain symbol before
    // transmission. Its value depends on over-the-air and RF delays, and is
    // currently calculated by manual tuning.
    size_t ofdm_tx_zero_postfix_;

    // The number of IQ samples to skip from the beginning of symbol received by
    // Agora on the uplink at the base station. Due to over-the-air and RF
    // delays, this can be different from (prefix + CP_LEN), and is currently
    // calculated by manual tuning.
    size_t ofdm_rx_zero_prefix_bs_;

    // The number of IQ samples to skip from the beginning of symbol received by
    // Agora on the downlink at the client. Due to over-the-air and RF
    // delays, this can be different from (prefix + CP_LEN), and is currently
    // calculated by manual tuning.
    size_t ofdm_rx_zero_prefix_client_;

    // The total number of IQ samples in one physical layer time-domain packet
    // received or sent by Agora
    size_t sampsPerSymbol;

    // The number of bytes in one physical layer time-domain packet received or
    // sent by Agora. This includes Agora's packet header, but not the
    // Ethernet/IP/UDP headers.
    size_t packet_length;

    size_t OFDM_PILOT_SPACING;
    size_t TX_PREFIX_LEN;

    size_t DL_PILOT_SYMS;
    size_t UL_PILOT_SYMS;
    int cl_tx_advance;
    // Indicates all UEs that are in this experiment,
    // including the ones instantiated on other runs/machines.
    size_t total_ue_ant_num;
    // Indicates the (pilot) offset of the UEs in this instance,
    // with respect to all UEs used in the same experiment
    size_t ue_ant_offset;
    float scale; // Scaling factor for all transmit symbols

    // Total number of symbols in a frame, including all types of symbols (e.g.,
    // pilot symbols, uplink and downlink data symbols, and calibration symbols)
    size_t symbol_num_perframe;

    // Total number of beacon symbols in a frame
    size_t beacon_symbol_num_perframe;

    // Total number of pilot symbols in a frame
    size_t pilot_symbol_num_perframe;

    // Total number of data symbols in a frame, including uplink data symbols
    // and downlink data symbols
    size_t data_symbol_num_perframe;

    size_t ul_data_symbol_num_perframe, dl_data_symbol_num_perframe;
    size_t dl_data_symbol_start, dl_data_symbol_end;
    bool downlink_mode; // If true, the frame contains downlink symbols
    bool correct_phase_shift; // If true, do phase shift correction

    // The total number of uncoded data bytes in each OFDM symbol
    size_t data_bytes_num_persymbol;

    // The total number of MAC payload data bytes in each Frame
    size_t mac_data_bytes_num_perframe;

    // The total number of MAC packet bytes in each Frame
    size_t mac_bytes_num_perframe;

    // The length (in bytes) of a MAC packet including the header
    size_t mac_packet_length;

    // The length (in bytes) of a MAC packet payload
    size_t mac_payload_length;

    // The total number of mac packets sent/received in each frame
    size_t mac_packets_perframe;

    // IP address of the machine running the baseband processing for UE
    std::string ue_server_addr;

    // IP address of the machine running the baseband processing for BS
    std::string bs_server_addr;

    // IP address of the base station RRU, RRU emulator (sender),
    // or channel simulator
    std::string bs_rru_addr;

    // IP address of the data source/sink server communicating with MAC (BS/UE)
    std::string mac_remote_addr;

    int bs_server_port; // Base UDP port used by BS to receive data

    // Base RRU/channel simulator UDP port used by BS to transmit downlink data
    int bs_rru_port;

    int ue_server_port; // Base UDP port used by UEs to receive data

    // Base RRU/channel simulator UDP port used by UEs to transmit uplink data
    int ue_rru_port;

    // Port ID at MAC layer side
    int mac_rx_port;
    int mac_tx_port;
    bool init_mac_running;

    // Number of frames sent by sender during testing = number of frames
    // processed by Agora before exiting.
    size_t frames_to_test;

    // Size of tranport block given by upper layer
    size_t transport_block_size;
    LDPCconfig LDPC_config; // LDPC parameters

    // Number of bytes per code block
    size_t num_bytes_per_cb;

    // # subcarriers for each dosubcarrier worker, should be a multiple of
    // lcm(zf_block_size, demul_block_size)
    size_t subcarrier_block_size;

    // # threads for decoding each user data
    size_t decode_thread_num_per_ue;

    // The list of IP addresses of all Agora servers
    std::vector<std::string> bs_server_addr_list;
    size_t bs_server_addr_idx; // The index of this Agora server in the list

    // This Agora server takes charge of subcarrier range
    // [subcarrier_start, subcarrier_end]
    std::vector<size_t> subcarrier_num_list;
    std::vector<size_t> subcarrier_num_start;
    size_t subcarrier_start, subcarrier_end;

    // This Agora server takes charge of ue range [ue_start, ue_end]
    size_t ue_start, ue_end;

    uint16_t fft_tx_port;
    uint16_t fft_rx_port;

    uint16_t demod_tx_port; // UDP port used to receive post-demodulation data
    uint16_t demod_rx_port; // UDP port used to send post-demodulation data

    uint16_t encode_tx_port; // UDP port used to receive encoded data
    uint16_t encode_rx_port; // UDP port used to send encoded data

    // The list of MAC addresses of all Agora servers 
    // Only used in distributed&DPDK version
    std::vector<std::string> bs_server_mac_list;
    std::string bs_rru_mac_addr;

    // NIC PCIe address (for DPDK)
    std::string pci_addr;

    // Dynamic workload
    int fixed_control;
    std::vector<size_t> user_level_list;
    size_t num_load_levels;
    bool sleep_mode;

    // IQ sample mode
    bool use_time_domain_iq;
    size_t fft_thread_num = 0;
    size_t fft_tx_thread_num = 0;

    size_t ant_start, ant_end;

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

    inline size_t get_num_ant_to_process() const
    {
        return ant_end - ant_start;
    }

    // Get the number of subcarriers this server takes charge of
    inline size_t get_num_sc_per_server() const
    {
        return OFDM_DATA_NUM / bs_server_addr_list.size();
    }

    inline size_t get_num_sc_to_process() const
    {
        return subcarrier_end - subcarrier_start;
    }

    // Get the number of UEs this server takes charge of
    inline size_t get_num_ues_to_process() const
    {
        return ue_end - ue_start;
    }

    // Get the Agora server index given an UE ID
    inline size_t get_server_idx_by_ue(size_t ue_id) const
    {
        size_t ue_num_low = UE_NUM / bs_server_addr_list.size();
        size_t num_extra_ue = UE_NUM % bs_server_addr_list.size();
        if (ue_id < num_extra_ue * (ue_num_low + 1)) {
            return ue_id / (ue_num_low + 1);
        } else {
            return num_extra_ue
                + (ue_id - num_extra_ue * (ue_num_low + 1)) / ue_num_low;
        }
    }

    // Update this config to use [new_mod_order_bits] as the modulation order
    inline void update_mod_cfgs(size_t new_mod_order_bits)
    {
        mod_order_bits = new_mod_order_bits;
        mod_order = (size_t)pow(2, mod_order_bits);
        init_modulation_table(mod_table, mod_order);
        LDPC_config.nblocksInSymbol
            = OFDM_DATA_NUM * mod_order_bits / LDPC_config.cbCodewLen;
    }

    /// Return total number of data symbols of all frames in a buffer
    /// that holds data of TASK_BUFFER_FRAME_NUM frames
    inline size_t get_total_data_symbol_idx(
        size_t frame_id, size_t symbol_id) const
    {
        return ((frame_id % TASK_BUFFER_FRAME_NUM) * data_symbol_num_perframe)
            + symbol_id;
    }

    /// Return total number of uplink data symbols of all frames in a buffer
    /// that holds data of TASK_BUFFER_FRAME_NUM frames
    inline size_t get_total_data_symbol_idx_ul(
        size_t frame_id, size_t symbol_idx_ul) const
    {
        return ((frame_id % TASK_BUFFER_FRAME_NUM)
                   * ul_data_symbol_num_perframe)
            + symbol_idx_ul;
    }

    /// Return total number of downlink data symbols of all frames in a buffer
    /// that holds data of TASK_BUFFER_FRAME_NUM frames
    inline size_t get_total_data_symbol_idx_dl(
        size_t frame_id, size_t symbol_idx_dl) const
    {
        return ((frame_id % TASK_BUFFER_FRAME_NUM)
                   * dl_data_symbol_num_perframe)
            + symbol_idx_dl;
    }

    /// Return the frame duration in seconds
    inline double get_frame_duration_sec()
    {
        return symbol_num_perframe * sampsPerSymbol / rate;
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

    /// Return the subcarrier ID to which we should refer to for the zeroforcing
    /// matrices of subcarrier [sc_id].
    inline size_t get_zf_sc_id(size_t sc_id) const
    {
        return freq_orthogonal_pilot ? sc_id - (sc_id % UE_NUM) : sc_id;
    }

    inline char* get_time_domain_iq_buffer(
        Table<char>& time_domain_iq_buffer, size_t ant_id, size_t frame_id, size_t symbol_id) const
    {
        return &time_domain_iq_buffer[ant_id][((frame_id % kFrameWnd) * symbol_num_perframe + symbol_id) * packet_length];    
    }

    inline char* get_freq_domain_iq_buffer(
        Table<char>& freq_domain_iq_buffer, size_t ant_id, size_t frame_id, size_t symbol_id) const
    {
        return &freq_domain_iq_buffer[ant_id][((frame_id % kFrameWnd) * symbol_num_perframe + symbol_id) * packet_length];
    }

    /// Get the calibration buffer for this frame and subcarrier ID
    inline complex_float* get_calib_buffer(
        Table<complex_float>& calib_buffer, size_t frame_id, size_t sc_id) const
    {
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        return &calib_buffer[frame_slot][sc_id * BS_ANT_NUM];
    }

    /// Get the decode buffer for this frame, symbol, user and code block ID
    inline uint8_t* get_decode_buf(Table<uint8_t>& decoded_buffer,
        size_t frame_id, size_t symbol_id, size_t ue_id, size_t cb_id) const
    {
        size_t total_data_symbol_id
            = get_total_data_symbol_idx_ul(frame_id, symbol_id);
        return &decoded_buffer[total_data_symbol_id][roundup<64>(
                                                         num_bytes_per_cb)
            * (LDPC_config.nblocksInSymbol * ue_id + cb_id)];
    }

    inline int8_t* get_demod_buf_to_send(
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_to_send,
        size_t frame_id, size_t symbol_id_ul, size_t ue_id) const
    {
        return &demod_buffer_to_send[frame_id % kFrameWnd]
            [symbol_id_ul][ue_id][subcarrier_start * mod_order_bits];
    }

    inline int8_t* get_demod_buf_to_decode(
        Table<int8_t>& demod_buffer_to_decode, size_t frame_id,
        size_t symbol_id_ul, size_t ue_id, size_t sc_id) const
    {
        size_t total_data_symbol_id
            = get_total_data_symbol_idx_ul(frame_id, symbol_id_ul);
        return &demod_buffer_to_decode[total_data_symbol_id]
                                      [OFDM_DATA_NUM * kMaxModType * ue_id
                                          + sc_id * mod_order_bits];
    }

    /// Get ul_bits for this symbol, user and code block ID
    inline int8_t* get_info_bits(Table<int8_t>& info_bits, size_t symbol_id,
        size_t ue_id, size_t cb_id) const
    {
        return &info_bits[symbol_id][roundup<64>(num_bytes_per_cb)
            * (LDPC_config.nblocksInSymbol * ue_id + cb_id)];
    }

    /// Get encoded_buffer for this frame, symbol, user and code block ID
    inline int8_t* get_encoded_buf(Table<int8_t>& encoded_buffer,
        size_t frame_id, size_t symbol_id_dl, size_t ue_id, size_t cb_id) const
    {
        size_t total_data_symbol_id
            = get_total_data_symbol_idx_dl(frame_id, symbol_id_dl);
        size_t num_encoded_bytes_per_cb
            = LDPC_config.cbCodewLen / mod_order_bits;
        return &encoded_buffer[total_data_symbol_id]
                              [roundup<64>(OFDM_DATA_NUM) * ue_id
                                  + num_encoded_bytes_per_cb * cb_id];
    }

    // Returns the number of pilot subcarriers in downlink symbols used for
    // phase tracking
    inline size_t get_ofdm_pilot_num() const
    {
        return OFDM_DATA_NUM / OFDM_PILOT_SPACING;
    }

    Config(std::string);
    void genData();
    ~Config();
};
#endif
