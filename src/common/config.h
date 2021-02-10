#ifndef CONFIG_HEADER
#define CONFIG_HEADER

#include <emmintrin.h>
#include <immintrin.h>
#include <unistd.h>

#include <boost/range/algorithm/count.hpp>
#include <fstream>  // std::ifstream
#include <iostream>
#include <vector>
#define JSON
#ifdef JSON
#include <nlohmann/json.hpp>

#include "buffer.h"
#include "comms-lib.h"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.h"
#include "symbols.h"
#include "utils.h"
#include "utils_ldpc.h"

using json = nlohmann::json;
#endif
typedef unsigned char uchar;
typedef unsigned short ushort;

class LDPCconfig {
 public:
  uint16_t bg_;           /// The 5G NR LDPC base graph (one or two)
  uint16_t zc_;           /// The 5G NR LDPC expansion factor
  int16_t decoder_iter_;  /// Maximum number of decoder iterations per codeblock

  /// Allow the LDPC decoder to terminate without completing all iterations
  /// if it decodes the codeblock eariler
  bool early_termination_;

  size_t n_rows_;          /// Number of rows in the LDPC base graph to use
  uint32_t cb_len_;        /// Number of information bits input to LDPC encoding
  uint32_t cb_codew_len_;  /// Number of codeword bits output from LDPC encoding
  size_t nblocks_in_symbol_;

  // Return the number of bytes in the information bit sequence for LDPC
  // encoding of one code block
  size_t NumInputBytes() const {
    return BitsToBytes(LdpcNumInputBits(bg_, zc_));
  }

  // Return the number of bytes in the encoded LDPC code word
  size_t NumEncodedBytes() const {
    return BitsToBytes(LdpcNumEncodedBits(bg_, zc_, n_rows_));
  }
};

class Config {
 public:
  const double freq_ghz_;  // RDTSC frequency in GHz

  std::string modulation_;  // Modulation order as a string, e.g., "16QAM"
  size_t mod_order_;  // Modulation order (e.g., 4: QPSK, 16: 16QAM, 64: 64QAM)
  size_t mod_order_bits_;  // Number of binary bits used for a modulation order

  // Modulation lookup table for mapping binary bits to constellation points
  Table<complex_float> mod_table_;

  std::vector<std::string> radio_ids_;
  std::vector<std::string> hub_ids_;

  // A string in \p frames contains letters representing the symbol types in
  // the frame (e.g., 'P' for pilot symbols, 'U' for uplink data symbols)
  std::vector<std::string> frames_;

  // beaconSymbols[i] contains IDs of beacon symbols in frames[i]
  std::vector<std::vector<size_t>> beacon_symbols_;

  // pilotSymbols[i] contains IDs of pilot symbols in frames[i]
  std::vector<std::vector<size_t>> pilot_symbols_;

  // ULSymbols[i] contains IDs of uplink data symbols in frames[i]
  std::vector<std::vector<size_t>> ul_symbols_;

  // DLSymbols[i] contains IDs of downlink data symbols in frames[i]
  std::vector<std::vector<size_t>> dl_symbols_;

  // ULCalSymbols[i] contains IDs of uplink calibration symbols in
  // frames[i]
  std::vector<std::vector<size_t>> ul_cal_symbols_;

  // DLCalSymbols[i] contains IDs of downlink calibration symbols in
  // frames[i]
  std::vector<std::vector<size_t>> dl_cal_symbols_;

  // Controls whether the synchronization and frame time keeping is done
  // in hardware or software
  // true: use hardware correlator; false: use software corrleator
  bool hw_framer_;

  std::vector<std::complex<float>> gold_cf32_;
  std::vector<std::complex<int16_t>> beacon_ci16_;
  std::vector<std::vector<uint32_t>> beacon_weights_;
  std::vector<uint32_t> coeffs_;
  std::vector<std::complex<int16_t>> pilot_ci16_;
  std::vector<std::complex<float>> pilot_cf32_;
  std::vector<uint32_t> pilot_;
  std::vector<uint32_t> beacon_;
  complex_float* pilots_;
  complex_float* pilots_sgn_;
  Table<int8_t> dl_bits_;
  Table<int8_t> ul_bits_;
  Table<int8_t> ul_encoded_bits_;
  Table<uint8_t> ul_mod_input_;
  Table<uint8_t> dl_mod_input_;
  Table<complex_float> dl_iq_f_;
  Table<complex_float> ul_iq_f_;
  Table<std::complex<int16_t>> dl_iq_t_;
  Table<std::complex<int16_t>> ul_iq_t_;
  Table<complex_float> ue_specific_pilot_;
  Table<std::complex<int16_t>> ue_specific_pilot_t_;
  std::vector<std::complex<float>> common_pilot_;

  double freq_;
  double rate_;
  double nco_;
  double radio_rf_freq_;
  double bw_filter_;
  bool single_gain_;
  double tx_gain_a_;
  double rx_gain_a_;
  double tx_gain_b_;
  double rx_gain_b_;
  double calib_tx_gain_a_;
  double calib_tx_gain_b_;
  std::vector<double> client_gain_adj_a_;
  std::vector<double> client_gain_adj_b_;

  size_t n_cells_;
  size_t n_radios_;
  size_t n_antennas_;
  size_t n_channels_;
  size_t ref_ant_;
  size_t beacon_ant_;
  size_t beacon_len_;
  size_t init_calib_repeat_;
  bool beamsweep_;
  bool sample_cal_en_;
  bool imbalance_cal_en_;
  bool recip_cal_en_;
  bool external_ref_node_;
  std::string channel_;
  size_t ant_group_num_;
  size_t ant_per_group_;

  size_t core_offset_;
  size_t worker_thread_num_;
  size_t socket_thread_num_;
  size_t fft_thread_num_;
  size_t demul_thread_num_;
  size_t decode_thread_num_;
  size_t zf_thread_num_;

  // Number of OFDM data subcarriers handled in one demodulation event
  size_t demul_block_size_;
  size_t demul_events_per_symbol_;  // Derived from demul_block_size

  // Number of OFDM data subcarriers handled in one doZF function call
  size_t zf_block_size_;

  // Number of doZF function call handled in on event
  size_t zf_batch_size_;
  size_t zf_events_per_symbol_;  // Derived from zf_block_size

  // Number of antennas handled in one FFT event
  size_t fft_block_size_;

  // Number of code blocks handled in one encode event
  size_t encode_block_size_;

  bool freq_orthogonal_pilot_;
  size_t bs_ant_num_;  // Total number of BS antennas
  size_t bf_ant_num_;  // Number of antennas used in beamforming
  size_t ue_num_;
  size_t ue_ant_num_;

  // The total number of OFDM subcarriers, which is a power of two
  size_t ofdm_ca_num_;

  // The number of cyclic prefix IQ samples. These are taken from the tail of
  // the time-domain OFDM samples and prepended to the beginning.
  size_t cp_len_;

  // The number of OFDM subcarriers that are non-zero in the frequency domain
  size_t ofdm_data_num_;

  // The index of the first non-zero OFDM subcarrier (in the frequency domain)
  // in block of OFDM_CA_NUM subcarriers.
  size_t ofdm_data_start_;

  // The index of the last non-zero OFDM subcarrier (in the frequency domain)
  // in block of OFDM_CA_NUM subcarriers.
  size_t ofdm_data_stop_;

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

  size_t ofdm_rx_zero_prefix_cal_ul_;
  size_t ofdm_rx_zero_prefix_cal_dl_;

  // The number of IQ samples to skip from the beginning of symbol received by
  // Agora on the downlink at the client. Due to over-the-air and RF
  // delays, this can be different from (prefix + CP_LEN), and is currently
  // calculated by manual tuning.
  size_t ofdm_rx_zero_prefix_client_;

  // The total number of IQ samples in one physical layer time-domain packet
  // received or sent by Agora
  size_t samps_per_symbol_;

  // The number of bytes in one physical layer time-domain packet received or
  // sent by Agora. This includes Agora's packet header, but not the
  // Ethernet/IP/UDP headers.
  size_t packet_length_;
  size_t dl_packet_length_;

  size_t ofdm_pilot_spacing_;

  size_t dl_pilot_syms_;
  size_t ul_pilot_syms_;
  std::vector<int> cl_tx_advance_;
  // Indicates all UEs that are in this experiment,
  // including the ones instantiated on other runs/machines.
  size_t total_ue_ant_num_;
  // Indicates the (pilot) offset of the UEs in this instance,
  // with respect to all UEs used in the same experiment
  size_t ue_ant_offset_;
  float scale_;  // Scaling factor for all transmit symbols

  // Total number of symbols in a frame, including all types of symbols (e.g.,
  // pilot symbols, uplink and downlink data symbols, and calibration symbols)
  size_t symbol_num_perframe_;

  // Total number of beacon symbols in a frame
  size_t beacon_symbol_num_perframe_;

  // Total number of pilot symbols in a frame
  size_t pilot_symbol_num_perframe_;

  // Total number of data symbols in a frame, including uplink data symbols
  // and downlink data symbols
  size_t data_symbol_num_perframe_;

  // Total number of pilot symbols in a frame
  size_t recip_pilot_symbol_num_perframe_;

  size_t ul_data_symbol_num_perframe_, dl_data_symbol_num_perframe_;
  size_t dl_data_symbol_start_, dl_data_symbol_end_;
  bool downlink_mode_;        // If true, the frame contains downlink symbols
  bool bigstation_mode_;      // If true, use pipeline-parallel scheduling
  bool correct_phase_shift_;  // If true, do phase shift correction

  // The total number of uncoded data bytes in each OFDM symbol
  size_t data_bytes_num_persymbol_;

  // The total number of MAC payload data bytes in each Frame
  size_t mac_data_bytes_num_perframe_;

  // The total number of MAC packet bytes in each Frame
  size_t mac_bytes_num_perframe_;

  // The length (in bytes) of a MAC packet including the header
  size_t mac_packet_length_;

  // The length (in bytes) of a MAC packet payload
  size_t mac_payload_length_;

  // The total number of mac packets sent/received in each frame
  size_t mac_packets_perframe_;

  // IP address of the machine running the baseband processing for UE
  std::string ue_server_addr_;

  // IP address of the machine running the baseband processing for BS
  std::string bs_server_addr_;

  // IP address of the base station RRU, RRU emulator (sender),
  // or channel simulator
  std::string bs_rru_addr_;

  // IP address of the data source/sink server communicating with MAC (BS/UE)
  std::string mac_remote_addr_;

  int bs_server_port_;  // Base UDP port used by BS to receive data

  // Base RRU/channel simulator UDP port used by BS to transmit downlink data
  int bs_rru_port_;

  int ue_server_port_;  // Base UDP port used by UEs to receive data

  // Base RRU/channel simulator UDP port used by UEs to transmit uplink data
  int ue_rru_port_;

  // Number of NIC ports used for DPDK
  uint16_t dpdk_num_ports_;

  // Port ID at MAC layer side
  int mac_rx_port_;
  int mac_tx_port_;
  bool init_mac_running_;

  // Number of frames sent by sender during testing = number of frames
  // processed by Agora before exiting.
  size_t frames_to_test_;

  // Size of tranport block given by upper layer
  size_t transport_block_size_;

  float noise_level_;
  LDPCconfig ldpc_config_;  // LDPC parameters

  // Number of bytes per code block
  size_t num_bytes_per_cb_;

  bool fft_in_rru_;  // If true, the RRU does FFT instead of Agora

  bool is_ue_;
  const size_t max_frame_ = 1 << 30;
  const size_t data_offset_ = sizeof(int) * 16;
  // int dl_data_symbol_perframe;
  std::atomic<bool> running_;

  size_t GetNumAntennas() const { return n_radios_ * n_channels_; }
  int GetSymbolId(size_t symbol_id);

  // Get the index of this downlink symbol among this frame's downlink symbols
  size_t GetDlSymbolIdx(size_t frame_id, size_t symbol_id) const;

  // Get the index of this uplink symbol among this frame's uplink symbols
  size_t GetUlSymbolIdx(size_t frame_id, size_t symbol_id) const;

  // Get the index of this pilot symbol among this frame's pilot symbols
  size_t GetPilotSymbolIdx(size_t frame_id, size_t symbol_id) const;

  bool IsPilot(size_t /*frame_id*/, size_t /*symbol_id*/);
  bool IsCalDlPilot(size_t /*frame_id*/, size_t /*symbol_id*/);
  bool IsCalUlPilot(size_t /*frame_id*/, size_t /*symbol_id*/);
  bool IsDownlink(size_t /*frame_id*/, size_t /*symbol_id*/);
  bool IsUplink(size_t /*frame_id*/, size_t /*symbol_id*/);

  /// Return the single-gain control decision
  inline bool SingleGain(void) const { return this->single_gain_; }

  /// Return the symbol type of this symbol in this frame
  SymbolType GetSymbolType(size_t frame_id, size_t symbol_id);

  inline void UpdateModCfgs(size_t new_mod_order_bits) {
    mod_order_bits_ = new_mod_order_bits;
    mod_order_ = (size_t)pow(2, mod_order_bits_);
    InitModulationTable(mod_table_, mod_order_);
    ldpc_config_.nblocks_in_symbol_ =
        ofdm_data_num_ * mod_order_bits_ / ldpc_config_.cb_codew_len_;
  }

  /// Return total number of data symbols of all frames in a buffer
  /// that holds data of kFrameWnd frames
  inline size_t GetTotalDataSymbolIdx(size_t frame_id, size_t symbol_id) const {
    return ((frame_id % kFrameWnd) * data_symbol_num_perframe_) + symbol_id;
  }

  /// Return total number of uplink data symbols of all frames in a buffer
  /// that holds data of kFrameWnd frames
  inline size_t GetTotalDataSymbolIdxUl(size_t frame_id,
                                        size_t symbol_idx_ul) const {
    return ((frame_id % kFrameWnd) * ul_data_symbol_num_perframe_) +
           symbol_idx_ul;
  }

  /// Return total number of downlink data symbols of all frames in a buffer
  /// that holds data of kFrameWnd frames
  inline size_t GetTotalDataSymbolIdxDl(size_t frame_id,
                                        size_t symbol_idx_dl) const {
    return ((frame_id % kFrameWnd) * dl_data_symbol_num_perframe_) +
           symbol_idx_dl;
  }

  /// Return the frame duration in seconds
  inline double GetFrameDurationSec() const {
    return symbol_num_perframe_ * samps_per_symbol_ / rate_;
  }

  /// Fetch the data buffer for this frame and symbol ID. The symbol must
  /// be an uplink symbol.
  inline complex_float* GetDataBuf(Table<complex_float>& data_buffers,
                                   size_t frame_id, size_t symbol_id) const {
    size_t frame_slot = frame_id % kFrameWnd;
    size_t symbol_offset = (frame_slot * ul_data_symbol_num_perframe_) +
                           GetUlSymbolIdx(frame_id, symbol_id);
    return data_buffers[symbol_offset];
  }

  /// Return the subcarrier ID to which we should refer to for the zeroforcing
  /// matrices of subcarrier [sc_id].
  inline size_t GetZfScId(size_t sc_id) const {
    return freq_orthogonal_pilot_ ? sc_id - (sc_id % ue_num_) : sc_id;
  }

  /// Get the calibration buffer for this frame and subcarrier ID
  inline complex_float* GetCalibBuffer(Table<complex_float>& calib_buffer,
                                       size_t frame_id, size_t sc_id) const {
    size_t frame_slot = frame_id % kFrameWnd;
    return &calib_buffer[frame_slot][sc_id * bs_ant_num_];
  }

  /// Get the decode buffer for this frame, symbol, user and code block ID
  inline uint8_t* GetDecodeBuf(Table<uint8_t>& decoded_buffer, size_t frame_id,
                               size_t symbol_id, size_t ue_id,
                               size_t cb_id) const {
    size_t total_data_symbol_id = GetTotalDataSymbolIdxUl(frame_id, symbol_id);
    return &decoded_buffer[total_data_symbol_id]
                          [Roundup<64>(num_bytes_per_cb_) *
                           (ldpc_config_.nblocks_in_symbol_ * ue_id + cb_id)];
  }

  /// Get ul_bits for this symbol, user and code block ID
  inline int8_t* GetInfoBits(Table<int8_t>& info_bits, size_t symbol_id,
                             size_t ue_id, size_t cb_id) const {
    return &info_bits[symbol_id]
                     [Roundup<64>(num_bytes_per_cb_) *
                      (ldpc_config_.nblocks_in_symbol_ * ue_id + cb_id)];
  }

  /// Get encoded_buffer for this frame, symbol, user and code block ID
  inline int8_t* GetEncodedBuf(Table<int8_t>& encoded_buffer, size_t frame_id,
                               size_t symbol_id, size_t ue_id,
                               size_t cb_id) const {
    size_t total_data_symbol_id = GetTotalDataSymbolIdxDl(frame_id, symbol_id);
    size_t num_encoded_bytes_per_cb =
        ldpc_config_.cb_codew_len_ / mod_order_bits_;
    return &encoded_buffer[total_data_symbol_id]
                          [Roundup<64>(ofdm_data_num_) * ue_id +
                           num_encoded_bytes_per_cb * cb_id];
  }

  // Returns the number of pilot subcarriers in downlink symbols used for
  // phase tracking
  inline size_t GetOfdmPilotNum() const {
    return ofdm_data_num_ / ofdm_pilot_spacing_;
  }

  Config(std::string /*jsonfile*/);
  void GenData();
  ~Config();
};
#endif
