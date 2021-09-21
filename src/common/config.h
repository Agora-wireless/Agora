// Copyright (c) 2018-2020, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file config.h
 * @brief Declaration file for the configuration class which importants
 * json configuration values into class variables
 */

#ifndef CONFIG_H_
#define CONFIG_H_

#include <emmintrin.h>
#include <immintrin.h>
#include <unistd.h>

#include <boost/range/algorithm/count.hpp>
#include <fstream>  // std::ifstream
#include <iostream>
#include <vector>

#include "buffer.h"
#include "comms-lib.h"
#include "framestats.h"
#include "gettime.h"
#include "ldpc_config.h"
#include "memory_manage.h"
#include "modulation.h"
#include "symbols.h"
#include "utils.h"
#include "utils_ldpc.h"

// Helper classes / structs
namespace AgoraRadio {
struct SdrParameters {
  size_t num_channels_;
  std::string id_;
};

struct HubParameters {
  std::vector<SdrParameters> sdr_;  // 1 or >
  std::string id_;
};

enum class RefNodeType : uint8_t { invalid, internal, external };

struct ReferenceNodeParameters {
  SdrParameters sdr_;
  RefNodeType type_ = RefNodeType::invalid;
  std::string hub_id_;
};

struct CellParameters {
  std::vector<HubParameters> hubs_;
  std::string id_;
  //Tracking variable so we don't have to traverse the SDR list
  size_t cell_antennas_ = 0;
  size_t cell_radios_ = 0;
};
};  // namespace AgoraRadio

class Config {
 public:
  // Constructor
  explicit Config(const std::string& /*jsonfile*/);
  ~Config();

  inline void Running(bool value) { this->running_.store(value); }
  inline bool Running() const { return this->running_.load(); }
  inline size_t BsAntNum() const { return this->bs_total_antennas_; }
  inline void BsAntNum(size_t n_bs_ant) { this->bs_total_antennas_ = n_bs_ant; }

  // Inline accessors (basic types)
  inline bool IsUe() const { return this->is_ue_; }
  inline size_t BfAntNum() const { return this->bs_beamforming_ants_; }

  //inline size_t UeNum() const { return this->ue_num_; }
  inline size_t UeAntInstancCnt() const { return this->ue_ant_instance_cnt_; }
  inline size_t UeAntInsanceOffset() const {
    return this->ue_ant_instance_offset_;
  }

  ///\todo Verify the usage of UeNum here (radios?)
  inline size_t UeNum() const { return this->ue_total_antennas_; }
  inline size_t UeAntTotal() const { return this->ue_total_antennas_; }

  inline size_t OfdmCaNum() const { return this->ofdm_ca_num_; }
  inline size_t CpLen() const { return this->cp_len_; }
  inline size_t OfdmDataNum() const { return this->ofdm_data_num_; }
  inline size_t OfdmDataStart() const { return this->ofdm_data_start_; }

  inline size_t OfdmDataStop() const { return this->ofdm_data_stop_; }
  inline size_t OfdmPilotSpacing() const { return this->ofdm_pilot_spacing_; }
  inline double FreqGhz() const { return this->freq_ghz_; };
  inline size_t DlPacketLength() const { return this->dl_packet_length_; }
  inline std::string Modulation() const { return this->modulation_; }

  inline size_t ModOrderBits() const { return this->mod_order_bits_; }
  inline bool HwFramer() const { return this->hw_framer_; }
  inline double Freq() const { return this->freq_; }
  inline double Rate() const { return this->rate_; }
  inline double Nco() const { return this->nco_; }
  inline bool ScrambleEnabled() const { return this->scramble_enabled_; }

  inline double RadioRfFreq() const { return this->radio_rf_freq_; }
  inline double BwFilter() const { return this->bw_filter_; }
  inline bool SingleGain() const { return this->single_gain_; }
  inline double TxGainA() const { return this->tx_gain_a_; }
  inline double RxGainA() const { return this->rx_gain_a_; }

  inline double TxGainB() const { return this->tx_gain_b_; }
  inline double RxGainB() const { return this->rx_gain_b_; }
  inline double CalibTxGainA() const { return this->calib_tx_gain_a_; }
  inline double CalibTxGainB() const { return this->calib_tx_gain_b_; }
  inline size_t NumCells() const { return this->cells_.size(); }
  inline size_t InitCalibRepeat() const { return this->init_calib_repeat_; }

  inline size_t NumRadios() const {
    if (this->is_ue_) {
      return this->ue_total_radios_;
    } else {
      return this->bs_total_radios_;
    }
  }

  inline size_t NumAntennas() const {
    if (this->is_ue_) {
      return this->ue_total_antennas_;
    } else {
      return this->bs_total_antennas_;
    }
  }

  inline size_t NumChannels() const { return this->num_channels_; }
  inline size_t RefAnt() const { return this->bs_ref_ant_; }
  inline size_t RefRadio() const {
    return this->bs_ref_ant_ / this->num_channels_;
  }
  inline size_t BeaconAnt() const { return this->beacon_ant_; }
  inline size_t BeaconLen() const { return this->beacon_len_; }

  inline bool Beamsweep() const { return this->beamsweep_; }
  inline bool SampleCalEn() const { return this->sample_cal_en_; }
  inline bool ImbalanceCalEn() const { return this->imbalance_cal_en_; }
  inline bool ExternalRefNode() const {
    return this->ref_node_.type_ == AgoraRadio::RefNodeType::external;
  }
  inline std::string Channel() const { return this->channel_list_; }

  inline size_t AntGroupNum() const { return this->ant_group_num_; }
  inline size_t AntPerGroup() const { return this->ant_per_group_; }
  inline size_t RadioGroupNum() const {
    return this->ant_group_num_ / this->num_channels_;
  }
  inline size_t RadioPerGroup() const {
    return this->ant_per_group_ / this->num_channels_;
  }
  inline size_t CoreOffset() const { return this->core_offset_; }
  inline size_t WorkerThreadNum() const { return this->worker_thread_num_; }
  inline size_t SocketThreadNum() const { return this->socket_thread_num_; }

  inline size_t FftThreadNum() const { return this->fft_thread_num_; }
  inline size_t DemulThreadNum() const { return this->demul_thread_num_; }
  inline size_t DecodeThreadNum() const { return this->decode_thread_num_; }
  inline size_t ZfThreadNum() const { return this->zf_thread_num_; }
  inline size_t DemulBlockSize() const { return this->demul_block_size_; }

  inline size_t DemulEventsPerSymbol() const {
    return this->demul_events_per_symbol_;
  }
  inline size_t ZfBlockSize() const { return this->zf_block_size_; }
  inline size_t ZfBatchSize() const { return this->zf_batch_size_; }
  inline size_t ZfEventsPerSymbol() const {
    return this->zf_events_per_symbol_;
  }
  inline size_t FftBlockSize() const { return this->fft_block_size_; }

  inline size_t EncodeBlockSize() const { return this->encode_block_size_; }
  inline bool FreqOrthogonalPilot() const {
    return this->freq_orthogonal_pilot_;
  }
  inline size_t OfdmTxZeroPrefix() const { return this->ofdm_tx_zero_prefix_; }
  inline size_t OfdmTxZeroPostfix() const {
    return this->ofdm_tx_zero_postfix_;
  }
  inline size_t OfdmRxZeroPrefixBs() const {
    return this->ofdm_rx_zero_prefix_bs_;
  }

  inline size_t OfdmRxZeroPrefixCalUl() const {
    return this->ofdm_rx_zero_prefix_cal_ul_;
  }
  inline size_t OfdmRxZeroPrefixCalDl() const {
    return this->ofdm_rx_zero_prefix_cal_dl_;
  }
  inline size_t OfdmRxZeroPrefixClient() const {
    return this->ofdm_rx_zero_prefix_client_;
  }
  inline size_t SampsPerSymbol() const { return this->samps_per_symbol_; }
  inline size_t PacketLength() const { return this->packet_length_; }

  inline float Scale() const { return this->scale_; }
  inline bool BigstationMode() const { return this->bigstation_mode_; }
  inline size_t UlMacDataBytesNumPerframe() const {
    return this->ul_mac_data_bytes_num_perframe_;
  }
  inline size_t UlMacBytesNumPerframe() const {
    return this->ul_mac_bytes_num_perframe_;
  }
  inline size_t DlMacDataBytesNumPerframe() const {
    return this->dl_mac_data_bytes_num_perframe_;
  }
  inline size_t DlMacBytesNumPerframe() const {
    return this->dl_mac_bytes_num_perframe_;
  }

  inline size_t MacPacketLength() const { return this->mac_packet_length_; }
  inline size_t MacPayloadLength() const { return this->mac_payload_length_; }
  inline size_t UlMacPacketsPerframe() const {
    return this->ul_mac_packets_perframe_;
  }
  inline size_t DlMacPacketsPerframe() const {
    return this->dl_mac_packets_perframe_;
  }
  inline std::string UeServerAddr() const { return this->ue_server_addr_; }
  inline std::string BsServerAddr() const { return this->bs_server_addr_; }

  inline std::string BsRruAddr() const { return this->bs_rru_addr_; }
  inline int BsServerPort() const { return this->bs_server_port_; }
  inline int BsRruPort() const { return this->bs_rru_port_; }
  inline int UeServerPort() const { return this->ue_server_port_; }
  inline int UeRruPort() const { return this->ue_rru_port_; }

  inline size_t FramesToTest() const { return this->frames_to_test_; }
  inline float NoiseLevel() const { return this->noise_level_; }
  inline size_t NumBytesPerCb() const { return this->num_bytes_per_cb_; }
  inline bool FftInRru() const { return this->fft_in_rru_; }

  inline uint16_t DpdkNumPorts() const { return this->dpdk_num_ports_; }
  inline uint16_t DpdkPortOffset() const { return this->dpdk_port_offset_; }

  inline size_t BsMacRxPort() const { return this->bs_mac_rx_port_; }
  inline size_t BsMacTxPort() const { return this->bs_mac_tx_port_; }

  inline size_t UeMacRxPort() const { return this->ue_mac_rx_port_; }
  inline size_t UeMacTxPort() const { return this->ue_mac_tx_port_; }

  /* Inline accessors (complex types) */
  inline const std::vector<int>& ClTxAdvance() const {
    return this->cl_tx_advance_;
  }

  inline const LDPCconfig& LdpcConfig() const { return this->ldpc_config_; }
  inline const FrameStats& Frame() const { return this->frame_; }
  inline const std::vector<std::complex<float>>& PilotCf32() const {
    return this->pilot_cf32_;
  };
  inline const std::vector<std::complex<float>>& GoldCf32() const {
    return this->gold_cf32_;
  };
  inline const std::vector<uint32_t>& Coeffs() const { return this->coeffs_; };

  inline const std::vector<uint32_t>& Pilot() const { return this->pilot_; };
  inline const std::vector<uint32_t>& Beacon() const { return this->beacon_; };
  // inline const complex_float *pilots (void ) const { return this->pilots_; };
  inline const complex_float* PilotsSgn() const { return this->pilots_sgn_; };
  inline const std::vector<std::complex<float>>& CommonPilot() const {
    return this->common_pilot_;
  };
  inline const std::vector<double>& ClientGainAdjA() const {
    return this->client_gain_adj_a_;
  };

  inline const std::vector<double>& ClientGainAdjB() const {
    return this->client_gain_adj_b_;
  };

  inline const std::vector<AgoraRadio::CellParameters>& CellDefs() const {
    return this->cells_;
  };

  inline const std::vector<AgoraRadio::SdrParameters>& UserDefs() const {
    return this->users_;
  };

  // non-const (can modify)
  inline Table<complex_float>& UeSpecificPilot() {
    return this->ue_specific_pilot_;
  };
  inline Table<std::complex<int16_t>>& UeSpecificPilotT() {
    return this->ue_specific_pilot_t_;
  };
  inline std::vector<std::complex<int16_t>>& PilotCi16() {
    return this->pilot_ci16_;
  };
  inline std::vector<std::complex<int16_t>>& BeaconCi16() {
    return this->beacon_ci16_;
  };

  inline Table<int8_t>& DlBits() { return this->dl_bits_; }
  inline Table<int8_t>& UlBits() { return this->ul_bits_; }
  inline Table<complex_float>& UlIqF() { return this->ul_iq_f_; }
  inline Table<complex_float>& DlIqF() { return this->dl_iq_f_; }
  inline Table<std::complex<int16_t>>& DlIqT() { return this->dl_iq_t_; }
  inline Table<complex_float>& ModTable() { return this->mod_table_; };

  // Public functions
  void GenData();

  /// TODO document and review
  size_t GetSymbolId(size_t input_id) const;

  bool IsPilot(size_t /*unused*/, size_t /*symbol_id*/) const;
  bool IsCalDlPilot(size_t /*unused*/, size_t /*symbol_id*/) const;
  bool IsCalUlPilot(size_t /*unused*/, size_t /*symbol_id*/) const;
  bool IsDownlink(size_t /*frame_id*/, size_t /*symbol_id*/) const;
  bool IsUplink(size_t /*unused*/, size_t /*symbol_id*/) const;

  /* Public functions that do not meet coding standard format */
  /// Return the symbol type of this symbol in this frame
  SymbolType GetSymbolType(size_t symbol_id) const;

  /* Inline functions */
  inline void UpdateModCfgs(size_t new_mod_order_bits) {
    this->mod_order_bits_ = new_mod_order_bits;
    this->mod_order_ = static_cast<size_t>(pow(2, this->mod_order_bits_));
    InitModulationTable(this->mod_table_, this->mod_order_);
    this->ldpc_config_.NumBlocksInSymbol(
        (this->ofdm_data_num_ * this->mod_order_bits_) /
        this->ldpc_config_.NumCbCodewLen());
  }

  /// Return total number of data symbols of all frames in a buffer
  /// that holds data of kFrameWnd frames
  inline size_t GetTotalDataSymbolIdx(size_t frame_id, size_t symbol_id) const {
    return ((frame_id % kFrameWnd) * this->frame_.NumDataSyms() + symbol_id);
  }

  /// Return total number of uplink data symbols of all frames in a buffer
  /// that holds data of kFrameWnd frames
  inline size_t GetTotalDataSymbolIdxUl(size_t frame_id,
                                        size_t symbol_idx_ul) const {
    return ((frame_id % kFrameWnd) * this->frame_.NumULSyms() + symbol_idx_ul);
  }

  /// Return total number of downlink data symbols of all frames in a buffer
  /// that holds data of kFrameWnd frames
  inline size_t GetTotalDataSymbolIdxDl(size_t frame_id,
                                        size_t symbol_idx_dl) const {
    return ((frame_id % kFrameWnd) * this->frame_.NumDLSyms() + symbol_idx_dl);
  }

  /// Return the frame duration in seconds
  inline double GetFrameDurationSec() const {
    return ((this->frame_.NumTotalSyms() * this->samps_per_symbol_) /
            this->rate_);
  }

  /// Fetch the data buffer for this frame and symbol ID. The symbol must
  /// be an uplink symbol.
  inline complex_float* GetDataBuf(Table<complex_float>& data_buffers,
                                   size_t frame_id, size_t symbol_id) const {
    size_t frame_slot = frame_id % kFrameWnd;
    size_t symbol_offset = (frame_slot * this->frame_.NumULSyms()) +
                           this->frame_.GetULSymbolIdx(symbol_id);
    return data_buffers[symbol_offset];
  }

  /// Return the subcarrier ID to which we should refer to for the zeroforcing
  /// matrices of subcarrier [sc_id].
  inline size_t GetZfScId(size_t sc_id) const {
    return this->freq_orthogonal_pilot_ ? sc_id - (sc_id % ue_total_antennas_)
                                        : sc_id;
  }

  /// Get the calibration buffer for this frame and subcarrier ID
  inline complex_float* GetCalibBuffer(Table<complex_float>& calib_buffer,
                                       size_t frame_id, size_t sc_id) const {
    size_t frame_slot = frame_id % kFrameWnd;
    return &calib_buffer[frame_slot][sc_id * bs_total_antennas_];
  }

  /// Get mac bits for this frame, symbol, user and code block ID
  inline int8_t* GetMacBits(Table<int8_t>& info_bits, size_t frame_id,
                            size_t symbol_id, size_t ue_id,
                            size_t cb_id) const {
    size_t mac_bytes_perframe;
    if (is_ue_ == false) {
      mac_bytes_perframe = dl_mac_bytes_num_perframe_;
    } else {
      mac_bytes_perframe = ul_mac_bytes_num_perframe_;
    }
    return &info_bits[ue_id][(frame_id % kFrameWnd) * mac_bytes_perframe +
                             symbol_id * mac_packet_length_ +
                             cb_id * this->num_bytes_per_cb_];
  }

  /// Get info bits for this symbol, user and code block ID
  inline int8_t* GetInfoBits(Table<int8_t>& info_bits, size_t symbol_id,
                             size_t ue_id, size_t cb_id) const {
    return &info_bits[symbol_id]
                     [Roundup<64>(this->num_bytes_per_cb_) *
                      (ldpc_config_.NumBlocksInSymbol() * ue_id + cb_id)];
  }

  /// Get encoded_buffer for this frame, symbol, user and code block ID
  inline int8_t* GetEncodedBuf(Table<int8_t>& encoded_buffer, size_t frame_id,
                               size_t symbol_id, size_t ue_id,
                               size_t cb_id) const {
    size_t total_data_symbol_id;

    if (is_ue_ == false) {
      total_data_symbol_id = GetTotalDataSymbolIdxDl(frame_id, symbol_id);
    } else {
      total_data_symbol_id = GetTotalDataSymbolIdxUl(frame_id, symbol_id);
    }

    size_t num_encoded_bytes_per_cb =
        ldpc_config_.NumCbCodewLen() / this->mod_order_bits_;

    return &encoded_buffer[total_data_symbol_id]
                          [Roundup<64>(ofdm_data_num_) * ue_id +
                           num_encoded_bytes_per_cb * cb_id];
  }

  // Returns the number of pilot subcarriers in downlink symbols used for
  // phase tracking
  inline size_t GetOFDMPilotNum() const {
    return ofdm_data_num_ / ofdm_pilot_spacing_;
  }

 private:
  /* Class constants */
  inline static const size_t kDefaultSymbolNumPerFrame = 70;
  inline static const size_t kDefaultPilotSymPerFrame = 8;
  inline static const size_t kDefaultULSymPerFrame = 30;
  inline static const size_t kDefaultULSymStart = 9;
  inline static const size_t kDefaultDLSymPerFrame = 30;
  inline static const size_t kDefaultDLSymStart = 40;

  /* Private class variables */
  const double freq_ghz_;  // RDTSC frequency in GHz
  bool is_ue_;

  // The count of ues an instance is responsable for
  //size_t ue_num_;
  // The count of ue antennas an instance is responsable for
  size_t ue_ant_instance_cnt_;
  // The offset into the number total ue antennas this instantiation is
  // responsible for.
  size_t ue_ant_instance_offset_;

  // Total number of us antennas in this experiment including the ones
  // instantiated on other runs/machines.
  size_t ue_total_antennas_;
  size_t ue_total_radios_;

  // The total number of OFDM subcarriers, which is a power of two
  size_t ofdm_ca_num_;

  // The number of cyclic prefix IQ samples. These are taken from the tail of
  // the time-domain OFDM samples and prepended to the beginning.
  size_t cp_len_;

  // The number of OFDM subcarriers that are non-zero in the frequency domain
  size_t ofdm_data_num_;

  // The index of the first non-zero OFDM subcarrier (in the frequency domain)
  // in block of ofdm_ca_num_ subcarriers.
  size_t ofdm_data_start_;

  // The index of the last non-zero OFDM subcarrier (in the frequency domain)
  // in block of ofdm_ca_num_ subcarriers.
  size_t ofdm_data_stop_;

  size_t ofdm_pilot_spacing_;

  LDPCconfig ldpc_config_;  // LDPC parameters

  // A class that holds the frame configuration the id contains letters
  // representing the symbol types in the frame (e.g., 'P' for pilot symbols,
  // 'U' for uplink data symbols)
  FrameStats frame_;

  std::atomic<bool> running_;

  size_t dl_packet_length_;  // HAS_TIME & END_BURST, fixme

  Table<int8_t> dl_bits_;
  Table<int8_t> ul_bits_;
  Table<int8_t> ul_encoded_bits_;
  Table<uint8_t> ul_mod_input_;
  Table<uint8_t> dl_mod_input_;
  Table<complex_float> dl_iq_f_;
  Table<complex_float> ul_iq_f_;
  Table<std::complex<int16_t>> dl_iq_t_;
  Table<std::complex<int16_t>> ul_iq_t_;

  std::vector<std::complex<float>> gold_cf32_;
  std::vector<std::complex<int16_t>> beacon_ci16_;
  std::vector<uint32_t> coeffs_;
  std::vector<std::complex<int16_t>> pilot_ci16_;
  std::vector<std::complex<float>> pilot_cf32_;
  std::vector<uint32_t> pilot_;
  std::vector<uint32_t> beacon_;
  complex_float* pilots_;
  complex_float* pilots_sgn_;
  Table<complex_float> ue_specific_pilot_;
  Table<std::complex<int16_t>> ue_specific_pilot_t_;
  std::vector<std::complex<float>> common_pilot_;

  std::vector<double> client_gain_adj_a_;
  std::vector<double> client_gain_adj_b_;

  std::string modulation_;  // Modulation order as a string, e.g., "16QAM"
  size_t mod_order_;  // Modulation order (e.g., 4: QPSK, 16: 16QAM, 64: 64QAM)
  size_t mod_order_bits_;  // Number of binary bits used for a modulation order

  // Modulation lookup table for mapping binary bits to constellation points
  Table<complex_float> mod_table_;

  //BaseStation Mode
  std::vector<AgoraRadio::CellParameters> cells_;
  //The Reference node will live outside the cell defination (1 reference node)
  AgoraRadio::ReferenceNodeParameters ref_node_;
  // UE mode
  std::vector<AgoraRadio::SdrParameters> users_;

  //Includes all bs SDRs and reference node
  size_t bs_total_radios_;
  size_t bs_beamforming_ants_;
  size_t bs_ref_ant_;
  size_t bs_total_antennas_;

  size_t beacon_ant_;

  size_t num_channels_;
  std::string channel_list_;

  // Controls whether the synchronization and frame time keeping is done
  // in hardware or software
  // true: use hardware correlator; false: use software corrleator
  bool hw_framer_;

  double freq_;
  double rate_;
  double nco_;
  bool scramble_enabled_;
  double radio_rf_freq_;
  double bw_filter_;
  bool single_gain_;
  double tx_gain_a_;
  double rx_gain_a_;
  double tx_gain_b_;
  double rx_gain_b_;
  double calib_tx_gain_a_;
  double calib_tx_gain_b_;

  size_t beacon_len_;
  size_t init_calib_repeat_;
  bool beamsweep_;
  bool sample_cal_en_;
  bool imbalance_cal_en_;
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
  // delays, this can be different from (prefix + cp_len_), and is currently
  // calculated by manual tuning.
  size_t ofdm_rx_zero_prefix_bs_;

  size_t ofdm_rx_zero_prefix_cal_ul_;
  size_t ofdm_rx_zero_prefix_cal_dl_;

  // The number of IQ samples to skip from the beginning of symbol received by
  // Agora on the downlink at the client. Due to over-the-air and RF
  // delays, this can be different from (prefix + cp_len_), and is currently
  // calculated by manual tuning.
  size_t ofdm_rx_zero_prefix_client_;

  // The total number of IQ samples in one physical layer time-domain packet
  // received or sent by Agora
  size_t samps_per_symbol_;

  // The number of bytes in one physical layer time-domain packet received or
  // sent by Agora. This includes Agora's packet header, but not the
  // Ethernet/IP/UDP headers.
  size_t packet_length_;

  std::vector<int> cl_tx_advance_;

  float scale_;  // Scaling factor for all transmit symbols

  bool bigstation_mode_;      // If true, use pipeline-parallel scheduling
  bool correct_phase_shift_;  // If true, do phase shift correction

  // The total number of uncoded data bytes in each OFDM symbol
  size_t data_bytes_num_persymbol_;

  // The total number of uplink MAC payload data bytes in each Frame
  size_t ul_mac_data_bytes_num_perframe_;

  // The total number of uplink MAC packet bytes in each Frame
  size_t ul_mac_bytes_num_perframe_;

  // The total number of downlink MAC payload data bytes in each Frame
  size_t dl_mac_data_bytes_num_perframe_;

  // The total number of downlink MAC packet bytes in each Frame
  size_t dl_mac_bytes_num_perframe_;

  // The length (in bytes) of a MAC packet including the header
  size_t mac_packet_length_;

  // The length (in bytes) of a MAC packet payload
  size_t mac_payload_length_;

  // The total number of downlink mac packets sent/received in each frame
  size_t dl_mac_packets_perframe_;

  // The total number of uplink mac packets sent/received in each frame
  size_t ul_mac_packets_perframe_;

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

  // Offset of the first NIC port used by Agora's DPDK mode
  uint16_t dpdk_port_offset_;

  // Port ID at BaseStation MAC layer side
  size_t bs_mac_rx_port_;
  size_t bs_mac_tx_port_;

  // Port ID at Client MAC layer side
  size_t ue_mac_rx_port_;
  size_t ue_mac_tx_port_;

  // Number of frames_ sent by sender during testing = number of frames_
  // processed by Agora before exiting.
  size_t frames_to_test_;

  // Size of tranport block given by upper layer
  size_t transport_block_size_;

  float noise_level_;

  // Number of bytes per code block
  size_t num_bytes_per_cb_;

  bool fft_in_rru_;  // If true, the RRU does FFT instead of Agora
};
#endif /* CONFIG_HPP_ */
