#ifndef MAC_UTILS_H_
#define MAC_UTILS_H_

#include "comms-lib.h"
#include "framestats.h"
#include "ldpc_config.h"
#include "modulation.h"
#include "nlohmann/json.hpp"
#include "phy_ldpc_decoder_5gnr.h"
#include "utils.h"
#include "utils_ldpc.h"

using json = nlohmann::json;
static constexpr size_t kMaxSupportedZc = 256;
//static constexpr size_t kVarNodesSize = 1024 * 1024 * sizeof(int16_t);
static constexpr size_t kControlMCS = 5;  // QPSK, 379/1024
class MacUtils {
 public:
  explicit MacUtils(FrameStats frame);
  MacUtils(FrameStats frame, double frame_duration, size_t ul_ofdm_data_num,
           size_t dl_ofdm_data_num, size_t ctrl_ofdm_data_num);

  void SetMacParams(const nlohmann::json& ul_mcs_json,
                    const nlohmann::json& dl_mcs_json, bool verbose = false);
  void UpdateUlMcsParams(const nlohmann::json& ul_mcs_json);
  void UpdateUlMcsParams(size_t ul_mcs_index);
  void UpdateDlMcsParams(const nlohmann::json& dl_mcs_json);
  void UpdateDlMcsParams(size_t dl_mcs_index);
  void UpdateUlMacParams();
  void UpdateDlMacParams();
  inline nlohmann::json GetMcsJson(Direction dir) {
    return dir == Direction::kUplink ? this->ul_mcs_json_ : this->dl_mcs_json_;
  }
  inline std::string Modulation(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_modulation_
                                     : this->dl_modulation_;
  }
  inline size_t ModOrderBits(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_mod_order_bits_
                                     : this->dl_mod_order_bits_;
  }
  inline size_t NumBytesPerCb(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_num_bytes_per_cb_
                                     : this->dl_num_bytes_per_cb_;
  }
  inline size_t NumPaddingBytesPerCb(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_num_padding_bytes_per_cb_
                                     : this->dl_num_padding_bytes_per_cb_;
  }
  inline size_t NumBytesPerSymbol(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_data_bytes_num_persymbol_
                                     : this->dl_data_bytes_num_persymbol_;
  }
  inline size_t MacDataBytesNumPerframe(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_mac_data_bytes_num_perframe_
                                     : this->dl_mac_data_bytes_num_perframe_;
  }
  inline size_t MacBytesNumPerframe(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_mac_bytes_num_perframe_
                                     : this->dl_mac_bytes_num_perframe_;
  }

  inline size_t MacPacketLength(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_mac_packet_length_
                                     : this->dl_mac_packet_length_;
  }
  inline size_t MacPayloadMaxLength(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_mac_data_length_max_
                                     : this->dl_mac_data_length_max_;
  }
  inline size_t MacPacketsPerframe(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_mac_packets_perframe_
                                     : this->dl_mac_packets_perframe_;
  }
  inline const LDPCconfig& LdpcConfig(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_ldpc_config_
                                     : this->dl_ldpc_config_;
  }
  inline const LDPCconfig& BcLdpcConfig() const {
    return dl_bcast_ldpc_config_;
  }
  inline size_t BcModOrderBits() const {
    return this->dl_bcast_mod_order_bits_;
  }
  inline const Table<complex_float>& ModTable(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_mod_table_
                                     : this->dl_mod_table_;
  }
  inline size_t SubcarrierPerCodeBlock(Direction dir) const {
    return this->LdpcConfig(dir).NumCbCodewLen() / this->ModOrderBits(dir);
  }
  inline size_t McsIndex(Direction dir) const {
    return dir == Direction::kUplink ? this->ul_mcs_index_
                                     : this->dl_mcs_index_;
  }

  inline size_t MaxPacketBytes(Direction dir) const {
    size_t max_mod_bits = GetModOrderBits(kMaxMcsIndex);
    size_t max_code_rate = GetCodeRate(kMaxMcsIndex);
    size_t num_sc =
        (dir == Direction::kUplink) ? ul_ofdm_data_num_ : dl_ofdm_data_num_;
    return (static_cast<size_t>(num_sc * max_code_rate * max_mod_bits /
                                1024.0) +
            7) /
           8;
  }
  /// Get mac bits for this frame, symbol, user and code block ID
  inline int8_t* GetMacBits(Table<int8_t>& info_bits, Direction dir,
                            size_t frame_id, size_t symbol_id, size_t ue_id,
                            size_t cb_id) const {
    size_t mac_offset_perframe;
    size_t num_bytes_per_cb;
    size_t max_packet_length;
    if (dir == Direction::kDownlink) {
      num_bytes_per_cb = this->dl_num_bytes_per_cb_;
      max_packet_length = this->MaxPacketBytes(Direction::kDownlink);
      mac_offset_perframe = this->dl_mac_packets_perframe_ * max_packet_length;
    } else {
      num_bytes_per_cb = this->ul_num_bytes_per_cb_;
      max_packet_length = this->MaxPacketBytes(Direction::kUplink);
      mac_offset_perframe = this->ul_mac_packets_perframe_ * max_packet_length;
    }
    return &info_bits[ue_id][(frame_id % kFrameWnd) * mac_offset_perframe +
                             symbol_id * max_packet_length +
                             cb_id * num_bytes_per_cb];
  }

 private:
  nlohmann::json Parse(const nlohmann::json& in_json,
                       const std::string& json_handle);
  void UpdateUlMCS(const json& ul_mcs);
  void UpdateUlMCS(size_t ul_mcs_index);
  void UpdateDlMCS(const json& dl_mcs);
  void UpdateDlMCS(size_t dl_mcs_index);
  void UpdateCtrlMCS();
  void DumpMcsInfo();
  // Number of code blocks per OFDM symbol
  // Temporarily set to 1
  // TODO: This number should independent of OFDM symbols
  static constexpr size_t kCbPerSymbol = 1;

  FrameStats frame_;
  double frame_duration_;
  size_t ul_ofdm_data_num_;
  size_t dl_ofdm_data_num_;
  size_t ctrl_ofdm_data_num_;

  nlohmann::json ul_mcs_json_;
  nlohmann::json dl_mcs_json_;

  std::string ul_modulation_;  // Modulation order as a string, e.g., "16QAM"
  size_t
      ul_mod_order_bits_;  // Number of binary bits used for a modulation order
  std::string dl_modulation_;
  size_t dl_mod_order_bits_;
  std::string dl_bcast_modulation_;
  size_t dl_bcast_mod_order_bits_;

  // Modulation lookup table for mapping binary bits to constellation points
  Table<complex_float> ul_mod_table_;
  Table<complex_float> dl_mod_table_;

  LDPCconfig ul_ldpc_config_;        // Uplink LDPC parameters
  LDPCconfig dl_ldpc_config_;        // Downlink LDPC parameters
  LDPCconfig dl_bcast_ldpc_config_;  // Downlink Broadcast LDPC parameters
  size_t ul_mcs_index_;
  size_t dl_mcs_index_;
  size_t dl_code_rate_;
  size_t ul_code_rate_;

  // The total number of uncoded uplink data bytes in each OFDM symbol
  size_t ul_data_bytes_num_persymbol_;

  // The total number of uplink MAC payload data bytes in each Frame
  size_t ul_mac_data_bytes_num_perframe_;

  // The total number of uplink MAC packet bytes in each Frame
  size_t ul_mac_bytes_num_perframe_;

  // The length (in bytes) of a uplink MAC packet including the header
  size_t ul_mac_packet_length_;

  // The length (in bytes) of a uplink MAC packet payload (data)
  size_t ul_mac_data_length_max_;

  // The total number of uncoded downlink data bytes in each OFDM symbol
  size_t dl_data_bytes_num_persymbol_;

  // The total number of downlink MAC payload data bytes in each Frame
  size_t dl_mac_data_bytes_num_perframe_;

  // The total number of downlink MAC packet bytes in each Frame
  size_t dl_mac_bytes_num_perframe_;

  // The length (in bytes) of a downlink MAC packet including the header
  size_t dl_mac_packet_length_;

  // The length (in bytes) of a downlink MAC packet payload (data)
  size_t dl_mac_data_length_max_;

  // The total number of downlink mac packets sent/received in each frame
  size_t dl_mac_packets_perframe_;

  // The total number of uplink mac packets sent/received in each frame
  size_t ul_mac_packets_perframe_;
  // Number of bytes per code block
  size_t ul_num_bytes_per_cb_;
  size_t dl_num_bytes_per_cb_;

  // Number of padding bytes per code block
  size_t ul_num_padding_bytes_per_cb_;
  size_t dl_num_padding_bytes_per_cb_;
};
#endif /* MAC_UTILS_H_ */
