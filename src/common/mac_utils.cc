#include "mac_utils.h"

#include <utility>

#include "comms-constants.inc"
#include "comms-lib.h"
#include "logger.h"

MacUtils::MacUtils(FrameStats frame)
    : frame_(std::move(frame)),
      ul_ldpc_config_(0, 0, 0, false, 0, 0, 0, 0),
      dl_ldpc_config_(0, 0, 0, false, 0, 0, 0, 0),
      dl_bcast_ldpc_config_(0, 0, 0, false, 0, 0, 0, 0) {}

MacUtils::MacUtils(FrameStats frame, double frame_duration,
                   size_t ul_ofdm_data_num, size_t dl_ofdm_data_num,
                   size_t ctrl_ofdm_data_num)
    : frame_(std::move(frame)),
      frame_duration_(frame_duration),
      ul_ofdm_data_num_(ul_ofdm_data_num),
      dl_ofdm_data_num_(dl_ofdm_data_num),
      ctrl_ofdm_data_num_(ctrl_ofdm_data_num),
      ul_ldpc_config_(0, 0, 0, false, 0, 0, 0, 0),
      dl_ldpc_config_(0, 0, 0, false, 0, 0, 0, 0),
      dl_bcast_ldpc_config_(0, 0, 0, false, 0, 0, 0, 0) {}

/*MacUtils::~MacUtils() {
  ul_mod_table_.Free();
  dl_mod_table_.Free();
}*/

void MacUtils::SetMacParams(const nlohmann::json& ul_mcs_json,
                            const nlohmann::json& dl_mcs_json, bool verbose) {
  this->UpdateUlMcsParams(ul_mcs_json);
  this->UpdateDlMcsParams(dl_mcs_json);
  this->UpdateCtrlMCS();
  if (verbose) {
    this->DumpMcsInfo();
  }
  AGORA_LOG_INFO(
      "UL modulation %s, DL modulation %s, \n"
      "\t%zu UL codeblocks per symbol, %zu UL bytes per code block,\n"
      "\t%zu DL codeblocks per symbol, %zu DL bytes per code block,\n"
      "\t%zu UL MAC data bytes per frame, %zu UL MAC bytes per frame,\n"
      "\t%zu DL MAC data bytes per frame, %zu DL MAC bytes per frame,\n"
      "\tUplink Max Mac data per-user tp (Mbps) %.3f\n"
      "\tDownlink Max Mac data per-user tp (Mbps) %.3f\n",
      ul_modulation_.c_str(), dl_modulation_.c_str(),
      ul_ldpc_config_.NumBlocksInSymbol(), ul_num_bytes_per_cb_,
      dl_ldpc_config_.NumBlocksInSymbol(), dl_num_bytes_per_cb_,
      ul_mac_data_bytes_num_perframe_, ul_mac_bytes_num_perframe_,
      dl_mac_data_bytes_num_perframe_, dl_mac_bytes_num_perframe_,
      (ul_mac_data_bytes_num_perframe_ * 8.0f) / (frame_duration_ * 1e6),
      (dl_mac_data_bytes_num_perframe_ * 8.0f) / (frame_duration_ * 1e6));
}

void MacUtils::UpdateUlMcsParams(const nlohmann::json& ul_mcs_json) {
  this->UpdateUlMCS(ul_mcs_json);
  this->UpdateUlMacParams();
}

void MacUtils::UpdateUlMcsParams(size_t ul_mcs_index) {
  this->UpdateUlMCS(ul_mcs_index);
  this->UpdateUlMacParams();
}

void MacUtils::UpdateDlMcsParams(const nlohmann::json& dl_mcs_json) {
  this->UpdateDlMCS(dl_mcs_json);
  this->UpdateDlMacParams();
}

void MacUtils::UpdateDlMcsParams(size_t dl_mcs_index) {
  this->UpdateDlMCS(dl_mcs_index);
  this->UpdateDlMacParams();
}

void MacUtils::UpdateUlMacParams() {
  ul_num_bytes_per_cb_ =
      (frame_.NumUlDataSyms() > 0) ? (ul_ldpc_config_.NumCbLen() / 8) : 0;
  ul_num_padding_bytes_per_cb_ =
      Roundup<64>(ul_num_bytes_per_cb_) - ul_num_bytes_per_cb_;
  ul_data_bytes_num_persymbol_ =
      ul_num_bytes_per_cb_ * ul_ldpc_config_.NumBlocksInSymbol();
  ul_mac_packet_length_ = ul_data_bytes_num_persymbol_;

  //((cb_len_bits / zc_size) - 1) * (zc_size / 8) + kProcBytes(32)
  const size_t ul_ldpc_input_min =
      (((ul_ldpc_config_.NumCbLen() / ul_ldpc_config_.ExpansionFactor()) - 1) *
           (ul_ldpc_config_.ExpansionFactor() / 8) +
       32);
  const size_t ul_ldpc_sugg_input = LdpcEncodingInputBufSize(
      ul_ldpc_config_.BaseGraph(), ul_ldpc_config_.ExpansionFactor());

  if (ul_ldpc_input_min >
      (ul_num_bytes_per_cb_ + ul_num_padding_bytes_per_cb_)) {
    // Can cause a lot of wasted space, specifically the second argument of the max
    const size_t increased_padding =
        Roundup<64>(ul_ldpc_sugg_input) - ul_num_bytes_per_cb_;

    AGORA_LOG_WARN(
        "LDPC required Input Buffer size exceeds uplink code block size!, "
        "Increased cb padding from %zu to %zu uplink CB Bytes %zu, LDPC "
        "Input Min for zc 64:256: %zu\n",
        ul_num_padding_bytes_per_cb_, increased_padding, ul_num_bytes_per_cb_,
        ul_ldpc_input_min);
    ul_num_padding_bytes_per_cb_ = increased_padding;
  }

  // Smallest over the air packet structure
  RtAssert(this->frame_.NumULSyms() == 0 ||
               ul_mac_packet_length_ > sizeof(MacPacketHeaderPacked),
           "Uplink MAC Packet size must be larger than MAC header size");
  ul_mac_data_length_max_ =
      ul_mac_packet_length_ - sizeof(MacPacketHeaderPacked);

  ul_mac_packets_perframe_ = this->frame_.NumUlDataSyms();
  ul_mac_data_bytes_num_perframe_ =
      ul_mac_data_length_max_ * ul_mac_packets_perframe_;
  ul_mac_bytes_num_perframe_ = ul_mac_packet_length_ * ul_mac_packets_perframe_;
}

void MacUtils::UpdateDlMacParams() {
  dl_num_bytes_per_cb_ =
      (frame_.NumDlDataSyms() > 0) ? (dl_ldpc_config_.NumCbLen() / 8) : 0;
  dl_num_padding_bytes_per_cb_ =
      Roundup<64>(dl_num_bytes_per_cb_) - dl_num_bytes_per_cb_;
  dl_data_bytes_num_persymbol_ =
      dl_num_bytes_per_cb_ * dl_ldpc_config_.NumBlocksInSymbol();
  dl_mac_packet_length_ = dl_data_bytes_num_persymbol_;
  // Smallest over the air packet structure
  RtAssert(this->frame_.NumDLSyms() == 0 ||
               dl_mac_packet_length_ > sizeof(MacPacketHeaderPacked),
           "Downlink MAC Packet size must be larger than MAC header size");
  dl_mac_data_length_max_ =
      dl_mac_packet_length_ - sizeof(MacPacketHeaderPacked);

  dl_mac_packets_perframe_ = this->frame_.NumDlDataSyms();
  dl_mac_data_bytes_num_perframe_ =
      dl_mac_data_length_max_ * dl_mac_packets_perframe_;
  dl_mac_bytes_num_perframe_ = dl_mac_packet_length_ * dl_mac_packets_perframe_;

  //((cb_len_bits / zc_size) - 1) * (zc_size / 8) + kProcBytes(32)
  const size_t dl_ldpc_input_min =
      (((dl_ldpc_config_.NumCbLen() / dl_ldpc_config_.ExpansionFactor()) - 1) *
           (dl_ldpc_config_.ExpansionFactor() / 8) +
       32);
  const size_t dl_ldpc_sugg_input = LdpcEncodingInputBufSize(
      dl_ldpc_config_.BaseGraph(), dl_ldpc_config_.ExpansionFactor());

  if (dl_ldpc_input_min >
      (dl_num_bytes_per_cb_ + dl_num_padding_bytes_per_cb_)) {
    // Can cause a lot of wasted space, specifically the second argument of the max
    const size_t increased_padding =
        Roundup<64>(dl_ldpc_sugg_input) - dl_num_bytes_per_cb_;

    AGORA_LOG_WARN(
        "LDPC required Input Buffer size exceeds downlink code block size!, "
        "Increased cb padding from %zu to %zu Downlink CB Bytes %zu, LDPC "
        "Input Min for zc 64:256: %zu\n",
        dl_num_padding_bytes_per_cb_, increased_padding, dl_num_bytes_per_cb_,
        dl_ldpc_input_min);
    dl_num_padding_bytes_per_cb_ = increased_padding;
  }
}

inline size_t SelectZc(size_t base_graph, size_t code_rate,
                       size_t mod_order_bits, size_t num_sc, size_t cb_per_sym,
                       const std::string& dir) {
  size_t n_zc = sizeof(kZc) / sizeof(size_t);
  std::vector<size_t> zc_vec(kZc, kZc + n_zc);
  std::sort(zc_vec.begin(), zc_vec.end());
  // According to cyclic_shift.cc cyclic shifter for zc
  // larger than 256 has not been implemented, so we skip them here.
  size_t max_zc_index =
      (std::find(zc_vec.begin(), zc_vec.end(), kMaxSupportedZc) -
       zc_vec.begin());
  size_t max_uncoded_bits =
      static_cast<size_t>(num_sc * code_rate * mod_order_bits / 1024.0);
  size_t zc = SIZE_MAX;
  size_t i = 0;
  for (; i < max_zc_index; i++) {
    if ((zc_vec.at(i) * LdpcNumInputCols(base_graph) * cb_per_sym <
         max_uncoded_bits) &&
        (zc_vec.at(i + 1) * LdpcNumInputCols(base_graph) * cb_per_sym >
         max_uncoded_bits)) {
      zc = zc_vec.at(i);
      break;
    }
  }
  if (zc == SIZE_MAX) {
    AGORA_LOG_WARN(
        "Exceeded possible range of LDPC lifting Zc for " + dir +
            "! Setting lifting size to max possible value(%zu).\nThis may lead "
            "to too many unused subcarriers. For better use of the PHY "
            "resources, you may reduce your coding or modulation rate.\n",
        kMaxSupportedZc);
    zc = kMaxSupportedZc;
  }
  return zc;
}

void MacUtils::UpdateUlMCS(const json& ul_mcs) {
  ul_mcs_json_ = ul_mcs;
  size_t ul_mcs_index = SIZE_MAX;
  if (ul_mcs.find("mcs_index") == ul_mcs.end()) {
    auto ul_modulation = ul_mcs.value("modulation", "16QAM");
    auto ul_mod_order_bits = kModulStringMap.at(ul_modulation);

    double ul_code_rate_usr = ul_mcs.value("code_rate", 0.333);
    size_t code_rate_int =
        static_cast<size_t>(std::round(ul_code_rate_usr * 1024.0));

    ul_mcs_index = CommsLib::GetMcsIndex(ul_mod_order_bits, code_rate_int);
    size_t ul_code_rate = GetCodeRate(ul_mcs_index);
    if (ul_code_rate / 1024.0 != ul_code_rate_usr) {
      AGORA_LOG_WARN(
          "Rounded the user-defined uplink code rate to the closest standard "
          "rate %zu/1024.\n",
          ul_code_rate);
    }
  } else {
    // 16QAM, 340/1024
    ul_mcs_index = ul_mcs.value("mcs_index", kDefaultMcsIndex);
  }
  this->UpdateUlMCS(ul_mcs_index);
}

void MacUtils::UpdateUlMCS(size_t ul_mcs_index) {
  ul_mcs_index_ = ul_mcs_index;
  ul_mod_order_bits_ = GetModOrderBits(ul_mcs_index_);
  ul_modulation_ = MapModToStr(ul_mod_order_bits_);
  ul_code_rate_ = GetCodeRate(ul_mcs_index_);
  InitModulationTable(this->ul_mod_table_, ul_mod_order_bits_);

  // TODO: find the optimal base_graph
  const uint16_t base_graph = 1;
  const bool early_term = true;
  const int16_t max_decoder_iter = 5;

  size_t zc = SelectZc(base_graph, ul_code_rate_, ul_mod_order_bits_,
                       ul_ofdm_data_num_, kCbPerSymbol, "uplink");

  // Always positive since ul_code_rate is smaller than 1024
  size_t num_rows =
      static_cast<size_t>(
          std::round(1024.0 * LdpcNumInputCols(base_graph) / ul_code_rate_)) -
      (LdpcNumInputCols(base_graph) - 2);

  uint32_t num_cb_len = LdpcNumInputBits(base_graph, zc);
  uint32_t num_cb_codew_len = LdpcNumEncodedBits(base_graph, zc, num_rows);
  ul_ldpc_config_ = LDPCconfig(base_graph, zc, max_decoder_iter, early_term,
                               num_cb_len, num_cb_codew_len, num_rows, 0);

  ul_ldpc_config_.NumBlocksInSymbol((ul_ofdm_data_num_ * ul_mod_order_bits_) /
                                    ul_ldpc_config_.NumCbCodewLen());
  RtAssert(
      (frame_.NumULSyms() == 0) || (ul_ldpc_config_.NumBlocksInSymbol() > 0),
      "Uplink LDPC expansion factor is too large for number of OFDM data "
      "subcarriers.");
}

void MacUtils::UpdateDlMCS(const json& dl_mcs) {
  dl_mcs_json_ = dl_mcs;
  size_t dl_mcs_index = SIZE_MAX;
  if (dl_mcs.find("mcs_index") == dl_mcs.end()) {
    auto dl_modulation = dl_mcs.value("modulation", "16QAM");
    auto dl_mod_order_bits = kModulStringMap.at(dl_modulation);

    double dl_code_rate_usr = dl_mcs.value("code_rate", 0.333);
    size_t code_rate_int =
        static_cast<size_t>(std::round(dl_code_rate_usr * 1024.0));
    dl_mcs_index = CommsLib::GetMcsIndex(dl_mod_order_bits, code_rate_int);
    size_t dl_code_rate = GetCodeRate(dl_mcs_index);
    if (dl_code_rate / 1024.0 != dl_code_rate_usr) {
      AGORA_LOG_WARN(
          "Rounded the user-defined downlink code rate to the closest standard "
          "rate %zu/1024.\n",
          dl_code_rate);
    }
  } else {
    // 16QAM, 340/1024
    dl_mcs_index = dl_mcs.value("mcs_index", kDefaultMcsIndex);
  }
  this->UpdateDlMCS(dl_mcs_index);
}

void MacUtils::UpdateDlMCS(size_t dl_mcs_index) {
  // 16QAM, 340/1024
  dl_mcs_index_ = dl_mcs_index;
  dl_mod_order_bits_ = GetModOrderBits(dl_mcs_index_);
  dl_modulation_ = MapModToStr(dl_mod_order_bits_);
  dl_code_rate_ = GetCodeRate(dl_mcs_index_);
  InitModulationTable(this->dl_mod_table_, dl_mod_order_bits_);

  // TODO: find the optimal base_graph
  const uint16_t base_graph = 1;
  const bool early_term = true;
  const int16_t max_decoder_iter = 5;

  size_t zc = SelectZc(base_graph, dl_code_rate_, dl_mod_order_bits_,
                       dl_ofdm_data_num_, kCbPerSymbol, "downlink");

  // Always positive since dl_code_rate is smaller than 1024
  size_t num_rows =
      static_cast<size_t>(
          std::round(1024.0 * LdpcNumInputCols(base_graph) / dl_code_rate_)) -
      (LdpcNumInputCols(base_graph) - 2);

  uint32_t num_cb_len = LdpcNumInputBits(base_graph, zc);
  uint32_t num_cb_codew_len = LdpcNumEncodedBits(base_graph, zc, num_rows);
  dl_ldpc_config_ = LDPCconfig(base_graph, zc, max_decoder_iter, early_term,
                               num_cb_len, num_cb_codew_len, num_rows, 0);

  dl_ldpc_config_.NumBlocksInSymbol((dl_ofdm_data_num_ * dl_mod_order_bits_) /
                                    dl_ldpc_config_.NumCbCodewLen());
  RtAssert(
      this->frame_.NumDLSyms() == 0 || dl_ldpc_config_.NumBlocksInSymbol() > 0,
      "Downlink LDPC expansion factor is too large for number of OFDM data "
      "subcarriers.");
}

void MacUtils::UpdateCtrlMCS() {
  if (this->frame_.NumDlControlSyms() > 0) {
    const size_t dl_bcast_mcs_index = kControlMCS;
    const size_t bcast_base_graph =
        1;  // TODO: For MCS < 5, base_graph 1 doesn't work
    dl_bcast_mod_order_bits_ = GetModOrderBits(dl_bcast_mcs_index);
    const size_t dl_bcast_code_rate = GetCodeRate(dl_bcast_mcs_index);
    dl_bcast_modulation_ = MapModToStr(dl_bcast_mod_order_bits_);
    const int16_t max_decoder_iter = 5;
    size_t bcast_zc =
        SelectZc(bcast_base_graph, dl_bcast_code_rate, dl_bcast_mod_order_bits_,
                 ctrl_ofdm_data_num_, kCbPerSymbol, "downlink broadcast");

    // Always positive since dl_code_rate is smaller than 1
    size_t bcast_num_rows =
        static_cast<size_t>(std::round(
            1024.0 * LdpcNumInputCols(bcast_base_graph) / dl_bcast_code_rate)) -
        (LdpcNumInputCols(bcast_base_graph) - 2);

    uint32_t bcast_num_cb_len = LdpcNumInputBits(bcast_base_graph, bcast_zc);
    uint32_t bcast_num_cb_codew_len =
        LdpcNumEncodedBits(bcast_base_graph, bcast_zc, bcast_num_rows);
    dl_bcast_ldpc_config_ =
        LDPCconfig(bcast_base_graph, bcast_zc, max_decoder_iter, true,
                   bcast_num_cb_len, bcast_num_cb_codew_len, bcast_num_rows, 0);

    dl_bcast_ldpc_config_.NumBlocksInSymbol(
        (ctrl_ofdm_data_num_ * dl_bcast_mod_order_bits_) /
        dl_bcast_ldpc_config_.NumCbCodewLen());
    RtAssert(dl_bcast_ldpc_config_.NumBlocksInSymbol() > 0,
             "Downlink Broadcast LDPC expansion factor is too large for number "
             "of OFDM data "
             "subcarriers.");
  }
}

void MacUtils::DumpMcsInfo() {
  AGORA_LOG_INFO(
      "Uplink MCS Info: LDPC: Zc: %d, %zu code blocks per symbol, %d "
      "information "
      "bits per encoding, %d bits per encoded code word, decoder "
      "iterations: %d, code rate %.3f (nRows = %zu), modulation %s\n",
      ul_ldpc_config_.ExpansionFactor(), ul_ldpc_config_.NumBlocksInSymbol(),
      ul_ldpc_config_.NumCbLen(), ul_ldpc_config_.NumCbCodewLen(),
      ul_ldpc_config_.MaxDecoderIter(),
      1.f * LdpcNumInputCols(ul_ldpc_config_.BaseGraph()) /
          (LdpcNumInputCols(ul_ldpc_config_.BaseGraph()) - 2 +
           ul_ldpc_config_.NumRows()),
      ul_ldpc_config_.NumRows(), ul_modulation_.c_str());
  AGORA_LOG_INFO(
      "Downlink MCS Info: LDPC: Zc: %d, %zu code blocks per symbol, %d "
      "information "
      "bits per encoding, %d bits per encoded code word, decoder "
      "iterations: %d, code rate %.3f (nRows = %zu), modulation %s\n",
      dl_ldpc_config_.ExpansionFactor(), dl_ldpc_config_.NumBlocksInSymbol(),
      dl_ldpc_config_.NumCbLen(), dl_ldpc_config_.NumCbCodewLen(),
      dl_ldpc_config_.MaxDecoderIter(),
      1.f * LdpcNumInputCols(dl_ldpc_config_.BaseGraph()) /
          (LdpcNumInputCols(dl_ldpc_config_.BaseGraph()) - 2 +
           dl_ldpc_config_.NumRows()),
      dl_ldpc_config_.NumRows(), dl_modulation_.c_str());
  AGORA_LOG_INFO(
      "Downlink Broadcast MCS Info: LDPC: Zc: %d, %zu code blocks per "
      "symbol, "
      "%d "
      "information "
      "bits per encoding, %d bits per encoded code word, decoder "
      "iterations: %d, code rate %.3f (nRows = %zu), modulation %s\n",
      dl_bcast_ldpc_config_.ExpansionFactor(),
      dl_bcast_ldpc_config_.NumBlocksInSymbol(),
      dl_bcast_ldpc_config_.NumCbLen(), dl_bcast_ldpc_config_.NumCbCodewLen(),
      dl_bcast_ldpc_config_.MaxDecoderIter(),
      1.f * LdpcNumInputCols(dl_bcast_ldpc_config_.BaseGraph()) /
          (LdpcNumInputCols(dl_bcast_ldpc_config_.BaseGraph()) - 2 +
           dl_bcast_ldpc_config_.NumRows()),
      dl_bcast_ldpc_config_.NumRows(), dl_bcast_modulation_.c_str());
}
