// Copyright (c) 2018-2022, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file config.cc
 * @brief Implementation file for the configuration class which importants
 * json configuration values into class variables
 */

#include "config.h"

#include <boost/range/algorithm/count.hpp>

#include "logger.h"
#include "nlohmann/json.hpp"
#include "scrambler.h"
#include "utils_ldpc.h"

using json = nlohmann::json;

static const size_t kMacAlignmentBytes = 64u;
static constexpr bool kDebugPrintConfiguration = false;

Config::Config(const std::string& jsonfile)
    : freq_ghz_(GetTime::MeasureRdtscFreq()),
      ldpc_config_(0, 0, 0, false, 0, 0, 0, 0),
      frame_("") {
  pilots_ = nullptr;
  pilots_sgn_ = nullptr;

  std::string conf;
  Utils::LoadTddConfig(jsonfile, conf);
  // Allow json comments
  const auto tdd_conf = json::parse(conf, nullptr, true, true);

  // Initialize the compute configuration
  // Default exclude 1 core with id = 0
  std::vector<size_t> excluded(1, 0);
  if (tdd_conf.contains("exclude_cores")) {
    auto exclude_cores = tdd_conf.at("exclude_cores");
    excluded.resize(exclude_cores.size());
    for (size_t i = 0; i < exclude_cores.size(); i++) {
      excluded.at(i) = exclude_cores.at(i);
    }
  }
  SetCpuLayoutOnNumaNodes(true, excluded);

  num_cells_ = tdd_conf.value("cells", 1);
  num_radios_ = 0;
  ue_num_ = 0;

  std::string serials_str;
  std::string serial_file = tdd_conf.value("serial_file", "");
  Utils::LoadTddConfig(serial_file, serials_str);
  if (serials_str.empty() == false) {
    const auto j_serials = json::parse(serials_str, nullptr, true, true);

    std::stringstream ss;
    json j_bs_serials;
    ss << j_serials.value("BaseStations", j_bs_serials);
    j_bs_serials = json::parse(ss);
    ss.str(std::string());
    ss.clear();

    RtAssert(j_bs_serials.size() == num_cells_, "Incorrect cells number!");
    external_ref_node_.resize(num_cells_, false);
    for (size_t i = 0; i < num_cells_; i++) {
      json serials_conf;
      std::string cell_str = "BS" + std::to_string(i);
      ss << j_bs_serials.value(cell_str, serials_conf);
      serials_conf = json::parse(ss);
      ss.str(std::string());
      ss.clear();

      auto hub_serial = serials_conf.value("hub", "");
      hub_id_.push_back(hub_serial);
      auto sdr_serials = serials_conf.value("sdr", json::array());
      RtAssert(!sdr_serials.empty(), "BS has zero sdrs!");
      radio_id_.insert(radio_id_.end(), sdr_serials.begin(), sdr_serials.end());
      num_radios_ += sdr_serials.size();
      cell_id_.resize(num_radios_, i);

      auto refnode_serial = serials_conf.value("reference", "");
      if (refnode_serial.empty()) {
        MLPD_INFO(
            "No reference node ID found in topology file! Taking the last node "
            "%s as reference node!\n",
            radio_id_.back().c_str());
        refnode_serial = radio_id_.back();
        ref_radio_.push_back(radio_id_.size() - 1);
      } else {
        auto serial_iterator =
            std::find(sdr_serials.begin(), sdr_serials.end(), refnode_serial);
        if (serial_iterator == sdr_serials.end()) {
          radio_id_.push_back(refnode_serial);
          ref_radio_.push_back(radio_id_.size() - 1);
          num_radios_++;
          cell_id_.resize(num_radios_, i);
          external_ref_node_.at(i) = true;
        } else {
          size_t index = radio_id_.size() - sdr_serials.size() +
                         serial_iterator - sdr_serials.begin();
          ref_radio_.push_back(index);
        }
      }
    }

    json j_ue_serials;
    ss << j_serials.value("Clients", j_ue_serials);
    j_ue_serials = json::parse(ss);
    ss.str(std::string());
    ss.clear();

    auto ue_serials = j_ue_serials.value("sdr", json::array());
    ue_radio_id_.assign(ue_serials.begin(), ue_serials.end());
  }

  if (radio_id_.empty() == true) {
    num_radios_ = tdd_conf.value("bs_radio_num", 8);
    external_ref_node_.resize(num_cells_, false);
    cell_id_.resize(num_radios_, 0);
  }

  if (ue_radio_id_.empty() == false) {
    ue_num_ = ue_radio_id_.size();
  } else {
    ue_num_ = tdd_conf.value("ue_radio_num", 8);
  }

  channel_ = tdd_conf.value("channel", "A");
  ue_channel_ = tdd_conf.value("ue_channel", channel_);
  num_channels_ = std::min(channel_.size(), kMaxChannels);
  num_ue_channels_ = std::min(ue_channel_.size(), kMaxChannels);
  bs_ant_num_ = num_channels_ * num_radios_;
  ue_ant_num_ = ue_num_ * num_ue_channels_;

  bf_ant_num_ = bs_ant_num_;
  for (size_t i = 0; i < num_cells_; i++) {
    if (external_ref_node_.at(i) == true) {
      bf_ant_num_ = bs_ant_num_ - num_channels_;
    }
  }

  if (ref_radio_.empty() == false) {
    for (size_t i = 0; i < num_cells_; i++) {
      ref_ant_.push_back(ref_radio_.at(i) * num_channels_);
    }
  }

  if ((kUseArgos == true) || (kUseUHD == true)) {
    RtAssert(num_radios_ != 0, "Error: No radios exist in Argos mode");
  }

  /* radio configurations */
  freq_ = tdd_conf.value("frequency", 3.6e9);
  single_gain_ = tdd_conf.value("single_gain", true);
  tx_gain_a_ = tdd_conf.value("tx_gain_a", 20);
  rx_gain_a_ = tdd_conf.value("rx_gain_a", 20);
  tx_gain_b_ = tdd_conf.value("tx_gain_b", 20);
  rx_gain_b_ = tdd_conf.value("rx_gain_b", 20);
  calib_tx_gain_a_ = tdd_conf.value("calib_tx_gain_a", tx_gain_a_);
  calib_tx_gain_b_ = tdd_conf.value("calib_tx_gain_b", tx_gain_b_);

  auto gain_tx_json_a = tdd_conf.value("ue_tx_gain_a", json::array());
  if (gain_tx_json_a.empty()) {
    client_tx_gain_a_.resize(ue_num_, 20);
  } else {
    RtAssert(gain_tx_json_a.size() == ue_num_,
             "ue_tx_gain_a size must be same as the number of clients!");
    client_tx_gain_a_.assign(gain_tx_json_a.begin(), gain_tx_json_a.end());
  }
  auto gain_tx_json_b = tdd_conf.value("ue_tx_gain_b", json::array());
  if (gain_tx_json_b.empty()) {
    client_tx_gain_b_.resize(ue_num_, 0);
  } else {
    RtAssert(gain_tx_json_b.size() == ue_num_,
             "ue_tx_gain_b size must be same as the number of clients!");
    client_tx_gain_b_.assign(gain_tx_json_b.begin(), gain_tx_json_b.end());
  }
  auto gain_rx_json_a = tdd_conf.value("ue_rx_gain_a", json::array());
  if (gain_rx_json_a.empty()) {
    client_rx_gain_a_.resize(ue_num_, 20);
  } else {
    RtAssert(gain_rx_json_a.size() == ue_num_,
             "ue_rx_gain_a size must be same as the number of clients!");
    client_rx_gain_a_.assign(gain_rx_json_a.begin(), gain_rx_json_a.end());
  }
  auto gain_rx_json_b = tdd_conf.value("ue_rx_gain_b", json::array());
  if (gain_rx_json_b.empty()) {
    client_rx_gain_b_.resize(ue_num_, 0);
  } else {
    RtAssert(gain_rx_json_b.size() == ue_num_,
             "ue_rx_gain_b size must be same as the number of clients!");
    client_rx_gain_b_.assign(gain_rx_json_b.begin(), gain_rx_json_b.end());
  }

  rate_ = tdd_conf.value("sample_rate", 5e6);
  nco_ = tdd_conf.value("nco_frequency", 0.75 * rate_);
  bw_filter_ = rate_ + 2 * nco_;
  radio_rf_freq_ = freq_ - nco_;
  beacon_ant_ = tdd_conf.value("beacon_antenna", 0);
  beamsweep_ = tdd_conf.value("beamsweep", false);
  sample_cal_en_ = tdd_conf.value("sample_calibrate", false);
  imbalance_cal_en_ = tdd_conf.value("imbalance_calibrate", false);
  init_calib_repeat_ = tdd_conf.value("init_calib_repeat", 0);

  modulation_ = tdd_conf.value("modulation", "16QAM");

  bs_server_addr_ = tdd_conf.value("bs_server_addr", "127.0.0.1");
  bs_rru_addr_ = tdd_conf.value("bs_rru_addr", "127.0.0.1");
  ue_server_addr_ = tdd_conf.value("ue_server_addr", "127.0.0.1");
  mac_remote_addr_ = tdd_conf.value("mac_remote_addr", "127.0.0.1");
  bs_server_port_ = tdd_conf.value("bs_server_port", 8000);
  bs_rru_port_ = tdd_conf.value("bs_rru_port", 9000);
  ue_rru_port_ = tdd_conf.value("ue_rru_port", 7000);
  ue_server_port_ = tdd_conf.value("ue_server_port", 6000);

  dpdk_num_ports_ = tdd_conf.value("dpdk_num_ports", 1);
  dpdk_port_offset_ = tdd_conf.value("dpdk_port_offset", 0);

  ue_mac_tx_port_ = tdd_conf.value("ue_mac_tx_port", kMacUserRemotePort);
  ue_mac_rx_port_ = tdd_conf.value("ue_mac_rx_port", kMacUserLocalPort);
  bs_mac_tx_port_ = tdd_conf.value("bs_mac_tx_port", kMacBaseRemotePort);
  bs_mac_rx_port_ = tdd_conf.value("bs_mac_rx_port", kMacBaseLocalPort);

  /* frame configurations */
  cp_len_ = tdd_conf.value("cp_size", 0);
  ofdm_ca_num_ = tdd_conf.value("fft_size", 2048);
  ofdm_data_num_ = tdd_conf.value("ofdm_data_num", 1200);
  ofdm_tx_zero_prefix_ = tdd_conf.value("ofdm_tx_zero_prefix", 0);
  ofdm_tx_zero_postfix_ = tdd_conf.value("ofdm_tx_zero_postfix", 0);
  ofdm_rx_zero_prefix_bs_ =
      tdd_conf.value("ofdm_rx_zero_prefix_bs", 0) + cp_len_;
  ofdm_rx_zero_prefix_client_ = tdd_conf.value("ofdm_rx_zero_prefix_client", 0);
  ofdm_rx_zero_prefix_cal_ul_ =
      tdd_conf.value("ofdm_rx_zero_prefix_cal_ul", 0) + cp_len_;
  ofdm_rx_zero_prefix_cal_dl_ =
      tdd_conf.value("ofdm_rx_zero_prefix_cal_dl", 0) + cp_len_;
  RtAssert(ofdm_data_num_ % kSCsPerCacheline == 0,
           "ofdm_data_num must be a multiple of subcarriers per cacheline");
  RtAssert(ofdm_data_num_ % kTransposeBlockSize == 0,
           "Transpose block size must divide number of OFDM data subcarriers");
  ofdm_pilot_spacing_ = tdd_conf.value("ofdm_pilot_spacing", 16);
  ofdm_data_start_ =
      tdd_conf.value("ofdm_data_start", (ofdm_ca_num_ - ofdm_data_num_) / 2);
  ofdm_data_stop_ = ofdm_data_start_ + ofdm_data_num_;

  bigstation_mode_ = tdd_conf.value("bigstation_mode", false);
  freq_orthogonal_pilot_ = tdd_conf.value("freq_orthogonal_pilot", false);
  correct_phase_shift_ = tdd_conf.value("correct_phase_shift", false);

  hw_framer_ = tdd_conf.value("hw_framer", true);
  if (kUseUHD) {
    hw_framer_ = false;
  } else {
    RtAssert(hw_framer_ == true,
             "Base Station hardware framer (hw_framer) set to false is "
             "unsupported in this version of Agora");
  }
  ue_hw_framer_ = tdd_conf.value("ue_hw_framer", false);
  RtAssert(ue_hw_framer_ == false,
           "User equiptment hardware framer (ue_hw_framer) set to true is "
           "unsupported in this version of Agora");

  // If frames not specified explicitly, construct default based on frame_type /
  // symbol_num_perframe / pilot_num / ul_symbol_num_perframe /
  // dl_symbol_num_perframe / dl_data_symbol_start
  if (tdd_conf.find("frame_schedule") == tdd_conf.end()) {
    size_t ul_data_symbol_num_perframe = kDefaultULSymPerFrame;
    size_t ul_data_symbol_start = kDefaultULSymStart;
    size_t dl_data_symbol_num_perframe = kDefaultDLSymPerFrame;
    size_t dl_data_symbol_start = kDefaultDLSymStart;

    size_t symbol_num_perframe =
        tdd_conf.value("symbol_num_perframe", kDefaultSymbolNumPerFrame);
    size_t pilot_symbol_num_perframe = tdd_conf.value(
        "pilot_num",
        freq_orthogonal_pilot_ ? kDefaultPilotSymPerFrame : ue_ant_num_);

    size_t beacon_symbol_position = tdd_conf.value("beacon_position", SIZE_MAX);

    // Start position of the first UL symbol
    ul_data_symbol_start =
        tdd_conf.value("ul_data_symbol_start", ul_data_symbol_start);
    ul_data_symbol_num_perframe =
        tdd_conf.value("ul_symbol_num_perframe", ul_data_symbol_num_perframe);

    // Start position of the first DL symbol
    dl_data_symbol_start =
        tdd_conf.value("dl_data_symbol_start", dl_data_symbol_start);
    dl_data_symbol_num_perframe =
        tdd_conf.value("dl_symbol_num_perframe", dl_data_symbol_num_perframe);

    size_t ul_data_symbol_stop =
        ul_data_symbol_start + ul_data_symbol_num_perframe;
    size_t dl_data_symbol_stop =
        dl_data_symbol_start + dl_data_symbol_num_perframe;

    if ((ul_data_symbol_num_perframe + dl_data_symbol_num_perframe +
         pilot_symbol_num_perframe) > symbol_num_perframe) {
      MLPD_ERROR(
          "!!!!! Invalid configuration pilot + ul + dl exceeds total symbols "
          "!!!!!\n");
      MLPD_ERROR(
          "Uplink symbols: %zu, Downlink Symbols :%zu, Pilot Symbols: %zu, "
          "Total Symbols: %zu\n",
          ul_data_symbol_num_perframe, dl_data_symbol_num_perframe,
          pilot_symbol_num_perframe, symbol_num_perframe);
      throw std::runtime_error("Invalid Frame Configuration");
    } else if (((ul_data_symbol_num_perframe > 0) &&
                (dl_data_symbol_num_perframe > 0)) &&
               (((ul_data_symbol_start >= dl_data_symbol_start) &&
                 (ul_data_symbol_start < dl_data_symbol_stop)) ||
                ((ul_data_symbol_stop > dl_data_symbol_start) &&
                 (ul_data_symbol_stop <= dl_data_symbol_stop)))) {
      MLPD_ERROR(
          "!!!!! Invalid configuration ul and dl symbol overlap detected "
          "!!!!!\n");
      MLPD_ERROR(
          "Uplink - start: %zu - stop :%zu, Downlink - start: %zu - stop %zu\n",
          ul_data_symbol_start, ul_data_symbol_stop, dl_data_symbol_start,
          dl_data_symbol_stop);
      throw std::runtime_error("Invalid Frame Configuration");
    }

    char first_sym;
    char second_sym;
    size_t first_sym_start;
    size_t first_sym_count;
    size_t second_sym_start;
    size_t second_sym_count;
    if (dl_data_symbol_start <= ul_data_symbol_start) {
      first_sym = 'D';
      first_sym_start = dl_data_symbol_start;
      first_sym_count = dl_data_symbol_num_perframe;
      second_sym = 'U';
      second_sym_start = ul_data_symbol_start;
      second_sym_count = ul_data_symbol_num_perframe;
    } else {
      first_sym = 'U';
      first_sym_start = ul_data_symbol_start;
      first_sym_count = ul_data_symbol_num_perframe;
      second_sym = 'D';
      second_sym_start = dl_data_symbol_start;
      second_sym_count = dl_data_symbol_num_perframe;
    }
    MLPD_SYMBOL(
        "Symbol %c, start %zu, count %zu. Symbol %c, start %zu, count %zu. "
        "Total Symbols: %zu\n",
        first_sym, first_sym_start, first_sym_start, second_sym,
        second_sym_start, second_sym_start, symbol_num_perframe);

    std::string sched = "";
    // Offset the pilots, if the beacon comes first
    if (beacon_symbol_position == 0) {
      sched = "G";
    }
    sched.append(pilot_symbol_num_perframe, 'P');
    // Could roll this up into a loop but will leave like this for readability
    int add_symbols = 0;
    // ( )PGGGG1111111111GGGG2222222222GGGG
    if (first_sym_start > 0) {
      add_symbols = first_sym_start - sched.length();
      assert(add_symbols >= 0);
      sched.append(first_sym_start - sched.length(), 'G');
      sched.append(first_sym_count, first_sym);
    }

    if (second_sym_start > 0) {
      add_symbols = second_sym_start - sched.length();
      assert(add_symbols >= 0);
      sched.append(add_symbols, 'G');
      sched.append(second_sym_count, second_sym);
    }
    add_symbols = symbol_num_perframe - sched.length();
    assert(add_symbols >= 0);
    sched.append(add_symbols, 'G');

    // Add the beacon
    if (beacon_symbol_position < sched.length()) {
      if (sched.at(beacon_symbol_position) != 'G') {
        MLPD_ERROR("Invalid beacon location %zu replacing %c\n",
                   beacon_symbol_position, sched.at(beacon_symbol_position));
        throw std::runtime_error("Invalid Frame Configuration");
      }
      sched.replace(beacon_symbol_position, 1, "B");
    }

    frame_ = FrameStats(sched);
  } else {
    json jframes = tdd_conf.value("frame_schedule", json::array());

    // Only allow 1 unique frame type
    assert(jframes.size() == 1);
    frame_ = FrameStats(jframes.at(0).get<std::string>());
  }
  MLPD_INFO("Config: Frame schedule %s (%zu symbols)\n",
            frame_.FrameIdentifier().c_str(), frame_.NumTotalSyms());

  // Check for frame validity.
  // We should remove the restriction of the beacon symbol placement when tested
  // more thoroughly
  if (((frame_.NumBeaconSyms() > 1)) ||
      ((frame_.NumBeaconSyms() == 1) && (frame_.GetBeaconSymbolLast() > 1))) {
    MLPD_ERROR("Invalid beacon symbol placement\n");
    throw std::runtime_error("Invalid beacon symbol placement");
  }

  // client_dl_pilot_sym uses the first x 'D' symbols for downlink channel
  // estimation for each user.
  size_t client_dl_pilot_syms = tdd_conf.value("client_dl_pilot_syms", 0);
  // client_ul_pilot_sym uses the first x 'U' symbols for downlink channel
  // estimation for each user.
  size_t client_ul_pilot_syms = tdd_conf.value("client_ul_pilot_syms", 0);

  frame_.SetClientPilotSyms(client_ul_pilot_syms, client_dl_pilot_syms);

  ant_per_group_ = frame_.NumDLCalSyms();
  RtAssert(ant_per_group_ % num_channels_ == 0,
           "Number of Downlink calibration symbols per frame must be "
           "multiplier of number of channels!");
  ant_group_num_ =
      frame_.IsRecCalEnabled() ? (bf_ant_num_ / ant_per_group_) : 0;

  if ((freq_orthogonal_pilot_ == false) &&
      (ue_ant_num_ != frame_.NumPilotSyms())) {
    RtAssert(
        false,
        "Number of pilot symbols: " + std::to_string(frame_.NumPilotSyms()) +
            " does not match number of UEs: " + std::to_string(ue_ant_num_));
  }
  if ((freq_orthogonal_pilot_ == false) && (ue_radio_id_.empty() == true) &&
      (tdd_conf.find("ue_radio_num") == tdd_conf.end())) {
    ue_num_ = frame_.NumPilotSyms();
    ue_ant_num_ = ue_num_ * num_ue_channels_;
  }
  ue_ant_offset_ = tdd_conf.value("ue_ant_offset", 0);
  ue_ant_total_ = tdd_conf.value("ue_ant_total", ue_ant_num_);

  auto tx_advance = tdd_conf.value("tx_advance", json::array());
  if (tx_advance.empty()) {
    cl_tx_advance_.resize(ue_num_, 0);
  } else {
    RtAssert(tx_advance.size() == ue_num_,
             "tx_advance size must be same as the number of clients!");
    cl_tx_advance_.assign(tx_advance.begin(), tx_advance.end());
  }

  // Agora configurations
  frames_to_test_ = tdd_conf.value("max_frame", 9600);
  core_offset_ = tdd_conf.value("core_offset", 0);
  worker_thread_num_ = tdd_conf.value("worker_thread_num", 25);
  socket_thread_num_ = tdd_conf.value("socket_thread_num", 4);
  ue_core_offset_ = tdd_conf.value("ue_core_offset", 0);
  ue_worker_thread_num_ = tdd_conf.value("ue_worker_thread_num", 25);
  ue_socket_thread_num_ = tdd_conf.value("ue_socket_thread_num", 4);
  fft_thread_num_ = tdd_conf.value("fft_thread_num", 5);
  demul_thread_num_ = tdd_conf.value("demul_thread_num", 5);
  decode_thread_num_ = tdd_conf.value("decode_thread_num", 10);
  zf_thread_num_ = worker_thread_num_ - fft_thread_num_ - demul_thread_num_ -
                   decode_thread_num_;

  demul_block_size_ = tdd_conf.value("demul_block_size", 48);
  RtAssert(demul_block_size_ % kSCsPerCacheline == 0,
           "Demodulation block size must be a multiple of subcarriers per "
           "cacheline");
  RtAssert(
      demul_block_size_ % kTransposeBlockSize == 0,
      "Demodulation block size must be a multiple of transpose block size");
  demul_events_per_symbol_ = 1 + (ofdm_data_num_ - 1) / demul_block_size_;

  zf_batch_size_ = tdd_conf.value("zf_batch_size", 1);
  zf_block_size_ =
      freq_orthogonal_pilot_ ? ue_ant_num_ : tdd_conf.value("zf_block_size", 1);
  zf_events_per_symbol_ = 1 + (ofdm_data_num_ - 1) / zf_block_size_;

  fft_block_size_ = tdd_conf.value("fft_block_size", 1);
  fft_block_size_ = std::max(fft_block_size_, num_channels_);
  encode_block_size_ = tdd_conf.value("encode_block_size", 1);

  noise_level_ = tdd_conf.value("noise_level", 0.03);  // default: 30 dB
  MLPD_SYMBOL("Noise level: %.2f\n", noise_level_);

  // LDPC Coding configurations
  uint16_t base_graph = tdd_conf.value("base_graph", 1);
  uint16_t zc = tdd_conf.value("Zc", 72);
  bool early_term = tdd_conf.value("earlyTermination", true);
  int16_t max_decoder_iter = tdd_conf.value("decoderIter", 5);
  size_t num_rows = tdd_conf.value("nRows", (base_graph == 1) ? 46 : 42);
  uint32_t num_cb_len = LdpcNumInputBits(base_graph, zc);
  uint32_t num_cb_codew_len = LdpcNumEncodedBits(base_graph, zc, num_rows);

  ldpc_config_ = LDPCconfig(base_graph, zc, max_decoder_iter, early_term,
                            num_cb_len, num_cb_codew_len, num_rows, 0);

  // Scrambler and descrambler configurations
  scramble_enabled_ = tdd_conf.value("wlan_scrambler", true);

  // Modulation configurations
  mod_order_bits_ =
      modulation_ == "64QAM"
          ? CommsLib::kQaM64
          : (modulation_ == "16QAM" ? CommsLib::kQaM16 : CommsLib::kQpsk);
  // Updates num_block_in_sym
  UpdateModCfgs(mod_order_bits_);

  RtAssert(ldpc_config_.NumBlocksInSymbol() > 0,
           "LDPC expansion factor is too large for number of OFDM data "
           "subcarriers.");

  MLPD_INFO(
      "Config: LDPC: Zc: %d, %zu code blocks per symbol, %d information "
      "bits per encoding, %d bits per encoded code word, decoder "
      "iterations: %d, code rate %.3f (nRows = %zu)\n",
      ldpc_config_.ExpansionFactor(), ldpc_config_.NumBlocksInSymbol(),
      ldpc_config_.NumCbLen(), ldpc_config_.NumCbCodewLen(),
      ldpc_config_.MaxDecoderIter(),
      1.f * LdpcNumInputCols(ldpc_config_.BaseGraph()) /
          (LdpcNumInputCols(ldpc_config_.BaseGraph()) - 2 +
           ldpc_config_.NumRows()),
      ldpc_config_.NumRows());

  fft_in_rru_ = tdd_conf.value("fft_in_rru", false);

  samps_per_symbol_ =
      ofdm_tx_zero_prefix_ + ofdm_ca_num_ + cp_len_ + ofdm_tx_zero_postfix_;
  packet_length_ =
      Packet::kOffsetOfData + ((kUse12BitIQ ? 3 : 4) * samps_per_symbol_);
  dl_packet_length_ = Packet::kOffsetOfData + (samps_per_symbol_ * 4);

  //Don't check for jumbo frames when using the hardware, this might be temp
  if (false) {
    RtAssert(packet_length_ < 9000,
             "Packet size must be smaller than jumbo frame");
  }

  num_bytes_per_cb_ = ldpc_config_.NumCbLen() / 8;
  data_bytes_num_persymbol_ =
      num_bytes_per_cb_ * ldpc_config_.NumBlocksInSymbol();

  mac_packet_length_ = data_bytes_num_persymbol_;
  // Smallest over the air packet structure
  mac_data_length_max_ = mac_packet_length_ - sizeof(MacPacketHeaderPacked);

  ul_mac_packets_perframe_ = this->frame_.NumUlDataSyms();
  ul_mac_data_bytes_num_perframe_ =
      mac_data_length_max_ * ul_mac_packets_perframe_;
  ul_mac_bytes_num_perframe_ = mac_packet_length_ * ul_mac_packets_perframe_;

  dl_mac_packets_perframe_ = this->frame_.NumDlDataSyms();
  dl_mac_data_bytes_num_perframe_ =
      mac_data_length_max_ * dl_mac_packets_perframe_;
  dl_mac_bytes_num_perframe_ = mac_packet_length_ * dl_mac_packets_perframe_;

  this->running_.store(true);
  MLPD_INFO(
      "Config: %zu BS antennas, %zu UE antennas, %zu pilot symbols per "
      "frame,\n\t%zu uplink data symbols per frame, %zu downlink data "
      "symbols per frame,\n\t%zu OFDM subcarriers (%zu data subcarriers), "
      "modulation %s,\n\t%zu codeblocks per symbol, %zu bytes per code block,"
      "\n\t%zu UL MAC data bytes per frame, %zu UL MAC bytes per frame, "
      "\n\t%zu DL MAC data bytes per frame, %zu DL MAC bytes per frame, "
      "frame time %.3f usec \nUplink Max Mac data tp (Mbps) %.3f "
      "\nDownlink Max Mac data tp (Mbps) %.3f \n",
      bs_ant_num_, ue_ant_num_, frame_.NumPilotSyms(), frame_.NumULSyms(),
      frame_.NumDLSyms(), ofdm_ca_num_, ofdm_data_num_, modulation_.c_str(),
      ldpc_config_.NumBlocksInSymbol(), num_bytes_per_cb_,
      ul_mac_data_bytes_num_perframe_, ul_mac_bytes_num_perframe_,
      dl_mac_data_bytes_num_perframe_, dl_mac_bytes_num_perframe_,
      this->GetFrameDurationSec() * 1e6,
      (ul_mac_data_bytes_num_perframe_ * 8.0f) /
          (this->GetFrameDurationSec() * 1e6),
      (dl_mac_data_bytes_num_perframe_ * 8.0f) /
          (this->GetFrameDurationSec() * 1e6));
  Print();
}

void Config::GenData() {
  if ((kUseArgos == true) || (kUseUHD == true)) {
    std::vector<std::vector<double>> gold_ifft =
        CommsLib::GetSequence(128, CommsLib::kGoldIfft);
    std::vector<std::complex<int16_t>> gold_ifft_ci16 =
        Utils::DoubleToCint16(gold_ifft);
    for (size_t i = 0; i < 128; i++) {
      this->gold_cf32_.emplace_back(gold_ifft[0][i], gold_ifft[1][i]);
    }

    std::vector<std::vector<double>> sts_seq =
        CommsLib::GetSequence(0, CommsLib::kStsSeq);
    std::vector<std::complex<int16_t>> sts_seq_ci16 =
        Utils::DoubleToCint16(sts_seq);

    // Populate STS (stsReps repetitions)
    int sts_reps = 15;
    for (int i = 0; i < sts_reps; i++) {
      this->beacon_ci16_.insert(this->beacon_ci16_.end(), sts_seq_ci16.begin(),
                                sts_seq_ci16.end());
    }

    // Populate gold sequence (two reps, 128 each)
    int gold_reps = 2;
    for (int i = 0; i < gold_reps; i++) {
      this->beacon_ci16_.insert(this->beacon_ci16_.end(),
                                gold_ifft_ci16.begin(), gold_ifft_ci16.end());
    }

    this->beacon_len_ = this->beacon_ci16_.size();

    if (this->samps_per_symbol_ <
        (this->beacon_len_ + this->ofdm_tx_zero_prefix_ +
         this->ofdm_tx_zero_postfix_)) {
      std::string msg = "Minimum supported symbol_size is ";
      msg += std::to_string(this->beacon_len_);
      throw std::invalid_argument(msg);
    }

    this->beacon_ = Utils::Cint16ToUint32(this->beacon_ci16_, false, "QI");
    this->coeffs_ = Utils::Cint16ToUint32(gold_ifft_ci16, true, "QI");

    // Add addition padding for beacon sent from host
    int frac_beacon = this->samps_per_symbol_ % this->beacon_len_;
    std::vector<std::complex<int16_t>> pre_beacon(this->ofdm_tx_zero_prefix_,
                                                  0);
    std::vector<std::complex<int16_t>> post_beacon(
        this->ofdm_tx_zero_postfix_ + frac_beacon, 0);
    this->beacon_ci16_.insert(this->beacon_ci16_.begin(), pre_beacon.begin(),
                              pre_beacon.end());
    this->beacon_ci16_.insert(this->beacon_ci16_.end(), post_beacon.begin(),
                              post_beacon.end());
  }

  // Generate common pilots based on Zadoff-Chu sequence for channel estimation
  auto zc_seq_double =
      CommsLib::GetSequence(this->ofdm_data_num_, CommsLib::kLteZadoffChu);
  auto zc_seq = Utils::DoubleToCfloat(zc_seq_double);
  this->common_pilot_ =
      CommsLib::SeqCyclicShift(zc_seq, M_PI / 4);  // Used in LTE SRS

  this->pilots_ = static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
      Agora_memory::Alignment_t::kAlign64,
      this->ofdm_data_num_ * sizeof(complex_float)));
  this->pilots_sgn_ =
      static_cast<complex_float*>(Agora_memory::PaddedAlignedAlloc(
          Agora_memory::Alignment_t::kAlign64,
          this->ofdm_data_num_ *
              sizeof(complex_float)));  // used in CSI estimation
  for (size_t i = 0; i < ofdm_data_num_; i++) {
    this->pilots_[i] = {this->common_pilot_[i].real(),
                        this->common_pilot_[i].imag()};
    auto pilot_sgn = this->common_pilot_[i] /
                     (float)std::pow(std::abs(this->common_pilot_[i]), 2);
    this->pilots_sgn_[i] = {pilot_sgn.real(), pilot_sgn.imag()};
  }
  complex_float* pilot_ifft;
  AllocBuffer1d(&pilot_ifft, this->ofdm_ca_num_,
                Agora_memory::Alignment_t::kAlign64, 1);
  for (size_t j = 0; j < ofdm_data_num_; j++) {
    pilot_ifft[j + this->ofdm_data_start_] = this->pilots_[j];
  }
  CommsLib::IFFT(pilot_ifft, this->ofdm_ca_num_, false);

  // Generate UE-specific pilots based on Zadoff-Chu sequence for phase tracking
  this->ue_specific_pilot_.Malloc(this->ue_ant_num_, this->ofdm_data_num_,
                                  Agora_memory::Alignment_t::kAlign64);
  this->ue_specific_pilot_t_.Calloc(this->ue_ant_num_, this->samps_per_symbol_,
                                    Agora_memory::Alignment_t::kAlign64);

  Table<complex_float> ue_pilot_ifft;
  ue_pilot_ifft.Calloc(this->ue_ant_num_, this->ofdm_ca_num_,
                       Agora_memory::Alignment_t::kAlign64);
  auto zc_ue_pilot_double =
      CommsLib::GetSequence(this->ofdm_data_num_, CommsLib::kLteZadoffChu);
  auto zc_ue_pilot = Utils::DoubleToCfloat(zc_ue_pilot_double);
  for (size_t i = 0; i < ue_ant_num_; i++) {
    auto zc_ue_pilot_i = CommsLib::SeqCyclicShift(
        zc_ue_pilot,
        (i + this->ue_ant_offset_) * (float)M_PI / 6);  // LTE DMRS
    for (size_t j = 0; j < this->ofdm_data_num_; j++) {
      this->ue_specific_pilot_[i][j] = {zc_ue_pilot_i[j].real(),
                                        zc_ue_pilot_i[j].imag()};
      ue_pilot_ifft[i][j + this->ofdm_data_start_] =
          this->ue_specific_pilot_[i][j];
    }
    CommsLib::IFFT(ue_pilot_ifft[i], ofdm_ca_num_, false);
  }

  // Get uplink and downlink raw bits either from file or random numbers
  size_t num_bytes_per_ue_pad = Roundup<64>(this->num_bytes_per_cb_) *
                                this->ldpc_config_.NumBlocksInSymbol();
  dl_bits_.Malloc(this->frame_.NumDLSyms(),
                  num_bytes_per_ue_pad * this->ue_ant_num_,
                  Agora_memory::Alignment_t::kAlign64);
  dl_iq_f_.Calloc(this->frame_.NumDLSyms(), ofdm_ca_num_ * ue_ant_num_,
                  Agora_memory::Alignment_t::kAlign64);
  dl_iq_t_.Calloc(this->frame_.NumDLSyms(),
                  this->samps_per_symbol_ * this->ue_ant_num_,
                  Agora_memory::Alignment_t::kAlign64);

  ul_bits_.Malloc(this->frame_.NumULSyms(),
                  num_bytes_per_ue_pad * this->ue_ant_num_,
                  Agora_memory::Alignment_t::kAlign64);
  ul_iq_f_.Calloc(this->frame_.NumULSyms(),
                  this->ofdm_ca_num_ * this->ue_ant_num_,
                  Agora_memory::Alignment_t::kAlign64);
  ul_iq_t_.Calloc(this->frame_.NumULSyms(),
                  this->samps_per_symbol_ * this->ue_ant_num_,
                  Agora_memory::Alignment_t::kAlign64);

#ifdef GENERATE_DATA
  for (size_t ue_id = 0; ue_id < this->ue_ant_num_; ue_id++) {
    for (size_t j = 0; j < num_bytes_per_ue_pad; j++) {
      int cur_offset = j * ue_ant_num_ + ue_id;
      for (size_t i = 0; i < this->frame_.NumULSyms(); i++) {
        this->ul_bits_[i][cur_offset] = rand() % mod_order;
      }
      for (size_t i = 0; i < this->frame_.NumDLSyms(); i++) {
        this->dl_bits_[i][cur_offset] = rand() % mod_order;
      }
    }
  }
#else
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  if (this->frame_.NumUlDataSyms() > 0) {
    std::string ul_data_file = cur_directory + "/data/LDPC_orig_ul_data_" +
                               std::to_string(this->ofdm_ca_num_) + "_ant" +
                               std::to_string(this->ue_ant_total_) + ".bin";
    MLPD_SYMBOL("Config: Reading raw ul data from %s\n", ul_data_file.c_str());
    FILE* fd = std::fopen(ul_data_file.c_str(), "rb");
    if (fd == nullptr) {
      MLPD_ERROR("Failed to open antenna file %s. Error %s.\n",
                 ul_data_file.c_str(), strerror(errno));
      throw std::runtime_error("Config: Failed to open antenna file");
    }

    for (size_t i = this->frame_.ClientUlPilotSymbols();
         i < this->frame_.NumULSyms(); i++) {
      if (std::fseek(fd, (data_bytes_num_persymbol_ * this->ue_ant_offset_),
                     SEEK_CUR) != 0) {
        MLPD_ERROR(" *** Error: failed to seek propertly (pre) into %s file\n",
                   ul_data_file.c_str());
        RtAssert(false,
                 "Failed to seek propertly into " + ul_data_file + "file\n");
      }
      for (size_t j = 0; j < this->ue_ant_num_; j++) {
        size_t r = std::fread(this->ul_bits_[i] + (j * num_bytes_per_ue_pad),
                              sizeof(int8_t), data_bytes_num_persymbol_, fd);
        if (r < data_bytes_num_persymbol_) {
          MLPD_ERROR(
              " *** Error: Uplink bad read from file %s (batch %zu : %zu) "
              "%zu : %zu\n",
              ul_data_file.c_str(), i, j, r, data_bytes_num_persymbol_);
        }
      }
      if (std::fseek(fd,
                     data_bytes_num_persymbol_ *
                         (this->ue_ant_total_ - this->ue_ant_offset_ -
                          this->ue_ant_num_),
                     SEEK_CUR) != 0) {
        MLPD_ERROR(" *** Error: failed to seek propertly (post) into %s file\n",
                   ul_data_file.c_str());
        RtAssert(false,
                 "Failed to seek propertly into " + ul_data_file + "file\n");
      }
    }
    std::fclose(fd);
  }

  if (this->frame_.NumDlDataSyms() > 0) {
    std::string dl_data_file = cur_directory + "/data/LDPC_orig_dl_data_" +
                               std::to_string(this->ofdm_ca_num_) + "_ant" +
                               std::to_string(this->ue_ant_total_) + ".bin";

    MLPD_SYMBOL("Config: Reading raw dl data from %s\n", dl_data_file.c_str());
    FILE* fd = std::fopen(dl_data_file.c_str(), "rb");
    if (fd == nullptr) {
      MLPD_ERROR("Failed to open antenna file %s. Error %s.\n",
                 dl_data_file.c_str(), strerror(errno));
      throw std::runtime_error("Config: Failed to open dl antenna file");
    }

    for (size_t i = this->frame_.ClientDlPilotSymbols();
         i < this->frame_.NumDLSyms(); i++) {
      for (size_t j = 0; j < this->ue_ant_num_; j++) {
        size_t r = std::fread(this->dl_bits_[i] + j * num_bytes_per_ue_pad,
                              sizeof(int8_t), data_bytes_num_persymbol_, fd);
        if (r < data_bytes_num_persymbol_) {
          MLPD_ERROR(
              "***Error: Downlink bad read from file %s (batch %zu : %zu) "
              "\n",
              dl_data_file.c_str(), i, j);
        }
      }
    }
    std::fclose(fd);
  }
#endif

  auto scrambler = std::make_unique<AgoraScrambler::Scrambler>();

  const size_t encoded_bytes_per_block =
      BitsToBytes(this->ldpc_config_.NumCbCodewLen());
  const size_t num_blocks_per_symbol =
      this->ldpc_config_.NumBlocksInSymbol() * this->ue_ant_num_;

  // Used as an input ptr to
  auto* scramble_buffer =
      new int8_t[num_bytes_per_cb_ +
                 kLdpcHelperFunctionInputBufferSizePaddingBytes]();
  int8_t* ldpc_input = nullptr;

  // Encode uplink bits
  Table<int8_t> ul_encoded_bits;
  ul_encoded_bits.Malloc(this->frame_.NumULSyms() * num_blocks_per_symbol,
                         encoded_bytes_per_block,
                         Agora_memory::Alignment_t::kAlign64);
  ul_mod_bits_.Calloc(this->frame_.NumULSyms(),
                      Roundup<64>(this->ofdm_data_num_) * this->ue_ant_num_,
                      Agora_memory::Alignment_t::kAlign32);
  auto* temp_parity_buffer = new int8_t[LdpcEncodingParityBufSize(
      this->ldpc_config_.BaseGraph(), this->ldpc_config_.ExpansionFactor())];

  for (size_t i = 0; i < frame_.NumULSyms(); i++) {
    for (size_t j = 0; j < ue_ant_num_; j++) {
      for (size_t k = 0; k < ldpc_config_.NumBlocksInSymbol(); k++) {
        int8_t* coded_bits_ptr =
            ul_encoded_bits[i * num_blocks_per_symbol +
                            j * ldpc_config_.NumBlocksInSymbol() + k];

        if (scramble_enabled_) {
          std::memcpy(scramble_buffer, GetInfoBits(ul_bits_, i, j, k),
                      num_bytes_per_cb_);
          scrambler->Scramble(scramble_buffer, num_bytes_per_cb_);
          ldpc_input = scramble_buffer;
        } else {
          ldpc_input = GetInfoBits(ul_bits_, i, j, k);
        }

        LdpcEncodeHelper(ldpc_config_.BaseGraph(),
                         ldpc_config_.ExpansionFactor(), ldpc_config_.NumRows(),
                         coded_bits_ptr, temp_parity_buffer, ldpc_input);
        int8_t* mod_input_ptr =
            GetModBitsBuf(ul_mod_bits_, Direction::kUplink, 0, i, j, k);
        AdaptBitsForMod(reinterpret_cast<uint8_t*>(coded_bits_ptr),
                        reinterpret_cast<uint8_t*>(mod_input_ptr),
                        encoded_bytes_per_block, mod_order_bits_);
      }
    }
  }

  // Generate freq-domain uplink symbols
  Table<complex_float> ul_iq_ifft;
  ul_iq_ifft.Calloc(this->frame_.NumULSyms(),
                    this->ofdm_ca_num_ * this->ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
  for (size_t i = 0; i < this->frame_.NumULSyms(); i++) {
    for (size_t u = 0; u < this->ue_ant_num_; u++) {
      size_t p = u * this->ofdm_data_num_;
      size_t q = u * this->ofdm_ca_num_;

      for (size_t j = this->ofdm_data_start_; j < this->ofdm_data_stop_; j++) {
        size_t k = j - ofdm_data_start_;
        size_t s = p + k;
        ul_iq_f_[i][q + j] = ModSingleUint8(ul_mod_bits_[i][s], mod_table_);
        ul_iq_ifft[i][q + j] = ul_iq_f_[i][q + j];
      }
      CommsLib::IFFT(&ul_iq_ifft[i][q], ofdm_ca_num_, false);
    }
  }

  // Encode downlink bits
  Table<int8_t> dl_encoded_bits;
  dl_encoded_bits.Malloc(this->frame_.NumDLSyms() * num_blocks_per_symbol,
                         encoded_bytes_per_block,
                         Agora_memory::Alignment_t::kAlign64);
  dl_mod_bits_.Calloc(this->frame_.NumDLSyms(),
                      Roundup<64>(ofdm_data_num_) * ue_ant_num_,
                      Agora_memory::Alignment_t::kAlign32);

  for (size_t i = 0; i < this->frame_.NumDLSyms(); i++) {
    for (size_t j = 0; j < this->ue_ant_num_; j++) {
      for (size_t k = 0; k < ldpc_config_.NumBlocksInSymbol(); k++) {
        int8_t* coded_bits_ptr =
            dl_encoded_bits[i * num_blocks_per_symbol +
                            j * ldpc_config_.NumBlocksInSymbol() + k];

        if (scramble_enabled_) {
          std::memcpy(scramble_buffer, GetInfoBits(dl_bits_, i, j, k),
                      num_bytes_per_cb_);
          scrambler->Scramble(scramble_buffer, num_bytes_per_cb_);
          ldpc_input = scramble_buffer;
        } else {
          ldpc_input = GetInfoBits(dl_bits_, i, j, k);
        }

        LdpcEncodeHelper(ldpc_config_.BaseGraph(),
                         ldpc_config_.ExpansionFactor(), ldpc_config_.NumRows(),
                         coded_bits_ptr, temp_parity_buffer, ldpc_input);
        int8_t* mod_input_ptr =
            GetModBitsBuf(dl_mod_bits_, Direction::kDownlink, 0, i, j, k);
        AdaptBitsForMod(reinterpret_cast<uint8_t*>(coded_bits_ptr),
                        reinterpret_cast<uint8_t*>(mod_input_ptr),
                        encoded_bytes_per_block, mod_order_bits_);
      }
    }
  }

  // Generate freq-domain downlink symbols
  Table<complex_float> dl_iq_ifft;
  dl_iq_ifft.Calloc(this->frame_.NumDLSyms(), ofdm_ca_num_ * ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
  for (size_t i = 0; i < this->frame_.NumDLSyms(); i++) {
    for (size_t u = 0; u < ue_ant_num_; u++) {
      size_t p = u * ofdm_data_num_;
      size_t q = u * ofdm_ca_num_;

      for (size_t j = ofdm_data_start_; j < ofdm_data_stop_; j++) {
        int k = j - ofdm_data_start_;
        size_t s = p + k;
        if (k % ofdm_pilot_spacing_ != 0) {
          dl_iq_f_[i][q + j] = ModSingleUint8(dl_mod_bits_[i][s], mod_table_);
        } else {
          dl_iq_f_[i][q + j] = ue_specific_pilot_[u][k];
        }
        dl_iq_ifft[i][q + j] = dl_iq_f_[i][q + j];
      }
      CommsLib::IFFT(&dl_iq_ifft[i][q], ofdm_ca_num_, false);
    }
  }

  // Find normalization factor through searching for max value in IFFT results
  float max_val = CommsLib::FindMaxAbs(ul_iq_ifft, this->frame_.NumULSyms(),
                                       this->ue_ant_num_ * this->ofdm_ca_num_);
  float cur_max_val =
      CommsLib::FindMaxAbs(dl_iq_ifft, this->frame_.NumDLSyms(),
                           this->ue_ant_num_ * this->ofdm_ca_num_);
  if (cur_max_val > max_val) {
    max_val = cur_max_val;
  }
  cur_max_val = CommsLib::FindMaxAbs(ue_pilot_ifft, this->ue_ant_num_,
                                     this->ofdm_ca_num_);
  if (cur_max_val > max_val) {
    max_val = cur_max_val;
  }
  cur_max_val = CommsLib::FindMaxAbs(pilot_ifft, this->ofdm_ca_num_);
  if (cur_max_val > max_val) {
    max_val = cur_max_val;
  }

  this->scale_ = 2 * max_val;  // additional 2^2 (6dB) power backoff

  // Generate time domain symbols for downlink
  for (size_t i = 0; i < this->frame_.NumDLSyms(); i++) {
    for (size_t u = 0; u < this->ue_ant_num_; u++) {
      size_t q = u * this->ofdm_ca_num_;
      size_t r = u * this->samps_per_symbol_;
      CommsLib::Ifft2tx(&dl_iq_ifft[i][q], &this->dl_iq_t_[i][r],
                        this->ofdm_ca_num_, this->ofdm_tx_zero_prefix_,
                        this->cp_len_, kDebugDownlink ? 1 : this->scale_);
    }
  }

  // Generate time domain uplink symbols
  for (size_t i = 0; i < this->frame_.NumULSyms(); i++) {
    for (size_t u = 0; u < this->ue_ant_num_; u++) {
      size_t q = u * this->ofdm_ca_num_;
      size_t r = u * this->samps_per_symbol_;
      CommsLib::Ifft2tx(&ul_iq_ifft[i][q], &ul_iq_t_[i][r], this->ofdm_ca_num_,
                        this->ofdm_tx_zero_prefix_, this->cp_len_,
                        this->scale_);
    }
  }

  // Generate time domain ue-specific pilot symbols
  for (size_t i = 0; i < this->ue_ant_num_; i++) {
    CommsLib::Ifft2tx(ue_pilot_ifft[i], this->ue_specific_pilot_t_[i],
                      this->ofdm_ca_num_, this->ofdm_tx_zero_prefix_,
                      this->cp_len_, kDebugDownlink ? 1 : this->scale_);
    if (kDebugPrintPilot == true) {
      std::printf("ue_specific_pilot_t%zu=[", i);
      for (size_t j = 0; j < this->ofdm_ca_num_; j++) {
        std::printf("%2.4f+%2.4fi ", ue_pilot_ifft[i][j].re,
                    ue_pilot_ifft[i][j].im);
      }
      std::printf("]\n");
    }
  }

  this->pilot_ci16_.resize(samps_per_symbol_, 0);
  CommsLib::Ifft2tx(pilot_ifft,
                    (std::complex<int16_t>*)this->pilot_ci16_.data(),
                    ofdm_ca_num_, ofdm_tx_zero_prefix_, cp_len_, scale_);

  for (size_t i = 0; i < ofdm_ca_num_; i++) {
    this->pilot_cf32_.emplace_back(pilot_ifft[i].re / scale_,
                                   pilot_ifft[i].im / scale_);
  }
  this->pilot_cf32_.insert(this->pilot_cf32_.begin(),
                           this->pilot_cf32_.end() - this->cp_len_,
                           this->pilot_cf32_.end());  // add CP

  // generate a UINT32 version to write to FPGA buffers
  this->pilot_ = Utils::Cfloat32ToUint32(this->pilot_cf32_, false, "QI");

  std::vector<uint32_t> pre_uint32(this->ofdm_tx_zero_prefix_, 0);
  this->pilot_.insert(this->pilot_.begin(), pre_uint32.begin(),
                      pre_uint32.end());
  this->pilot_.resize(this->samps_per_symbol_);

  if (kDebugPrintPilot == true) {
    std::cout << "Pilot data: " << std::endl;
    for (size_t i = 0; i < this->ofdm_data_num_; i++) {
      std::cout << this->pilots_[i].re << "+1i*" << this->pilots_[i].im << ",";
    }
    std::cout << std::endl;
  }

  if (kDebugPrintPilot) {
    for (size_t ue_id = 0; ue_id < ue_ant_num_; ue_id++) {
      std::cout << "UE" << ue_id << "_pilot_data =[" << std::endl;
      for (size_t i = 0; i < ofdm_data_num_; i++) {
        std::cout << ue_specific_pilot_[ue_id][i].re << "+1i*"
                  << ue_specific_pilot_[ue_id][i].im << " ";
      }
      std::cout << "];" << std::endl;
    }
  }

  delete[](temp_parity_buffer);
  ul_iq_ifft.Free();
  dl_iq_ifft.Free();
  ue_pilot_ifft.Free();
  dl_encoded_bits.Free();
  ul_encoded_bits.Free();
  FreeBuffer1d(&pilot_ifft);
  delete[] scramble_buffer;
}

Config::~Config() {
  if (pilots_ != nullptr) {
    std::free(pilots_);
    pilots_ = nullptr;
  }
  if (pilots_sgn_ != nullptr) {
    std::free(pilots_sgn_);
    pilots_sgn_ = nullptr;
  }
  mod_table_.Free();
  dl_bits_.Free();
  ul_bits_.Free();
  ul_mod_bits_.Free();
  dl_mod_bits_.Free();
  dl_iq_f_.Free();
  dl_iq_t_.Free();
  ul_iq_f_.Free();
  ul_iq_t_.Free();

  ue_specific_pilot_t_.Free();
  ue_specific_pilot_.Free();
}

/* TODO Inspect and document */
size_t Config::GetSymbolId(size_t input_id) const {
  size_t symbol_id = SIZE_MAX;

  if (input_id < this->frame_.NumPilotSyms()) {
    symbol_id = this->Frame().GetPilotSymbol(input_id);
  } else {
    int new_idx = input_id - this->frame_.NumPilotSyms();

    // std::printf("\n*****GetSymbolId %d %zu\n", new_idx, input_id);
    if ((new_idx >= 0) &&
        (static_cast<size_t>(new_idx) < this->frame_.NumULSyms())) {
      symbol_id = this->Frame().GetULSymbol(new_idx);
    }
  }
  return symbol_id;
}

/* Returns True if symbol is valid index and is of symbol type 'P'
   False otherwise */
bool Config::IsPilot(size_t /*frame_id*/, size_t symbol_id) const {
  bool is_pilot = false;
  assert(symbol_id < this->frame_.NumTotalSyms());
  char s = frame_.FrameIdentifier().at(symbol_id);
#ifdef DEBUG3
  std::printf("IsPilot(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
  /* TODO should use the symbol type here */
  is_pilot = (s == 'P');
  return is_pilot;
}

/* Returns True if user equiptment and is a client dl pilot_
 * False otherwise */
bool Config::IsDlPilot(size_t /*frame_id*/, size_t symbol_id) const {
  bool is_pilot = false;
  assert(symbol_id < this->frame_.NumTotalSyms());
  char s = frame_.FrameIdentifier().at(symbol_id);
#ifdef DEBUG3
  std::printf("IsDlPilot(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
  if ((s == 'D') && (this->frame_.ClientDlPilotSymbols() > 0)) {
    size_t dl_index = this->frame_.GetDLSymbolIdx(symbol_id);
    is_pilot = (this->frame_.ClientDlPilotSymbols() > dl_index);
  }
  return is_pilot;
}

bool Config::IsCalDlPilot(size_t /*frame_id*/, size_t symbol_id) const {
  bool is_cal_dl_pilot = false;
  assert(symbol_id < this->frame_.NumTotalSyms());
  is_cal_dl_pilot = (this->frame_.FrameIdentifier().at(symbol_id) == 'C');
  return is_cal_dl_pilot;
}

bool Config::IsCalUlPilot(size_t /*frame_id*/, size_t symbol_id) const {
  bool is_cal_ul_pilot = false;
  assert(symbol_id < this->frame_.NumTotalSyms());
  is_cal_ul_pilot = (this->frame_.FrameIdentifier().at(symbol_id) == 'L');
  return is_cal_ul_pilot;
}

bool Config::IsUplink(size_t /*frame_id*/, size_t symbol_id) const {
  assert(symbol_id < this->frame_.NumTotalSyms());
  char s = frame_.FrameIdentifier().at(symbol_id);
#ifdef DEBUG3
  std::printf("IsUplink(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
  return (s == 'U');
}

bool Config::IsDownlink(size_t frame_id, size_t symbol_id) const {
  assert(symbol_id < this->frame_.NumTotalSyms());
  char s = frame_.FrameIdentifier().at(symbol_id);
#ifdef DEBUG3
  std::printf("IsDownlink(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#else
  unused(frame_id);
#endif
  return (s == 'D');
}

SymbolType Config::GetSymbolType(size_t symbol_id) const {
  return kSymbolMap.at(this->frame_.FrameIdentifier().at(symbol_id));
}

void Config::Print() const {
  if (kDebugPrintConfiguration == true) {
    std::cout << "Freq Ghz: " << freq_ghz_ << std::endl
              << "BaseStation ant num: " << bs_ant_num_ << std::endl
              << "BeamForming ant num: " << bf_ant_num_ << std::endl
              << "Ue num: " << ue_num_ << std::endl
              << "Ue ant num: " << ue_ant_num_ << std::endl
              << "Ue ant total: " << ue_ant_total_ << std::endl
              << "Ue ant offset: " << ue_ant_offset_ << std::endl
              << "OFDM Ca num: " << ofdm_ca_num_ << std::endl
              << "Cp Len: " << cp_len_ << std::endl
              << "Ofdm data num: " << ofdm_data_num_ << std::endl
              << "Ofdm data start: " << ofdm_data_start_ << std::endl
              << "Ofdm data stop: " << ofdm_data_stop_ << std::endl
              << "Ofdm pilot spacing: " << ofdm_pilot_spacing_ << std::endl
              << "Hardware framer: " << hw_framer_ << std::endl
              << "Ue Hardware framer: " << ue_hw_framer_ << std::endl
              << "Freq: " << freq_ << std::endl
              << "Rate: " << rate_ << std::endl
              << "NCO: " << nco_ << std::endl
              << "Scrambler Enabled: " << scramble_enabled_ << std::endl
              << "Radio Rf Freq: " << radio_rf_freq_ << std::endl
              << "Bw filter: " << bw_filter_ << std::endl
              << "Single Gain: " << single_gain_ << std::endl
              << "Tx Gain A: " << tx_gain_a_ << std::endl
              << "Rx Gain A: " << rx_gain_a_ << std::endl
              << "Tx Gain B: " << tx_gain_b_ << std::endl
              << "Rx Gain B: " << rx_gain_b_ << std::endl
              << "Calib Tx Gain A: " << calib_tx_gain_a_ << std::endl
              << "Calib Tx Gain B: " << calib_tx_gain_b_ << std::endl
              << "Num Cells: " << num_cells_ << std::endl
              << "Num Bs Radios: " << num_radios_ << std::endl
              << "Num Bs Channels: " << num_channels_ << std::endl
              << "Num Ue Channels: " << num_ue_channels_ << std::endl
              << "Beacon Ant: " << beacon_ant_ << std::endl
              << "Beacon len: " << beacon_len_ << std::endl
              << "Calib init repeat: " << init_calib_repeat_ << std::endl
              << "Beamsweep " << beamsweep_ << std::endl
              << "Sample Cal En: " << sample_cal_en_ << std::endl
              << "Imbalance Cal: " << imbalance_cal_en_ << std::endl
              << "Bs Channel: " << channel_ << std::endl
              << "Ue Channel: " << ue_channel_ << std::endl
              << "Ant Group num: " << ant_group_num_ << std::endl
              << "Ant Per Group: " << ant_per_group_ << std::endl
              << "Max Frames: " << frames_to_test_ << std::endl
              << "Transport Block Size: " << transport_block_size_ << std::endl
              << "Noise Level: " << noise_level_ << std::endl
              << "Bytes per CB: " << num_bytes_per_cb_ << std::endl
              << "FFT in rru: " << fft_in_rru_ << std::endl;
  }
}

extern "C" {
__attribute__((visibility("default"))) Config* ConfigNew(char* filename) {
  auto* cfg = new Config(filename);
  cfg->GenData();
  return cfg;
}
}
