// Copyright (c) 2018-2022, Rice University
// RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license

/**
 * @file config.cc
 * @brief Implementation file for the configuration class which importants
 * json configuration values into class variables
 */

#include "config.h"

#include <ctime>
#include <filesystem>
#include <utility>

#include "comms-constants.inc"
#include "comms-lib.h"
#include "data_generator.h"
#include "datatype_conversion.h"
#include "fivegconfig.h"
#include "gettime.h"
#include "logger.h"
#include "message.h"
#include "modulation.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "scrambler.h"
#include "simd_types.h"
#include "utils_ldpc.h"

using json = nlohmann::json;

static constexpr double kTddSwitchingTime = 20e-6;  // in sec
static constexpr size_t kBeaconTxZeroPostfix = 64;  // arbitrary
static constexpr size_t kMacAlignmentBytes = 64u;
static constexpr bool kDebugPrintConfiguration = false;
static constexpr size_t kMaxSupportedZc = 256;
static constexpr size_t kShortIdLen = 3;
static constexpr size_t kVarNodesSize = 1024 * 1024 * sizeof(int16_t);
static constexpr size_t kControlMCS = 5;  // QPSK, 379/1024

/// Print the I/Q samples in the pilots
static constexpr bool kDebugPrintPilot = false;

static const std::string kLogFilepath =
    TOSTRING(PROJECT_DIRECTORY) "/files/log/";

Config::Config(std::string jsonfilename)
    : freq_ghz_(GetTime::MeasureRdtscFreq()),
      ul_ldpc_config_(0, 0, 0, false, 0, 0, 0, 0),
      dl_ldpc_config_(0, 0, 0, false, 0, 0, 0, 0),
      dl_bcast_ldpc_config_(0, 0, 0, false, 0, 0, 0, 0),
      frame_(""),
      pilot_ifft_(nullptr),
      config_filename_(std::move(jsonfilename)) {
  auto time = std::time(nullptr);
  auto local_time = *std::localtime(&time);
  timestamp_ = std::to_string(1900 + local_time.tm_year) + "-" +
               std::to_string(1 + local_time.tm_mon) + "-" +
               std::to_string(local_time.tm_mday) + "-" +
               std::to_string(local_time.tm_hour) + "-" +
               std::to_string(local_time.tm_min) + "-" +
               std::to_string(local_time.tm_sec);

  pilots_ = nullptr;
  pilots_sgn_ = nullptr;

  std::string conf;
  Utils::LoadTddConfig(config_filename_, conf);
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
  if (serial_file.empty() == false) {
    Utils::LoadTddConfig(serial_file, serials_str);
  }
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
        AGORA_LOG_INFO(
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
  } else if (kUseArgos == true) {
    throw std::runtime_error(
        "Hardware is enabled but the serials files was not accessable");
  }

  if (radio_id_.empty()) {
    num_radios_ = tdd_conf.value("bs_radio_num", 8);
    external_ref_node_.resize(num_cells_, false);
    cell_id_.resize(num_radios_, 0);

    //Add in serial numbers
    for (size_t radio = 0; radio < num_radios_; radio++) {
      AGORA_LOG_TRACE("Adding BS_SIM_RADIO_%d\n", radio);
      radio_id_.emplace_back("BS_SIM_RADIO_" + std::to_string(radio));
    }
  }

  if (ue_radio_id_.empty()) {
    ue_num_ = tdd_conf.value("ue_radio_num", 8);
    for (size_t ue_radio = 0; ue_radio < ue_num_; ue_radio++) {
      std::stringstream ss;
      ss << std::setw(kShortIdLen) << std::setfill('0') << ue_radio;
      const std::string ue_name = "UE_SIM_RADIO_" + ss.str();
      AGORA_LOG_TRACE("Adding %s\n", ue_name.c_str());
      ue_radio_id_.push_back(ue_name);
    }
  }
  ue_num_ = ue_radio_id_.size();
  for (size_t i = 0; i < ue_num_; i++) {
    ue_radio_name_.push_back(
        "UE" + (ue_radio_id_.at(i).length() > kShortIdLen
                    ? ue_radio_id_.at(i).substr(ue_radio_id_.at(i).length() -
                                                kShortIdLen)
                    : ue_radio_id_.at(i)));
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

  if ((kUseArgos == true) || (kUseUHD == true) || (kUsePureUHD == true)) {
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
  sample_cal_en_ = tdd_conf.value("calibrate_digital", false);
  imbalance_cal_en_ = tdd_conf.value("calibrate_analog", false);
  init_calib_repeat_ = tdd_conf.value("init_calib_repeat", 0);
  smooth_calib_ = tdd_conf.value("smooth_calib", false);
  beamforming_str_ = tdd_conf.value("beamforming", "ZF");
  beamforming_algo_ = kBeamformingStr.at(beamforming_str_);
  num_spatial_streams_ = tdd_conf.value("spatial_streams", ue_ant_num_);

  bs_server_addr_ = tdd_conf.value("bs_server_addr", "127.0.0.1");
  bs_rru_addr_ = tdd_conf.value("bs_rru_addr", "127.0.0.1");
  ue_server_addr_ = tdd_conf.value("ue_server_addr", "127.0.0.1");
  ue_rru_addr_ = tdd_conf.value("ue_rru_addr", "127.0.0.1");
  mac_remote_addr_ = tdd_conf.value("mac_remote_addr", "127.0.0.1");
  bs_server_port_ = tdd_conf.value("bs_server_port", 8000);
  bs_rru_port_ = tdd_conf.value("bs_rru_port", 9000);
  ue_rru_port_ = tdd_conf.value("ue_rru_port", 7000);
  ue_server_port_ = tdd_conf.value("ue_server_port", 6000);

  dpdk_num_ports_ = tdd_conf.value("dpdk_num_ports", 1);
  dpdk_port_offset_ = tdd_conf.value("dpdk_port_offset", 0);
  dpdk_mac_addrs_ = tdd_conf.value("dpdk_mac_addrs", "");

  ue_mac_tx_port_ = tdd_conf.value("ue_mac_tx_port", kMacUserRemotePort);
  ue_mac_rx_port_ = tdd_conf.value("ue_mac_rx_port", kMacUserLocalPort);
  bs_mac_tx_port_ = tdd_conf.value("bs_mac_tx_port", kMacBaseRemotePort);
  bs_mac_rx_port_ = tdd_conf.value("bs_mac_rx_port", kMacBaseLocalPort);

  log_listener_addr_ = tdd_conf.value("log_listener_addr", "");
  log_listener_port_ = tdd_conf.value("log_listener_port", 33300);

  log_sc_num_ = tdd_conf.value("log_sc_num", 4);
  log_timestamp_ = tdd_conf.value("log_timestamp", false);

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
  RtAssert(cp_len_ % 16 == 0,
           "cyclic prefix must be a multiple of subcarriers "
           "per cacheline.");
  RtAssert(ofdm_data_num_ % kSCsPerCacheline == 0,
           "ofdm_data_num must be a multiple of subcarriers per cacheline");
  RtAssert(ofdm_data_num_ % kTransposeBlockSize == 0,
           "Transpose block size must divide number of OFDM data subcarriers");
  ofdm_pilot_spacing_ = tdd_conf.value("ofdm_pilot_spacing", 16);
  ofdm_data_start_ = tdd_conf.value("ofdm_data_start",
                                    ((ofdm_ca_num_ - ofdm_data_num_) / 2) /
                                        kSCsPerCacheline * kSCsPerCacheline);
  RtAssert(ofdm_data_start_ % kSCsPerCacheline == 0,
           "ofdm_data_start must be a multiple of subcarriers per cacheline");
  ofdm_data_stop_ = ofdm_data_start_ + ofdm_data_num_;
  tdd_switching_gap_ =
      16 * (static_cast<size_t>(kTddSwitchingTime * rate_ + 15.0) / 16);
  AGORA_LOG_INFO("TDD Switching: %zu samples.\n", tdd_switching_gap_);

  // Build subcarrier map for data ofdm symbols
  ul_symbol_map_.resize(ofdm_data_num_, SubcarrierType::kData);
  dl_symbol_map_.resize(ofdm_data_num_);
  control_symbol_map_.resize(ofdm_data_num_);
  // Maps subcarrier index to data index
  dl_symbol_data_id_.resize(ofdm_data_num_, 0);
  dl_symbol_ctrl_id_.resize(ofdm_data_num_, 0);
  size_t data_idx = 0;
  size_t ctrl_idx = 0;
  for (size_t i = 0; i < ofdm_data_num_; i++) {
    if (i % ofdm_pilot_spacing_ == 0) {  // TODO: make this index configurable
      dl_symbol_map_.at(i) = SubcarrierType::kDMRS;
      control_symbol_map_.at(i) = SubcarrierType::kDMRS;
    } else {
      dl_symbol_map_.at(i) = SubcarrierType::kData;
      dl_symbol_data_id_.at(i) = data_idx++;
      if (i % ofdm_pilot_spacing_ == 1) {
        control_symbol_map_.at(i) = SubcarrierType::kPTRS;
      } else {
        control_symbol_map_.at(i) = SubcarrierType::kData;
        dl_symbol_ctrl_id_.at(i) = ctrl_idx++;
      }
    }
  }

  bigstation_mode_ = tdd_conf.value("bigstation_mode", false);
  freq_orthogonal_pilot_ = tdd_conf.value("freq_orthogonal_pilot", false);
  pilot_sc_group_size_ =
      tdd_conf.value("pilot_sc_group_size", kTransposeBlockSize);
  if (freq_orthogonal_pilot_) {
    RtAssert(pilot_sc_group_size_ == kTransposeBlockSize,
             "In this version, pilot_sc_group_size must be equal to Transpose "
             "Block Size " +
                 std::to_string(kTransposeBlockSize));
    RtAssert(ofdm_data_num_ % pilot_sc_group_size_ == 0,
             "ofdm_data_num must be evenly divided by pilot_sc_group_size " +
                 std::to_string(pilot_sc_group_size_));
    RtAssert(ue_ant_num_ <= pilot_sc_group_size_,
             "user antennas must be no more than pilot_sc_group_size " +
                 std::to_string(pilot_sc_group_size_));
  }

  hw_framer_ = tdd_conf.value("hw_framer", true);
  if (kUseUHD || kUsePureUHD) {
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
  ue_resync_period_ = tdd_conf.value("ue_resync_period", 0);

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
        freq_orthogonal_pilot_ ? kDefaultFreqOrthPilotSymbolNum : ue_ant_num_);

    size_t beacon_symbol_position = tdd_conf.value("beacon_position", SIZE_MAX);

    ul_data_symbol_num_perframe =
        tdd_conf.value("ul_symbol_num_perframe", ul_data_symbol_num_perframe);

    if (ul_data_symbol_num_perframe == 0) {
      ul_data_symbol_start = 0;
    } else {
      // Start position of the first UL symbol
      ul_data_symbol_start =
          tdd_conf.value("ul_data_symbol_start", ul_data_symbol_start);
    }
    const size_t ul_data_symbol_stop =
        ul_data_symbol_start + ul_data_symbol_num_perframe;

    //Dl symbols
    dl_data_symbol_num_perframe =
        tdd_conf.value("dl_symbol_num_perframe", dl_data_symbol_num_perframe);

    if (dl_data_symbol_num_perframe == 0) {
      dl_data_symbol_start = 0;
    } else {
      // Start position of the first DL symbol
      dl_data_symbol_start =
          tdd_conf.value("dl_data_symbol_start", dl_data_symbol_start);
    }
    const size_t dl_data_symbol_stop =
        dl_data_symbol_start + dl_data_symbol_num_perframe;

    if ((ul_data_symbol_num_perframe + dl_data_symbol_num_perframe +
         pilot_symbol_num_perframe) > symbol_num_perframe) {
      AGORA_LOG_ERROR(
          "!!!!! Invalid configuration pilot + ul + dl exceeds total symbols "
          "!!!!!\n");
      AGORA_LOG_ERROR(
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
      AGORA_LOG_ERROR(
          "!!!!! Invalid configuration ul and dl symbol overlap detected "
          "!!!!!\n");
      AGORA_LOG_ERROR(
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
    if ((dl_data_symbol_num_perframe > 0) &&
        (dl_data_symbol_start <= ul_data_symbol_start)) {
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
    AGORA_LOG_SYMBOL(
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
    // ( )PGGGG1111111111GGGG2222222222GGGG
    if (first_sym_start > 0) {
      const int guard_symbols = first_sym_start - sched.length();
      if (guard_symbols > 0) {
        sched.append(guard_symbols, 'G');
      }
      if (first_sym_count > 0) {
        sched.append(first_sym_count, first_sym);
      }
    }
    if (second_sym_start > 0) {
      const int guard_symbols = second_sym_start - sched.length();
      if (guard_symbols > 0) {
        sched.append(guard_symbols, 'G');
      }
      if (second_sym_count > 0) {
        sched.append(second_sym_count, second_sym);
      }
    }
    const int guard_symbols = symbol_num_perframe - sched.length();
    if (guard_symbols > 0) {
      sched.append(guard_symbols, 'G');
    }

    // Add the beacon
    if (beacon_symbol_position < sched.length()) {
      if (sched.at(beacon_symbol_position) != 'G') {
        AGORA_LOG_ERROR("Invalid beacon location %zu replacing %c\n",
                        beacon_symbol_position,
                        sched.at(beacon_symbol_position));
        throw std::runtime_error("Invalid Frame Configuration");
      }
      sched.replace(beacon_symbol_position, 1, "B");
    }
    frame_ = FrameStats(sched);
  } else {
    json jframes = tdd_conf.value("frame_schedule", json::array());

    // Only allow 1 unique frame type
    assert(jframes.size() == 1);
    std::string frame = jframes.at(0).get<std::string>();
    /*
    If an apostrophe delimiter is found in the frame string, execute logic to
    convert a subframe formated frame into the symbol formated frame that Agora
    is designed to handle.
    */
    if (frame.find(",") != std::string::npos) {
      std::vector<std::string> flex_formats =
          tdd_conf.value("flex_formats", json::array());
      FiveGConfig fivegconfig = FiveGConfig(tdd_conf, ue_ant_num_);
      frame = fivegconfig.FiveGFormat();
      rate_ = fivegconfig.SamplingRate();
      ofdm_ca_num_ = fivegconfig.FftSize();
      ofdm_data_num_ = fivegconfig.OfdmDataNum();
      ofdm_data_start_ = fivegconfig.OfdmDataStart();
      ofdm_data_stop_ = ofdm_data_start_ + ofdm_data_num_;
    }
    frame_ = FrameStats(frame);
  }
  AGORA_LOG_INFO("Config: Frame schedule %s (%zu symbols)\n",
                 frame_.FrameIdentifier().c_str(), frame_.NumTotalSyms());

  if (frame_.IsRecCalEnabled()) {
    RtAssert(bf_ant_num_ >= frame_.NumDLCalSyms(),
             "Too many DL Cal symbols for the number of base station antennas");

    RtAssert(((bf_ant_num_ % frame_.NumDLCalSyms()) == 0),
             "Number of Downlink calibration symbols per frame must complete "
             "calibration on frame boundary!");
  }

  // Check for frame validity.
  // We should remove the restriction of the beacon symbol placement when tested
  // more thoroughly
  if (((frame_.NumBeaconSyms() > 1)) ||
      ((frame_.NumBeaconSyms() == 1) && (frame_.GetBeaconSymbolLast() > 1))) {
    AGORA_LOG_ERROR("Invalid beacon symbol placement\n");
    throw std::runtime_error("Invalid beacon symbol placement");
  }

  // client_dl_pilot_sym uses the first x 'D' symbols for downlink channel
  // estimation for each user.
  size_t client_dl_pilot_syms = tdd_conf.value("client_dl_pilot_syms", 0);
  RtAssert(client_dl_pilot_syms <= frame_.NumDLSyms(),
           "Number of DL pilot symbol exceeds number of DL symbols!");
  // client_ul_pilot_sym uses the first x 'U' symbols for downlink channel
  // estimation for each user.
  size_t client_ul_pilot_syms = tdd_conf.value("client_ul_pilot_syms", 0);
  RtAssert(client_ul_pilot_syms <= frame_.NumULSyms(),
           "Number of UL pilot symbol exceeds number of UL symbols!");

  frame_.SetClientPilotSyms(client_ul_pilot_syms, client_dl_pilot_syms);

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

  auto corr_scale = tdd_conf.value("corr_scale", json::array());
  if (corr_scale.empty()) {
    cl_corr_scale_.resize(ue_num_, 1.f);
  } else {
    RtAssert(corr_scale.size() == ue_num_,
             "corr_scale size must be same as the number of clients!");
    cl_corr_scale_.assign(corr_scale.begin(), corr_scale.end());
  }

  if (std::filesystem::is_directory(kExperimentFilepath) == false) {
    std::filesystem::create_directory(kExperimentFilepath);
  }

  if (std::filesystem::is_directory(kLogFilepath) == false) {
    std::filesystem::create_directory(kLogFilepath);
  }

  // set trace file path
  const std::string ul_present_str = (frame_.NumULSyms() > 0 ? "uplink-" : "");
  const std::string dl_present_str =
      (frame_.NumDLSyms() > 0 ? "downlink-" : "");
  std::string filename =
      kLogFilepath + "trace-" + ul_present_str + dl_present_str + timestamp_ +
      "_" + std::to_string(num_cells_) + "_" + std::to_string(BsAntNum()) +
      "x" + std::to_string(UeAntTotal()) + ".hdf5";
  trace_file_ = tdd_conf.value("trace_file", filename);

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
  beam_thread_num_ = worker_thread_num_ - fft_thread_num_ - demul_thread_num_ -
                     decode_thread_num_;

  demul_block_size_ = tdd_conf.value("demul_block_size", 48);
  RtAssert(demul_block_size_ % kSCsPerCacheline == 0,
           "Demodulation block size must be a multiple of subcarriers per "
           "cacheline");
  RtAssert(
      demul_block_size_ % kTransposeBlockSize == 0,
      "Demodulation block size must be a multiple of transpose block size");
  demul_events_per_symbol_ = 1 + (ofdm_data_num_ - 1) / demul_block_size_;

  beam_block_size_ = tdd_conf.value("beam_block_size", 1);
  if (freq_orthogonal_pilot_) {
    if (beam_block_size_ == 1) {
      AGORA_LOG_INFO("Setting beam_block_size to pilot_sc_group_size %zu\n",
                     pilot_sc_group_size_);
      beam_block_size_ = pilot_sc_group_size_;
    }

    //Set beam block size to the pilot sc group size so events arn't generated for the redundant sc
    if ((beam_block_size_ % pilot_sc_group_size_) != 0) {
      AGORA_LOG_WARN(
          "beam_block_size(%zu) is not a multiple of pilot_sc_group_size(%zu). "
          "Efficiency will be decreased.  Please consider updating your "
          "settings\n",
          beam_block_size_, pilot_sc_group_size_);
    }
  }
  beam_events_per_symbol_ = 1 + (ofdm_data_num_ - 1) / beam_block_size_;

  fft_block_size_ = tdd_conf.value("fft_block_size", 1);
  fft_block_size_ = std::max(fft_block_size_, num_channels_);
  RtAssert(bs_ant_num_ % fft_block_size_ == 0,
           "FFT block size is set to an invalid value - all rx symbols per "
           "frame must fit inside an fft block");

  encode_block_size_ = tdd_conf.value("encode_block_size", 1);

  noise_level_ = tdd_conf.value("noise_level", 0.03);  // default: 30 dB
  AGORA_LOG_SYMBOL("Noise level: %.2f\n", noise_level_);

  // Scrambler and descrambler configurations
  scramble_enabled_ = tdd_conf.value("wlan_scrambler", true);

  // LDPC Coding and Modulation configurations
  ul_mcs_params_ = this->Parse(tdd_conf, "ul_mcs");
  this->UpdateUlMCS(ul_mcs_params_);

  dl_mcs_params_ = this->Parse(tdd_conf, "dl_mcs");
  this->UpdateDlMCS(dl_mcs_params_);
  this->DumpMcsInfo();
  this->UpdateCtrlMCS();

  fft_in_rru_ = tdd_conf.value("fft_in_rru", false);

  samps_per_symbol_ =
      ofdm_tx_zero_prefix_ + ofdm_ca_num_ + cp_len_ + ofdm_tx_zero_postfix_;
  packet_length_ =
      Packet::kOffsetOfData + ((kUse12BitIQ ? 3 : 4) * samps_per_symbol_);
  dl_packet_length_ = Packet::kOffsetOfData + (samps_per_symbol_ * 4);

  //Don't check for jumbo frames when using the hardware, this might be temp
  if (!kUseArgos) {
    RtAssert(packet_length_ < 9000,
             "Packet size must be smaller than jumbo frame");
  }

  ul_num_bytes_per_cb_ = ul_ldpc_config_.NumCbLen() / 8;
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

  dl_num_bytes_per_cb_ = dl_ldpc_config_.NumCbLen() / 8;
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

  this->running_.store(true);
  /* 12 bit samples x2 for I + Q */
  static const size_t kBitsPerSample = 12 * 2;
  const double bit_rate_mbps = (rate_ * kBitsPerSample) / 1e6;
  //For framer mode, we can ignore the Beacon
  //Double count the UlCal and DLCal to simplify things
  //Peak network traffic is the bit rate for 1 symbol, for non-hardware framer mode
  //the device can generate 2*rate_ traffic (for each tx symbol)
  const size_t bs_tx_symbols =
      frame_.NumDLSyms() + frame_.NumDLCalSyms() + frame_.NumULCalSyms();
  const size_t bs_rx_symbols = frame_.NumPilotSyms() + frame_.NumULSyms() +
                               frame_.NumDLCalSyms() + frame_.NumULCalSyms();
  const double per_bs_radio_traffic =
      ((static_cast<double>(bs_tx_symbols + bs_rx_symbols)) /
       frame_.NumTotalSyms()) *
      bit_rate_mbps;

  const size_t ue_tx_symbols = frame_.NumULSyms() + frame_.NumPilotSyms();
  //Rx all symbols, Tx the tx symbols (ul + pilots)
  const double per_ue_radio_traffic =
      (bit_rate_mbps *
       (static_cast<double>(ue_tx_symbols) / frame_.NumTotalSyms())) +
      bit_rate_mbps;

  AGORA_LOG_INFO(
      "Config: %zu BS antennas, %zu UE antennas, %zu pilot symbols per "
      "frame,\n"
      "\t%zu uplink data symbols per frame, %zu downlink data symbols "
      "per frame,\n"
      "\t%zu OFDM subcarriers (%zu data subcarriers),\n"
      "\tUL modulation %s, DL modulation %s, Beamforming %s, \n"
      "\t%zu UL codeblocks per symbol, "
      "%zu UL bytes per code block,\n"
      "\t%zu DL codeblocks per symbol, %zu DL bytes per code block,\n"
      "\t%zu UL MAC data bytes per frame, %zu UL MAC bytes per frame,\n"
      "\t%zu DL MAC data bytes per frame, %zu DL MAC bytes per frame,\n"
      "\tFrame time %.3f usec\n"
      "Uplink Max Mac data per-user tp (Mbps) %.3f\n"
      "Downlink Max Mac data per-user tp (Mbps) %.3f\n"
      "Radio Network Traffic Peak (Mbps): %.3f\n"
      "Radio Network Traffic Avg  (Mbps): %.3f\n"
      "Basestation Network Traffic Peak (Mbps): %.3f\n"
      "Basestation Network Traffic Avg  (Mbps): %.3f\n"
      "UE Network Traffic Peak (Mbps): %.3f\n"
      "UE Network Traffic Avg  (Mbps): %.3f\n"
      "All UEs Network Traffic Avg (Mbps): %.3f\n"
      "All UEs Network Traffic Avg (Mbps): %.3f\n",
      bs_ant_num_, ue_ant_num_, frame_.NumPilotSyms(), frame_.NumULSyms(),
      frame_.NumDLSyms(), ofdm_ca_num_, ofdm_data_num_, ul_modulation_.c_str(),
      dl_modulation_.c_str(), beamforming_str_.c_str(),
      ul_ldpc_config_.NumBlocksInSymbol(), ul_num_bytes_per_cb_,
      dl_ldpc_config_.NumBlocksInSymbol(), dl_num_bytes_per_cb_,
      ul_mac_data_bytes_num_perframe_, ul_mac_bytes_num_perframe_,
      dl_mac_data_bytes_num_perframe_, dl_mac_bytes_num_perframe_,
      this->GetFrameDurationSec() * 1e6,
      (ul_mac_data_bytes_num_perframe_ * 8.0f) /
          (this->GetFrameDurationSec() * 1e6),
      (dl_mac_data_bytes_num_perframe_ * 8.0f) /
          (this->GetFrameDurationSec() * 1e6),
      bit_rate_mbps, per_bs_radio_traffic, bit_rate_mbps * bs_ant_num_,
      per_bs_radio_traffic * bs_ant_num_, 2 * bit_rate_mbps,
      per_ue_radio_traffic, 2 * bit_rate_mbps * ue_ant_num_,
      per_ue_radio_traffic * ue_ant_num_);

  if (frame_.IsRecCalEnabled()) {
    AGORA_LOG_INFO(
        "Reciprocal Calibration Enabled.  Full calibration data ready every "
        "%zu frame(s) using %zu symbols per frame\n",
        RecipCalFrameCnt(), frame_.NumDLCalSyms());
  }

  Print();
}

json Config::Parse(const json& in_json, const std::string& json_handle) {
  json out_json;
  std::stringstream ss;
  ss << in_json.value(json_handle, out_json);
  out_json = json::parse(ss);
  if (out_json == nullptr) {
    out_json = json::object();
  }
  ss.str(std::string());
  ss.clear();
  return out_json;
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

void Config::UpdateUlMCS(const json& ul_mcs) {
  if (ul_mcs.find("mcs_index") == ul_mcs.end()) {
    ul_modulation_ = ul_mcs.value("modulation", "16QAM");
    ul_mod_order_bits_ = kModulStringMap.at(ul_modulation_);

    double ul_code_rate_usr = ul_mcs.value("code_rate", 0.333);
    size_t code_rate_int =
        static_cast<size_t>(std::round(ul_code_rate_usr * 1024.0));

    ul_mcs_index_ = CommsLib::GetMcsIndex(ul_mod_order_bits_, code_rate_int);
    ul_code_rate_ = GetCodeRate(ul_mcs_index_);
    if (ul_code_rate_ / 1024.0 != ul_code_rate_usr) {
      AGORA_LOG_WARN(
          "Rounded the user-defined uplink code rate to the closest standard "
          "rate %zu/1024.\n",
          ul_code_rate_);
    }
  } else {
    ul_mcs_index_ = ul_mcs.value("mcs_index", 10);  // 16QAM, 340/1024
    ul_mod_order_bits_ = GetModOrderBits(ul_mcs_index_);
    ul_modulation_ = MapModToStr(ul_mod_order_bits_);
    ul_code_rate_ = GetCodeRate(ul_mcs_index_);
    ul_modulation_ = MapModToStr(ul_mod_order_bits_);
  }
  InitModulationTable(this->ul_mod_table_, ul_mod_order_bits_);

  // TODO: find the optimal base_graph
  uint16_t base_graph = ul_mcs.value("base_graph", 1);
  bool early_term = ul_mcs.value("earlyTermination", true);
  int16_t max_decoder_iter = ul_mcs.value("decoderIter", 5);

  size_t zc = SelectZc(base_graph, ul_code_rate_, ul_mod_order_bits_,
                       ofdm_data_num_, kCbPerSymbol, "uplink");

  // Always positive since ul_code_rate is smaller than 1024
  size_t num_rows =
      static_cast<size_t>(
          std::round(1024.0 * LdpcNumInputCols(base_graph) / ul_code_rate_)) -
      (LdpcNumInputCols(base_graph) - 2);

  uint32_t num_cb_len = LdpcNumInputBits(base_graph, zc);
  uint32_t num_cb_codew_len = LdpcNumEncodedBits(base_graph, zc, num_rows);
  ul_ldpc_config_ = LDPCconfig(base_graph, zc, max_decoder_iter, early_term,
                               num_cb_len, num_cb_codew_len, num_rows, 0);

  ul_ldpc_config_.NumBlocksInSymbol((ofdm_data_num_ * ul_mod_order_bits_) /
                                    ul_ldpc_config_.NumCbCodewLen());
  RtAssert(
      (frame_.NumULSyms() == 0) || (ul_ldpc_config_.NumBlocksInSymbol() > 0),
      "Uplink LDPC expansion factor is too large for number of OFDM data "
      "subcarriers.");
}

void Config::UpdateDlMCS(const json& dl_mcs) {
  if (dl_mcs.find("mcs_index") == dl_mcs.end()) {
    dl_modulation_ = dl_mcs.value("modulation", "16QAM");
    dl_mod_order_bits_ = kModulStringMap.at(dl_modulation_);

    double dl_code_rate_usr = dl_mcs.value("code_rate", 0.333);
    size_t code_rate_int =
        static_cast<size_t>(std::round(dl_code_rate_usr * 1024.0));
    dl_mcs_index_ = CommsLib::GetMcsIndex(dl_mod_order_bits_, code_rate_int);
    dl_code_rate_ = GetCodeRate(dl_mcs_index_);
    if (dl_code_rate_ / 1024.0 != dl_code_rate_usr) {
      AGORA_LOG_WARN(
          "Rounded the user-defined downlink code rate to the closest standard "
          "rate %zu/1024.\n",
          dl_code_rate_);
    }
  } else {
    dl_mcs_index_ = dl_mcs.value("mcs_index", 10);  // 16QAM, 340/1024
    dl_mod_order_bits_ = GetModOrderBits(dl_mcs_index_);
    dl_modulation_ = MapModToStr(dl_mod_order_bits_);
    dl_code_rate_ = GetCodeRate(dl_mcs_index_);
    dl_modulation_ = MapModToStr(dl_mod_order_bits_);
  }
  InitModulationTable(this->dl_mod_table_, dl_mod_order_bits_);

  // TODO: find the optimal base_graph
  uint16_t base_graph = dl_mcs.value("base_graph", 1);
  bool early_term = dl_mcs.value("earlyTermination", true);
  int16_t max_decoder_iter = dl_mcs.value("decoderIter", 5);

  size_t zc = SelectZc(base_graph, dl_code_rate_, dl_mod_order_bits_,
                       GetOFDMDataNum(), kCbPerSymbol, "downlink");

  // Always positive since dl_code_rate is smaller than 1024
  size_t num_rows =
      static_cast<size_t>(
          std::round(1024.0 * LdpcNumInputCols(base_graph) / dl_code_rate_)) -
      (LdpcNumInputCols(base_graph) - 2);

  uint32_t num_cb_len = LdpcNumInputBits(base_graph, zc);
  uint32_t num_cb_codew_len = LdpcNumEncodedBits(base_graph, zc, num_rows);
  dl_ldpc_config_ = LDPCconfig(base_graph, zc, max_decoder_iter, early_term,
                               num_cb_len, num_cb_codew_len, num_rows, 0);

  dl_ldpc_config_.NumBlocksInSymbol((GetOFDMDataNum() * dl_mod_order_bits_) /
                                    dl_ldpc_config_.NumCbCodewLen());
  RtAssert(
      this->frame_.NumDLSyms() == 0 || dl_ldpc_config_.NumBlocksInSymbol() > 0,
      "Downlink LDPC expansion factor is too large for number of OFDM data "
      "subcarriers.");
}

void Config::UpdateCtrlMCS() {
  if (this->frame_.NumDlControlSyms() > 0) {
    const size_t dl_bcast_mcs_index = kControlMCS;
    const size_t bcast_base_graph =
        1;  // TODO: For MCS < 5, base_graph 1 doesn't work
    dl_bcast_mod_order_bits_ = GetModOrderBits(dl_bcast_mcs_index);
    const size_t dl_bcast_code_rate = GetCodeRate(dl_bcast_mcs_index);
    std::string dl_bcast_modulation = MapModToStr(dl_bcast_mod_order_bits_);
    const int16_t max_decoder_iter = 5;
    size_t bcast_zc =
        SelectZc(bcast_base_graph, dl_bcast_code_rate, dl_bcast_mod_order_bits_,
                 this->GetOFDMCtrlNum(), kCbPerSymbol, "downlink broadcast");

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
        (GetOFDMCtrlNum() * dl_bcast_mod_order_bits_) /
        dl_bcast_ldpc_config_.NumCbCodewLen());
    RtAssert(dl_bcast_ldpc_config_.NumBlocksInSymbol() > 0,
             "Downlink Broadcast LDPC expansion factor is too large for number "
             "of OFDM data "
             "subcarriers.");
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
        dl_bcast_ldpc_config_.NumRows(), dl_bcast_modulation.c_str());
  }
}

void Config::DumpMcsInfo() {
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
}

void Config::GenPilots() {
  if ((kUseArgos == true) || (kUseUHD == true) || (kUsePureUHD == true)) {
    std::vector<std::vector<double>> gold_ifft =
        CommsLib::GetSequence(128, CommsLib::kGoldIfft);
    std::vector<std::complex<int16_t>> gold_ifft_ci16 =
        Utils::DoubleToCint16(gold_ifft);
    for (size_t i = 0; i < 128; i++) {
      this->gold_cf32_.emplace_back(gold_ifft[0][i], gold_ifft[1][i]);
    }

    // Populate gold sequence (two reps, 128 each)
    int gold_reps = 2;
    for (int i = 0; i < gold_reps; i++) {
      this->beacon_ci16_.insert(this->beacon_ci16_.end(),
                                gold_ifft_ci16.begin(), gold_ifft_ci16.end());
    }

    std::vector<std::vector<double>> sts_seq =
        CommsLib::GetSequence(0, CommsLib::kStsSeq);
    std::vector<std::complex<int16_t>> sts_seq_ci16 =
        Utils::DoubleToCint16(sts_seq);

    auto beacon_sym_size = beacon_ci16_.size() + ofdm_tx_zero_prefix_ +
                           tdd_switching_gap_ + kBeaconTxZeroPostfix;
    int sts_reps = (samps_per_symbol_ - beacon_sym_size) / sts_seq_ci16.size();
    if (this->samps_per_symbol_ <
        (beacon_sym_size + sts_reps * sts_seq_ci16.size())) {
      std::string msg = "Beacon content does not fit into one symbol size ";
      msg += std::to_string(samps_per_symbol_);
      throw std::invalid_argument(msg);
    } else {
      AGORA_LOG_INFO(
          "Forming the beacon symbol with %zu reps of STS and 2 reps of gold "
          "code.\n",
          sts_reps);
    }

    // Populate STS (stsReps repetitions)
    for (int i = 0; i < sts_reps; i++) {
      this->beacon_ci16_.insert(this->beacon_ci16_.begin(),
                                sts_seq_ci16.begin(), sts_seq_ci16.end());
    }

    this->beacon_len_ = this->beacon_ci16_.size();

    this->beacon_ = Utils::Cint16ToUint32(this->beacon_ci16_, false, "QI");
    this->coeffs_ = Utils::Cint16ToUint32(gold_ifft_ci16, true, "QI");

    // Add addition padding for beacon sent from host
    std::vector<std::complex<int16_t>> pre_beacon(
        this->ofdm_tx_zero_prefix_ + tdd_switching_gap_, 0);
    this->beacon_ci16_.insert(this->beacon_ci16_.begin(), pre_beacon.begin(),
                              pre_beacon.end());

    int post_pad = samps_per_symbol_ - this->beacon_ci16_.size();
    std::vector<std::complex<int16_t>> post_beacon(post_pad, 0);
    this->beacon_ci16_.insert(this->beacon_ci16_.end(), post_beacon.begin(),
                              post_beacon.end());

    if (kDebugPrintPilot) {
      std::cout << "beacon_slot = [";
      for (size_t i = 0; i < beacon_ci16_.size(); i++) {
        std::cout << beacon_ci16_.at(i).real() << "+"
                  << beacon_ci16_.at(i).imag() << "i ";
      }
      std::cout << "];" << std::endl;
    }
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

  RtAssert(pilot_ifft_ == nullptr, "pilot_ifft_ should be null");
  AllocBuffer1d(&pilot_ifft_, this->ofdm_ca_num_,
                Agora_memory::Alignment_t::kAlign64, 1);

  for (size_t j = 0; j < ofdm_data_num_; j++) {
    // FFT Shift
    const size_t k = j + ofdm_data_start_ >= ofdm_ca_num_ / 2
                         ? j + ofdm_data_start_ - ofdm_ca_num_ / 2
                         : j + ofdm_data_start_ + ofdm_ca_num_ / 2;
    pilot_ifft_[k] = this->pilots_[j];
  }
  CommsLib::IFFT(pilot_ifft_, this->ofdm_ca_num_, false);

  // Generate UE-specific pilots based on Zadoff-Chu sequence for phase tracking
  this->ue_specific_pilot_.Malloc(this->ue_ant_num_, this->ofdm_data_num_,
                                  Agora_memory::Alignment_t::kAlign64);
  this->ue_specific_pilot_t_.Calloc(this->ue_ant_num_, this->samps_per_symbol_,
                                    Agora_memory::Alignment_t::kAlign64);

  ue_pilot_ifft_.Calloc(this->ue_ant_num_, this->ofdm_ca_num_,
                        Agora_memory::Alignment_t::kAlign64);
  for (size_t i = 0; i < ue_ant_num_; i++) {
    auto zc_ue_pilot_i = CommsLib::SeqCyclicShift(
        zc_seq,
        (i + this->ue_ant_offset_) * (float)M_PI / 6);  // LTE DMRS
    for (size_t j = 0; j < this->ofdm_data_num_; j++) {
      this->ue_specific_pilot_[i][j] = {zc_ue_pilot_i[j].real(),
                                        zc_ue_pilot_i[j].imag()};
      // FFT Shift
      const size_t k = j + ofdm_data_start_ >= ofdm_ca_num_ / 2
                           ? j + ofdm_data_start_ - ofdm_ca_num_ / 2
                           : j + ofdm_data_start_ + ofdm_ca_num_ / 2;
      ue_pilot_ifft_[i][k] = this->ue_specific_pilot_[i][j];
    }
    CommsLib::IFFT(ue_pilot_ifft_[i], ofdm_ca_num_, false);
  }
}

void Config::LoadUplinkData() {
  if (this->frame_.NumUlDataSyms() > 0) {
    const size_t ul_num_bytes_per_ue_pad =
        Roundup<64>(this->ul_num_bytes_per_cb_) *
        this->ul_ldpc_config_.NumBlocksInSymbol();
    // Uplink LDPC input bits
    ul_bits_.Calloc(this->frame_.NumUlDataSyms(),
                    ul_num_bytes_per_ue_pad * this->ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    const std::string ul_data_file =
        kExperimentFilepath + kUlLdpcDataPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    AGORA_LOG_FRAME("Config: Reading uplink data bits from %s\n",
                    ul_data_file.c_str());

    // \todo assumes one code block per symbol
    const size_t code_block_in_symbol_i = 0u;
    size_t seek_offset = 0u;
    for (size_t i = 0; i < this->frame_.NumUlDataSyms(); i++) {
      seek_offset +=
          ul_data_bytes_num_persymbol_ * this->ue_ant_offset_ * sizeof(int8_t);
      for (size_t j = 0; j < this->ue_ant_num_; j++) {
        int8_t* ul_bits_ptr = this->GetInfoBits(
            this->ul_bits_, Direction::kUplink, i, j, code_block_in_symbol_i);
        Utils::ReadBinaryFile(ul_data_file, sizeof(int8_t),
                              ul_data_bytes_num_persymbol_, seek_offset,
                              ul_bits_ptr);
        seek_offset += ul_data_bytes_num_persymbol_ * sizeof(int8_t);
      }
      seek_offset +=
          ul_data_bytes_num_persymbol_ *
          (this->ue_ant_total_ - this->ue_ant_offset_ - this->ue_ant_num_) *
          sizeof(int8_t);
    }

    // Uplink modulation input bits
    ul_mod_bits_.Calloc(this->frame_.NumUlDataSyms(),
                        Roundup<64>(this->ofdm_data_num_) * this->ue_ant_num_,
                        Agora_memory::Alignment_t::kAlign32);
    const std::string ul_mod_data_file =
        kExperimentFilepath + kUlModDataPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    // reset seek offset for new file read
    seek_offset = 0;
    const size_t subcarr_i = 0u;
    for (size_t i = 0; i < this->frame_.NumUlDataSyms(); i++) {
      seek_offset += ofdm_data_num_ * this->ue_ant_offset_ * sizeof(int8_t);
      for (size_t j = 0; j < this->ue_ant_num_; j++) {
        int8_t* ul_mod_data_ptr = this->GetModBitsBuf(
            ul_mod_bits_, Direction::kUplink, 0u, i, j, subcarr_i);
        Utils::ReadBinaryFile(ul_mod_data_file, sizeof(int8_t), ofdm_data_num_,
                              seek_offset, ul_mod_data_ptr);
        seek_offset += ofdm_data_num_ * sizeof(int8_t);
      }
      seek_offset +=
          ofdm_data_num_ *
          (this->ue_ant_total_ - this->ue_ant_offset_ - this->ue_ant_num_) *
          sizeof(int8_t);
    }
  }
}

void Config::LoadDownlinkData() {
  if (this->frame_.NumDlDataSyms() > 0) {
    const size_t dl_num_bytes_per_ue_pad =
        Roundup<64>(this->dl_num_bytes_per_cb_) *
        this->dl_ldpc_config_.NumBlocksInSymbol();
    // Downlink LDPC input bits
    dl_bits_.Calloc(this->frame_.NumDlDataSyms(),
                    dl_num_bytes_per_ue_pad * this->ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    const std::string dl_data_file =
        kExperimentFilepath + kDlLdpcDataPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    AGORA_LOG_FRAME("Config: Reading downlink data bits from %s\n",
                    dl_data_file.c_str());

    // \todo assumes one code block per symbol
    const size_t code_block_in_symbol_i = 0;
    size_t seek_offset = 0;
    for (size_t i = 0; i < this->frame_.NumDlDataSyms(); i++) {
      seek_offset +=
          dl_data_bytes_num_persymbol_ * this->ue_ant_offset_ * sizeof(int8_t);
      for (size_t j = 0; j < this->ue_ant_num_; j++) {
        int8_t* dl_bits_ptr = this->GetInfoBits(
            this->dl_bits_, Direction::kDownlink, i, j, code_block_in_symbol_i);
        Utils::ReadBinaryFile(
            dl_data_file, sizeof(int8_t), dl_data_bytes_num_persymbol_,
            seek_offset,
            dl_bits_ptr);  //this->dl_bits_[i] + (j * dl_num_bytes_per_ue_pad));
        seek_offset += dl_data_bytes_num_persymbol_ * sizeof(int8_t);
      }
      seek_offset +=
          dl_data_bytes_num_persymbol_ *
          (this->ue_ant_total_ - this->ue_ant_offset_ - this->ue_ant_num_) *
          sizeof(int8_t);
    }

    // Downlink modulation input bits
    dl_mod_bits_.Calloc(this->frame_.NumDlDataSyms(),
                        Roundup<64>(this->GetOFDMDataNum()) * ue_ant_num_,
                        Agora_memory::Alignment_t::kAlign32);
    const std::string dl_mod_data_file =
        kExperimentFilepath + kDlModDataPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    // reset seek offset for new file read
    seek_offset = 0;
    const size_t subcarr_i = 0u;
    for (size_t i = 0; i < this->frame_.NumDlDataSyms(); i++) {
      seek_offset +=
          this->GetOFDMDataNum() * this->ue_ant_offset_ * sizeof(int8_t);
      for (size_t j = 0; j < this->ue_ant_num_; j++) {
        int8_t* dl_mod_data_ptr = this->GetModBitsBuf(
            dl_mod_bits_, Direction::kDownlink, 0, i, j, subcarr_i);
        Utils::ReadBinaryFile(dl_mod_data_file, sizeof(int8_t),
                              this->GetOFDMDataNum(), seek_offset,
                              dl_mod_data_ptr);
        seek_offset += this->GetOFDMDataNum() * sizeof(int8_t);
      }
      seek_offset +=
          this->GetOFDMDataNum() *
          (this->ue_ant_total_ - this->ue_ant_offset_ - this->ue_ant_num_) *
          sizeof(int8_t);
    }
  }
}

void Config::LoadTestVectors() {
  this->GenPilots();

  // Freq-domain uplink symbols
  this->LoadUplinkData();
  Table<complex_float> ul_iq_ifft;
  if (this->frame_.NumUlDataSyms() > 0) {
    ul_iq_ifft.Calloc(this->frame_.NumUlDataSyms(),
                      this->ofdm_ca_num_ * this->ue_ant_num_,
                      Agora_memory::Alignment_t::kAlign64);
    ul_iq_f_.Calloc(this->frame_.NumUlDataSyms(),
                    this->ofdm_data_num_ * this->ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    ul_iq_t_.Calloc(this->frame_.NumUlDataSyms(),
                    this->samps_per_symbol_ * this->ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    const std::string ul_ifft_data_file =
        kExperimentFilepath + kUlIfftPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    size_t seek_offset = 0;
    for (size_t i = 0; i < this->frame_.NumUlDataSyms(); i++) {
      seek_offset +=
          ofdm_ca_num_ * this->ue_ant_offset_ * sizeof(complex_float);
      for (size_t j = 0; j < this->ue_ant_num_; j++) {
        Utils::ReadBinaryFile(ul_ifft_data_file, sizeof(complex_float),
                              ofdm_ca_num_, seek_offset,
                              &ul_iq_ifft[i][j * ofdm_ca_num_]);
        std::memcpy(&ul_iq_f_[i][j * ofdm_data_num_],
                    &ul_iq_ifft[i][j * ofdm_ca_num_ + ofdm_data_start_],
                    ofdm_data_num_ * sizeof(complex_float));
        CommsLib::FFTShift(&ul_iq_ifft[i][j * ofdm_ca_num_], ofdm_ca_num_);
        CommsLib::IFFT(&ul_iq_ifft[i][j * ofdm_ca_num_], ofdm_ca_num_, false);
        seek_offset += ofdm_ca_num_ * sizeof(complex_float);
      }
      seek_offset +=
          ofdm_ca_num_ *
          (this->ue_ant_total_ - this->ue_ant_offset_ - this->ue_ant_num_) *
          sizeof(complex_float);
    }
  }

  // Generate freq-domain downlink symbols
  this->LoadDownlinkData();
  Table<complex_float> dl_iq_ifft;
  if (this->frame_.NumDlDataSyms() > 0) {
    dl_iq_ifft.Calloc(this->frame_.NumDlDataSyms(),
                      this->ofdm_ca_num_ * this->ue_ant_num_,
                      Agora_memory::Alignment_t::kAlign64);
    dl_iq_f_.Calloc(this->frame_.NumDlDataSyms(), ofdm_data_num_ * ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    dl_iq_t_.Calloc(this->frame_.NumDlDataSyms(),
                    this->samps_per_symbol_ * this->ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    const std::string dl_ifft_data_file =
        kExperimentFilepath + kDlIfftPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    size_t seek_offset = 0;
    for (size_t i = 0; i < this->frame_.NumDlDataSyms(); i++) {
      seek_offset +=
          ofdm_ca_num_ * this->ue_ant_offset_ * sizeof(complex_float);
      for (size_t j = 0; j < this->ue_ant_num_; j++) {
        Utils::ReadBinaryFile(dl_ifft_data_file, sizeof(complex_float),
                              ofdm_ca_num_, seek_offset,
                              &dl_iq_ifft[i][j * ofdm_ca_num_]);
        std::memcpy(&dl_iq_f_[i][j * ofdm_data_num_],
                    &dl_iq_ifft[i][j * ofdm_ca_num_ + ofdm_data_start_],
                    ofdm_data_num_ * sizeof(complex_float));
        CommsLib::FFTShift(&dl_iq_ifft[i][j * ofdm_ca_num_], ofdm_ca_num_);
        CommsLib::IFFT(&dl_iq_ifft[i][j * ofdm_ca_num_], ofdm_ca_num_, false);
        seek_offset += ofdm_ca_num_ * sizeof(complex_float);
      }
      seek_offset +=
          ofdm_ca_num_ *
          (this->ue_ant_total_ - this->ue_ant_offset_ - this->ue_ant_num_) *
          sizeof(complex_float);
    }
  }

  // Find normalization factor through searching for max value in IFFT results

  float ul_max_mag =
      (this->frame_.NumUlDataSyms() > 0)
          ? CommsLib::FindMaxAbs(ul_iq_ifft, this->frame_.NumUlDataSyms(),
                                 this->ue_ant_num_ * this->ofdm_ca_num_)
          : 0;
  float dl_max_mag =
      (this->frame_.NumDlDataSyms() > 0)
          ? CommsLib::FindMaxAbs(dl_iq_ifft, this->frame_.NumDlDataSyms(),
                                 this->ue_ant_num_ * this->ofdm_ca_num_)
          : 0;
  float ue_pilot_max_mag = CommsLib::FindMaxAbs(
      ue_pilot_ifft_, this->ue_ant_num_, this->ofdm_ca_num_);
  float pilot_max_mag = CommsLib::FindMaxAbs(pilot_ifft_, this->ofdm_ca_num_);
  // additional 2^2 (6dB) power backoff
  this->scale_ =
      2 * std::max({ul_max_mag, dl_max_mag, ue_pilot_max_mag, pilot_max_mag});

  float dl_papr =
      (this->frame_.NumDlDataSyms() > 0)
          ? dl_max_mag /
                CommsLib::FindMeanAbs(dl_iq_ifft, this->frame_.NumDlDataSyms(),
                                      this->ue_ant_num_ * this->ofdm_ca_num_)
          : 0;
  float ul_papr =
      (this->frame_.NumUlDataSyms() > 0)
          ? ul_max_mag /
                CommsLib::FindMeanAbs(ul_iq_ifft, this->frame_.NumUlDataSyms(),
                                      this->ue_ant_num_ * this->ofdm_ca_num_)
          : 0;
  std::printf(
      "Uplink PAPR %2.2f dB, Downlink PAPR %2.2f dB, using scale %2.2f\n",
      10 * std::log10(ul_papr), 10 * std::log10(dl_papr), this->scale_);

  // Generate time domain symbols for downlink
  for (size_t i = 0; i < this->frame_.NumDlDataSyms(); i++) {
    for (size_t u = 0; u < this->ue_ant_num_; u++) {
      size_t q = u * this->ofdm_ca_num_;
      size_t r = u * this->samps_per_symbol_;
      CommsLib::Ifft2tx(&dl_iq_ifft[i][q], &this->dl_iq_t_[i][r],
                        this->ofdm_ca_num_, this->ofdm_tx_zero_prefix_,
                        this->cp_len_, kDebugDownlink ? 1 : this->scale_);
    }
  }

  // Generate time domain uplink symbols
  for (size_t i = 0; i < this->frame_.NumUlDataSyms(); i++) {
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
    CommsLib::Ifft2tx(ue_pilot_ifft_[i], this->ue_specific_pilot_t_[i],
                      this->ofdm_ca_num_, this->ofdm_tx_zero_prefix_,
                      this->cp_len_, kDebugDownlink ? 1 : this->scale_);
  }

  this->pilot_ci16_.resize(samps_per_symbol_, 0);
  CommsLib::Ifft2tx(pilot_ifft_, this->pilot_ci16_.data(), ofdm_ca_num_,
                    ofdm_tx_zero_prefix_, cp_len_, scale_);

  for (size_t i = 0; i < ofdm_ca_num_; i++) {
    this->pilot_cf32_.emplace_back(pilot_ifft_[i].re / scale_,
                                   pilot_ifft_[i].im / scale_);
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

  this->pilot_ue_sc_.resize(ue_ant_num_);
  this->pilot_ue_ci16_.resize(ue_ant_num_);
  for (size_t ue_id = 0; ue_id < this->ue_ant_num_; ue_id++) {
    this->pilot_ue_ci16_.at(ue_id).resize(this->frame_.NumPilotSyms());
    for (size_t pilot_idx = 0; pilot_idx < this->frame_.NumPilotSyms();
         pilot_idx++) {
      this->pilot_ue_ci16_.at(ue_id).at(pilot_idx).resize(samps_per_symbol_, 0);
      if (this->freq_orthogonal_pilot_ || ue_id == pilot_idx) {
        std::vector<arma::uword> pilot_sc_list;
        for (size_t sc_id = 0; sc_id < ofdm_data_num_; sc_id++) {
          const size_t org_sc = sc_id + ofdm_data_start_;
          const size_t center_sc = ofdm_ca_num_ / 2;
          // FFT Shift
          const size_t shifted_sc = (org_sc >= center_sc)
                                        ? (org_sc - center_sc)
                                        : (org_sc + center_sc);
          if (this->freq_orthogonal_pilot_ == false ||
              sc_id % this->pilot_sc_group_size_ == ue_id) {
            pilot_ifft_[shifted_sc] = this->pilots_[sc_id];
            pilot_sc_list.push_back(org_sc);
          } else {
            pilot_ifft_[shifted_sc].re = 0.0f;
            pilot_ifft_[shifted_sc].im = 0.0f;
          }
        }
        pilot_ue_sc_.at(ue_id) = arma::uvec(pilot_sc_list);
        CommsLib::IFFT(pilot_ifft_, this->ofdm_ca_num_, false);
        CommsLib::Ifft2tx(pilot_ifft_,
                          this->pilot_ue_ci16_.at(ue_id).at(pilot_idx).data(),
                          ofdm_ca_num_, ofdm_tx_zero_prefix_, cp_len_, scale_);
      }
    }
  }

  if (kDebugPrintPilot) {
    std::cout << "Pilot data = [" << std::endl;
    for (size_t sc_id = 0; sc_id < ofdm_data_num_; sc_id++) {
      std::cout << pilots_[sc_id].re << "+1i*" << pilots_[sc_id].im << " ";
    }
    std::cout << std::endl << "];" << std::endl;
    for (size_t ue_id = 0; ue_id < ue_ant_num_; ue_id++) {
      std::cout << "pilot_ue_sc_[" << ue_id << "] = [" << std::endl
                << pilot_ue_sc_.at(ue_id).as_row() << "];" << std::endl;
      std::cout << "ue_specific_pilot_[" << ue_id << "] = [" << std::endl;
      for (size_t sc_id = 0; sc_id < ofdm_data_num_; sc_id++) {
        std::cout << ue_specific_pilot_[ue_id][sc_id].re << "+1i*"
                  << ue_specific_pilot_[ue_id][sc_id].im << " ";
      }
      std::cout << std::endl << "];" << std::endl;
      std::cout << "ue_pilot_ifft_[" << ue_id << "] = [" << std::endl;
      for (size_t ifft_idx = 0; ifft_idx < ofdm_ca_num_; ifft_idx++) {
        std::cout << ue_pilot_ifft_[ue_id][ifft_idx].re << "+1i*"
                  << ue_pilot_ifft_[ue_id][ifft_idx].im << " ";
      }
      std::cout << std::endl << "];" << std::endl;
    }
  }

  if (pilot_ifft_ != nullptr) {
    FreeBuffer1d(&pilot_ifft_);
  }
  ul_iq_ifft.Free();
  dl_iq_ifft.Free();
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
  ue_specific_pilot_t_.Free();
  ue_specific_pilot_.Free();
  ue_pilot_ifft_.Free();

  ul_mod_table_.Free();
  dl_mod_table_.Free();
  dl_bits_.Free();
  ul_bits_.Free();
  ul_mod_bits_.Free();
  dl_mod_bits_.Free();
  dl_iq_f_.Free();
  dl_iq_t_.Free();
  ul_iq_f_.Free();
  ul_iq_t_.Free();
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
              << "Beamforming: " << beamforming_str_ << std::endl
              << "Bs Channel: " << channel_ << std::endl
              << "Ue Channel: " << ue_channel_ << std::endl
              << "Max Frames: " << frames_to_test_ << std::endl
              << "Transport Block Size: " << transport_block_size_ << std::endl
              << "Noise Level: " << noise_level_ << std::endl
              << "UL Bytes per CB: " << ul_num_bytes_per_cb_ << std::endl
              << "DL Bytes per CB: " << dl_num_bytes_per_cb_ << std::endl
              << "FFT in rru: " << fft_in_rru_ << std::endl;
  }
}

extern "C" {
__attribute__((visibility("default"))) Config* ConfigNew(char* filename) {
  auto* cfg = new Config(filename);
  cfg->LoadTestVectors();
  return cfg;
}
}
