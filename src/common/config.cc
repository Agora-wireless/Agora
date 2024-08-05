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
#include "simd_types.h"
/*#include "modulation.h"
#include "phy_ldpc_decoder_5gnr.h"
#include "scrambler.h"
#include "utils_ldpc.h"*/

using json = nlohmann::json;

static constexpr size_t kMacAlignmentBytes = 64u;
static constexpr size_t kShortIdLen = 3;
static constexpr bool kDebugPrintConfiguration = false;
static constexpr size_t kDefaultSpectralEff = 2;

/// Print the I/Q samples in the pilots
static constexpr bool kDebugPrintPilot = false;

static const std::string kLogFilepath =
    TOSTRING(PROJECT_DIRECTORY) "/files/log/";

Config::Config(std::string jsonfilename)
    : freq_ghz_(GetTime::MeasureRdtscFreq()),
      frame_(""),
      mac_params_(frame_),
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
  excluded_.emplace_back(0);
  if (tdd_conf.contains("exclude_cores")) {
    auto exclude_cores = tdd_conf.at("exclude_cores");
    excluded_.resize(exclude_cores.size());
    for (size_t i = 0; i < exclude_cores.size(); i++) {
      excluded_.at(i) = exclude_cores.at(i);
    }
  }
  SetCpuLayoutOnNumaNodes(true, excluded_);
  dynamic_core_allocation_ = tdd_conf.value("dynamic_core", false);

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
  adapt_ues_ = tdd_conf.value("adapt_ues", false);

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

  rp_remote_host_name_ = tdd_conf.value("rp_remote_host_name", "127.0.0.1");
  rp_tx_port_ = tdd_conf.value("rp_tx_port", 3000);
  rp_rx_port_ = tdd_conf.value("rp_rx_port", 4000);

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

  ue_app_rx_addr_ = tdd_conf.value("ue_app_rx_addr", "127.0.0.1");
  bs_app_rx_addr_ = tdd_conf.value("bs_app_rx_addr", "127.0.0.1");
  ue_app_rx_port_ = tdd_conf.value("ue_app_rx_port", kAppUserLocalPort);
  bs_app_rx_port_ = tdd_conf.value("bs_app_rx_port", kAppBaseLocalPort);

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
      //data_idx++;
      if (i % ofdm_pilot_spacing_ == 1) {
        control_symbol_map_.at(i) = SubcarrierType::kPTRS;
      } else {
        control_symbol_map_.at(i) = SubcarrierType::kData;
        dl_symbol_ctrl_id_.at(i) = ctrl_idx++;
        //ctrl_idx++;
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
    if (frame.find(',') != std::string::npos) {
      std::vector<std::string> flex_formats =
          tdd_conf.value("flex_formats", json::array());
      FiveGConfig fivegconfig = FiveGConfig(tdd_conf);
      frame = fivegconfig.FiveGFormat();
      rate_ = fivegconfig.SamplingRate();
      ofdm_data_start_ = fivegconfig.OfdmDataStart();
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
  frame_to_profile_ = tdd_conf.value(
      "profiling_frame", SIZE_MAX);  // Profiling disabled by default
  core_offset_ = tdd_conf.value("core_offset", 0);
  // use all available cores
  if (dynamic_core_allocation_) {
    worker_thread_num_ = sysconf(_SC_NPROCESSORS_ONLN) -
                         (core_offset_ + socket_thread_num_ +
                          (dynamic_core_allocation_ ? 1 : 0) + 1);
  } else {
    worker_thread_num_ = tdd_conf.value("worker_thread_num", 25);
  }
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
  AGORA_LOG_SYMBOL("Noise level: %.3f\n", noise_level_);

  // Scrambler and descrambler configurations
  scramble_enabled_ = tdd_conf.value("wlan_scrambler", true);

  // LDPC Coding and Modulation configurations
  ul_mcs_params_ = this->Parse(tdd_conf, "ul_mcs");
  dl_mcs_params_ = this->Parse(tdd_conf, "dl_mcs");
  mac_params_ =
      MacUtils(this->frame_, this->GetFrameDurationSec(), ofdm_data_num_,
               this->GetOFDMDataNum(), GetOFDMCtrlNum());
  mac_params_.SetMacParams(ul_mcs_params_, dl_mcs_params_, true);

  /*ul_mac_packet_size_ = kDefaultSpectralEff * ofdm_data_num_;
  dl_mac_packet_size_ = kDefaultSpectralEff * ofdm_data_num_;
  ul_mac_payload_size_ = ul_mac_packet_size_ - sizeof(MacPacketHeaderPacked);
  dl_mac_payload_size_ = dl_mac_packet_size_ - sizeof(MacPacketHeaderPacked);*/

  freq_domain_channel_ = tdd_conf.value("freq_domain_channel", false);
  scheduler_type_ =
      tdd_conf.value("scheduler_type", adapt_ues_ ? "custom" : "round_robbin");

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

  this->running_.store(true);
  AGORA_LOG_INFO(
      "Config: %zu BS antennas, %zu UE antennas, %zu pilot symbols per "
      "frame,\n"
      "\t%zu uplink data symbols per frame, %zu downlink data symbols "
      "per frame,\n"
      "\t%zu OFDM subcarriers (%zu data subcarriers),\n"
      "\tBeamforming %s, \n"
      "\tSymbol time %.3f usec\n"
      "\tFrame time %.3f usec\n"
      "Radio Network Traffic Peak (Mbps): %.3f\n"
      "Radio Network Traffic Avg  (Mbps): %.3f\n"
      "Basestation Network Traffic Peak (Mbps): %.3f\n"
      "Basestation Network Traffic Avg  (Mbps): %.3f\n"
      "UE Network Traffic Peak (Mbps): %.3f\n"
      "UE Network Traffic Avg  (Mbps): %.3f\n"
      "All UEs Network Traffic Peak (Mbps): %.3f\n"
      "All UEs Network Traffic Avg (Mbps): %.3f\n",
      bs_ant_num_, ue_ant_num_, frame_.NumPilotSyms(), frame_.NumULSyms(),
      frame_.NumDLSyms(), ofdm_ca_num_, ofdm_data_num_,
      beamforming_str_.c_str(), this->GetSymbolDurationSec() * 1e6,
      this->GetFrameDurationSec() * 1e6, bit_rate_mbps, per_bs_radio_traffic,
      bit_rate_mbps * bs_ant_num_, per_bs_radio_traffic * bs_ant_num_,
      2 * bit_rate_mbps, per_ue_radio_traffic, 2 * bit_rate_mbps * ue_ant_num_,
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

void Config::GenPilots() {
  if ((kUseArgos == true) || (kUseUHD == true) || (kUsePureUHD == true)) {
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

  RtAssert(pilot_ifft_ == nullptr, "pilot_ifft_ should be null");
  AllocBuffer1d(&pilot_ifft_, this->ofdm_ca_num_,
                Agora_memory::Alignment_t::kAlign64, 1);

  RtAssert(pilot_pre_ifft_ == nullptr, "pilot_pre_ifft_ should be null");
  AllocBuffer1d(&pilot_pre_ifft_, this->ofdm_ca_num_,
                Agora_memory::Alignment_t::kAlign64, 1);

  std::memcpy(pilot_pre_ifft_ + ofdm_data_start_, this->pilots_,
              ofdm_data_num_ * sizeof(complex_float));

  //pilot_pre_ifft_ == pilot_ifft_;
  std::memcpy(pilot_ifft_, pilot_pre_ifft_,
              ofdm_ca_num_ * sizeof(complex_float));

  if (this->freq_domain_channel_ == false) {
    CommsLib::FFTShift(pilot_ifft_, ofdm_ca_num_);
    CommsLib::IFFT(pilot_ifft_, ofdm_ca_num_, false);
  }

  // Generate UE-specific pilots based on Zadoff-Chu sequence for phase tracking
  this->ue_specific_pilot_.Malloc(this->ue_ant_num_, this->ofdm_data_num_,
                                  Agora_memory::Alignment_t::kAlign64);
  this->ue_specific_pilot_t_.Calloc(this->ue_ant_num_, this->samps_per_symbol_,
                                    Agora_memory::Alignment_t::kAlign64);

  ue_pilot_ifft_.Calloc(this->ue_ant_num_, this->ofdm_ca_num_,
                        Agora_memory::Alignment_t::kAlign64);
  ue_pilot_pre_ifft_.Calloc(this->ue_ant_num_, this->ofdm_ca_num_,
                            Agora_memory::Alignment_t::kAlign64);
  for (size_t i = 0; i < ue_ant_num_; i++) {
    auto zc_ue_pilot_i = CommsLib::SeqCyclicShift(
        zc_seq,
        (i + this->ue_ant_offset_) * (float)M_PI / 6);  // LTE DMRS

    for (size_t j = 0; j < this->ofdm_data_num_; j++) {
      this->ue_specific_pilot_[i][j] = {zc_ue_pilot_i[j].real(),
                                        zc_ue_pilot_i[j].imag()};
    }

    std::memcpy(ue_pilot_ifft_[i] + ofdm_data_start_,
                this->ue_specific_pilot_[i],
                ofdm_data_num_ * sizeof(complex_float));
    //Save a copy of the frequency domain info
    std::memcpy(ue_pilot_pre_ifft_[i] + ofdm_data_start_,
                ue_pilot_ifft_[i] + ofdm_data_start_,
                ofdm_data_num_ * sizeof(complex_float));

    CommsLib::FFTShift(ue_pilot_ifft_[i], ofdm_ca_num_);
    CommsLib::IFFT(ue_pilot_ifft_[i], ofdm_ca_num_, false);
  }
}

void Config::LoadUplinkData() {
  if (this->frame_.NumUlDataSyms() > 0) {
    // Uplink modulation input bits
    ul_mod_bits_.Calloc(this->frame_.NumUlDataSyms(),
                        Roundup<64>(this->ofdm_data_num_) * this->ue_ant_num_,
                        Agora_memory::Alignment_t::kAlign32);
    const std::string ul_mod_data_file =
        kExperimentFilepath + kUlModDataPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    // reset seek offset for new file read
    size_t seek_offset = 0;
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
    // Downlink modulation input bits
    dl_mod_bits_.Calloc(this->frame_.NumDlDataSyms(),
                        Roundup<64>(this->GetOFDMDataNum()) * ue_ant_num_,
                        Agora_memory::Alignment_t::kAlign32);
    const std::string dl_mod_data_file =
        kExperimentFilepath + kDlModDataPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    // reset seek offset for new file read
    size_t seek_offset = 0;
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

  size_t n_frames = 1;
  if (this->adapt_ues_) {
    static const std::string kFilename = kExperimentFilepath +
                                         kUeSchedulePrefix +
                                         std::to_string(this->ue_ant_num_);
    std::vector<uint8_t> ue_map_array(frames_to_test_ * ue_ant_num_);
    Utils::ReadBinaryFile(kFilename + "ue.bin", sizeof(uint8_t),
                          frames_to_test_ * ue_ant_num_, 0,
                          ue_map_array.data());
    std::vector<size_t> ue_sched_set;
    for (size_t fn = 0u; fn < this->frames_to_test_; fn++) {
      size_t ue_sched_id = 0;
      for (size_t ue = 0; ue < this->ue_ant_num_; ue++) {
        uint8_t sched_bit = ue_map_array.at(fn * this->ue_ant_num_ + ue);
        ue_sched_id += static_cast<size_t>(sched_bit * std::pow(2, ue));
      }
      if (ue_sched_set.empty()) {
        ue_sched_set.push_back(ue_sched_id);
      } else {
        std::vector<size_t>::iterator it;
        for (it = ue_sched_set.begin(); it < ue_sched_set.end(); it++) {
          if (ue_sched_id == *it) {  // dont's push this to keep vector unique
            break;
          } else if (ue_sched_id > *it && (it + 1) == ue_sched_set.end()) {
            ue_sched_set.push_back(ue_sched_id);
            break;
          } else if (ue_sched_id < *it && it == ue_sched_set.begin()) {
            ue_sched_set.insert(it, ue_sched_id);
            break;
          } else if (ue_sched_id > *it && ue_sched_id < *(it + 1)) {
            ue_sched_set.insert(it + 1, ue_sched_id);
            break;
          }
        }
      }
    }
    n_frames = ue_sched_set.size();
  }
  AGORA_LOG_INFO("Loading data for %zu schedules\n", n_frames);
  // Freq-domain uplink symbols
  this->LoadUplinkData();
  Table<complex_float> ul_iq_ifft;
  size_t total_ul_syms = n_frames * this->frame_.NumUlDataSyms();
  if (total_ul_syms > 0) {
    ul_iq_ifft.Calloc(total_ul_syms, this->ofdm_ca_num_ * this->ue_ant_num_,
                      Agora_memory::Alignment_t::kAlign64);
    ul_iq_f_.Calloc(total_ul_syms, this->ofdm_data_num_ * this->ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    ul_iq_t_.Calloc(this->frame_.NumUlDataSyms(),
                    this->samps_per_symbol_ * this->ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    const std::string ul_ifft_data_file =
        kExperimentFilepath + kUlIfftPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    size_t seek_offset = 0;
    for (size_t fr = 0; fr < n_frames; fr++) {
      for (size_t i = 0; i < this->frame_.NumUlDataSyms(); i++) {
        seek_offset +=
            ofdm_ca_num_ * this->ue_ant_offset_ * sizeof(complex_float);
        size_t total_sym_id = fr * frame_.NumUlDataSyms() + i;
        for (size_t j = 0; j < this->ue_ant_num_; j++) {
          Utils::ReadBinaryFile(ul_ifft_data_file, sizeof(complex_float),
                                ofdm_ca_num_, seek_offset,
                                &ul_iq_ifft[total_sym_id][j * ofdm_ca_num_]);
          std::memcpy(
              &ul_iq_f_[total_sym_id][j * ofdm_data_num_],
              &ul_iq_ifft[total_sym_id][j * ofdm_ca_num_ + ofdm_data_start_],
              ofdm_data_num_ * sizeof(complex_float));
          seek_offset += ofdm_ca_num_ * sizeof(complex_float);
        }
        seek_offset +=
            ofdm_ca_num_ *
            (this->ue_ant_total_ - this->ue_ant_offset_ - this->ue_ant_num_) *
            sizeof(complex_float);
        AGORA_LOG_TRACE("SEEK Offset %zu\n", seek_offset);
      }
    }
  }

  // Generate freq-domain downlink symbols
  this->LoadDownlinkData();
  Table<complex_float> dl_iq_ifft;
  size_t total_dl_syms = n_frames * this->frame_.NumDlDataSyms();
  if (total_dl_syms > 0) {
    dl_iq_ifft.Calloc(total_dl_syms, this->ofdm_ca_num_ * this->ue_ant_num_,
                      Agora_memory::Alignment_t::kAlign64);
    dl_iq_f_.Calloc(total_dl_syms, ofdm_data_num_ * ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    dl_iq_t_.Calloc(this->frame_.NumDlDataSyms(),
                    this->samps_per_symbol_ * this->ue_ant_num_,
                    Agora_memory::Alignment_t::kAlign64);
    const std::string dl_ifft_data_file =
        kExperimentFilepath + kDlIfftPrefix +
        std::to_string(this->ofdm_ca_num_) + "_ue" +
        std::to_string(this->ue_ant_total_) + ".bin";
    size_t seek_offset = 0;
    for (size_t fr = 0; fr < n_frames; fr++) {
      for (size_t i = 0; i < this->frame_.NumDlDataSyms(); i++) {
        seek_offset +=
            ofdm_ca_num_ * this->ue_ant_offset_ * sizeof(complex_float);
        size_t total_sym_id = fr * frame_.NumDlDataSyms() + i;
        for (size_t j = 0; j < this->ue_ant_num_; j++) {
          Utils::ReadBinaryFile(dl_ifft_data_file, sizeof(complex_float),
                                ofdm_ca_num_, seek_offset,
                                &dl_iq_ifft[total_sym_id][j * ofdm_ca_num_]);
          std::memcpy(
              &dl_iq_f_[total_sym_id][j * ofdm_data_num_],
              &dl_iq_ifft[total_sym_id][j * ofdm_ca_num_ + ofdm_data_start_],
              ofdm_data_num_ * sizeof(complex_float));
          seek_offset += ofdm_ca_num_ * sizeof(complex_float);
        }
        seek_offset +=
            ofdm_ca_num_ *
            (this->ue_ant_total_ - this->ue_ant_offset_ - this->ue_ant_num_) *
            sizeof(complex_float);
        AGORA_LOG_TRACE("SEEK Offset %zu\n", seek_offset);
      }
    }
  }

  // Find normalization factor through searching for max value in IFFT results

  float ul_max_mag =
      (this->frame_.NumUlDataSyms() > 0)
          ? CommsLib::FindMaxAbs(ul_iq_ifft, total_ul_syms,
                                 this->ue_ant_num_ * this->ofdm_ca_num_)
          : 0;
  float dl_max_mag =
      (this->frame_.NumDlDataSyms() > 0)
          ? CommsLib::FindMaxAbs(dl_iq_ifft, total_dl_syms,
                                 this->ue_ant_num_ * this->ofdm_ca_num_)
          : 0;
  float ue_pilot_max_mag = CommsLib::FindMaxAbs(
      ue_pilot_ifft_, this->ue_ant_num_, this->ofdm_ca_num_);
  float pilot_max_mag = CommsLib::FindMaxAbs(pilot_ifft_, this->ofdm_ca_num_);
  // additional 2^2 (6dB) power backoff
  this->scale_ =
      2 * std::max({ul_max_mag, dl_max_mag, ue_pilot_max_mag, pilot_max_mag});

  float dl_papr = (this->frame_.NumDlDataSyms() > 0)
                      ? dl_max_mag / CommsLib::FindMeanAbs(
                                         dl_iq_ifft, total_dl_syms,
                                         this->ue_ant_num_ * this->ofdm_ca_num_)
                      : 0;
  float ul_papr = (this->frame_.NumUlDataSyms() > 0)
                      ? ul_max_mag / CommsLib::FindMeanAbs(
                                         ul_iq_ifft, total_ul_syms,
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
    complex_float* ue_pilot = (this->freq_domain_channel_)
                                  ? ue_pilot_pre_ifft_[i]
                                  : ue_pilot_ifft_[i];
    CommsLib::Ifft2tx(ue_pilot, this->ue_specific_pilot_t_[i],
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
          if (this->freq_orthogonal_pilot_ == false ||
              sc_id % this->pilot_sc_group_size_ == ue_id) {
            pilot_ifft_[org_sc] = this->pilots_[sc_id];
            pilot_sc_list.push_back(org_sc);
          } else {
            pilot_ifft_[org_sc].re = 0.0f;
            pilot_ifft_[org_sc].im = 0.0f;
          }
        }

        pilot_ue_sc_.at(ue_id) = arma::uvec(pilot_sc_list);

        std::memcpy(pilot_pre_ifft_, pilot_ifft_,
                    ofdm_ca_num_ * sizeof(complex_float));
        CommsLib::FFTShift(pilot_ifft_, this->ofdm_ca_num_);
        CommsLib::IFFT(pilot_ifft_, this->ofdm_ca_num_, false);

        const complex_float* pilot_to_tx =
            (this->freq_domain_channel_) ? pilot_pre_ifft_ : pilot_ifft_;
        CommsLib::Ifft2tx(pilot_to_tx,
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
  if (pilot_pre_ifft_ != nullptr) {
    FreeBuffer1d(&pilot_pre_ifft_);
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
  ue_pilot_pre_ifft_.Free();

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
              << "UL Bytes per CB: "
              << mac_params_.NumBytesPerCb(Direction::kUplink) << std::endl
              << "DL Bytes per CB: "
              << mac_params_.NumBytesPerCb(Direction::kDownlink) << std::endl
              << "Frequency domain channel: " << freq_domain_channel_
              << "Scheduler type: " << scheduler_type_ << std::endl;
  }
}

extern "C" {
__attribute__((visibility("default"))) Config* ConfigNew(const char* filename) {
  auto* cfg = new Config(filename);
  cfg->LoadTestVectors();
  return cfg;
}
}
