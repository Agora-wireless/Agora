#include "config.h"

#include <boost/range/algorithm/count.hpp>

#include "utils_ldpc.h"

Config::Config(std::string jsonfile) : freq_ghz_(MeasureRdtscFreq()) {
  pilots_ = nullptr;
  pilots_sgn_ = nullptr;
  SetCpuLayoutOnNumaNodes();
  std::string conf;
  Utils::LoadTddConfig(jsonfile, conf);
  const auto tdd_conf = json::parse(conf);

  /* antenna configurations */
  if (!kUseUHD) {
    std::string hub_file = tdd_conf.value("hubs", "");
    if (!hub_file.empty()) {
      Utils::LoadDevices(hub_file, hub_ids_);
    }
  }
  std::string serial_file = tdd_conf.value("irises", "");
  ref_ant_ = tdd_conf.value("ref_ant", 0);
  external_ref_node_ = tdd_conf.value("external_ref_node", false);
  n_cells_ = tdd_conf.value("cells", 1);
  channel_ = tdd_conf.value("channel", "A");
  n_channels_ = std::min(channel_.size(), (size_t)2);
  bs_ant_num_ = tdd_conf.value("antenna_num", 8);
  is_ue_ = tdd_conf.value("UE", false);
  ue_num_ = tdd_conf.value("ue_num", 8);
  ue_ant_num_ = ue_num_;
  if (!serial_file.empty()) {
    Utils::LoadDevices(serial_file, radio_ids_);
  }
  if (!radio_ids_.empty()) {
    n_radios_ = radio_ids_.size();
    n_antennas_ = n_channels_ * n_radios_;
    if (is_ue_) {
      ue_ant_num_ = n_antennas_;
      ue_num_ = n_radios_;
    } else {
      if (ref_ant_ >= n_antennas_) {
        ref_ant_ = 0;
      }
      if (bs_ant_num_ != n_antennas_) {
        bs_ant_num_ = n_antennas_;
      }
    }
  } else {
    n_radios_ = tdd_conf.value("radio_num", is_ue_ ? ue_ant_num_ : bs_ant_num_);
  }
  bf_ant_num_ = bs_ant_num_;
  if (external_ref_node_) {
    bf_ant_num_ = bs_ant_num_ - n_channels_;
  }

  if (kUseArgos || kUseUHD) {
    RtAssert(n_radios_ != 0, "Error: No radios exist in Argos mode");
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
  auto gain_adj_json_a = tdd_conf.value("client_gain_adjust_a", json::array());
  if (gain_adj_json_a.empty()) {
    client_gain_adj_a_.resize(n_radios_, 0);
  } else {
    client_gain_adj_a_.assign(gain_adj_json_a.begin(), gain_adj_json_a.end());
  }
  auto gain_adj_json_b = tdd_conf.value("client_gain_adjust_b", json::array());
  if (gain_adj_json_b.empty()) {
    client_gain_adj_b_.resize(n_radios_, 0);
  } else {
    client_gain_adj_b_.assign(gain_adj_json_b.begin(), gain_adj_json_b.end());
  }
  rate_ = tdd_conf.value("rate", 5e6);
  nco_ = tdd_conf.value("nco_frequency", 0.75 * rate_);
  bw_filter_ = rate_ + 2 * nco_;
  radio_rf_freq_ = freq_ - nco_;
  beacon_ant_ = tdd_conf.value("beacon_antenna", 0);
  beamsweep_ = tdd_conf.value("beamsweep", false);
  sample_cal_en_ = tdd_conf.value("sample_calibrate", false);
  imbalance_cal_en_ = tdd_conf.value("imbalance_calibrate", false);
  init_calib_repeat_ = tdd_conf.value("init_calib_repeat", 1);

  modulation_ = tdd_conf.value("modulation", "16QAM");

  bs_server_addr_ = tdd_conf.value("bs_server_addr", "127.0.0.1");
  bs_rru_addr_ = tdd_conf.value("bs_rru_addr", "127.0.0.1");
  ue_server_addr_ = tdd_conf.value("ue_server_addr", "127.0.0.1");
  mac_remote_addr_ = tdd_conf.value("mac_remote_addr", "127.0.0.1");
  bs_server_port_ = tdd_conf.value("bs_server_port", 8000);
  bs_rru_port_ = tdd_conf.value("bs_rru_port", 9000);
  ue_rru_port_ = tdd_conf.value("ue_rru_port", 7000);
  ue_server_port_ = tdd_conf.value("ue_sever_port", 6000);

  dpdk_num_ports_ = tdd_conf.value("dpdk_num_ports", 1);

  mac_rx_port_ = tdd_conf.value("mac_rx_port", 5000);
  mac_tx_port_ = tdd_conf.value("mac_tx_port", 4000);
  init_mac_running_ = tdd_conf.value("init_mac_running", false);

  /* frame configurations */
  cp_len_ = tdd_conf.value("cp_len", 0);
  ofdm_ca_num_ = tdd_conf.value("ofdm_ca_num", 2048);
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
           "OFDM_DATA_NUM must be a multiple of subcarriers per cacheline");
  RtAssert(ofdm_data_num_ % kTransposeBlockSize == 0,
           "Transpose block size must divide number of OFDM data subcarriers");
  ofdm_pilot_spacing_ = tdd_conf.value("ofdm_pilot_spacing", 16);
  ofdm_data_start_ =
      tdd_conf.value("ofdm_data_start", (ofdm_ca_num_ - ofdm_data_num_) / 2);
  ofdm_data_stop_ = ofdm_data_start_ + ofdm_data_num_;
  downlink_mode_ = tdd_conf.value("downlink_mode", false);
  bigstation_mode_ = tdd_conf.value("bigstation_mode", false);
  freq_orthogonal_pilot_ = tdd_conf.value("freq_orthogonal_pilot", false);
  correct_phase_shift_ = tdd_conf.value("correct_phase_shift", false);
  dl_pilot_syms_ = tdd_conf.value("client_dl_pilot_syms", 0);
  ul_pilot_syms_ = tdd_conf.value("client_ul_pilot_syms", 0);
  auto tx_advance = tdd_conf.value("tx_advance", json::array());
  if (tx_advance.empty()) {
    cl_tx_advance_.resize(n_radios_, 0);
  } else {
    cl_tx_advance_.assign(tx_advance.begin(), tx_advance.end());
  }
  hw_framer_ = tdd_conf.value("hw_framer", true);
  if (tdd_conf.find("frames") == tdd_conf.end()) {
    symbol_num_perframe_ = tdd_conf.value("symbol_num_perframe", 70);
    pilot_symbol_num_perframe_ =
        tdd_conf.value("pilot_num", freq_orthogonal_pilot_ ? 1 : ue_ant_num_);
    ul_data_symbol_num_perframe_ = tdd_conf.value(
        "ul_symbol_num_perframe",
        downlink_mode_ ? 0
                       : symbol_num_perframe_ - pilot_symbol_num_perframe_ - 1);
    dl_data_symbol_num_perframe_ =
        tdd_conf.value("dl_symbol_num_perframe", downlink_mode_ ? 10 : 0);
    dl_data_symbol_start_ = tdd_conf.value("dl_data_symbol_start", 10);
    std::string sched("B");
    for (size_t s = 0; s < pilot_symbol_num_perframe_; s++) {
      sched += "P";
    }
    // Below it is assumed either dl or ul to be active at one time
    if (downlink_mode_) {
      size_t dl_symbol_start =
          1 + pilot_symbol_num_perframe_ + dl_data_symbol_start_;
      size_t dl_symbol_end = dl_symbol_start + dl_data_symbol_num_perframe_;
      for (size_t s = 1 + pilot_symbol_num_perframe_; s < dl_symbol_start;
           s++) {
        sched += "G";
      }
      for (size_t s = dl_symbol_start; s < dl_symbol_end; s++) {
        sched += "D";
      }
      for (size_t s = dl_symbol_end; s < symbol_num_perframe_; s++) {
        sched += "G";
      }
    } else {
      size_t ul_data_symbol_end =
          1 + pilot_symbol_num_perframe_ + ul_data_symbol_num_perframe_;
      for (size_t s = 1 + pilot_symbol_num_perframe_; s < ul_data_symbol_end;
           s++) {
        sched += "U";
      }
      for (size_t s = ul_data_symbol_end; s < symbol_num_perframe_; s++) {
        sched += "G";
      }
    }
    frames_.push_back(sched);
  } else {
    json jframes = tdd_conf.value("frames", json::array());
    for (size_t f = 0; f < jframes.size(); f++) {
      frames_.push_back(jframes.at(f).get<std::string>());
    }
  }
  std::printf("Config: Frame schedule %s (%zu symbols)\n",
              frames_.at(0).c_str(), frames_.at(0).size());

  beacon_symbols_ = Utils::LoadSymbols(frames_, 'B');
  pilot_symbols_ = Utils::LoadSymbols(frames_, 'P');
  ul_symbols_ = Utils::LoadSymbols(frames_, 'U');
  dl_symbols_ = Utils::LoadSymbols(frames_, 'D');
  ul_cal_symbols_ = Utils::LoadSymbols(frames_, 'L');
  dl_cal_symbols_ = Utils::LoadSymbols(frames_, 'C');
  recip_cal_en_ = (!ul_cal_symbols_[0].empty() and !dl_cal_symbols_[0].empty());
  ant_per_group_ = dl_cal_symbols_[0].size();
  ant_group_num_ = recip_cal_en_ ? bf_ant_num_ / ant_per_group_ : 0;

  symbol_num_perframe_ = frames_.at(0).size();
  beacon_symbol_num_perframe_ = beacon_symbols_[0].size();
  pilot_symbol_num_perframe_ = pilot_symbols_[0].size();
  data_symbol_num_perframe_ = symbol_num_perframe_ -
                              pilot_symbol_num_perframe_ -
                              beacon_symbol_num_perframe_;
  ul_data_symbol_num_perframe_ = ul_symbols_[0].size();
  dl_data_symbol_num_perframe_ = dl_symbols_[0].size();
  downlink_mode_ = dl_data_symbol_num_perframe_ > 0;
  dl_data_symbol_start_ =
      dl_data_symbol_num_perframe_ > 0 ? dl_symbols_[0].front() : 0;
  dl_data_symbol_end_ =
      dl_data_symbol_num_perframe_ > 0 ? dl_symbols_[0].back() + 1 : 0;
  recip_pilot_symbol_num_perframe_ = recip_cal_en_ ? 1 : 0;

  if (is_ue_ and !freq_orthogonal_pilot_ and
      ue_ant_num_ != pilot_symbol_num_perframe_) {
    RtAssert(false, "Number of pilot symbols doesn't match number of UEs");
  }
  if (!is_ue_ and !freq_orthogonal_pilot_ and
      tdd_conf.find("ue_num") == tdd_conf.end()) {
    ue_num_ = pilot_symbol_num_perframe_;
    ue_ant_num_ = ue_num_;
  }
  ue_ant_offset_ = tdd_conf.value("ue_ant_offset", 0);
  total_ue_ant_num_ = tdd_conf.value("total_ue_ant_num", ue_ant_num_);

  /* Agora configurations */
  frames_to_test_ = tdd_conf.value("frames_to_test", 9600);
  core_offset_ = tdd_conf.value("core_offset", 0);
  worker_thread_num_ = tdd_conf.value("worker_thread_num", 25);
  socket_thread_num_ = tdd_conf.value("socket_thread_num", 4);
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
  encode_block_size_ = tdd_conf.value("encode_block_size", 1);

  noise_level_ = tdd_conf.value("noise_level", 0.03);  // default: 30 dB
  std::printf("Noise level: %.2f\n", noise_level_);
  /* LDPC Coding configurations */
  ldpc_config_.bg_ = tdd_conf.value("base_graph", 1);
  ldpc_config_.early_termination_ =
      (tdd_conf.value("earlyTermination", 1) != 0);
  ldpc_config_.decoder_iter_ = tdd_conf.value("decoderIter", 5);
  ldpc_config_.zc_ = tdd_conf.value("Zc", 72);
  ldpc_config_.n_rows_ =
      tdd_conf.value("nRows", (ldpc_config_.bg_ == 1) ? 46 : 42);
  ldpc_config_.cb_len_ = LdpcNumInputBits(ldpc_config_.bg_, ldpc_config_.zc_);
  ldpc_config_.cb_codew_len_ = LdpcNumEncodedBits(
      ldpc_config_.bg_, ldpc_config_.zc_, ldpc_config_.n_rows_);

  /* Modulation configurations */
  mod_order_bits_ =
      modulation_ == "64QAM"
          ? CommsLib::kQaM64
          : (modulation_ == "16QAM" ? CommsLib::kQaM16 : CommsLib::kQpsk);
  UpdateModCfgs(mod_order_bits_);

  RtAssert(ldpc_config_.nblocks_in_symbol_ > 0,
           "LDPC expansion factor is too large for number of OFDM data "
           "subcarriers.");

  std::printf(
      "Config: LDPC: Zc: %d, %zu code blocks per symbol, %d information "
      "bits per encoding, %d bits per encoded code word, decoder "
      "iterations: %d, code rate %.3f (nRows = %zu)\n",
      ldpc_config_.zc_, ldpc_config_.nblocks_in_symbol_, ldpc_config_.cb_len_,
      ldpc_config_.cb_codew_len_, ldpc_config_.decoder_iter_,
      1.f * LdpcNumInputCols(ldpc_config_.bg_) /
          (LdpcNumInputCols(ldpc_config_.bg_) - 2 + ldpc_config_.n_rows_),
      ldpc_config_.n_rows_);

  fft_in_rru_ = tdd_conf.value("fft_in_rru", false);

  samps_per_symbol_ =
      ofdm_tx_zero_prefix_ + ofdm_ca_num_ + cp_len_ + ofdm_tx_zero_postfix_;
  packet_length_ =
      Packet::kOffsetOfData + ((kUse12BitIQ ? 3 : 4) * samps_per_symbol_);
  dl_packet_length_ = Packet::kOffsetOfData + samps_per_symbol_ * 4;
  RtAssert(packet_length_ < 9000,
           "Packet size must be smaller than jumbo frame");

  num_bytes_per_cb_ = ldpc_config_.cb_len_ / 8;  // TODO: Use bits_to_bytes()?
  data_bytes_num_persymbol_ =
      num_bytes_per_cb_ * ldpc_config_.nblocks_in_symbol_;
  mac_packet_length_ = data_bytes_num_persymbol_;
  mac_payload_length_ = mac_packet_length_ - MacPacket::kOffsetOfData;
  mac_packets_perframe_ = ul_data_symbol_num_perframe_ - ul_pilot_syms_;
  mac_data_bytes_num_perframe_ = mac_payload_length_ * mac_packets_perframe_;
  mac_bytes_num_perframe_ = mac_packet_length_ * mac_packets_perframe_;

  // Done!
  running_ = true;
  std::printf(
      "Config: %zu BS antennas, %zu UE antennas, %zu pilot symbols per "
      "frame,\n\t"
      "%zu uplink data symbols per frame, %zu downlink data "
      "symbols per frame,\n\t"
      "%zu OFDM subcarriers (%zu data subcarriers), modulation %s,\n\t"
      "%zu MAC data bytes per frame, %zu MAC bytes per frame\n",
      bs_ant_num_, ue_ant_num_, pilot_symbol_num_perframe_,
      ul_data_symbol_num_perframe_, dl_data_symbol_num_perframe_, ofdm_ca_num_,
      ofdm_data_num_, modulation_.c_str(), mac_data_bytes_num_perframe_,
      mac_bytes_num_perframe_);
}

void Config::GenData() {
  if (kUseArgos || kUseUHD) {
    std::vector<std::vector<double>> gold_ifft =
        CommsLib::GetSequence(128, CommsLib::kGoldIfft);
    std::vector<std::complex<int16_t>> gold_ifft_ci16 =
        Utils::DoubleToCint16(gold_ifft);
    for (size_t i = 0; i < 128; i++) {
      gold_cf32_.push_back(
          std::complex<float>(gold_ifft[0][i], gold_ifft[1][i]));
    }

    std::vector<std::vector<double>> sts_seq =
        CommsLib::GetSequence(0, CommsLib::kStsSeq);
    std::vector<std::complex<int16_t>> sts_seq_ci16 =
        Utils::DoubleToCint16(sts_seq);

    // Populate STS (stsReps repetitions)
    int sts_reps = 15;
    for (int i = 0; i < sts_reps; i++) {
      beacon_ci16_.insert(beacon_ci16_.end(), sts_seq_ci16.begin(),
                          sts_seq_ci16.end());
    }

    // Populate gold sequence (two reps, 128 each)
    int gold_reps = 2;
    for (int i = 0; i < gold_reps; i++) {
      beacon_ci16_.insert(beacon_ci16_.end(), gold_ifft_ci16.begin(),
                          gold_ifft_ci16.end());
    }

    beacon_len_ = beacon_ci16_.size();

    if (samps_per_symbol_ <
        beacon_len_ + ofdm_tx_zero_prefix_ + ofdm_tx_zero_postfix_) {
      std::string msg = "Minimum supported symbol_size is ";
      msg += std::to_string(beacon_len_);
      throw std::invalid_argument(msg);
    }

    beacon_ = Utils::Cint16ToUint32(beacon_ci16_, false, "QI");
    coeffs_ = Utils::Cint16ToUint32(gold_ifft_ci16, true, "QI");

    // Add addition padding for beacon sent from host
    int frac_beacon = samps_per_symbol_ % beacon_len_;
    std::vector<std::complex<int16_t>> pre_beacon(ofdm_tx_zero_prefix_, 0);
    std::vector<std::complex<int16_t>> post_beacon(
        ofdm_tx_zero_postfix_ + frac_beacon, 0);
    beacon_ci16_.insert(beacon_ci16_.begin(), pre_beacon.begin(),
                        pre_beacon.end());
    beacon_ci16_.insert(beacon_ci16_.end(), post_beacon.begin(),
                        post_beacon.end());
  }

  // Generate common pilots based on Zadoff-Chu sequence for channel estimation
  auto zc_seq_double =
      CommsLib::GetSequence(ofdm_data_num_, CommsLib::kLteZadoffChu);
  auto zc_seq = Utils::DoubleToCfloat(zc_seq_double);
  common_pilot_ =
      CommsLib::SeqCyclicShift(zc_seq, M_PI / 4);  // Used in LTE SRS

  pilots_ =
      (complex_float*)aligned_alloc(64, ofdm_data_num_ * sizeof(complex_float));
  pilots_sgn_ = (complex_float*)aligned_alloc(
      64, ofdm_data_num_ * sizeof(complex_float));  // used in CSI estimation
  for (size_t i = 0; i < ofdm_data_num_; i++) {
    pilots_[i] = {common_pilot_[i].real(), common_pilot_[i].imag()};
    auto pilot_sgn =
        common_pilot_[i] / (float)std::pow(std::abs(common_pilot_[i]), 2);
    pilots_sgn_[i] = {pilot_sgn.real(), pilot_sgn.imag()};
  }
  complex_float* pilot_ifft;
  AllocBuffer1d(&pilot_ifft, ofdm_ca_num_, Agora_memory::Alignment_t::kK64Align,
                1);
  for (size_t j = 0; j < ofdm_data_num_; j++) {
    pilot_ifft[j + ofdm_data_start_] = pilots_[j];
  }
  CommsLib::IFFT(pilot_ifft, ofdm_ca_num_, false);

  // Generate UE-specific pilots based on Zadoff-Chu sequence for phase tracking
  ue_specific_pilot_.Malloc(ue_ant_num_, ofdm_data_num_,
                            Agora_memory::Alignment_t::kK64Align);
  ue_specific_pilot_t_.Calloc(ue_ant_num_, samps_per_symbol_,
                              Agora_memory::Alignment_t::kK64Align);
  Table<complex_float> ue_pilot_ifft;
  ue_pilot_ifft.Calloc(ue_ant_num_, ofdm_ca_num_,
                       Agora_memory::Alignment_t::kK64Align);
  auto zc_ue_pilot_double =
      CommsLib::GetSequence(ofdm_data_num_, CommsLib::kLteZadoffChu);
  auto zc_ue_pilot = Utils::DoubleToCfloat(zc_ue_pilot_double);
  for (size_t i = 0; i < ue_ant_num_; i++) {
    auto zc_ue_pilot_i = CommsLib::SeqCyclicShift(
        zc_ue_pilot, (i + ue_ant_offset_) * (float)M_PI / 6);  // LTE DMRS
    for (size_t j = 0; j < ofdm_data_num_; j++) {
      ue_specific_pilot_[i][j] = {zc_ue_pilot_i[j].real(),
                                  zc_ue_pilot_i[j].imag()};
      ue_pilot_ifft[i][j + ofdm_data_start_] = ue_specific_pilot_[i][j];
    }
    CommsLib::IFFT(ue_pilot_ifft[i], ofdm_ca_num_, false);
  }

  // Get uplink and downlink raw bits either from file or random numbers
  size_t num_bytes_per_ue = num_bytes_per_cb_ * ldpc_config_.nblocks_in_symbol_;
  size_t num_bytes_per_ue_pad =
      Roundup<64>(num_bytes_per_cb_) * ldpc_config_.nblocks_in_symbol_;
  dl_bits_.Malloc(dl_data_symbol_num_perframe_,
                  num_bytes_per_ue_pad * ue_ant_num_,
                  Agora_memory::Alignment_t::kK64Align);
  dl_iq_f_.Calloc(dl_data_symbol_num_perframe_, ofdm_ca_num_ * ue_ant_num_,
                  Agora_memory::Alignment_t::kK64Align);
  dl_iq_t_.Calloc(dl_data_symbol_num_perframe_, samps_per_symbol_ * ue_ant_num_,
                  Agora_memory::Alignment_t::kK64Align);

  ul_bits_.Malloc(ul_data_symbol_num_perframe_,
                  num_bytes_per_ue_pad * ue_ant_num_,
                  Agora_memory::Alignment_t::kK64Align);
  ul_iq_f_.Calloc(ul_data_symbol_num_perframe_, ofdm_ca_num_ * ue_ant_num_,
                  Agora_memory::Alignment_t::kK64Align);
  ul_iq_t_.Calloc(ul_data_symbol_num_perframe_, samps_per_symbol_ * ue_ant_num_,
                  Agora_memory::Alignment_t::kK64Align);

#ifdef GENERATE_DATA
  for (size_t ue_id = 0; ue_id < UE_ANT_NUM; ue_id++) {
    for (size_t j = 0; j < num_bytes_per_ue_pad; j++) {
      int cur_offset = j * UE_ANT_NUM + ue_id;
      for (size_t i = 0; i < ul_data_symbol_num_perframe; i++)
        ul_bits[i][cur_offset] = rand() % mod_order;
      for (size_t i = 0; i < dl_data_symbol_num_perframe; i++)
        dl_bits[i][cur_offset] = rand() % mod_order;
    }
  }
#else
  std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
  std::string ul_data_file = cur_directory + "/data/LDPC_orig_ul_data_" +
                             std::to_string(ofdm_ca_num_) + "_ant" +
                             std::to_string(total_ue_ant_num_) + ".bin";
  std::cout << "Config: Reading raw data from " << ul_data_file << std::endl;
  FILE* fd = fopen(ul_data_file.c_str(), "rb");
  if (fd == nullptr) {
    std::printf("Failed to open antenna file %s. Error %s.\n",
                ul_data_file.c_str(), strerror(errno));
    std::exit(-1);
  }
  for (size_t i = 0; i < ul_data_symbol_num_perframe_; i++) {
    if (std::fseek(fd, num_bytes_per_ue * ue_ant_offset_, SEEK_CUR) != 0) {
      return;
    }
    for (size_t j = 0; j < ue_ant_num_; j++) {
      size_t r = std::fread(ul_bits_[i] + j * num_bytes_per_ue_pad,
                            sizeof(int8_t), num_bytes_per_ue, fd);
      if (r < num_bytes_per_ue) {
        std::printf(
            "uplink bad read from file %s (batch %zu : %zu) "
            "%zu : %zu\n",
            ul_data_file.c_str(), i, j, r, num_bytes_per_ue);
      }
    }
    if (std::fseek(fd,
                   num_bytes_per_ue *
                       (total_ue_ant_num_ - ue_ant_offset_ - ue_ant_num_),
                   SEEK_CUR) != 0) {
      return;
    }
  }
  std::fclose(fd);

  std::string dl_data_file = cur_directory + "/data/LDPC_orig_dl_data_" +
                             std::to_string(ofdm_ca_num_) + "_ant" +
                             std::to_string(total_ue_ant_num_) + ".bin";
  std::cout << "Config: Reading raw data from " << dl_data_file << std::endl;
  fd = fopen(dl_data_file.c_str(), "rb");
  if (fd == nullptr) {
    std::printf("Failed to open antenna file %s. Error %s.\n",
                dl_data_file.c_str(), strerror(errno));
    std::exit(-1);
  }
  for (size_t i = 0; i < dl_data_symbol_num_perframe_; i++) {
    for (size_t j = 0; j < ue_ant_num_; j++) {
      size_t r = std::fread(dl_bits_[i] + j * num_bytes_per_ue_pad,
                            sizeof(int8_t), num_bytes_per_ue, fd);
      if (r < num_bytes_per_ue) {
        std::printf("downlink bad read from file %s (batch %zu : %zu) \n",
                    dl_data_file.c_str(), i, j);
      }
    }
  }
  std::fclose(fd);
#endif

  const size_t encoded_bytes_per_block =
      BitsToBytes(ldpc_config_.cb_codew_len_);
  const size_t num_blocks_per_symbol =
      ldpc_config_.nblocks_in_symbol_ * ue_ant_num_;

  // Encode uplink bits
  ul_encoded_bits_.Malloc(ul_data_symbol_num_perframe_ * num_blocks_per_symbol,
                          encoded_bytes_per_block,
                          Agora_memory::Alignment_t::kK64Align);
  ul_mod_input_.Calloc(ul_data_symbol_num_perframe_,
                       ofdm_data_num_ * ue_ant_num_,
                       Agora_memory::Alignment_t::kK32Align);
  int8_t* temp_parity_buffer =
      new int8_t[LdpcEncodingParityBufSize(ldpc_config_.bg_, ldpc_config_.zc_)];

  for (size_t i = 0; i < ul_data_symbol_num_perframe_; i++) {
    for (size_t j = 0; j < ue_ant_num_; j++) {
      for (size_t k = 0; k < ldpc_config_.nblocks_in_symbol_; k++) {
        int8_t* bits_ptr = GetInfoBits(ul_bits_, i, j, k);
        int8_t* coded_bits_ptr =
            ul_encoded_bits_[i * num_blocks_per_symbol +
                             j * ldpc_config_.nblocks_in_symbol_ + k];
        LdpcEncodeHelper(ldpc_config_.bg_, ldpc_config_.zc_,
                         ldpc_config_.n_rows_, coded_bits_ptr,
                         temp_parity_buffer, bits_ptr);
        AdaptBitsForMod(reinterpret_cast<uint8_t*>(coded_bits_ptr),
                        ul_mod_input_[i] + j * ofdm_data_num_ +
                            k * encoded_bytes_per_block / mod_order_bits_,
                        encoded_bytes_per_block, mod_order_bits_);
      }
    }
  }

  // Generate freq-domain uplink symbols
  Table<complex_float> ul_iq_ifft;
  ul_iq_ifft.Calloc(ul_data_symbol_num_perframe_, ofdm_ca_num_ * ue_ant_num_,
                    Agora_memory::Alignment_t::kK64Align);
  for (size_t i = 0; i < ul_data_symbol_num_perframe_; i++) {
    for (size_t u = 0; u < ue_ant_num_; u++) {
      size_t p = u * ofdm_data_num_;
      size_t q = u * ofdm_ca_num_;

      for (size_t j = ofdm_data_start_; j < ofdm_data_stop_; j++) {
        size_t k = j - ofdm_data_start_;
        size_t s = p + k;
        ul_iq_f_[i][q + j] = ModSingleUint8(ul_mod_input_[i][s], mod_table_);
        ul_iq_ifft[i][q + j] = ul_iq_f_[i][q + j];
      }

      CommsLib::IFFT(&ul_iq_ifft[i][q], ofdm_ca_num_, false);
    }
  }

  // Encode downlink bits
  Table<int8_t> dl_encoded_bits;
  dl_encoded_bits.Malloc(dl_data_symbol_num_perframe_ * num_blocks_per_symbol,
                         encoded_bytes_per_block,
                         Agora_memory::Alignment_t::kK64Align);
  dl_mod_input_.Calloc(dl_data_symbol_num_perframe_,
                       ofdm_data_num_ * ue_ant_num_,
                       Agora_memory::Alignment_t::kK32Align);
  for (size_t i = 0; i < dl_data_symbol_num_perframe_; i++) {
    for (size_t j = 0; j < ue_ant_num_; j++) {
      for (size_t k = 0; k < ldpc_config_.nblocks_in_symbol_; k++) {
        int8_t* bits_ptr = GetInfoBits(dl_bits_, i, j, k);
        int8_t* coded_bits_ptr =
            dl_encoded_bits[i * num_blocks_per_symbol +
                            j * ldpc_config_.nblocks_in_symbol_ + k];
        LdpcEncodeHelper(ldpc_config_.bg_, ldpc_config_.zc_,
                         ldpc_config_.n_rows_, coded_bits_ptr,
                         temp_parity_buffer, bits_ptr);
        AdaptBitsForMod(reinterpret_cast<uint8_t*>(coded_bits_ptr),
                        dl_mod_input_[i] + j * ofdm_data_num_ +
                            k * encoded_bytes_per_block / mod_order_bits_,
                        encoded_bytes_per_block, mod_order_bits_);
      }
    }
  }

  // Generate freq-domain downlink symbols
  Table<complex_float> dl_iq_ifft;
  dl_iq_ifft.Calloc(dl_data_symbol_num_perframe_, ofdm_ca_num_ * ue_ant_num_,
                    Agora_memory::Alignment_t::kK64Align);
  for (size_t i = 0; i < dl_data_symbol_num_perframe_; i++) {
    for (size_t u = 0; u < ue_ant_num_; u++) {
      size_t p = u * ofdm_data_num_;
      size_t q = u * ofdm_ca_num_;

      for (size_t j = ofdm_data_start_; j < ofdm_data_stop_; j++) {
        int k = j - ofdm_data_start_;
        size_t s = p + k;
        if (k % ofdm_pilot_spacing_ != 0) {
          dl_iq_f_[i][q + j] = ModSingleUint8(dl_mod_input_[i][s], mod_table_);
        } else {
          dl_iq_f_[i][q + j] = ue_specific_pilot_[u][k];
        }
        dl_iq_ifft[i][q + j] = dl_iq_f_[i][q + j];
      }
      CommsLib::IFFT(&dl_iq_ifft[i][q], ofdm_ca_num_, false);
    }
  }

  // Find normalization factor through searching for max value in IFFT results
  float max_val = CommsLib::FindMaxAbs(ul_iq_ifft, ul_data_symbol_num_perframe_,
                                       ue_ant_num_ * ofdm_ca_num_);
  float cur_max_val = CommsLib::FindMaxAbs(
      dl_iq_ifft, dl_data_symbol_num_perframe_, ue_ant_num_ * ofdm_ca_num_);
  if (cur_max_val > max_val) {
    max_val = cur_max_val;
  }
  cur_max_val = CommsLib::FindMaxAbs(ue_pilot_ifft, ue_ant_num_, ofdm_ca_num_);
  if (cur_max_val > max_val) {
    max_val = cur_max_val;
  }
  cur_max_val = CommsLib::FindMaxAbs(pilot_ifft, ofdm_ca_num_);
  if (cur_max_val > max_val) {
    max_val = cur_max_val;
  }

  scale_ = 2 * max_val;  // additional 2^2 (6dB) power backoff

  // Generate time domain symbols for downlink
  for (size_t i = 0; i < dl_data_symbol_num_perframe_; i++) {
    for (size_t u = 0; u < ue_ant_num_; u++) {
      size_t q = u * ofdm_ca_num_;
      size_t r = u * samps_per_symbol_;
      CommsLib::Ifft2tx(&dl_iq_ifft[i][q], &dl_iq_t_[i][r], ofdm_ca_num_,
                        ofdm_tx_zero_prefix_, cp_len_, scale_);
    }
  }

  // Generate time domain uplink symbols
  for (size_t i = 0; i < ul_data_symbol_num_perframe_; i++) {
    for (size_t u = 0; u < ue_ant_num_; u++) {
      size_t q = u * ofdm_ca_num_;
      size_t r = u * samps_per_symbol_;
      CommsLib::Ifft2tx(&ul_iq_ifft[i][q], &ul_iq_t_[i][r], ofdm_ca_num_,
                        ofdm_tx_zero_prefix_, cp_len_, scale_);
    }
  }

  // Generate time domain ue-specific pilot symbols
  for (size_t i = 0; i < ue_ant_num_; i++) {
    CommsLib::Ifft2tx(ue_pilot_ifft[i], ue_specific_pilot_t_[i], ofdm_ca_num_,
                      ofdm_tx_zero_prefix_, cp_len_, scale_);
    if (kDebugPrintPilot) {
      std::printf("ue_specific_pilot_t%zu=[", i);
      for (size_t j = 0; j < ofdm_ca_num_; j++) {
        std::printf("%2.4f+%2.4fi ", ue_pilot_ifft[i][j].re,
                    ue_pilot_ifft[i][j].im);
      }
      std::printf("]\n");
    }
  }

  pilot_ci16_.resize(samps_per_symbol_, 0);
  CommsLib::Ifft2tx(pilot_ifft, (std::complex<int16_t>*)pilot_ci16_.data(),
                    ofdm_ca_num_, ofdm_tx_zero_prefix_, cp_len_, scale_);

  for (size_t i = 0; i < ofdm_ca_num_; i++) {
    pilot_cf32_.push_back(std::complex<float>(pilot_ifft[i].re / scale_,
                                              pilot_ifft[i].im / scale_));
  }
  pilot_cf32_.insert(pilot_cf32_.begin(), pilot_cf32_.end() - cp_len_,
                     pilot_cf32_.end());  // add CP

  // generate a UINT32 version to write to FPGA buffers
  pilot_ = Utils::Cfloat32ToUint32(pilot_cf32_, false, "QI");

  std::vector<uint32_t> pre_uint32(ofdm_tx_zero_prefix_, 0);
  pilot_.insert(pilot_.begin(), pre_uint32.begin(), pre_uint32.end());
  pilot_.resize(samps_per_symbol_);

  if (kDebugPrintPilot) {
    std::cout << "Pilot data: " << std::endl;
    for (size_t i = 0; i < ofdm_data_num_; i++) {
      std::cout << pilots_[i].re << "+1i*" << pilots_[i].im << ",";
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

  delete[] temp_parity_buffer;
  dl_encoded_bits.Free();
  ul_iq_ifft.Free();
  dl_iq_ifft.Free();
  ue_pilot_ifft.Free();
  FreeBuffer1d(&pilot_ifft);
}

Config::~Config() {
  if (pilots_ != nullptr) {
    FreeBuffer1d(&pilots_);
    pilots_ = nullptr;
  }
  if (pilots_sgn_ != nullptr) {
    FreeBuffer1d(&pilots_sgn_);
    pilots_sgn_ = nullptr;
  }
  mod_table_.Free();
  dl_bits_.Free();
  ul_bits_.Free();
  dl_iq_f_.Free();
  dl_iq_t_.Free();
  ul_iq_f_.Free();
  ul_iq_t_.Free();
}

int Config::GetSymbolId(size_t symbol_id) {
  return (symbol_id < pilot_symbol_num_perframe_
              ? pilot_symbols_[0][symbol_id]
              : ul_symbols_[0][symbol_id - pilot_symbol_num_perframe_]);
}

size_t Config::GetDlSymbolIdx(size_t frame_id, size_t symbol_id) const {
  size_t fid = frame_id % frames_.size();
  const auto it =
      find(dl_symbols_[fid].begin(), dl_symbols_[fid].end(), symbol_id);
  if (it != dl_symbols_[fid].end()) {
    return it - dl_symbols_[fid].begin();
  } else {
    return SIZE_MAX;
  }
}

size_t Config::GetPilotSymbolIdx(size_t frame_id, size_t symbol_id) const {
  size_t fid = frame_id % frames_.size();
  const auto it =
      find(pilot_symbols_[fid].begin(), pilot_symbols_[fid].end(), symbol_id);
  if (it != pilot_symbols_[fid].end()) {
#ifdef DEBUG3
    std::printf("get_pilot_symbol_idx(%zu, %zu) = %zu\n", frame_id, symbol_id,
                it - pilotSymbols[fid].begin());
#endif
    return it - pilot_symbols_[fid].begin();
  } else {
    return SIZE_MAX;
  }
}

size_t Config::GetUlSymbolIdx(size_t frame_id, size_t symbol_id) const {
  size_t fid = frame_id % frames_.size();
  const auto it =
      find(ul_symbols_[fid].begin(), ul_symbols_[fid].end(), symbol_id);
  if (it != ul_symbols_[fid].end()) {
#ifdef DEBUG3
    std::printf("get_ul_symbol_idx(%zu, %zu) = %zu\n", frame_id, symbol_id,
                it - ULSymbols[fid].begin());
#endif
    return it - ul_symbols_[fid].begin();
  } else {
    return SIZE_MAX;
  }
}

bool Config::IsPilot(size_t frame_id, size_t symbol_id) {
  assert(symbol_id < symbol_num_perframe_);
  size_t fid = frame_id % frames_.size();
  char s = frames_[fid].at(symbol_id);
#ifdef DEBUG3
  std::printf("isPilot(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
  if (is_ue_) {
    std::vector<size_t>::iterator it;
    it = find(dl_symbols_[fid].begin(), dl_symbols_[fid].end(), symbol_id);
    int ind = dl_pilot_syms_;
    if (it != dl_symbols_[fid].end()) {
      ind = it - dl_symbols_[fid].begin();
    }
    return (ind < (int)dl_pilot_syms_);
    // return cfg->frames[fid].at(symbol_id) == 'P' ? true : false;
  } else {
    return s == 'P';
  }
  // return (symbol_id < UE_NUM);
}

bool Config::IsCalDlPilot(size_t frame_id, size_t symbol_id) {
  assert(symbol_id < symbol_num_perframe_);
  if (is_ue_) {
    return false;
  }
  return frames_[frame_id % frames_.size()].at(symbol_id) == 'C';
}

bool Config::IsCalUlPilot(size_t frame_id, size_t symbol_id) {
  assert(symbol_id < symbol_num_perframe_);
  if (is_ue_) {
    return false;
  }
  return frames_[frame_id % frames_.size()].at(symbol_id) == 'L';
}

bool Config::IsUplink(size_t frame_id, size_t symbol_id) {
  assert(symbol_id < symbol_num_perframe_);
  char s = frames_[frame_id % frames_.size()].at(symbol_id);
#ifdef DEBUG3
  std::printf("isUplink(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
  return s == 'U';
}

bool Config::IsDownlink(size_t frame_id, size_t symbol_id) {
  assert(symbol_id < symbol_num_perframe_);
  char s = frames_[frame_id % frames_.size()].at(symbol_id);
#ifdef DEBUG3
  std::printf("isDownlink(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
  if (is_ue_) {
    return s == 'D' && !this->IsPilot(frame_id, symbol_id);
  } else {
    return s == 'D';
  }
}

SymbolType Config::GetSymbolType(size_t frame_id, size_t symbol_id) {
  assert(!is_ue_);  // Currently implemented for only the Agora server
  char s = frames_[frame_id % frames_.size()][symbol_id];
  switch (s) {
    case 'B':
      return SymbolType::kBeacon;
    case 'D':
      return SymbolType::kDL;
    case 'U':
      return SymbolType::kUL;
    case 'P':
      return SymbolType::kPilot;
    case 'C':
      return SymbolType::kCalDL;
    case 'L':
      return SymbolType::kCalUL;
    case 'G':
      return SymbolType::kGuard;
  }
  RtAssert(false, std::string("Should not reach here") + std::to_string(s));
  return SymbolType::kUnknown;
}

extern "C" {
__attribute__((visibility("default"))) Config* ConfigNew(char* filename) {
  auto* cfg = new Config(filename);
  cfg->GenData();
  return cfg;
}
}
