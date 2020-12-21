#include "config.hpp"
#include "utils_ldpc.hpp"
#include <boost/range/algorithm/count.hpp>

static const size_t kDefaultSymbolNumPerFrame = 70;
static const size_t kDefaultPilotSymPerFrame = 1;
static const size_t kDefaultULSymPerFrame = 61;
static const size_t kDefaultULSymStart = 9;
static const size_t kDefaultDLSymPerFrame = 10;
static const size_t kDefaultDLSymStart = 10;
static const bool kDefaultDownlinkMode = false;


Config::Config(std::string jsonfile)
    : freq_ghz(measure_rdtsc_freq()), ldpc_config_(0, 0, 0, 0, 0, 0, 0, 0), frame_("")
{
    pilots_ = nullptr;
    pilots_sgn_ = nullptr;
    set_cpu_layout_on_numa_nodes();
    std::string conf;
    Utils::loadTDDConfig(jsonfile, conf);
    const auto tddConf = json::parse(conf);

    /* antenna configurations */
    if (!kUseUHD) {
        std::string hub_file = tddConf.value("hubs", "");
        if (hub_file.size() > 0)
            Utils::loadDevices(hub_file, hub_ids);
    }
    std::string serial_file = tddConf.value("irises", "");
    ref_ant = tddConf.value("ref_ant", 0);
    external_ref_node = tddConf.value("external_ref_node", false);
    nCells = tddConf.value("cells", 1);
    channel = tddConf.value("channel", "A");
    nChannels = std::min(channel.size(), (size_t)2);
    bs_ant_num_ = tddConf.value("antenna_num", 8);
    isUE = tddConf.value("UE", false);
    ue_num_ = tddConf.value("ue_num", 8);
    ue_ant_num_ = ue_num_;
    if (serial_file.size() > 0)
        Utils::loadDevices(serial_file, radio_ids);
    if (radio_ids.size() != 0) {
        nRadios = radio_ids.size();
        nAntennas = nChannels * nRadios;
        if (isUE) {
            ue_ant_num_ = nAntennas;
            ue_num_ = nRadios;
        } else {
            if (ref_ant >= nAntennas)
                ref_ant = 0;
            if (bs_ant_num_ != nAntennas)
                bs_ant_num_ = nAntennas;
        }
    } else
        nRadios = tddConf.value("radio_num", isUE ? ue_ant_num_ : bs_ant_num_);
    bf_ant_num_ = bs_ant_num_;
    if (external_ref_node)
        bf_ant_num_ = bs_ant_num_ - nChannels;

    if (kUseArgos || kUseUHD) {
        rt_assert(nRadios != 0, "Error: No radios exist in Argos mode");
    }

    /* radio configurations */
    freq = tddConf.value("frequency", 3.6e9);
    single_gain_ = tddConf.value("single_gain", true);
    tx_gain_a = tddConf.value("tx_gain_a", 20);
    rx_gain_a = tddConf.value("rx_gain_a", 20);
    tx_gain_b = tddConf.value("tx_gain_b", 20);
    rx_gain_b = tddConf.value("rx_gain_b", 20);
    calib_tx_gain_a = tddConf.value("calib_tx_gain_a", tx_gain_a);
    calib_tx_gain_b = tddConf.value("calib_tx_gain_b", tx_gain_b);
    auto gain_adj_json_a = tddConf.value("client_gain_adjust_a", json::array());
    if (gain_adj_json_a.empty())
        client_gain_adj_a.resize(nRadios, 0);
    else
        client_gain_adj_a.assign(
            gain_adj_json_a.begin(), gain_adj_json_a.end());
    auto gain_adj_json_b = tddConf.value("client_gain_adjust_b", json::array());
    if (gain_adj_json_b.empty())
        client_gain_adj_b.resize(nRadios, 0);
    else
        client_gain_adj_b.assign(
            gain_adj_json_b.begin(), gain_adj_json_b.end());
    rate = tddConf.value("rate", 5e6);
    nco = tddConf.value("nco_frequency", 0.75 * rate);
    bwFilter = rate + 2 * nco;
    radioRfFreq = freq - nco;
    beacon_ant = tddConf.value("beacon_antenna", 0);
    beamsweep = tddConf.value("beamsweep", false);
    sampleCalEn = tddConf.value("sample_calibrate", false);
    imbalanceCalEn = tddConf.value("imbalance_calibrate", false);
    modulation = tddConf.value("modulation", "16QAM");

    bs_server_addr = tddConf.value("bs_server_addr", "127.0.0.1");
    bs_rru_addr = tddConf.value("bs_rru_addr", "127.0.0.1");
    ue_server_addr = tddConf.value("ue_server_addr", "127.0.0.1");
    mac_remote_addr = tddConf.value("mac_remote_addr", "127.0.0.1");
    bs_server_port = tddConf.value("bs_server_port", 8000);
    bs_rru_port = tddConf.value("bs_rru_port", 9000);
    ue_rru_port = tddConf.value("ue_rru_port", 7000);
    ue_server_port = tddConf.value("ue_sever_port", 6000);

    dpdk_num_ports = tddConf.value("dpdk_num_ports", 1);

    mac_rx_port = tddConf.value("mac_rx_port", 5000);
    mac_tx_port = tddConf.value("mac_tx_port", 4000);
    init_mac_running = tddConf.value("init_mac_running", false);

    /* frame configurations */
    cp_len_ = tddConf.value("cp_len", 0);
    ofdm_ca_num_ = tddConf.value("ofdm_ca_num", 2048);
    ofdm_data_num_ = tddConf.value("ofdm_data_num", 1200);
    ofdm_tx_zero_prefix_ = tddConf.value("ofdm_tx_zero_prefix", 0);
    ofdm_tx_zero_postfix_ = tddConf.value("ofdm_tx_zero_postfix", 0);
    ofdm_rx_zero_prefix_bs_
        = tddConf.value("ofdm_rx_zero_prefix_bs", 0) + cp_len_;
    ofdm_rx_zero_prefix_client_
        = tddConf.value("ofdm_rx_zero_prefix_client", 0);
    ofdm_rx_zero_prefix_cal_ul_
        = tddConf.value("ofdm_rx_zero_prefix_cal_ul", 0) + cp_len_;
    ofdm_rx_zero_prefix_cal_dl_
        = tddConf.value("ofdm_rx_zero_prefix_cal_dl", 0) + cp_len_;
    rt_assert(ofdm_data_num_ % kSCsPerCacheline == 0,
        "ofdm_data_num_ must be a multiple of subcarriers per cacheline");
    rt_assert(ofdm_data_num_ % kTransposeBlockSize == 0,
        "Transpose block size must divide number of OFDM data subcarriers");
    ofdm_pilot_spacing_ = tddConf.value("ofdm_pilot_spacing", 16);
    ofdm_data_start_
        = tddConf.value("ofdm_data_start", (ofdm_ca_num_ - ofdm_data_num_) / 2);
    ofdm_data_stop_ = ofdm_data_start_ + ofdm_data_num_;

    bigstation_mode = tddConf.value("bigstation_mode", false);
    freq_orthogonal_pilot = tddConf.value("freq_orthogonal_pilot", false);
    correct_phase_shift = tddConf.value("correct_phase_shift", false);

    cl_tx_advance = tddConf.value("tx_advance", 100);
    hw_framer = tddConf.value("hw_framer", true);

    /* If frames not specified explicitly, construct default based on frame_type / symbol_num_perframe / pilot_num / ul_symbol_num_perframe / dl_symbol_num_perframe / dl_data_symbol_start */
    if (tddConf.find("frames") == tddConf.end() ) {

        bool downlink_mode = tddConf.value("downlink_mode", kDefaultDownlinkMode);
        size_t ul_data_symbol_num_perframe = kDefaultULSymPerFrame;
        size_t ul_data_symbol_start = kDefaultULSymStart;
        size_t dl_data_symbol_num_perframe = kDefaultDLSymPerFrame;
        size_t dl_data_symbol_start = kDefaultDLSymStart;

        /* TODO remove */
        if (downlink_mode == true)
        {
            ul_data_symbol_num_perframe = 0;
            ul_data_symbol_start = 0;
        }
        else {
            dl_data_symbol_num_perframe = 0;
            dl_data_symbol_start = 0;
        }

        size_t symbol_num_perframe      = tddConf.value("symbol_num_perframe", kDefaultSymbolNumPerFrame);
        size_t pilot_symbol_num_perframe = tddConf.value("pilot_num", freq_orthogonal_pilot ? kDefaultPilotSymPerFrame : ue_ant_num_);

        ul_data_symbol_num_perframe  = tddConf.value("ul_symbol_num_perframe", ul_data_symbol_num_perframe);
        ul_data_symbol_start         = tddConf.value("ul_data_symbol_start",   ul_data_symbol_start); /* Start position of the first UL symbol */
        dl_data_symbol_num_perframe  = tddConf.value("dl_symbol_num_perframe", dl_data_symbol_num_perframe);
        dl_data_symbol_start         = tddConf.value("dl_data_symbol_start",   dl_data_symbol_start); /* Start position of the first DL symbol */

        /* TODO remove -- backward compatibility workaround */
        if (dl_data_symbol_start > 0)
        {
            dl_data_symbol_start += pilot_symbol_num_perframe + 1;
        }

        size_t ul_data_symbol_stop = ul_data_symbol_start + ul_data_symbol_num_perframe;
        size_t dl_data_symbol_stop = dl_data_symbol_start + dl_data_symbol_num_perframe;

        if ((ul_data_symbol_num_perframe + dl_data_symbol_num_perframe + pilot_symbol_num_perframe) > symbol_num_perframe)
        {
            std::printf("!!!!! Invalid configuration pilot + ul + dl exceeds total symbols !!!!!\n");
            std::printf("Uplink symbols: %zu, Downlink Symbols :%zu, Pilot Symbols: %zu, Total Symbols: %zu\n", ul_data_symbol_num_perframe, dl_data_symbol_num_perframe, pilot_symbol_num_perframe, symbol_num_perframe);
            std::fflush(stdout);
            assert(false);
        }
        else if (((ul_data_symbol_start >= dl_data_symbol_start) && (ul_data_symbol_start < dl_data_symbol_stop)) || 
                 ((ul_data_symbol_stop > dl_data_symbol_start)  && (ul_data_symbol_stop <= dl_data_symbol_stop)) )
        {
            std::printf("!!!!! Invalid configuration ul and dl symbol overlap detected !!!!!\n");
            std::printf("Uplink - start: %zu - stop :%zu, Downlink - start: %zu - stop %zu\n", ul_data_symbol_start, ul_data_symbol_stop, dl_data_symbol_start, dl_data_symbol_stop);
            std::fflush(stdout);
            assert(false);
        }

        char first_sym, second_sym;
        size_t first_sym_start, first_sym_count, second_sym_start, second_sym_count;
        if (dl_data_symbol_start <= ul_data_symbol_start)
        {
            first_sym = 'D';
            first_sym_start  = dl_data_symbol_start;
            first_sym_count  = dl_data_symbol_num_perframe;
            second_sym = 'U';
            second_sym_start  = ul_data_symbol_start;
            second_sym_count  = ul_data_symbol_num_perframe;
        }
        else
        {
            first_sym = 'U';
            first_sym_start  = ul_data_symbol_start;
            first_sym_count  = ul_data_symbol_num_perframe;
            second_sym = 'D';
            second_sym_start  = dl_data_symbol_start;
            second_sym_count  = dl_data_symbol_num_perframe;
        }
        std::printf("1st Start: %zu, Count: %zu, Symbol %c\n 2nd Start: %zu, Count: %zu, Symbol %c\n Total: %zu\n", 
                    first_sym_start, first_sym_start, first_sym, second_sym_start, second_sym_start, second_sym, symbol_num_perframe);

        std::string sched("B");
        sched.append(pilot_symbol_num_perframe, 'P');
        /* Could roll this up into a loop but will leave like this for readability */
        int add_symbols = 0;
        /* BPGGGG1111111111GGGG2222222222GGGG */
        if (first_sym_start > 0)
        {
            add_symbols  = first_sym_start - sched.length();
            assert(add_symbols >= 0);
            sched.append(first_sym_start - sched.length(), 'G');
            sched.append(first_sym_count, first_sym);
        }

        if (second_sym_start > 0)
        {
            add_symbols  = second_sym_start - sched.length();
            assert(add_symbols >= 0);
            sched.append(add_symbols, 'G');
            sched.append(second_sym_count, second_sym);
        }
        add_symbols  = symbol_num_perframe - sched.length();
        assert(add_symbols >= 0);
        sched.append(add_symbols, 'G');

        frame_ = FrameStats(sched);
        std::printf("Config: Frame schedule %s (%zu symbols)\n", sched.c_str(),
            sched.length());
    } else {
        json jframes = tddConf.value("frames", json::array());

        /* Only allow 1 unique frame type */
        assert(jframes.size() == 1);
        frame_ = FrameStats(jframes.at(0).get<std::string>());
    }

    /* client_dl_pilot_sym uses the first x 'D' symbols for downlink channel estimation for each user. */
    size_t client_dl_pilot_syms = tddConf.value("client_dl_pilot_syms", 0);
    /* client_dl_pilot_sym uses the first x 'D' symbols for downlink channel estimation for each user. */
    size_t client_ul_pilot_syms = tddConf.value("client_ul_pilot_syms", 0);

    frame_.SetClientPilotSyms(client_ul_pilot_syms, client_dl_pilot_syms);

    ant_per_group = frame_.NumDLCalSyms();
    ant_group_num = frame_.IsRecCalEnabled() ? (bf_ant_num_ / ant_per_group) : 0;
    downlink_mode_ = frame_.NumDLSyms() > 0;

    if ((isUE == true) && 
        (freq_orthogonal_pilot == false) &&
        (ue_ant_num_ != frame_.NumPilotSyms()) ) {
        rt_assert(false, "Number of pilot symbols doesn't match number of UEs");
    }
    if ((isUE == false) && (freq_orthogonal_pilot == false) && (tddConf.find("ue_num") == tddConf.end())) {
        ue_num_ = frame_.NumPilotSyms();
        ue_ant_num_ = ue_num_;
    }
    ue_ant_offset = tddConf.value("ue_ant_offset", 0);
    total_ue_ant_num = tddConf.value("total_ue_ant_num", ue_ant_num_);

    /* Agora configurations */
    frames_to_test = tddConf.value("frames_to_test", 9600);
    core_offset = tddConf.value("core_offset", 0);
    worker_thread_num = tddConf.value("worker_thread_num", 25);
    socket_thread_num = tddConf.value("socket_thread_num", 4);
    fft_thread_num = tddConf.value("fft_thread_num", 5);
    demul_thread_num = tddConf.value("demul_thread_num", 5);
    decode_thread_num = tddConf.value("decode_thread_num", 10);
    zf_thread_num = worker_thread_num - fft_thread_num - demul_thread_num
        - decode_thread_num;

    demul_block_size = tddConf.value("demul_block_size", 48);
    rt_assert(demul_block_size % kSCsPerCacheline == 0,
        "Demodulation block size must be a multiple of subcarriers per "
        "cacheline");
    rt_assert(demul_block_size % kTransposeBlockSize == 0,
        "Demodulation block size must be a multiple of transpose block size");
    demul_events_per_symbol = 1 + (ofdm_data_num_ - 1) / demul_block_size;

    zf_batch_size = tddConf.value("zf_batch_size", 1);
    zf_block_size = freq_orthogonal_pilot ? ue_ant_num_
                                          : tddConf.value("zf_block_size", 1);
    zf_events_per_symbol = 1 + (ofdm_data_num_ - 1) / zf_block_size;

    fft_block_size = tddConf.value("fft_block_size", 1);
    encode_block_size = tddConf.value("encode_block_size", 1);

    noise_level = tddConf.value("noise_level", 0.03); //default: 30 dB
    std::printf("Noise level: %.2f\n", noise_level);

    /* LDPC Coding configurations */
    uint16_t base_graph       = tddConf.value("base_graph", 1);
    uint16_t zc               = tddConf.value("Zc", 72);
    bool     early_term       = tddConf.value("earlyTermination", 1);
    int16_t  max_decoder_iter = tddConf.value("decoderIter", 5);
    size_t   num_rows         = tddConf.value("nRows", (base_graph == 1) ? 46 : 42);
    uint32_t num_cb_len       = ldpc_num_input_bits(base_graph, zc);
    uint32_t num_cb_codew_len = ldpc_num_encoded_bits(base_graph, zc, num_rows);

    /* */
    ldpc_config_ = LDPCconfig(base_graph, zc, early_term, max_decoder_iter, 
                              num_cb_len, num_cb_codew_len, num_rows, 0);

    /* Modulation configurations */
    mod_order_bits = modulation == "64QAM"
        ? CommsLib::QAM64
        : (modulation == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK);
    /* Updates num_block_in_sym */
    update_mod_cfgs(mod_order_bits);

    rt_assert(ldpc_config_.num_blocks_in_symbol() > 0,
        "LDPC expansion factor is too large for number of OFDM data "
        "subcarriers.");

    std::printf(
        "Config: LDPC: Zc: %d, %zu code blocks per symbol, %d information "
        "bits per encoding, %d bits per encoded code word, decoder "
        "iterations: %d, code rate %.3f (nRows = %zu)\n",
        ldpc_config_.expansion_factor(), ldpc_config_.num_blocks_in_symbol(), ldpc_config_.num_cb_len(),
        ldpc_config_.num_cb_codew_len(), ldpc_config_.max_decoder_iter(),
        1.f * ldpc_num_input_cols(ldpc_config_.base_graph())
            / (ldpc_num_input_cols(ldpc_config_.base_graph()) - 2 + ldpc_config_.num_rows()),
        ldpc_config_.num_rows());

    fft_in_rru = tddConf.value("fft_in_rru", false);

    sampsPerSymbol
        = ofdm_tx_zero_prefix_ + ofdm_ca_num_ + cp_len_ + ofdm_tx_zero_postfix_;
    packet_length
        = Packet::kOffsetOfData + ((kUse12BitIQ ? 3 : 4) * sampsPerSymbol);
    dl_packet_length_ = Packet::kOffsetOfData + (sampsPerSymbol * 4);
    rt_assert(
        packet_length < 9000, "Packet size must be smaller than jumbo frame");

    num_bytes_per_cb = ldpc_config_.num_cb_len() / 8; // TODO: Use bits_to_bytes()?
    data_bytes_num_persymbol = num_bytes_per_cb * ldpc_config_.num_blocks_in_symbol();
    mac_packet_length = data_bytes_num_persymbol;
    mac_payload_length = mac_packet_length - MacPacket::kOffsetOfData;
    mac_packets_perframe = this->frame_.NumULSyms() - client_ul_pilot_syms;
    mac_data_bytes_num_perframe = mac_payload_length * mac_packets_perframe;
    mac_bytes_num_perframe = mac_packet_length * mac_packets_perframe;

    this->running_.store( true );
    std::printf(
        "Config: %zu BS antennas, %zu UE antennas, %zu pilot symbols per "
        "frame,\n\t"
        "%zu uplink data symbols per frame, %zu downlink data "
        "symbols per frame,\n\t"
        "%zu OFDM subcarriers (%zu data subcarriers), modulation %s,\n\t"
        "%zu MAC data bytes per frame, %zu MAC bytes per frame\n",
        bs_ant_num_, ue_ant_num_, this->frame_.NumPilotSyms(),
        this->frame_.NumULSyms(), this->frame_.NumDLSyms(), ofdm_ca_num_,
        ofdm_data_num_, modulation.c_str(), mac_data_bytes_num_perframe,
        mac_bytes_num_perframe);
}

void Config::genData()
{
    if (kUseArgos || kUseUHD) {
        std::vector<std::vector<double>> gold_ifft
            = CommsLib::getSequence(128, CommsLib::GOLD_IFFT);
        std::vector<std::complex<int16_t>> gold_ifft_ci16
            = Utils::double_to_cint16(gold_ifft);
        for (size_t i = 0; i < 128; i++) {
            gold_cf32.push_back(
                std::complex<float>(gold_ifft[0][i], gold_ifft[1][i]));
        }

        std::vector<std::vector<double>> sts_seq
            = CommsLib::getSequence(0, CommsLib::STS_SEQ);
        std::vector<std::complex<int16_t>> sts_seq_ci16
            = Utils::double_to_cint16(sts_seq);

        // Populate STS (stsReps repetitions)
        int stsReps = 15;
        for (int i = 0; i < stsReps; i++) {
            beacon_ci16.insert(
                beacon_ci16.end(), sts_seq_ci16.begin(), sts_seq_ci16.end());
        }

        // Populate gold sequence (two reps, 128 each)
        int goldReps = 2;
        for (int i = 0; i < goldReps; i++) {
            beacon_ci16.insert(beacon_ci16.end(), gold_ifft_ci16.begin(),
                gold_ifft_ci16.end());
        }

        beacon_len = beacon_ci16.size();

        if (sampsPerSymbol
            < beacon_len + ofdm_tx_zero_prefix_ + ofdm_tx_zero_postfix_) {
            std::string msg = "Minimum supported symbol_size is ";
            msg += std::to_string(beacon_len);
            throw std::invalid_argument(msg);
        }

        beacon = Utils::cint16_to_uint32(beacon_ci16, false, "QI");
        coeffs = Utils::cint16_to_uint32(gold_ifft_ci16, true, "QI");

        // Add addition padding for beacon sent from host
        int fracBeacon = sampsPerSymbol % beacon_len;
        std::vector<std::complex<int16_t>> preBeacon(ofdm_tx_zero_prefix_, 0);
        std::vector<std::complex<int16_t>> postBeacon(
            ofdm_tx_zero_postfix_ + fracBeacon, 0);
        beacon_ci16.insert(
            beacon_ci16.begin(), preBeacon.begin(), preBeacon.end());
        beacon_ci16.insert(
            beacon_ci16.end(), postBeacon.begin(), postBeacon.end());
    }

    // Generate common pilots based on Zadoff-Chu sequence for channel estimation
    auto zc_seq_double
        = CommsLib::getSequence(ofdm_data_num_, CommsLib::LTE_ZADOFF_CHU);
    auto zc_seq = Utils::double_to_cfloat(zc_seq_double);
    common_pilot
        = CommsLib::seqCyclicShift(zc_seq, M_PI / 4); // Used in LTE SRS

    pilots_ = static_cast<complex_float*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align, ofdm_data_num_ * sizeof(complex_float)));
    pilots_sgn_ = static_cast<complex_float*>(
        Agora_memory::padded_aligned_alloc(Agora_memory::Alignment_t::k64Align, ofdm_data_num_ * sizeof(complex_float))); // used in CSI estimation
    for (size_t i = 0; i < ofdm_data_num_; i++) {
        pilots_[i] = { common_pilot[i].real(), common_pilot[i].imag() };
        auto pilot_sgn
            = common_pilot[i] / (float)std::pow(std::abs(common_pilot[i]), 2);
        pilots_sgn_[i] = { pilot_sgn.real(), pilot_sgn.imag() };
    }
    complex_float* pilot_ifft;
    alloc_buffer_1d(
        &pilot_ifft, ofdm_ca_num_, Agora_memory::Alignment_t::k64Align, 1);
    for (size_t j = 0; j < ofdm_data_num_; j++)
        pilot_ifft[j + ofdm_data_start_] = pilots_[j];
    CommsLib::IFFT(pilot_ifft, ofdm_ca_num_, false);

    // Generate UE-specific pilots based on Zadoff-Chu sequence for phase tracking
    ue_specific_pilot.malloc(
        ue_ant_num_, ofdm_data_num_, Agora_memory::Alignment_t::k64Align);
    ue_specific_pilot_t.calloc(
        ue_ant_num_, sampsPerSymbol, Agora_memory::Alignment_t::k64Align);
    
    Table<complex_float> ue_pilot_ifft;
    ue_pilot_ifft.calloc(
        ue_ant_num_, ofdm_ca_num_, Agora_memory::Alignment_t::k64Align);
    auto zc_ue_pilot_double
        = CommsLib::getSequence(ofdm_data_num_, CommsLib::LTE_ZADOFF_CHU);
    auto zc_ue_pilot = Utils::double_to_cfloat(zc_ue_pilot_double);
    for (size_t i = 0; i < ue_ant_num_; i++) {
        auto zc_ue_pilot_i = CommsLib::seqCyclicShift(
            zc_ue_pilot, (i + ue_ant_offset) * (float)M_PI / 6); // LTE DMRS
        for (size_t j = 0; j < ofdm_data_num_; j++) {
            ue_specific_pilot[i][j]
                = { zc_ue_pilot_i[j].real(), zc_ue_pilot_i[j].imag() };
            ue_pilot_ifft[i][j + ofdm_data_start_] = ue_specific_pilot[i][j];
        }
        CommsLib::IFFT(ue_pilot_ifft[i], ofdm_ca_num_, false);
    }

    // Get uplink and downlink raw bits either from file or random numbers
    size_t num_bytes_per_ue = num_bytes_per_cb * ldpc_config_.num_blocks_in_symbol();
    size_t num_bytes_per_ue_pad
        = roundup<64>(num_bytes_per_cb) * ldpc_config_.num_blocks_in_symbol();
    dl_bits_.malloc(this->frame_.NumDLSyms(),
        num_bytes_per_ue_pad * ue_ant_num_, Agora_memory::Alignment_t::k64Align);
    dl_iq_f_.calloc(this->frame_.NumDLSyms(), ofdm_ca_num_ * ue_ant_num_,
        Agora_memory::Alignment_t::k64Align);
    dl_iq_t_.calloc(this->frame_.NumDLSyms(), sampsPerSymbol * ue_ant_num_,
        Agora_memory::Alignment_t::k64Align);

    ul_bits_.malloc(this->frame_.NumULSyms(),
        num_bytes_per_ue_pad * ue_ant_num_, Agora_memory::Alignment_t::k64Align);
    ul_iq_f_.calloc(this->frame_.NumULSyms(), ofdm_ca_num_ * ue_ant_num_,
        Agora_memory::Alignment_t::k64Align);
    ul_iq_t_.calloc(this->frame_.NumULSyms(), sampsPerSymbol * ue_ant_num_,
        Agora_memory::Alignment_t::k64Align);

#ifdef GENERATE_DATA
    for (size_t ue_id = 0; ue_id < ue_ant_num_; ue_id++) {
        for (size_t j = 0; j < num_bytes_per_ue_pad; j++) {
            int cur_offset = j * ue_ant_num_ + ue_id;
            for (size_t i = 0; i < this->frame_.NumULSyms(); i++)
                ul_bits_[i][cur_offset] = rand() % mod_order;
            for (size_t i = 0; i < this->frame_.NumDLSyms(); i++)
                dl_bits_[i][cur_offset] = rand() % mod_order;
        }
    }
#else
    std::string cur_directory1 = TOSTRING(PROJECT_DIRECTORY);
    std::string filename1 = cur_directory1 + "/data/LDPC_orig_data_"
        + std::to_string(ofdm_ca_num_) + "_ant"
        + std::to_string(total_ue_ant_num) + ".bin";
    std::cout << "Config: Reading raw data from " << filename1 << std::endl;
    FILE* fd = std::fopen(filename1.c_str(), "rb");
    if (fd == nullptr) {
        std::printf("Failed to open antenna file %s. Error %s.\n",
            filename1.c_str(), strerror(errno));
        std::exit(-1);
    }
    for (size_t i = 0; i < this->frame_.NumULSyms(); i++) {
        if (std::fseek(fd, num_bytes_per_ue * ue_ant_offset, SEEK_SET) != 0)
            return;
        for (size_t j = 0; j < ue_ant_num_; j++) {
            size_t r = std::fread(ul_bits_[i] + j * num_bytes_per_ue_pad,
                sizeof(int8_t), num_bytes_per_ue, fd);
            if (r < num_bytes_per_ue)
                std::printf("bad read from file %s (batch %zu) \n",
                    filename1.c_str(), i);
        }
        if (std::fseek(fd,
                num_bytes_per_ue
                    * (total_ue_ant_num - ue_ant_offset - ue_ant_num_),
                SEEK_SET)
            != 0)
            return;
    }
    for (size_t i = 0; i < this->frame_.NumDLSyms(); i++) {
        for (size_t j = 0; j < ue_ant_num_; j++) {
            size_t r = std::fread(dl_bits_[i] + j * num_bytes_per_ue_pad,
                sizeof(int8_t), num_bytes_per_ue, fd);
            if (r < num_bytes_per_ue)
                std::printf("bad read from file %s (batch %zu) \n",
                    filename1.c_str(), i);
        }
    }
    std::fclose(fd);
#endif

    const size_t bytes_per_block = bits_to_bytes(ldpc_config_.num_cb_len());
    const size_t encoded_bytes_per_block
        = bits_to_bytes(ldpc_config_.num_cb_codew_len());
    const size_t num_blocks_per_symbol
        = ldpc_config_.num_blocks_in_symbol() * ue_ant_num_;

    // Encode uplink bits
    ul_encoded_bits.malloc(this->frame_.NumULSyms() * num_blocks_per_symbol,
        encoded_bytes_per_block, Agora_memory::Alignment_t::k64Align);

    int8_t* temp_parity_buffer = new int8_t[ldpc_encoding_parity_buf_size(
        ldpc_config_.base_graph(), ldpc_config_.expansion_factor())];
    for (size_t i = 0; i < this->frame_.NumULSyms(); i++) {
        for (size_t j = 0; j < ldpc_config_.num_blocks_in_symbol() * ue_ant_num_; j++) {
            ldpc_encode_helper(ldpc_config_.base_graph(), ldpc_config_.expansion_factor(),
                ldpc_config_.num_rows(),
                ul_encoded_bits[i * num_blocks_per_symbol + j],
                temp_parity_buffer, ul_bits_[i] + j * bytes_per_block);
        }
    }

    ul_mod_input.calloc(this->frame_.NumULSyms(), ofdm_data_num_ * ue_ant_num_,
        Agora_memory::Alignment_t::k32Align);
    for (size_t i = 0; i < this->frame_.NumULSyms(); i++) {
        for (size_t j = 0; j < ue_ant_num_; j++) {
            for (size_t k = 0; k < ldpc_config_.num_blocks_in_symbol(); k++) {
                adapt_bits_for_mod(
                    reinterpret_cast<uint8_t*>(
                        ul_encoded_bits[i * num_blocks_per_symbol
                            + j * ldpc_config_.num_blocks_in_symbol() + k]),
                    ul_mod_input[i] + j * ofdm_data_num_
                        + k * encoded_bytes_per_block,
                    encoded_bytes_per_block, mod_order_bits);
            }
        }
    }

    Table<int8_t> dl_encoded_bits;
    dl_encoded_bits.malloc(this->frame_.NumDLSyms() * num_blocks_per_symbol,
        encoded_bytes_per_block, Agora_memory::Alignment_t::k64Align);

    // Encode downlink bits
    for (size_t i = 0; i < this->frame_.NumDLSyms(); i++) {
        for (size_t j = 0; j < ldpc_config_.num_blocks_in_symbol() * ue_ant_num_; j++) {
            ldpc_encode_helper(ldpc_config_.base_graph(), ldpc_config_.expansion_factor(),
                ldpc_config_.num_rows(),
                dl_encoded_bits[i * num_blocks_per_symbol + j],
                temp_parity_buffer, dl_bits_[i] + j * bytes_per_block);
        }
    }
    dl_mod_input.calloc(this->frame_.NumDLSyms(), ofdm_data_num_ * ue_ant_num_,
        Agora_memory::Alignment_t::k32Align);
    for (size_t i = 0; i < this->frame_.NumDLSyms(); i++) {
        for (size_t j = 0; j < ue_ant_num_; j++) {
            for (size_t k = 0; k < ldpc_config_.num_blocks_in_symbol(); k++) {
                adapt_bits_for_mod(
                    reinterpret_cast<uint8_t*>(
                        dl_encoded_bits[i * num_blocks_per_symbol
                            + j * ldpc_config_.num_blocks_in_symbol() + k]),
                    dl_mod_input[i] + j * ofdm_data_num_
                        + k * encoded_bytes_per_block,
                    encoded_bytes_per_block, mod_order_bits);
            }
        }
    }

    // Generate freq-domain downlink symbols
    Table<complex_float> dl_iq_ifft;
    dl_iq_ifft.calloc(this->frame_.NumDLSyms(), ofdm_ca_num_ * ue_ant_num_,
        Agora_memory::Alignment_t::k64Align);
    for (size_t i = 0; i < this->frame_.NumDLSyms(); i++) {
        for (size_t u = 0; u < ue_ant_num_; u++) {
            size_t p = u * ofdm_data_num_;
            size_t q = u * ofdm_ca_num_;

            for (size_t j = ofdm_data_start_; j < ofdm_data_stop_; j++) {
                int k = j - ofdm_data_start_;
                size_t s = p + k;
                if (k % ofdm_pilot_spacing_ != 0) {
                    dl_iq_f_[i][q + j]
                        = mod_single_uint8(dl_mod_input[i][s], mod_table);
                } else
                    dl_iq_f_[i][q + j] = ue_specific_pilot[u][k];
                dl_iq_ifft[i][q + j] = dl_iq_f_[i][q + j];
            }
            CommsLib::IFFT(&dl_iq_ifft[i][q], ofdm_ca_num_, false);
        }
    }

    // Generate freq-domain uplink symbols
    Table<complex_float> ul_iq_ifft;
    ul_iq_ifft.calloc(this->frame_.NumULSyms(), ofdm_ca_num_ * ue_ant_num_,
        Agora_memory::Alignment_t::k64Align);
    for (size_t i = 0; i < this->frame_.NumULSyms(); i++) {
        for (size_t u = 0; u < ue_ant_num_; u++) {

            size_t p = u * ofdm_data_num_;
            size_t q = u * ofdm_ca_num_;

            for (size_t j = ofdm_data_start_; j < ofdm_data_stop_; j++) {
                size_t k = j - ofdm_data_start_;
                size_t s = p + k;
                ul_iq_f_[i][q + j]
                    = mod_single_uint8(ul_mod_input[i][s], mod_table);
                ul_iq_ifft[i][q + j] = ul_iq_f_[i][q + j];
            }

            CommsLib::IFFT(&ul_iq_ifft[i][q], ofdm_ca_num_, false);
        }
    }

    // Find normalization factor through searching for max value in IFFT results
    float max_val = CommsLib::find_max_abs(
        ul_iq_ifft, this->frame_.NumULSyms(), ue_ant_num_ * ofdm_ca_num_);
    float cur_max_val = CommsLib::find_max_abs(
        dl_iq_ifft, this->frame_.NumDLSyms(), ue_ant_num_ * ofdm_ca_num_);
    if (cur_max_val > max_val)
        max_val = cur_max_val;
    cur_max_val
        = CommsLib::find_max_abs(ue_pilot_ifft, ue_ant_num_, ofdm_ca_num_);
    if (cur_max_val > max_val)
        max_val = cur_max_val;
    cur_max_val = CommsLib::find_max_abs(pilot_ifft, ofdm_ca_num_);
    if (cur_max_val > max_val)
        max_val = cur_max_val;

    scale = 2 * max_val; // additional 2^2 (6dB) power backoff

    // Generate time domain symbols for downlink
    for (size_t i = 0; i < this->frame_.NumDLSyms(); i++) {
        for (size_t u = 0; u < ue_ant_num_; u++) {
            size_t q = u * ofdm_ca_num_;
            size_t r = u * sampsPerSymbol;
            CommsLib::ifft2tx(&dl_iq_ifft[i][q], &dl_iq_t_[i][r], ofdm_ca_num_,
                ofdm_tx_zero_prefix_, cp_len_, scale);
        }
    }

    // Generate time domain uplink symbols
    for (size_t i = 0; i < this->frame_.NumULSyms(); i++) {
        for (size_t u = 0; u < ue_ant_num_; u++) {
            size_t q = u * ofdm_ca_num_;
            size_t r = u * sampsPerSymbol;
            CommsLib::ifft2tx(&ul_iq_ifft[i][q], &ul_iq_t_[i][r], ofdm_ca_num_,
                ofdm_tx_zero_prefix_, cp_len_, scale);
        }
    }

    // Generate time domain ue-specific pilot symbols
    for (size_t i = 0; i < ue_ant_num_; i++) {
        CommsLib::ifft2tx(ue_pilot_ifft[i], ue_specific_pilot_t[i], ofdm_ca_num_,
            ofdm_tx_zero_prefix_, cp_len_, scale);
        if (kDebugPrintPilot) {
            std::printf("ue_specific_pilot%zu=[", i);
            for (size_t j = 0; j < ofdm_ca_num_; j++)
                std::printf("%2.4f+%2.4fi ", ue_pilot_ifft[i][j].re,
                    ue_pilot_ifft[i][j].im);
            std::printf("]\n");
        }
    }

    pilot_ci16.resize(sampsPerSymbol, 0);
    CommsLib::ifft2tx(pilot_ifft, (std::complex<int16_t>*)pilot_ci16.data(),
        ofdm_ca_num_, ofdm_tx_zero_prefix_, cp_len_, scale);

    for (size_t i = 0; i < ofdm_ca_num_; i++)
        pilot_cf32.push_back(std::complex<float>(
            pilot_ifft[i].re / scale, pilot_ifft[i].im / scale));
    pilot_cf32.insert(pilot_cf32.begin(), pilot_cf32.end() - cp_len_,
        pilot_cf32.end()); // add CP

    // generate a UINT32 version to write to FPGA buffers
    pilot = Utils::cfloat32_to_uint32(pilot_cf32, false, "QI");

    std::vector<uint32_t> pre_uint32(ofdm_tx_zero_prefix_, 0);
    pilot.insert(pilot.begin(), pre_uint32.begin(), pre_uint32.end());
    pilot.resize(sampsPerSymbol);

    if (kDebugPrintPilot) {
        std::cout << "Pilot data: " << std::endl;
        for (size_t i = 0; i < ofdm_data_num_; i++)
            std::cout << pilots_[i].re << "+1i*" << pilots_[i].im << ",";
        std::cout << std::endl;
    }

    delete[] temp_parity_buffer;
    dl_encoded_bits.free();
    ul_iq_ifft.free();
    dl_iq_ifft.free();
    ue_pilot_ifft.free();
    ul_mod_input.free();
    ul_encoded_bits.free();
    dl_mod_input.free(); 
    //ue_specific_pilot_t.free(); TODO: leaking but causes an assertion in UE mode
    free_buffer_1d(&pilot_ifft);
    //ue_specific_pilot.free();  TODO: leaking but causes an assertion when free'd here
}

Config::~Config()
{
    if (pilots_ != nullptr) {
        std::free(pilots_);
        pilots_ = nullptr;
    }
    if (pilots_sgn_ != nullptr) {
        std::free(pilots_sgn_);
        pilots_sgn_ = nullptr;
    }
    mod_table.free();
    dl_bits_.free();
    ul_bits_.free();
    dl_iq_f_.free();
    dl_iq_t_.free();
    ul_iq_f_.free();
    ul_iq_t_.free();
}

/* Returns SIZE_MAX if symbol not a DL symbol, otherwise the index into frame_.dl_symbols_ */
size_t Config::GetDLSymbolIdx(size_t frame_id, size_t symbol_id) const
{
    return this->frame_.GetDLSymbolIdx(symbol_id);
}

/* Returns SIZE_MAX if symbol not a DL symbol, otherwise the index into frame_.dl_symbols_ */
size_t Config::GetULSymbolIdx(size_t frame_id, size_t symbol_id) const
{
    return this->frame_.GetULSymbolIdx(symbol_id);
}

/* Returns SIZE_MAX if symbol not a Pilot symbol, otherwise the index into frame_.pilot_symbols_ */
size_t Config::GetPilotSymbolIdx(size_t frame_id, size_t symbol_id) const
{
    return this->frame_.GetPilotSymbolIdx(symbol_id);
}

/* TODO Inspect and document */
size_t Config::GetSymbolId(size_t input_id )
{
    size_t symbol_id = SIZE_MAX;

    if (input_id < this->frame_.NumPilotSyms())
    {
        symbol_id = this->frame().GetPilotSymbol( input_id );
    } else {
        int new_idx = input_id - this->frame_.NumPilotSyms();
        if ((new_idx >= 0) && (static_cast<size_t>(new_idx) < this->frame_.NumULSyms())) {
            symbol_id = this->frame().GetULSymbol( new_idx );
        }
    }
    return symbol_id;
}


/* Returns True if symbol is valid index and is of symbol type 'P' or 
   if user equiptment and is a client dl pilot.  False otherwise */
bool Config::IsPilot(size_t frame_id, size_t symbol_id) const {
    bool is_pilot = false;
    assert( symbol_id < this->frame_.NumTotalSyms());
    char s = frame_.frame_identifier().at(symbol_id);
#ifdef DEBUG3
    std::printf("IsPilot(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
    if (isUE == true) {
        if ((s == 'D') && (this->frame_.client_dl_pilot_symbols() > 0) )
        {
            size_t dl_index = this->frame_.GetDLSymbolIdx(symbol_id);
            is_pilot = (this->frame_.client_dl_pilot_symbols() > dl_index);
        }
        // else { is_pilot = false; } Not needed due to default init 
    } else { /* TODO should use the symbol type here */
        is_pilot = (s == 'P');
    }
    return is_pilot;
}

bool Config::IsCalDlPilot(size_t frame_id, size_t symbol_id) const
{
    bool is_cal_dl_pilot = false;
    assert( symbol_id < this->frame_.NumTotalSyms());
    if (isUE == false) {
        is_cal_dl_pilot = (this->frame_.frame_identifier().at(symbol_id) == 'C');
    }
    return is_cal_dl_pilot;
}

bool Config::IsCalUlPilot(size_t frame_id, size_t symbol_id) const
{
    bool is_cal_ul_pilot = false;
    assert( symbol_id < this->frame_.NumTotalSyms());
    if (isUE == false) {
        is_cal_ul_pilot = (this->frame_.frame_identifier().at(symbol_id) == 'L');
    }
    return is_cal_ul_pilot;
}

bool Config::IsUplink(size_t frame_id, size_t symbol_id) const
{
    assert( symbol_id < this->frame_.NumTotalSyms());
    char s = frame_.frame_identifier().at(symbol_id);
#ifdef DEBUG3
    std::printf("IsUplink(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
    return (s == 'U');
}

bool Config::IsDownlink(size_t frame_id, size_t symbol_id) const
{
    assert(symbol_id < this->frame_.NumTotalSyms());
    char s = frame_.frame_identifier().at(symbol_id);
#ifdef DEBUG3
    std::printf("IsDownlink(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
    if (isUE == true) {
        return ((s == 'D') && (this->IsPilot(frame_id, symbol_id) == false));
    }
    else {
        return (s == 'D');
    }
}

/* TODO change to table lookup */
SymbolType Config::get_symbol_type(size_t frame_id, size_t symbol_id)
{
    assert((isUE == false)); // Currently implemented for only the Agora server
    char s = this->frame_.frame_identifier().at(symbol_id);
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
    }
    rt_assert(false, std::string("Should not reach here") + std::to_string(s));
    return SymbolType::kUnknown;
} 

extern "C" {
__attribute__((visibility("default"))) Config* Config_new(char* filename)
{

    auto* cfg = new Config(filename);
    cfg->genData();
    return cfg;
}
}
