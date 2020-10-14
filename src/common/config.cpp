#include "config.hpp"
#include "utils_ldpc.hpp"
#include <boost/range/algorithm/count.hpp>

Config::Config(std::string jsonfile)
{
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
    nCells = tddConf.value("cells", 1);
    channel = tddConf.value("channel", "A");
    nChannels = std::min(channel.size(), (size_t)2);
    BS_ANT_NUM = tddConf.value("antenna_num", 8);
    isUE = tddConf.value("UE", false);
    UE_NUM = tddConf.value("ue_num", 8);
    UE_ANT_NUM = UE_NUM;
    if (serial_file.size() > 0)
        Utils::loadDevices(serial_file, radio_ids);
    if (radio_ids.size() != 0) {
        nRadios = radio_ids.size();
        nAntennas = nChannels * nRadios;
        if (isUE) {
            UE_ANT_NUM = nAntennas;
            UE_NUM = nRadios;
        } else {
            if (ref_ant >= nAntennas)
                ref_ant = 0;
            if (BS_ANT_NUM != nAntennas)
                BS_ANT_NUM = nAntennas;
        }
    } else
        nRadios = tddConf.value("radio_num", isUE ? UE_ANT_NUM : BS_ANT_NUM);

    if (kUseArgos || kUseUHD) {
        rt_assert(nRadios != 0, "Error: No radios exist in Argos mode");
    }

    /* radio configurations */
    freq = tddConf.value("frequency", 3.6e9);
    txgainA = tddConf.value("txgainA", 20);
    rxgainA = tddConf.value("rxgainA", 20);
    txgainB = tddConf.value("txgainB", 20);
    rxgainB = tddConf.value("rxgainB", 20);
    calTxGainA = tddConf.value("calTxGainA", 10);
    calTxGainB = tddConf.value("calTxGainB", 10);
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

    mac_rx_port = tddConf.value("mac_rx_port", 5000);
    mac_tx_port = tddConf.value("mac_tx_port", 4000);
    init_mac_running = tddConf.value("init_mac_running", false);

    /* frame configurations */
    CP_LEN = tddConf.value("cp_len", 0);
    OFDM_CA_NUM = tddConf.value("ofdm_ca_num", 2048);
    OFDM_DATA_NUM = tddConf.value("ofdm_data_num", 1200);
    ofdm_tx_zero_prefix_ = tddConf.value("ofdm_tx_zero_prefix", 0);
    ofdm_tx_zero_postfix_ = tddConf.value("ofdm_tx_zero_postfix", 0);
    ofdm_rx_zero_prefix_bs_
        = tddConf.value("ofdm_rx_zero_prefix_bs", 0) + CP_LEN;
    ofdm_rx_zero_prefix_client_
        = tddConf.value("ofdm_rx_zero_prefix_client", 0);
    rt_assert(OFDM_DATA_NUM % kSCsPerCacheline == 0,
        "OFDM_DATA_NUM must be a multiple of subcarriers per cacheline");
    rt_assert(OFDM_DATA_NUM % kTransposeBlockSize == 0,
        "Transpose block size must divide number of OFDM data subcarriers");
    OFDM_PILOT_SPACING = tddConf.value("ofdm_pilot_spacing", 16);
    OFDM_DATA_START
        = tddConf.value("ofdm_data_start", (OFDM_CA_NUM - OFDM_DATA_NUM) / 2);
    OFDM_DATA_STOP = OFDM_DATA_START + OFDM_DATA_NUM;
    downlink_mode = tddConf.value("downlink_mode", false);
    bigstation_mode = tddConf.value("bigstation_mode", false);
    freq_orthogonal_pilot = tddConf.value("freq_orthogonal_pilot", false);
    correct_phase_shift = tddConf.value("correct_phase_shift", false);
    DL_PILOT_SYMS = tddConf.value("client_dl_pilot_syms", 0);
    UL_PILOT_SYMS = tddConf.value("client_ul_pilot_syms", 0);
    cl_tx_advance = tddConf.value("tx_advance", 100);
    hw_framer = tddConf.value("hw_framer", true);
    if (tddConf.find("frames") == tddConf.end()) {
        symbol_num_perframe = tddConf.value("symbol_num_perframe", 70);
        pilot_symbol_num_perframe = tddConf.value(
            "pilot_num", freq_orthogonal_pilot ? 1 : UE_ANT_NUM);
        ul_data_symbol_num_perframe = tddConf.value("ul_symbol_num_perframe",
            downlink_mode
                ? 0
                : symbol_num_perframe - pilot_symbol_num_perframe - 1);
        dl_data_symbol_num_perframe
            = tddConf.value("dl_symbol_num_perframe", downlink_mode ? 10 : 0);
        dl_data_symbol_start = tddConf.value("dl_data_symbol_start", 10);
        std::string sched("B");
        for (size_t s = 0; s < pilot_symbol_num_perframe; s++)
            sched += "P";
        // Below it is assumed either dl or ul to be active at one time
        if (downlink_mode) {
            size_t dl_symbol_start
                = 1 + pilot_symbol_num_perframe + dl_data_symbol_start;
            size_t dl_symbol_end
                = dl_symbol_start + dl_data_symbol_num_perframe;
            for (size_t s = 1 + pilot_symbol_num_perframe; s < dl_symbol_start;
                 s++)
                sched += "G";
            for (size_t s = dl_symbol_start; s < dl_symbol_end; s++)
                sched += "D";
            for (size_t s = dl_symbol_end; s < symbol_num_perframe; s++)
                sched += "G";
        } else {
            size_t ul_data_symbol_end
                = 1 + pilot_symbol_num_perframe + ul_data_symbol_num_perframe;
            for (size_t s = 1 + pilot_symbol_num_perframe;
                 s < ul_data_symbol_end; s++)
                sched += "U";
            for (size_t s = ul_data_symbol_end; s < symbol_num_perframe; s++)
                sched += "G";
        }
        frames.push_back(sched);
        printf("Config: Frame schedule %s (%zu symbols)\n", sched.c_str(),
            sched.size());
    } else {
        json jframes = tddConf.value("frames", json::array());
        for (size_t f = 0; f < jframes.size(); f++) {
            frames.push_back(jframes.at(f).get<std::string>());
        }
    }

    beaconSymbols = Utils::loadSymbols(frames, 'B');
    pilotSymbols = Utils::loadSymbols(frames, 'P');
    ULSymbols = Utils::loadSymbols(frames, 'U');
    DLSymbols = Utils::loadSymbols(frames, 'D');
    ULCalSymbols = Utils::loadSymbols(frames, 'L');
    DLCalSymbols = Utils::loadSymbols(frames, 'C');
    recipCalEn = (ULCalSymbols[0].size() == 1 and DLCalSymbols[0].size() == 1);

    symbol_num_perframe = frames.at(0).size();
    beacon_symbol_num_perframe = beaconSymbols[0].size();
    pilot_symbol_num_perframe = pilotSymbols[0].size();
    data_symbol_num_perframe = symbol_num_perframe - pilot_symbol_num_perframe
        - beacon_symbol_num_perframe;
    ul_data_symbol_num_perframe = ULSymbols[0].size();
    dl_data_symbol_num_perframe = DLSymbols[0].size();
    downlink_mode = dl_data_symbol_num_perframe > 0;
    dl_data_symbol_start
        = dl_data_symbol_num_perframe > 0 ? DLSymbols[0].front() : 0;
    dl_data_symbol_end
        = dl_data_symbol_num_perframe > 0 ? DLSymbols[0].back() + 1 : 0;

    if (isUE and !freq_orthogonal_pilot
        and UE_ANT_NUM != pilot_symbol_num_perframe) {
        rt_assert(false, "Number of pilot symbols doesn't match number of UEs");
    }
    if (!isUE and !freq_orthogonal_pilot
        and tddConf.find("ue_num") == tddConf.end()) {
        UE_NUM = pilot_symbol_num_perframe;
        UE_ANT_NUM = UE_NUM;
    }
    ue_ant_offset = tddConf.value("ue_ant_offset", 0);
    total_ue_ant_num = tddConf.value("total_ue_ant_num", UE_ANT_NUM);

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
    demul_events_per_symbol = 1 + (OFDM_DATA_NUM - 1) / demul_block_size;

    zf_block_size = freq_orthogonal_pilot ? UE_ANT_NUM
                                          : tddConf.value("zf_block_size", 1);
    zf_events_per_symbol = 1 + (OFDM_DATA_NUM - 1) / zf_block_size;

    fft_block_size = tddConf.value("fft_block_size", 1);
    encode_block_size = tddConf.value("encode_block_size", 1);

    /* LDPC Coding configurations */
    LDPC_config_ul.Bg = tddConf.value("base_graph", 1);
    LDPC_config_ul.earlyTermination = tddConf.value("earlyTermination", 1);
    LDPC_config_ul.decoderIter = tddConf.value("decoderIter", 5);
    LDPC_config_ul.Zc = tddConf.value("Zc", 72);
    LDPC_config_ul.nRows
        = tddConf.value("nRows", (LDPC_config_ul.Bg == 1) ? 46 : 42);
    LDPC_config_ul.code_rate
        = compute_code_rate(LDPC_config_ul.nRows, LDPC_config_ul.Bg);
    LDPC_config_ul.cbLen
        = ldpc_num_input_bits(LDPC_config_ul.Bg, LDPC_config_ul.Zc);
    LDPC_config_ul.cbCodewLen = ldpc_num_encoded_bits(
        LDPC_config_ul.Bg, LDPC_config_ul.Zc, LDPC_config_ul.nRows);
    // Use one code block per symbol
    LDPC_config_ul.nCb = ul_data_symbol_num_perframe;
    LDPC_config_ul.map_symbols_to_cbs(
        ul_data_symbol_num_perframe, OFDM_DATA_NUM, mod_order_bits);

    LDPC_config_dl.Bg = LDPC_config_ul;
    LDPC_config_dl.earlyTermination = LDPC_config_ul.earlyTermination;
    LDPC_config_dl.decoderIter = LDPC_config_ul.decoderIter;
    LDPC_config_dl.Zc = LDPC_config_ul.Zc;
    LDPC_config_dl.nRows = LDPC_config_ul.nRows;
    LDPC_config_dl.code_rate = LDPC_config_ul.code_rate;
    LDPC_config_dl.cbLen = LDPC_config_ul.cbLen;
    LDPC_config_dl.cbCodewLen = LDPC_config_ul.cbCodewLen;
    // Use one code block per symbol
    LDPC_config_dl.nCb = dl_data_symbol_num_perframe;
    LDPC_config_dl.map_symbols_to_cbs(
        dl_data_symbol_num_perframe, OFDM_DATA_NUM, mod_order_bits);

    /* Modulation configurations */
    mod_order_bits = modulation == "64QAM"
        ? CommsLib::QAM64
        : (modulation == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK);
    update_mod_cfgs(mod_order_bits);

    printf("Config: LDPC: Zc: %d, (UL: %zu, DL: %zu) code blocks per frame, %d "
           "information bits per encoding, %d bits per encoded code word, "
           "decoder iterations: %d, code rate %.3f (nRows = %zu)\n",
        LDPC_config_ul.Zc, LDPC_config_ul.nCb, LDPC_config_dl.nCb,
        LDPC_config_ul.cbLen, LDPC_config_ul.cbCodewLen,
        LDPC_config_ul.decoderIter, LDPC_config_ul.code_rate,
        LDPC_config_ul.nRows);

    fft_in_rru = tddConf.value("fft_in_rru", false);

    sampsPerSymbol
        = ofdm_tx_zero_prefix_ + OFDM_CA_NUM + CP_LEN + ofdm_tx_zero_postfix_;
    packet_length
        = Packet::kOffsetOfData + (2 * sizeof(short) * sampsPerSymbol);
    rt_assert(
        packet_length < 9000, "Packet size must be smaller than jumbo frame");

    num_bytes_per_cb = LDPC_config_ul.cbLen / 8; // TODO: Use bits_to_bytes()?
    data_bytes_num_persymbol = num_bytes_per_cb; // One code block per symbol
    mac_packet_length = data_bytes_num_persymbol;
    mac_payload_length = mac_packet_length - MacPacket::kOffsetOfData;
    mac_packets_perframe = ul_data_symbol_num_perframe - UL_PILOT_SYMS;
    mac_data_bytes_num_perframe = mac_payload_length * mac_packets_perframe;
    mac_bytes_num_perframe = mac_packet_length * mac_packets_perframe;

    running = true;
    printf("Config: %zu BS antennas, %zu UE antennas, %zu pilot symbols per "
           "frame,\n\t"
           "%zu uplink data symbols per frame, %zu downlink data "
           "symbols per frame,\n\t"
           "%zu OFDM subcarriers (%zu data subcarriers), modulation %s,\n\t"
           "%zu MAC data bytes per frame, %zu MAC bytes per frame\n",
        BS_ANT_NUM, UE_ANT_NUM, pilot_symbol_num_perframe,
        ul_data_symbol_num_perframe, dl_data_symbol_num_perframe, OFDM_CA_NUM,
        OFDM_DATA_NUM, modulation.c_str(), mac_data_bytes_num_perframe,
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
        = CommsLib::getSequence(OFDM_DATA_NUM, CommsLib::LTE_ZADOFF_CHU);
    auto zc_seq = Utils::double_to_cfloat(zc_seq_double);
    auto common_pilot
        = CommsLib::seqCyclicShift(zc_seq, M_PI / 4); // Used in LTE SRS

    pilots_ = (complex_float*)aligned_alloc(
        64, OFDM_DATA_NUM * sizeof(complex_float));
    pilots_sgn_ = (complex_float*)aligned_alloc(
        64, OFDM_DATA_NUM * sizeof(complex_float)); // used in CSI estimation
    for (size_t i = 0; i < OFDM_DATA_NUM; i++) {
        pilots_[i] = { common_pilot[i].real(), common_pilot[i].imag() };
        auto pilot_sgn
            = common_pilot[i] / (float)std::pow(std::abs(common_pilot[i]), 2);
        pilots_sgn_[i] = { pilot_sgn.real(), pilot_sgn.imag() };
    }
    complex_float* pilot_ifft;
    alloc_buffer_1d(&pilot_ifft, OFDM_CA_NUM, 64, 1);
    for (size_t j = 0; j < OFDM_DATA_NUM; j++)
        pilot_ifft[j + OFDM_DATA_START] = pilots_[j];
    CommsLib::IFFT(pilot_ifft, OFDM_CA_NUM, false);

    // Generate UE-specific pilots based on Zadoff-Chu sequence for phase tracking
    ue_specific_pilot.malloc(UE_ANT_NUM, OFDM_DATA_NUM, 64);
    ue_specific_pilot_t.calloc(UE_ANT_NUM, sampsPerSymbol, 64);
    Table<complex_float> ue_pilot_ifft;
    ue_pilot_ifft.calloc(UE_ANT_NUM, OFDM_CA_NUM, 64);
    auto zc_ue_pilot_double
        = CommsLib::getSequence(OFDM_DATA_NUM, CommsLib::LTE_ZADOFF_CHU);
    auto zc_ue_pilot = Utils::double_to_cfloat(zc_ue_pilot_double);
    for (size_t i = 0; i < UE_ANT_NUM; i++) {
        auto zc_ue_pilot_i = CommsLib::seqCyclicShift(
            zc_ue_pilot, (i + ue_ant_offset) * (float)M_PI / 6); // LTE DMRS
        for (size_t j = 0; j < OFDM_DATA_NUM; j++) {
            ue_specific_pilot[i][j]
                = { zc_ue_pilot_i[j].real(), zc_ue_pilot_i[j].imag() };
            ue_pilot_ifft[i][j + OFDM_DATA_START] = ue_specific_pilot[i][j];
        }
        CommsLib::IFFT(ue_pilot_ifft[i], OFDM_CA_NUM, false);
    }

    // Get uplink and downlink raw bits either from file or random numbers
    size_t num_bytes_per_cb_pad = roundup<64>(num_bytes_per_cb);
    size_t num_bytes_per_cb_all_ue = num_bytes_per_cb_pad * UE_ANT_NUM;
    dl_bits = reinterpret_cast<int8_t*>(memalign(
        64, LDPC_config_dl.nCb * num_bytes_per_cb_all_ue * sizeof(int8_t)));
    dl_iq_f.calloc(dl_data_symbol_num_perframe, OFDM_CA_NUM * UE_ANT_NUM, 64);
    dl_iq_t.calloc(
        dl_data_symbol_num_perframe, sampsPerSymbol * UE_ANT_NUM, 64);

    ul_bits = reinterpret_cast<int8_t*>(memalign(
        64, LDPC_config_ul.nCb * num_bytes_per_cb_all_ue * sizeof(int8_t)));
    ul_iq_f.calloc(ul_data_symbol_num_perframe, OFDM_CA_NUM * UE_ANT_NUM, 64);
    ul_iq_t.calloc(
        ul_data_symbol_num_perframe, sampsPerSymbol * UE_ANT_NUM, 64);

#ifdef GENERATE_DATA
    for (size_t i = 0; i < LDPC_config_ul.nCb * num_bytes_per_cb_all_ue; i++)
        ul_bits[i] = static_cast<int8_t>(rand());
    for (size_t i = 0; i < LDPC_config_dl.nCb * num_bytes_per_cb_all_ue; i++)
        dl_bits[i] = static_cast<int8_t>(rand());
#else
    std::string cur_directory1 = TOSTRING(PROJECT_DIRECTORY);
    std::string filename1 = cur_directory1 + "/data/LDPC_orig_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant"
        + std::to_string(total_ue_ant_num) + ".bin";
    std::cout << "Config: Reading raw data from " << filename1 << std::endl;
    FILE* fd = fopen(filename1.c_str(), "rb");
    if (fd == nullptr) {
        printf("Failed to open antenna file %s. Error %s.\n", filename1.c_str(),
            strerror(errno));
        exit(-1);
    }
    for (size_t i = 0; i < LDPC_config_ul.nCb; i++) {
        if (fseek(fd, num_bytes_per_cb * ue_ant_offset, SEEK_SET) != 0)
            return;
        for (size_t j = 0; j < UE_ANT_NUM; j++) {
            // Read one code block to padded locations
            size_t r
                = fread(ul_bits + (i * UE_ANT_NUM + j) * num_bytes_per_cb_pad,
                    sizeof(int8_t), num_bytes_per_cb, fd);
            if (r < num_bytes_per_cb)
                printf("bad read from file %s (batch %zu) \n",
                    filename1.c_str(), i);
        }
        if (fseek(fd,
                num_bytes_per_cb
                    * (total_ue_ant_num - ue_ant_offset - UE_ANT_NUM),
                SEEK_SET)
            != 0)
            return;
    }
    for (size_t i = 0; i < LDPC_config_dl.nCb; i++) {
        for (size_t j = 0; j < UE_ANT_NUM; j++) {
            size_t r
                = fread(dl_bits + (i * UE_ANT_NUM + j) * num_bytes_per_cb_pad,
                    sizeof(int8_t), num_bytes_per_cb, fd);
            if (r < num_bytes_per_cb)
                printf("bad read from file %s (batch %zu) \n",
                    filename1.c_str(), i);
        }
    }
    fclose(fd);
#endif

    const size_t bytes_per_block = bits_to_bytes(LDPC_config.cbLen);
    const size_t encoded_bytes_per_block
        = bits_to_bytes(LDPC_config.cbCodewLen);

    // Encode uplink bits
    Table<int8_t> ul_encoded_bits;
    ul_encoded_bits.malloc(
        LDPC_config_ul.nCb * UE_ANT_NUM, encoded_bytes_per_block, 64);

    int8_t* temp_parity_buffer = new int8_t[ldpc_encoding_parity_buf_size(
        LDPC_config_ul.Bg, LDPC_config_ul.Zc)];
    for (size_t i = 0; i < LDPC_config_ul.nCb * UE_ANT_NUM; i++) {
        ldpc_encode_helper(LDPC_config_ul.Bg, LDPC_config_ul.Zc,
            LDPC_config_ul.nRows, ul_encoded_bits[i], temp_parity_buffer,
            ul_bits + i * num_bytes_per_cb_pad);
    }

    Table<uint8_t> ul_mod_input;
    ul_mod_input.calloc(
        ul_data_symbol_num_perframe, OFDM_DATA_NUM * UE_ANT_NUM, 32);

    for (size_t i = 0; i < LDPC_config_ul.nCb; i++) {
        for (size_t j = 0; j < UE_ANT_NUM; j++) {
            auto* ul_encoded_ptr = reinterpret_cast<uint8_t*>(
                ul_encoded_bits[i * UE_ANT_NUM + j]);
            for (size_t k = 0; k < LDPC_config_ul.lut_cb_chunks_bytes[i].size();
                 k++) {
                auto* ul_mod_ptr
                    = ul_mod_input[LDPC_config_ul.lut_cb_to_symbol[i][k]]
                    + j * OFDM_DATA_NUM
                    + LDPC_config_ul.get_chunk_start_in_symbol(i, k);
                adapt_bits_for_mod(ul_encoded_ptr, ul_mod_ptr,
                    LDPC_config_ul.lut_cb_chunks_bytes[i][k], mod_order_bits);
                ul_encoded_ptr += LDPC_config_ul.lut_cb_chunks_bytes[i];
            }
        }
    }

    Table<int8_t> dl_encoded_bits;
    dl_encoded_bits.malloc(
        LDPC_config_dl.nCb * UE_ANT_NUM, encoded_bytes_per_block, 64);

    // Encode downlink bits
    for (size_t i = 0; i < LDPC_config_dl.nCb * UE_ANT_NUM; i++) {
        ldpc_encode_helper(LDPC_config_dl.Bg, LDPC_config_dl.Zc,
            LDPC_config_dl.nRows, dl_encoded_bits[i], temp_parity_buffer,
            dl_bits + i * num_bytes_per_cb_pad);
    }

    Table<uint8_t> dl_mod_input;
    dl_mod_input.calloc(
        dl_data_symbol_num_perframe, OFDM_DATA_NUM * UE_ANT_NUM, 32);

    for (size_t i = 0; i < LDPC_config_dl.nCb; i++) {
        for (size_t j = 0; j < UE_ANT_NUM; j++) {
            auto* dl_encoded_ptr = reinterpret_cast<uint8_t*>(
                dl_encoded_bits[i * UE_ANT_NUM + j]);
            for (size_t k = 0; k < LDPC_config_dl.lut_cb_chunks_bytes[i].size();
                 k++) {
                auto* dl_mod_ptr
                    = dl_mod_input[LDPC_config_dl.lut_cb_to_symbol[i][k]]
                    + j * OFDM_DATA_NUM
                    + LDPC_config_dl.get_chunk_start_in_symbol(i, k);
                adapt_bits_for_mod(dl_encoded_ptr, dl_mod_ptr,
                    LDPC_config_dl.lut_cb_chunks_bytes[i][k], mod_order_bits);
                dl_encoded_ptr += LDPC_config_dl.lut_cb_chunks_bytes[i];
            }
        }
    }

    // Generate freq-domain downlink symbols
    Table<complex_float> dl_iq_ifft;
    dl_iq_ifft.calloc(
        dl_data_symbol_num_perframe, OFDM_CA_NUM * UE_ANT_NUM, 64);
    for (size_t i = 0; i < dl_data_symbol_num_perframe; i++) {
        for (size_t u = 0; u < UE_ANT_NUM; u++) {
            size_t p = u * OFDM_DATA_NUM;
            size_t q = u * OFDM_CA_NUM;

            for (size_t j = OFDM_DATA_START; j < OFDM_DATA_STOP; j++) {
                int k = j - OFDM_DATA_START;
                size_t s = p + k;
                if (k % OFDM_PILOT_SPACING != 0) {
                    dl_iq_f[i][q + j]
                        = mod_single_uint8(dl_mod_input[i][s], mod_table);
                } else
                    dl_iq_f[i][q + j] = ue_specific_pilot[u][k];
                dl_iq_ifft[i][q + j] = dl_iq_f[i][q + j];
            }
            CommsLib::IFFT(&dl_iq_ifft[i][q], OFDM_CA_NUM, false);
        }
    }

    // Generate freq-domain uplink symbols
    Table<complex_float> ul_iq_ifft;
    ul_iq_ifft.calloc(
        ul_data_symbol_num_perframe, OFDM_CA_NUM * UE_ANT_NUM, 64);
    for (size_t i = 0; i < ul_data_symbol_num_perframe; i++) {
        for (size_t u = 0; u < UE_ANT_NUM; u++) {

            size_t p = u * OFDM_DATA_NUM;
            size_t q = u * OFDM_CA_NUM;

            for (size_t j = OFDM_DATA_START; j < OFDM_DATA_STOP; j++) {
                size_t k = j - OFDM_DATA_START;
                size_t s = p + k;
                ul_iq_f[i][q + j]
                    = mod_single_uint8(ul_mod_input[i][s], mod_table);
                ul_iq_ifft[i][q + j] = ul_iq_f[i][q + j];
            }

            CommsLib::IFFT(&ul_iq_ifft[i][q], OFDM_CA_NUM, false);
        }
    }

    // Find normalization factor through searching for max value in IFFT results
    float max_val = CommsLib::find_max_abs(
        ul_iq_ifft, ul_data_symbol_num_perframe, UE_ANT_NUM * OFDM_CA_NUM);
    float cur_max_val = CommsLib::find_max_abs(
        dl_iq_ifft, dl_data_symbol_num_perframe, UE_ANT_NUM * OFDM_CA_NUM);
    if (cur_max_val > max_val)
        max_val = cur_max_val;
    cur_max_val
        = CommsLib::find_max_abs(ue_pilot_ifft, UE_ANT_NUM, OFDM_CA_NUM);
    if (cur_max_val > max_val)
        max_val = cur_max_val;
    cur_max_val = CommsLib::find_max_abs(pilot_ifft, OFDM_CA_NUM);
    if (cur_max_val > max_val)
        max_val = cur_max_val;

    scale = 2 * max_val; // additional 2^2 (6dB) power backoff

    // Generate time domain symbols for downlink
    for (size_t i = 0; i < dl_data_symbol_num_perframe; i++) {
        for (size_t u = 0; u < UE_ANT_NUM; u++) {
            size_t q = u * OFDM_CA_NUM;
            size_t r = u * sampsPerSymbol;
            CommsLib::ifft2tx(&dl_iq_ifft[i][q], &dl_iq_t[i][r], OFDM_CA_NUM,
                ofdm_tx_zero_prefix_, CP_LEN, scale);
        }
    }

    // Generate time domain uplink symbols
    for (size_t i = 0; i < ul_data_symbol_num_perframe; i++) {
        for (size_t u = 0; u < UE_ANT_NUM; u++) {
            size_t q = u * OFDM_CA_NUM;
            size_t r = u * sampsPerSymbol;
            CommsLib::ifft2tx(&ul_iq_ifft[i][q], &ul_iq_t[i][r], OFDM_CA_NUM,
                ofdm_tx_zero_prefix_, CP_LEN, scale);
        }
    }

    // Generate time domain ue-specific pilot symbols
    for (size_t i = 0; i < UE_ANT_NUM; i++) {
        CommsLib::ifft2tx(ue_pilot_ifft[i], ue_specific_pilot_t[i], OFDM_CA_NUM,
            ofdm_tx_zero_prefix_, CP_LEN, scale);
        if (kDebugPrintPilot) {
            printf("ue_specific_pilot%zu=[", i);
            for (size_t j = 0; j < OFDM_CA_NUM; j++)
                printf("%2.4f+%2.4fi ", ue_pilot_ifft[i][j].re,
                    ue_pilot_ifft[i][j].im);
            printf("]\n");
        }
    }

    pilot_ci16.resize(sampsPerSymbol, 0);
    CommsLib::ifft2tx(pilot_ifft, (std::complex<int16_t>*)pilot_ci16.data(),
        OFDM_CA_NUM, ofdm_tx_zero_prefix_, CP_LEN, scale);

    for (size_t i = 0; i < OFDM_CA_NUM; i++)
        pilot_cf32.push_back(std::complex<float>(
            pilot_ifft[i].re / scale, pilot_ifft[i].im / scale));
    pilot_cf32.insert(pilot_cf32.begin(), pilot_cf32.end() - CP_LEN,
        pilot_cf32.end()); // add CP

    // generate a UINT32 version to write to FPGA buffers
    pilot = Utils::cfloat32_to_uint32(pilot_cf32, false, "QI");

    std::vector<uint32_t> pre_uint32(ofdm_tx_zero_prefix_, 0);
    pilot.insert(pilot.begin(), pre_uint32.begin(), pre_uint32.end());
    pilot.resize(sampsPerSymbol);

    if (kDebugPrintPilot) {
        std::cout << "Pilot data: " << std::endl;
        for (size_t i = 0; i < OFDM_DATA_NUM; i++)
            std::cout << pilots_[i].re << "+1i*" << pilots_[i].im << ",";
        std::cout << std::endl;
    }

    ul_iq_ifft.free();
    dl_iq_ifft.free();
    ue_pilot_ifft.free();
    free_buffer_1d(&pilot_ifft);
}

Config::~Config()
{
    free_buffer_1d(&pilots_);
    free_buffer_1d(&pilots_sgn_);
    mod_table.free();
    dl_bits.free();
    ul_bits.free();
    dl_iq_f.free();
    dl_iq_t.free();
    ul_iq_f.free();
    ul_iq_t.free();
}

int Config::getSymbolId(size_t symbol_id)
{
    return (symbol_id < pilot_symbol_num_perframe
            ? pilotSymbols[0][symbol_id]
            : ULSymbols[0][symbol_id - pilot_symbol_num_perframe]);
}

size_t Config::get_dl_symbol_idx(size_t frame_id, size_t symbol_id) const
{
    size_t fid = frame_id % frames.size();
    const auto it
        = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
    if (it != DLSymbols[fid].end())
        return it - DLSymbols[fid].begin();
    else
        return SIZE_MAX;
}

size_t Config::get_pilot_symbol_idx(size_t frame_id, size_t symbol_id) const
{
    size_t fid = frame_id % frames.size();
    const auto it
        = find(pilotSymbols[fid].begin(), pilotSymbols[fid].end(), symbol_id);
    if (it != pilotSymbols[fid].end()) {
#ifdef DEBUG3
        printf("get_pilot_symbol_idx(%zu, %zu) = %zu\n", frame_id, symbol_id,
            it - pilotSymbols[fid].begin());
#endif
        return it - pilotSymbols[fid].begin();
    } else
        return SIZE_MAX;
}

size_t Config::get_ul_symbol_idx(size_t frame_id, size_t symbol_id) const
{
    size_t fid = frame_id % frames.size();
    const auto it
        = find(ULSymbols[fid].begin(), ULSymbols[fid].end(), symbol_id);
    if (it != ULSymbols[fid].end()) {
#ifdef DEBUG3
        printf("get_ul_symbol_idx(%zu, %zu) = %zu\n", frame_id, symbol_id,
            it - ULSymbols[fid].begin());
#endif
        return it - ULSymbols[fid].begin();
    } else
        return SIZE_MAX;
}

bool Config::isPilot(size_t frame_id, size_t symbol_id)
{
    assert(symbol_id < symbol_num_perframe);
    size_t fid = frame_id % frames.size();
    char s = frames[fid].at(symbol_id);
#ifdef DEBUG3
    printf("isPilot(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
    if (isUE) {
        std::vector<size_t>::iterator it;
        it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
        int ind = DL_PILOT_SYMS;
        if (it != DLSymbols[fid].end())
            ind = it - DLSymbols[fid].begin();
        return (ind < (int)DL_PILOT_SYMS);
        // return cfg->frames[fid].at(symbol_id) == 'P' ? true : false;
    } else
        return s == 'P';
    // return (symbol_id < UE_NUM);
}

bool Config::isCalDlPilot(size_t frame_id, size_t symbol_id)
{
    assert(symbol_id < symbol_num_perframe);
    if (isUE)
        return false;
    return frames[frame_id % frames.size()].at(symbol_id) == 'C';
}

bool Config::isCalUlPilot(size_t frame_id, size_t symbol_id)
{
    assert(symbol_id < symbol_num_perframe);
    if (isUE)
        return false;
    return frames[frame_id % frames.size()].at(symbol_id) == 'L';
}

bool Config::isUplink(size_t frame_id, size_t symbol_id)
{
    assert(symbol_id < symbol_num_perframe);
    char s = frames[frame_id % frames.size()].at(symbol_id);
#ifdef DEBUG3
    printf("isUplink(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
    return s == 'U';
}

bool Config::isDownlink(size_t frame_id, size_t symbol_id)
{
    assert(symbol_id < symbol_num_perframe);
    char s = frames[frame_id % frames.size()].at(symbol_id);
#ifdef DEBUG3
    printf("isDownlink(%zu, %zu) = %c\n", frame_id, symbol_id, s);
#endif
    if (isUE)
        return s == 'D' && !this->isPilot(frame_id, symbol_id);
    else
        return s == 'D';
}

SymbolType Config::get_symbol_type(size_t frame_id, size_t symbol_id)
{
    assert(!isUE); // Currently implemented for only the Agora server
    char s = frames[frame_id % frames.size()][symbol_id];
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
