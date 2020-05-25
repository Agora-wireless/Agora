
#include "config.hpp"
#include <boost/range/algorithm/count.hpp>

Config::Config(std::string jsonfile)
{
    std::string conf;
    Utils::loadTDDConfig(jsonfile, conf);
    const auto tddConf = json::parse(conf);

    /* antenna configurations */
    hub_file = tddConf.value("hubs", "");
    serial_file = tddConf.value("irises", "");
    ref_ant = tddConf.value("ref_ant", 0);
    nCells = tddConf.value("cells", 1);
    channel = tddConf.value("channel", "A");
    nChannels = std::min(channel.size(), (size_t)2);
    BS_ANT_NUM = tddConf.value("antenna_num", 8);
    isUE = tddConf.value("UE", false);
    UE_NUM = tddConf.value("ue_num", 8);
    UE_ANT_NUM = UE_NUM;
    if (hub_file.size() > 0)
        Utils::loadDevices(hub_file, hub_ids);
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
        nRadios = tddConf.value("radio_num", BS_ANT_NUM);

#ifdef USE_ARGOS
    rt_assert(nRadios != 0, "Error: No radios exist in Argos mode");
#endif

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

    rx_addr = tddConf.value("rx_addr", "127.0.0.1");
    tx_addr = tddConf.value("tx_addr", "127.0.0.1");
    bs_port = tddConf.value("bs_port", 8000);
    ue_rx_port = tddConf.value("ue_rx_port", 7000);
    ue_tx_port = tddConf.value("ue_tx_port", 6000);

    /* frame configurations */
    auto symbolSize = tddConf.value("symbol_size", 1);
    prefix = tddConf.value("prefix", 0);
    dl_prefix = tddConf.value("dl_prefix", 0);
    postfix = tddConf.value("postfix", 0);
    TX_PREFIX_LEN = tddConf.value("tx_prefix_len", 0);
    CP_LEN = tddConf.value("cp_len", 0);
    OFDM_PREFIX_LEN = tddConf.value("ofdm_prefix_len", 0) + CP_LEN;
    OFDM_CA_NUM = tddConf.value("ofdm_ca_num", 2048);
    OFDM_DATA_NUM = tddConf.value("ofdm_data_num", 1200);
    OFDM_PILOT_SPACING = tddConf.value("ofdm_pilot_spacing", 16);
    OFDM_PILOT_NUM = OFDM_DATA_NUM / OFDM_PILOT_SPACING;
    rt_assert(OFDM_DATA_NUM % kSCsPerCacheline == 0,
        "OFDM_DATA_NUM must be a multiple of subcarriers per cacheline");
    rt_assert(OFDM_DATA_NUM % kTransposeBlockSize == 0,
        "Transpose block size must divide number of OFDM data subcarriers");
    OFDM_DATA_START
        = tddConf.value("ofdm_data_start", (OFDM_CA_NUM - OFDM_DATA_NUM) / 2);
    OFDM_DATA_STOP = OFDM_DATA_START + OFDM_DATA_NUM;
    downlink_mode = tddConf.value("downlink_mode", false);
    bigstation_mode = tddConf.value("bigstation_mode", false);
    freq_orthogonal_pilot = tddConf.value("freq_orthogonal_pilot", false);
    correct_phase_shift = tddConf.value("correct_phase_shift", false);
    DL_PILOT_SYMS = tddConf.value("client_dl_pilot_syms", 0);
    UL_PILOT_SYMS = tddConf.value("client_ul_pilot_syms", 0);
    if (tddConf.find("frames") == tddConf.end()) {
        symbol_num_perframe = tddConf.value("symbol_num_perframe", 70);
        size_t pilot_num_default = freq_orthogonal_pilot ? 1 : UE_ANT_NUM;
        pilot_symbol_num_perframe
            = tddConf.value("pilot_num", pilot_num_default);
        data_symbol_num_perframe = tddConf.value("data_symbol_num_perframe",
            symbol_num_perframe - pilot_symbol_num_perframe);
        ul_data_symbol_num_perframe = tddConf.value("ul_symbol_num_perframe",
            downlink_mode ? 0
                          : symbol_num_perframe - pilot_symbol_num_perframe);
        dl_data_symbol_num_perframe
            = tddConf.value("dl_symbol_num_perframe", downlink_mode ? 10 : 0);
        dl_data_symbol_start = tddConf.value("dl_data_symbol_start", 10);
        dl_data_symbol_end = dl_data_symbol_start + dl_data_symbol_num_perframe;
        std::string sched("");
        for (size_t s = 0; s < pilot_symbol_num_perframe; s++)
            sched += "P";
        // Below it is assumed either dl or ul to be active at one time
        if (downlink_mode) {
            size_t dl_symbol_start
                = pilot_symbol_num_perframe + dl_data_symbol_start;
            size_t dl_symbol_end
                = dl_symbol_start + dl_data_symbol_num_perframe;
            for (size_t s = pilot_symbol_num_perframe; s < dl_symbol_start; s++)
                sched += "G";
            for (size_t s = dl_symbol_start; s < dl_symbol_end; s++)
                sched += "D";
            for (size_t s = dl_symbol_end; s < symbol_num_perframe; s++)
                sched += "G";
        } else {
            size_t ul_data_symbol_end
                = pilot_symbol_num_perframe + ul_data_symbol_num_perframe;
            for (size_t s = pilot_symbol_num_perframe; s < ul_data_symbol_end;
                 s++)
                sched += "U";
            for (size_t s = ul_data_symbol_end; s < symbol_num_perframe; s++)
                sched += "G";
        }
        frames.push_back(sched);
        printf("Frame schedule %s\n", sched.c_str());
    } else {
        json jframes = tddConf.value("frames", json::array());
        for (size_t f = 0; f < jframes.size(); f++) {
            frames.push_back(jframes.at(f).get<std::string>());
        }
    }

    pilotSymbols = Utils::loadSymbols(frames, 'P');
    ULSymbols = Utils::loadSymbols(frames, 'U');
    DLSymbols = Utils::loadSymbols(frames, 'D');
    ULCalSymbols = Utils::loadSymbols(frames, 'L');
    DLCalSymbols = Utils::loadSymbols(frames, 'C');
    recipCalEn = (ULCalSymbols[0].size() == 1 and DLCalSymbols[0].size() == 1);

    symbol_num_perframe = frames.at(0).size();
    pilot_symbol_num_perframe = pilotSymbols[0].size();
    data_symbol_num_perframe = symbol_num_perframe - pilot_symbol_num_perframe;
    ul_data_symbol_num_perframe = ULSymbols[0].size();
    dl_data_symbol_num_perframe = DLSymbols[0].size();
    downlink_mode = dl_data_symbol_num_perframe > 0;
    dl_data_symbol_start = dl_data_symbol_num_perframe > 0
        ? DLSymbols[0][0] - pilot_symbol_num_perframe
        : 0;
    dl_data_symbol_end = dl_data_symbol_num_perframe > 0
        ? DLSymbols[0].back() - pilot_symbol_num_perframe + 1
        : 0;

    if (isUE and !freq_orthogonal_pilot
        and UE_ANT_NUM != pilot_symbol_num_perframe) {
        rt_assert(false, "Number of pilot symbols doesn't match number of UEs");
    }
    if (!isUE and !freq_orthogonal_pilot
        and tddConf.find("ue_num") == tddConf.end()) {
        UE_NUM = pilot_symbol_num_perframe;
        UE_ANT_NUM = UE_NUM;
    }
    rt_assert(
        BS_ANT_NUM % 4 == 0, "Number of BS Antennas must be multiple of 4");

    /* Millipede configurations */
    frames_to_test = tddConf.value("frames_to_test", 9600);
    core_offset = tddConf.value("core_offset", 18);
    worker_thread_num = tddConf.value("worker_thread_num", 25);
    socket_thread_num = tddConf.value("socket_thread_num", 4);
    fft_thread_num = tddConf.value("fft_thread_num", 4);
    demul_thread_num = tddConf.value("demul_thread_num", 11);
    zf_thread_num = worker_thread_num - fft_thread_num - demul_thread_num;

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

    fft_block_size = tddConf.value("fft_block_size", 4);

    /* Modulation configurations */
    mod_type = modulation == "64QAM"
        ? CommsLib::QAM64
        : (modulation == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK);
    printf("Modulation: %s\n", modulation.c_str());
    mod_order = (size_t)pow(2, mod_type);

    /* LDPC Coding configurations */
    LDPC_config.Bg = tddConf.value("base_graph", 1);
    LDPC_config.earlyTermination = tddConf.value("earlyTermination", 1);
    LDPC_config.decoderIter = tddConf.value("decoderIter", 5);
    LDPC_config.Zc = tddConf.value("Zc", 72);
    LDPC_config.nRows = (LDPC_config.Bg == 1) ? 46 : 42;
    LDPC_config.cbEncLen = LDPC_config.nRows * LDPC_config.Zc;
    LDPC_config.cbLen
        = (LDPC_config.Bg == 1) ? LDPC_config.Zc * 22 : LDPC_config.Zc * 10;
    LDPC_config.cbCodewLen
        = (LDPC_config.Bg == 1) ? LDPC_config.Zc * 66 : LDPC_config.Zc * 50;
    LDPC_config.nblocksInSymbol
        = OFDM_DATA_NUM * mod_type / LDPC_config.cbCodewLen;

    printf("Encoder: Zc: %d, code block per symbol: %d, code block len: %d, "
           "encoded block len: %d, decoder iterations: %d\n",
        LDPC_config.Zc, LDPC_config.nblocksInSymbol, LDPC_config.cbLen,
        LDPC_config.cbCodewLen, LDPC_config.decoderIter);

    OFDM_SYM_LEN = OFDM_CA_NUM + CP_LEN;
    OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;
    sampsPerSymbol = symbolSize * OFDM_SYM_LEN + prefix + postfix;
    packet_length = offsetof(Packet, data) + sizeof(short) * sampsPerSymbol * 2;

    running = true;
    std::cout << "Config: "
              << "BS_ANT_NUM " << BS_ANT_NUM << ", UE_ANT_NUM " << UE_ANT_NUM
              << ", PILOT SYM NUM " << pilot_symbol_num_perframe
              << ", UL SYM NUM " << ul_data_symbol_num_perframe
              << ", DL SYM NUM " << dl_data_symbol_num_perframe
              << ", OFDM_CA_NUM " << OFDM_CA_NUM << ", OFDM_DATA_NUM "
              << OFDM_DATA_NUM << ", packet length " << packet_length
              << std::endl;

    if (packet_length >= 9000) {
        std::cout << "\033[1;31mWarning: packet length is larger than jumbo "
                     "frame size (9000)! "
                  << "Packets will be fragmented.\033[0m" << std::endl;
    }
    std::cout << "Config done!" << std::endl;
}

void Config::genData()
{
#ifdef USE_ARGOS
    std::vector<std::vector<double>> gold_ifft
        = CommsLib::getSequence(128, CommsLib::GOLD_IFFT);
    std::vector<std::complex<int16_t>> gold_ifft_ci16
        = Utils::double_to_cint16(gold_ifft);

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
        beacon_ci16.insert(
            beacon_ci16.end(), gold_ifft_ci16.begin(), gold_ifft_ci16.end());
    }

    beacon_len = beacon_ci16.size();

    if (sampsPerSymbol < beacon_len + prefix + postfix) {
        std::string msg = "Minimum supported symbol_size is ";
        msg += std::to_string(beacon_len);
        throw std::invalid_argument(msg);
    }

    beacon = Utils::cint16_to_uint32(beacon_ci16, false, "QI");
    coeffs = Utils::cint16_to_uint32(gold_ifft_ci16, true, "QI");
#endif

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
    pilotsF.resize(OFDM_DATA_START);
    pilotsF.insert(pilotsF.end(), common_pilot.begin(), common_pilot.end());
    pilotsF.resize(OFDM_CA_NUM);
    pilot_cf32 = CommsLib::IFFT(pilotsF, OFDM_CA_NUM, false);

    // Generate UE-specific pilots based on Zadoff-Chu sequence for phase tracking
    ue_specific_pilot.malloc(UE_ANT_NUM, OFDM_DATA_NUM, 64);
    ue_specific_pilot_t.calloc(UE_ANT_NUM, sampsPerSymbol, 64);
    Table<complex_float> pilot_ifft;
    pilot_ifft.calloc(UE_ANT_NUM, OFDM_CA_NUM, 64);
    auto zc_ue_pilot_double
        = CommsLib::getSequence(OFDM_DATA_NUM, CommsLib::LTE_ZADOFF_CHU);
    auto zc_ue_pilot = Utils::double_to_cfloat(zc_ue_pilot_double);
    for (size_t i = 0; i < UE_ANT_NUM; i++) {
        auto zc_ue_pilot_i = CommsLib::seqCyclicShift(
            zc_ue_pilot, i * (float)M_PI / 6); // LTE DMRS
        for (size_t j = 0; j < OFDM_DATA_NUM; j++) {
            ue_specific_pilot[i][j]
                = { zc_ue_pilot_i[j].real(), zc_ue_pilot_i[j].imag() };
            pilot_ifft[i][j + OFDM_DATA_START] = ue_specific_pilot[i][j];
        }
        CommsLib::IFFT(pilot_ifft[i], OFDM_CA_NUM, false);
    }

    dl_bits.malloc(dl_data_symbol_num_perframe, OFDM_DATA_NUM * UE_ANT_NUM, 64);
    dl_iq_f.calloc(dl_data_symbol_num_perframe, OFDM_CA_NUM * UE_ANT_NUM, 64);
    dl_iq_t.calloc(
        dl_data_symbol_num_perframe, sampsPerSymbol * UE_ANT_NUM, 64);
    ul_bits.malloc(ul_data_symbol_num_perframe, OFDM_DATA_NUM * UE_ANT_NUM, 64);
    ul_iq_f.calloc(ul_data_symbol_num_perframe, OFDM_CA_NUM * UE_ANT_NUM, 64);
    ul_iq_t.calloc(
        ul_data_symbol_num_perframe, sampsPerSymbol * UE_ANT_NUM, 64);

#ifdef GENERATE_DATA
    for (size_t ue_id = 0; ue_id < UE_ANT_NUM; ue_id++) {
        for (size_t j = 0; j < OFDM_DATA_NUM; j++) {
            int cur_offset = j * UE_ANT_NUM + ue_id;
            for (size_t i = 0; i < ul_data_symbol_num_perframe; i++)
                ul_bits[i][cur_offset] = rand() % mod_order;
            for (size_t i = 0; i < dl_data_symbol_num_perframe; i++)
                dl_bits[i][cur_offset] = rand() % mod_order;
        }
    }
#else
    std::string cur_directory1 = TOSTRING(PROJECT_DIRECTORY);
#ifdef USE_LDPC
    std::string filename1 = cur_directory1 + "/data/LDPC_orig_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(UE_ANT_NUM)
        + ".bin";
    size_t num_bytes_per_ue = (LDPC_config.cbLen + 7) >> 3;
#else
    std::string filename1 = cur_directory1 + "/data/orig_data_"
        + std::to_string(OFDM_CA_NUM) + "_ant" + std::to_string(UE_ANT_NUM)
        + ".bin";
    size_t num_bytes_per_ue = OFDM_DATA_NUM;
#endif
    FILE* fd = fopen(filename1.c_str(), "rb");
    if (fd == NULL) {
        printf("open antenna file %s failed.\n", filename1.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for (size_t i = 0; i < ul_data_symbol_num_perframe; i++) {
        size_t r = fread(
            ul_bits[i], sizeof(int8_t), num_bytes_per_ue * UE_ANT_NUM, fd);
        if (r < num_bytes_per_ue * UE_ANT_NUM)
            printf(
                "bad read from file %s (batch %zu) \n", filename1.c_str(), i);
    }
    for (size_t i = 0; i < dl_data_symbol_num_perframe; i++) {
        size_t r = fread(
            dl_bits[i], sizeof(int8_t), num_bytes_per_ue * UE_ANT_NUM, fd);
        if (r < num_bytes_per_ue * UE_ANT_NUM)
            printf(
                "bad read from file %s (batch %zu) \n", filename1.c_str(), i);
    }
    fclose(fd);
#endif

    Table<float> qam_table;
    init_modulation_table(qam_table, mod_type);

    // Generate freq-domain downlink symbols
    Table<complex_float> dl_iq_ifft;
    dl_iq_ifft.calloc(
        dl_data_symbol_num_perframe, OFDM_CA_NUM * UE_ANT_NUM, 64);
    for (size_t i = 0; i < dl_data_symbol_num_perframe; i++) {
        for (size_t u = 0; u < UE_ANT_NUM; u++) {
            size_t p = u * OFDM_DATA_NUM;
            size_t q = u * OFDM_CA_NUM;

            for (size_t j = OFDM_DATA_START; j < OFDM_DATA_STOP; j++) {
                int k = (j - OFDM_DATA_START);
                size_t s = p + k;
                if (k % OFDM_PILOT_SPACING != 0) {
                    dl_iq_f[i][q + j]
                        = mod_single_uint8(dl_bits[i][s], qam_table);
                } else
                    dl_iq_f[i][q + j] = ue_specific_pilot[0][k];
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
                ul_iq_f[i][q + j] = mod_single_uint8(ul_bits[i][s], qam_table);
                ul_iq_ifft[i][q + j] = ul_iq_f[i][q + j];
            }

            CommsLib::IFFT(&ul_iq_ifft[i][q], OFDM_CA_NUM, false);
        }
    }

    // Find normalization factor through searching for max value in IFFT results
    float max_val = 0;
    for (size_t i = 0; i < ul_data_symbol_num_perframe; i++) {
        for (size_t u = 0; u < UE_ANT_NUM; u++) {
            size_t q = u * OFDM_CA_NUM;
            for (size_t j = 0; j < OFDM_CA_NUM; j++) {
                auto cur_val = std::abs(std::complex<float>(
                    ul_iq_ifft[i][q + j].re, ul_iq_ifft[i][q + j].im));
                if (cur_val > max_val) {
                    max_val = cur_val;
                }
            }
        }
    }

    for (size_t i = 0; i < dl_data_symbol_num_perframe; i++) {
        for (size_t u = 0; u < UE_ANT_NUM; u++) {
            size_t q = u * OFDM_CA_NUM;
            for (size_t j = 0; j < OFDM_CA_NUM; j++) {
                auto cur_val = std::abs(std::complex<float>(
                    dl_iq_ifft[i][q + j].re, dl_iq_ifft[i][q + j].im));
                if (cur_val > max_val) {
                    max_val = cur_val;
                }
            }
        }
    }

    for (size_t u = 0; u < UE_ANT_NUM; u++) {
        for (size_t j = 0; j < OFDM_CA_NUM; j++) {
            auto cur_val = std::abs(std::complex<float>(
                pilot_ifft[u][j].re, pilot_ifft[u][j].im));
            if (cur_val > max_val) {
                max_val = cur_val;
            }
        }
    }

    for (size_t j = 0; j < OFDM_CA_NUM; j++) {
        auto cur_val = std::abs(pilot_cf32[j]);
        if (cur_val > max_val) {
            max_val = cur_val;
        }
    }

    float scale = 2 * max_val; // additional 2^2 (6dB) power backoff

    // Generate time domain symbols for downlink
    for (size_t i = 0; i < dl_data_symbol_num_perframe; i++) {
        for (size_t u = 0; u < UE_ANT_NUM; u++) {

            size_t q = u * OFDM_CA_NUM;
            size_t r = u * sampsPerSymbol;

            size_t t = prefix + CP_LEN;
            for (size_t j = t; j < t + OFDM_CA_NUM; j++) {
                complex_float sc_data = { dl_iq_ifft[i][q + j - t].re / scale,
                    dl_iq_ifft[i][q + j - t].im / scale };
                dl_iq_t[i][r + j] = { (int16_t)(sc_data.re * 32768),
                    (int16_t)(sc_data.im * 32768) };
            }
            memcpy(dl_iq_t[i] + r + prefix,
                dl_iq_t[i] + r + prefix + OFDM_CA_NUM,
                CP_LEN * sizeof(short) * 2);
        }
    }

    // Generate freq-domain uplink symbols
    for (size_t i = 0; i < ul_data_symbol_num_perframe; i++) {
        for (size_t u = 0; u < UE_ANT_NUM; u++) {

            size_t q = u * OFDM_CA_NUM;
            size_t r = u * sampsPerSymbol;

            size_t t = prefix + CP_LEN;
            for (size_t j = t; j < t + OFDM_CA_NUM; j++) {
                complex_float sc_data = { ul_iq_ifft[i][q + j - t].re / scale,
                    ul_iq_ifft[i][q + j - t].im / scale };
                ul_iq_t[i][r + j] = { (int16_t)(sc_data.re * 32768),
                    (int16_t)(sc_data.im * 32768) };
            }
            memcpy(ul_iq_t[i] + r + prefix,
                ul_iq_t[i] + r + prefix + OFDM_CA_NUM,
                CP_LEN * sizeof(short) * 2);
        }
    }
    ul_iq_ifft.free();
    dl_iq_ifft.free();

    for (size_t i = 0; i < UE_ANT_NUM; i++) {
        for (size_t j = 0; j < OFDM_CA_NUM; j++) {
            ue_specific_pilot_t[i][prefix + CP_LEN + j] = std::complex<int16_t>(
                (int16_t)((pilot_ifft[i][j].re / scale) * 32768),
                (int16_t)((pilot_ifft[i][j].im / scale) * 32768));
        }
        for (size_t j = 0; j < CP_LEN; j++) {
            ue_specific_pilot_t[i][prefix + j]
                = ue_specific_pilot_t[i][prefix + OFDM_CA_NUM + j];
        }
    }

    pilot_ifft.free();

    for (size_t i = 0; i < OFDM_CA_NUM; i++)
        pilot_cf32[i] /= scale;
    pilot_cf32.insert(pilot_cf32.begin(), pilot_cf32.end() - CP_LEN,
        pilot_cf32.end()); // add CP

    // generate a UINT32 version to write to FPGA buffers
    pilot = Utils::cfloat32_to_uint32(pilot_cf32, false, "QI");

    std::vector<uint32_t> pre_uint32(prefix, 0);
    pilot.insert(pilot.begin(), pre_uint32.begin(), pre_uint32.end());
    pilot.resize(sampsPerSymbol);
}

Config::~Config()
{
    free_buffer_1d(&pilots_);
    free_buffer_1d(&pilots_sgn_);
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
        return -1;
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
        return -1;
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
        return -1;
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
    assert(!isUE); // Currently implemented for only the Millipede server
    char s = frames[frame_id % frames.size()][symbol_id];
    switch (s) {
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
    rt_assert(false, "Should not reach here");
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
