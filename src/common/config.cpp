
#include "config.hpp"
#include "comms-lib.h"
#include <boost/range/algorithm/count.hpp>

Config::Config(std::string jsonfile)
{
    std::string conf;
    Utils::loadTDDConfig(jsonfile, conf);
    const auto tddConf = json::parse(conf);
    hub_file = tddConf.value("hubs", "hub_serials.txt");
    Utils::loadDevices(hub_file, hub_ids);
    serial_file = tddConf.value("irises", "iris_serials.txt");
    ref_ant = tddConf.value("ref_ant", 0);
    nCells = tddConf.value("cells", 1);
    nChannels = tddConf.value("channels", 1);
    isUE = tddConf.value("UE", false);
    freq = tddConf.value("frequency", 3.6e9);
    bbf_ratio = 0.75;
    txgainA = tddConf.value("txgainA", 20);
    rxgainA = tddConf.value("rxgainA", 20);
    txgainB = tddConf.value("txgainB", 20);
    rxgainB = tddConf.value("rxgainB", 20);
    calTxGainA = tddConf.value("calTxGainA", 10);
    calTxGainB = tddConf.value("calTxGainB", 10);
    rate = tddConf.value("rate", 5e6);
    sampsPerSymbol = tddConf.value("symbol_size", 0);
    prefix = tddConf.value("prefix", 0);
    dl_prefix = tddConf.value("dl_prefix", 0);
    postfix = tddConf.value("postfix", 0);
    beacon_ant = tddConf.value("beacon_antenna", 0);
    beacon_len = tddConf.value("beacon_len", 256);
    beacon_mode = tddConf.value("beacon_mode", "single");
    sampleCalEn = tddConf.value("sample_calibrate", false);
    modulation = tddConf.value("modulation", "QPSK");

    /* Millipede configurations */
    core_offset = tddConf.value("core_offset", 18);
    worker_thread_num = tddConf.value("worker_thread_num", 25);
    socket_thread_num = tddConf.value("socket_thread_num", 4);
    fft_thread_num = tddConf.value("fft_thread_num", 4);
    demul_thread_num = tddConf.value("demul_thread_num", 11);
    zf_thread_num = worker_thread_num - fft_thread_num - demul_thread_num;

    demul_block_size = tddConf.value("demul_block_size", 48);
    zf_block_size = tddConf.value("zf_block_size", 1);

    rx_addr = tddConf.value("rx_addr", "127.0.0.1");
    tx_addr = tddConf.value("tx_addr", "127.0.0.1");
    tx_port = tddConf.value("tx_port", 7991);
    rx_port = tddConf.value("rx_port", 7891);

    json jframes = tddConf.value("frames", json::array());
    framePeriod = jframes.size();
    for (size_t f = 0; f < framePeriod; f++) {
        frames.push_back(jframes.at(0).get<std::string>());
    }
    Utils::loadDevices(hub_file, hub_ids);
    Utils::loadDevices(serial_file, radio_ids);
    nRadios = radio_ids.size();
    nAntennas = nChannels * nRadios;
    if (ref_ant >= nAntennas)
	ref_ant = 0;
    if (beacon_mode == "beamsweep" && nAntennas > 1) {
        int hadamardSize = int(pow(2, ceil(log2(nAntennas))));
        std::vector<std::vector<double> > hadamard_weights = CommsLib::getSequence(hadamardSize, CommsLib::HADAMARD);
        beacon_weights.resize(nAntennas);
        for (size_t i = 0; i < nAntennas; i++)
            for (size_t j = 0; j < nAntennas; j++)
                beacon_weights[i].push_back((unsigned)hadamard_weights[i][j]);
        printf("Hadamard Matrix Size %d\n", hadamardSize);
        printf("beacon_weights size %zu\n", beacon_weights.size());
        printf("beacon_weights[0] size %zu\n", beacon_weights[0].size());
    }
    symbolsPerFrame = frames.at(0).size();
    nUEs = std::count(frames.at(0).begin(), frames.at(0).end(), 'P');
    pilotSymbols = Utils::loadSymbols(frames, 'P');
    ULSymbols = Utils::loadSymbols(frames, 'U');
    DLSymbols = Utils::loadSymbols(frames, 'D');
    pilotSymsPerFrame = pilotSymbols[0].size();
    ulSymsPerFrame = ULSymbols[0].size();
    dlSymsPerFrame = DLSymbols[0].size();
    if (isUE and nRadios != pilotSymsPerFrame) {
        std::cerr << "Number of Pilot Symbols don't match number of Clients!" << std::endl;
        exit(0);
    }

    OFDM_CA_NUM = tddConf.value("ofdm_ca_num", 2048);
    OFDM_DATA_NUM = tddConf.value("ofdm_data_num", 1200);
    OFDM_DATA_START = tddConf.value("ofdm_data_start", (OFDM_CA_NUM - OFDM_DATA_NUM) / 2);
#ifdef USE_ARGOS
    /* base station configurations */
    BS_ANT_NUM = nAntennas; //tddConf.value("bs_ant_num", 8);
    UE_NUM = nUEs; //tddConf.value("ue_num", 2);

    TX_PREFIX_LEN = tddConf.value("tx_prefix_len", 128);
    CP_LEN = tddConf.value("cp_len", 128);
    OFDM_PREFIX_LEN = tddConf.value("ofdm_prefix_len", 152 + CP_LEN);
    OFDM_FRAME_LEN = OFDM_CA_NUM + 2 * TX_PREFIX_LEN;

    /* frame configurations */
    symbol_num_perframe = symbolsPerFrame; //tddConf.value("subframe_num_perframe", 5);
    pilot_symbol_num_perframe = pilotSymsPerFrame; // tddConf.value("pilot_num", UE_NUM);
    ul_data_symbol_num_perframe = ulSymsPerFrame; //tddConf.value("ul_subframe_num_perframe", symbol_num_perframe - pilot_symbol_num_perframe);
    dl_data_symbol_num_perframe = dlSymsPerFrame; //tddConf.value("dl_subframe_num_perframe", 3);
    dl_data_symbol_start = dlSymsPerFrame > 0 ? DLSymbols[0][0] - pilot_symbol_num_perframe : 0; //pilotSymsPerFrame; //dlSymsPerFrame > 0 ? DLSymbols[0][0] : 0;//tddConf.value("dl_data_subframe_start", 0);
    dl_data_symbol_end = dl_data_symbol_start + dl_data_symbol_num_perframe;
    data_symbol_num_perframe = symbol_num_perframe - pilotSymsPerFrame; // - pilotSymbols[0][0]; //std::max(ulSymsPerFrame, dlSymsPerFrame); //symbol_num_perframe - pilot_symbol_num_perframe; //tddConf.value("data_subframe_num_perframe", symbol_num_perframe -  pilot_symbol_num_perframe);
    //symbol_num_perframe = data_symbol_num_perframe+pilot_symbol_num_perframe;//symbolsPerFrame; //tddConf.value("subframe_num_perframe", 5);

    packet_header_offset = tddConf.value("packet_header_offset", 64);
    packet_length = packet_header_offset + sizeof(short) * sampsPerSymbol * 2;
    downlink_mode = dl_data_symbol_num_perframe > 0;
#else
    /* base station configurations */
    BS_ANT_NUM = tddConf.value("bs_ant_num", 8);
    UE_NUM = tddConf.value("ue_num", 8);

    TX_PREFIX_LEN = tddConf.value("tx_prefix_len", 0);
    CP_LEN = tddConf.value("cp_len", 0);
    OFDM_PREFIX_LEN = tddConf.value("ofdm_prefix_len", 0 + CP_LEN);
    OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;

    /* frame configurations */
    symbol_num_perframe = tddConf.value("subframe_num_perframe", 70);
    pilot_symbol_num_perframe = tddConf.value("pilot_num", UE_NUM);
    data_symbol_num_perframe = tddConf.value("data_subframe_num_perframe", symbol_num_perframe - pilot_symbol_num_perframe);
    ul_data_symbol_num_perframe = tddConf.value("ul_subframe_num_perframe", symbol_num_perframe - pilot_symbol_num_perframe);
    dl_data_symbol_num_perframe = tddConf.value("dl_subframe_num_perframe", 10);
    dl_data_symbol_start = tddConf.value("dl_data_subframe_start", 10);
    dl_data_symbol_end = dl_data_symbol_start + dl_data_symbol_num_perframe;
    packet_header_offset = tddConf.value("packet_header_offset", 64);
    packet_length = packet_header_offset + sizeof(short) * OFDM_FRAME_LEN * 2;
    downlink_mode = tddConf.value("downlink_mode", false);
#endif
    std::cout << "Config file loaded!" << std::endl;
    std::cout << "BS_ANT_NUM " << BS_ANT_NUM << std::endl;
    std::cout << "UE_NUM " << nUEs << std::endl;
    std::cout << "pilot sym num " << pilotSymsPerFrame << std::endl;
    std::cout << "UL sym num " << ulSymsPerFrame << std::endl;
    std::cout << "DL sym num " << dlSymsPerFrame << std::endl;

#ifdef USE_ARGOS
    std::vector<std::vector<double> > gold_ifft = CommsLib::getSequence(128, CommsLib::GOLD_IFFT);
    std::vector<std::complex<int16_t> > coeffs_ci16 = Utils::double_to_int16(gold_ifft);
    coeffs = Utils::cint16_to_uint32(coeffs_ci16, true, "QI");
    beacon_ci16.resize(256);
    for (size_t i = 0; i < 128; i++) {
        beacon_ci16[i] = std::complex<int16_t>((int16_t)(gold_ifft[0][i] * 32768), (int16_t)(gold_ifft[1][i] * 32768));
        beacon_ci16[i + 128] = beacon_ci16[i];
    }

    std::vector<std::complex<int16_t> > pre0(prefix, 0);
    std::vector<std::complex<int16_t> > post0(sampsPerSymbol - 256 - prefix, 0);
    beacon_ci16.insert(beacon_ci16.begin(), pre0.begin(), pre0.end());
    beacon_ci16.insert(beacon_ci16.end(), post0.begin(), post0.end());
    beacon = Utils::cint16_to_uint32(beacon_ci16, false, "QI");
#endif

    pilots_ = (float*)aligned_alloc(64, OFDM_CA_NUM * sizeof(float));
    size_t r = 0;
#ifdef GENERATE_PILOT
    for (size_t i = 0; i < OFDM_CA_NUM; i++) {
        if (i < OFDM_DATA_START || i >= OFDM_DATA_START + OFDM_DATA_NUM)
            pilots_[i] = 0;
        else
            pilots_[i] = 1 - 2 * (rand() % 2);
    }

#else
    // read pilots from file
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/pilot_f_2048.bin";
    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        printf("open file %s faild.\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    r = fread(pilots_, sizeof(float), OFDM_CA_NUM, fp);
    if (r < OFDM_CA_NUM) printf("bad read from file %s \n", filename.c_str());
    fclose(fp);
#endif
    std::vector<std::complex<float> > pilotsF(OFDM_CA_NUM);
    for (size_t i = 0; i < OFDM_CA_NUM; i++)
        pilotsF[i] = pilots_[i];
    pilot_cf32 = CommsLib::IFFT(pilotsF, OFDM_CA_NUM);
    pilot_cf32.insert(pilot_cf32.begin(), pilot_cf32.end() - CP_LEN, pilot_cf32.end()); // add CP

    std::vector<std::complex<int16_t>> pre_ci16(prefix, 0);
    std::vector<std::complex<int16_t>> post_ci16(postfix, 0);
    for (size_t i = 0; i < OFDM_CA_NUM + CP_LEN; i++)
        pilot_ci16.push_back(std::complex<int16_t>((int16_t)(pilot_cf32[i].real()*32768), (int16_t)(pilot_cf32[i].imag()*32768)));
    pilot_ci16.insert(pilot_ci16.begin(), pre_ci16.begin(), pre_ci16.end());
    pilot_ci16.insert(pilot_ci16.end(), post_ci16.begin(), post_ci16.end());

#ifdef USE_ARGOS
    pilot = Utils::cfloat32_to_uint32(pilot_cf32, false, "QI");
    std::vector<uint32_t> pre(prefix, 0);
    std::vector<uint32_t> post(postfix, 0);
    pilot.insert(pilot.begin(), pre.begin(), pre.end());
    pilot.insert(pilot.end(), post.begin(), post.end());
    if (pilot.size() != sampsPerSymbol) {
        std::cout << "generated pilot symbol size does not match configured symbol size!" << std::endl;
        exit(1);
    }

#endif

    alloc_buffer_2d(&dl_IQ_data, data_symbol_num_perframe * UE_NUM, OFDM_CA_NUM, 64, 0);
    alloc_buffer_2d(&dl_IQ_modul, data_symbol_num_perframe * UE_NUM, OFDM_CA_NUM, 64, 0); // used for debug
    alloc_buffer_2d(&dl_IQ_symbol, data_symbol_num_perframe, sampsPerSymbol, 64, 0); // used for debug
    alloc_buffer_2d(&ul_IQ_data, ul_data_symbol_num_perframe * UE_NUM, OFDM_DATA_NUM, 64, 0);
    alloc_buffer_2d(&ul_IQ_modul, ul_data_symbol_num_perframe * UE_NUM, OFDM_CA_NUM, 64, 0);

    size_t mod_type = modulation == "64QAM" ? CommsLib::QAM64 : (modulation == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK);
    mod_order = (size_t)pow(2, mod_type);

#ifdef GENERATE_DATA
    for (size_t i = 0; i < data_symbol_num_perframe * UE_NUM; i++) {
        for (size_t j = 0; j < OFDM_CA_NUM; j++)
            dl_IQ_data[i][j] = rand() % mod_order;
        std::vector<std::complex<float> > modul_data = CommsLib::modulate(std::vector<int>(dl_IQ_data[i], dl_IQ_data[i] + OFDM_CA_NUM), mod_type);
        for (size_t j = 0; j < OFDM_CA_NUM; j++) {
            if (j < OFDM_DATA_START || j >= OFDM_DATA_START + OFDM_DATA_NUM) {
                dl_IQ_modul[i][j].real = 0;
                dl_IQ_modul[i][j].imag = 0;
	    } else {
                dl_IQ_modul[i][j].real = modul_data[j].real();
                dl_IQ_modul[i][j].imag = modul_data[j].imag();
	    }
        }

	if (i % UE_NUM == 0) {
	    int c = i / UE_NUM;
            std::vector<std::complex<float> > ifft_dl_data = CommsLib::IFFT(modul_data, OFDM_CA_NUM);
	    ifft_dl_data.insert(ifft_dl_data.begin(), ifft_dl_data.end() - CP_LEN, ifft_dl_data.end());
            for (size_t j = 0; j < sampsPerSymbol; j++) {
                if (j < prefix || j >= prefix + CP_LEN + OFDM_CA_NUM) {
                    dl_IQ_symbol[c][j] = 0;
	        } else {
                    dl_IQ_symbol[c][j] = {(int16_t)(ifft_dl_data[j-prefix].real()*32768), (int16_t)(ifft_dl_data[j-prefix].imag()*32768)};
	        }
            }
	}
    }

    for (size_t i = 0; i < ul_data_symbol_num_perframe * UE_NUM; i++) {
        for (size_t j = 0; j < OFDM_DATA_NUM; j++)
            ul_IQ_data[i][j] = rand() % mod_order;
        std::vector<std::complex<float> > modul_data = CommsLib::modulate(std::vector<int>(ul_IQ_data[i], ul_IQ_data[i] + OFDM_DATA_NUM), mod_type);
        for (size_t j = 0; j < OFDM_CA_NUM; j++) {
            if (j < OFDM_DATA_START || j >= OFDM_DATA_START + OFDM_DATA_NUM)
                continue;
            size_t k = j - OFDM_DATA_START;
            ul_IQ_modul[i][j].real = modul_data[k].real();
            ul_IQ_modul[i][j].imag = modul_data[k].imag();
        }
    }
#else
    std::string cur_directory1 = TOSTRING(PROJECT_DIRECTORY);
    std::string filename1 = cur_directory1 + "/data/orig_data_2048_ant" + std::to_string(BS_ANT_NUM) + ".bin";
    FILE* fd = fopen(filename1.c_str(), "rb");
    if (fd == NULL) {
        printf("open file %s faild.\n", filename1.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    for (size_t i = 0; i < data_symbol_num_perframe * UE_NUM; i++) {
        r = fread(dl_IQ_data[i], sizeof(int), OFDM_CA_NUM, fd);
        if (r < OFDM_CA_NUM) printf("bad read from file %s (batch %d) \n", filename1.c_str(), i);
    }
    fclose(fd);

    // read uplink
    std::string filename2 = cur_directory1 + "/data/tx_ul_data_" + std::to_string(BS_ANT_NUM) + "x" + std::to_string(nUEs) + ".bin";
    fp = fopen(filename2.c_str(), "rb");
    if (fp == NULL) {
        std::cerr << "Openning File " << filename2 << " fails. Error: " << strerror(errno) << std::endl;
    }
    size_t total_sc = OFDM_DATA_NUM * UE_NUM * ul_data_symbol_num_perframe; // coding is not considered yet
    L2_data = new mac_dtype[total_sc];
    r = fread(L2_data, sizeof(mac_dtype), total_sc, fp);
    if (r < total_sc) printf("bad read from file %s \n", filename2.c_str());
    fclose(fp);
    for (size_t i = 0; i < total_sc; i++) {
        size_t sid = i / (data_sc_len * nUEs);
        size_t cid = i % (data_sc_len * nUEs) + OFDM_DATA_START;
        ul_IQ_modul[sid][cid] = L2_data[i];
    }
#endif

    running = true;
}

Config::~Config()
{
    free_buffer_1d(&pilots_);
}

int Config::getDownlinkPilotId(size_t frame_id, size_t symbol_id)
{
    std::vector<size_t>::iterator it;
    size_t fid = frame_id % framePeriod;
    it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
    if (it != DLSymbols[fid].end()) {
        int id = it - DLSymbols[fid].begin();
        if (id < DL_PILOT_SYMS) {
#ifdef DEBUG3
            printf("getDownlinkPilotId(%d, %d) = %d\n", frame_id, symbol_id, id);
#endif
            return id;
        }
    }
    return -1;
}

int Config::getDlSFIndex(size_t frame_id, size_t symbol_id)
{
    std::vector<size_t>::iterator it;
    size_t fid = frame_id % framePeriod;
    it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
    if (it != DLSymbols[fid].end())
        return it - DLSymbols[fid].begin();
    else
        return -1;
}

int Config::getPilotSFIndex(size_t frame_id, size_t symbol_id)
{
    std::vector<size_t>::iterator it;
    size_t fid = frame_id % framePeriod;
    it = find(pilotSymbols[fid].begin(), pilotSymbols[fid].end(), symbol_id);
    if (it != pilotSymbols[fid].end()) {
#ifdef DEBUG3
        printf("getPilotSFIndex(%d, %d) = %d\n", frame_id, symbol_id, it - pilotSymbols[fid].begin());
#endif
        return it - pilotSymbols[fid].begin();
    } else
        return -1;
}

int Config::getUlSFIndex(size_t frame_id, size_t symbol_id)
{
    std::vector<size_t>::iterator it;
    size_t fid = frame_id % framePeriod;
    it = find(ULSymbols[fid].begin(), ULSymbols[fid].end(), symbol_id);
    if (it != ULSymbols[fid].end()) {
#ifdef DEBUG3
        printf("getUlSFIndexId(%d, %d) = %d\n", frame_id, symbol_id, it - ULSymbols[fid].begin());
#endif
        return it - ULSymbols[fid].begin();
    } else
        return -1;
}

bool Config::isPilot(size_t frame_id, size_t symbol_id)
{
    size_t fid = frame_id % framePeriod;
#ifdef USE_UNDEF //USE_ARGOS
    if (symbol_id > symbolsPerFrame) {
        printf("\x1B[31mERROR: Received out of range symbol %d at frame %d\x1B[0m\n", symbol_id, frame_id);
        return false;
    }
#ifdef DEBUG3
    printf("isPilot(%d, %d) = %c\n", frame_id, symbol_id, frames[fid].at(symbol_id));
#endif
    if (isUE) {
        std::vector<size_t>::iterator it;
        it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
        int ind = DL_PILOT_SYMS;
        if (it != DLSymbols[fid].end())
            ind = it - DLSymbols[fid].begin();
        return (ind < DL_PILOT_SYMS);
        //return cfg->frames[fid].at(symbol_id) == 'P' ? true : false;
    } else
        return frames[fid].at(symbol_id) == 'P';
#else
    if (isUE) {
        std::vector<size_t>::iterator it;
        it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
        int ind = DL_PILOT_SYMS;
        if (it != DLSymbols[fid].end())
            ind = it - DLSymbols[fid].begin();
        return (ind < DL_PILOT_SYMS);
        //return cfg->frames[fid].at(symbol_id) == 'P' ? true : false;
    } else
        return (symbol_id < UE_NUM);
#endif
}

bool Config::isUplink(size_t frame_id, size_t symbol_id)
{
    size_t fid = frame_id % framePeriod;
#ifdef USE_UNDEF //USE_ARGOS
    size_t fid = frame_id % framePeriod;
    if (symbol_id > symbolsPerFrame) {
        printf("\x1B[31mERROR: Received out of range symbol %d at frame %d\x1B[0m\n", symbol_id, frame_id);
        return false;
    }
#ifdef DEBUG3
    printf("isUplink(%d, %d) = %c\n", frame_id, symbol_id, frames[fid].at(symbol_id));
#endif
    return frames[fid].at(symbol_id) == 'U';
#else
    if (isUE)
        return frames[fid].at(symbol_id) == 'U';
    else
        return (symbol_id < symbol_num_perframe) && (symbol_id >= UE_NUM);
#endif
}

bool Config::isDownlink(size_t frame_id, size_t symbol_id)
{
    size_t fid = frame_id % framePeriod;
#ifdef DEBUG3
    printf("isDownlink(%d, %d) = %c\n", frame_id, symbol_id, frames[fid].at(symbol_id));
#endif
    if (isUE)
        return frames[fid].at(symbol_id) == 'D' && !this->isPilot(frame_id, symbol_id);
    else
        return frames[fid].at(symbol_id) == 'D';
}

extern "C" {
__attribute__((visibility("default"))) Config* Config_new(char* filename)
{

    Config* cfg = new Config(filename);

    return cfg;
}
}
