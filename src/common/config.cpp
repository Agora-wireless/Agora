
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
    core_offset = tddConf.value("core_offset", 17);
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

    package_header_offset = tddConf.value("package_header_offset", 64);
    package_length = package_header_offset + sizeof(short) * sampsPerSymbol * 2;
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
    package_header_offset = tddConf.value("package_header_offset", 64);
    package_length = package_header_offset + sizeof(short) * OFDM_FRAME_LEN * 2;
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
    for (int i = 0; i < 128; i++) {
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
#ifdef GENERATE_PILOT
    for (int i = 0; i < OFDM_CA_NUM; i++) {
        if (i < OFDM_DATA_START || i >= OFDM_DATA_START + OFDM_DATA_NUM)
            pilots_[i] = 0;
        else
            pilots_[i] = 1 - 2 * (rand() % 2);
    }

//// compose pilot subframe
//std::vector<std::vector<double>> lts = CommsLib::getSequence(80, CommsLib::LTS_SEQ);
//std::vector<std::complex<int16_t>> lts_ci16 = Utils::double_to_int16(lts);
//int nSamps = sampsPerSymbol - prefix - postfix;
//int rep = nSamps / 80;
//int frac = nSamps % 80;
//std::vector<std::complex<int16_t>> pre(prefix, 0);
//std::vector<std::complex<int16_t>> post(postfix, 0);
//pilot_ci16.insert(pilot_ci16.begin(), pre.begin(), pre.end());

//for (int i = 0 ; i < rep; i++)
//    pilot_ci16.insert(pilot_ci16.end(), lts_ci16.begin(), lts_ci16.end());

//pilot_ci16.insert(pilot_ci16.end(),     lts_ci16.begin(), lts_ci16.begin()+frac);
//pilot_ci16.insert(pilot_ci16.end(), post.begin(), post.end());
//pilot = Utils::cint16_to_uint32(pilot_ci16, false, "QI");
#else
    // read pilots from file
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/pilot_f_2048.bin";
    FILE* fp = fopen(filename.c_str(), "rb");
    if (fp == NULL) {
        printf("open file %s faild.\n", filename.c_str());
        std::cerr << "Error: " << strerror(errno) << std::endl;
    }
    fread(pilots_, sizeof(float), OFDM_CA_NUM, fp);
    fclose(fp);
#endif
    std::vector<std::complex<float> > pilotsF(OFDM_CA_NUM);
    for (int i = 0; i < OFDM_CA_NUM; i++)
        pilotsF[i] = pilots_[i];
    pilot_cf32 = CommsLib::IFFT(pilotsF, OFDM_CA_NUM);
    pilot_cf32.insert(pilot_cf32.begin(), pilot_cf32.end() - CP_LEN, pilot_cf32.end()); // add CP
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
    alloc_buffer_2d(&ul_IQ_data, ul_data_symbol_num_perframe * UE_NUM, OFDM_DATA_NUM, 64, 0);
    alloc_buffer_2d(&ul_IQ_modul, ul_data_symbol_num_perframe * UE_NUM, OFDM_CA_NUM, 64, 0);

#ifdef GENERATE_DATA
    int mod_type = modulation == "64QAM" ? CommsLib::QAM64 : (modulation == "16QAM" ? CommsLib::QAM16 : CommsLib::QPSK);
    int mod_order = (int)pow(2, mod_type);

    for (int i = 0; i < data_symbol_num_perframe * UE_NUM; i++) {
        for (int j = 0; j < OFDM_CA_NUM; j++)
            dl_IQ_data[i][j] = rand() % mod_order;
    }

    for (int i = 0; i < ul_data_symbol_num_perframe * UE_NUM; i++) {
        for (int j = 0; j < OFDM_DATA_NUM; j++)
            ul_IQ_data[i][j] = rand() % mod_order;
        std::vector<std::complex<float> > modul_data = CommsLib::modulate(std::vector<int>(ul_IQ_data[i], ul_IQ_data[i] + OFDM_DATA_NUM), mod_type);
        for (int j = 0; j < OFDM_CA_NUM; j++) {
            if (j < OFDM_DATA_START || j >= OFDM_DATA_START + OFDM_DATA_NUM)
                continue;
            int k = j - OFDM_DATA_START;
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
    for (int i = 0; i < data_symbol_num_perframe * UE_NUM; i++) {
        fread(dl_IQ_data[i], sizeof(int), OFDM_CA_NUM, fd);
    }
    fclose(fd);

    // read uplink
    std::string filename2 = cur_directory1 + "/data/tx_ul_data_" + std::to_string(BS_ANT_NUM) + "x" + std::to_string(nUEs) + ".bin";
    fp = fopen(filename2.c_str(), "rb");
    if (fp == NULL) {
        std::cerr << "Openning File " << filename2 << " fails. Error: " << strerror(errno) << std::endl;
    }
    int total_sc = OFDM_DATA_NUM * UE_NUM * ul_data_symbol_num_perframe; // coding is not considered yet
    L2_data = new mac_dtype[total_sc];
    fread(L2_data, sizeof(mac_dtype), total_sc, fp);
    fclose(fp);
    for (int i = 0; i < total_sc; i++) {
        int sid = i / (data_sc_len * nUEs);
        int cid = i % (data_sc_len * nUEs) + OFDM_DATA_START;
        ul_IQ_modul[sid][cid] = L2_data[i];
    }
#endif

    running = true;
}

Config::~Config()
{
    free_buffer_1d(&pilots_);
}

int Config::getDownlinkPilotId(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
    if (it != DLSymbols[fid].end()) {
        int id = it - DLSymbols[fid].begin();
        if (id < DL_PILOT_SYMS) {
#ifdef DEBUG3
            printf("getDownlinkPilotId(%d, %d) = %d\n", frame_id, symbol_id, id);
#endif
            return it - DLSymbols[fid].begin();
        }
    }
    return -1;
}

int Config::getDlSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
    if (it != DLSymbols[fid].end())
        return it - DLSymbols[fid].begin();
    else
        return -1;
}

int Config::getPilotSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(pilotSymbols[fid].begin(), pilotSymbols[fid].end(), symbol_id);
    if (it != pilotSymbols[fid].end()) {
#ifdef DEBUG3
        printf("getPilotSFIndex(%d, %d) = %d\n", frame_id, symbol_id, it - pilotSymbols[fid].begin());
#endif
        return it - pilotSymbols[fid].begin();
    } else
        return -1;
}

int Config::getUlSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(ULSymbols[fid].begin(), ULSymbols[fid].end(), symbol_id);
    if (it != ULSymbols[fid].end()) {
#ifdef DEBUG3
        printf("getUlSFIndexId(%d, %d) = %d\n", frame_id, symbol_id, it - ULSymbols[fid].begin());
#endif
        return it - ULSymbols[fid].begin();
    } else
        return -1;
}

bool Config::isPilot(int frame_id, int symbol_id)
{
    int fid = frame_id % framePeriod;
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
        return (symbol_id >= 0) && (symbol_id < UE_NUM);
#endif
}

bool Config::isUplink(int frame_id, int symbol_id)
{
    int fid = frame_id % framePeriod;
#ifdef USE_UNDEF //USE_ARGOS
    int fid = frame_id % framePeriod;
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

bool Config::isDownlink(int frame_id, int symbol_id)
{
    int fid = frame_id % framePeriod;
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
