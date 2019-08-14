
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
    ref_ant = tddConf.value("ref_ant", "0");
    nCells = tddConf.value("cells", 1);
    nChannels = tddConf.value("channels", 1);
    isUE = tddConf.value("UE", false);
    freq = tddConf.value("frequency", 3.6e9);
    txgainA = tddConf.value("txgainA", 20);
    rxgainA = tddConf.value("rxgainA", 20);
    txgainB = tddConf.value("txgainB", 20);
    rxgainB = tddConf.value("rxgainB", 20);
    calTxGainA = tddConf.value("calTxGainA", 40);
    calTxGainB = tddConf.value("calTxGainB", 40);
    rate = tddConf.value("rate", 5e6);
    sampsPerSymbol = tddConf.value("symbol_size", 0);
    prefix = tddConf.value("prefix", 0);
    postfix = tddConf.value("postfix", 0);
    beacon_ant = tddConf.value("beacon_antenna", 0);
    beacon_len = tddConf.value("beacon_len", 256);
    beacon_mode = tddConf.value("beacon_mode", "single");
    sampleCalEn = tddConf.value("sample_calibrate", true);
    modulation = tddConf.value("modulation", "QPSK");
    core_offset = tddConf.value("core_offset", 1);
    rx_addr = tddConf.value("rx_addr", "127.0.0.1");
    tx_addr = tddConf.value("tx_addr", "127.0.0.1");
    tx_port = tddConf.value("tx_port", 7991);
    rx_port = tddConf.value("rx_port", 7891);
    json jframes = tddConf.value("frames", json::array());
    framePeriod = jframes.size();
    for(int f = 0; f < framePeriod; f++)
    {
        frames.push_back(jframes.at(0).get<std::string>());
    }
    Utils::loadDevices(hub_file, hub_ids);
    Utils::loadDevices(serial_file, radio_ids);
    nRadios = radio_ids.size();
    nAntennas = nChannels * nRadios;
    if (beacon_mode == "beamsweep" && nAntennas > 1)
    {
        int hadamardSize =  int(pow(2,ceil(log2(nAntennas))));
        std::vector<std::vector<double>> hadamard_weights = CommsLib::getSequence(hadamardSize, CommsLib::HADAMARD);
        beacon_weights.resize(nAntennas);
        for (int i = 0; i < nAntennas; i++)
            for (int j = 0; j < nAntennas; j++) 
                beacon_weights[i].push_back((unsigned)hadamard_weights[i][j]);
        printf("Hadamard Matrix Size %d\n", hadamardSize);
        printf("beacon_weights size %d\n", beacon_weights.size());
        printf("beacon_weights[0] size %d\n", beacon_weights[0].size());
    }
    symbolsPerFrame = frames.at(0).size();
    //nUEs = boost::count(frames.at(0), 'P'); // FIXME: this shouldn't be the case. UEs could be more  // tddConf.value("user_num", 1);
    nUEs = std::count(frames.at(0).begin(), frames.at(0).end(), 'P');
    pilotSymbols.resize(framePeriod);
    for(int f = 0; f < framePeriod; f++)
    {
        std::string fr = frames[f]; 
        for (int g = 0; g < symbolsPerFrame; g++)
        {
            if (fr[g] == 'P')
                pilotSymbols[f].push_back(g);
        }
    }
    pilotSymsPerFrame = pilotSymbols[0].size();
    if (isUE and nRadios != pilotSymsPerFrame)
    {
        std::cerr << "Number of Pilot Symbols don't match number of Clients!" << std::endl;
        exit(0);
    }

    ULSymbols.resize(framePeriod);
    for(int f = 0; f < framePeriod; f++)
    {
        std::string fr = frames[f]; 
        for (int g = 0; g < symbolsPerFrame; g++)
        {
            if (fr[g] == 'U')
                ULSymbols[f].push_back(g);
        }
    }
    ulSymsPerFrame = ULSymbols[0].size();
    DLSymbols.resize(framePeriod);
    for(int f = 0; f < framePeriod; f++)
    {
        std::string fr = frames[f]; 
        for (int g = 0; g < symbolsPerFrame; g++)
        {
            if (fr[g] == 'D')
                DLSymbols[f].push_back(g);
        }
    }
    dlSymsPerFrame = DLSymbols[0].size();
    std::cout << "Config file loaded!" << std::endl;

#ifdef USE_ARGOS
    std::vector<std::vector<double>> gold_ifft = CommsLib::getSequence(128, CommsLib::GOLD_IFFT);
    std::vector<std::complex<int16_t>> coeffs_ci16 = Utils::double_to_int16(gold_ifft);
    coeffs = Utils::cint16_to_uint32(coeffs_ci16, true, "QI");
    beacon_ci16.resize(256); 
    for (int i = 0; i < 128; i++)
    {
        beacon_ci16[i] = std::complex<int16_t>( (int16_t)(gold_ifft[0][i]*32768), (int16_t)(gold_ifft[1][i]*32768) );
        beacon_ci16[i+128] = beacon_ci16[i];
    }

    std::vector<std::complex<int16_t>> pre0(prefix, 0);
    std::vector<std::complex<int16_t>> post0(sampsPerSymbol-256-prefix, 0);
    beacon_ci16.insert(beacon_ci16.begin(),pre0.begin(),pre0.end());
    beacon_ci16.insert(beacon_ci16.end(),post0.begin(),post0.end());
    beacon = Utils::cint16_to_uint32(beacon_ci16, false, "QI"); 
#endif

#ifdef GENERATE_PILOT 
    // compose pilot subframe
    std::vector<std::vector<double>> lts = CommsLib::getSequence(80, CommsLib::LTS_SEQ);
    std::vector<std::complex<int16_t>> lts_ci16 = Utils::double_to_int16(lts);
    int nSamps = sampsPerSymbol - prefix - postfix;
    int rep = nSamps / 80;
    int frac = nSamps % 80;
    std::vector<std::complex<int16_t>> pre(prefix, 0);
    std::vector<std::complex<int16_t>> post(postfix, 0);
    pilot_ci16.insert(pilot_ci16.begin(), pre.begin(), pre.end());

    for (int i = 0 ; i < rep; i++)
        pilot_ci16.insert(pilot_ci16.end(), lts_ci16.begin(), lts_ci16.end());

    pilot_ci16.insert(pilot_ci16.end(),     lts_ci16.begin(), lts_ci16.begin()+frac);
    pilot_ci16.insert(pilot_ci16.end(), post.begin(), post.end());
    pilot = Utils::cint16_to_uint32(pilot_ci16, false, "QI");
#else
    // read pilots from file
    pilots_ = (float *)aligned_alloc(64, OFDM_CA_NUM * sizeof(float));
#ifdef USE_ARGOS
    FILE* fp = fopen("data/pilot2_f_2048.bin","rb");
#else
    FILE* fp = fopen("data/pilot_f_2048.bin","rb");
#endif
    fread(pilots_, sizeof(float), OFDM_CA_NUM, fp);
    fclose(fp);
    std::vector<std::complex<float>> pilotsF(OFDM_CA_NUM);
    for (int i = 0; i < OFDM_CA_NUM; i++)  pilotsF[i] = pilots_[i];
    std::vector<std::complex<float>> pilot_cf32 = CommsLib::IFFT(pilotsF, OFDM_CA_NUM);
    pilot = Utils::cfloat32_to_uint32(pilot_cf32, false, "QI");
#endif
    running = true;
}

Config::~Config(){}

int Config::getDownlinkPilotId(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
    if (it != DLSymbols[fid].end()) 
    {
        int id = it-DLSymbols[fid].begin();
        if (id < DL_PILOT_SYMS) {
#ifdef DEBUG3
            printf("getDownlinkPilotId(%d, %d) = %d\n",frame_id, symbol_id, id);
#endif
            return it-DLSymbols[fid].begin();
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
        return it-DLSymbols[fid].begin();
    else 
        return -1;
}

int Config::getPilotSFIndex(int frame_id, int symbol_id)
{
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(pilotSymbols[fid].begin(), pilotSymbols[fid].end(), symbol_id);
    if (it != pilotSymbols[fid].end()) 
    {
#ifdef DEBUG3
        printf("getPilotSFIndex(%d, %d) = %d\n",frame_id, symbol_id, it-pilotSymbols[fid].begin());
#endif
        return it-pilotSymbols[fid].begin();
    }else 
        return -1;
}

int Config::getUlSFIndex(int frame_id, int symbol_id)
{
    //return subframe_id-6; // fix
    std::vector<size_t>::iterator it;
    int fid = frame_id % framePeriod;
    it = find(ULSymbols[fid].begin(), ULSymbols[fid].end(), symbol_id);
    if (it != ULSymbols[fid].end()) 
    {
#ifdef DEBUG3
        printf("getUlSFIndexId(%d, %d) = %d\n",frame_id, symbol_id, it-ULSymbols[fid].begin());
#endif
        return it-ULSymbols[fid].begin();
    }else 
        return -1;
}

bool Config::isPilot(int frame_id, int symbol_id) 
{
    int fid = frame_id % framePeriod;
    if (symbol_id > symbolsPerFrame)
    {
        printf("\x1B[31mERROR: Received out of range symbol %d at frame %d\x1B[0m\n",symbol_id, frame_id);
        return false;
    }
#ifdef DEBUG3
    printf("isPilot(%d, %d) = %c\n",frame_id, symbol_id, frames[fid].at(symbol_id));
#endif
    if (isUE)
    {
        std::vector<size_t>::iterator it;
        it = find(DLSymbols[fid].begin(), DLSymbols[fid].end(), symbol_id);
        int ind = DL_PILOT_SYMS;
        if (it != DLSymbols[fid].end()) 
            ind = it-DLSymbols[fid].begin();
        return (ind < DL_PILOT_SYMS);
        //return cfg->frames[fid].at(symbol_id) == 'P' ? true : false;
    }
    else
        return frames[fid].at(symbol_id) == 'P';
} 

bool Config::isUplink(int frame_id, int symbol_id) 
{
    int fid = frame_id % framePeriod;
    if (symbol_id > symbolsPerFrame)
    {
        printf("\x1B[31mERROR: Received out of range symbol %d at frame %d\x1B[0m\n",symbol_id, frame_id);
        return false;
    }
#ifdef DEBUG3
    printf("isUplink(%d, %d) = %c\n",frame_id, symbol_id, frames[fid].at(symbol_id));
#endif
    return frames[fid].at(symbol_id) == 'U';
}

bool Config::isDownlink(int frame_id, int symbol_id) 
{
    int fid = frame_id % framePeriod;
#ifdef DEBUG3
    printf("isDownlink(%d, %d) = %c\n",frame_id, symbol_id, frames[fid].at(symbol_id));
#endif
    if (isUE)
        return frames[fid].at(symbol_id) == 'D' && !this->isPilot(frame_id, symbol_id);
    else
        return frames[fid].at(symbol_id) == 'D';
} 

extern "C"
{
    __attribute__((visibility("default"))) Config* Config_new(char *filename) {
        
        Config *cfg = new Config(filename);
        
        return cfg;
    }
}
