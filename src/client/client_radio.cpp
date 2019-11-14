#include "client_radio.hpp"
#include "comms-lib.h"

ClientRadioConfig::ClientRadioConfig(Config* cfg)
    : _cfg(cfg)
{
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    //load channels
    std::vector<size_t> channels;
    if (_cfg->nChannels == 1)
        channels = { 0 };
    else if (_cfg->nChannels == 2)
        channels = { 0, 1 };
    else {
        std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
        _cfg->nChannels = 2;
        channels = { 0, 1 };
    }

    this->_radioNum = _cfg->nRadios;
    this->_antennaNum = _radioNum * _cfg->nChannels;
    std::cout << "radio num is " << this->_radioNum << std::endl;

    clStn.resize(_radioNum);
    txStreams.resize(_radioNum);
    rxStreams.resize(_radioNum);
    context = new ClientRadioConfigContext[_radioNum];
    remainingJobs = _radioNum;
    for (size_t i = 0; i < this->_radioNum; i++) {
        context[i].ptr = this;
        context[i].tid = i;
#ifdef THREADED_INIT
        pthread_t init_thread_;
        if (pthread_create(&init_thread_, NULL, ClientRadioConfig::initClientRadio, (void*)(&context[i])) != 0) {
            perror("init thread create failed");
            exit(0);
        }
#else
        ClientRadioConfig::initClientRadio((void*)&context[i]);
#endif
    }

    while (remainingJobs > 0)
        ;

    for (size_t i = 0; i < this->_radioNum; i++) {
        std::cout << _cfg->radio_ids.at(i) << ": Front end " << clStn[i]->getHardwareInfo()["frontend"] << std::endl;
        for (size_t c = 0; c < _cfg->nChannels; c++) {
            if (c < clStn[i]->getNumChannels(SOAPY_SDR_RX)) {
                printf("RX Channel %zu\n", c);
                printf("Actual RX sample rate: %fMSps...\n", (clStn[i]->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
                printf("Actual RX frequency: %fGHz...\n", (clStn[i]->getFrequency(SOAPY_SDR_RX, c) / 1e9));
                printf("Actual RX gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_RX, c)));
                printf("Actual RX LNA gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_RX, c, "LNA")));
                printf("Actual RX PGA gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_RX, c, "PGA")));
                printf("Actual RX TIA gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_RX, c, "TIA")));
                if (clStn[i]->getHardwareInfo()["frontend"].compare("CBRS") == 0) {
                    printf("Actual RX LNA1 gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_RX, c, "LNA1")));
                    printf("Actual RX LNA2 gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_RX, c, "LNA2")));
                }
                printf("Actual RX bandwidth: %fM...\n", (clStn[i]->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
                printf("Actual RX antenna: %s...\n", (clStn[i]->getAntenna(SOAPY_SDR_RX, c).c_str()));
            }
        }

        for (size_t c = 0; c < _cfg->nChannels; c++) {
            if (c < clStn[i]->getNumChannels(SOAPY_SDR_TX)) {
                printf("TX Channel %zu\n", c);
                printf("Actual TX sample rate: %fMSps...\n", (clStn[i]->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
                printf("Actual TX frequency: %fGHz...\n", (clStn[i]->getFrequency(SOAPY_SDR_TX, c) / 1e9));
                printf("Actual TX gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_TX, c)));
                printf("Actual TX PAD gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_TX, c, "PAD")));
                printf("Actual TX IAMP gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_TX, c, "IAMP")));
                if (clStn[i]->getHardwareInfo()["frontend"].compare("CBRS") == 0) {
                    printf("Actual TX PA1 gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_TX, c, "PA1")));
                    printf("Actual TX PA2 gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_TX, c, "PA2")));
                    printf("Actual TX PA3 gain: %f...\n", (clStn[i]->getGain(SOAPY_SDR_TX, c, "PA3")));
                }
                printf("Actual TX bandwidth: %fM...\n", (clStn[i]->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
                printf("Actual TX antenna: %s...\n", (clStn[i]->getAntenna(SOAPY_SDR_TX, c).c_str()));
            }
        }
        std::cout << std::endl;
    }


    std::cout << "radio init done!" << std::endl;
}

void* ClientRadioConfig::initClientRadio(void* in_context)
{
    ClientRadioConfig* rc = ((ClientRadioConfigContext*)in_context)->ptr;
    size_t i = ((ClientRadioConfigContext*)in_context)->tid;
    Config* cfg = rc->_cfg;

    //load channels
    std::vector<size_t> channels;
    if (cfg->nChannels == 1)
        channels = { 0 };
    else if (cfg->nChannels == 2)
        channels = { 0, 1 };
    else {
        std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
        cfg->nChannels = 2;
        channels = { 0, 1 };
    }

    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["driver"] = "iris";
    args["timeout"] = "1000000";
    args["serial"] = cfg->radio_ids.at(i);
    rc->clStn[i] = SoapySDR::Device::make(args);
    for (auto ch : { 0, 1 }) {
        rc->clStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        rc->clStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);
    }
    rc->rxStreams[i] = rc->clStn[i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    rc->txStreams[i] = rc->clStn[i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);

    // resets the DATA_clk domain logic.
    rc->clStn[i]->writeRegister("IRIS30", 48, (1 << 29) | 0x1);
    rc->clStn[i]->writeRegister("IRIS30", 48, (1 << 29));
    rc->clStn[i]->writeRegister("IRIS30", 48, 0);

    //use the TRX antenna port for both tx and rx
    for (auto ch : channels)
        rc->clStn[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");

    SoapySDR::Kwargs info = rc->clStn[i]->getHardwareInfo();
    for (auto ch : { 0, 1 }) {
        rc->clStn[i]->setBandwidth(SOAPY_SDR_RX, ch, cfg->bwFilter);
        rc->clStn[i]->setBandwidth(SOAPY_SDR_TX, ch, cfg->bwFilter);

        //rc->clStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        //rc->clStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

        rc->clStn[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg->radioRfFreq);
        rc->clStn[i]->setFrequency(SOAPY_SDR_RX, ch, "BB", cfg->nco);
        rc->clStn[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg->radioRfFreq);
        rc->clStn[i]->setFrequency(SOAPY_SDR_TX, ch, "BB", cfg->nco);

        if (info["frontend"].find("CBRS") != std::string::npos) {
            if (cfg->freq > 3e9)
                rc->clStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -6); //[-18,0]
            else if (cfg->freq > 2e9 && cfg->freq < 3e9)
                rc->clStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -18); //[-18,0]
            else
                rc->clStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
            rc->clStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
        }

        rc->clStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA", ch ? cfg->rxgainB : cfg->rxgainA); //[0,30]
        rc->clStn[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
        rc->clStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]

        if (info["frontend"].find("CBRS") != std::string::npos) {
            rc->clStn[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
            rc->clStn[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|15]
        }
        rc->clStn[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0); //[0,12]
        rc->clStn[i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? cfg->txgainB : cfg->txgainA); //[0,30]
    }

    for (auto ch : channels) {
        //clStn[i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
        //clStn[i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
        rc->clStn[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    // we disable channel 1 because of the internal LDO issue.
    // This will be fixed in the next revision (E) of Iris.
    if (cfg->nChannels == 1) {
        if (cfg->freq > 3e9 and cfg->radio_ids[i].find("RF3E") == std::string::npos) {
            std::cout << "setting up SPI_TDD" << std::endl;
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = rc->clStn[i]->readRegister("LMS7IC", 0x0020);
            rc->clStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            //unsigned regRfeA = rc->clStn[i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRfeALo = rc->clStn[i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = rc->clStn[i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfA = rc->clStn[i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbA = rc->clStn[i]->readRegister("LMS7IC", 0x0105);

            // disable TX
            txActive = {
                //0xa10C0000 | 0xfe, //RFE in power down
                //0xa10D0000 | 0x0, //RFE SISO and disables
                0xa1150000 | 0xe, //RBB in power down
                //0xa1000000 | regTrfA //TRF stays the same
                0xa1050000 | regTbbA //TBB stays the same
            };
            rc->clStn[i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
            // disable RX
            rxActive = {
                //0xa10C0000 | regRfeA, //RFE stays the same
                //0xa10D0000 | regRfeALo, //RFE stays the same
                0xa1150000 | regRbbA, //RBB stays the same
                //0xa1000000 | 0xe //TRF in power down + SISO
                0xa1050000 | 0x1e //TBB in power down
            };
            rc->clStn[i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset
        }
        rc->clStn[i]->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
        rc->clStn[i]->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
    } else if (cfg->nChannels == 2) {
        if (cfg->freq > 3e9 and cfg->radio_ids[i].find("RF3E") == std::string::npos) {
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = rc->clStn[i]->readRegister("LMS7IC", 0x0020);
            rc->clStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            //unsigned regRfeA = rc->clStn[i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRfeALo = rc->clStn[i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = rc->clStn[i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfA = rc->clStn[i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbA = rc->clStn[i]->readRegister("LMS7IC", 0x0105);

            ch = rc->clStn[i]->readRegister("LMS7IC", 0x0020);
            rc->clStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 2);
            //unsigned regRfeB = rc->clStn[i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRbbB = rc->clStn[i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfB = rc->clStn[i]->readRegister("LMS7IC", 0x0100);
            //unsigned regTbbB = rc->clStn[i]->readRegister("LMS7IC", 0x0105);

            txActive = {
                //0xe10C0000 | 0xfe, //RFE in power down
                //0xe10D0000 | 0x0, //RFE SISO and disables
                0xe1150000 | 0xe, //RBB in power down
                //0xe1000000 | regTrfA, //TRF stays the same
                0xe1050000 | regTbbA
            }; //TBB stays the same

            rxActive = {
                //0xe10C0000 | regRfeA, //RFE stays the same
                //0xe10D0000 | regRfeALo, //RFE stays the same
                0xe1150000 | regRbbA, //RBB stays the same
                //0xe1000000 | 0xe, //TRF in power down + SISO
                0xe1050000 | 0x1e
            }; //TBB in power down

            rc->clStn[i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
            rc->clStn[i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset
            //clStn[i]->writeSetting("SPI_TDD_MODE", "MIMO");
        }
    }
    // resets the DATA_clk domain logic.
    rc->clStn[i]->writeRegister("IRIS30", 48, (1 << 29) | 0x1);
    rc->clStn[i]->writeRegister("IRIS30", 48, (1 << 29));
    rc->clStn[i]->writeRegister("IRIS30", 48, 0);

    rc->remainingJobs--;
    return 0;
}

bool ClientRadioConfig::radioStart()
{
    // send through the first radio for now
    //int beacon_ant = 1;
    int flags(0); // = SOAPY_SDR_WAIT_TRIGGER;
    std::vector<unsigned> zeros(_cfg->sampsPerSymbol, 0);
    std::vector<uint32_t> beacon = _cfg->beacon;
    std::vector<unsigned> beacon_weights(_cfg->nAntennas);

    std::vector<uint32_t> pilot = _cfg->pilot;

    std::vector<std::string> _tddSched;
    _tddSched.resize(this->_radioNum);
    for (size_t r = 0; r < _radioNum; r++) {
        _tddSched[r] = _cfg->frames[0];
        for (size_t s = 0; s < _cfg->frames[0].size(); s++) {
            char c = _cfg->frames[0].at(s);
            if (c == 'P' and _cfg->pilotSymbols[0][r] != s)
                _tddSched[r].replace(s, 1, "G");
            else if (c == 'U')
                _tddSched[r].replace(s, 1, "T");
            else if (c == 'D')
                _tddSched[r].replace(s, 1, "R");
            else
                _tddSched[r].replace(s, 1, "G");
        }
        std::cout << _tddSched[r] << std::endl;
    }
    //beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) + 82 (Client FE Delay)
    int clTrigOffset = _cfg->beacon_len + 249;
    int sf_start = clTrigOffset / _cfg->sampsPerSymbol;
    int sp_start = clTrigOffset % _cfg->sampsPerSymbol;
    for (size_t i = 0; i < this->_radioNum; i++) {
        json conf;
        conf["tdd_enabled"] = true;
        conf["frame_mode"] = "continuous_resync";
        int max_frame_ = (int)(2.0 / ((_cfg->sampsPerSymbol * _cfg->symbolsPerFrame) / _cfg->rate));
        conf["max_frame"] = max_frame_;
        conf["dual_pilot"] = (_cfg->nChannels == 2);
        std::vector<std::string> jframes;
        jframes.push_back(_tddSched[i]);
        conf["frames"] = jframes;
        conf["symbol_size"] = _cfg->sampsPerSymbol;
        std::string confString = conf.dump();
        clStn[i]->writeSetting("TDD_CONFIG", confString);
        clStn[i]->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, _cfg->rate), "TRIGGER");
        clStn[i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
        clStn[i]->writeSetting("TDD_MODE", "true");
        clStn[i]->writeRegisters("TX_RAM_A", 0, pilot);
        if (_cfg->nChannels == 2)
            clStn[i]->writeRegisters("TX_RAM_B", 0, pilot);
        clStn[i]->activateStream(this->rxStreams[i], flags, 0);
        clStn[i]->activateStream(this->txStreams[i]);

        std::string corrConfString = "{\"corr_enabled\":true,\"corr_threshold\":" + std::to_string(1) + "}";
        clStn[i]->writeSetting("CORR_CONFIG", corrConfString);
        clStn[i]->writeRegisters("CORR_COE", 0, _cfg->coeffs);

        clStn[i]->writeSetting("CORR_START", "A");
    }

    std::cout << "radio start done!" << std::endl;
    return true;
}

void ClientRadioConfig::go()
{
    if (hubs.size() == 0) {
        //std::cout << "triggering first Iris ..." << std::endl;
        clStn[0]->writeSetting("TRIGGER_GEN", "");
    } else {
        //std::cout << "triggering Hub ..." << std::endl;
        hubs[0]->writeSetting("TRIGGER_GEN", "");
    }
}

void ClientRadioConfig::radioTx(void** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (size_t i = 0; i < this->_radioNum; i++) {
        clStn[i]->writeStream(this->txStreams[i], buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int ClientRadioConfig::radioTx(size_t r /*radio id*/, void** buffs, int flags, long long& frameTime)
{
    int txFlags = 0;
    if (flags == 1)
        txFlags = SOAPY_SDR_HAS_TIME;
    else if (flags == 2)
        txFlags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
    //long long frameTime(0);
    int w = clStn[r]->writeStream(this->txStreams[r], buffs, _cfg->sampsPerSymbol, txFlags, frameTime, 1000000);
#if DEBUG_RADIO_TX
    size_t chanMask;
    long timeoutUs(0);
    int statusFlag = 0;
    int s = clStn[r]->readStreamStatus(this->txStreams[r], chanMask, statusFlag, frameTime, timeoutUs);
    std::cout << "radio " << r << " tx returned " << w << " and status " << s << " when flags was " << flags << std::endl;
#endif
    return w;
}

void ClientRadioConfig::radioRx(void** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (size_t i = 0; i < this->_radioNum; i++) {
        void** buff = buffs + (i * 2);
        clStn[i]->readStream(this->rxStreams[i], buff, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int ClientRadioConfig::radioRx(size_t r /*radio id*/, void** buffs, long long& frameTime)
{
    int flags = 0;
    if (r < this->_radioNum) {
        long long frameTimeNs = 0;
        int ret = clStn[r]->readStream(this->rxStreams[r], buffs, _cfg->sampsPerSymbol, flags, frameTimeNs, 1000000);
        frameTime = frameTimeNs; //SoapySDR::timeNsToTicks(frameTimeNs, _rate);
        if (ret != (int)_cfg->sampsPerSymbol)
            std::cout << "invalid return " << ret << " from radio " << r << std::endl;
#if DEBUG_RADIO_RX
        else
            std::cout << "radio " << r << "received " << ret << std::endl;
#endif
        return ret;
    }
    std::cout << "invalid radio id " << r << std::endl;
    return 0;
}

void ClientRadioConfig::drain_buffers()
{
    std::vector<std::complex<int16_t>> dummyBuff0(_cfg->sampsPerSymbol);
    std::vector<std::complex<int16_t>> dummyBuff1(_cfg->sampsPerSymbol);
    std::vector<void*> dummybuffs(2);
    dummybuffs[0] = dummyBuff0.data();
    dummybuffs[1] = dummyBuff1.data();
    for (size_t i = 0; i < _cfg->nRadios; i++) {
        ClientRadioConfig::drain_rx_buffer(clStn[i], rxStreams[i], dummybuffs, _cfg->sampsPerSymbol);
    }
}

void ClientRadioConfig::drain_rx_buffer(SoapySDR::Device* ibsSdrs, SoapySDR::Stream* istream, std::vector<void*> buffs, size_t symSamp)
{
    long long frameTime = 0;
    int flags = 0, r = 0, i = 0;
    long timeoutUs(0);
    while (r != -1) {
        r = ibsSdrs->readStream(istream, buffs.data(), symSamp, flags, frameTime, timeoutUs);
        i++;
    }
    //std::cout << "Number of reads needed to drain: " << i << std::endl;
}

void ClientRadioConfig::readSensors()
{
    for (size_t i = 0; i < this->_radioNum; i++) {
        std::cout << "TEMPs on Iris " << i << std::endl;
        std::cout << "ZYNQ_TEMP: " << clStn[i]->readSensor("ZYNQ_TEMP") << std::endl;
        std::cout << "LMS7_TEMP  : " << clStn[i]->readSensor("LMS7_TEMP") << std::endl;
        std::cout << "FE_TEMP  : " << clStn[i]->readSensor("FE_TEMP") << std::endl;
        std::cout << "TX0 TEMP  : " << clStn[i]->readSensor(SOAPY_SDR_TX, 0, "TEMP") << std::endl;
        std::cout << "TX1 TEMP  : " << clStn[i]->readSensor(SOAPY_SDR_TX, 1, "TEMP") << std::endl;
        std::cout << "RX0 TEMP  : " << clStn[i]->readSensor(SOAPY_SDR_RX, 0, "TEMP") << std::endl;
        std::cout << "RX1 TEMP  : " << clStn[i]->readSensor(SOAPY_SDR_RX, 1, "TEMP") << std::endl;
        std::cout << std::endl;
    }
}

void ClientRadioConfig::radioStop()
{
    std::vector<uint32_t> zeros(4096, 0);
    std::string corrConfStr = "{\"corr_enabled\":false}";
    std::string tddConfStr = "{\"tdd_enabled\":false}";
    for (size_t i = 0; i < this->_radioNum; i++) {
        clStn[i]->deactivateStream(this->rxStreams[i]);
        clStn[i]->deactivateStream(this->txStreams[i]);
        clStn[i]->writeSetting("TDD_MODE", "false");
        clStn[i]->writeSetting("TDD_CONFIG", tddConfStr);
        clStn[i]->writeSetting("CORR_CONFIG", corrConfStr);
    }
}

ClientRadioConfig::~ClientRadioConfig()
{
    for (size_t i = 0; i < this->_radioNum; i++) {
        clStn[i]->closeStream(this->rxStreams[i]);
        clStn[i]->closeStream(this->txStreams[i]);
        SoapySDR::Device::unmake(clStn[i]);
    }
}
