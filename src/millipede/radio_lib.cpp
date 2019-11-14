#include "radio_lib.hpp"
#include "comms-lib.h"

RadioConfig::RadioConfig(Config* cfg)
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
    isUE = _cfg->isUE;
    if (!isUE and _cfg->hub_ids.size() != 0) {
        args["driver"] = "remote";
        args["timeout"] = "1000000";
        args["serial"] = _cfg->hub_ids.at(0);
        hubs.push_back(SoapySDR::Device::make(args));
    }

    baStn.resize(_radioNum);
    txStreams.resize(_radioNum);
    rxStreams.resize(_radioNum);
    context = new RadioConfigContext[_radioNum];
    remainingJobs = _radioNum;
    for (size_t i = 0; i < this->_radioNum; i++) {
        context[i].ptr = this;
        context[i].tid = i;
#ifdef THREADED_INIT
        pthread_t init_thread_;
        if (pthread_create(&init_thread_, NULL, RadioConfig::initBSRadio, (void*)(&context[i])) != 0) {
            perror("init thread create failed");
            exit(0);
        }
#else
        RadioConfig::initBSRadio((void*)&context[i]);
#endif
    }

    while (remainingJobs > 0)
        ;

    // Perform DC Offset & IQ Imbalance Calibration
    if (_cfg->imbalanceCalEn) {
        if (_cfg->channel.find('A') != std::string::npos)
            dciqCalibrationProc(0);
        if (_cfg->channel.find('B') != std::string::npos)
            dciqCalibrationProc(1);
    }

    remainingJobs = _radioNum;
    for (size_t i = 0; i < this->_radioNum; i++) {
#ifdef THREADED_INIT
        pthread_t init_thread_;
        if (pthread_create(&init_thread_, NULL, RadioConfig::configureBSRadio, (void*)(&context[i])) != 0) {
            perror("init thread create failed");
            exit(0);
        }
#else
        RadioConfig::initBSRadio((void*)&context[i]);
#endif
    }

    while (remainingJobs > 0)
        ;
    for (size_t i = 0; i < this->_radioNum; i++) {
        std::cout << _cfg->radio_ids.at(i) << ": Front end " << baStn[i]->getHardwareInfo()["frontend"] << std::endl;
        for (size_t c = 0; c < _cfg->nChannels; c++) {
            if (c < baStn[i]->getNumChannels(SOAPY_SDR_RX)) {
                printf("RX Channel %zu\n", c);
                printf("Actual RX sample rate: %fMSps...\n", (baStn[i]->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
                printf("Actual RX frequency: %fGHz...\n", (baStn[i]->getFrequency(SOAPY_SDR_RX, c) / 1e9));
                printf("Actual RX gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_RX, c)));
                printf("Actual RX LNA gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_RX, c, "LNA")));
                printf("Actual RX PGA gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_RX, c, "PGA")));
                printf("Actual RX TIA gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_RX, c, "TIA")));
                if (baStn[i]->getHardwareInfo()["frontend"].compare("CBRS") == 0) {
                    printf("Actual RX LNA1 gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_RX, c, "LNA1")));
                    printf("Actual RX LNA2 gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_RX, c, "LNA2")));
                }
                printf("Actual RX bandwidth: %fM...\n", (baStn[i]->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
                printf("Actual RX antenna: %s...\n", (baStn[i]->getAntenna(SOAPY_SDR_RX, c).c_str()));
            }
        }

        for (size_t c = 0; c < _cfg->nChannels; c++) {
            if (c < baStn[i]->getNumChannels(SOAPY_SDR_TX)) {
                printf("TX Channel %zu\n", c);
                printf("Actual TX sample rate: %fMSps...\n", (baStn[i]->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
                printf("Actual TX frequency: %fGHz...\n", (baStn[i]->getFrequency(SOAPY_SDR_TX, c) / 1e9));
                printf("Actual TX gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_TX, c)));
                printf("Actual TX PAD gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_TX, c, "PAD")));
                printf("Actual TX IAMP gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_TX, c, "IAMP")));
                if (baStn[i]->getHardwareInfo()["frontend"].compare("CBRS") == 0) {
                    printf("Actual TX PA1 gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_TX, c, "PA1")));
                    printf("Actual TX PA2 gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_TX, c, "PA2")));
                    printf("Actual TX PA3 gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_TX, c, "PA3")));
                }
                printf("Actual TX bandwidth: %fM...\n", (baStn[i]->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
                printf("Actual TX antenna: %s...\n", (baStn[i]->getAntenna(SOAPY_SDR_TX, c).c_str()));
            }
        }
        std::cout << std::endl;
    }

    if (hubs.size() == 0)
        baStn[0]->writeSetting("SYNC_DELAYS", "");
    else
        hubs[0]->writeSetting("SYNC_DELAYS", "");

    std::cout << "radio init done!" << std::endl;
}

void* RadioConfig::initBSRadio(void* in_context)
{
    RadioConfig* rc = ((RadioConfigContext*)in_context)->ptr;
    size_t i = ((RadioConfigContext*)in_context)->tid;
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
    rc->baStn[i] = SoapySDR::Device::make(args);
    for (auto ch : { 0, 1 }) {
        rc->baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        rc->baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);
    }
    rc->rxStreams[i] = rc->baStn[i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    rc->txStreams[i] = rc->baStn[i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);

    rc->remainingJobs--;
    return 0;
}

void* RadioConfig::configureBSRadio(void* in_context)
{
    RadioConfig* rc = ((RadioConfigContext*)in_context)->ptr;
    size_t i = ((RadioConfigContext*)in_context)->tid;
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

    // resets the DATA_clk domain logic.
    rc->baStn[i]->writeRegister("IRIS30", 48, (1 << 29) | 0x1);
    rc->baStn[i]->writeRegister("IRIS30", 48, (1 << 29));
    rc->baStn[i]->writeRegister("IRIS30", 48, 0);

    //use the TRX antenna port for both tx and rx
    for (auto ch : channels)
        rc->baStn[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");

    SoapySDR::Kwargs info = rc->baStn[i]->getHardwareInfo();
    for (auto ch : { 0, 1 }) {
        rc->baStn[i]->setBandwidth(SOAPY_SDR_RX, ch, cfg->bwFilter);
        rc->baStn[i]->setBandwidth(SOAPY_SDR_TX, ch, cfg->bwFilter);

        //rc->baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        //rc->baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

        rc->baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg->radioRfFreq);
        rc->baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "BB", cfg->nco);
        rc->baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg->radioRfFreq);
        rc->baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "BB", cfg->nco);

        if (info["frontend"].find("CBRS") != std::string::npos) {
            if (cfg->freq > 3e9)
                rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -6); //[-18,0]
            else if (cfg->freq > 2e9 && cfg->freq < 3e9)
                rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -18); //[-18,0]
            else
                rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
            rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
        }

        rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA", ch ? cfg->rxgainB : cfg->rxgainA); //[0,30]
        rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
        rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]

        if (info["frontend"].find("CBRS") != std::string::npos) {
            rc->baStn[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
            rc->baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|15]
        }
        rc->baStn[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0); //[0,12]
        rc->baStn[i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? cfg->txgainB : cfg->txgainA); //[0,30]
    }

    for (auto ch : channels) {
        //baStn[i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
        //baStn[i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
        rc->baStn[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    // we disable channel 1 because of the internal LDO issue.
    // This will be fixed in the next revision (E) of Iris.
    if (cfg->nChannels == 1) {
        if (cfg->freq > 3e9 and cfg->radio_ids[i].find("RF3E") == std::string::npos) {
            std::cout << "setting up SPI_TDD" << std::endl;
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = rc->baStn[i]->readRegister("LMS7IC", 0x0020);
            rc->baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            //unsigned regRfeA = rc->baStn[i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRfeALo = rc->baStn[i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = rc->baStn[i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfA = rc->baStn[i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbA = rc->baStn[i]->readRegister("LMS7IC", 0x0105);

            // disable TX
            txActive = {
                //0xa10C0000 | 0xfe, //RFE in power down
                //0xa10D0000 | 0x0, //RFE SISO and disables
                0xa1150000 | 0xe, //RBB in power down
                //0xa1000000 | regTrfA //TRF stays the same
                0xa1050000 | regTbbA //TBB stays the same
            };
            rc->baStn[i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
            // disable RX
            rxActive = {
                //0xa10C0000 | regRfeA, //RFE stays the same
                //0xa10D0000 | regRfeALo, //RFE stays the same
                0xa1150000 | regRbbA, //RBB stays the same
                //0xa1000000 | 0xe //TRF in power down + SISO
                0xa1050000 | 0x1e //TBB in power down
            };
            rc->baStn[i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset
        }
        rc->baStn[i]->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
        rc->baStn[i]->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
    } else if (cfg->nChannels == 2) {
        if (cfg->freq > 3e9 and cfg->radio_ids[i].find("RF3E") == std::string::npos) {
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = rc->baStn[i]->readRegister("LMS7IC", 0x0020);
            rc->baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            //unsigned regRfeA = rc->baStn[i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRfeALo = rc->baStn[i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = rc->baStn[i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfA = rc->baStn[i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbA = rc->baStn[i]->readRegister("LMS7IC", 0x0105);

            ch = rc->baStn[i]->readRegister("LMS7IC", 0x0020);
            rc->baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 2);
            //unsigned regRfeB = rc->baStn[i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRbbB = rc->baStn[i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfB = rc->baStn[i]->readRegister("LMS7IC", 0x0100);
            //unsigned regTbbB = rc->baStn[i]->readRegister("LMS7IC", 0x0105);

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

            rc->baStn[i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
            rc->baStn[i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset
            //baStn[i]->writeSetting("SPI_TDD_MODE", "MIMO");
        }
    }
    // resets the DATA_clk domain logic.
    rc->baStn[i]->writeRegister("IRIS30", 48, (1 << 29) | 0x1);
    rc->baStn[i]->writeRegister("IRIS30", 48, (1 << 29));
    rc->baStn[i]->writeRegister("IRIS30", 48, 0);

    rc->remainingJobs--;
    return 0;
}

bool RadioConfig::radioStart()
{
    bool good_calib = false;
    if (!isUE && _cfg->downlink_mode) {
        std::cout << "start sample offset correction" << std::endl;
        int iter = 0;
        int max_iter = 3;
        size_t ref_ant = _cfg->ref_ant;
        while (!good_calib) {
            good_calib = correctSampleOffset(ref_ant, _cfg->sampleCalEn);
            iter++;
            if (iter == max_iter && !good_calib) {
                std::cout << "attempted " << max_iter << " unsucessful calibration, stopping ..." << std::endl;
                break;
            }
        }
        if (!good_calib)
            return good_calib;
        else
            std::cout << "sample offset calibration successful!" << std::endl;
    }

    // send through the first radio for now
    //int beacon_ant = 1;
    int flags(0); // = SOAPY_SDR_WAIT_TRIGGER;
    std::vector<unsigned> zeros(_cfg->sampsPerSymbol, 0);
    std::vector<uint32_t> beacon = _cfg->beacon;
    std::vector<unsigned> beacon_weights(_cfg->nAntennas);

    std::vector<uint32_t> pilot = _cfg->pilot;

    std::vector<std::string> _tddSched;
    if (isUE) {
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
            baStn[i]->writeSetting("TDD_CONFIG", confString);
            baStn[i]->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, _cfg->rate), "TRIGGER");
            baStn[i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            baStn[i]->writeSetting("TDD_MODE", "true");
            baStn[i]->writeRegisters("TX_RAM_A", 0, pilot);
            if (_cfg->nChannels == 2)
                baStn[i]->writeRegisters("TX_RAM_B", 0, pilot);
            baStn[i]->activateStream(this->rxStreams[i], flags, 0);
            baStn[i]->activateStream(this->txStreams[i]);

            std::string corrConfString = "{\"corr_enabled\":true,\"corr_threshold\":" + std::to_string(1) + "}";
            baStn[i]->writeSetting("CORR_CONFIG", corrConfString);
            baStn[i]->writeRegisters("CORR_COE", 0, _cfg->coeffs);

            baStn[i]->writeSetting("CORR_START", "A");
        }
    } else {
        drain_buffers();
        for (size_t f = 0; f < _cfg->framePeriod; f++) {
            std::string sched = _cfg->frames[f];
            if (sched.find("C") != std::string::npos
                && sched.find("L") != std::string::npos)
                calib = true;
        }
        json conf;
        conf["tdd_enabled"] = true;
        conf["frame_mode"] = "free_running";
        conf["max_frame"] = 0;
        conf["symbol_size"] = _cfg->sampsPerSymbol;
        conf["beacon_start"] = _cfg->prefix;
        conf["beacon_stop"] = _cfg->prefix + _cfg->beacon_len;

        std::vector<unsigned> zeros(_cfg->sampsPerSymbol, 0);
        size_t ndx = 0;
        for (size_t i = 0; i < this->_radioNum; i++) {
            bool isRefAnt = (i == _cfg->ref_ant);
            baStn[i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            baStn[i]->writeSetting("TDD_MODE", "true");
            std::vector<std::string> tddSched;
            for (size_t f = 0; f < _cfg->framePeriod; f++) {
                std::string sched = _cfg->frames[f];
                size_t schedSize = sched.size();
                for (size_t s = 0; s < schedSize; s++) {
                    char c = _cfg->frames[f].at(s);
                    if (c == 'C')
                        sched.replace(s, 1, isRefAnt ? "R" : "P");
                    else if (c == 'L')
                        sched.replace(s, 1, isRefAnt ? "P" : "R");
                    else if (c == 'P')
                        sched.replace(s, 1, isRefAnt ? "R" : "R");
                    else if (c == 'U')
                        sched.replace(s, 1, isRefAnt ? "R" : "R");
                    else if (c == 'D')
                        sched.replace(s, 1, isRefAnt ? "T" : "T");
                    else if (c != 'B')
                        sched.replace(s, 1, "G");
                }
                std::cout << sched << std::endl;
                tddSched.push_back(sched);
            }
            conf["frames"] = tddSched;
            std::string confString = conf.dump();
            baStn[i]->writeSetting("TDD_CONFIG", confString);

            baStn[i]->writeRegisters("BEACON_RAM", 0, beacon);
            for (char const& c : _cfg->channel) {
                bool isBeaconAntenna = !_cfg->beamsweep && ndx == _cfg->beacon_ant;
                std::vector<unsigned> beacon_weights(_cfg->nAntennas, isBeaconAntenna ? 1 : 0);
                std::string tx_ram_wgt = "BEACON_RAM_WGT_";
                if (_cfg->beamsweep) {
                    for (size_t j = 0; j < _cfg->nAntennas; j++)
                        beacon_weights[j] = CommsLib::hadamard2(ndx, j);
                }
                baStn[i]->writeRegisters(tx_ram_wgt + c, 0, beacon_weights);
                ++ndx;
            }
            baStn[i]->writeSetting("BEACON_START", std::to_string(_radioNum));
            if (calib) {
                if (isRefAnt) {
                    baStn[i]->writeRegisters("TX_RAM_A", 0, pilot);
		    // looks like the best solution is to just use one
		    // antenna at the reference node and leave the 2nd
		    // antenna unused. We either have to use one anntena
		    // per board, or if we use both channels we need to
		    // exclude reference board from beamforming
                } else {
                    std::vector<std::complex<float>> recipCalDlPilot;
                    recipCalDlPilot = CommsLib::composeRefSymbol(_cfg->pilotsF, 2 * i, this->_radioNum, _cfg->OFDM_CA_NUM);
                    baStn[i]->writeRegisters("TX_RAM_A", 0, Utils::cfloat32_to_uint32(recipCalDlPilot, false, "QI"));
                    if (_cfg->nChannels == 2) {
                        recipCalDlPilot = CommsLib::composeRefSymbol(_cfg->pilotsF, 2 * i + 1, this->_radioNum, _cfg->OFDM_CA_NUM);
                        baStn[i]->writeRegisters("TX_RAM_B", 0, Utils::cfloat32_to_uint32(recipCalDlPilot, false, "QI"));
                    }
                }
            }

            baStn[i]->setHardwareTime(0, "TRIGGER");
            baStn[i]->activateStream(this->rxStreams[i]);
            baStn[i]->activateStream(this->txStreams[i]);
        }
    }

    std::cout << "radio start done!" << std::endl;
    return true;
}

void RadioConfig::go()
{
    if (hubs.size() == 0) {
        //std::cout << "triggering first Iris ..." << std::endl;
        baStn[0]->writeSetting("TRIGGER_GEN", "");
    } else {
        //std::cout << "triggering Hub ..." << std::endl;
        hubs[0]->writeSetting("TRIGGER_GEN", "");
    }
}

void RadioConfig::radioTx(void** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (size_t i = 0; i < this->_radioNum; i++) {
        baStn[i]->writeStream(this->txStreams[i], buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int RadioConfig::radioTx(size_t r /*radio id*/, void** buffs, int flags, long long& frameTime)
{
    int txFlags = 0;
    if (flags == 1)
        txFlags = SOAPY_SDR_HAS_TIME;
    else if (flags == 2)
        txFlags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
    //long long frameTime(0);
    int w = baStn[r]->writeStream(this->txStreams[r], buffs, _cfg->sampsPerSymbol, txFlags, frameTime, 1000000);
#if DEBUG_RADIO_TX
    size_t chanMask;
    long timeoutUs(0);
    int statusFlag = 0;
    int s = baStn[r]->readStreamStatus(this->txStreams[r], chanMask, statusFlag, frameTime, timeoutUs);
    std::cout << "radio " << r << " tx returned " << w << " and status " << s << " when flags was " << flags << std::endl;
#endif
    return w;
}

void RadioConfig::radioRx(void** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (size_t i = 0; i < this->_radioNum; i++) {
        void** buff = buffs + (i * 2);
        baStn[i]->readStream(this->rxStreams[i], buff, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int RadioConfig::radioRx(size_t r /*radio id*/, void** buffs, long long& frameTime)
{
    int flags = 0;
    if (r < this->_radioNum) {
        long long frameTimeNs = 0;
        int ret = baStn[r]->readStream(this->rxStreams[r], buffs, _cfg->sampsPerSymbol, flags, frameTimeNs, 1000000);
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

void RadioConfig::drain_buffers()
{
    std::vector<std::complex<int16_t>> dummyBuff0(_cfg->sampsPerSymbol);
    std::vector<std::complex<int16_t>> dummyBuff1(_cfg->sampsPerSymbol);
    std::vector<void*> dummybuffs(2);
    dummybuffs[0] = dummyBuff0.data();
    dummybuffs[1] = dummyBuff1.data();
    for (size_t i = 0; i < _cfg->nRadios; i++) {
        RadioConfig::drain_rx_buffer(baStn[i], rxStreams[i], dummybuffs, _cfg->sampsPerSymbol);
    }
}

void RadioConfig::drain_rx_buffer(SoapySDR::Device* ibsSdrs, SoapySDR::Stream* istream, std::vector<void*> buffs, size_t symSamp)
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

void RadioConfig::adjustDelays(std::vector<int> offset, size_t ref_ant)
{
    size_t ref_offset = ref_ant == 0 ? 1 : 0;
    size_t R = _cfg->nRadios;
    for (size_t i = 0; i < R; i++) {
        int delta = offset[ref_offset] - offset[i];
        std::cout << "sample_adjusting delay of node " << i << " by " << delta << std::endl;
        int iter = delta < 0 ? -delta : delta;
        for (int j = 0; j < iter; j++) {
            if (delta < 0)
                baStn[i]->writeSetting("ADJUST_DELAYS", "-1");
            else
                baStn[i]->writeSetting("ADJUST_DELAYS", "1");
        }
    }
}

void RadioConfig::readSensors()
{
    for (size_t i = 0; i < this->_radioNum; i++) {
        std::cout << "TEMPs on Iris " << i << std::endl;
        std::cout << "ZYNQ_TEMP: " << baStn[i]->readSensor("ZYNQ_TEMP") << std::endl;
        std::cout << "LMS7_TEMP  : " << baStn[i]->readSensor("LMS7_TEMP") << std::endl;
        std::cout << "FE_TEMP  : " << baStn[i]->readSensor("FE_TEMP") << std::endl;
        std::cout << "TX0 TEMP  : " << baStn[i]->readSensor(SOAPY_SDR_TX, 0, "TEMP") << std::endl;
        std::cout << "TX1 TEMP  : " << baStn[i]->readSensor(SOAPY_SDR_TX, 1, "TEMP") << std::endl;
        std::cout << "RX0 TEMP  : " << baStn[i]->readSensor(SOAPY_SDR_RX, 0, "TEMP") << std::endl;
        std::cout << "RX1 TEMP  : " << baStn[i]->readSensor(SOAPY_SDR_RX, 1, "TEMP") << std::endl;
        std::cout << std::endl;
    }
}

void RadioConfig::radioStop()
{
    std::vector<uint32_t> zeros(4096, 0);
    std::string corrConfStr = "{\"corr_enabled\":false}";
    std::string tddConfStr = "{\"tdd_enabled\":false}";
    for (size_t i = 0; i < this->_radioNum; i++) {
        baStn[i]->deactivateStream(this->rxStreams[i]);
        baStn[i]->deactivateStream(this->txStreams[i]);
        baStn[i]->writeSetting("TDD_MODE", "false");
        baStn[i]->writeSetting("TDD_CONFIG", tddConfStr);
        if (isUE)
            baStn[i]->writeSetting("CORR_CONFIG", corrConfStr);
    }
}

RadioConfig::~RadioConfig()
{
    for (size_t i = 0; i < this->_radioNum; i++) {
        baStn[i]->closeStream(this->rxStreams[i]);
        baStn[i]->closeStream(this->txStreams[i]);
        SoapySDR::Device::unmake(baStn[i]);
    }
}
