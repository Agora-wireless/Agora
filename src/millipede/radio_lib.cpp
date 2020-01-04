#include "radio_lib.hpp"
#include "comms-lib.h"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

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
    remainingJobs = _radioNum;
    for (size_t i = 0; i < this->_radioNum; i++) {
        //args["driver"] = "iris";
        //args["timeout"] = "1000000";
        //args["serial"] = _cfg->radio_ids.at(i);
        //baStn.push_back(SoapySDR::Device::make(args));
#ifdef THREADED_INIT
        RadioConfigContext *context = new RadioConfigContext;
        context->ptr = this;
        context->tid = i;
        pthread_t init_thread_;
        if (pthread_create(&init_thread_, NULL, initBSRadio_launch, context) != 0) {
            perror("init thread create failed");
            exit(0);
        }
#else
        initBSRadio(i);
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

void* RadioConfig::initBSRadio_launch(void* in_context)
{
    RadioConfigContext *context = (RadioConfigContext *)in_context;
    RadioConfig* me = context->ptr;
    int tid = context->tid;
    delete context;
    me->initBSRadio(tid);
    return 0;
}

void RadioConfig::initBSRadio(int i)
{
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

    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["driver"] = "iris";
    args["timeout"] = "1000000";
    args["serial"] = _cfg->radio_ids.at(i);
    baStn[i] = SoapySDR::Device::make(args);

    //use the TRX antenna port for both tx and rx
    for (auto ch : channels)
        baStn[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");

    SoapySDR::Kwargs info = baStn[i]->getHardwareInfo();
    for (auto ch : { 0, 1 }) {
        baStn[i]->setBandwidth(SOAPY_SDR_RX, ch, (1 + 2 * _cfg->bbf_ratio) * _cfg->rate);
        baStn[i]->setBandwidth(SOAPY_SDR_TX, ch, (1 + 2 * _cfg->bbf_ratio) * _cfg->rate);

        baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, _cfg->rate);
        baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, _cfg->rate);

        baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->freq - _cfg->bbf_ratio * _cfg->rate);
        baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "BB", _cfg->bbf_ratio * _cfg->rate);
        baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->freq - _cfg->bbf_ratio * _cfg->rate);
        baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "BB", _cfg->bbf_ratio * _cfg->rate);

        if (info["frontend"].find("CBRS") != std::string::npos) {
            if (_cfg->freq > 3e9)
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]
            else if (_cfg->freq > 2e9 && _cfg->freq < 3e9)
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -18); //[-18,0]
            else
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
            baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
        }

        baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA", ch ? _cfg->rxgainB : _cfg->rxgainA); //[0,30]
        baStn[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
        baStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]

        if (info["frontend"].find("CBRS") != std::string::npos) {
            baStn[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
            baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|15]
        }
        baStn[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0); //[0,12]
        baStn[i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? _cfg->txgainB : _cfg->txgainA); //[0,30]
    }

    for (auto ch : channels) {
        //baStn[i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
        //baStn[i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
        baStn[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    // we disable channel 1 because of the internal LDO issue.
    // This will be fixed in the next revision (E) of Iris.
    if (_cfg->nChannels == 1) {
        if (_cfg->freq > 3e9 and _cfg->radio_ids[i].find("RF3E") == std::string::npos) {
            std::cout << "setting up SPI_TDD" << std::endl;
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = baStn[i]->readRegister("LMS7IC", 0x0020);
            baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            //unsigned regRfeA = baStn[i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRfeALo = baStn[i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = baStn[i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfA = baStn[i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbA = baStn[i]->readRegister("LMS7IC", 0x0105);

            // disable TX
            txActive = {
                //0xa10C0000 | 0xfe, //RFE in power down
                //0xa10D0000 | 0x0, //RFE SISO and disables
                0xa1150000 | 0xe, //RBB in power down
                //0xa1000000 | regTrfA //TRF stays the same
                0xa1050000 | regTbbA //TBB stays the same
            };
            baStn[i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
            // disable RX
            rxActive = {
                //0xa10C0000 | regRfeA, //RFE stays the same
                //0xa10D0000 | regRfeALo, //RFE stays the same
                0xa1150000 | regRbbA, //RBB stays the same
                //0xa1000000 | 0xe //TRF in power down + SISO
                0xa1050000 | 0x1e //TBB in power down
            };
            baStn[i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset

        }
        baStn[i]->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
        baStn[i]->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
    } else if (_cfg->nChannels == 2) {
        if (_cfg->freq > 3e9 and _cfg->radio_ids[i].find("RF3E") == std::string::npos) {
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = baStn[i]->readRegister("LMS7IC", 0x0020);
            baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            //unsigned regRfeA = baStn[i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRfeALo = baStn[i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = baStn[i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfA = baStn[i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbA = baStn[i]->readRegister("LMS7IC", 0x0105);

            ch = baStn[i]->readRegister("LMS7IC", 0x0020);
            baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 2);
            //unsigned regRfeB = baStn[i]->readRegister("LMS7IC", 0x010C);
            //unsigned regRbbB = baStn[i]->readRegister("LMS7IC", 0x0115);
            //unsigned regTrfB = baStn[i]->readRegister("LMS7IC", 0x0100);
            //unsigned regTbbB = baStn[i]->readRegister("LMS7IC", 0x0105);

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

            baStn[i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
            baStn[i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset
            //baStn[i]->writeSetting("SPI_TDD_MODE", "MIMO");
        }
    }
    // resets the DATA_clk domain logic.
    baStn[i]->writeRegister("IRIS30", 48, (1 << 29) | 0x1);
    baStn[i]->writeRegister("IRIS30", 48, (1 << 29));
    baStn[i]->writeRegister("IRIS30", 48, 0);

    rxStreams[i] = baStn[i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    txStreams[i] = baStn[i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
    remainingJobs--;
}

bool RadioConfig::radioStart()
{
    bool good_calib = false;
    if (!isUE && _cfg->downlink_mode) {
        std::cout << "start reciprocity CSI collection" << std::endl;
        int iter = 0;
	int max_iter = 3;
        size_t ref_ant = _cfg->ref_ant;
        while (!good_calib) {
            good_calib = calib_proc(ref_ant, _cfg->sampleCalEn);
            iter++;
            if (iter == max_iter && !good_calib) {
                std::cout << "attempted "<< max_iter <<" unsucessful calibration procs, stopping ..." << std::endl;
                break;
            }
        }
	if (!good_calib)
	    return good_calib;
	else
	    std::cout << "calibration procedure successful!" << std::endl;	
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
                if (c == 'B')
                    _tddSched[r].replace(s, 1, "G");
                else if (c == 'P' and _cfg->pilotSymbols[0][r] != s)
                    _tddSched[r].replace(s, 1, "G");
                else if (c == 'U')
                    _tddSched[r].replace(s, 1, "T");
                else if (c == 'D')
                    _tddSched[r].replace(s, 1, "R");
            }
            std::cout << _tddSched[r] << std::endl;
        }
        int ueTrigOffset = 505; //_cfg->prefix + _cfg->beacon_len + _cfg->postfix + 17 + _cfg->prefix;
        int sf_start = ueTrigOffset / _cfg->sampsPerSymbol;
        int sp_start = ueTrigOffset % _cfg->sampsPerSymbol;
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
            baStn[i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            baStn[i]->writeSetting("TDD_MODE", "true");
            baStn[i]->writeRegisters("TX_RAM_A", 0, pilot);
            if (_cfg->nChannels == 2)
                baStn[i]->writeRegisters("TX_RAM_B", 2048, pilot);
            baStn[i]->activateStream(this->rxStreams[i], flags, 0);
            baStn[i]->activateStream(this->txStreams[i]);

            baStn[i]->writeRegister("IRIS30", CORR_CONF, 0x1);
            for (size_t j = 0; j < _cfg->coeffs.size(); j++)
                baStn[i]->writeRegister("ARGCOE", j * 4, 0);
            usleep(100000);
            baStn[i]->writeRegister("IRIS30", 64, 1); // reset faros_corr
            baStn[i]->writeRegister("IRIS30", 64, 0); // unreset faros_corr
            baStn[i]->writeRegister("IRIS30", 92, 1); // threshold is left-shifted by this many bits
            for (size_t j = 0; j < _cfg->coeffs.size(); j++)
                baStn[i]->writeRegister("ARGCOE", j * 4, _cfg->coeffs[j]);
            baStn[i]->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, _cfg->rate), "TRIGGER");

            baStn[i]->writeRegister("IRIS30", CORR_CONF, 0x11);
        }
    } else {
        drain_buffers();
        _tddSched.resize(_cfg->framePeriod);
        for (size_t f = 0; f < _cfg->framePeriod; f++) {
            _tddSched[f] = _cfg->frames[f];
            for (size_t s = 0; s < _cfg->frames[f].size(); s++) {
                char c = _cfg->frames[f].at(s);
                if (c == 'B')
                    _tddSched[f].replace(s, 1, "P");
                else if (c == 'P')
                    _tddSched[f].replace(s, 1, "R");
                else if (c == 'U')
                    _tddSched[f].replace(s, 1, "R");
                else if (c == 'D')
                    _tddSched[f].replace(s, 1, "T");
            }
            std::cout << _tddSched[f] << std::endl;
        }

        json conf;
        conf["tdd_enabled"] = true;
        conf["frame_mode"] = "free_running";
        conf["max_frame"] = 0;
        conf["frames"] = _tddSched;
        conf["symbol_size"] = _cfg->sampsPerSymbol;
        std::string confString = conf.dump();

        for (size_t i = 0; i < this->_radioNum; i++) {
            baStn[i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            baStn[i]->writeSetting("TDD_MODE", "true");
            baStn[i]->writeSetting("TDD_CONFIG", confString);

            if (_cfg->beacon_mode == "single" or _cfg->nAntennas == 1) {
                if (i * _cfg->nChannels == _cfg->beacon_ant)
                    baStn[i]->writeRegisters("TX_RAM_A", 0, beacon);
                else if (_cfg->nChannels == 2 and i * 2 + 1 == _cfg->beacon_ant)
                    baStn[i]->writeRegisters("TX_RAM_B", 0, beacon);
                else {
                    baStn[i]->writeRegisters("TX_RAM_A", 0, zeros);
                    baStn[i]->writeRegisters("TX_RAM_B", 0, zeros);
                }
            } else // beamsweep
            {
                baStn[i]->writeRegisters("TX_RAM_A", 0, beacon);
                if (_cfg->nChannels == 2)
                    baStn[i]->writeRegisters("TX_RAM_B", 0, beacon);
                //int residue = int(pow(2, ceil(log2(_cfg->nAntennas)))) - _cfg->nAntennas;
                beacon_weights.assign(_cfg->beacon_weights.at(i * _cfg->nChannels).begin(), _cfg->beacon_weights.at(i * _cfg->nChannels).begin() + _cfg->nAntennas);
                baStn[i]->writeRegisters("TX_RAM_WGT_A", 0, beacon_weights);
                if (_cfg->nChannels == 2) {
                    beacon_weights.assign(_cfg->beacon_weights[i * _cfg->nChannels + 1].begin(), _cfg->beacon_weights[i * _cfg->nChannels + 1].begin() + _cfg->nAntennas);
                    baStn[i]->writeRegisters("TX_RAM_WGT_B", 0, beacon_weights);
                }
                baStn[i]->writeRegister("RFCORE", 156, _radioNum);
                baStn[i]->writeRegister("RFCORE", 160, 1); // enable beamsweeping
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
#ifdef DEBUG_RADIO
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
#ifdef DEBUG_RADIO_RX
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
    std::vector<std::complex<int16_t> > dummyBuff0(_cfg->sampsPerSymbol);
    std::vector<std::complex<int16_t> > dummyBuff1(_cfg->sampsPerSymbol);
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

bool RadioConfig::calib_proc(size_t ref_ant, bool sample_adjust)
{
    bool good_csi = true;

    size_t seq_len = _cfg->pilot_cf32.size();
    size_t fft_len = _cfg->OFDM_CA_NUM;
    std::cout << "calibration seq_len " << seq_len << " fft_len " << fft_len << std::endl;
    std::vector<std::complex<double> > pilot_cd64;
    std::vector<std::complex<int16_t> > pilot_cs16;

    for (size_t i = 0; i < seq_len; i++) {
        std::complex<float> cf = _cfg->pilot_cf32[i];
        pilot_cs16.push_back(std::complex<int16_t>((int16_t)(cf.real() * 32768), (int16_t)(cf.imag() * 32768)));
        pilot_cd64.push_back(std::complex<double>(cf.real(), cf.imag()));
    }

    std::vector<std::complex<int16_t> > pre(_cfg->prefix, 0);
    std::vector<std::complex<int16_t> > post(_cfg->postfix, 0);
    pilot_cs16.insert(pilot_cs16.begin(), pre.begin(), pre.end());
    pilot_cs16.insert(pilot_cs16.end(), post.begin(), post.end());
    size_t read_len = pilot_cs16.size();

    // Transmitting from only one chain, create a null vector for chainB
    std::vector<std::complex<int16_t> > dummy_cs16(read_len, 0);

    std::vector<void*> txbuff0(2);
    txbuff0[0] = pilot_cs16.data();
    txbuff0[1] = dummy_cs16.data();

    std::vector<std::vector<std::complex<int16_t> > > buff;
    //int ant = _cfg->nChannels;
    size_t M = _cfg->nAntennas;
    size_t R = _cfg->nRadios;
    assert(M == R); // for now
    buff.resize(M * M);
    for (size_t i = 0; i < M; i++) {
        for (size_t j = 0; j < M; j++) {
            if (i == j)
                buff[i * M + j] = pilot_cs16;
            else
                buff[i * M + j].resize(read_len);
        }
    }

    std::vector<std::complex<int16_t> > dummybuff(read_len);
    drain_buffers();

    for (size_t i = 0; i < R; i++) {
        baStn[i]->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->calTxGainA);
        baStn[i]->writeSetting("TDD_CONFIG", "{\"tdd_enabled\":false}");
        baStn[i]->writeSetting("TDD_MODE", "false");
        baStn[i]->activateStream(this->txStreams[i]);
    }

    long long txTime(0);
    long long rxTime(0);
    for (size_t i = 0; i < R; i++) {
        int tx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
        int ret = baStn[i]->writeStream(this->txStreams[i], txbuff0.data(), pilot_cs16.size(), tx_flags, txTime, 1000000);
        if (ret < (int)read_len)
            std::cout << "bad write\n";
        for (size_t j = 0; j < R; j++) {
            if (j == i)
                continue;
            int rx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
            ret = baStn[j]->activateStream(this->rxStreams[j], rx_flags, rxTime, read_len);
        }

        go();

        int flags = 0;
        for (size_t j = 0; j < R; j++) {
            if (j == i)
                continue;
            std::vector<void*> rxbuff(2);
            rxbuff[0] = buff[(i * R + j)].data();
            //rxbuff[1] = ant == 2 ? buff[(i*M+j)*ant+1].data() : dummyBuff.data();
            rxbuff[1] = dummybuff.data();
            ret = baStn[j]->readStream(this->rxStreams[j], rxbuff.data(), read_len, flags, rxTime, 1000000);
            if (ret < (int)read_len)
                std::cout << "bad read (" << ret << ") at node " << j << " from node " << i << std::endl;
        }
    }

    for (size_t i = 0; i < R; i++) {
        baStn[i]->deactivateStream(this->txStreams[i]);
        baStn[i]->deactivateStream(this->rxStreams[i]);
        baStn[i]->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->txgainA); //[0,30]
    }

    //int ref_ant = 0;
    size_t ref_offset = ref_ant == 0 ? 1 : 0;
    std::vector<int> offset(R);
    std::vector<size_t> start_up(R);
    std::vector<size_t> start_dn(R);

    for (size_t i = 0; i < R; i++) {
        std::vector<std::complex<double> > up(read_len);
        std::vector<std::complex<double> > dn(read_len);
        std::transform(buff[ref_ant * R + i].begin(), buff[ref_ant * R + i].end(), up.begin(),
            [](std::complex<int16_t> ci) { return std::complex<double>(ci.real() / 32768.0, ci.imag() / 32768.0); });
        std::transform(buff[i * R + ref_ant].begin(), buff[i * R + ref_ant].end(), dn.begin(),
            [](std::complex<int16_t> ci) { return std::complex<double>(ci.real() / 32768.0, ci.imag() / 32768.0); });

        size_t peak_up = CommsLib::find_pilot_seq(up, pilot_cd64, seq_len);
        size_t peak_dn = CommsLib::find_pilot_seq(dn, pilot_cd64, seq_len);
        start_up[i] = peak_up < seq_len ? 0 : peak_up - seq_len + _cfg->CP_LEN;
        start_dn[i] = peak_dn < seq_len ? 0 : peak_dn - seq_len + _cfg->CP_LEN;
        std::cout << "receive starting position from/to node " << i
                  << ": " << start_up[i] << "/" << start_dn[i] << std::endl;
        if (start_up[i] == 0 || start_dn[i] == 0)
            good_csi = false;
        if (i == ref_offset) {
            offset[i] = start_up[i];
            offset[ref_ant] = start_dn[i];
        } else if (i != ref_ant)
            offset[i] = start_up[i];

#if DEBUG_PLOT
        std::vector<double> up_I(read_len);
        std::transform( up.begin(), up.end(), up_I.begin(), []( std::complex<double> cd ) {
        return cd.real(); });

        std::vector<double> dn_I(read_len);
        std::transform( dn.begin(), dn.end(), dn_I.begin(), []( std::complex<double> cd ) {
        return cd.real(); });

        plt::figure_size(1200, 780);
        plt::plot(up_I);
        plt::xlim(0, read_len);
        plt::ylim(-1, 1);
        plt::title("ant "+std::to_string(ref_ant)+" (ref) to ant " + std::to_string(i));
        plt::legend();
        plt::save("up_"+std::to_string(i)+".png");

        plt::figure_size(1200, 780);
        plt::plot(dn_I);
        plt::xlim(0, read_len);
        plt::ylim(-1, 1);
        plt::title("ant "+std::to_string(i)+" to ant (ref)" + std::to_string(ref_ant));
        plt::legend();
        plt::save("dn_"+std::to_string(i)+".png");
#endif
    }

    // sample_adjusting trigger delays based on lts peak index
    if (good_csi) {
        if (sample_adjust)
            adjustDelays(offset, ref_offset);
        calib_mat.resize(R);
        for (size_t i = 0; i < R; i++) {
            std::vector<std::complex<float> > dn_t(fft_len);
            std::vector<std::complex<float> > up_t(fft_len);
            std::transform(buff[i * R + ref_ant].begin() + start_dn[i], buff[i * R + ref_ant].begin() + start_dn[i] + fft_len, dn_t.begin(),
                [](std::complex<int16_t> cf) { return std::complex<float>(cf.real() / 32768.0, cf.imag() / 32768.0); });
            std::transform(buff[ref_ant * R + i].begin() + start_up[i], buff[ref_ant * R + i].begin() + start_up[i] + fft_len, up_t.begin(),
                [](std::complex<int16_t> cf) { return std::complex<float>(cf.real() / 32768.0, cf.imag() / 32768.0); });
            std::vector<std::complex<float> > dn_f = CommsLib::FFT(dn_t, fft_len);
            std::vector<std::complex<float> > up_f = CommsLib::FFT(up_t, fft_len);
            for (size_t f = 0; f < fft_len; f++) {
                if (f < _cfg->OFDM_DATA_START || f >= _cfg->OFDM_DATA_START + _cfg->OFDM_DATA_NUM) {
		    continue;
		}
                float dre = dn_f[f].real();
                float dim = dn_f[f].imag();
                float ure = up_f[f].real();
                float uim = up_f[f].imag();
                float re = (dre * ure + dim * uim) / (ure * ure + uim * uim);
                float im = (dim * ure - dre * uim) / (ure * ure + uim * uim);
                if (i == ref_ant)
                    calib_mat[i].push_back(1);
                else
                    calib_mat[i].push_back(std::complex<float>(re, im));
            }
        }
    }

    return good_csi;
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
    int schedLen = _cfg->symbolsPerFrame; //std::max(this->_radioNum+3, _cfg->symbolsPerFrame);
    std::vector<uint32_t> zeros(4096, 0);
    json conf;
    conf["dummy"] = "";
    for (size_t i = 0; i < this->_radioNum; i++) {
        baStn[i]->deactivateStream(this->rxStreams[i]);
        baStn[i]->deactivateStream(this->txStreams[i]);
        baStn[i]->writeSetting("TDD_MODE", "false");
        // write schedule
        for (int k = 0; k < schedLen; k++) // symnum <= 256
        {
            baStn[i]->writeRegister("RFCORE", 136, k);
            baStn[i]->writeRegister("RFCORE", 140, 0);
        }
        baStn[i]->writeSetting("TDD_CONFIG", conf.dump());
        //baStn[i]->writeRegister("RFCORE", 120, 0);
        //baStn[i]->writeRegister("RFCORE", 124, 0);
        //baStn[i]->writeRegister("RFCORE", 128, 0);
        //baStn[i]->writeRegister("RFCORE", 132, 0);
        baStn[i]->writeRegisters("TX_RAM_A", 0, zeros);
        baStn[i]->writeRegisters("TX_RAM_B", 0, zeros);
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
