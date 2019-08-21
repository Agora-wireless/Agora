#include "radio_lib.hpp"
#include "comms-lib.h"

RadioConfig::RadioConfig(Config *cfg):
    _cfg(cfg)
{
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    //load channels
    std::vector<size_t> channels;
    if (_cfg->nChannels == 1) channels = {0};
    else if (_cfg->nChannels == 2) channels = {0, 1};
    else
    {
        std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
        _cfg->nChannels = 2;
        channels = {0, 1};
    }

    this->_radioNum = _cfg->nRadios;
    this->_antennaNum = _radioNum * _cfg->nChannels;
    std::cout << "radio num is " << this->_radioNum << std::endl;
    isUE = _cfg->isUE;
    if (!isUE and _cfg->hub_ids.size() != 0)
    { 
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
    for (int i = 0; i < this->_radioNum; i++)
    {
        //args["driver"] = "iris";
        //args["timeout"] = "1000000";
        //args["serial"] = _cfg->radio_ids.at(i);
        //baStn.push_back(SoapySDR::Device::make(args));
        context[i].ptr = this;
        context[i].tid = i;
#ifdef THREADED_INIT
        pthread_t init_thread_;
        if(pthread_create( &init_thread_, NULL, RadioConfig::initBSRadio, (void *)(&context[i])) != 0)
        {
            perror("init thread create failed");
            exit(0);
        }
#else
        RadioConfig::initBSRadio((void *)&context[i]);
#endif
    }

    while(remainingJobs>0);
    for (int i = 0; i < this->_radioNum; i++)
    {
        std::cout << _cfg->radio_ids.at(i) << ": Front end " << baStn[i]->getHardwareInfo()["frontend"] << std::endl;
        for (int c = 0; c < _cfg->nChannels; c++) {
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

        for (int c = 0; c < _cfg->nChannels; c++) {
            if (c < baStn[i]->getNumChannels(SOAPY_SDR_TX)) {
                printf("TX Channel %zu\n", c);
                printf("Actual TX sample rate: %fMSps...\n", (baStn[i]->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
                printf("Actual TX frequency: %fGHz...\n", (baStn[i]->getFrequency(SOAPY_SDR_TX, c) / 1e9));
                printf("Actual TX gain: %f...\n", (baStn[i]->getGain(SOAPY_SDR_TX, c)));
                printf("Actual TX PAD gain: %f...\n", ( baStn[i]->getGain(SOAPY_SDR_TX, c, "PAD")));
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

void *RadioConfig::initBSRadio(void *in_context)
{
    RadioConfig* rc = ((RadioConfigContext *)in_context)->ptr;
    int i = ((RadioConfigContext *)in_context)->tid;
    Config *cfg = rc->_cfg;

    //load channels
    std::vector<size_t> channels;
    if (cfg->nChannels == 1) channels = {0};
    else if (cfg->nChannels == 2) channels = {0, 1};
    else
    {
        std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
        cfg->nChannels = 2;
        channels = {0, 1};
    }

    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["driver"] = "iris";
    args["timeout"] = "1000000";
    args["serial"] = cfg->radio_ids.at(i);
    rc->baStn[i] = SoapySDR::Device::make(args);

    //use the TRX antenna port for both tx and rx
    for (auto ch : channels) rc->baStn[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");

    SoapySDR::Kwargs info = rc->baStn[i]->getHardwareInfo();
    for (auto ch : {0, 1})
    {
        rc->baStn[i]->setBandwidth(SOAPY_SDR_RX, ch, (1+2*cfg->bbf_ratio)*cfg->rate);
        rc->baStn[i]->setBandwidth(SOAPY_SDR_TX, ch, (1+2*cfg->bbf_ratio)*cfg->rate);

        rc->baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        rc->baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

        rc->baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg->freq-cfg->bbf_ratio*cfg->rate);
        rc->baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "BB", cfg->bbf_ratio*cfg->rate);
        rc->baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg->freq-cfg->bbf_ratio*cfg->rate);
        rc->baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "BB", cfg->bbf_ratio*cfg->rate);
 
        if (info["frontend"].find("CBRS") != std::string::npos)
        {
            if (cfg->freq > 3e9) 
                rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", 0); //[-18,0]
            else if (cfg->freq > 2e9 && cfg->freq < 3e9)
                rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -18); //[-18,0]
            else
                rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
            rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
        }

        rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA", ch ? cfg->rxgainB : cfg->rxgainA);  //[0,30]
        rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0);  //[0,12]
        rc->baStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0);  //[-12,19]

        if (info["frontend"].find("CBRS") != std::string::npos)
        {
            rc->baStn[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", -6);  //[-18,0] by 3
            rc->baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0);   //[0|15]
        }
        rc->baStn[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);     //[0,12] 
        rc->baStn[i]->setGain(SOAPY_SDR_TX, ch, "PAD", ch ? cfg->txgainB : cfg->txgainA);  //[0,30]

    }

    for (auto ch : channels)
    {
        //baStn[i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
        //baStn[i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
        rc->baStn[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    // we disable channel 1 because of the internal LDO issue.
    // This will be fixed in the next revision (E) of Iris.
    if (cfg->nChannels == 1)
    {
        if (cfg->freq > 3e9 and cfg->radio_ids[i].find("RF3E") == std::string::npos)
        {
            std::cout << "setting up SPI_TDD" << std::endl;
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = rc->baStn[i]->readRegister("LMS7IC", 0x0020);
            rc->baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            unsigned regRfeA = rc->baStn[i]->readRegister("LMS7IC", 0x010C);
            unsigned regRfeALo = rc->baStn[i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = rc->baStn[i]->readRegister("LMS7IC", 0x0115);
            unsigned regTrfA = rc->baStn[i]->readRegister("LMS7IC", 0x0100);
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

            rc->baStn[i]->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
            rc->baStn[i]->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
        }
    } 
    else if (cfg->nChannels == 2)
    {
        if (cfg->freq > 3e9 and cfg->radio_ids[i].find("RF3E") == std::string::npos)
        {
            std::vector<unsigned> txActive, rxActive;
            unsigned ch = rc->baStn[i]->readRegister("LMS7IC", 0x0020);
            rc->baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 1);
            unsigned regRfeA = rc->baStn[i]->readRegister("LMS7IC", 0x010C);
            unsigned regRfeALo = rc->baStn[i]->readRegister("LMS7IC", 0x010D);
            unsigned regRbbA = rc->baStn[i]->readRegister("LMS7IC", 0x0115);
            unsigned regTrfA = rc->baStn[i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbA = rc->baStn[i]->readRegister("LMS7IC", 0x0105);

            ch = rc->baStn[i]->readRegister("LMS7IC", 0x0020);
            rc->baStn[i]->writeRegister("LMS7IC", 0x0020, (ch & 0xFFFC) | 2);
            unsigned regRfeB = rc->baStn[i]->readRegister("LMS7IC", 0x010C);
            unsigned regRbbB = rc->baStn[i]->readRegister("LMS7IC", 0x0115);
            unsigned regTrfB = rc->baStn[i]->readRegister("LMS7IC", 0x0100);
            unsigned regTbbB = rc->baStn[i]->readRegister("LMS7IC", 0x0105);

            txActive = {
                //0xe10C0000 | 0xfe, //RFE in power down
                //0xe10D0000 | 0x0, //RFE SISO and disables
                0xe1150000 | 0xe, //RBB in power down
                //0xe1000000 | regTrfA, //TRF stays the same
                0xe1050000 | regTbbA}; //TBB stays the same

            rxActive = {
                //0xe10C0000 | regRfeA, //RFE stays the same
                //0xe10D0000 | regRfeALo, //RFE stays the same
                0xe1150000 | regRbbA, //RBB stays the same
                //0xe1000000 | 0xe, //TRF in power down + SISO
                0xe1050000 | 0x1e}; //TBB in power down

            rc->baStn[i]->writeRegisters("LMS7_PROG_SPI", 16, txActive); //trig1 offset
            rc->baStn[i]->writeRegisters("LMS7_PROG_SPI", 32, rxActive); //trig2 offset
            //baStn[i]->writeSetting("SPI_TDD_MODE", "MIMO");
        }
    }
    // resets the DATA_clk domain logic. 
    rc->baStn[i]->writeRegister("IRIS30", 48, (1<<29) | 0x1);
    rc->baStn[i]->writeRegister("IRIS30", 48, (1<<29));
    rc->baStn[i]->writeRegister("IRIS30", 48, 0);

    rc->rxStreams[i] = rc->baStn[i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    rc->txStreams[i] = rc->baStn[i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CF32, channels, sargs);
    rc->remainingJobs--;

}

void RadioConfig::radioStart()
{
    long long frameTime(0);
    // send through the first radio for now
    //int beacon_ant = 1;
    int flags(0); // = SOAPY_SDR_WAIT_TRIGGER;
    std::vector<unsigned> zeros(_cfg->sampsPerSymbol,0);
    std::vector<uint32_t> beacon = _cfg->beacon; 
    std::vector<unsigned> beacon_weights(_cfg->nAntennas); 

    std::vector<uint32_t> pilot = _cfg->pilot; 

    std::vector<std::string> _tddSched;
    if (isUE)
    {
        _tddSched.resize(this->_radioNum);
        for (int r = 0; r < _radioNum; r++)
        {
            _tddSched[r] = _cfg->frames[0];
            for (int s =0; s < _cfg->frames[0].size(); s++)
            {
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
        int ueTrigOffset = _cfg->prefix + _cfg->beacon_len + _cfg->postfix + 17 + _cfg->prefix;
        int sf_start = ueTrigOffset/_cfg->sampsPerSymbol;
        int sp_start = ueTrigOffset%_cfg->sampsPerSymbol;
        for (int i = 0; i < this->_radioNum; i++)
        {
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
            if (_cfg->nChannels == 2) baStn[i]->writeRegisters("TX_RAM_B", 2048, pilot);
            baStn[i]->activateStream(this->rxStreams[i], flags, 0);
            baStn[i]->activateStream(this->txStreams[i]);

            baStn[i]->writeRegister("IRIS30", CORR_CONF, 0x1);
            for (int j = 0; j < _cfg->coeffs.size(); j++)
                baStn[i]->writeRegister("ARGCOE", j*4, 0);
            usleep(100000);
            baStn[i]->writeRegister("IRIS30", 64, 1); // reset faros_corr
            baStn[i]->writeRegister("IRIS30", 64, 0); // unreset faros_corr
            baStn[i]->writeRegister("IRIS30", 92, 1); // threshold is left-shifted by this many bits
            for (int j = 0; j < _cfg->coeffs.size(); j++)
                baStn[i]->writeRegister("ARGCOE", j*4, _cfg->coeffs[j]);
            baStn[i]->setHardwareTime(SoapySDR::ticksToTimeNs((sf_start << 16) | sp_start, _cfg->rate), "TRIGGER");

            baStn[i]->writeRegister("IRIS30", CORR_CONF, 0x11);
        }
    }
    else
    {
        drain_buffers();
        _tddSched.resize(_cfg->framePeriod);
        for (int f = 0; f < _cfg->framePeriod; f++)
        {
            _tddSched[f] = _cfg->frames[f];
            for (int s =0; s < _cfg->frames[f].size(); s++)
            {
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

        for (int i = 0; i < this->_radioNum; i++)
        {
            baStn[i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
            baStn[i]->writeSetting("TDD_MODE", "true");
            baStn[i]->writeSetting("TDD_CONFIG", confString);

            if (_cfg->beacon_mode == "single" or _cfg->nAntennas == 1)
            {
                if (i*_cfg->nChannels == _cfg->beacon_ant)
                    baStn[i]->writeRegisters("TX_RAM_A", 0, beacon);
                else if (_cfg->nChannels == 2 and i*2+1 == _cfg->beacon_ant)
                    baStn[i]->writeRegisters("TX_RAM_B", 0, beacon);
                else 
                {
                    baStn[i]->writeRegisters("TX_RAM_A", 0, zeros);
                    baStn[i]->writeRegisters("TX_RAM_B", 0, zeros);
                }
            } 
            else // beamsweep
            {
                baStn[i]->writeRegisters("TX_RAM_A", 0, beacon);
                if (_cfg->nChannels == 2)
                    baStn[i]->writeRegisters("TX_RAM_B", 0, beacon);
                int residue = int(pow(2,ceil(log2(_cfg->nAntennas))))-_cfg->nAntennas;
                beacon_weights.assign(_cfg->beacon_weights.at(i*_cfg->nChannels).begin(), _cfg->beacon_weights.at(i*_cfg->nChannels).begin()+_cfg->nAntennas);
                baStn[i]->writeRegisters("TX_RAM_WGT_A", 0, beacon_weights);
                if (_cfg->nChannels == 2)
                {
                    beacon_weights.assign(_cfg->beacon_weights[i*_cfg->nChannels+1].begin(), _cfg->beacon_weights[i*_cfg->nChannels+1].begin()+_cfg->nAntennas);
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
}

void RadioConfig::go()
{
    if (hubs.size() == 0)
    {
        //std::cout << "triggering first Iris ..." << std::endl;
        baStn[0]->writeSetting("TRIGGER_GEN", "");
    }
    else
    {
        //std::cout << "triggering Hub ..." << std::endl;
        hubs[0]->writeSetting("TRIGGER_GEN", "");
    }
}

void RadioConfig::radioTx(void ** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->writeStream(this->txStreams[i], buffs, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

void RadioConfig::radioTx(int r /*radio id*/, void ** buffs, int flags, long long & frameTime)
{
    int txFlags = 0;
    if (flags == 1) txFlags = SOAPY_SDR_HAS_TIME;
    else if (flags == 2) txFlags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
    //long long frameTime(0);
    int w = baStn[r]->writeStream(this->txStreams[r], buffs, _cfg->sampsPerSymbol, txFlags, frameTime, 1000000);
#ifdef DEBUG_RADIO
    size_t chanMask;
    long timeoutUs(0);
    int statusFlag = 0;
    int s = baStn[r]->readStreamStatus(this->txStreams[r], chanMask, statusFlag, frameTime, timeoutUs);
    std::cout << "radio " << r << " tx returned " << w << " and status " << s << " when flags was " << flags << std::endl;
#endif
}

void RadioConfig::radioRx(void ** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < this->_radioNum; i++)
    {
        void **buff = buffs + (i * 2);
        baStn[i]->readStream(this->rxStreams[i], buff, _cfg->sampsPerSymbol, flags, frameTime, 1000000);
    }
}

int RadioConfig::radioRx(int r /*radio id*/, void ** buffs, long long & frameTime)
{
    int flags = 0;
    if (r < this->_radioNum)
    {
        long long frameTimeNs = 0;
        int ret = baStn[r]->readStream(this->rxStreams[r], buffs, _cfg->sampsPerSymbol, flags, frameTimeNs, 1000000);
        frameTime = frameTimeNs; //SoapySDR::timeNsToTicks(frameTimeNs, _rate);
        if (ret != _cfg->sampsPerSymbol)
            std::cout << "invalid return " << ret << " from radio " << r <<std::endl;
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
    std::vector<std::complex<int16_t>> dummyBuff0(_cfg->sampsPerSymbol);
    std::vector<std::complex<int16_t>> dummyBuff1(_cfg->sampsPerSymbol);
    std::vector<void *> dummybuffs(2);
    dummybuffs[0] = dummyBuff0.data();
    dummybuffs[1] = dummyBuff1.data();
    for (int i = 0; i < _cfg->nRadios; i++)
    {
        RadioConfig::drain_rx_buffer(baStn[i], rxStreams[i], dummybuffs, _cfg->sampsPerSymbol);
    }
}

void RadioConfig::drain_rx_buffer(SoapySDR::Device * ibsSdrs, SoapySDR::Stream * istream, std::vector<void *> buffs, int symSamp) {
    long long frameTime=0;
    int flags=0, r=0, i=0;
    long timeoutUs(0);
    while (r != -1){
        r = ibsSdrs->readStream(istream, buffs.data(), symSamp, flags, frameTime, timeoutUs);
        i++;
    }
    //std::cout << "Number of reads needed to drain: " << i << std::endl;
}

void RadioConfig::adjustDelays(std::vector<int> offset, int ref_ant)
{
    int ref_offset = ref_ant == 0 ? 1 : 0;
    int R = _cfg->nRadios;
    for (int i = 0; i < R; i++)
    {
        int delta = offset[ref_offset] - offset[i];
        std::cout << "adjusting delay of node " << i << " by " << delta << std::endl;
        int iter = delta < 0 ? -delta : delta;
        for (int j = 0; j < iter; j++)
        {
            if (delta < 0)
                baStn[i]->writeSetting("ADJUST_DELAYS", "-1");
            else
                baStn[i]->writeSetting("ADJUST_DELAYS", "1");
        }
    }
}

std::vector<std::vector<std::complex<int16_t>>> RadioConfig::collectCSI(bool &adjust)
{

    std::vector<std::vector<double>> pilot;
    std::vector<std::complex<float>> pilot_cf32;
    std::vector<std::complex<int16_t>> pilot_cint16;
    int type = CommsLib::LTS_SEQ;
    int seqLen = 160;  // Sequence length
    pilot = CommsLib::getSequence(seqLen, type);
    // double array to complex 32-bit float vector
    double max_abs = 0;
    for (int i=0; i < seqLen; i++) {
        std::complex<double> samp(pilot[0][i], pilot[1][i]); 
        max_abs = max_abs > std::abs(samp) ? max_abs : std::abs(samp);
    }

    for (int i=0; i < seqLen; i++) {
        pilot[0][i] = 0.25*pilot[0][i]/max_abs;
        pilot[1][i] = 0.25*pilot[1][i]/max_abs;
        pilot_cint16.push_back(std::complex<int16_t>((int16_t)(pilot[0][i]*32767),(int16_t)(pilot[1][i]*32767)));
        pilot_cf32.push_back(std::complex<float>(pilot[0][i],pilot[1][i]));
    }

    // Prepend/Append vectors with prefix/postfix number of null samples
    std::vector<std::complex<int16_t>> prefix_vec(_cfg->prefix, 0);
    std::vector<std::complex<int16_t>> postfix_vec(_cfg->sampsPerSymbol - _cfg->prefix - seqLen , 0);
    pilot_cint16.insert(pilot_cint16.begin(), prefix_vec.begin(), prefix_vec.end());
    pilot_cint16.insert(pilot_cint16.end(), postfix_vec.begin(), postfix_vec.end());
    // Transmitting from only one chain, create a null vector for chainB
    std::vector<std::complex<int16_t>> dummy_cint16(pilot_cint16.size(), 0);

    std::vector<void *> txbuff0(2);
    txbuff0[0] = pilot_cint16.data(); //pilot_cf32.data();
    txbuff0[1] = dummy_cint16.data(); //dummy_cf32.data();

    std::vector<void *> txbuff1(2);
    txbuff1[0] = dummy_cint16.data(); //dummy_cf32.data();
    txbuff1[1] = pilot_cint16.data(); //pilot_cf32.data();

    // Prepend/Append vectors with prefix/postfix number of null samples
    std::vector<std::complex<float>> prefix_vcf32(_cfg->prefix, 0);
    std::vector<std::complex<float>> postfix_vcf32(_cfg->sampsPerSymbol - _cfg->prefix - seqLen , 0);
    pilot_cf32.insert(pilot_cf32.begin(), prefix_vcf32.begin(), prefix_vcf32.end());
    pilot_cf32.insert(pilot_cf32.end(), postfix_vcf32.begin(), postfix_vcf32.end());
    // Transmitting from only one chain, create a null vector for chainB
    std::vector<std::complex<float>> dummy_cf32(pilot_cf32.size(), 0);

    std::vector<void *> txbuff0_cf32(2);
    txbuff0_cf32[0] = pilot_cf32.data();
    txbuff0_cf32[1] = dummy_cf32.data();

    std::vector<void *> txbuff1_cf32(2);
    txbuff1_cf32[0] = dummy_cf32.data();
    txbuff1_cf32[1] = pilot_cf32.data();

    std::vector<std::vector<std::complex<int16_t>>> buff;
    //std::vector<std::vector<std::complex<int16_t>>> buff;
    int ant = _cfg->nChannels;
    int M = _cfg->nAntennas;
    int R = _cfg->nRadios;
    assert(M == R); // for now
    buff.resize(M * M);
    for (int i = 0; i < M; i++)
    {
        for (int j = 0; j < M; j++)
        {
            if (i == j) buff[i*M+j] = pilot_cint16;
            else buff[i*M+j].resize(_cfg->sampsPerSymbol);
        }
    }

    std::vector<std::complex<int16_t>> dummyBuff0(_cfg->sampsPerSymbol);
    std::vector<std::complex<int16_t>> dummyBuff1(_cfg->sampsPerSymbol);
    //std::vector<std::complex<float>> dummyBuff0(_cfg->sampsPerSymbol);
    //std::vector<std::complex<float>> dummyBuff1(_cfg->sampsPerSymbol);
    drain_buffers();

    for (int i = 0; i < R; i++)
    {
        baStn[i]->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->calTxGainA);
        if (_cfg->nChannels == 2)
            baStn[i]->setGain(SOAPY_SDR_TX, 1, "PAD", _cfg->calTxGainB);
        baStn[i]->writeSetting("TDD_CONFIG", "{\"tdd_enabled\":false}");
        baStn[i]->writeSetting("TDD_MODE", "false");
        baStn[i]->activateStream(this->txStreams[i]);
    }

    long long txTime(0);
    long long rxTime(0);
    for (int i = 0; i < R; i++)
    {
        auto ref_sdr = baStn[i];
        int tx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
        int ret = ref_sdr->writeStream(this->txStreams[i], txbuff0_cf32.data(), _cfg->sampsPerSymbol, tx_flags, txTime, 1000000);
        if (ret < 0) std::cout << "bad write\n";
        for (int j = 0; j < R; j++)
        {
            if (j == i)
                continue;
            int rx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
            ret = baStn[j]->activateStream(this->rxStreams[j], rx_flags, rxTime, _cfg->sampsPerSymbol);
        }

        go();

        int flags = 0;
        for (int j = 0; j < R; j++)
        {
            if (j == i)
                continue;
            std::vector<void *> rxbuff(2);
            rxbuff[0] = buff[(i*R+j)].data();
            //rxbuff[1] = ant == 2 ? buff[(i*M+j)*ant+1].data() : dummyBuff.data();
            rxbuff[1] = dummyBuff0.data();
            ret = baStn[j]->readStream(this->rxStreams[j], rxbuff.data(), _cfg->sampsPerSymbol, flags, rxTime, 1000000);
            if (ret < 0) std::cout << "bad read at node " << j << std::endl;
        }
    }

    int ref_ant = 0;
    int ref_offset = ref_ant == 0 ? 1 : 0;
    std::vector<int> offset(R);

    bool good_csi = true;
    for (int i = 0; i < R; i++)
    {
        std::vector<std::complex<double>> rx(_cfg->sampsPerSymbol);
        if (i == ref_ant)
        {
            std::transform( buff[ref_offset*R+i].begin(), buff[ref_offset*R+i].end(), rx.begin(), []( std::complex<int16_t> cf ) {
                return std::complex<double>( cf.real()/32768.0, cf.imag()/32768.0 ); });
        }
        else
        {
            std::transform( buff[ref_ant*R+i].begin(), buff[ref_ant*R+i].end(), rx.begin(), []( std::complex<int16_t> cf ) {
                return std::complex<double>( cf.real()/32768.0, cf.imag()/32768.0 ); });
        }

        int peak = CommsLib::findLTS(rx, seqLen);
        offset[i] = peak < 128 ? 0 : peak - 128;
        std::cout << i << " " << offset[i] << std::endl;
        if (offset[i] == 0) good_csi = false;

    }

    // adjusting trigger delays based on lts peak index
    if (adjust)
    {
        if (good_csi) adjustDelays(offset, ref_offset);
        else adjust = false;
    }

    for (int i = 0; i < R; i++)
    {
        //baStn[i]->deactivateStream(this->txStreams[i]);
        //baStn[i]->deactivateStream(this->rxStreams[i]);
        baStn[i]->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->txgainA);  //[0,30]
        if (_cfg->nChannels == 2)
            baStn[i]->setGain(SOAPY_SDR_TX, 1, "PAD", _cfg->txgainB);  //[0,30]
    }

    return buff;
}

void RadioConfig::readSensors()
{
    for (int i = 0; i < this->_radioNum; i++)
    {
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
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->deactivateStream(this->rxStreams[i]);
        baStn[i]->deactivateStream(this->txStreams[i]);
        baStn[i]->writeSetting("TDD_MODE", "false");
        // write schedule
        for(int k = 0; k < schedLen; k++) // symnum <= 256
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
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->closeStream(this->rxStreams[i]);
        baStn[i]->closeStream(this->txStreams[i]);
        SoapySDR::Device::unmake(baStn[i]);
    }
}

