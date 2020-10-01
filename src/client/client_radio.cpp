#include "client_radio.hpp"
#include "comms-lib.h"

std::atomic<size_t> num_client_radios_initialized;

ClientRadioConfig::ClientRadioConfig(Config* cfg)
    : _cfg(cfg)
{
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    // load channels
    auto channels = Utils::strToChannels(_cfg->channel);

    this->_radioNum = _cfg->nRadios;
    this->_antennaNum = _radioNum * _cfg->nChannels;
    std::cout << "radio num is " << this->_radioNum << std::endl;

    clStn.resize(_radioNum);
    txStreams.resize(_radioNum);
    rxStreams.resize(_radioNum);

    std::vector<ClientRadioConfigContext> client_radio_config_ctx(
        this->_radioNum);
    for (size_t i = 0; i < this->_radioNum; i++) {
        auto* context = &client_radio_config_ctx[i];
        context->ptr = this;
        context->tid = i;
#ifdef THREADED_INIT
        pthread_t init_thread_;
        if (pthread_create(&init_thread_, NULL, initClientRadio_launch, context)
            != 0) {
            perror("init thread create failed");
            exit(0);
        }
#else
        initClientRadio(context);
#endif
    }

#ifdef THREADED_INIT
    size_t num_checks = 0;
    while (num_client_radios_initialized != this->_radioNum) {
        size_t _num_client_radios_initialized = num_client_radios_initialized;
        num_checks++;
        if (num_checks > 1e9) {
            printf("RadioConfig: Waiting for radio initialization, %zu of %zu "
                   "ready\n",
                _num_client_radios_initialized, this->_radioNum);
            num_checks = 0;
        }
    }
#endif

    for (size_t i = 0; i < this->_radioNum; i++) {
        std::cout << _cfg->radio_ids.at(i) << ": Front end "
                  << clStn[i]->getHardwareInfo()["frontend"] << std::endl;
        for (auto c : channels) {
            if (c < clStn[i]->getNumChannels(SOAPY_SDR_RX)) {
                printf("RX Channel %zu\n", c);
                printf("Actual RX sample rate: %fMSps...\n",
                    (clStn[i]->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
                printf("Actual RX frequency: %fGHz...\n",
                    (clStn[i]->getFrequency(SOAPY_SDR_RX, c) / 1e9));
                printf("Actual RX gain: %f...\n",
                    (clStn[i]->getGain(SOAPY_SDR_RX, c)));
                if (!kUseUHD) {
                    printf("Actual RX LNA gain: %f...\n",
                        (clStn[i]->getGain(SOAPY_SDR_RX, c, "LNA")));
                    printf("Actual RX PGA gain: %f...\n",
                        (clStn[i]->getGain(SOAPY_SDR_RX, c, "PGA")));
                    printf("Actual RX TIA gain: %f...\n",
                        (clStn[i]->getGain(SOAPY_SDR_RX, c, "TIA")));
                    if (clStn[i]->getHardwareInfo()["frontend"].compare("CBRS")
                        == 0) {
                        printf("Actual RX LNA1 gain: %f...\n",
                            (clStn[i]->getGain(SOAPY_SDR_RX, c, "LNA1")));
                        printf("Actual RX LNA2 gain: %f...\n",
                            (clStn[i]->getGain(SOAPY_SDR_RX, c, "LNA2")));
                    }
                }
                printf("Actual RX bandwidth: %fM...\n",
                    (clStn[i]->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
                printf("Actual RX antenna: %s...\n",
                    (clStn[i]->getAntenna(SOAPY_SDR_RX, c).c_str()));
            }
        }

        for (auto c : channels) {
            if (c < clStn[i]->getNumChannels(SOAPY_SDR_TX)) {
                printf("TX Channel %zu\n", c);
                printf("Actual TX sample rate: %fMSps...\n",
                    (clStn[i]->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
                printf("Actual TX frequency: %fGHz...\n",
                    (clStn[i]->getFrequency(SOAPY_SDR_TX, c) / 1e9));
                printf("Actual TX gain: %f...\n",
                    (clStn[i]->getGain(SOAPY_SDR_TX, c)));
                if (!kUseUHD) {
                    printf("Actual TX PAD gain: %f...\n",
                        (clStn[i]->getGain(SOAPY_SDR_TX, c, "PAD")));
                    printf("Actual TX IAMP gain: %f...\n",
                        (clStn[i]->getGain(SOAPY_SDR_TX, c, "IAMP")));
                    if (clStn[i]->getHardwareInfo()["frontend"].compare("CBRS")
                        == 0) {
                        printf("Actual TX PA1 gain: %f...\n",
                            (clStn[i]->getGain(SOAPY_SDR_TX, c, "PA1")));
                        printf("Actual TX PA2 gain: %f...\n",
                            (clStn[i]->getGain(SOAPY_SDR_TX, c, "PA2")));
                        printf("Actual TX PA3 gain: %f...\n",
                            (clStn[i]->getGain(SOAPY_SDR_TX, c, "PA3")));
                    }
                }
                printf("Actual TX bandwidth: %fM...\n",
                    (clStn[i]->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
                printf("Actual TX antenna: %s...\n",
                    (clStn[i]->getAntenna(SOAPY_SDR_TX, c).c_str()));
            }
        }
        std::cout << std::endl;
    }

    std::cout << "radio init done!" << std::endl;
}

void* ClientRadioConfig::initClientRadio_launch(void* in_context)
{
    auto* context = (ClientRadioConfigContext*)in_context;
    context->ptr->initClientRadio(context);
    return 0;
}

void ClientRadioConfig::initClientRadio(ClientRadioConfigContext* in_context)
{
    size_t i = in_context->tid;
    Config* cfg = _cfg;

    // load channels
    auto channels = Utils::strToChannels(cfg->channel);

    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["timeout"] = "1000000";
    if (!kUseUHD) {
        args["driver"] = "iris";
        args["serial"] = cfg->radio_ids.at(i);
    } else {
        args["driver"] = "uhd";
        args["addr"] = cfg->radio_ids.at(i);
    }
    clStn[i] = SoapySDR::Device::make(args);
    for (auto ch : channels) {
        clStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        clStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);
    }
    rxStreams[i]
        = clStn[i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    txStreams[i]
        = clStn[i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);

    // resets the DATA_clk domain logic.
    if (!kUseUHD)
        clStn[i]->writeSetting("RESET_DATA_LOGIC", "");

    // use the TRX antenna port for both tx and rx
    for (auto ch : channels)
        if (!kUseUHD)
            clStn[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");
        else {
            clStn[i]->setAntenna(SOAPY_SDR_RX, ch, "RX2");
            clStn[i]->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
        }

    SoapySDR::Kwargs info = clStn[i]->getHardwareInfo();
    for (auto ch : channels) {
        clStn[i]->setBandwidth(SOAPY_SDR_RX, ch, cfg->bwFilter);
        clStn[i]->setBandwidth(SOAPY_SDR_TX, ch, cfg->bwFilter);

        // clStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        // clStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

        clStn[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg->radioRfFreq);
        clStn[i]->setFrequency(SOAPY_SDR_RX, ch, "BB", kUseUHD ? 0 : cfg->nco);
        clStn[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg->radioRfFreq);
        clStn[i]->setFrequency(SOAPY_SDR_TX, ch, "BB", kUseUHD ? 0 : cfg->nco);

        if (!kUseUHD) {
            if (info["frontend"].find("CBRS") != std::string::npos) {
                if (cfg->freq > 3e9)
                    clStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -6); //[-18,0]
                else if (cfg->freq > 2e9 && cfg->freq < 3e9)
                    clStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -18); //[-18,0]
                else
                    clStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
                clStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
            }

            clStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA",
                ch ? cfg->rxgainB : cfg->rxgainA); //[0,30]
            clStn[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
            clStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]

            if (info["frontend"].find("CBRS") != std::string::npos) {
                clStn[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
                clStn[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|15]
            }
            clStn[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0); //[0,12]
            clStn[i]->setGain(SOAPY_SDR_TX, ch, "PAD",
                ch ? cfg->txgainB : cfg->txgainA); //[0,30]
        } else {
            clStn[i]->setGain(
                SOAPY_SDR_RX, ch, "PGA0", ch ? cfg->rxgainB : cfg->rxgainA);
            clStn[i]->setGain(
                SOAPY_SDR_TX, ch, "PGA0", ch ? cfg->txgainB : cfg->txgainA);
        }
    }

    for (auto ch : channels) {
        // clStn[i]->writeSetting(SOAPY_SDR_RX, ch, "CALIBRATE", "SKLK");
        // clStn[i]->writeSetting(SOAPY_SDR_TX, ch, "CALIBRATE", "");
        if (!kUseUHD)
            clStn[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    num_client_radios_initialized++;
}

bool ClientRadioConfig::radioStart()
{
    // send through the first radio for now
    // int beacon_ant = 1;
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
            if (c == 'P'
                and ((_cfg->nChannels == 1 and _cfg->pilotSymbols[0][r] != s)
                        or (_cfg->nChannels == 2
                               and (_cfg->pilotSymbols[0][2 * r] != s
                                       and _cfg->pilotSymbols[0][r * 2 + 1]
                                           != s)))) // TODO: change this for
                // orthogonal pilots
                _tddSched[r].replace(s, 1, "G");
            else if (c == 'U')
                _tddSched[r].replace(s, 1, "T");
            else if (c == 'D')
                _tddSched[r].replace(s, 1, "R");
            else if (c != 'P')
                _tddSched[r].replace(s, 1, "G");
        }
        std::cout << _tddSched[r] << std::endl;
    }
    // beaconSize + 82 (BS FE delay) + 68 (path delay) + 17 (correlator delay) +
    // 82 (Client FE Delay)
    int clTrigOffset = _cfg->beacon_len + 249;
    int sf_start = clTrigOffset / _cfg->sampsPerSymbol;
    int sp_start = clTrigOffset % _cfg->sampsPerSymbol;
    for (size_t i = 0; i < this->_radioNum; i++) {
        if (_cfg->hw_framer) {
            json conf;
            conf["tdd_enabled"] = true;
            conf["frame_mode"] = "continuous_resync";
            int max_frame_ = (int)(2.0
                / ((_cfg->sampsPerSymbol * _cfg->symbol_num_perframe)
                      / _cfg->rate));
            conf["max_frame"] = max_frame_;
            conf["dual_pilot"] = (_cfg->nChannels == 2);
            std::vector<std::string> jframes;
            jframes.push_back(_tddSched[i]);
            conf["frames"] = jframes;
            conf["symbol_size"] = _cfg->sampsPerSymbol;
            std::string confString = conf.dump();
            clStn[i]->writeSetting("TDD_CONFIG", confString);
            clStn[i]->setHardwareTime(
                SoapySDR::ticksToTimeNs(
                    (sf_start << 16) | sp_start, _cfg->rate),
                "TRIGGER");
            clStn[i]->writeSetting("TX_SW_DELAY",
                "30"); // experimentally good value for dev front-end
            clStn[i]->writeSetting("TDD_MODE", "true");
            for (char const& c : _cfg->channel) {
                std::string tx_ram = "TX_RAM_";
                clStn[i]->writeRegisters(tx_ram + c, 0, pilot);
            }
            clStn[i]->activateStream(this->rxStreams[i], flags, 0);
            clStn[i]->activateStream(this->txStreams[i]);

            std::string corrConfString
                = "{\"corr_enabled\":true,\"corr_threshold\":"
                + std::to_string(1) + "}";
            clStn[i]->writeSetting("CORR_CONFIG", corrConfString);
            clStn[i]->writeRegisters("CORR_COE", 0, _cfg->coeffs);

            clStn[i]->writeSetting(
                "CORR_START", (_cfg->channel == "B") ? "B" : "A");
        } else {
            if (!kUseUHD) {
                clStn[i]->setHardwareTime(0, "TRIGGER");
                clStn[i]->activateStream(this->rxStreams[i], flags, 0);
                clStn[i]->activateStream(this->txStreams[i]);
                clStn[i]->writeSetting("TRIGGER_GEN", "");
            } else {
                clStn[i]->setHardwareTime(0, "UNKNOWN_PPS");
                clStn[i]->activateStream(
                    this->rxStreams[i], SOAPY_SDR_HAS_TIME, 1e9, 0);
                clStn[i]->activateStream(
                    this->txStreams[i], SOAPY_SDR_HAS_TIME, 1e9, 0);
            }
        }
    }

    std::cout << "radio start done!" << std::endl;
    return true;
}

void ClientRadioConfig::go()
{
    if (!kUseUHD) {
        if (hubs.size() == 0) {
            // std::cout << "triggering first Iris ..." << std::endl;
            clStn[0]->writeSetting("TRIGGER_GEN", "");
        } else {
            // std::cout << "triggering Hub ..." << std::endl;
            hubs[0]->writeSetting("TRIGGER_GEN", "");
        }
    }
}

int ClientRadioConfig::radioTx(size_t r /*radio id*/, void** buffs,
    size_t num_samps, int flags, long long& frameTime)
{
    int txFlags = 0;
    if (flags == 1)
        txFlags = SOAPY_SDR_HAS_TIME;
    else if (flags == 2)
        txFlags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
    int w(0);
    if (_cfg->hw_framer) {
        w = clStn[r]->writeStream(
            this->txStreams[r], buffs, num_samps, txFlags, frameTime, 1000000);
    } else {
        long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate);
        w = clStn[r]->writeStream(this->txStreams[r], buffs, num_samps, txFlags,
            frameTimeNs, 1000000);
    }
    if (kDebugRadioTX) {
        size_t chanMask;
        long timeoutUs(0);
        int statusFlag = 0;
        int s = clStn[r]->readStreamStatus(
            this->txStreams[r], chanMask, statusFlag, frameTime, timeoutUs);
        std::cout << "radio " << r << " tx returned " << w << " and status "
                  << s << " when flags was " << flags << std::endl;
    }
    return w;
}

int ClientRadioConfig::radioRx(
    size_t r /*radio id*/, void** buffs, size_t num_samps, long long& frameTime)
{
    int flags(0);
    if (r < this->_radioNum) {
        int ret(0);
        if (_cfg->hw_framer) {
            ret = clStn[r]->readStream(this->rxStreams[r], buffs, num_samps,
                flags, frameTime, 1000000);
        } else {
            long long frameTimeNs = 0;
            ret = clStn[r]->readStream(this->rxStreams[r], buffs, num_samps,
                flags, frameTimeNs, 1000000);
            frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate);
        }
        if (kDebugRadioRX) {
            if (ret != (int)num_samps)
                std::cout << "invalid return " << ret << " from radio " << r
                          << std::endl;
            else
                std::cout << "radio " << r << "received " << ret << std::endl;
        }
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
        ClientRadioConfig::drain_rx_buffer(
            clStn[i], rxStreams[i], dummybuffs, _cfg->sampsPerSymbol);
    }
}

void ClientRadioConfig::drain_rx_buffer(SoapySDR::Device* dev,
    SoapySDR::Stream* istream, std::vector<void*> buffs, size_t symSamp)
{
    long long frameTime = 0;
    int flags = 0, r = 0, i = 0;
    long timeoutUs(0);
    while (r != -1) {
        r = dev->readStream(
            istream, buffs.data(), symSamp, flags, frameTime, timeoutUs);
        i++;
    }
    // std::cout << "Number of reads needed to drain: " << i << std::endl;
}

int ClientRadioConfig::triggers(int i)
{
    return std::stoi(clStn[i]->readSetting("TRIGGER_COUNT"));
}

void ClientRadioConfig::readSensors()
{
    for (size_t i = 0; i < this->_radioNum; i++) {
        std::cout << "TEMPs on Iris " << i << std::endl;
        std::cout << "ZYNQ_TEMP: " << clStn[i]->readSensor("ZYNQ_TEMP")
                  << std::endl;
        std::cout << "LMS7_TEMP  : " << clStn[i]->readSensor("LMS7_TEMP")
                  << std::endl;
        std::cout << "FE_TEMP  : " << clStn[i]->readSensor("FE_TEMP")
                  << std::endl;
        std::cout << "TX0 TEMP  : "
                  << clStn[i]->readSensor(SOAPY_SDR_TX, 0, "TEMP") << std::endl;
        std::cout << "TX1 TEMP  : "
                  << clStn[i]->readSensor(SOAPY_SDR_TX, 1, "TEMP") << std::endl;
        std::cout << "RX0 TEMP  : "
                  << clStn[i]->readSensor(SOAPY_SDR_RX, 0, "TEMP") << std::endl;
        std::cout << "RX1 TEMP  : "
                  << clStn[i]->readSensor(SOAPY_SDR_RX, 1, "TEMP") << std::endl;
        std::cout << std::endl;
    }
}

void ClientRadioConfig::radioStop()
{
    std::string corrConfStr = "{\"corr_enabled\":false}";
    std::string tddConfStr = "{\"tdd_enabled\":false}";
    for (size_t i = 0; i < this->_radioNum; i++) {
        clStn[i]->deactivateStream(this->rxStreams[i]);
        clStn[i]->deactivateStream(this->txStreams[i]);
        if (_cfg->hw_framer) {
            clStn[i]->writeSetting("TDD_MODE", "false");
            clStn[i]->writeSetting("TDD_CONFIG", tddConfStr);
            clStn[i]->writeSetting("CORR_CONFIG", corrConfStr);
        }
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
