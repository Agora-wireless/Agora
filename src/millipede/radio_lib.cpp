#include "radio_lib.hpp"
#include "comms-lib.h"

std::atomic<size_t> num_radios_initialized;
std::atomic<size_t> num_radios_configured;

RadioConfig::RadioConfig(Config* cfg)
    : _cfg(cfg)
{
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    // load channels
    auto channels = Utils::strToChannels(_cfg->channel);

    this->_radioNum = _cfg->nRadios;
    this->_antennaNum = _radioNum * _cfg->nChannels;
    std::cout << "radio num is " << this->_radioNum << std::endl;
    if (_cfg->isUE)
        throw std::invalid_argument("Bad config! Not a UE!");
    if (_cfg->hub_ids.size() != 0) {
        args["driver"] = "remote";
        args["timeout"] = "1000000";
        args["serial"] = _cfg->hub_ids.at(0);
        hubs.push_back(SoapySDR::Device::make(args));
    }

    baStn.resize(_radioNum);
    txStreams.resize(_radioNum);
    rxStreams.resize(_radioNum);

    std::vector<RadioConfigContext> radio_config_ctx_vec(this->_radioNum);
    for (size_t i = 0; i < this->_radioNum; i++) {
        auto* context = &radio_config_ctx_vec[i];
        context->brs = this;
        context->tid = i;
#ifdef THREADED_INIT
        pthread_t init_thread_;
        if (pthread_create(&init_thread_, NULL, initBSRadio_launch, context)
            != 0) {
            perror("init thread create failed");
            exit(0);
        }
#else
        initBSRadio(context);
#endif
    }

    // Block until all radios are initialized
    size_t num_checks = 0;
    while (num_radios_initialized != this->_radioNum) {
        size_t _num_radios_initialized = num_radios_initialized;
        num_checks++;
        if (num_checks > 1e9) {
            printf("RadioConfig: Waiting for radio initialization, %zu of %zu "
                   "ready\n",
                _num_radios_initialized, this->_radioNum);
            num_checks = 0;
        }
    }

    // Perform DC Offset & IQ Imbalance Calibration
    if (_cfg->imbalanceCalEn) {
        if (_cfg->channel.find('A') != std::string::npos)
            dciqCalibrationProc(0);
        if (_cfg->channel.find('B') != std::string::npos)
            dciqCalibrationProc(1);
    }

    for (size_t i = 0; i < this->_radioNum; i++) {
        auto* context = &radio_config_ctx_vec[i];
        context->brs = this;
        context->tid = i;
#ifdef THREADED_INIT
        pthread_t configure_thread_;
        if (pthread_create(&configure_thread_, NULL,
                RadioConfig::configureBSRadio_launch, context)
            != 0) {
            perror("init thread create failed");
            exit(0);
        }
#else
        configureBSRadio(context);
#endif
    }

    // Block until all radios are configured
    while (num_radios_configured != this->_radioNum) {
        size_t _num_radios_configured = num_radios_configured;
        num_checks++;
        if (num_checks > 1e9) {
            printf("RadioConfig: Waiting for radio initialization, %zu of %zu "
                   "ready\n",
                _num_radios_configured, this->_radioNum);
            num_checks = 0;
        }
    }

    for (size_t i = 0; i < this->_radioNum; i++) {
        std::cout << _cfg->radio_ids.at(i) << ": Front end "
                  << baStn[i]->getHardwareInfo()["frontend"] << std::endl;
        for (size_t c = 0; c < _cfg->nChannels; c++) {
            if (c < baStn[i]->getNumChannels(SOAPY_SDR_RX)) {
                printf("RX Channel %zu\n", c);
                printf("Actual RX sample rate: %fMSps...\n",
                    (baStn[i]->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
                printf("Actual RX frequency: %fGHz...\n",
                    (baStn[i]->getFrequency(SOAPY_SDR_RX, c) / 1e9));
                printf("Actual RX gain: %f...\n",
                    (baStn[i]->getGain(SOAPY_SDR_RX, c)));
                printf("Actual RX LNA gain: %f...\n",
                    (baStn[i]->getGain(SOAPY_SDR_RX, c, "LNA")));
                printf("Actual RX PGA gain: %f...\n",
                    (baStn[i]->getGain(SOAPY_SDR_RX, c, "PGA")));
                printf("Actual RX TIA gain: %f...\n",
                    (baStn[i]->getGain(SOAPY_SDR_RX, c, "TIA")));
                if (baStn[i]->getHardwareInfo()["frontend"].compare("CBRS")
                    == 0) {
                    printf("Actual RX LNA1 gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_RX, c, "LNA1")));
                    printf("Actual RX LNA2 gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_RX, c, "LNA2")));
                }
                printf("Actual RX bandwidth: %fM...\n",
                    (baStn[i]->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
                printf("Actual RX antenna: %s...\n",
                    (baStn[i]->getAntenna(SOAPY_SDR_RX, c).c_str()));
            }
        }

        for (size_t c = 0; c < _cfg->nChannels; c++) {
            if (c < baStn[i]->getNumChannels(SOAPY_SDR_TX)) {
                printf("TX Channel %zu\n", c);
                printf("Actual TX sample rate: %fMSps...\n",
                    (baStn[i]->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
                printf("Actual TX frequency: %fGHz...\n",
                    (baStn[i]->getFrequency(SOAPY_SDR_TX, c) / 1e9));
                printf("Actual TX gain: %f...\n",
                    (baStn[i]->getGain(SOAPY_SDR_TX, c)));
                printf("Actual TX PAD gain: %f...\n",
                    (baStn[i]->getGain(SOAPY_SDR_TX, c, "PAD")));
                printf("Actual TX IAMP gain: %f...\n",
                    (baStn[i]->getGain(SOAPY_SDR_TX, c, "IAMP")));
                if (baStn[i]->getHardwareInfo()["frontend"].compare("CBRS")
                    == 0) {
                    printf("Actual TX PA1 gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_TX, c, "PA1")));
                    printf("Actual TX PA2 gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_TX, c, "PA2")));
                    printf("Actual TX PA3 gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_TX, c, "PA3")));
                }
                printf("Actual TX bandwidth: %fM...\n",
                    (baStn[i]->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
                printf("Actual TX antenna: %s...\n",
                    (baStn[i]->getAntenna(SOAPY_SDR_TX, c).c_str()));
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
    auto* context = (RadioConfigContext*)in_context;
    context->brs->initBSRadio(context);
    return 0;
}

void RadioConfig::initBSRadio(RadioConfigContext* context)
{
    size_t i = context->tid;
    auto channels = Utils::strToChannels(_cfg->channel);
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["driver"] = "iris";
    args["timeout"] = "1000000";
    args["serial"] = _cfg->radio_ids.at(i);
    baStn[i] = SoapySDR::Device::make(args);
    for (auto ch : { 0, 1 }) {
        baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, _cfg->rate);
        baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, _cfg->rate);
    }
    rxStreams[i]
        = baStn[i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    txStreams[i]
        = baStn[i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
    num_radios_initialized++;
}

void* RadioConfig::configureBSRadio_launch(void* in_context)
{
    RadioConfigContext* context = ((RadioConfigContext*)in_context);
    RadioConfig* brs = context->brs;
    brs->configureBSRadio(context);
    return 0;
}

void RadioConfig::configureBSRadio(RadioConfigContext* context)
{
    size_t i = context->tid;

    // load channels
    auto channels = Utils::strToChannels(_cfg->channel);

    // resets the DATA_clk domain logic.
    baStn[i]->writeSetting("RESET_DATA_LOGIC", "");

    // use the TRX antenna port for both tx and rx
    for (auto ch : channels)
        baStn[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");

    SoapySDR::Kwargs info = baStn[i]->getHardwareInfo();
    for (auto ch : { 0, 1 }) {
        baStn[i]->setBandwidth(SOAPY_SDR_RX, ch, _cfg->bwFilter);
        baStn[i]->setBandwidth(SOAPY_SDR_TX, ch, _cfg->bwFilter);

        // baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        // baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

        baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->radioRfFreq);
        baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "BB", _cfg->nco);
        baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->radioRfFreq);
        baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "BB", _cfg->nco);

        if (info["frontend"].find("CBRS") != std::string::npos) {
            if (_cfg->freq > 3e9)
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -6); //[-18,0]
            else if (_cfg->freq > 2e9 && _cfg->freq < 3e9)
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -18); //[-18,0]
            else
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
            baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
        }

        baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA",
            ch ? _cfg->rxgainB : _cfg->rxgainA); //[0,30]
        baStn[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
        baStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]

        if (info["frontend"].find("CBRS") != std::string::npos) {
            baStn[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
            baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|15]
        }
        baStn[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0); //[0,12]
        baStn[i]->setGain(SOAPY_SDR_TX, ch, "PAD",
            ch ? _cfg->txgainB : _cfg->txgainA); //[0,30]
    }

    for (auto ch : channels) {
        baStn[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    // we disable channel 1 because of the internal LDO issue.
    // This will be fixed in the next revision (E) of Iris.
    if (_cfg->nChannels == 1) {
        baStn[i]->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
        baStn[i]->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
    }
    num_radios_configured++;
}

bool RadioConfig::radioStart()
{
    bool good_calib = false;
    if (_cfg->sampleCalEn) {
        std::cout << "start sample offset correction" << std::endl;
        int iter = 0;
        int max_iter = 3;
        size_t ref_ant = _cfg->ref_ant;
        while (!good_calib) {
            good_calib = correctSampleOffset(ref_ant, _cfg->sampleCalEn);
            iter++;
            if (iter == max_iter && !good_calib) {
                std::cout << "attempted " << max_iter
                          << " unsucessful calibration, stopping ..."
                          << std::endl;
                break;
            }
        }
        if (!good_calib)
            return good_calib;
        else
            std::cout << "sample offset calibration successful!" << std::endl;
    }

    std::vector<unsigned> zeros(_cfg->sampsPerSymbol, 0);
    std::vector<uint32_t> beacon = _cfg->beacon;
    std::vector<unsigned> beacon_weights(_cfg->nAntennas);

    std::vector<uint32_t> pilot = _cfg->pilot;

    std::vector<std::string> _tddSched;
    drain_buffers();
    json conf;
    conf["tdd_enabled"] = true;
    conf["frame_mode"] = "free_running";
    conf["max_frame"] = 0;
    conf["symbol_size"] = _cfg->sampsPerSymbol;
    conf["beacon_start"] = _cfg->prefix;
    conf["beacon_stop"] = _cfg->prefix + _cfg->beacon_len;

    size_t ndx = 0;
    for (size_t i = 0; i < this->_radioNum; i++) {
        bool isRefAnt = (i == _cfg->ref_ant);
        baStn[i]->writeSetting(
            "TX_SW_DELAY", "30"); // experimentally good value for dev front-end
        baStn[i]->writeSetting("TDD_MODE", "true");
        std::vector<std::string> tddSched;
        for (size_t f = 0; f < _cfg->frames.size(); f++) {
            std::string sched = _cfg->frames[f];
            size_t schedSize = sched.size();
            for (size_t s = 0; s < schedSize; s++) {
                char c = _cfg->frames[f].at(s);
                if (c == 'C')
                    sched.replace(s, 1, isRefAnt ? "R" : "P");
                else if (c == 'L')
                    sched.replace(s, 1, isRefAnt ? "P" : "R");
                else if (c == 'P')
                    sched.replace(s, 1, "R");
                else if (c == 'U')
                    sched.replace(s, 1, "R");
                else if (c == 'D')
                    sched.replace(s, 1, "T");
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
            std::vector<unsigned> beacon_weights(
                _cfg->nAntennas, isBeaconAntenna ? 1 : 0);
            std::string tx_ram_wgt = "BEACON_RAM_WGT_";
            if (_cfg->beamsweep) {
                for (size_t j = 0; j < _cfg->nAntennas; j++)
                    beacon_weights[j] = CommsLib::hadamard2(ndx, j);
            }
            baStn[i]->writeRegisters(tx_ram_wgt + c, 0, beacon_weights);
            ++ndx;
        }
        baStn[i]->writeSetting("BEACON_START", std::to_string(_radioNum));
        if (_cfg->recipCalEn) {
            if (isRefAnt) {
                baStn[i]->writeRegisters("TX_RAM_A", 0, pilot);
                // looks like the best solution is to just use one
                // antenna at the reference node and leave the 2nd
                // antenna unused. We either have to use one anntena
                // per board, or if we use both channels we need to
                // exclude reference board from beamforming
            } else {
                std::vector<std::complex<float>> recipCalDlPilot;
                std::vector<std::complex<float>> pre(_cfg->prefix, 0);
                std::vector<std::complex<float>> post(_cfg->postfix, 0);
                recipCalDlPilot = CommsLib::composeRefSymbol(_cfg->pilotsF,
                    _cfg->nChannels * i, _cfg->BS_ANT_NUM, _cfg->OFDM_CA_NUM,
                    _cfg->OFDM_DATA_NUM, _cfg->OFDM_DATA_START, _cfg->CP_LEN);
                recipCalDlPilot.insert(
                    recipCalDlPilot.begin(), pre.begin(), pre.end());
                recipCalDlPilot.insert(
                    recipCalDlPilot.end(), post.begin(), post.end());
                if (kDebugPrintPilot) {
                    std::cout << "recipCalPilot[" << i << "]: ";
                    for (auto const& calP : recipCalDlPilot)
                        std::cout << real(calP) << ", ";
                    std::cout << std::endl;
                }
                baStn[i]->writeRegisters("TX_RAM_A", 0,
                    Utils::cfloat32_to_uint32(recipCalDlPilot, false, "QI"));
                if (_cfg->nChannels == 2) {
                    recipCalDlPilot = CommsLib::composeRefSymbol(_cfg->pilotsF,
                        2 * i + 1, _cfg->BS_ANT_NUM, _cfg->OFDM_CA_NUM,
                        _cfg->OFDM_DATA_NUM, _cfg->OFDM_DATA_START,
                        _cfg->CP_LEN);
                    baStn[i]->writeRegisters("TX_RAM_B", 0,
                        Utils::cfloat32_to_uint32(
                            recipCalDlPilot, false, "QI"));
                }
            }
        }

        baStn[i]->setHardwareTime(0, "TRIGGER");
        baStn[i]->activateStream(this->rxStreams[i]);
        baStn[i]->activateStream(this->txStreams[i]);
    }

    std::cout << "radio start done!" << std::endl;
    return true;
}

void RadioConfig::go()
{
    if (hubs.size() == 0) {
        // std::cout << "triggering first Iris ..." << std::endl;
        baStn[0]->writeSetting("TRIGGER_GEN", "");
    } else {
        // std::cout << "triggering Hub ..." << std::endl;
        hubs[0]->writeSetting("TRIGGER_GEN", "");
    }
}

void RadioConfig::radioTx(void** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (size_t i = 0; i < this->_radioNum; i++) {
        baStn[i]->writeStream(this->txStreams[i], buffs, _cfg->sampsPerSymbol,
            flags, frameTime, 1000000);
    }
}

int RadioConfig::radioTx(
    size_t r /*radio id*/, void** buffs, int flags, long long& frameTime)
{
    int txFlags = 0;
    if (flags == 1)
        txFlags = SOAPY_SDR_HAS_TIME;
    else if (flags == 2)
        txFlags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
    // long long frameTime(0);
    int w = baStn[r]->writeStream(this->txStreams[r], buffs,
        _cfg->sampsPerSymbol, txFlags, frameTime, 1000000);
#if DEBUG_RADIO_TX
    size_t chanMask;
    long timeoutUs(0);
    int statusFlag = 0;
    int s = baStn[r]->readStreamStatus(
        this->txStreams[r], chanMask, statusFlag, frameTime, timeoutUs);
    std::cout << "radio " << r << " tx returned " << w << " and status " << s
              << " when flags was " << flags << std::endl;
#endif
    return w;
}

void RadioConfig::radioRx(void** buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (size_t i = 0; i < this->_radioNum; i++) {
        void** buff = buffs + (i * 2);
        baStn[i]->readStream(this->rxStreams[i], buff, _cfg->sampsPerSymbol,
            flags, frameTime, 1000000);
    }
}

int RadioConfig::radioRx(
    size_t r /*radio id*/, void** buffs, long long& frameTime)
{
    int flags = 0;
    if (r < this->_radioNum) {
        long long frameTimeNs = 0;
        int ret = baStn[r]->readStream(this->rxStreams[r], buffs,
            _cfg->sampsPerSymbol, flags, frameTimeNs, 1000000);
        frameTime = frameTimeNs; // SoapySDR::timeNsToTicks(frameTimeNs, _rate);
        if (ret != (int)_cfg->sampsPerSymbol)
            std::cout << "invalid return " << ret << " from radio " << r
                      << std::endl;
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
        RadioConfig::drain_rx_buffer(
            baStn[i], rxStreams[i], dummybuffs, _cfg->sampsPerSymbol);
    }
}

void RadioConfig::drain_rx_buffer(SoapySDR::Device* ibsSdrs,
    SoapySDR::Stream* istream, std::vector<void*> buffs, size_t symSamp)
{
    long long frameTime = 0;
    int flags = 0, r = 0, i = 0;
    long timeoutUs(0);
    while (r != -1) {
        r = ibsSdrs->readStream(
            istream, buffs.data(), symSamp, flags, frameTime, timeoutUs);
        i++;
    }
    // std::cout << "Number of reads needed to drain: " << i << std::endl;
}

void RadioConfig::adjustDelays(std::vector<int> offset, size_t ref_ant)
{
    size_t ref_offset = ref_ant == 0 ? 1 : 0;
    size_t R = _cfg->nRadios;
    for (size_t i = 0; i < R; i++) {
        int delta = offset[ref_offset] - offset[i];
        std::cout << "sample_adjusting delay of node " << i << " by " << delta
                  << std::endl;
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
        std::cout << "ZYNQ_TEMP: " << baStn[i]->readSensor("ZYNQ_TEMP")
                  << std::endl;
        std::cout << "LMS7_TEMP  : " << baStn[i]->readSensor("LMS7_TEMP")
                  << std::endl;
        std::cout << "FE_TEMP  : " << baStn[i]->readSensor("FE_TEMP")
                  << std::endl;
        std::cout << "TX0 TEMP  : "
                  << baStn[i]->readSensor(SOAPY_SDR_TX, 0, "TEMP") << std::endl;
        std::cout << "TX1 TEMP  : "
                  << baStn[i]->readSensor(SOAPY_SDR_TX, 1, "TEMP") << std::endl;
        std::cout << "RX0 TEMP  : "
                  << baStn[i]->readSensor(SOAPY_SDR_RX, 0, "TEMP") << std::endl;
        std::cout << "RX1 TEMP  : "
                  << baStn[i]->readSensor(SOAPY_SDR_RX, 1, "TEMP") << std::endl;
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
