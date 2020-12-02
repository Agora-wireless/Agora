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
    if (!kUseUHD || _cfg->hub_ids.size() != 0) {
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
            std::exit(0);
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
            std::printf("RadioConfig: Waiting for radio initialization, %zu of %zu "
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
            std::exit(0);
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
            std::printf("RadioConfig: Waiting for radio initialization, %zu of %zu "
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
                std::printf("RX Channel %zu\n", c);
                std::printf("Actual RX sample rate: %fMSps...\n",
                    (baStn[i]->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
                std::printf("Actual RX frequency: %fGHz...\n",
                    (baStn[i]->getFrequency(SOAPY_SDR_RX, c) / 1e9));
                std::printf("Actual RX gain: %f...\n",
                    (baStn[i]->getGain(SOAPY_SDR_RX, c)));
                if (!kUseUHD) {
                    std::printf("Actual RX LNA gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_RX, c, "LNA")));
                    std::printf("Actual RX PGA gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_RX, c, "PGA")));
                    std::printf("Actual RX TIA gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_RX, c, "TIA")));
                    if (baStn[i]->getHardwareInfo()["frontend"].compare("CBRS")
                        == 0) {
                        std::printf("Actual RX LNA1 gain: %f...\n",
                            (baStn[i]->getGain(SOAPY_SDR_RX, c, "LNA1")));
                        std::printf("Actual RX LNA2 gain: %f...\n",
                            (baStn[i]->getGain(SOAPY_SDR_RX, c, "LNA2")));
                    }
                }
                std::printf("Actual RX bandwidth: %fM...\n",
                    (baStn[i]->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
                std::printf("Actual RX antenna: %s...\n",
                    (baStn[i]->getAntenna(SOAPY_SDR_RX, c).c_str()));
            }
        }

        for (size_t c = 0; c < _cfg->nChannels; c++) {
            if (c < baStn[i]->getNumChannels(SOAPY_SDR_TX)) {
                std::printf("TX Channel %zu\n", c);
                std::printf("Actual TX sample rate: %fMSps...\n",
                    (baStn[i]->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
                std::printf("Actual TX frequency: %fGHz...\n",
                    (baStn[i]->getFrequency(SOAPY_SDR_TX, c) / 1e9));
                std::printf("Actual TX gain: %f...\n",
                    (baStn[i]->getGain(SOAPY_SDR_TX, c)));
                if (!kUseUHD) {
                    std::printf("Actual TX PAD gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_TX, c, "PAD")));
                    std::printf("Actual TX IAMP gain: %f...\n",
                        (baStn[i]->getGain(SOAPY_SDR_TX, c, "IAMP")));
                    if (baStn[i]->getHardwareInfo()["frontend"].compare("CBRS")
                        == 0) {
                        std::printf("Actual TX PA1 gain: %f...\n",
                            (baStn[i]->getGain(SOAPY_SDR_TX, c, "PA1")));
                        std::printf("Actual TX PA2 gain: %f...\n",
                            (baStn[i]->getGain(SOAPY_SDR_TX, c, "PA2")));
                        std::printf("Actual TX PA3 gain: %f...\n",
                            (baStn[i]->getGain(SOAPY_SDR_TX, c, "PA3")));
                    }
                }
                std::printf("Actual TX bandwidth: %fM...\n",
                    (baStn[i]->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
                std::printf("Actual TX antenna: %s...\n",
                    (baStn[i]->getAntenna(SOAPY_SDR_TX, c).c_str()));
            }
        }
        std::cout << std::endl;
    }

    if (!kUseUHD) {
        if (hubs.size() == 0)
            baStn[0]->writeSetting("SYNC_DELAYS", "");
        else
            hubs[0]->writeSetting("SYNC_DELAYS", "");
    }

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
    args["timeout"] = "1000000";
    if (!kUseUHD) {
        args["driver"] = "iris";
        args["serial"] = _cfg->radio_ids.at(i);
    } else {
        args["driver"] = "uhd";
        args["addr"] = _cfg->radio_ids.at(i);
    }
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
        if (!kUseUHD)
            baStn[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");
        else {
            baStn[i]->setAntenna(SOAPY_SDR_RX, ch, "RX2");
            baStn[i]->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
        }

    SoapySDR::Kwargs info = baStn[i]->getHardwareInfo();
    for (auto ch : { 0, 1 }) {
        if (!kUseUHD) {
            baStn[i]->setBandwidth(SOAPY_SDR_RX, ch, _cfg->bwFilter);
            baStn[i]->setBandwidth(SOAPY_SDR_TX, ch, _cfg->bwFilter);
        }

        // baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        // baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

        baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", _cfg->radioRfFreq);
        baStn[i]->setFrequency(SOAPY_SDR_RX, ch, "BB", kUseUHD ? 0 : _cfg->nco);
        baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", _cfg->radioRfFreq);
        baStn[i]->setFrequency(SOAPY_SDR_TX, ch, "BB", kUseUHD ? 0 : _cfg->nco);

        if (!kUseUHD) {
            // Unified gains for both lime and frontend
            if (_cfg->single_gain()) {
                // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
                baStn[i]->setGain(
                    SOAPY_SDR_RX, ch, ch ? _cfg->rx_gain_b : _cfg->rx_gain_a);
                // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
                baStn[i]->setGain(
                    SOAPY_SDR_TX, ch, ch ? _cfg->tx_gain_b : _cfg->tx_gain_a);
            } else {
                if (info["frontend"].find("CBRS") != std::string::npos) {
                    if (_cfg->freq > 3e9)
                        baStn[i]->setGain(
                            SOAPY_SDR_RX, ch, "ATTN", -6); //[-18,0]
                    else if (_cfg->freq > 2e9 && _cfg->freq < 3e9)
                        baStn[i]->setGain(
                            SOAPY_SDR_RX, ch, "ATTN", -18); //[-18,0]
                    else
                        baStn[i]->setGain(
                            SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
                    baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
                }

                baStn[i]->setGain(SOAPY_SDR_RX, ch, "LNA",
                    ch ? _cfg->rx_gain_b : _cfg->rx_gain_a); //[0,30]
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]

                if (info["frontend"].find("CBRS") != std::string::npos) {
                    baStn[i]->setGain(
                        SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
                    baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|15]
                }
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0); //[-12,12]
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "PAD",
                    ch ? _cfg->tx_gain_b : _cfg->tx_gain_a); //[0,30]
            }
        } else {
            baStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA0",
                ch ? _cfg->rx_gain_b : _cfg->rx_gain_a);
            baStn[i]->setGain(SOAPY_SDR_TX, ch, "PGA0",
                ch ? _cfg->tx_gain_b : _cfg->tx_gain_a);
        }
    }

    for (auto ch : channels) {
        baStn[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    // we disable channel 1 because of the internal LDO issue.
    // This will be fixed in the next revision (E) of Iris.
    if (_cfg->nChannels == 1) {
        if (!kUseUHD) {
            baStn[i]->writeSetting(SOAPY_SDR_RX, 1, "ENABLE_CHANNEL", "false");
            baStn[i]->writeSetting(SOAPY_SDR_TX, 1, "ENABLE_CHANNEL", "false");
        }
    }
    num_radios_configured++;
}

bool RadioConfig::radioStart()
{
    bool good_calib = false;
    alloc_buffer_1d(&init_calib_dl_,
        _cfg->OFDM_DATA_NUM * _cfg->BF_ANT_NUM * sizeof(arma::cx_float), Agora_memory::Alignment_t::k64Align, 1);
    alloc_buffer_1d(&init_calib_ul_,
        _cfg->OFDM_DATA_NUM * _cfg->BF_ANT_NUM * sizeof(arma::cx_float), Agora_memory::Alignment_t::k64Align, 1);
    // initialize init_calib to a matrix of ones
    for (size_t i = 0; i < _cfg->OFDM_DATA_NUM * _cfg->BF_ANT_NUM; i++) {
        init_calib_dl_[i] = 1;
        init_calib_ul_[i] = 1;
    }
    if (_cfg->downlink_mode) {
        int iter = 0;
        int max_iter = 3;
        while (!good_calib) {
            good_calib = initial_calib(_cfg->sampleCalEn);
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
            std::cout << "initial calibration successful!" << std::endl;
        //arma::cx_fmat calib_dl_mat(
        //    init_calib_dl_, _cfg->OFDM_DATA_NUM, _cfg->BF_ANT_NUM, false);
        //arma::cx_fmat calib_ul_mat(
        //    init_calib_ul_, _cfg->OFDM_DATA_NUM, _cfg->BF_ANT_NUM, false);
        //Utils::print_mat(calib_dl_mat);
        //Utils::print_mat(calib_ul_mat);
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
    conf["beacon_start"] = _cfg->ofdm_tx_zero_prefix_;
    conf["beacon_stop"] = _cfg->ofdm_tx_zero_prefix_ + _cfg->beacon_len;

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
                if (c == 'C') {
                    sched.replace(s, 1, isRefAnt ? "R" : "T");
                } else if (c == 'L') {
                    sched.replace(s, 1, isRefAnt ? "P" : "R");
                } else if (c == 'P')
                    sched.replace(s, 1, "R");
                else if (c == 'U')
                    sched.replace(s, 1, "R");
                else if (c == 'D')
                    sched.replace(s, 1, "T");
                else if (c != 'B')
                    sched.replace(s, 1, "G");
            }
            std::cout << "Radio " << i << " Frame " << f << ": " << sched
                      << std::endl;
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
                std::vector<std::complex<float>> pre(
                    _cfg->ofdm_tx_zero_prefix_, 0);
                std::vector<std::complex<float>> post(
                    _cfg->ofdm_tx_zero_postfix_, 0);
                recipCalDlPilot = CommsLib::compose_partial_pilot_sym(
                    _cfg->common_pilot, _cfg->nChannels * i * kCalibScGroupSize,
                    kCalibScGroupSize, _cfg->OFDM_CA_NUM, _cfg->OFDM_DATA_NUM,
                    _cfg->OFDM_DATA_START, _cfg->CP_LEN, false /*block type*/);
                if (kDebugPrintPilot) {
                    std::cout << "recipCalPilot[" << i << "]: ";
                    for (auto const& calP : recipCalDlPilot)
                        std::cout << real(calP) << ", ";
                    std::cout << std::endl;
                }
                recipCalDlPilot.insert(
                    recipCalDlPilot.begin(), pre.begin(), pre.end());
                recipCalDlPilot.insert(
                    recipCalDlPilot.end(), post.begin(), post.end());
                baStn[i]->writeRegisters("TX_RAM_A", 0,
                    Utils::cfloat32_to_uint32(recipCalDlPilot, false, "QI"));
                if (_cfg->nChannels == 2) {
                    recipCalDlPilot = CommsLib::compose_partial_pilot_sym(
                        _cfg->common_pilot, (2 * i + 1) * kCalibScGroupSize,
                        kCalibScGroupSize, _cfg->OFDM_CA_NUM,
                        _cfg->OFDM_DATA_NUM, _cfg->OFDM_DATA_START,
                        _cfg->CP_LEN, false);
                    baStn[i]->writeRegisters("TX_RAM_B", 0,
                        Utils::cfloat32_to_uint32(
                            recipCalDlPilot, false, "QI"));
                }
            }
        }

        if (!kUseUHD) {
            baStn[i]->setHardwareTime(0, "TRIGGER");
            baStn[i]->activateStream(this->rxStreams[i]);
            baStn[i]->activateStream(this->txStreams[i]);
        } else {
            baStn[i]->setHardwareTime(0, "UNKNOWN_PPS");
            baStn[i]->activateStream(
                this->rxStreams[i], SOAPY_SDR_HAS_TIME, 1e9, 0);
            baStn[i]->activateStream(
                this->txStreams[i], SOAPY_SDR_HAS_TIME, 1e9, 0);
        }
    }

    std::cout << "radio start done!" << std::endl;
    return true;
}

void RadioConfig::go()
{
    if (!kUseUHD) {
        if (hubs.size() == 0) {
            // std::cout << "triggering first Iris ..." << std::endl;
            baStn[0]->writeSetting("TRIGGER_GEN", "");
        } else {
            // std::cout << "triggering Hub ..." << std::endl;
            hubs[0]->writeSetting("TRIGGER_GEN", "");
        }
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

    int w;
    if (!kUseUHD) {
        w = baStn[r]->writeStream(this->txStreams[r], buffs,
            _cfg->sampsPerSymbol, txFlags, frameTime, 1000000);
    } else {
        // For UHD device xmit from host using frameTimeNs
        long long frameTimeNs = SoapySDR::ticksToTimeNs(frameTime, _cfg->rate);
        w = baStn[r]->writeStream(this->txStreams[r], buffs,
            _cfg->sampsPerSymbol, txFlags, frameTimeNs, 1000000);
    }
    if (kDebugRadioTX) {
        size_t chanMask;
        long timeoutUs(0);
        int statusFlag = 0;
        int s = baStn[r]->readStreamStatus(
            this->txStreams[r], chanMask, statusFlag, frameTime, timeoutUs);
        std::cout << "radio " << r << " tx returned " << w << " and status "
                  << s << " when flags was " << flags << std::endl;
    }
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

        if (!kUseUHD) {
            // SoapySDR::timeNsToTicks(frameTimeNs, _rate);
            frameTime = frameTimeNs;
        } else {
            // for UHD device recv using ticks
            frameTime = SoapySDR::timeNsToTicks(frameTimeNs, _cfg->rate);
        }

        if (kDebugRadioRX) {
            if (ret != (int)_cfg->sampsPerSymbol)
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
