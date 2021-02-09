#include "radio_lib.h"
#include "comms-lib.h"

static constexpr bool kPrintCalibrationMats = false;

std::atomic<size_t> num_radios_initialized;
std::atomic<size_t> num_radios_configured;

RadioConfig::RadioConfig(Config* cfg)
    : cfg_(cfg)
{
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    // load channels
    auto channels = Utils::StrToChannels(cfg_->channel_);

    this->radio_num_ = cfg_->n_radios_;
    this->antenna_num_ = radio_num_ * cfg_->n_channels_;
    std::cout << "radio num is " << this->radio_num_ << std::endl;
    if (cfg_->is_ue_) {
        throw std::invalid_argument("Bad config! Not a UE!");
    }
    if (!kUseUHD && cfg_->hub_ids_.empty() == false) {
        args["driver"] = "remote";
        args["timeout"] = "1000000";
        args["serial"] = cfg_->hub_ids_.at(0);
        hubs_.push_back(SoapySDR::Device::make(args));
    }

    ba_stn_.resize(radio_num_);
    tx_streams_.resize(radio_num_);
    rx_streams_.resize(radio_num_);

    std::vector<RadioConfigContext> radio_config_ctx_vec(this->radio_num_);
    for (size_t i = 0; i < this->radio_num_; i++) {
        auto* context = &radio_config_ctx_vec[i];
        context->brs_ = this;
        context->tid_ = i;
#ifdef THREADED_INIT
        pthread_t init_thread;
        if (pthread_create(&init_thread, NULL, InitBsRadioLaunch, context)
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
    while (num_radios_initialized != this->radio_num_) {
        size_t num_radios_initialized = num_radios_initialized;
        num_checks++;
        if (num_checks > 1e9) {
            std::printf(
                "RadioConfig: Waiting for radio initialization, %zu of %zu "
                "ready\n",
                num_radios_initialized, this->radio_num_);
            num_checks = 0;
        }
    }

    // Perform DC Offset & IQ Imbalance Calibration
    if (cfg_->imbalance_cal_en_) {
        if (cfg_->channel_.find('A') != std::string::npos) {
            DciqCalibrationProc(0);
        }
        if (cfg_->channel_.find('B') != std::string::npos) {
            DciqCalibrationProc(1);
        }
    }

    for (size_t i = 0; i < this->radio_num_; i++) {
        auto* context = &radio_config_ctx_vec[i];
        context->brs_ = this;
        context->tid_ = i;
#ifdef THREADED_INIT
        pthread_t configure_thread;
        if (pthread_create(&configure_thread, NULL,
                RadioConfig::ConfigureBsRadioLaunch, context)
            != 0) {
            perror("init thread create failed");
            std::exit(0);
        }
#else
        configureBSRadio(context);
#endif
    }

    // Block until all radios are configured
    while (num_radios_configured != this->radio_num_) {
        size_t num_radios_configured = num_radios_configured;
        num_checks++;
        if (num_checks > 1e9) {
            std::printf(
                "RadioConfig: Waiting for radio initialization, %zu of %zu "
                "ready\n",
                num_radios_configured, this->radio_num_);
            num_checks = 0;
        }
    }

    for (size_t i = 0; i < this->radio_num_; i++) {
        std::cout << cfg_->radio_ids_.at(i) << ": Front end "
                  << ba_stn_[i]->getHardwareInfo()["frontend"] << std::endl;
        for (auto c : channels) {
            if (c < ba_stn_[i]->getNumChannels(SOAPY_SDR_RX)) {
                std::printf("RX Channel %zu\n", c);
                std::printf("Actual RX sample rate: %fMSps...\n",
                    (ba_stn_[i]->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
                std::printf("Actual RX frequency: %fGHz...\n",
                    (ba_stn_[i]->getFrequency(SOAPY_SDR_RX, c) / 1e9));
                std::printf("Actual RX gain: %f...\n",
                    (ba_stn_[i]->getGain(SOAPY_SDR_RX, c)));
                if (!kUseUHD) {
                    std::printf("Actual RX LNA gain: %f...\n",
                        (ba_stn_[i]->getGain(SOAPY_SDR_RX, c, "LNA")));
                    std::printf("Actual RX PGA gain: %f...\n",
                        (ba_stn_[i]->getGain(SOAPY_SDR_RX, c, "PGA")));
                    std::printf("Actual RX TIA gain: %f...\n",
                        (ba_stn_[i]->getGain(SOAPY_SDR_RX, c, "TIA")));
                    if (ba_stn_[i]->getHardwareInfo()["frontend"].find("CBRS")
                        != std::string::npos) {
                        std::printf("Actual RX LNA1 gain: %f...\n",
                            (ba_stn_[i]->getGain(SOAPY_SDR_RX, c, "LNA1")));
                        std::printf("Actual RX LNA2 gain: %f...\n",
                            (ba_stn_[i]->getGain(SOAPY_SDR_RX, c, "LNA2")));
                    }
                }
                std::printf("Actual RX bandwidth: %fM...\n",
                    (ba_stn_[i]->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
                std::printf("Actual RX antenna: %s...\n",
                    (ba_stn_[i]->getAntenna(SOAPY_SDR_RX, c).c_str()));
            }
        }

        for (auto c : channels) {
            if (c < ba_stn_[i]->getNumChannels(SOAPY_SDR_TX)) {
                std::printf("TX Channel %zu\n", c);
                std::printf("Actual TX sample rate: %fMSps...\n",
                    (ba_stn_[i]->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
                std::printf("Actual TX frequency: %fGHz...\n",
                    (ba_stn_[i]->getFrequency(SOAPY_SDR_TX, c) / 1e9));
                std::printf("Actual TX gain: %f...\n",
                    (ba_stn_[i]->getGain(SOAPY_SDR_TX, c)));
                if (!kUseUHD) {
                    std::printf("Actual TX PAD gain: %f...\n",
                        (ba_stn_[i]->getGain(SOAPY_SDR_TX, c, "PAD")));
                    std::printf("Actual TX IAMP gain: %f...\n",
                        (ba_stn_[i]->getGain(SOAPY_SDR_TX, c, "IAMP")));
                    if (ba_stn_[i]->getHardwareInfo()["frontend"].find("CBRS")
                        != std::string::npos) {
                        std::printf("Actual TX PA1 gain: %f...\n",
                            (ba_stn_[i]->getGain(SOAPY_SDR_TX, c, "PA1")));
                        std::printf("Actual TX PA2 gain: %f...\n",
                            (ba_stn_[i]->getGain(SOAPY_SDR_TX, c, "PA2")));
                        std::printf("Actual TX PA3 gain: %f...\n",
                            (ba_stn_[i]->getGain(SOAPY_SDR_TX, c, "PA3")));
                    }
                }
                std::printf("Actual TX bandwidth: %fM...\n",
                    (ba_stn_[i]->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
                std::printf("Actual TX antenna: %s...\n",
                    (ba_stn_[i]->getAntenna(SOAPY_SDR_TX, c).c_str()));
            }
        }
        std::cout << std::endl;
    }

    if (!kUseUHD) {
        if (hubs_.empty()) {
            ba_stn_[0]->writeSetting("SYNC_DELAYS", "");
        } else {
            hubs_[0]->writeSetting("SYNC_DELAYS", "");
        }
    }

    std::cout << "radio init done!" << std::endl;
}

void* RadioConfig::InitBsRadioLaunch(void* in_context)
{
    auto* context = (RadioConfigContext*)in_context;
    context->brs_->InitBsRadio(context);
    return 0;
}

void RadioConfig::InitBsRadio(RadioConfigContext* context)
{
    size_t i = context->tid_;
    auto channels = Utils::StrToChannels(cfg_->channel_);
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    args["timeout"] = "1000000";
    if (!kUseUHD) {
        args["driver"] = "iris";
        args["serial"] = cfg_->radio_ids_.at(i);
    } else {
        args["driver"] = "uhd";
        args["addr"] = cfg_->radio_ids_.at(i);
    }
    ba_stn_[i] = SoapySDR::Device::make(args);
    for (auto ch : { 0, 1 }) {
        ba_stn_[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg_->rate_);
        ba_stn_[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg_->rate_);
    }
    rx_streams_[i] = ba_stn_[i]->setupStream(
        SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    tx_streams_[i] = ba_stn_[i]->setupStream(
        SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
    num_radios_initialized++;
}

void* RadioConfig::ConfigureBsRadioLaunch(void* in_context)
{
    RadioConfigContext* context = ((RadioConfigContext*)in_context);
    RadioConfig* brs = context->brs_;
    brs->ConfigureBsRadio(context);
    return 0;
}

void RadioConfig::ConfigureBsRadio(RadioConfigContext* context)
{
    size_t i = context->tid_;

    // load channels
    auto channels = Utils::StrToChannels(cfg_->channel_);

    // resets the DATA_clk domain logic.
    ba_stn_[i]->writeSetting("RESET_DATA_LOGIC", "");

    // use the TRX antenna port for both tx and rx
    for (auto ch : channels) {
        if (!kUseUHD) {
            ba_stn_[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");
        } else {
            ba_stn_[i]->setAntenna(SOAPY_SDR_RX, ch, "RX2");
            ba_stn_[i]->setAntenna(SOAPY_SDR_TX, ch, "TX/RX");
        }
    }

    SoapySDR::Kwargs info = ba_stn_[i]->getHardwareInfo();
    for (auto ch : channels) {
        if (!kUseUHD) {
            ba_stn_[i]->setBandwidth(SOAPY_SDR_RX, ch, cfg_->bw_filter_);
            ba_stn_[i]->setBandwidth(SOAPY_SDR_TX, ch, cfg_->bw_filter_);
        }

        // baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, cfg->rate);
        // baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, cfg->rate);

        ba_stn_[i]->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg_->radio_rf_freq_);
        ba_stn_[i]->setFrequency(
            SOAPY_SDR_RX, ch, "BB", kUseUHD ? 0 : cfg_->nco_);
        ba_stn_[i]->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg_->radio_rf_freq_);
        ba_stn_[i]->setFrequency(
            SOAPY_SDR_TX, ch, "BB", kUseUHD ? 0 : cfg_->nco_);

        if (!kUseUHD) {
            // Unified gains for both lime and frontend
            if (cfg_->SingleGain()) {
                // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
                ba_stn_[i]->setGain(SOAPY_SDR_RX, ch,
                    ch != 0u ? cfg_->rx_gain_b_ : cfg_->rx_gain_a_);
                // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
                ba_stn_[i]->setGain(SOAPY_SDR_TX, ch,
                    ch != 0u ? cfg_->tx_gain_b_ : cfg_->tx_gain_a_);
            } else {
                if (info["frontend"].find("CBRS") != std::string::npos) {
                    if (cfg_->freq_ > 3e9) {
                        ba_stn_[i]->setGain(
                            SOAPY_SDR_RX, ch, "ATTN", -6); //[-18,0]
                    } else if (cfg_->freq_ > 2e9 && cfg_->freq_ < 3e9) {
                        ba_stn_[i]->setGain(
                            SOAPY_SDR_RX, ch, "ATTN", -18); //[-18,0]
                    } else {
                        ba_stn_[i]->setGain(
                            SOAPY_SDR_RX, ch, "ATTN", -12); //[-18,0]
                    }
                    ba_stn_[i]->setGain(SOAPY_SDR_RX, ch, "LNA2", 17); //[0,17]
                }

                ba_stn_[i]->setGain(SOAPY_SDR_RX, ch, "LNA",
                    ch != 0u ? cfg_->rx_gain_b_ : cfg_->rx_gain_a_); //[0,30]
                ba_stn_[i]->setGain(SOAPY_SDR_RX, ch, "TIA", 0); //[0,12]
                ba_stn_[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0); //[-12,19]

                if (info["frontend"].find("CBRS") != std::string::npos) {
                    ba_stn_[i]->setGain(
                        SOAPY_SDR_TX, ch, "ATTN", -6); //[-18,0] by 3
                    ba_stn_[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0); //[0|15]
                }
                ba_stn_[i]->setGain(SOAPY_SDR_TX, ch, "IAMP", 0); //[-12,12]
                ba_stn_[i]->setGain(SOAPY_SDR_TX, ch, "PAD",
                    ch != 0u ? cfg_->tx_gain_b_ : cfg_->tx_gain_a_); //[0,30]
            }
        } else {
            ba_stn_[i]->setGain(SOAPY_SDR_RX, ch, "PGA0",
                ch != 0u ? cfg_->rx_gain_b_ : cfg_->rx_gain_a_);
            ba_stn_[i]->setGain(SOAPY_SDR_TX, ch, "PGA0",
                ch != 0u ? cfg_->tx_gain_b_ : cfg_->tx_gain_a_);
        }
    }

    for (auto ch : channels) {
        ba_stn_[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
    }

    num_radios_configured++;
}

bool RadioConfig::RadioStart()
{
    bool good_calib = false;
    AllocBuffer1d(&init_calib_dl_processed_,
        cfg_->ofdm_data_num_ * cfg_->bf_ant_num_ * sizeof(arma::cx_float),
        Agora_memory::Alignment_t::kK64Align, 1);
    AllocBuffer1d(&init_calib_ul_processed_,
        cfg_->ofdm_data_num_ * cfg_->bf_ant_num_ * sizeof(arma::cx_float),
        Agora_memory::Alignment_t::kK64Align, 1);
    // initialize init_calib to a matrix of ones
    for (size_t i = 0; i < cfg_->ofdm_data_num_ * cfg_->bf_ant_num_; i++) {
        init_calib_dl_processed_[i] = 1;
        init_calib_ul_processed_[i] = 1;
    }

    calib_meas_num_ = cfg_->init_calib_repeat_;
    if (calib_meas_num_ != 0u) {
        init_calib_ul_.Calloc(calib_meas_num_,
            cfg_->ofdm_data_num_ * cfg_->bf_ant_num_,
            Agora_memory::Alignment_t::kK64Align);
        init_calib_dl_.Calloc(calib_meas_num_,
            cfg_->ofdm_data_num_ * cfg_->bf_ant_num_,
            Agora_memory::Alignment_t::kK64Align);
        if (cfg_->downlink_mode_) {
            int iter = 0;
            int max_iter = 3;
            std::cout << "Start initial reciprocity calibration..."
                      << std::endl;
            while (!good_calib) {
                good_calib = InitialCalib(cfg_->sample_cal_en_);
                iter++;
                if (iter == max_iter && !good_calib) {
                    std::cout << "attempted " << max_iter
                              << " unsucessful calibration, stopping ..."
                              << std::endl;
                    break;
                }
            }
            if (!good_calib) {
                return good_calib;
            } else {
                std::cout << "initial calibration successful!" << std::endl;
            }

            // process initial measurements
            arma::cx_fcube calib_dl_cube(cfg_->ofdm_data_num_,
                cfg_->bf_ant_num_, calib_meas_num_, arma::fill::zeros);
            arma::cx_fcube calib_ul_cube(cfg_->ofdm_data_num_,
                cfg_->bf_ant_num_, calib_meas_num_, arma::fill::zeros);
            for (size_t i = 0; i < calib_meas_num_; i++) {
                arma::cx_fmat calib_dl_mat(init_calib_dl_[i],
                    cfg_->ofdm_data_num_, cfg_->bf_ant_num_, false);
                arma::cx_fmat calib_ul_mat(init_calib_ul_[i],
                    cfg_->ofdm_data_num_, cfg_->bf_ant_num_, false);
                calib_dl_cube.slice(i) = calib_dl_mat;
                calib_ul_cube.slice(i) = calib_ul_mat;
                if (kPrintCalibrationMats) {
                    Utils::PrintMat(
                        calib_dl_mat, "calib_dl_mat" + std::to_string(i));
                    Utils::PrintMat(
                        calib_ul_mat, "calib_ul_mat" + std::to_string(i));
                    Utils::PrintMat(calib_dl_mat / calib_ul_mat,
                        "calib_mat" + std::to_string(i));
                }
            }
            arma::cx_fmat calib_dl_mean_mat(init_calib_dl_processed_,
                cfg_->ofdm_data_num_, cfg_->bf_ant_num_, false);
            arma::cx_fmat calib_ul_mean_mat(init_calib_ul_processed_,
                cfg_->ofdm_data_num_, cfg_->bf_ant_num_, false);
            calib_dl_mean_mat
                = arma::mean(calib_dl_cube, 2); // mean along dim 2
            calib_ul_mean_mat
                = arma::mean(calib_ul_cube, 2); // mean along dim 2
            if (kPrintCalibrationMats) {
                Utils::PrintMat(calib_dl_mean_mat, "calib_dl_mat");
                Utils::PrintMat(calib_ul_mean_mat, "calib_ul_mat");
                Utils::PrintMat(
                    calib_dl_mean_mat / calib_ul_mean_mat, "calib_mat");
            }
        }
        init_calib_dl_.Free();
        init_calib_ul_.Free();
    }

    std::vector<unsigned> zeros(cfg_->samps_per_symbol_, 0);
    std::vector<uint32_t> beacon = cfg_->beacon_;
    std::vector<unsigned> beacon_weights(cfg_->n_antennas_);

    std::vector<uint32_t> pilot = cfg_->pilot_;

    std::vector<std::string> tdd_sched;
    DrainBuffers();
    json conf;
    conf["tdd_enabled"] = true;
    conf["frame_mode"] = "free_running";
    conf["max_frame"] = 0;
    conf["symbol_size"] = cfg_->samps_per_symbol_;
    conf["beacon_start"] = cfg_->ofdm_tx_zero_prefix_;
    conf["beacon_stop"] = cfg_->ofdm_tx_zero_prefix_ + cfg_->beacon_len_;

    size_t ndx = 0;
    for (size_t i = 0; i < this->radio_num_; i++) {
        bool is_ref_ant = (i == cfg_->ref_ant_);
        ba_stn_[i]->writeSetting(
            "TX_SW_DELAY", "30"); // experimentally good value for dev front-end
        ba_stn_[i]->writeSetting("TDD_MODE", "true");
        std::vector<std::string> tdd_sched;
        for (size_t f = 0; f < cfg_->frames_.size(); f++) {
            std::string sched = cfg_->frames_[f];
            size_t sched_size = sched.size();
            for (size_t s = 0; s < sched_size; s++) {
                char c = cfg_->frames_[f].at(s);
                if (c == 'C') {
                    sched.replace(s, 1, is_ref_ant ? "R" : "T");
                } else if (c == 'L') {
                    sched.replace(s, 1, is_ref_ant ? "P" : "R");
                } else if (c == 'P') {
                    sched.replace(s, 1, "R");
                } else if (c == 'U') {
                    sched.replace(s, 1, "R");
                } else if (c == 'D') {
                    sched.replace(s, 1, "T");
                } else if (c != 'B') {
                    sched.replace(s, 1, "G");
                }
            }
            std::cout << "Radio " << i << " Frame " << f << ": " << sched
                      << std::endl;
            tdd_sched.push_back(sched);
        }
        conf["frames"] = tdd_sched;
        std::string conf_string = conf.dump();
        ba_stn_[i]->writeSetting("TDD_CONFIG", conf_string);

        ba_stn_[i]->writeRegisters("BEACON_RAM", 0, beacon);
        for (char const& c : cfg_->channel_) {
            bool is_beacon_antenna
                = !cfg_->beamsweep_ && ndx == cfg_->beacon_ant_;
            std::vector<unsigned> beacon_weights(
                cfg_->n_antennas_, is_beacon_antenna ? 1 : 0);
            std::string tx_ram_wgt = "BEACON_RAM_WGT_";
            if (cfg_->beamsweep_) {
                for (size_t j = 0; j < cfg_->n_antennas_; j++) {
                    beacon_weights[j] = CommsLib::Hadamard2(ndx, j);
                }
            }
            ba_stn_[i]->writeRegisters(tx_ram_wgt + c, 0, beacon_weights);
            ++ndx;
        }
        ba_stn_[i]->writeSetting("BEACON_START", std::to_string(radio_num_));
        if (cfg_->recip_cal_en_) {
            if (is_ref_ant) {
                ba_stn_[i]->writeRegisters("TX_RAM_A", 0, pilot);
                // looks like the best solution is to just use one
                // antenna at the reference node and leave the 2nd
                // antenna unused. We either have to use one anntena
                // per board, or if we use both channels we need to
                // exclude reference board from beamforming
            } else {
                std::vector<std::complex<float>> recip_cal_dl_pilot;
                std::vector<std::complex<float>> pre(
                    cfg_->ofdm_tx_zero_prefix_, 0);
                std::vector<std::complex<float>> post(
                    cfg_->ofdm_tx_zero_postfix_, 0);
                recip_cal_dl_pilot
                    = CommsLib::ComposePartialPilotSym(cfg_->common_pilot_,
                        cfg_->n_channels_ * i * kCalibScGroupSize,
                        kCalibScGroupSize, cfg_->ofdm_ca_num_,
                        cfg_->ofdm_data_num_, cfg_->ofdm_data_start_,
                        cfg_->cp_len_, false /*block type*/);
                if (kDebugPrintPilot) {
                    std::cout << "recipCalPilot[" << i << "]: ";
                    for (auto const& cal_p : recip_cal_dl_pilot) {
                        std::cout << real(cal_p) << ", ";
                    }
                    std::cout << std::endl;
                }
                recip_cal_dl_pilot.insert(
                    recip_cal_dl_pilot.begin(), pre.begin(), pre.end());
                recip_cal_dl_pilot.insert(
                    recip_cal_dl_pilot.end(), post.begin(), post.end());
                ba_stn_[i]->writeRegisters("TX_RAM_A", 0,
                    Utils::Cfloat32ToUint32(recip_cal_dl_pilot, false, "QI"));
                if (cfg_->n_channels_ == 2) {
                    recip_cal_dl_pilot
                        = CommsLib::ComposePartialPilotSym(cfg_->common_pilot_,
                            (2 * i + 1) * kCalibScGroupSize, kCalibScGroupSize,
                            cfg_->ofdm_ca_num_, cfg_->ofdm_data_num_,
                            cfg_->ofdm_data_start_, cfg_->cp_len_, false);
                    ba_stn_[i]->writeRegisters("TX_RAM_B", 0,
                        Utils::Cfloat32ToUint32(
                            recip_cal_dl_pilot, false, "QI"));
                }
            }
        }

        if (!kUseUHD) {
            ba_stn_[i]->setHardwareTime(0, "TRIGGER");
            ba_stn_[i]->activateStream(this->rx_streams_[i]);
            ba_stn_[i]->activateStream(this->tx_streams_[i]);
        } else {
            ba_stn_[i]->setHardwareTime(0, "UNKNOWN_PPS");
            ba_stn_[i]->activateStream(
                this->rx_streams_[i], SOAPY_SDR_HAS_TIME, 1e9, 0);
            ba_stn_[i]->activateStream(
                this->tx_streams_[i], SOAPY_SDR_HAS_TIME, 1e9, 0);
        }
    }

    std::cout << "radio start done!" << std::endl;
    return true;
}

void RadioConfig::Go()
{
    if (!kUseUHD) {
        if (hubs_.empty()) {
            // std::cout << "triggering first Iris ..." << std::endl;
            ba_stn_[0]->writeSetting("TRIGGER_GEN", "");
        } else {
            // std::cout << "triggering Hub ..." << std::endl;
            hubs_[0]->writeSetting("TRIGGER_GEN", "");
        }
    }
}

void RadioConfig::RadioTx(void** buffs)
{
    int flags = 0;
    long long frame_time(0);
    for (size_t i = 0; i < this->radio_num_; i++) {
        ba_stn_[i]->writeStream(this->tx_streams_[i], buffs,
            cfg_->samps_per_symbol_, flags, frame_time, 1000000);
    }
}

int RadioConfig::RadioTx(
    size_t r /*radio id*/, void** buffs, int flags, long long& frameTime)
{
    int tx_flags = 0;
    if (flags == 1) {
        tx_flags = SOAPY_SDR_HAS_TIME;
    } else if (flags == 2) {
        tx_flags = SOAPY_SDR_HAS_TIME | SOAPY_SDR_END_BURST;
    }
    // long long frameTime(0);

    int w;
    if (!kUseUHD) {
        w = ba_stn_[r]->writeStream(this->tx_streams_[r], buffs,
            cfg_->samps_per_symbol_, tx_flags, frameTime, 1000000);
    } else {
        // For UHD device xmit from host using frameTimeNs
        long long frame_time_ns
            = SoapySDR::ticksToTimeNs(frameTime, cfg_->rate_);
        w = ba_stn_[r]->writeStream(this->tx_streams_[r], buffs,
            cfg_->samps_per_symbol_, tx_flags, frame_time_ns, 1000000);
    }
    if (kDebugRadioTX) {
        size_t chan_mask;
        long timeout_us(0);
        int status_flag = 0;
        int s = ba_stn_[r]->readStreamStatus(this->tx_streams_[r], chan_mask,
            status_flag, frameTime, timeout_us);
        std::cout << "radio " << r << " tx returned " << w << " and status "
                  << s << " when flags was " << flags << std::endl;
    }
    return w;
}

void RadioConfig::RadioRx(void** buffs)
{
    int flags = 0;
    long long frame_time(0);
    for (size_t i = 0; i < this->radio_num_; i++) {
        void** buff = buffs + (i * 2);
        ba_stn_[i]->readStream(this->rx_streams_[i], buff,
            cfg_->samps_per_symbol_, flags, frame_time, 1000000);
    }
}

int RadioConfig::RadioRx(
    size_t r /*radio id*/, void** buffs, long long& frameTime)
{
    int flags = 0;
    if (r < this->radio_num_) {
        long long frame_time_ns = 0;
        int ret = ba_stn_[r]->readStream(this->rx_streams_[r], buffs,
            cfg_->samps_per_symbol_, flags, frame_time_ns, 1000000);

        if (!kUseUHD) {
            // SoapySDR::timeNsToTicks(frameTimeNs, _rate);
            frameTime = frame_time_ns;
        } else {
            // for UHD device recv using ticks
            frameTime = SoapySDR::timeNsToTicks(frame_time_ns, cfg_->rate_);
        }

        if (kDebugRadioRX) {
            if (ret != (int)cfg_->samps_per_symbol_) {
                std::cout << "invalid return " << ret << " from radio " << r
                          << std::endl;
            } else {
                std::cout << "radio " << r << "received " << ret << std::endl;
            }
        }
        return ret;
    }
    std::cout << "invalid radio id " << r << std::endl;
    return 0;
}

void RadioConfig::DrainBuffers()
{
    std::vector<std::complex<int16_t>> dummy_buff0(cfg_->samps_per_symbol_);
    std::vector<std::complex<int16_t>> dummy_buff1(cfg_->samps_per_symbol_);
    std::vector<void*> dummybuffs(2);
    dummybuffs[0] = dummy_buff0.data();
    dummybuffs[1] = dummy_buff1.data();
    for (size_t i = 0; i < cfg_->n_radios_; i++) {
        RadioConfig::DrainRxBuffer(
            ba_stn_[i], rx_streams_[i], dummybuffs, cfg_->samps_per_symbol_);
    }
}

void RadioConfig::DrainRxBuffer(SoapySDR::Device* ibsSdrs,
    SoapySDR::Stream* istream, std::vector<void*> buffs, size_t symSamp)
{
    long long frame_time = 0;
    int flags = 0;
    int r = 0;
    int i = 0;
    long timeout_us(0);
    while (r != -1) {
        r = ibsSdrs->readStream(
            istream, buffs.data(), symSamp, flags, frame_time, timeout_us);
        i++;
    }
    // std::cout << "Number of reads needed to drain: " << i << std::endl;
}

void RadioConfig::ReadSensors()
{
    for (size_t i = 0; i < this->radio_num_; i++) {
        std::cout << "TEMPs on Iris " << i << std::endl;
        std::cout << "ZYNQ_TEMP: " << ba_stn_[i]->readSensor("ZYNQ_TEMP")
                  << std::endl;
        std::cout << "LMS7_TEMP  : " << ba_stn_[i]->readSensor("LMS7_TEMP")
                  << std::endl;
        std::cout << "FE_TEMP  : " << ba_stn_[i]->readSensor("FE_TEMP")
                  << std::endl;
        std::cout << "TX0 TEMP  : "
                  << ba_stn_[i]->readSensor(SOAPY_SDR_TX, 0, "TEMP")
                  << std::endl;
        std::cout << "TX1 TEMP  : "
                  << ba_stn_[i]->readSensor(SOAPY_SDR_TX, 1, "TEMP")
                  << std::endl;
        std::cout << "RX0 TEMP  : "
                  << ba_stn_[i]->readSensor(SOAPY_SDR_RX, 0, "TEMP")
                  << std::endl;
        std::cout << "RX1 TEMP  : "
                  << ba_stn_[i]->readSensor(SOAPY_SDR_RX, 1, "TEMP")
                  << std::endl;
        std::cout << std::endl;
    }
}

void RadioConfig::RadioStop()
{
    std::vector<uint32_t> zeros(4096, 0);
    std::string corr_conf_str = "{\"corr_enabled\":false}";
    std::string tdd_conf_str = "{\"tdd_enabled\":false}";
    for (size_t i = 0; i < this->radio_num_; i++) {
        ba_stn_[i]->deactivateStream(this->rx_streams_[i]);
        ba_stn_[i]->deactivateStream(this->tx_streams_[i]);
        ba_stn_[i]->writeSetting("TDD_MODE", "false");
        ba_stn_[i]->writeSetting("TDD_CONFIG", tdd_conf_str);
    }
}

RadioConfig::~RadioConfig()
{
    FreeBuffer1d(&init_calib_dl_processed_);
    FreeBuffer1d(&init_calib_ul_processed_);
    for (size_t i = 0; i < this->radio_num_; i++) {
        ba_stn_[i]->closeStream(this->rx_streams_[i]);
        ba_stn_[i]->closeStream(this->tx_streams_[i]);
        SoapySDR::Device::unmake(ba_stn_[i]);
    }
}
