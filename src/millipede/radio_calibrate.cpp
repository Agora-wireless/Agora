#include "matplotlibcpp.h"
#include "radio_lib.hpp"

namespace plt = matplotlibcpp;

std::vector<std::complex<float>> RadioConfig::snoopSamples(
    SoapySDR::Device* dev, size_t channel, size_t readSize)
{
    std::vector<uint32_t> samps_int
        = dev->readRegisters("RX_SNOOPER", channel, readSize);
    std::vector<std::complex<float>> samps
        = Utils::uint32tocfloat(samps_int, "IQ");
    return samps;
}

void RadioConfig::adjustCalibrationGains(std::vector<SoapySDR::Device*> rxDevs,
    SoapySDR::Device* txDev, size_t channel, double fftBin, bool plot)
{
    using std::cout;
    using std::endl;
    double targetLevel = -10;
    double attnMax = -18;
    size_t N = 1024;
    size_t rxDevsSize = rxDevs.size();
    auto win = CommsLib::hannWindowFunction(N);
    const auto windowGain = CommsLib::windowFunctionPower(win);

    // reset all gains
    for (size_t ch = 0; ch < 2; ch++) {
        txDev->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
        txDev->setGain(SOAPY_SDR_TX, ch, "PAD", 0);
        txDev->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);
        txDev->setGain(SOAPY_SDR_TX, ch, "ATTN", attnMax);
    }

    for (size_t r = 0; r < rxDevsSize; r++) {
        for (size_t ch = 0; ch < 2; ch++) {
            rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "LNA", 0);
            rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "PGA", 0);
            rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "TIA", 0);
            rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "ATTN", attnMax);
            rxDevs[r]->setGain(SOAPY_SDR_RX, ch, "LNA2", 14.0);
        }
    }

    txDev->setGain(SOAPY_SDR_TX, channel, "PAD", 40);
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

    float maxToneLevel = -200;
    std::vector<bool> adjustedRadios(rxDevsSize, 0);
    std::vector<float> toneLevels(rxDevsSize, 0);
    size_t remainingRadios = adjustedRadios.size();
    for (size_t r = 0; r < rxDevsSize; r++) {
        const auto samps = RadioConfig::snoopSamples(rxDevs[r], channel, N);
        auto toneLevel
            = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
        if (toneLevel >= targetLevel) {
            adjustedRadios[r] = true;
            remainingRadios--;
        }
        toneLevels[r] = toneLevel;
        if (toneLevel > maxToneLevel)
            maxToneLevel = toneLevel;
        cout << "Node " << r << ": toneLevel0=" << toneLevel << endl;
    }

    std::string nextGainStage;
    if (remainingRadios == rxDevsSize) {
        // if all need adjustment, try bumping up tx gain first
        txDev->setGain(SOAPY_SDR_TX, channel, "ATTN",
            std::min((targetLevel - maxToneLevel) + attnMax, -6.0));
        cout << "Increasing TX gain level (ATTN) to "
             << std::min((targetLevel - maxToneLevel) + attnMax, -6.0) << endl;
        nextGainStage = "ATTN";
    } else {
        for (size_t r = 0; r < rxDevsSize; r++) {
            if (adjustedRadios[r])
                continue;
            rxDevs[r]->setGain(SOAPY_SDR_RX, channel, "ATTN",
                std::min((targetLevel - toneLevels[r]) + attnMax, 0.0));
            cout << "Increasing RX gain level (ATTN) to "
                 << std::min((targetLevel - maxToneLevel) + attnMax, 0.0)
                 << endl;
        }
        nextGainStage = "LNA";
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        const auto samps = RadioConfig::snoopSamples(rxDevs[r], channel, N);
        float toneLevel
            = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
        if (toneLevel >= targetLevel) {
            adjustedRadios[r] = true;
            remainingRadios--;
        }
        toneLevels[r] = toneLevel;
        cout << "Node " << r << ": toneLevel1=" << toneLevel << endl;
    }

    if (remainingRadios == 0)
        return;
    double minGain = 0;
    double maxGain = 30;
    if (nextGainStage == "ATTN") {
        minGain = attnMax;
        maxGain = 0;
    }

    // adjust next gain stage
    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        rxDevs[r]->setGain(SOAPY_SDR_RX, channel, nextGainStage,
            std::min((targetLevel - toneLevels[r]) + minGain, maxGain));
        cout << "Increasing RX gain level (" << nextGainStage << ") to "
             << std::min((targetLevel - toneLevels[r]) + minGain, maxGain)
             << endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        const auto samps = RadioConfig::snoopSamples(rxDevs[r], channel, N);
        auto toneLevel
            = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
        if (toneLevel > targetLevel) {
            adjustedRadios[r] = true;
            remainingRadios--;
        }
        toneLevels[r] = toneLevel;
        cout << "Node " << r << ": toneLevel2=" << toneLevel << endl;
    }

    if (remainingRadios == 0 || nextGainStage == "LNA")
        return;

    // adjust next gain stage
    minGain = 0;
    maxGain = 30;
    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        rxDevs[r]->setGain(SOAPY_SDR_RX, channel, "LNA",
            std::min((targetLevel - toneLevels[r]), maxGain));
        cout << "Increasing RX gain level (LNA) to "
             << std::min((targetLevel - toneLevels[r]) + minGain, maxGain)
             << endl;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));

    for (size_t r = 0; r < rxDevsSize; r++) {
        if (adjustedRadios[r])
            continue;
        auto samps = RadioConfig::snoopSamples(rxDevs[r], channel, N);
        float toneLevel
            = CommsLib::measureTone(samps, win, windowGain, fftBin, N);
        if (toneLevel > targetLevel) {
            adjustedRadios[r] = true;
            remainingRadios--;
        }
        toneLevels[r] = toneLevel;
        cout << "Node " << r << ": toneLevel3=" << toneLevel << endl;
        if (plot) {
            auto fftMag = CommsLib::magnitudeFFT(samps, win, N);
            std::vector<double> magDouble(N);
            std::transform(
                fftMag.begin(), fftMag.end(), magDouble.begin(), [](float cf) {
                    return 10 * std::max(std::log10((double)cf), -20.0);
                });
            // std::vector<double> sampsDouble(N);
            // std::transform(samps.begin(), samps.end(), sampsDouble.begin(),
            //    [](std::complex<float> cf) {
            //        return cf.real();
            //    });
            plt::figure_size(1200, 780);
            // plt::plot(sampsDouble);
            plt::plot(magDouble);
            plt::xlim(0, (int)N);
            plt::ylim(-100, 100);
            // plt::ylim(-1, 1);
            plt::title(
                "Spectrum figure After Gain Adjustment, FFT Window POWER "
                + std::to_string(windowGain));
            plt::legend();
            plt::save("rx" + std::to_string(rxDevsSize) + "_"
                + std::to_string(r) + "_ch" + std::to_string(channel) + ".png");
        }
    }

    std::cout << rxDevsSize - remainingRadios << " radios reached target level"
              << std::endl;
}

void RadioConfig::setIQBalance(
    SoapySDR::Device* dev, int direction, size_t channel, int gcorr, int iqcorr)
{
    auto gcorri = (gcorr < 0) ? 2047 - std::abs(gcorr) : 2047;
    auto gcorrq = (gcorr > 0) ? 2047 - std::abs(gcorr) : 2047;
    double gainIQ = double(gcorrq) / double(gcorri);
    double phaseIQ = 2 * std::atan(iqcorr / 2047.0);
    std::complex<double> IQcorr
        = gainIQ * std::exp(std::complex<double>(0, phaseIQ));
    dev->setIQBalance(direction, channel, IQcorr);
}

void RadioConfig::dciqMinimize(SoapySDR::Device* targetDev,
    SoapySDR::Device* refDev, int direction, size_t channel,
    double rxCenterTone, double txCenterTone)
{
    size_t N = 1024;
    std::vector<float> win = CommsLib::hannWindowFunction(N);
    const auto windowGain = CommsLib::windowFunctionPower(win);

    targetDev->setIQBalance(direction, channel, 0.0);
    targetDev->setDCOffset(direction, channel, 0.0);

    // match the internal fixed point representation for DC correction
    const int fixedScale = (direction == SOAPY_SDR_RX) ? 64 : 128;

    // measure initial
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
        const auto samps = snoopSamples(refDev, channel, N);
        const auto measDCLevel
            = CommsLib::measureTone(samps, win, windowGain, rxCenterTone, N);
        const auto measImbalanceLevel = CommsLib::measureTone(
            samps, win, windowGain, rxCenterTone - txCenterTone, N);
        const auto desiredToneLevel = CommsLib::measureTone(
            samps, win, windowGain, rxCenterTone + txCenterTone, N);
        std::cout << "dciqMinimize initial: dcLvl=" << measDCLevel
                  << " dB, imLvl=" << measImbalanceLevel
                  << " dB, toneLevel=" << desiredToneLevel << "dB" << std::endl;
    }

    // look through each correction arm twice
    float minDcLevel(0);
    std::complex<double> bestDcCorr(0.0, 0.0);
    for (size_t iter = 0; iter < 4; iter++) {
        int start = -fixedScale, stop = +fixedScale, step = 8;
        if (iter == 2)
            minDcLevel = 0; // restart with finer search
        if (iter > 1) // narrow in for the final iteration set
        {
            const int center = int(
                (((iter % 2) == 0) ? bestDcCorr.imag() : bestDcCorr.real())
                * fixedScale);
            start = std::max<int>(start, center - 8),
            stop = std::min<int>(stop, center + 8), step = 1;
        }
        for (int i = start; i < stop; i += step) {
            // try I or Q arm based on iteration
            const auto dcCorr = ((iter % 2) == 0)
                ? std::complex<double>(
                      bestDcCorr.real(), double(i) / fixedScale)
                : std::complex<double>(
                      double(i) / fixedScale, bestDcCorr.imag());
            targetDev->setDCOffset(direction, channel, dcCorr);

            // measure the efficacy
            std::this_thread::sleep_for(
                std::chrono::milliseconds(SETTLE_TIME_MS));
            const auto samps = snoopSamples(refDev, channel, N);
            const auto measDcLevel = CommsLib::measureTone(
                samps, win, windowGain, rxCenterTone, N);

            // save desired results
            if (measDcLevel < minDcLevel) {
                minDcLevel = measDcLevel;
                bestDcCorr = dcCorr;
            }
        }
    }

    targetDev->setDCOffset(direction, channel, bestDcCorr);
    if (direction == SOAPY_SDR_TX) {
        long dccorri = std::lround(bestDcCorr.real() * 128);
        long dccorrq = std::lround(bestDcCorr.imag() * 128);
        std::cout << "Optimized TX DC Offset: (" << dccorri << "," << dccorrq
                  << ")\n";
    } else {
        long dcoffi = std::lround(bestDcCorr.real() * 64);
        if (dcoffi < 0)
            dcoffi = (1 << 6) | std::abs(dcoffi);

        long dcoffq = std::lround(bestDcCorr.imag() * 64);
        if (dcoffq < 0)
            dcoffq = (1 << 6) | std::abs(dcoffq);
        std::cout << "Optimized RX DC Offset: (" << dcoffi << "," << dcoffq
                  << ")\n";
    }

    // correct IQ imbalance
    float minImbalanceLevel(0);
    int bestgcorr = 0, bestiqcorr = 0;
    for (size_t iter = 0; iter < 4; iter++) {
        int start = -512, stop = 512, step = 8;
        if (iter == 2)
            minImbalanceLevel = 0; // restart with finer search
        if (iter > 1) {
            const int center = ((iter % 2) == 0) ? bestgcorr : bestiqcorr;
            start = std::max<int>(start, center - 8);
            stop = std::min<int>(stop, center + 8), step = 1;
        }
        // SoapySDR::logf(debugLogLevel, "start=%d, stop=%d, step=%d", start,
        // stop, step);
        for (int i = start; i < stop; i += step) {
            const int gcorr = ((iter % 2) == 0) ? i : bestgcorr;
            const int iqcorr = ((iter % 2) == 1) ? i : bestiqcorr;
            RadioConfig::setIQBalance(
                targetDev, direction, channel, gcorr, iqcorr);

            // measure the efficacy
            std::this_thread::sleep_for(
                std::chrono::milliseconds(SETTLE_TIME_MS));
            const auto samps = snoopSamples(refDev, channel, N);
            const auto measImbalanceLevel = CommsLib::measureTone(
                samps, win, windowGain, rxCenterTone - txCenterTone, N);

            // save desired results
            if (measImbalanceLevel < minImbalanceLevel) {
                minImbalanceLevel = measImbalanceLevel;
                bestgcorr = gcorr;
                bestiqcorr = iqcorr;
            }
        }
    }

    // apply the ideal correction
    RadioConfig::setIQBalance(
        targetDev, direction, channel, bestgcorr, bestiqcorr);
    auto gcorri = (bestgcorr < 0) ? 2047 - std::abs(bestgcorr) : 2047;
    auto gcorrq = (bestgcorr > 0) ? 2047 - std::abs(bestgcorr) : 2047;
    std::cout << "Optimized IQ Imbalance Setting: GCorr (" << gcorri << ","
              << gcorrq << "), iqcorr=" << bestiqcorr << "\n";

    // measure corrections
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(SETTLE_TIME_MS));
        const auto samps = snoopSamples(refDev, channel, N);
        const auto measDCLevel
            = CommsLib::measureTone(samps, win, windowGain, rxCenterTone, N);
        const auto measImbalanceLevel = CommsLib::measureTone(
            samps, win, windowGain, rxCenterTone - txCenterTone, N);
        const auto desiredToneLevel = CommsLib::measureTone(
            samps, win, windowGain, rxCenterTone + txCenterTone, N);
        std::cout << "dciqMinimize final: dcLvl=" << measDCLevel
                  << " dB, imLvl=" << measImbalanceLevel
                  << " dB, toneLevel=" << desiredToneLevel << "dB" << std::endl;
    }
}

void RadioConfig::dciqCalibrationProc(size_t channel)
{
    std::cout << "****************************************************\n";
    std::cout << "   DC Offset and IQ Imbalance Calibration: Ch " << channel
              << std::endl;
    std::cout << "****************************************************\n";
    double sampleRate = _cfg->rate;
    double centerRfFreq = _cfg->radioRfFreq;
    double toneBBFreq = sampleRate / 7;
    size_t radioSize = _cfg->nRadios;

    size_t referenceRadio = radioSize / 2;
    SoapySDR::Device* refDev = baStn[referenceRadio];

    /*
     * Start with calibrating the rx paths on all radios using the reference
     * radio
     */
    std::cout << "Calibrating Rx Channels with Tx Reference Radio\n";
    refDev->setFrequency(
        SOAPY_SDR_TX, channel, "RF", centerRfFreq + toneBBFreq);
    refDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
    refDev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
    std::vector<SoapySDR::Device*> allButRefDevs;
    for (size_t r = 0; r < radioSize; r++) {
        if (r == referenceRadio)
            continue;
        SoapySDR::Device* dev = baStn[r];
        // must set TX "RF" Freq to make sure, we continue using the same LO for
        // rx cal
        dev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq);
        dev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
        dev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
        dev->setDCOffsetMode(SOAPY_SDR_RX, channel, false);
        allButRefDevs.push_back(dev);
    }
    refDev->writeSetting(
        SOAPY_SDR_TX, channel, "TSP_TSG_CONST", std::to_string(1 << 14));
    refDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");

    // Tune rx gains for calibration on all radios except reference radio
    // Tune tx gain on reference radio
    RadioConfig::adjustCalibrationGains(
        allButRefDevs, refDev, channel, toneBBFreq / sampleRate, true);

    // Minimize Rx DC offset and IQ Imbalance on all receiving radios
    // TODO: Parallelize this loop
    for (size_t r = 0; r < radioSize - 1; r++) {
        RadioConfig::dciqMinimize(allButRefDevs[r], allButRefDevs[r],
            SOAPY_SDR_RX, channel, 0.0, toneBBFreq / sampleRate);
    }

    refDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
    refDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");

    /*
     * Calibrate the rx path of the reference radio
     */
    std::cout << "Calibrating Rx Channel of the Reference Radio\n";
    std::vector<SoapySDR::Device*> refDevContainer;
    refDevContainer.push_back(refDev);
    SoapySDR::Device* refRefDev = allButRefDevs[referenceRadio - 1];

    refRefDev->setFrequency(
        SOAPY_SDR_TX, channel, "RF", centerRfFreq + toneBBFreq);
    refRefDev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
    // must set TX "RF" Freq to make sure, we continue using the same LO for rx
    // cal
    refDev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq);
    refDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
    refDev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);
    refDev->setDCOffsetMode(SOAPY_SDR_RX, channel, false);

    refRefDev->writeSetting(
        SOAPY_SDR_TX, channel, "TSP_TSG_CONST", std::to_string(1 << 14));
    refRefDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");

    // Tune rx gain for calibraion on reference radio
    // Tune tx gain on neighboring radio to reference radio
    RadioConfig::adjustCalibrationGains(
        refDevContainer, refRefDev, channel, toneBBFreq / sampleRate);
    RadioConfig::dciqMinimize(
        refDev, refDev, SOAPY_SDR_RX, channel, 0.0, toneBBFreq / sampleRate);

    refRefDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
    refRefDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");

    /*
     * Calibrate the tx path of the reference radio
     */
    std::cout << "Calibrating Tx Channels with Rx Reference Radio\n";
    double txToneBBFreq = sampleRate / 21;
    std::vector<SoapySDR::Device*> refRefDevContainer;
    refRefDevContainer.push_back(refRefDev);

    // must set TX "RF" Freq to make sure, we continue using the same LO for rx
    // cal
    refRefDev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq);
    refRefDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
    refRefDev->setFrequency(SOAPY_SDR_RX, channel, "BB",
        -toneBBFreq); // Should this be nagative if we need
                      // centerRfFreq-toneBBFreq at true center?
    refDev->setFrequency(SOAPY_SDR_TX, channel, "RF", centerRfFreq);
    refDev->setFrequency(SOAPY_SDR_TX, channel, "BB", txToneBBFreq);
    refDev->writeSetting(
        SOAPY_SDR_TX, channel, "TSP_TSG_CONST", std::to_string(1 << 14));
    refDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");

    // Tune tx gain for calibraion on reference antenna
    // Tune rx gain on neighboring radio to reference radio
    RadioConfig::adjustCalibrationGains(refRefDevContainer, refDev, channel,
        (toneBBFreq + txToneBBFreq) / sampleRate);
    RadioConfig::dciqMinimize(refDev, refRefDev, SOAPY_SDR_TX, channel,
        toneBBFreq / sampleRate, txToneBBFreq / sampleRate);

    // kill TX on ref at the end
    refDev->writeSetting(SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
    refDev->writeSetting(SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");
    refDev->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
    refRefDev->setFrequency(SOAPY_SDR_RX, channel, "BB", 0);

    /*
     * Now calibrate the tx paths on all other radios using the reference radio
     */
    std::cout << "Calibrating Tx Channel of the Reference Radio\n";
    // refDev->setFrequency(SOAPY_SDR_RX, channel, "RF", centerRfFreq);
    refDev->setFrequency(SOAPY_SDR_RX, channel, "BB",
        -toneBBFreq); // Should this be nagative if we need
                      // centerRfFreq-toneBBFreq at true center?
    for (size_t r = 0; r < radioSize - 1; r++) {
        allButRefDevs[r]->setFrequency(
            SOAPY_SDR_TX, channel, "RF", centerRfFreq);
        allButRefDevs[r]->setFrequency(
            SOAPY_SDR_TX, channel, "BB", txToneBBFreq);
        allButRefDevs[r]->writeSetting(
            SOAPY_SDR_TX, channel, "TSP_TSG_CONST", std::to_string(1 << 14));
        allButRefDevs[r]->writeSetting(
            SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "true");
        // Tune tx gain for calibraion of the current radio
        // Tune rx gain on the reference radio
        RadioConfig::adjustCalibrationGains(refDevContainer, allButRefDevs[r],
            channel, (toneBBFreq + txToneBBFreq) / sampleRate);
        RadioConfig::dciqMinimize(allButRefDevs[r], refDev, SOAPY_SDR_TX,
            channel, toneBBFreq / sampleRate, txToneBBFreq / sampleRate);
        allButRefDevs[r]->writeSetting(
            SOAPY_SDR_TX, channel, "TX_ENB_OVERRIDE", "false");
        allButRefDevs[r]->writeSetting(
            SOAPY_SDR_TX, channel, "TSP_TSG_CONST", "NONE");
        allButRefDevs[r]->setFrequency(SOAPY_SDR_TX, channel, "BB", 0);
    }
    std::cout << "****************************************************\n";
    std::cout << "   Ending DC Offset and IQ Imbalance Calibration\n";
    std::cout << "****************************************************\n";
}

bool RadioConfig::correctSampleOffset(size_t ref_ant, bool sample_adjust)
{
    bool good_csi = true;

    size_t seq_len = _cfg->pilot_cf32.size();
    size_t fft_len = _cfg->ofdm_ca_num;
    std::cout << "calibration seq_len " << seq_len << " fft_len " << fft_len
              << std::endl;
    std::vector<std::complex<double>> pilot_cd64;
    std::vector<std::complex<int16_t>> pilot_cs16;

    for (size_t i = 0; i < seq_len; i++) {
        std::complex<float> cf = _cfg->pilot_cf32[i];
        pilot_cs16.push_back(std::complex<int16_t>(
            (int16_t)(cf.real() * 32768), (int16_t)(cf.imag() * 32768)));
        pilot_cd64.push_back(std::complex<double>(cf.real(), cf.imag()));
    }

    std::vector<std::complex<int16_t>> pre(_cfg->prefix, 0);
    std::vector<std::complex<int16_t>> post(_cfg->postfix, 0);
    pilot_cs16.insert(pilot_cs16.begin(), pre.begin(), pre.end());
    pilot_cs16.insert(pilot_cs16.end(), post.begin(), post.end());
    size_t read_len = pilot_cs16.size();

    // Transmitting from only one chain, create a null vector for chainB
    std::vector<std::complex<int16_t>> dummy_cs16(read_len, 0);

    std::vector<void*> txbuff0(2);
    txbuff0[0] = pilot_cs16.data();
    txbuff0[1] = dummy_cs16.data();

    std::vector<std::vector<std::complex<int16_t>>> buff;
    // int ant = _cfg->nChannels;
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

    std::vector<std::complex<int16_t>> dummybuff(read_len);
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
        int ret = baStn[i]->writeStream(this->txStreams[i], txbuff0.data(),
            pilot_cs16.size(), tx_flags, txTime, 1000000);
        if (ret < (int)read_len)
            std::cout << "bad write\n";
        for (size_t j = 0; j < R; j++) {
            if (j == i)
                continue;
            int rx_flags = SOAPY_SDR_WAIT_TRIGGER | SOAPY_SDR_END_BURST;
            ret = baStn[j]->activateStream(
                this->rxStreams[j], rx_flags, rxTime, read_len);
        }

        go();

        int flags = 0;
        for (size_t j = 0; j < R; j++) {
            if (j == i)
                continue;
            std::vector<void*> rxbuff(2);
            rxbuff[0] = buff[(i * R + j)].data();
            // rxbuff[1] = ant == 2 ? buff[(i*M+j)*ant+1].data() :
            // dummyBuff.data();
            rxbuff[1] = dummybuff.data();
            ret = baStn[j]->readStream(this->rxStreams[j], rxbuff.data(),
                read_len, flags, rxTime, 1000000);
            if (ret < (int)read_len)
                std::cout << "bad read (" << ret << ") at node " << j
                          << " from node " << i << std::endl;
        }
    }

    for (size_t i = 0; i < R; i++) {
        baStn[i]->deactivateStream(this->txStreams[i]);
        baStn[i]->deactivateStream(this->rxStreams[i]);
        baStn[i]->setGain(SOAPY_SDR_TX, 0, "PAD", _cfg->txgainA); //[0,30]
    }

    // int ref_ant = 0;
    size_t ref_offset = ref_ant == 0 ? 1 : 0;
    std::vector<int> offset(R);
    std::vector<size_t> start_up(R);
    std::vector<size_t> start_dn(R);

    for (size_t i = 0; i < R; i++) {
        std::vector<std::complex<double>> up(read_len);
        std::vector<std::complex<double>> dn(read_len);
        std::transform(buff[ref_ant * R + i].begin(),
            buff[ref_ant * R + i].end(), up.begin(),
            [](std::complex<int16_t> ci) {
                return std::complex<double>(
                    ci.real() / 32768.0, ci.imag() / 32768.0);
            });
        std::transform(buff[i * R + ref_ant].begin(),
            buff[i * R + ref_ant].end(), dn.begin(),
            [](std::complex<int16_t> ci) {
                return std::complex<double>(
                    ci.real() / 32768.0, ci.imag() / 32768.0);
            });

        size_t peak_up = CommsLib::find_pilot_seq(up, pilot_cd64, seq_len);
        size_t peak_dn = CommsLib::find_pilot_seq(dn, pilot_cd64, seq_len);
        start_up[i] = peak_up < seq_len ? 0 : peak_up - seq_len + _cfg->cp_len;
        start_dn[i] = peak_dn < seq_len ? 0 : peak_dn - seq_len + _cfg->cp_len;
        std::cout << "receive starting position from/to node " << i << ": "
                  << start_up[i] << "/" << start_dn[i] << std::endl;
        if (start_up[i] == 0 || start_dn[i] == 0)
            good_csi = false;
        if (i == ref_offset) {
            offset[i] = start_up[i];
            offset[ref_ant] = start_dn[i];
        } else if (i != ref_ant)
            offset[i] = start_up[i];

#if DEBUG_PLOT
        std::vector<double> up_I(read_len);
        std::transform(up.begin(), up.end(), up_I.begin(),
            [](std::complex<double> cd) { return cd.real(); });

        std::vector<double> dn_I(read_len);
        std::transform(dn.begin(), dn.end(), dn_I.begin(),
            [](std::complex<double> cd) { return cd.real(); });

        plt::figure_size(1200, 780);
        plt::plot(up_I);
        plt::xlim(0, read_len);
        plt::ylim(-1, 1);
        plt::title("ant " + std::to_string(ref_ant) + " (ref) to ant "
            + std::to_string(i));
        plt::legend();
        plt::save("up_" + std::to_string(i) + ".png");

        plt::figure_size(1200, 780);
        plt::plot(dn_I);
        plt::xlim(0, read_len);
        plt::ylim(-1, 1);
        plt::title("ant " + std::to_string(i) + " to ant (ref)"
            + std::to_string(ref_ant));
        plt::legend();
        plt::save("dn_" + std::to_string(i) + ".png");
#endif
    }

    // sample_adjusting trigger delays based on lts peak index
    if (good_csi) {
        if (sample_adjust)
            adjustDelays(offset, ref_offset);
    }

    return good_csi;
}

// void RadioConfig::calcReciprocityCalib()
//{
//    calib_mat.resize(R);
//    for (size_t i = 0; i < R; i++) {
//        std::vector<std::complex<float> > dn_t(fft_len);
//        std::vector<std::complex<float> > up_t(fft_len);
//        std::transform(buff[i * R + ref_ant].begin() + start_dn[i], buff[i * R
//        + ref_ant].begin() + start_dn[i] + fft_len, dn_t.begin(),
//            [](std::complex<int16_t> cf) { return
//            std::complex<float>(cf.real() / 32768.0, cf.imag() / 32768.0); });
//        std::transform(buff[ref_ant * R + i].begin() + start_up[i],
//        buff[ref_ant * R + i].begin() + start_up[i] + fft_len, up_t.begin(),
//            [](std::complex<int16_t> cf) { return
//            std::complex<float>(cf.real() / 32768.0, cf.imag() / 32768.0); });
//        std::vector<std::complex<float> > dn_f = CommsLib::FFT(dn_t, fft_len);
//        std::vector<std::complex<float> > up_f = CommsLib::FFT(up_t, fft_len);
//        for (size_t f = 0; f < fft_len; f++) {
//            if (f < _cfg->ofdm_data_start || f >= _cfg->ofdm_data_start +
//            _cfg->ofdm_data_num) {
//    	    continue;
//    	}
//            float dre = dn_f[f].real();
//            float dim = dn_f[f].imag();
//            float ure = up_f[f].real();
//            float uim = up_f[f].imag();
//            float re = (dre * ure + dim * uim) / (ure * ure + uim * uim);
//            float im = (dim * ure - dre * uim) / (ure * ure + uim * uim);
//            if (i == ref_ant)
//                calib_mat[i].push_back(1);
//            else
//                calib_mat[i].push_back(std::complex<float>(re, im));
//        }
//    }
//}
