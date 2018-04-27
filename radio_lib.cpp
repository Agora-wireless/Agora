#include "radio_lib.hpp"

RadioConfig::RadioConfig(std::vector<std::string> radioId, int numCh, double rate, 
double freq, double rxgain, double txgain, int symlen, int symnum, int maxFrame, std::vector<int> sched):
    _radioId(radioId), // 1 or 2
    _numCh(numCh), // 1 or 2
    _symlen(symlen),
    _symnum(symnum),
    _maxFrame(maxFrame),
    _rate(rate),
    _freq(freq),
    _rxgain(rxgain),
    _txgain(txgain),
    _tddSched(sched)
{
    SoapySDR::Kwargs args;
    SoapySDR::Kwargs sargs;
    sargs["MTU"] = "1500";
    //load channels
    std::vector<size_t> channels;
    if (_numCh == 1) channels = {0};
    else if (_numCh == 2) channels = {0, 1};
    else
    {
        std::cout << "Error! Supported number of channels 1 or 2, setting to 2!" << std::endl;
        _numCh = 2;
        channels = {0, 1};
    }

    this->_radioNum = _radioId.size();
     std::cout << "radio num is " << this->_radioNum << std::endl;
    for (int i = 0; i < this->_radioNum; i++)
    {
        args["serial"] = _radioId.at(i);
        baStn.push_back(SoapySDR::Device::make(args));
    }

    for (int i = 0; i < this->_radioNum; i++)
    { 
        //use the RX only antenna, and TRX for tx
        for (auto ch : channels) baStn[i]->setAntenna(SOAPY_SDR_RX, ch, "TRX");

        std::cout << "setting samples rates to " << _rate/1e6 << " Msps..." << std::endl;
        SoapySDR::Kwargs info = baStn[i]->getHardwareInfo();
        for (auto ch : channels)
        {
            baStn[i]->setSampleRate(SOAPY_SDR_RX, ch, _rate);
            baStn[i]->setSampleRate(SOAPY_SDR_TX, ch, _rate);

            baStn[i]->setFrequency(SOAPY_SDR_RX, ch, _freq);
            baStn[i]->setFrequency(SOAPY_SDR_TX, ch, _freq);

            baStn[i]->setGain(SOAPY_SDR_RX, ch, _rxgain);
            if (info["frontend"] == "CBRS")
            {
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "ATTN", 0);
                baStn[i]->setGain(SOAPY_SDR_RX, ch, "PGA", 0);
            }

            baStn[i]->setGain(SOAPY_SDR_TX, ch, _txgain);
            if (info["frontend"] == "CBRS")
            {
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "PAD", (_txgain>45) ? -7 : _txgain-52);
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "ATTN", 0);
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA1", 15);
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA2", 0);
                baStn[i]->setGain(SOAPY_SDR_TX, ch, "PA3", 30);
            }

            baStn[i]->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
        }
        this->rxStreams.push_back(baStn[i]->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs));
        this->txStreams.push_back(baStn[i]->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs));
    }
    std::cout << "radio init done!" << std::endl;
}

void RadioConfig::radioStart(std::vector<void *> buffs)
{
    int flags = SOAPY_SDR_END_BURST | SOAPY_SDR_TX_REPLAY;
    long long frameTime(0);
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->writeRegister("RFCORE", 104, this->_symlen);
        baStn[i]->writeRegister("RFCORE", 108, this->_symnum);
        baStn[i]->writeRegister("RFCORE", 112, this->_maxFrame);
        baStn[i]->writeSetting("TX_SW_DELAY", "30"); // experimentally good value for dev front-end
        baStn[i]->writeSetting("TDD_MODE", std::to_string(0x80000000));
        // write schedule
	    for (int j = 0; j < 16; j++)
            { 
                for(int k = 0; k < this->_symnum; k++) // symnum <= 256
                {
	                  baStn[i]->writeRegister("RFCORE", 116, j*256+k);
	                  baStn[i]->writeRegister("RFCORE", 120, this->_tddSched[k]);
                }
            }
        // write beacons to FPGA buffers
        baStn[i]->writeStream(this->txStreams[i], buffs.data(), this->_symlen, flags, frameTime);
    }
    baStn[0]->writeSetting("TRIGGER_GEN", "");
    std::cout << "radio start done!" << std::endl;
}

void RadioConfig::radioStop()
{
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->writeSetting("TDD_MODE", "0");
        // write schedule
	for (int j = 0; j < 16; j++) 
            for(int k = 0; k < this->_symnum; k++) // symnum <= 256
            {
		baStn[i]->writeRegister("RFCORE", 116, j*256+k);
		baStn[i]->writeRegister("RFCORE", 120, 0);
            }
        baStn[i]->closeStream(this->rxStreams[i]);
        baStn[i]->closeStream(this->txStreams[i]);
    }
}

void RadioConfig::radioTx(std::vector<void *> buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->writeStream(this->txStreams[i], buffs.data(), this->_symlen, flags, frameTime, 1000000);
    }
}

void RadioConfig::radioRx(std::vector<void *> buffs)
{
    int flags = 0;
    long long frameTime(0);
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->readStream(this->rxStreams[i], buffs.data(), this->_symlen, flags, frameTime, 1000000);
    }
}
void RadioConfig::radioSched(std::vector<int> sched)
{
    // make sure we can pause the framer before we rewrite schedules to avoid
    // any race conditions, for now I set tdd mode to 0
    for (int i = 0; i < this->_radioNum; i++)
    {
        baStn[i]->writeSetting("TDD_MODE", "0");
        // write schedule
	for (int j = 0; j < 16; j++)
        { 
            for(int k = 0; k < this->_symnum; k++) // symnum <= 256
            {
		baStn[i]->writeRegister("RFCORE", 116, j*256+k);
		baStn[i]->writeRegister("RFCORE", 120, sched[k]);
            }
        }
        baStn[i]->writeSetting("TDD_MODE", std::to_string(0x80000000));
    }
}
RadioConfig::~RadioConfig(){}

