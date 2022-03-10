/**
 * @file radio_lib_mod.cc
 * @brief Implementation file for the RadioConfigNoRxStream class.
 */
#include "radio_lib_mod.h"

#include <SoapySDR/Logger.hpp>

#include "SoapyURLUtils.hpp"
#include "comms-lib.h"
#include "nlohmann/json.hpp"

static constexpr bool kPrintCalibrationMats = false;
static constexpr bool kPrintRadioSettings = false;

static constexpr size_t kSoapyMakeMaxAttempts = 3;
static constexpr size_t kHubMissingWaitMs = 100;

//#define MAX_TX_STATUS_DEPTH (64)

// #define RX_SOCKET_BUFFER_BYTES
//   (50 * 1024 * 1024)  // arbitrary and large PC buffer size

// #define ETHERNET_MTU (1500)  // L2 MTU without the 14-byte eth header
// #define ROUTE_HDR_SIZE (16)  // 128-bit transfer for routing header
// #define PADDED_ETH_HDR_SIZE
//   (16)  // 14 bytes + 2 bytes padding (holds size in bytes)
// #define IPv6_UDP_SIZE (40 + 8)  // 40 bytes of IPv6 + 8 bytes of UDP header
// #define TWBW_HDR_SIZE (sizeof(uint64_t) * 4)  // 4 transfers at 64-bits width

constexpr size_t kDebugIrisRx = false;
//constexpr size_t kDebugIrisTxStatus = false;

/// Iris tx status header flags
#define SEQUENCE_MASK (0xffff)
#define HAS_SEQUENCE_bp (16)
#define HAS_SEQUENCE_bf (uint64_t(1) << HAS_SEQUENCE_bp)
#define SEQUENCE_ERROR_bp (17)
#define SEQUENCE_ERROR_bf (uint64_t(1) << SEQUENCE_ERROR_bp)
#define HAS_STATUS_bp (18)
#define HAS_STATUS_bf (uint64_t(1) << HAS_STATUS_bp)
#define HAS_TIME_bp (19)
#define HAS_TIME_bf (uint64_t(1) << HAS_TIME_bp)
#define TIME_ERROR_bp (20)
#define TIME_ERROR_bf (uint64_t(1) << TIME_ERROR_bp)
#define UNDERFLOW_ERROR_bp (21)
#define UNDERFLOW_ERROR_bf (uint64_t(1) << UNDERFLOW_ERROR_bp)
#define BURST_END_bp (22)
#define BURST_END_bf (uint64_t(1) << BURST_END_bp)
#define OVERFLOW_ERROR_bp (23)
#define OVERFLOW_ERROR_bf (uint64_t(1) << OVERFLOW_ERROR_bp)

/// Iris rx header flags
#define SEQ_REQUEST_bp (25)
#define SEQ_REQUEST_bf (uint64_t(1) << SEQ_REQUEST_bp)
#define IS_TRIGGER_bp (26)
#define IS_TRIGGER_bf (uint64_t(1) << IS_TRIGGER_bp)
#define IS_BURST_bp (28)
#define IS_BURST_bf (uint64_t(1) << IS_BURST_bp)
#define RX_OVERFLOW_bp (29)
#define RX_OVERFLOW_bf (uint64_t(1) << RX_OVERFLOW_bp)
#define RX_TIME_ERROR_bp (30)
#define RX_TIME_ERROR_bf (uint64_t(1) << RX_TIME_ERROR_bp)
#define HAS_TIME_RX_bp (31)
#define HAS_TIME_RX_bf (uint64_t(1) << HAS_TIME_RX_bp)

// Packing this would probably be advisable
struct IrisCommData {
  uint64_t header_[2u];
  // Raw sample data, byte accessable
  uint8_t data_[];
};
static_assert(sizeof(IrisCommData::header_) == 16);

RadioConfigNoRxStream::RadioConfigNoRxStream(Config* cfg)
    : cfg_(cfg), num_radios_initialized_(0), num_radios_configured_(0) {
  SoapySDR::Kwargs args;
  // load channels
  auto channels = Utils::StrToChannels(cfg_->Channel());
  ///Reduce the soapy log level
  SoapySDR::setLogLevel(SoapySDR::LogLevel::SOAPY_SDR_SSI);

  radio_num_ = cfg_->NumRadios();
  antenna_num_ = cfg_->BsAntNum();

  std::cout << "BS Radio num is " << radio_num_
            << ", Antenna num: " << antenna_num_ << std::endl;

  //Build hub devices
  for (size_t i = 0; i < cfg_->NumCells(); i++) {
    SoapySDR::Device* hub_device = nullptr;
    if (!cfg_->HubId().at(i).empty()) {
      args["driver"] = "remote";
      args["timeout"] = "100000";
      args["serial"] = cfg_->HubId().at(i);
      args["remote:type"] = "faros";
      args["remote:driver"] = "faros";
      args["remote:mtu"] = "1500";
      args["remote:ipver"] = "6";
      args["remote:prot"] = "udp";

      for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
        try {
          hub_device = SoapySDR::Device::make(args);
          break;
        } catch (const std::runtime_error& e) {
          const auto* message = e.what();
          std::printf("RadioConfigNoRxStream::Soapy error[%zu] -- %s\n", tries,
                      message);
          std::this_thread::sleep_for(
              std::chrono::milliseconds(kHubMissingWaitMs));
        }
      }
      if (hub_device == nullptr) {
        std::printf(
            "SoapySDR failed to locate the hub device %s in %zu tries\n",
            cfg_->HubId().at(i).c_str(), kSoapyMakeMaxAttempts);
        throw std::runtime_error("SoapySDR failed to locate the hub device");
      }
    }
    hubs_.push_back(hub_device);
  }

  ba_stn_.resize(radio_num_);
  tx_streams_.resize(radio_num_);
  rx_streams_.resize(radio_num_);
  std::vector<std::thread> init_bs_threads;

  for (size_t i = 0; i < radio_num_; i++) {
    //init_bs_threads.emplace_back(&RadioConfigNoRxStream::InitBsRadio, this, i);
    InitBsRadio(i);
  }

  // Block until all radios are initialized
  size_t num_checks = 0;
  size_t num_radios_init = num_radios_initialized_.load();
  while (num_radios_init != radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      std::printf(
          "RadioConfigNoRxStream: Waiting for radio initialization, %zu of %zu "
          "ready\n",
          num_radios_init, radio_num_);
      num_checks = 0;
    }
    num_radios_init = num_radios_initialized_.load();
  }

  for (auto& join_thread : init_bs_threads) {
    join_thread.join();
  }

  std::vector<std::thread> config_bs_threads;
  for (size_t i = 0; i < radio_num_; i++) {
    //config_bs_threads.emplace_back(&RadioConfigNoRxStream::ConfigureBsRadio,
    //                               this, i);
    ConfigureBsRadio(i);
  }

  num_checks = 0;
  // Block until all radios are configured
  size_t num_radios_config = num_radios_configured_.load();
  while (num_radios_config != radio_num_) {
    num_checks++;
    if (num_checks > 1e9) {
      std::printf(
          "RadioConfigNoRxStream: Waiting for radio initialization, %zu of %zu "
          "ready\n",
          num_radios_config, radio_num_);
      num_checks = 0;
    }
    num_radios_config = num_radios_configured_.load();
  }

  for (auto& join_thread : config_bs_threads) {
    join_thread.join();
  }

  for (size_t i = 0; i < radio_num_; i++) {
    std::cout << cfg_->RadioId().at(i) << ": Front end "
              << ba_stn_.at(i)->getHardwareInfo()["frontend"] << std::endl;

    if (kPrintRadioSettings) {
      for (auto c : channels) {
        if (c < ba_stn_.at(i)->getNumChannels(SOAPY_SDR_RX)) {
          std::printf("RX Channel %zu\n", c);
          std::printf("Actual RX sample rate: %fMSps...\n",
                      (ba_stn_.at(i)->getSampleRate(SOAPY_SDR_RX, c) / 1e6));
          std::printf("Actual RX frequency: %fGHz...\n",
                      (ba_stn_.at(i)->getFrequency(SOAPY_SDR_RX, c) / 1e9));
          std::printf("Actual RX gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c)));

          std::printf("Actual RX LNA gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "LNA")));
          std::printf("Actual RX PGA gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "PGA")));
          std::printf("Actual RX TIA gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "TIA")));
          if (ba_stn_.at(i)->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            std::printf("Actual RX LNA1 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "LNA1")));
            std::printf("Actual RX LNA2 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_RX, c, "LNA2")));
          }
          std::printf("Actual RX bandwidth: %fM...\n",
                      (ba_stn_.at(i)->getBandwidth(SOAPY_SDR_RX, c) / 1e6));
          std::printf("Actual RX antenna: %s...\n",
                      (ba_stn_.at(i)->getAntenna(SOAPY_SDR_RX, c).c_str()));
        }
      }

      for (auto c : channels) {
        if (c < ba_stn_.at(i)->getNumChannels(SOAPY_SDR_TX)) {
          std::printf("TX Channel %zu\n", c);
          std::printf("Actual TX sample rate: %fMSps...\n",
                      (ba_stn_.at(i)->getSampleRate(SOAPY_SDR_TX, c) / 1e6));
          std::printf("Actual TX frequency: %fGHz...\n",
                      (ba_stn_.at(i)->getFrequency(SOAPY_SDR_TX, c) / 1e9));
          std::printf("Actual TX gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c)));

          std::printf("Actual TX PAD gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PAD")));
          std::printf("Actual TX IAMP gain: %f...\n",
                      (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "IAMP")));
          if (ba_stn_.at(i)->getHardwareInfo()["frontend"].find("CBRS") !=
              std::string::npos) {
            std::printf("Actual TX PA1 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PA1")));
            std::printf("Actual TX PA2 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PA2")));
            std::printf("Actual TX PA3 gain: %f...\n",
                        (ba_stn_.at(i)->getGain(SOAPY_SDR_TX, c, "PA3")));
          }
          std::printf("Actual TX bandwidth: %fM...\n",
                      (ba_stn_.at(i)->getBandwidth(SOAPY_SDR_TX, c) / 1e6));
          std::printf("Actual TX antenna: %s...\n",
                      (ba_stn_.at(i)->getAntenna(SOAPY_SDR_TX, c).c_str()));
        }
      }
      std::cout << std::endl;
    }
  }

  // TODO: For multi-cell, this procedure needs modification
  for (size_t i = 0; i < cfg_->NumCells(); i++) {
    if (hubs_.at(i) == nullptr) {
      ba_stn_.at(i)->writeSetting("SYNC_DELAYS", "");
    } else {
      hubs_[i]->writeSetting("SYNC_DELAYS", "");
    }
  }
  std::cout << "radio init done!" << std::endl;
}

RadioConfigNoRxStream::~RadioConfigNoRxStream() {
  if (radio_num_ != ba_stn_.size()) {
    std::printf(
        "**************************** BAD NEWS ****************************");
    std::printf("radio_num_ != the size of the radio array!");
  }

  for (size_t i = 0; i < radio_num_; i++) {
    ba_stn_.at(i)->closeStream(rx_streams_.at(i));
    ba_stn_.at(i)->closeStream(tx_streams_.at(i));
    SoapySDR::Device::unmake(ba_stn_.at(i));
    ba_stn_.at(i) = nullptr;
  }
  ba_stn_.clear();

  for (auto* hub : hubs_) {
    SoapySDR::Device::unmake(hub);
  }
  hubs_.clear();
}

void RadioConfigNoRxStream::InitBsRadio(size_t tid) {
  size_t i = tid;
  auto channels = Utils::StrToChannels(cfg_->Channel());
  SoapySDR::Kwargs args;
  SoapySDR::Kwargs sargs;
  args["timeout"] = "1000000";
  args["driver"] = "iris";
  args["serial"] = cfg_->RadioId().at(i);

  SoapySDR::Device* bs_device = nullptr;
  for (size_t tries = 0; tries < kSoapyMakeMaxAttempts; tries++) {
    try {
      bs_device = SoapySDR::Device::make(args);
      break;
    } catch (const std::runtime_error& e) {
      const auto* message = e.what();
      std::printf("InitBsRadio[%zu] - Soapy error try %zu -- %s\n", tid, tries,
                  message);
    }
  }
  if (bs_device == nullptr) {
    std::printf("SoapySDR failed to locate the Bs radio %s in %zu attempts\n",
                cfg_->RadioId().at(tid).c_str(), kSoapyMakeMaxAttempts);
    throw std::runtime_error("SoapySDR failed to locate the Bs radio");
  }
  ba_stn_.at(i) = bs_device;
  for (auto ch : {0, 1}) {
    ba_stn_.at(i)->setSampleRate(SOAPY_SDR_RX, ch, cfg_->Rate());
    ba_stn_.at(i)->setSampleRate(SOAPY_SDR_TX, ch, cfg_->Rate());
  }

  // resets the DATA_clk domain logic.
  ba_stn_.at(i)->writeSetting("RESET_DATA_LOGIC", "");

  ConfigureRx(i);

  //rx_streams_.at(i) =
  //    ba_stn_.at(i)->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
  tx_streams_.at(i) =
      ba_stn_.at(i)->setupStream(SOAPY_SDR_TX, SOAPY_SDR_CS16, channels, sargs);
  num_radios_initialized_.fetch_add(1);
}

void RadioConfigNoRxStream::ConfigureBsRadio(size_t tid) {
  // load channels
  auto channels = Utils::StrToChannels(cfg_->Channel());

  // use the TRX antenna port for both tx and rx
  for (auto ch : channels) {
    ba_stn_.at(tid)->setAntenna(SOAPY_SDR_RX, ch, "TRX");
  }

  SoapySDR::Kwargs info = ba_stn_.at(tid)->getHardwareInfo();
  for (auto ch : channels) {
    ba_stn_.at(tid)->setBandwidth(SOAPY_SDR_RX, ch, cfg_->BwFilter());
    ba_stn_.at(tid)->setBandwidth(SOAPY_SDR_TX, ch, cfg_->BwFilter());

    // ba_stn_.at(tid)->setSampleRate(SOAPY_SDR_RX, ch, cfg->Rate());
    // ba_stn_.at(tid)->setSampleRate(SOAPY_SDR_TX, ch, cfg->Rate());

    ba_stn_.at(tid)->setFrequency(SOAPY_SDR_RX, ch, "RF", cfg_->RadioRfFreq());
    ba_stn_.at(tid)->setFrequency(SOAPY_SDR_RX, ch, "BB",
                                  kUseUHD ? 0 : cfg_->Nco());
    ba_stn_.at(tid)->setFrequency(SOAPY_SDR_TX, ch, "RF", cfg_->RadioRfFreq());
    ba_stn_.at(tid)->setFrequency(SOAPY_SDR_TX, ch, "BB",
                                  kUseUHD ? 0 : cfg_->Nco());

    // Unified gains for both lime and frontend
    if (cfg_->SingleGain()) {
      // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:108]
      ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch,
                               ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());
      // w/CBRS 3.6GHz [0:105], 2.5GHZ [0:105]
      ba_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch,
                               ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());
    } else {
      if (info["frontend"].find("CBRS") != std::string::npos) {
        if (cfg_->Freq() > 3e9) {
          ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "ATTN", -6);  //[-18,0]
        } else if ((cfg_->Freq() > 2e9) && (cfg_->Freq() < 3e9)) {
          ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "ATTN", -18);  //[-18,0]
        } else {
          ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "ATTN", -12);  //[-18,0]
        }
        ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "LNA2", 17);  //[0,17]
      }

      ba_stn_.at(tid)->setGain(
          SOAPY_SDR_RX, ch, "LNA",
          ch != 0u ? cfg_->RxGainB() : cfg_->RxGainA());     //[0,30]
      ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "TIA", 0);  //[0,12]
      ba_stn_.at(tid)->setGain(SOAPY_SDR_RX, ch, "PGA", 0);  //[-12,19]

      if (info["frontend"].find("CBRS") != std::string::npos) {
        ba_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "ATTN",
                                 -6);                          //[-18,0] by 3
        ba_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "PA2", 0);  //[0|15]
      }
      ba_stn_.at(tid)->setGain(SOAPY_SDR_TX, ch, "IAMP", 0);  //[-12,12]
      ba_stn_.at(tid)->setGain(
          SOAPY_SDR_TX, ch, "PAD",
          ch != 0u ? cfg_->TxGainB() : cfg_->TxGainA());  //[0,30]
    }
  }

  for (auto ch : channels) {
    ba_stn_.at(tid)->setDCOffsetMode(SOAPY_SDR_RX, ch, true);
  }
  num_radios_configured_.fetch_add(1);
}

void RadioConfigNoRxStream::ConfigureRx(size_t radio_id) {
  auto channels = Utils::StrToChannels(cfg_->Channel());

  std::string stream_protocol =
      ba_stn_.at(radio_id)->readSetting("STREAM_PROTOCOL");
  if (stream_protocol != "twbw64") {
    throw std::runtime_error("Stream protocol mismatch");
  }

  //query remote iris endpoint configuration
  auto remote_address = ba_stn_.at(radio_id)->readSetting("ETH0_IPv6_ADDR");
  const auto remote_port =
      ba_stn_.at(radio_id)->readSetting("UDP_SERVICE_PORT");
  if (remote_address.empty()) {
    throw std::runtime_error(
        "Iris::setupStream: Failed to query Iris IPv6 address");
  }
  if (remote_port.empty()) {
    throw std::runtime_error(
        "Iris::setupStream: Failed to query Iris UDP service port");
  }

  std::printf(
      " STREAM_PROTOCOL  %s\n ETH0_IPv6_ADDR   %s\n UDP_SERVICE_PORT %s\n",
      stream_protocol.c_str(), remote_address.c_str(), remote_port.c_str());

  std::string connect_address;
  const size_t local_interface = 5;
  //get the scope id to get the remote ipv6 address with the local scope id
  std::printf(" Remote address  %s\n", remote_address.c_str());
  const auto percent_pos = remote_address.find_last_of('%');
  if (percent_pos != std::string::npos) {
    connect_address = remote_address.substr(0, percent_pos + 1) +
                      std::to_string(local_interface);
  }
  std::printf(" Connect address %s\n", connect_address.c_str());

  //Setup the socket interface to the radio for the rx stream
  rx_sockets_.emplace_back();
  sklk_SoapyRPCSocket& sock = rx_sockets_.back();

  const SoapyURL bindURL("udp", "::", "0");
  //Bind to the LOCAL_ADDRESS
  //What is the local port?
  int ret = sock.bind(bindURL.toString());
  if (ret != 0)
    throw std::runtime_error("Iris::setupStream: Failed to bind to " +
                             bindURL.toString() + ": " + sock.lastErrorMsg());
  const SoapyURL connect_url("udp", connect_address, remote_port);
  ret = sock.connect(connect_url.toString());
  if (ret != 0)
    throw std::runtime_error("Iris::setupStream: Failed to connect to " +
                             connect_url.toString() + ": " +
                             sock.lastErrorMsg());

  sock.setNonBlocking(true);

  //lookup the local mac address to program the framer
  SoapyURL local_endpoint(sock.getsockname());

  std::printf(" ip6_dst %s\n udp_dst %s\n", local_endpoint.getNode().c_str(),
              local_endpoint.getService().c_str());

  SoapySDR::Kwargs sargs;
  //Not sure if "bypass mode" works
  sargs["remote:prot"] = "none";
  sargs["iris:ip6_dst"] = local_endpoint.getNode();
  sargs["iris:udp_dst"] = local_endpoint.getService();
  rx_streams_.at(radio_id) = ba_stn_.at(radio_id)->setupStream(
      SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
}

bool RadioConfigNoRxStream::RadioStart() {
  DrainBuffers();
  nlohmann::json conf;
  conf["tdd_enabled"] = true;
  conf["frame_mode"] = "free_running";
  conf["max_frame"] = 0;
  conf["symbol_size"] = cfg_->SampsPerSymbol();
  conf["beacon_start"] = cfg_->OfdmTxZeroPrefix();
  conf["beacon_stop"] = cfg_->OfdmTxZeroPrefix() + cfg_->BeaconLen();

  size_t ndx = 0;
  for (size_t i = 0; i < radio_num_; i++) {
    size_t cell_id = cfg_->CellId().at(i);
    bool is_ref_radio = (i == cfg_->RefRadio(cell_id));
    if (cfg_->HwFramer() == true) {
      ba_stn_.at(i)->writeSetting(
          "TX_SW_DELAY",
          "30");  // experimentally good value for dev front-end
      ba_stn_.at(i)->writeSetting("TDD_MODE", "true");
      std::vector<std::string> tdd_sched;

      std::string sched = cfg_->Frame().FrameIdentifier();
      size_t sched_size = sched.length();
      for (size_t s = 0; s < sched_size; s++) {
        char c = cfg_->Frame().FrameIdentifier().at(s);
        if (c == 'C') {
          sched.replace(s, 1, is_ref_radio ? "R" : "T");
        } else if (c == 'L') {
          sched.replace(s, 1, is_ref_radio ? "T" : "R");
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
      std::cout << "Radio " << i << " Frame 1: " << sched << std::endl;
      tdd_sched.push_back(sched);

      conf["frames"] = tdd_sched;
      std::string conf_string = conf.dump();
      ba_stn_.at(i)->writeSetting("TDD_CONFIG", conf_string);

      ba_stn_.at(i)->writeRegisters("BEACON_RAM", 0, cfg_->Beacon());
      for (char const& c : cfg_->Channel()) {
        bool is_beacon_antenna = !cfg_->Beamsweep() && ndx == cfg_->BeaconAnt();
        std::vector<unsigned> beacon_weights(
            cfg_->NumRadios() * cfg_->NumChannels(), is_beacon_antenna ? 1 : 0);
        std::string tx_ram_wgt = "BEACON_RAM_WGT_";
        if (cfg_->Beamsweep()) {
          for (size_t j = 0; j < beacon_weights.size(); j++) {
            beacon_weights.at(j) = CommsLib::Hadamard2(ndx, j);
          }
        }
        ba_stn_.at(i)->writeRegisters(tx_ram_wgt + c, 0, beacon_weights);
        ++ndx;
      }
      ba_stn_.at(i)->writeSetting("BEACON_START", std::to_string(radio_num_));
    }
    ba_stn_.at(i)->setHardwareTime(0, "TRIGGER");
    ba_stn_.at(i)->activateStream(rx_streams_.at(i));
    ba_stn_.at(i)->activateStream(tx_streams_.at(i));
  }
  std::cout << "radio start done!" << std::endl;
  return true;
}

void RadioConfigNoRxStream::Go() {
  // TODO: For multi-cell trigger process needs modification
  for (size_t i = 0; i < cfg_->NumCells(); i++) {
    if (hubs_.at(i) == nullptr) {
      ba_stn_.at(i)->writeSetting("TRIGGER_GEN", "");
    } else {
      hubs_.at(i)->writeSetting("TRIGGER_GEN", "");
    }
  }
}

void RadioConfigNoRxStream::RadioTx(void** buffs) {}

int RadioConfigNoRxStream::RadioTx(size_t radio_id, const void* const* buffs,
                                   int flags, long long& frameTime) {
  return 0;
}

int RadioConfigNoRxStream::RadioTx(
    size_t radio_id,
    const std::vector<std::vector<std::complex<int16_t>>>& tx_data, int flags,
    long long& frameTime)

{
  std::vector<const void*> buffs(tx_data.size());
  for (size_t i = 0; i < tx_data.size(); i++) {
    buffs.at(i) = tx_data.at(i).data();
  }
  return RadioTx(radio_id, buffs.data(), flags, frameTime);
}

void RadioConfigNoRxStream::RadioRx(void** buffs) {}

/* Returns the number of samples */
int RadioConfigNoRxStream::RadioRx(size_t radio_id, void** buffs,
                                   long long& rx_time_ns) {
  constexpr size_t kMaxRxTries = 10000000;
  /* For now, radio rx will return 1 symbol */
  int rx_return = 0;
  constexpr size_t kRxBufSize = 8192u;
  char rcv_buff[kRxBufSize];
  size_t byte_offset = 0;
  size_t num_samples = 0;

  for (size_t i = 0; i < kMaxRxTries; i++) {
    rx_return = rx_sockets_.at(radio_id).recv(&rcv_buff[byte_offset],
                                              (kRxBufSize - byte_offset));
    if (rx_return > 0) {
      const bool rx_complete = ParseRxStream(&rcv_buff[byte_offset], rx_return,
                                             buffs, num_samples, rx_time_ns);
      byte_offset += rx_return;
      std::printf("Rx'd sample count %zu\n", num_samples);

      if (rx_complete) {
        // Rx'd everything
        break;
      }
    } else if (rx_return < 0) {
      if ((errno != EAGAIN) && (errno != EWOULDBLOCK)) {
        throw std::runtime_error("Error in RadioRx!");
      } else if (num_samples == 0) {
        //No Data Stop Trying
        break;
      }
    }
  }
  return num_samples;
}

bool RadioConfigNoRxStream::ParseRxStream(const char* raw_rx_data,
                                          size_t num_rx_bytes,
                                          void** samples_out,
                                          size_t& sample_offset,
                                          long long& rx_time_ns) {
  bool finished;
  // unpacker logic for twbw_rx_framer64
  const auto* rx_data = reinterpret_cast<const IrisCommData*>(raw_rx_data);
  const auto* payload = rx_data->data_;

  const long long rx_time_ticks = static_cast<long long>(rx_data->header_[1u]);
  const size_t burst_count = size_t(rx_data->header_[0u] & 0xffff) + 1;
  const size_t payload_bytes = size_t(num_rx_bytes) - sizeof(rx_data->header_);
  //bytes_per_element is based on "WIRE" format
  //info.options = {SOAPY_SDR_CS16, SOAPY_SDR_CS12, SOAPY_SDR_CS8};
  //info.optionNames = {"Complex int16", "Complex int12", "Complex int8"};
  // this is also a bit more complex and needs to be reviewed for multiple cases
  // 3 bytes == 1 Sample (8 bit + 4 bit) Real (8 bit + 4 bit) Img = 24 bits = 3 Bytes
  const size_t bytes_per_element = 3;
  const size_t rx_samples = payload_bytes / bytes_per_element;
  const size_t num_output_channels = cfg_->NumChannels();
  RtAssert(((payload_bytes % bytes_per_element) == 0), "Invalid payload size!");
  RtAssert(((rx_samples % num_output_channels) == 0),
           "Unexpected number of received samples");

  //Return time
  rx_time_ns = rx_time_ticks;
  if (kDebugIrisRx) {
    const bool has_time = (rx_data->header_[0u] & HAS_TIME_RX_bf) != 0;
    const bool error_time = (rx_data->header_[0u] & RX_TIME_ERROR_bf) != 0;
    const bool error_overflow = (rx_data->header_[0u] & RX_OVERFLOW_bf) != 0;
    const bool is_burst = (rx_data->header_[0u] & IS_BURST_bf) != 0;
    const bool is_trigger = (rx_data->header_[0u] & IS_TRIGGER_bf) != 0;
    std::cout << "===========================================" << std::endl
              << "Received " << std::dec << num_rx_bytes << " bytes "
              << std::endl
              << "hdr0 " << std::hex << rx_data->header_[0u] << std::endl
              << "hdr1 " << std::hex << rx_data->header_[1u] << std::endl
              << std::dec << "Has Time " << has_time << " Time Error "
              << error_time << std::endl
              << "Overflow " << error_overflow << std::endl
              << "Is burst " << is_burst << " is Trigger " << is_trigger
              << std::endl
              << "Burst Count = " << burst_count
              << " payload bytes = " << payload_bytes
              << " samples = " << rx_samples << std::endl
              << "Rx Time: " << rx_time_ticks << std::endl
              << "Frame: "
              << static_cast<size_t>((rx_time_ns >> 32u) & 0xFFFFFFFF)
              << "  Symbol: "
              << static_cast<size_t>((rx_time_ticks >> 16u) & 0xFFFF)
              << std::endl
              << "===========================================" << std::endl;
  }

  //static void wire48_to_cs16x1(const void* inBuff, void* const* outBuffs,
  //                             const size_t num) {
  size_t byte_offset = 0;
  size_t channel_number = 0;
  const size_t total_samples = sample_offset + rx_samples;
  while (sample_offset < total_samples) {
    const uint16_t i_lsb = uint16_t(payload[byte_offset]);
    const uint16_t split = uint16_t(payload[byte_offset + 1u]);
    const uint16_t q_msb = uint16_t(payload[byte_offset + 2u]);
    //Sign extend the 16 bit value (zero out lsb);
    auto output =
        reinterpret_cast<std::complex<int16_t>*>(samples_out[channel_number]);
    output[sample_offset++] =
        std::complex<int16_t>(int16_t((split << 12u) | (i_lsb << 4u)),
                              int16_t((q_msb << 8u) | (split & 0xf0)));
    byte_offset += bytes_per_element;
    channel_number++;
    if (channel_number == cfg_->NumChannels()) {
      channel_number = 0;
    }
  }
  RtAssert(total_samples <= cfg_->SampsPerSymbol(),
           "Number of samples exceeds samples per symbol");

  if ((total_samples == cfg_->SampsPerSymbol()) || (burst_count > rx_samples)) {
    finished = true;
  } else {
    finished = false;
  }
  return finished;
}

int RadioConfigNoRxStream::RadioRx(
    size_t radio_id, std::vector<std::vector<std::complex<int16_t>>>& rx_data,
    long long& rx_time_ns) {
  std::vector<void*> buffs(rx_data.size());
  for (size_t i = 0u; i < rx_data.size(); i++) {
    buffs.at(i) = rx_data.at(i).data();
  }
  return RadioRx(radio_id, buffs.data(), rx_time_ns);
}

void RadioConfigNoRxStream::DrainBuffers() {
  std::vector<std::vector<std::complex<int16_t>>> sample_storage(
      cfg_->NumChannels(),
      std::vector<std::complex<int16_t>>(cfg_->SampsPerSymbol(),
                                         std::complex<int16_t>(0, 0)));
}

void RadioConfigNoRxStream::ReadSensors() {
  for (size_t i = 0; i < radio_num_; i++) {
    std::cout << "TEMPs on Iris " << i << std::endl;
    std::cout << "ZYNQ_TEMP: " << ba_stn_.at(i)->readSensor("ZYNQ_TEMP")
              << std::endl;
    std::cout << "LMS7_TEMP  : " << ba_stn_.at(i)->readSensor("LMS7_TEMP")
              << std::endl;
    std::cout << "FE_TEMP  : " << ba_stn_.at(i)->readSensor("FE_TEMP")
              << std::endl;
    std::cout << "TX0 TEMP  : "
              << ba_stn_.at(i)->readSensor(SOAPY_SDR_TX, 0, "TEMP")
              << std::endl;
    std::cout << "TX1 TEMP  : "
              << ba_stn_.at(i)->readSensor(SOAPY_SDR_TX, 1, "TEMP")
              << std::endl;
    std::cout << "RX0 TEMP  : "
              << ba_stn_.at(i)->readSensor(SOAPY_SDR_RX, 0, "TEMP")
              << std::endl;
    std::cout << "RX1 TEMP  : "
              << ba_stn_.at(i)->readSensor(SOAPY_SDR_RX, 1, "TEMP")
              << std::endl;
    std::cout << std::endl;
  }
}

void RadioConfigNoRxStream::RadioStop() {
  std::vector<uint32_t> zeros(4096, 0);
  std::string corr_conf_str = "{\"corr_enabled\":false}";
  std::string tdd_conf_str = "{\"tdd_enabled\":false}";
  for (size_t i = 0; i < radio_num_; i++) {
    ba_stn_.at(i)->deactivateStream(rx_streams_.at(i));
    ba_stn_.at(i)->deactivateStream(tx_streams_.at(i));
    ba_stn_.at(i)->writeSetting("TDD_MODE", "false");
    ba_stn_.at(i)->writeSetting("TDD_CONFIG", tdd_conf_str);
  }
}
