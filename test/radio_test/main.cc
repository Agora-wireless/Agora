/**
 * @file main.cc
 * @brief Main file for the radio test program
 */
#include "client_radio.h"
#include "comms-lib.h"
#include "gflags/gflags.h"
#include "logger.h"
#include "network_utils.h"
#include "radio_lib.h"
#include "rx_status_tracker.h"
#include "signal_handler.h"
#include "version_config.h"

DEFINE_string(
    conf_file,
    TOSTRING(PROJECT_DIRECTORY) "/files/config/examples/ul-one-vulture.json",
    "Config filename");

DEFINE_uint32(rx_symbols, 10,
              "The number of symbols to receive before the program terminates");

//Forward declaration
static void TestBsRadioRx(Config* cfg, const uint32_t max_rx,
                          Radio::RadioType type);
static void TestUeRadioRx(Config* cfg, const uint32_t max_rx,
                          Radio::RadioType type);

int main(int argc, char* argv[]) {
  gflags::SetUsageMessage("conf_file : set the configuration filename");
  gflags::SetVersionString(GetAgoraProjectVersion());
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::string conf_file;
  AGORA_LOG_INIT();

  // For backwards compatibility
  if (argc == 2) {
    conf_file = std::string(argv[1]);
    std::printf("User: Setting configuration filename to %s\n",
                conf_file.c_str());
  } else {
    conf_file = FLAGS_conf_file;
  }

  std::unique_ptr<Config> cfg = std::make_unique<Config>(conf_file.c_str());
  cfg->GenData();

  int ret;
  try {
    SignalHandler signal_handler;

    // Register signal handler to handle kill signal
    signal_handler.SetupSignalHandlers();

    PinToCoreWithOffset(ThreadType::kMasterTX, 0, 0);
    agora_comm::ListLocalInterfaces();
    TestBsRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::kSoapySdrStream);
    TestBsRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::kSoapySdrSocket);
    TestUeRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::kSoapySdrStream);
    TestUeRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::kSoapySdrSocket);
    ret = EXIT_SUCCESS;
  } catch (SignalException& e) {
    std::cerr << "SignalException: " << e.what() << std::endl;
    ret = EXIT_FAILURE;
  }
  PrintCoreAssignmentSummary();
  gflags::ShutDownCommandLineFlags();
  AGORA_LOG_SHUTDOWN();
  return ret;
}

void TestBsRadioRx(Config* cfg, const uint32_t max_rx, Radio::RadioType type) {
  //Tested with 1 and 4 (!= 1 probably doesn't work after mods)
  constexpr size_t kRxPerSymbol = 1;
  const size_t total_radios = cfg->NumRadios();
  const size_t num_channels = cfg->NumChannels();
  const bool hw_framer = cfg->HwFramer();
  const size_t radio_lo = 0;
  const size_t radio_hi = total_radios;
  const size_t cell_id = 0;
  const size_t target_samples = cfg->SampsPerSymbol() / kRxPerSymbol;
  RtAssert(
      (cfg->SampsPerSymbol() % kRxPerSymbol) == 0,
      "Target must be a multiple of samples per symbol for hw framer mode!");
  // Radio::RxFlags::RxFlagNone | RxFlagNone
  auto rx_flag = Radio::RxFlags::kRxFlagNone;
  // Using RxFlagNone with a soapy radio will sometimes return sub symbol packets

  std::cout << "Testing " << total_radios << " radios with " << num_channels
            << " channels" << std::endl;
  size_t num_rx_symbols = 0;

  //Create the memory for the rx samples
  //Create Packet buffers for a better test
  std::vector<std::vector<std::vector<std::byte>>> rx_buffers(
      total_radios, std::vector<std::vector<std::byte>>(
                        num_channels, std::vector<std::byte>(
                                          cfg->PacketLength(), std::byte(0))));

  std::vector<RxPacket> packets;
  for (size_t radio = 0; radio < total_radios; radio++) {
    for (size_t ch = 0; ch < num_channels; ch++) {
      auto* pkt_loc =
          reinterpret_cast<Packet*>(rx_buffers.at(radio).at(ch).data());
      packets.emplace_back(pkt_loc);
    }
  }

  std::vector<TxRxWorkerRx::RxStatusTracker> rx_status;
  size_t pkt_idx = 0;
  rx_status.resize(total_radios, TxRxWorkerRx::RxStatusTracker(num_channels));
  std::vector<RxPacket*> rx_packets(num_channels);
  for (auto& status : rx_status) {
    for (auto& new_packet : rx_packets) {
      new_packet = &packets.at(pkt_idx);
      AGORA_LOG_TRACE("TestBsRadioRx: Using Packet at location %d\n",
                      reinterpret_cast<intptr_t>(new_packet));
      pkt_idx++;
    }
    //Allocate memory for each interface / channel
    status.Reset(rx_packets);
  }

  if (kUseArgos) {
    // Makes the soapy remote "HUB" / InitBsRadio / ConfigureBsRadio
    auto radioconfig = std::make_unique<RadioConfig>(cfg, type);
    radioconfig->RadioStart();

    //std::this_thread::sleep_for(std::chrono::seconds(2));
    //Radio Trigger (start rx)
    radioconfig->Go();
    //Sometimes messages are "stuck, until the hub is triggered?"

    //Super thread loop
    while ((SignalHandler::GotExitSignal() == false) &&
           (num_rx_symbols < max_rx)) {
      for (auto radio = radio_lo; radio < radio_hi; radio++) {
        long long rx_time;

        auto& rx_info = rx_status.at(radio);
        //Need to modify so it works with multiple rx calls (target != samples_per_symbol)
        const size_t request_samples =
            target_samples - rx_info.SamplesAvailable();
        RtAssert(target_samples > rx_info.SamplesAvailable(),
                 "Requested Samples must be > 0");
        auto rx_locations = rx_info.GetRxPtrs();

        //In Hw framer mode, we should not call RX for bytes over the symbol boundaries
        const int rx_return = radioconfig->RadioRx(
            radio, rx_locations, request_samples, rx_flag, rx_time);

        if (rx_return > 0) {
          const auto new_samples = static_cast<size_t>(rx_return);
          AGORA_LOG_TRACE(
              "Called radiorx for %zu samples and received %zu with %zu "
              "already loaded at time %lld\n",
              request_samples, new_samples, rx_info.SamplesAvailable(),
              rx_time);

          rx_info.Update(new_samples, rx_time);
          if (new_samples < request_samples) {
            if (rx_flag == Radio::RxFlags::kEndReceive) {
              AGORA_LOG_WARN(
                  "Radio[%zu] : Received less than symbol amount of samples "
                  "%zu:%zu:%zu rx time %lld (Frame %zu, Symbol %zu)\n",
                  radio, new_samples, request_samples,
                  rx_info.SamplesAvailable(), rx_time,
                  static_cast<size_t>(rx_time >> 32),
                  static_cast<size_t>((rx_time >> 16) & 0xFFFF));
              num_rx_symbols++;
              rx_info.Reset();
              //throw std::runtime_error("Why are there not enough samples?");
              //This is an error.... should probably dump samples maybe?
            } else {
              AGORA_LOG_TRACE(
                  "Received less than symbol amount of samples %zu:%zu:%zu rx "
                  "time %lld (Frame %zu, Symbol %zu) - Retrying\n",
                  new_samples, request_samples, rx_info.SamplesAvailable(),
                  rx_time, static_cast<size_t>(rx_time >> 32),
                  static_cast<size_t>((rx_time >> 16) & 0xFFFF));
            }
          } else if (new_samples == request_samples) {
            //Rx data.....
            size_t frame_id = 0;
            size_t symbol_id = num_rx_symbols;
            if (hw_framer) {
              frame_id = static_cast<size_t>(rx_time >> 32);
              symbol_id = static_cast<size_t>((rx_time >> 16) & 0xFFFF);
            }

            //There will be NumChannels "Packets" at this spot
            auto rx_mem = rx_info.GetRxPackets();
            for (size_t ch = 0; ch < num_channels; ch++) {
              auto* rx_packet = rx_mem.at(ch);
              auto* raw_pkt = rx_packet->RawPacket();
              new (raw_pkt) Packet(frame_id, symbol_id, cell_id,
                                   (radio * num_channels) + ch);

              std::printf("Rx Packet: %s Rx samples: %zu:%zu\n",
                          raw_pkt->ToString().c_str(),
                          rx_info.SamplesAvailable(), cfg->SampsPerSymbol());
            }
            num_rx_symbols++;
            //Reset stats, reuse the old rx locations
            rx_info.Reset();
          } else if (new_samples > request_samples) {
            throw std::runtime_error(
                "RX samples exceeds the requested samples");
          }
        } else if (rx_return < 0) {
          AGORA_LOG_ERROR("Radio rx error %d - message %s\n", rx_return,
                          strerror(errno));
          throw std::runtime_error("Radio rx error!!");
        }  // end new samples
      }    // end for each radio
    }      // while no exit signal
    radioconfig->RadioStop();
  } else {
    AGORA_LOG_ERROR(
        "Hardware is not enabled in the compile settings.  Please fix and "
        "try again!\n");
  }
}

//Removed the hw_framer logic for this test
//Good logic for short timeout (no hw framer) radio testing
void TestUeRadioRx(Config* cfg, const uint32_t max_rx, Radio::RadioType type) {
  constexpr size_t kRxPerSymbol = 1;
  const size_t total_radios = cfg->UeNum();
  const size_t num_channels = cfg->NumUeChannels();
  const bool hw_framer = cfg->UeHwFramer();
  const size_t radio_lo = 0;
  const size_t radio_hi = total_radios;
  const size_t cell_id = 0;
  const size_t target_samples = cfg->SampsPerSymbol();
  // Radio::RxFlags::RxFlagNone | EndReceive
  auto rx_flag = Radio::RxFlags::kRxFlagNone;

  RtAssert(
      (cfg->SampsPerSymbol() % kRxPerSymbol) == 0,
      "Target must be a multiple of samples per symbol for hw framer mode!");

  // Using RxFlagNone with a soapy radio will sometimes return sub symbol packets

  std::cout << "Testing " << total_radios << " radios with " << num_channels
            << " channels" << std::endl;
  size_t num_rx_symbols = 0;

  //Create the memory for the rx samples
  //Create Packet buffers for a better test
  std::vector<std::vector<std::vector<std::byte>>> rx_buffers(
      total_radios, std::vector<std::vector<std::byte>>(
                        num_channels, std::vector<std::byte>(
                                          cfg->PacketLength(), std::byte(0))));

  std::vector<RxPacket> packets;
  for (size_t radio = 0; radio < total_radios; radio++) {
    for (size_t ch = 0; ch < num_channels; ch++) {
      auto* pkt_loc =
          reinterpret_cast<Packet*>(rx_buffers.at(radio).at(ch).data());
      packets.emplace_back(pkt_loc);
    }
  }

  std::vector<TxRxWorkerRx::RxStatusTracker> rx_status;
  size_t pkt_idx = 0;
  rx_status.resize(total_radios, TxRxWorkerRx::RxStatusTracker(num_channels));
  std::vector<RxPacket*> rx_packets(num_channels);
  for (auto& status : rx_status) {
    for (auto& new_packet : rx_packets) {
      new_packet = &packets.at(pkt_idx);
      AGORA_LOG_TRACE("TestBsRadioRx: Using Packet at location %d\n",
                      reinterpret_cast<intptr_t>(new_packet));
      pkt_idx++;
    }
    //Allocate memory for each interface / channel
    status.Reset(rx_packets);
  }

  if (kUseArgos) {
    // Makes the soapy remote "HUB" / InitBsRadio / ConfigureBsRadio
    auto radioconfig = std::make_unique<ClientRadioConfig>(cfg, type);
    radioconfig->RadioStart();

    //std::this_thread::sleep_for(std::chrono::seconds(2));
    //Radio Trigger (start rx)
    radioconfig->Go();
    //Sometimes messages are "stuck, until the hub is triggered?"

    //Super thread loop
    while ((SignalHandler::GotExitSignal() == false) &&
           (num_rx_symbols < max_rx)) {
      for (auto radio = radio_lo; radio < radio_hi; radio++) {
        long long rx_time;

        auto& rx_info = rx_status.at(radio);
        //Need to modify so it works with multiple rx calls (target != samples_per_symbol)
        const size_t request_samples =
            target_samples - rx_info.SamplesAvailable();
        RtAssert(target_samples > rx_info.SamplesAvailable(),
                 "Requested Samples must be > 0");
        auto rx_locations = rx_info.GetRxPtrs();

        //In Hw framer mode, we should not call RX for bytes over the symbol boundaries
        const int rx_return = radioconfig->RadioRx(
            radio, rx_locations, request_samples, rx_flag, rx_time);

        if (rx_return > 0) {
          const auto new_samples = static_cast<size_t>(rx_return);
          AGORA_LOG_TRACE(
              "Called radiorx for %zu samples and received %zu with %zu "
              "already loaded\n",
              request_samples, new_samples, rx_info.SamplesAvailable());

          rx_info.Update(new_samples, rx_time);
          if (new_samples < request_samples) {
            if (rx_flag == Radio::RxFlags::kEndReceive) {
              AGORA_LOG_WARN(
                  "Received less than symbol amount of samples %zu:%zu:%zu rx "
                  "time %lld (Frame %zu, Symbol %zu)\n",
                  new_samples, request_samples, rx_info.SamplesAvailable(),
                  rx_time, static_cast<size_t>(rx_time >> 32),
                  static_cast<size_t>((rx_time >> 16) & 0xFFFF));
              num_rx_symbols++;
              rx_info.Reset();
              //throw std::runtime_error("Why are there not enough samples?");
              //This is an error.... should probably dump samples maybe?
            } else {
              AGORA_LOG_TRACE(
                  "Received less than symbol amount of samples %zu:%zu:%zu rx "
                  "time %lld (Frame %zu, Symbol %zu) - Retrying\n",
                  new_samples, request_samples, rx_info.SamplesAvailable(),
                  rx_time, 0, num_rx_symbols);
            }
          } else if (new_samples == request_samples) {
            //Rx data.....
            size_t frame_id = 0;
            size_t symbol_id = num_rx_symbols;
            if (hw_framer) {
              frame_id = static_cast<size_t>(rx_time >> 32);
              symbol_id = static_cast<size_t>((rx_time >> 16) & 0xFFFF);
            }

            //There will be NumChannels "Packets" at this spot
            auto rx_mem = rx_info.GetRxPackets();
            for (size_t ch = 0; ch < num_channels; ch++) {
              auto* rx_packet = rx_mem.at(ch);
              auto* raw_pkt = rx_packet->RawPacket();
              new (raw_pkt) Packet(frame_id, symbol_id, cell_id,
                                   (radio * num_channels) + ch);

              std::printf("Rx Packet: %s Rx samples: %zu:%zu\n",
                          raw_pkt->ToString().c_str(),
                          rx_info.SamplesAvailable(), cfg->SampsPerSymbol());
            }
            num_rx_symbols++;
            //Reset stats, reuse the old rx locations
            rx_info.Reset();
          } else if (new_samples > request_samples) {
            throw std::runtime_error(
                "RX samples exceeds the requested samples");
          }
        } else if (rx_return < 0) {
          AGORA_LOG_ERROR("Radio rx error %d - message %s\n", rx_return,
                          strerror(errno));
          throw std::runtime_error("Radio rx error!!");
        }  // end new samples
      }    // end for each radio
    }      // while no exit signal
    radioconfig->RadioStop();
  } else {
    AGORA_LOG_ERROR(
        "Hardware is not enabled in the compile settings.  Please fix and "
        "try again!\n");
  }
}

ssize_t SyncBeacon(const std::complex<int16_t>* samples, size_t detect_window,
                   const Config* cfg) {
  ssize_t sync_index = 0;
  sync_index = CommsLib::FindBeaconAvx(samples, cfg->GoldCf32(), detect_window);

  if (sync_index >= 0) {
    auto rx_adjust_samples =
        sync_index - cfg->BeaconLen() - cfg->OfdmTxZeroPrefix();
    AGORA_LOG_INFO(
        "RadioTest - Beacon detected for at sync_index: %ld, rx sample offset: "
        "%ld\n",
        sync_index, rx_adjust_samples);
  } else {
    AGORA_LOG_INFO("RadioTest - No beacon detected %ld in window %zu\n",
                   sync_index, detect_window);
  }
  return sync_index;
}