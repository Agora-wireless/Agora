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
#include "signal_handler.h"
#include "version_config.h"

DEFINE_string(conf_file,
              TOSTRING(PROJECT_DIRECTORY) "/data/ul-one-vulture.json",
              "Config filename");

DEFINE_uint32(rx_symbols, 10,
              "The number of symbols to receive before the program terminates");

//Forward declaration
static void TestBsRadioRx(Config* cfg, const uint32_t max_rx,
                          Radio::RadioType type);
static void TestBsRadioRxNoRetry(Config* cfg, const uint32_t max_rx,
                                 Radio::RadioType type);
static void TestUeRadioRx(Config* cfg, const uint32_t max_rx,
                          Radio::RadioType type);
static ssize_t SyncBeacon(const std::complex<int16_t>* samples,
                          size_t detect_window, const Config* cfg);

static bool HwFramerMatch(long long old_time, long long new_time);

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
    //TestBsRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::SoapySdrStream);
    //TestBsRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::SoapySdrSocket);
    TestBsRadioRxNoRetry(cfg.get(), FLAGS_rx_symbols, Radio::SoapySdrSocket);
    //TestUeRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::SoapySdrStream);
    //UE socket doesnt work right now...
    //TestUeRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::SoapySdrSocket);
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
  const size_t total_radios = cfg->NumRadios();
  const size_t num_channels = cfg->NumChannels();
  const size_t hw_framer = cfg->HwFramer();
  const size_t radio_lo = 0;
  const size_t radio_hi = total_radios;
  const size_t cell_id = 0;
  const size_t target_samples = cfg->SampsPerSymbol();
  // Radio::RxFlagCompleteSymbol | RxFlagNone
  const auto rx_flag = Radio::RxFlagCompleteSymbol;
  // Using RxFlagNone with a soapy radio will sometimes return sub symbol packets

  std::cout << "Testing " << total_radios << " radios with " << num_channels
            << " channels" << std::endl;

  //Create the memory for the rx samples
  //Radio x Channels x SamplesPerSymbol, filled with 0 + 0i
  std::vector<std::vector<std::vector<std::complex<int16_t>>>> rx_buffer(
      total_radios,
      std::vector<std::vector<std::complex<int16_t>>>(
          num_channels, std::vector<std::complex<int16_t>>(
                            target_samples, std::complex<int16_t>(0, 0))));

  //Radio x Channels
  std::vector<std::vector<void*>> rx_buffs(
      total_radios, std::vector<void*>(num_channels, nullptr));
  //Setup pointers to all of the memory rx sections
  for (auto radio = radio_lo; radio < radio_hi; radio++) {
    const size_t radio_channels = rx_buffs.at(radio).size();
    for (size_t ch = 0; ch < radio_channels; ch++) {
      rx_buffs.at(radio).at(ch) = rx_buffer.at(radio).at(ch).data();
    }
  }

  //Memory for 1 packet for each channel (could be extended for each radio if necessary)
  // Channels x PacketLength (bytes)
  std::vector<std::vector<std::byte>> packet_buffer(
      num_channels, std::vector<std::byte>(cfg->PacketLength(), std::byte(0)));

  if (kUseArgos) {
    // Makes the soapy remote "HUB" / InitBsRadio / ConfigureBsRadio
    auto radioconfig = std::make_unique<RadioConfig>(cfg, type);
    radioconfig->RadioStart();

    //Radio Trigger (start rx)
    radioconfig->Go();

    size_t num_rx_symbols = 0;
    size_t rx_samples = 0;
    size_t rx_calls_per_symbol = 0;
    long long first_rx_time_of_symbol = 0;

    std::vector<void*> rx_locations(num_channels);
    //Super thread loop
    while ((SignalHandler::GotExitSignal() == false) &&
           (num_rx_symbols < max_rx)) {
      for (auto radio = radio_lo; radio < radio_hi; radio++) {
        long long rx_time;

        for (size_t ch = 0; ch < num_channels; ch++) {
          rx_locations.at(ch) = &rx_buffer.at(radio).at(ch).at(rx_samples);
        }

        const size_t rx_size = target_samples - rx_samples;
        const int new_samples = radioconfig->RadioRx(radio, rx_locations,
                                                     rx_size, rx_flag, rx_time);

        if (new_samples > 0) {
          AGORA_LOG_TRACE("Called radiorx for %zu samples and received %d\n",
                          rx_size, new_samples);
          rx_samples += static_cast<size_t>(new_samples);
          if (rx_samples < target_samples) {
            //Symbol Rx not finished.... Adjust the subsequent reads
            if (hw_framer) {
              if (rx_calls_per_symbol == 0) {
                first_rx_time_of_symbol = rx_time;
              } else if (rx_time != 0) {
                //rx_time == 0 indicates we can use the last rx time
                const size_t rx_frame_id = static_cast<size_t>(rx_time >> 32);
                const size_t rx_symbol_id =
                    static_cast<size_t>((rx_time >> 16) & 0xFFFF);

                const size_t first_frame_id =
                    static_cast<size_t>(first_rx_time_of_symbol >> 32);
                const size_t first_symbol_id = static_cast<size_t>(
                    (first_rx_time_of_symbol >> 16) & 0xFFFF);

                if ((rx_frame_id != first_frame_id) ||
                    (rx_symbol_id != first_symbol_id)) {
                  std::printf(
                      "Unexpected Frame | Symbol pair during retry #%zu Frame: "
                      "%zu:%zu Symbol: %zu:%zu received new samples %d with "
                      "%zu total\n",
                      rx_calls_per_symbol, rx_frame_id, first_frame_id,
                      rx_symbol_id, first_symbol_id, new_samples, rx_samples);
                  //Now we have mixed up samples.....
                  //Need to shift back here??
                  rx_samples = new_samples;
                  rx_calls_per_symbol = SIZE_MAX;
                  first_rx_time_of_symbol = rx_time;
                }
              }
            }
            rx_calls_per_symbol++;
            //std::printf(
            //    "Received less than symbol amount of samples %d:%zu:%zu\n",
            //    new_samples, rx_samples, target_samples);
            //rx_samples = 0;
          } else if (rx_samples == target_samples) {
            //Rx data.....
            size_t frame_id = 0;
            size_t symbol_id = num_rx_symbols;
            if (hw_framer) {
              if (rx_calls_per_symbol > 0) {
                frame_id = static_cast<size_t>(first_rx_time_of_symbol >> 32);
                symbol_id = static_cast<size_t>(
                    (first_rx_time_of_symbol >> 16) & 0xFFFF);

                const size_t rx_frame_id = static_cast<size_t>(rx_time >> 32);
                const size_t rx_symbol_id =
                    static_cast<size_t>((rx_time >> 16) & 0xFFFF);

                //rx == 0 means we previously read the start of the frame and there was data still pending.
                if ((rx_time != 0) && ((frame_id != rx_frame_id) ||
                                       (symbol_id != rx_symbol_id))) {
                  std::printf(
                      "Unexpected Frame | Symbol pair with complete samples "
                      "%zu rx's - Frame: %zu:%zu Symbol: %zu:%zu received new "
                      "samples %d with %zu total  at time %lld starting %lld\n",
                      rx_calls_per_symbol + 1, rx_frame_id, frame_id,
                      rx_symbol_id, symbol_id, new_samples, rx_samples, rx_time,
                      first_rx_time_of_symbol);
                  //Now we have mixed up samples.....
                  //Need to shift data back  in buffer here!!!
                  rx_samples = new_samples;
                  rx_calls_per_symbol = 1;
                  first_rx_time_of_symbol = rx_time;
                  num_rx_symbols++;
                }
              } else {
                frame_id = static_cast<size_t>(rx_time >> 32);
                symbol_id = static_cast<size_t>((rx_time >> 16) & 0xFFFF);
              }
            }

            if (rx_samples == target_samples) {
              //There will be NumChannels "Packets" at this spot
              for (size_t ch = 0; ch < num_channels; ch++) {
                Packet* rx_packet =
                    reinterpret_cast<Packet*>(packet_buffer.at(ch).data());
                new (rx_packet) Packet(frame_id, symbol_id, cell_id,
                                       (radio * num_channels) + ch);

                std::printf("Rx Packet: %s Rx samples: %zu:%zu retries %zu\n",
                            rx_packet->ToString().c_str(), rx_samples,
                            target_samples, rx_calls_per_symbol);
              }
              num_rx_symbols++;
              rx_samples -= target_samples;
              rx_calls_per_symbol = 0;
            }
          } else {
            std::printf("Received too much data....%zu:%zu\n", rx_samples,
                        target_samples);
          }
        } else if (new_samples < 0) {
          std::printf("Radio rx error %d - message %s\n", new_samples,
                      strerror(errno));
          std::fflush(stdout);
          throw std::runtime_error("Radio rx error!!");
        }
      }  // end for each radio
    }    // while no exit signal
    radioconfig->RadioStop();
  } else {
    std::cout << "Hardware is not enabled in the compile settings.  Please fix "
                 "and try again!"
              << std::endl;
  }
}

void TestBsRadioRxNoRetry(Config* cfg, const uint32_t max_rx,
                          Radio::RadioType type) {
  //Tested with 1 and 4
  constexpr size_t kRxPerSymbol = 1;
  const size_t total_radios = cfg->NumRadios();
  const size_t num_channels = cfg->NumChannels();
  const size_t hw_framer = cfg->HwFramer();
  const size_t radio_lo = 0;
  const size_t radio_hi = total_radios;
  const size_t cell_id = 0;
  const size_t target_samples = cfg->SampsPerSymbol() / kRxPerSymbol;
  RtAssert(
      (cfg->SampsPerSymbol() % kRxPerSymbol) == 0,
      "Target must be a multiple of samples per symbol for hw framer mode!");
  // Radio::RxFlagCompleteSymbol | RxFlagNone
  const auto rx_flag = Radio::RxFlagNone;
  // Using RxFlagNone with a soapy radio will sometimes return sub symbol packets

  std::cout << "Testing " << total_radios << " radios with " << num_channels
            << " channels" << std::endl;
  size_t num_rx_symbols = 0;
  size_t rx_samples = 0;

  //Create the memory for the rx samples
  //Radio x Channels x SamplesPerSymbol, filled with 0 + 0i
  std::vector<std::vector<std::vector<std::complex<int16_t>>>> rx_buffer(
      total_radios, std::vector<std::vector<std::complex<int16_t>>>(
                        num_channels, std::vector<std::complex<int16_t>>(
                                          cfg->SampsPerSymbol(),
                                          std::complex<int16_t>(0, 0))));

  //Radio x Channels
  std::vector<std::vector<void*>> rx_buffs(
      total_radios, std::vector<void*>(num_channels, nullptr));
  //Setup pointers to all of the memory rx sections
  for (auto radio = radio_lo; radio < radio_hi; radio++) {
    const size_t radio_channels = rx_buffs.at(radio).size();
    for (size_t ch = 0; ch < radio_channels; ch++) {
      rx_buffs.at(radio).at(ch) = rx_buffer.at(radio).at(ch).data();
    }
  }

  //Memory for 1 packet for each channel (could be extended for each radio if necessary)
  // Channels x PacketLength (bytes)
  std::vector<std::vector<std::byte>> packet_buffer(
      num_channels, std::vector<std::byte>(cfg->PacketLength(), std::byte(0)));

  if (kUseArgos) {
    // Makes the soapy remote "HUB" / InitBsRadio / ConfigureBsRadio
    auto radioconfig = std::make_unique<RadioConfig>(cfg, type);
    radioconfig->RadioStart();

    //Radio Trigger (start rx)
    radioconfig->Go();

    std::vector<void*> rx_locations(num_channels);
    //Super thread loop
    while ((SignalHandler::GotExitSignal() == false) &&
           (num_rx_symbols < max_rx)) {
      for (auto radio = radio_lo; radio < radio_hi; radio++) {
        long long rx_time;

        for (size_t ch = 0; ch < num_channels; ch++) {
          rx_locations.at(ch) = &rx_buffer.at(radio).at(ch).at(rx_samples);
        }

        const size_t rx_size = target_samples;
        //In Hw framer mode, we should not call RX for bytes over the symbol boundaries
        const int rx_return = radioconfig->RadioRx(radio, rx_locations, rx_size,
                                                   rx_flag, rx_time);

        if (rx_return > 0) {
          const auto new_samples = static_cast<size_t>(rx_return);
          AGORA_LOG_INFO(
              "Called radiorx for %zu samples and received %zu with %zu "
              "already loaded\n",
              rx_size, new_samples, rx_samples);

          rx_samples += static_cast<size_t>(new_samples);
          if (new_samples < target_samples) {
            AGORA_LOG_INFO(
                "Received less than symbol amount of samples %zu:%zu:%zu rx "
                "time %lld\n",
                new_samples, target_samples, rx_samples, rx_time);
            num_rx_symbols++;
            rx_samples = 0;
            //This is an error.... should probably dump samples maybe?
          } else if (rx_samples == cfg->SampsPerSymbol()) {
            //Rx data.....
            size_t frame_id = 0;
            size_t symbol_id = num_rx_symbols;
            if (hw_framer) {
              frame_id = static_cast<size_t>(rx_time >> 32);
              symbol_id = static_cast<size_t>((rx_time >> 16) & 0xFFFF);
            }

            //There will be NumChannels "Packets" at this spot
            for (size_t ch = 0; ch < num_channels; ch++) {
              Packet* rx_packet =
                  reinterpret_cast<Packet*>(packet_buffer.at(ch).data());
              new (rx_packet) Packet(frame_id, symbol_id, cell_id,
                                     (radio * num_channels) + ch);

              std::printf("Rx Packet: %s Rx samples: %zu:%zu\n",
                          rx_packet->ToString().c_str(), rx_samples,
                          target_samples);
            }
            num_rx_symbols++;
            rx_samples = 0;
          } else if (rx_samples > cfg->SampsPerSymbol()) {
            throw std::runtime_error("RX samples exceeds samples per symbol");
          }
        } else if (rx_return < 0) {
          std::printf("Radio rx error %d - message %s\n", rx_return,
                      strerror(errno));
          std::fflush(stdout);
          throw std::runtime_error("Radio rx error!!");
        }  // end new samples
      }    // end for each radio
    }      // while no exit signal
    radioconfig->RadioStop();
  } else {
    std::cout << "Hardware is not enabled in the compile settings.  Please fix "
                 "and try again!"
              << std::endl;
  }
}

//Removed the hw_framer logic for this test
void TestUeRadioRx(Config* cfg, const uint32_t max_rx, Radio::RadioType type) {
  const size_t total_radios = cfg->UeNum();
  const size_t num_channels = cfg->NumUeChannels();
  // /const size_t hw_framer = cfg->UeHwFramer();
  const size_t radio_lo = 0;
  const size_t radio_hi = total_radios;
  const size_t cell_id = 0;
  const size_t target_samples = cfg->SampsPerSymbol();
  // Radio::RxFlagCompleteSymbol | RxFlagNone
  const auto rx_flag = Radio::RxFlagNone;

  std::cout << "Testing " << total_radios << " radios " << std::endl;

  //Create the memory for the rx samples
  //Radio x Channels x SamplesPerSymbol, filled with 0 + 0i
  std::vector<std::vector<std::vector<std::complex<int16_t>>>> rx_buffer(
      total_radios,
      std::vector<std::vector<std::complex<int16_t>>>(
          num_channels, std::vector<std::complex<int16_t>>(
                            target_samples, std::complex<int16_t>(0, 0))));

  //Radio x Channels
  std::vector<std::vector<void*>> rx_buffs(
      total_radios, std::vector<void*>(num_channels, nullptr));
  //Setup pointers to all of the memory rx sections
  for (auto radio = radio_lo; radio < radio_hi; radio++) {
    const size_t radio_channels = rx_buffs.at(radio).size();
    for (size_t ch = 0; ch < radio_channels; ch++) {
      rx_buffs.at(radio).at(ch) = rx_buffer.at(radio).at(ch).data();
    }
  }

  //Memory for 1 packet for each channel (could be extended for each radio if necessary)
  // Channels x PacketLength (bytes)
  std::vector<std::vector<std::byte>> packet_buffer(
      num_channels, std::vector<std::byte>(cfg->PacketLength(), std::byte(0)));

  if (kUseArgos) {
    auto radioconfig = std::make_unique<ClientRadioConfig>(cfg, type);
    radioconfig->RadioStart();

    //Radio Trigger (start rx)
    //Why is the trigger not needed here?  Maybe the channel didn't activate properly
    radioconfig->Go();

    size_t num_rx_symbols = 0;
    size_t rx_samples = 0;
    size_t rx_calls_per_symbol = 0;

    std::vector<void*> rx_locations(num_channels);
    //Super thread loop
    while ((SignalHandler::GotExitSignal() == false) &&
           (num_rx_symbols < max_rx)) {
      for (auto radio = radio_lo; radio < radio_hi; radio++) {
        long long rx_time;

        for (size_t ch = 0; ch < num_channels; ch++) {
          rx_locations.at(ch) = &rx_buffer.at(radio).at(ch).at(rx_samples);
        }

        const size_t rx_size = target_samples - rx_samples;
        const int new_samples = radioconfig->RadioRx(radio, rx_locations,
                                                     rx_size, rx_flag, rx_time);

        if (new_samples > 0) {
          //std::printf("Called radiorx for %zu samples and received %d\n",
          //            rx_size, new_samples);
          rx_samples += static_cast<size_t>(new_samples);
          if (rx_samples < target_samples) {
            rx_calls_per_symbol++;
            AGORA_LOG_INFO("Received %zu:%zu samples\n", rx_samples,
                           target_samples);
          } else if (rx_samples == target_samples) {
            //Rx data.....
            size_t frame_id = 0;
            size_t symbol_id = num_rx_symbols;

            //There will be NumChannels "Packets" at this spot
            for (size_t ch = 0; ch < num_channels; ch++) {
              Packet* rx_packet =
                  reinterpret_cast<Packet*>(packet_buffer.at(ch).data());
              new (rx_packet) Packet(frame_id, symbol_id, cell_id,
                                     (radio * num_channels) + ch);

              AGORA_LOG_INFO("Rx Packet: %s Rx samples: %zu:%zu retries %zu\n",
                             rx_packet->ToString().c_str(), rx_samples,
                             target_samples, rx_calls_per_symbol);
            }
            num_rx_symbols++;
            rx_samples -= target_samples;
            rx_calls_per_symbol = 0;
          } else {
            AGORA_LOG_ERROR("Received too much data....%zu:%zu\n", rx_samples,
                            target_samples);
          }
        } else if (new_samples < 0) {
          AGORA_LOG_ERROR("Radio rx error %d - message %s\n", new_samples,
                          strerror(errno));
          throw std::runtime_error("Radio rx error!!");
        }
      }  // end for each radio
    }    // while no exit signal
    radioconfig->RadioStop();
  } else {
    std::cout << "Hardware is not enabled in the compile settings.  Please fix "
                 "and try again!"
              << std::endl;
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

bool HwFramerMatch(long long old_time, long long new_time) {
  bool match;

  const auto old_frame_id = static_cast<size_t>(old_time >> 32);
  const auto old_symbol_id = static_cast<size_t>((old_time >> 16) & 0xFFFF);

  const size_t new_frame_id = static_cast<size_t>(new_time >> 32);
  const size_t new_symbol_id = static_cast<size_t>((new_time >> 16) & 0xFFFF);

  if ((old_time == 0) || (new_time == 0)) {
    AGORA_LOG_INFO("Old time %lld New Time %lld\n", old_time, new_time);
  }

  if ((old_frame_id == new_frame_id) && (old_symbol_id == new_symbol_id)) {
    match = true;
  } else {
    match = false;
    AGORA_LOG_INFO("Unexpected Frame: %zu:%zu Symbol: %zu:%zu received\n",
                   old_frame_id, new_frame_id, old_symbol_id, new_symbol_id);
  }
  return match;
}