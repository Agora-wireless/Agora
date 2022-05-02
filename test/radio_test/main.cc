/**
 * @file main.cc
 * @brief Main file for the radio test program
 */
#include "client_radio.h"
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
void TestBsRadioRx(Config* cfg, const uint32_t max_rx, Radio::RadioType type);
void TestUeRadioRx(Config* cfg, const uint32_t max_rx, Radio::RadioType type);

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
    TestBsRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::SoapySdrStream);
    //TestBsRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::SoapySdrSocket);
    //TestUeRadioRx(cfg.get(), FLAGS_rx_symbols, Radio::SoapySdrStream);
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
          //std::printf("Called radiorx for %zu samples and received %d\n",
          //            rx_size, new_samples);
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

void TestUeRadioRx(Config* cfg, const uint32_t max_rx, Radio::RadioType type) {
  const size_t total_radios = cfg->UeNum();
  const size_t num_channels = cfg->NumUeChannels();
  const size_t hw_framer = cfg->UeHwFramer();
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
    //radioconfig->Go();

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

        const int new_samples = radioconfig->RadioRx(
            radio, rx_locations, target_samples - rx_samples, rx_flag, rx_time);

        if (new_samples > 0) {
          rx_samples += static_cast<size_t>(new_samples);
          if (rx_samples < target_samples) {
            rx_calls_per_symbol++;
            //std::printf(
            //    "Received less than symbol amount of samples %d:%zu:%zu\n",
            //    new_samples, rx_samples, target_samples);
            //rx_samples = 0;
            //Symbol Rx not finished.... Adjust the subsequent reads
          } else if (rx_samples == target_samples) {
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

              std::printf("Rx Packet: %s Rx samples: %zu:%zu retries %zu\n",
                          rx_packet->ToString().c_str(), rx_samples,
                          target_samples, rx_calls_per_symbol);
            }
            num_rx_symbols++;
            rx_samples -= target_samples;
            rx_calls_per_symbol = 0;
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