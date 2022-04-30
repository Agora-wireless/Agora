/**
 * @file main.cc
 * @brief Main file for the radio test program
 */
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
void TestRadioRxStream(Config* cfg, const uint32_t max_rx);
void TestRadioRxSocket(Config* cfg, const uint32_t max_rx);

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

    agora_comm::ListLocalInterfaces();
    TestRadioRxStream(cfg.get(), FLAGS_rx_symbols);
    TestRadioRxSocket(cfg.get(), FLAGS_rx_symbols);
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

void TestRadioRxStream(Config* cfg, const uint32_t max_rx) {
  const size_t total_radios = cfg->NumRadios();
  const size_t radio_lo = 0;
  const size_t radio_hi = total_radios;
  const size_t cell_id = 0;

  PinToCoreWithOffset(ThreadType::kMasterTX, 0, 0);
  std::cout << "Testing " << total_radios << " radios " << std::endl;

  //Create the memory for the rx samples
  //Radio x Channels x SamplesPerSymbol, filled with 0 + 0i
  std::vector<std::vector<std::vector<std::complex<int16_t>>>> rx_buffer(
      total_radios, std::vector<std::vector<std::complex<int16_t>>>(
                        cfg->NumChannels(), std::vector<std::complex<int16_t>>(
                                                cfg->SampsPerSymbol(),
                                                std::complex<int16_t>(0, 0))));

  //Radio x Channels
  std::vector<std::vector<void*>> rx_buffs(
      total_radios, std::vector<void*>(cfg->NumChannels(), nullptr));
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
      cfg->NumChannels(),
      std::vector<std::byte>(cfg->PacketLength(), std::byte(0)));

  if (kUseArgos) {
    // Makes the soapy remote "HUB" / InitBsRadio / ConfigureBsRadio
    auto radioconfig =
        std::make_unique<RadioConfig>(cfg, Radio::SoapySdrStream);
    radioconfig->RadioStart();

    //Radio Trigger (start rx)
    radioconfig->Go();

    uint32_t num_rx_symbols = 0;
    //Super thread loop
    while ((SignalHandler::GotExitSignal() == false) &&
           (num_rx_symbols < max_rx)) {
      for (auto radio = radio_lo; radio < radio_hi; radio++) {
        long long rx_time;
        int rx_samples = radioconfig->RadioRx(
            radio, rx_buffs.at(radio), cfg->SampsPerSymbol(), 0, rx_time);
        if (rx_samples > 0) {
          //Rx data.....
          size_t frame_id = 0;
          size_t symbol_id = 0;
          if (cfg->HwFramer() == true) {
            frame_id = static_cast<size_t>(rx_time >> 32);
            symbol_id = static_cast<size_t>((rx_time >> 16) & 0xFFFF);
          }

          //There will be NumChannels "Packets" at this spot
          for (size_t ch = 0; ch < cfg->NumChannels(); ch++) {
            Packet* rx_packet =
                reinterpret_cast<Packet*>(packet_buffer.at(ch).data());
            new (rx_packet) Packet(frame_id, symbol_id, cell_id,
                                   (radio * cfg->NumChannels()) + ch);

            std::printf("Rx Packet: %s Rx samples: %d:%zu\n",
                        rx_packet->ToString().c_str(), rx_samples,
                        cfg->SampsPerSymbol());
          }
          num_rx_symbols++;
        } else if (rx_samples < 0) {
          std::printf("Radio rx error %d - message %s\n", rx_samples,
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

void TestRadioRxSocket(Config* cfg, const uint32_t max_rx) {
  const size_t total_radios = cfg->NumRadios();
  const size_t radio_lo = 0;
  const size_t radio_hi = total_radios;
  const size_t cell_id = 0;

  PinToCoreWithOffset(ThreadType::kMasterTX, 0, 0);
  std::cout << "Testing " << total_radios << " radios " << std::endl;

  //Create the memory for the rx samples
  //Radio x Channels x SamplesPerSymbol, filled with 0 + 0i
  std::vector<std::vector<std::vector<std::complex<int16_t>>>> rx_buffer(
      total_radios, std::vector<std::vector<std::complex<int16_t>>>(
                        cfg->NumChannels(), std::vector<std::complex<int16_t>>(
                                                cfg->SampsPerSymbol(),
                                                std::complex<int16_t>(0, 0))));

  //Memory for 1 packet for each channel (could be extended for each radio if necessary)
  // Channels x PacketLength (bytes)
  std::vector<std::vector<std::byte>> packet_buffer(
      cfg->NumChannels(),
      std::vector<std::byte>(cfg->PacketLength(), std::byte(0)));

  if (kUseArgos) {
    // Makes the soapy remote "HUB" / InitBsRadio / ConfigureBsRadio
    auto radioconfig =
        std::make_unique<RadioConfig>(cfg, Radio::SoapySdrSocket);
    radioconfig->RadioStart();

    //Radio Trigger (start rx)
    radioconfig->Go();

    uint32_t num_rx_symbols = 0;
    //Super thread loop
    while ((SignalHandler::GotExitSignal() == false) &&
           (num_rx_symbols < max_rx)) {
      for (auto radio = radio_lo; radio < radio_hi; radio++) {
        long long rx_time;
        const int rx_samples = radioconfig->RadioRx(
            radio, rx_buffer.at(radio), cfg->SampsPerSymbol(), 0, rx_time);
        if (rx_samples > 0) {
          //Rx data.....
          size_t frame_id = 0;
          size_t symbol_id = 0;
          if (cfg->HwFramer() == true) {
            frame_id = static_cast<size_t>(rx_time >> 32);
            symbol_id = static_cast<size_t>((rx_time >> 16) & 0xFFFF);
          }

          //There will be NumChannels "Packets" at this spot
          for (size_t ch = 0; ch < cfg->NumChannels(); ch++) {
            Packet* rx_packet =
                reinterpret_cast<Packet*>(packet_buffer.at(ch).data());
            new (rx_packet) Packet(frame_id, symbol_id, cell_id,
                                   (radio * cfg->NumChannels()) + ch);

            std::printf("Rx Packet: %s Rx samples: %d:%zu\n",
                        rx_packet->ToString().c_str(), rx_samples,
                        cfg->SampsPerSymbol());
          }
          num_rx_symbols++;
        } else if (rx_samples < 0) {
          std::printf("Radio rx error %d - message %s\n", rx_samples,
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