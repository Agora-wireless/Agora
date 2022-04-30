/**
 * @file radio_data_plane_socket.cc
 * @brief Defination file for the RadioDataPlaneSocket Class
 */

#include "radio_data_plane_socket.h"

#include "SoapySDR/Formats.h"
#include "logger.h"
#include "radio_soapysdr.h"
#include "utils.h"

RadioDataPlaneSocket::RadioDataPlaneSocket() : RadioDataPlane() {}
RadioDataPlaneSocket::~RadioDataPlaneSocket() { Close(); }

void RadioDataPlaneSocket::Init(Radio* radio, const Config* cfg) {
  return RadioDataPlane::Init(radio, cfg);
}

inline void RadioDataPlaneSocket::Activate() {
  return RadioDataPlane::Activate();
}

inline void RadioDataPlaneSocket::Deactivate() {
  return RadioDataPlane::Deactivate();
}

inline void RadioDataPlaneSocket::Close() { return RadioDataPlane::Close(); }

void RadioDataPlaneSocket::Setup() {
  if (CheckMode() == RadioDataPlane::Mode::kModeShutdown) {
    SoapySDR::Device* soapy_device =
        dynamic_cast<RadioSoapySdr*>(radio_)->SoapyDevice();
    std::string stream_protocol = soapy_device->readSetting("STREAM_PROTOCOL");
    if (stream_protocol != "twbw64") {
      throw std::runtime_error("Stream protocol mismatch");
    }

    //query remote iris endpoint configuration
    auto remote_address = soapy_device->readSetting("ETH0_IPv6_ADDR");
    const auto remote_port = soapy_device->readSetting("UDP_SERVICE_PORT");
    if (remote_address.empty()) {
      throw std::runtime_error(
          "RadioDataPlaneSocket::Configure: Failed to query Iris IPv6 address");
    }
    if (remote_port.empty()) {
      throw std::runtime_error(
          "RadioDataPlaneSocket::Configure: Failed to query Iris UDP service "
          "port");
    }

    AGORA_LOG_INFO(
        " STREAM_PROTOCOL  %s\n ETH0_IPv6_ADDR   %s\n UDP_SERVICE_PORT %s\n",
        stream_protocol.c_str(), remote_address.c_str(), remote_port.c_str());

    std::string connect_address;
    ///\todo Needs to be fixed
    const size_t local_interface = 5;
    //get the scope id to get the remote ipv6 address with the local scope id
    AGORA_LOG_INFO(" Remote address  %s\n", remote_address.c_str());
    const auto percent_pos = remote_address.find_last_of('%');
    if (percent_pos != std::string::npos) {
      connect_address = remote_address.substr(0, percent_pos + 1) +
                        std::to_string(local_interface);
    }
    AGORA_LOG_INFO(" Connect address %s\n", connect_address.c_str());

    //Setup the socket interface to the radio for the rx stream
    socket_.Create(
        Configuration()->SampsPerSymbol(), Configuration()->BsServerAddr(),
        connect_address,
        std::to_string(Configuration()->BsServerPort() + radio_->Id()),
        remote_port);

    SoapySDR::Kwargs rxstream_args;
    //Not sure if "bypass mode" works
    rxstream_args["remote:prot"] = "none";
    rxstream_args["iris:ip6_dst"] = socket_.Address();
    rxstream_args["iris:udp_dst"] = socket_.Port();

    AGORA_LOG_INFO(" iris:ip6_dst %s\n iris:udp_dst %s\n",
                   rxstream_args["iris:ip6_dst"].c_str(),
                   rxstream_args["iris:udp_dst"].c_str());

    RadioDataPlane::Setup(rxstream_args);
  } else {
    AGORA_LOG_WARN(
        "Attempting to Setup a previously configured data plane "
        "connection\n");
  }
}

// For now, radio rx will return 1 symbol
int RadioDataPlaneSocket::Rx(
    std::vector<std::vector<std::complex<int16_t>>>& rx_data, size_t rx_size,
    int rx_flags, long long& rx_time_ns) {
  std::vector<void*> rx_locs;
  rx_locs.reserve(rx_data.size());
  for (auto& loc : rx_data) {
    rx_locs.emplace_back(loc.data());
  }
  return Rx(rx_locs, rx_size, rx_flags, rx_time_ns);
}

int RadioDataPlaneSocket::Rx(
    std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs, size_t rx_size,
    int rx_flags, long long& rx_time_ns) {
  std::vector<void*> rx_locs;
  rx_locs.reserve(rx_buffs.size());
  for (auto& loc : rx_buffs) {
    rx_locs.emplace_back(loc->data());
  }
  return Rx(rx_locs, rx_size, rx_flags, rx_time_ns);
}

int RadioDataPlaneSocket::Rx(std::vector<void*>& rx_locations, size_t rx_size,
                             int rx_flags, long long& rx_time_ns) {
  const int rx_return = socket_.RxSymbol(rx_locations, rx_time_ns);
  if (rx_return > 0) {
    AGORA_LOG_TRACE("Rx'd sample count %d\n", rx_return);
  } else if (rx_return < 0) {
    throw std::runtime_error("Error in RadioRx!");
  }
  return rx_return;
}

void RadioDataPlaneSocket::Flush() { socket_.Flush(); }
