/**
 * @file radio_data_plane_socket.cc
 * @brief Defination file for the RadioDataPlaneSocket Class
 */

#include "radio_data_plane_socket.h"

#include "SoapySDR/Formats.h"
#include "logger.h"
#include "network_utils.h"
#include "radio_soapysdr.h"
#include "utils.h"

RadioDataPlaneSocket::RadioDataPlaneSocket() = default;
RadioDataPlaneSocket::~RadioDataPlaneSocket() { Close(); }

void RadioDataPlaneSocket::Init(Radio* radio, const Config* cfg,
                                bool hw_framer) {
  RadioDataPlane::Init(radio, cfg, hw_framer);
}

inline void RadioDataPlaneSocket::Activate(Radio::ActivationTypes type,
                                           long long act_time_ns,
                                           size_t samples) {
  socket_.Flush();
  RadioDataPlane::Activate(type, act_time_ns, samples);
}

inline void RadioDataPlaneSocket::Deactivate() { RadioDataPlane::Deactivate(); }

inline void RadioDataPlaneSocket::Close() { return RadioDataPlane::Close(); }

//Need to add a port
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

    //radio_ip is the IP address that is accessable from "this" server (this scope)
    //remote_address is the IP address that is accessable from the remote device (local scope)
    const std::string radio_ip =
        dynamic_cast<RadioSoapySdr*>(radio_)->IpAddress();

    const std::string radio_ip_no_scope =
        radio_ip.substr(0, radio_ip.find_last_of('%'));

    const std::string remote_ip_no_scope =
        remote_address.substr(0, remote_address.find_last_of('%'));

    AGORA_LOG_FRAME(
        " STREAM_PROTOCOL  %s\n ETH0_IPv6_ADDR   %s   RADIO IP   %s\n "
        "UDP_SERVICE_PORT %s\n",
        stream_protocol.c_str(), remote_ip_no_scope.c_str(),
        radio_ip_no_scope.c_str(), remote_port.c_str());

    RtAssert(radio_ip_no_scope.compare(remote_ip_no_scope) == 0,
             "Remote Ip address is not that same as expected address");

    const auto scope_pos = radio_ip.find_last_of('%');
    const size_t scope_id = std::atoi(radio_ip.substr(scope_pos + 1).c_str());
    auto local_address = agora_comm::GetLocalAddressFromScope(scope_id);
    /// \todo Not sure why we need this???
    local_address = local_address + radio_ip.substr(scope_pos);

    //Setup the socket interface to the radio for the rx streams
    //Need to use Bs / Ue depending on what radios?
    socket_.Create(
        Configuration()->SampsPerSymbol(), local_address, radio_ip,
        std::to_string(Configuration()->BsServerPort() + radio_->Id()),
        remote_port);

    SoapySDR::Kwargs rxstream_args;
    //Not sure if "bypass mode" works
    rxstream_args["remote:prot"] = "none";
    rxstream_args["remote:mtu"] = "1500";
    rxstream_args["iris:ip6_dst"] = socket_.Address();
    rxstream_args["iris:udp_dst"] = socket_.Port();
    //Helps with the disable stream, errors (-2)
    rxstream_args["SYNC_ACTIVATE"] = "false";

    AGORA_LOG_FRAME(
        "Setting stream destination - iris:ip6_dst %s\n iris:udp_dst %s\n",
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
    Radio::RxFlags& out_flags, long long& rx_time_ns) {
  std::vector<void*> rx_locs;
  rx_locs.reserve(rx_data.size());
  for (auto& loc : rx_data) {
    rx_locs.emplace_back(loc.data());
  }
  return Rx(rx_locs, rx_size, out_flags, rx_time_ns);
}

int RadioDataPlaneSocket::Rx(
    std::vector<std::vector<std::complex<int16_t>>*>& rx_buffs, size_t rx_size,
    Radio::RxFlags& out_flags, long long& rx_time_ns) {
  std::vector<void*> rx_locs;
  rx_locs.reserve(rx_buffs.size());
  for (auto& loc : rx_buffs) {
    rx_locs.emplace_back(loc->data());
  }
  return Rx(rx_locs, rx_size, out_flags, rx_time_ns);
}

int RadioDataPlaneSocket::Rx(std::vector<void*>& rx_locations, size_t rx_size,
                             Radio::RxFlags& out_flags, long long& rx_time_ns) {
  const int rx_return = socket_.RxSamples(rx_locations, rx_time_ns, rx_size);

  if (rx_return > 0) {
    const size_t rx_samples = static_cast<size_t>(rx_return);
    if (rx_samples != rx_size) {
      out_flags = Radio::RxFlags::kEndReceive;
    } else {
      out_flags = Radio::RxFlags::kRxFlagNone;
    }
    AGORA_LOG_TRACE("Rx'd sample count %d\n", rx_return);
  } else if (rx_return < 0) {
    throw std::runtime_error("Error in RadioRx!");
  }
  return rx_return;
}

void RadioDataPlaneSocket::Flush() { socket_.Flush(); }
