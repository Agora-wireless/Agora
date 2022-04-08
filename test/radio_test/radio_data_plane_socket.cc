/**
 * @file radio_data_plane_socket.cc
 * @brief Defination file for the RadioDataPlaneSocket Class
 */

#include "radio_data_plane_socket.h"

#include "SoapySDR/Formats.h"
#include "logger.h"
#include "utils.h"

RadioDataPlaneSocket::RadioDataPlaneSocket()
    : RadioDataPlaneSocket(nullptr, nullptr, 0) {}

RadioDataPlaneSocket::RadioDataPlaneSocket(const Config* cfg,
                                           SoapySDR::Device* device, size_t id)
    : radio_id_(id),
      mode_(kModeUninit),
      cfg_(cfg),
      device_(device),
      rx_stream_(nullptr),
      socket_() {
  if ((cfg != nullptr) && (device != nullptr)) {
    mode_ = kModeShutdown;
  }
}

RadioDataPlaneSocket::~RadioDataPlaneSocket() { Close(); }

void RadioDataPlaneSocket::Init(const Config* cfg, SoapySDR::Device* device,
                                size_t id) {
  if (mode_ == kModeUninit) {
    if ((cfg != nullptr) && (device != nullptr)) {
      cfg_ = cfg;
      device_ = device;
      radio_id_ = id;
      mode_ = kModeShutdown;
    } else {
      AGORA_LOG_WARN(
          "Attempted to init the data plane with null cfg or device pointer\n");
    }
  } else {
    AGORA_LOG_ERROR(
        "Attempted to init the data plane while in the wrong mode %d\n",
        static_cast<int>(mode_));
  }
}

void RadioDataPlaneSocket::Activate() {
  if (mode_ == kModeDeactive) {
    if ((device_ == nullptr) || (rx_stream_ == nullptr)) {
      AGORA_LOG_ERROR(
          "Device pointer or rx_stream is null in RadioDataPlaneSocket "
          "Activate");
    } else {
      device_->activateStream(rx_stream_);
      mode_ = kModeActive;
    }
  } else {
    AGORA_LOG_WARN("Attempting to Activate data plane when in wrong state %d\n",
                   static_cast<int>(mode_));
  }
}

void RadioDataPlaneSocket::Deactivate() {
  if (mode_ == kModeActive) {
    if ((device_ == nullptr) || (rx_stream_ == nullptr)) {
      AGORA_LOG_ERROR(
          "Device pointer or rx_stream is null in RadioDataPlaneSocket "
          "Deactivate");
    } else {
      device_->deactivateStream(rx_stream_);
      mode_ = kModeDeactive;
    }
  } else {
    AGORA_LOG_WARN(
        "Attempting to Deactivate data plane when in wrong state %d\n",
        static_cast<int>(mode_));
  }
}

void RadioDataPlaneSocket::Close() {
  if (device_ == nullptr) {
    AGORA_LOG_ERROR("Device pointer is null in RadioDataPlaneSocket Close");
    return;
  }

  if (mode_ == kModeActive) {
    Deactivate();
  }

  if (mode_ == kModeDeactive) {
    device_->closeStream(rx_stream_);
    rx_stream_ = nullptr;
  }
  mode_ = kModeShutdown;
}

void RadioDataPlaneSocket::Setup() {
  if (mode_ == kModeShutdown) {
    auto channels = Utils::StrToChannels(cfg_->Channel());

    std::string stream_protocol = device_->readSetting("STREAM_PROTOCOL");
    if (stream_protocol != "twbw64") {
      throw std::runtime_error("Stream protocol mismatch");
    }

    //query remote iris endpoint configuration
    auto remote_address = device_->readSetting("ETH0_IPv6_ADDR");
    const auto remote_port = device_->readSetting("UDP_SERVICE_PORT");
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
        cfg_->SampsPerSymbol(), cfg_->BsServerAddr(), connect_address,
        std::to_string(cfg_->BsServerPort() + radio_id_), remote_port);

    SoapySDR::Kwargs sargs;
    //Not sure if "bypass mode" works
    sargs["remote:prot"] = "none";
    sargs["iris:ip6_dst"] = socket_.Address();
    sargs["iris:udp_dst"] = socket_.Port();

    AGORA_LOG_INFO(" iris:ip6_dst %s\n iris:udp_dst %s\n",
                   sargs["iris:ip6_dst"].c_str(),
                   sargs["iris:udp_dst"].c_str());

    rx_stream_ =
        device_->setupStream(SOAPY_SDR_RX, SOAPY_SDR_CS16, channels, sargs);
    mode_ = kModeDeactive;
  } else {
    AGORA_LOG_WARN(
        "Attempting to Setup a previously configured data plane "
        "connection\n");
  }
}

// For now, radio rx will return 1 symbol
int RadioDataPlaneSocket::Rx(
    std::vector<std::vector<std::complex<int16_t>>>& rx_data,
    long long& rx_time_ns) {
  const int rx_return = socket_.RxSymbol(rx_data, rx_time_ns);
  if (rx_return > 0) {
    AGORA_LOG_INFO("Rx'd sample count %d\n", rx_return);
  } else if (rx_return < 0) {
    throw std::runtime_error("Error in RadioRx!");
  }
  return rx_return;
}

void RadioDataPlaneSocket::Flush() { socket_.Flush(); }
