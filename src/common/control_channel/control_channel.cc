/**
 * @file control_channel.cc
 * @brief Defination file for ControlChannel class
 */
#include "control_channel.h"

#include "control_message.h"
#include "logger.h"

static constexpr size_t kMaxUdpComm = 1400;

ControlChannel::ControlChannel(std::string bs_ip_address, uint16_t bs_rx_port,
                               size_t rx_buffer_size, size_t tx_buffer_size)
    : comm_(bs_ip_address, bs_rx_port, rx_buffer_size, tx_buffer_size),
      running_(false) {}

ControlChannel::~ControlChannel() {
  Stop();
  if (thread_.joinable()) {
    thread_.join();
  }
}

void ControlChannel::Start() {
  thread_ = std::thread(&ControlChannel::BsControlServer, this);
}

void ControlChannel::Stop() { term_ = true; }

//The BS control server will validate connections with each client and go through a power control sequence
void ControlChannel::BsControlServer() {
  running_ = true;
  std::array<std::byte, kMaxUdpComm> udp_rx_buffer;

  while (term_ == false) {
    auto rx_status = comm_.Recv(udp_rx_buffer.data(), kMaxUdpComm);
    if (rx_status > 0) {
      const size_t message_size = static_cast<size_t>(rx_status);
      auto rx_message =
          ControlMessage::Construct(udp_rx_buffer.data(), message_size);
      if (rx_message != nullptr) {
        HandleControlMessage(rx_message.get());
      }
    } else if (rx_status < 0) {
      AGORA_LOG_ERROR("Control Channel returned error status %ld\n", rx_status);
    }
  }
  running_ = false;
}

void ControlChannel::HandleControlMessage(ControlMessage* control_message) {
  control_message->Print();
  if (control_message->Type() == ControlMessage::kAssociation) {
    //Do something?
  }
}