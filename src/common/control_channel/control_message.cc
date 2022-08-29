/**
 * @file control_message.cc
 * @brief Defination file for Control Messages
 */
#include "control_message.h"

#include <cstring>

std::unique_ptr<ControlMessage> ControlMessage::Construct(
    const std::byte* raw_data, size_t data_size) {
  switch (static_cast<ControlMessageType>(raw_data[0u])) {
    case ControlMessageType::kAssociation: {
      auto rx_message = std::make_unique<AssociationMessage>();
      rx_message->Deserialize(raw_data, data_size);
      return rx_message;
    }
    case ControlMessageType::kConnectAccept: {
      auto rx_message = std::make_unique<ConnectionAcceptMessage>();
      rx_message->Deserialize(raw_data, data_size);
      return rx_message;
    }
    case ControlMessageType::kPowerControl: {
      auto rx_message = std::make_unique<PowerControlMessage>();
      rx_message->Deserialize(raw_data, data_size);
      return rx_message;
    }
    default:
      return std::unique_ptr<ControlMessage>(nullptr);
  }
}

void AssociationMessage::Deserialize(const std::byte* raw_data,
                                     size_t data_size) {
  if (static_cast<uint8_t>(raw_data[0u]) != Type()) {
    throw std::runtime_error("Invalid type");
  }
  client_serial_.append(reinterpret_cast<const char*>(&raw_data[1u]),
                        data_size - 1);
}

size_t AssociationMessage::Serialize(std::byte* raw_data) {
  raw_data[0u] = static_cast<std::byte>(Type());
  std::memcpy(&raw_data[1u], client_serial_.data(), client_serial_.length());
  return client_serial_.length() + 1;
}

void ConnectionAcceptMessage::Deserialize(const std::byte* raw_data,
                                          size_t data_size) {
  if (static_cast<uint8_t>(raw_data[0u]) != Type()) {
    throw std::runtime_error("Invalid type");
  }
  client_id_ = static_cast<uint8_t>(raw_data[1u]);
  client_serial_.append(reinterpret_cast<const char*>(&raw_data[2u]),
                        data_size - 2);
}

size_t ConnectionAcceptMessage::Serialize(std::byte* raw_data) {
  raw_data[0u] = static_cast<std::byte>(Type());
  raw_data[1u] = static_cast<std::byte>(client_id_);
  std::memcpy(&raw_data[2u], client_serial_.data(), client_serial_.length());
  return client_serial_.length() + 2;
}

void PowerControlMessage::Deserialize(const std::byte* raw_data,
                                      size_t data_size) {
  if (static_cast<uint8_t>(raw_data[0u]) != Type()) {
    throw std::runtime_error("Invalid type");
  }
  if (data_size != 3) {
    throw std::runtime_error("Invalid Size of Power Control Message");
  }
  client_id_ = static_cast<uint8_t>(raw_data[1u]);
  power_adjust_ = static_cast<int8_t>(raw_data[2u]);
}

size_t PowerControlMessage::Serialize(std::byte* raw_data) {
  raw_data[0u] = static_cast<std::byte>(Type());
  raw_data[1u] = static_cast<std::byte>(client_id_);
  raw_data[2u] = static_cast<std::byte>(power_adjust_);
  return 3;
}