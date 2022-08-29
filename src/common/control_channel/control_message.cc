/**
 * @file control_message.cc
 * @brief Defination file for Control Messages
 */
#include "control_message.h"

std::unique_ptr<ControlMessage> ControlMessage::Construct(
    const std::byte* raw_data, size_t data_size) {
  switch (static_cast<ControlMessageType>(raw_data[0u])) {
    case ControlMessageType::kAssociation: {
      auto rx_message = std::make_unique<AssociationMessage>();
      rx_message->Deserialize(raw_data, data_size);
      return rx_message;
    }
    default:
      return std::unique_ptr<ControlMessage>(nullptr);
  }
}

void AssociationMessage::Deserialize(const std::byte* raw_data,
                                     size_t data_size) {
  ((void)(raw_data));
  ((void)(data_size));
  client_id_ = 4;
  //Null terminated c style string?
  client_serial_ = {'0', '0', '\0'};
}