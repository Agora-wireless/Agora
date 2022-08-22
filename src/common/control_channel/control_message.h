/**
 * @file control_message.h
 * @brief Declaration file for Control Messages
 */
#ifndef CONTROL_MESSAGE_H_
#define CONTROL_MESSAGE_H_

#include <array>
#include <cstddef>
#include <cstdint>

static constexpr size_t kClientIdCharCnt = 8;

class AssociationMessage : ControlMessage {
 public:
  AssociationMessage() { type_ = ControlMessageType::kAssociation; }
  virtual void Print() const final {
    std::printf("Association Message from client %d, with serial %s\n",
                client_id_, client_serial_);
  }

 private:
  uint8_t client_id_;
  //Null terminated c style string?
  std::array<char, kClientIdCharCnt> client_serial_;
};

class PowerControlMessage : ControlMessage {
 public:
  PowerControlMessage() { type_ = ControlMessageType::kPowerControl; }
  //void Serialize();
 private:
  int8_t power_adjust_;
  uint8_t client_id_;
};

class ControlMessage {
 public:
  enum ControlMessageType : uint8_t {
    kInvalidMessage = 0x0,
    kAssociation = 0x01,
    kPowerControl = 0x02
  };
  ControlMessageType Type() const { return type_; }
  virtual void Print();

  static ControlMessage Deserialize(const std::byte* raw_data,
                                    size_t data_size) {}

 protected:
  ControlMessageType type_;
};

#endif /* CONTROL_MESSAGE_H_ */