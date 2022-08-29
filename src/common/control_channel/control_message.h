/**
 * @file control_message.h
 * @brief Declaration file for Control Messages
 */
#ifndef CONTROL_MESSAGE_H_
#define CONTROL_MESSAGE_H_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

static constexpr size_t kClientIdCharCnt = 8;

class ControlMessage {
 public:
  enum ControlMessageType : uint8_t {
    kInvalidMessage = 0x0,
    kAssociation = 0x01,
    kPowerControl = 0x02
  };
  ControlMessage(ControlMessageType type) : type_(type) {}
  virtual ~ControlMessage() = default;

  ControlMessageType Type() const { return type_; }
  virtual void Print() const = 0;
  static std::unique_ptr<ControlMessage> Construct(const std::byte* raw_data,
                                                   size_t data_size);

 private:
  ControlMessageType type_;
  virtual void Deserialize(const std::byte* raw_data, size_t data_size) = 0;
};

class AssociationMessage : public ControlMessage {
 public:
  AssociationMessage() : ControlMessage(ControlMessageType::kAssociation) {}
  virtual ~AssociationMessage() final = default;

  virtual void Print() const final {
    std::printf("Association Message from client %d, with serial %s\n",
                client_id_, client_serial_.data());
  }
  virtual void Deserialize(const std::byte* raw_data, size_t data_size) final;

 private:
  uint8_t client_id_;
  //Null terminated c style string?
  std::array<char, kClientIdCharCnt> client_serial_;
};

/*
class PowerControlMessage : public ControlMessage {
 public:
  PowerControlMessage() { type_ = ControlMessageType::kPowerControl; }
  //void Serialize();
 private:
  int8_t power_adjust_;
  uint8_t client_id_;
};
*/

#endif /* CONTROL_MESSAGE_H_ */