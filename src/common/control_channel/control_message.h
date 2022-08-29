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
    kConnectAccept = 0x02,
    kPowerControl = 0x03,
  };
  ControlMessage(ControlMessageType type) : type_(type) {}
  virtual ~ControlMessage() = default;

  inline ControlMessageType Type() const { return type_; }
  virtual void Print() const = 0;
  static std::unique_ptr<ControlMessage> Construct(const std::byte* raw_data,
                                                   size_t data_size);

  virtual void Deserialize(const std::byte* raw_data, size_t data_size) = 0;
  virtual size_t Serialize(std::byte* raw_data) = 0;

 private:
  ControlMessageType type_;
};

class AssociationMessage : public ControlMessage {
 public:
  AssociationMessage() : ControlMessage(ControlMessageType::kAssociation) {}
  virtual ~AssociationMessage() final = default;

  virtual void Print() const final {
    std::printf("Association Message from client with serial %s\n",
                client_serial_.c_str());
  }
  virtual void Deserialize(const std::byte* raw_data, size_t data_size) final;
  virtual size_t Serialize(std::byte* raw_data) final;

 private:
  std::string client_serial_;
};

class ConnectionAcceptMessage : public ControlMessage {
 public:
  ConnectionAcceptMessage()
      : ControlMessage(ControlMessageType::kConnectAccept) {}
  virtual ~ConnectionAcceptMessage() final = default;

  virtual void Print() const final {
    std::printf("Connection Accept Message to client %d, with serial %s\n",
                client_id_, client_serial_.c_str());
  }
  virtual void Deserialize(const std::byte* raw_data, size_t data_size) final;
  virtual size_t Serialize(std::byte* raw_data) final;

 private:
  uint8_t client_id_;
  std::string client_serial_;
};

class PowerControlMessage : public ControlMessage {
 public:
  PowerControlMessage() : ControlMessage(ControlMessageType::kPowerControl) {}
  virtual ~PowerControlMessage() final = default;

  virtual void Print() const final {
    std::printf("Power Control Message to client %d, adjust power %d\n",
                client_id_, power_adjust_);
  }
  virtual void Deserialize(const std::byte* raw_data, size_t data_size) final;
  virtual size_t Serialize(std::byte* raw_data) final;

 private:
  uint8_t client_id_;
  int8_t power_adjust_;
};

#endif /* CONTROL_MESSAGE_H_ */