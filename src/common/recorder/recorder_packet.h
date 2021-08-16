/**
 * @file recorder_packet.h
 * @brief Defination for the RecorderPacket interface
 */
#ifndef AGORA_RECORDER_PACKET_H_
#define AGORA_RECORDER_PACKET_H_

#include <stdint.h>

#include <cstdlib>

#include "rx_memory.h"
#include "symbols.h"

namespace Recorder {

class RecorderPacket : public AgoraNetwork::RxPacket {
 public:
  inline const uint32_t &GetAntId() const { return this->RawPacket()->ant_id_; }
  inline EventType GetEventType() const { return EventType::kPacketRX; }
};

union recorder_tag_t {
  RecorderPacket *recorder_packet_;
  size_t tag_;

  static_assert(sizeof(RecorderPacket *) >= sizeof(size_t),
                "RecorderPacket pointer must fit inside a size_t value");

  explicit recorder_tag_t(RecorderPacket &recorder_packet)
      : recorder_packet_(&recorder_packet) {}
  explicit recorder_tag_t(RecorderPacket *recorder_packet)
      : recorder_packet_(recorder_packet) {}
  explicit recorder_tag_t(size_t tag) : tag_(tag) {}
};
static_assert(sizeof(recorder_tag_t) == sizeof(size_t));

}  // namespace Recorder

#endif /* AGORA_RECORDER_PACKET_H_ */