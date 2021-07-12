#ifndef AGORA_RECORDER_PACKET_H_
#define AGORA_RECORDER_PACKET_H_

#include <stdint.h>
#include <cstdlib>
#include "symbols.h"

namespace Recorder {
class RecorderPacket {
 public:
  RecorderPacket() = default;
  explicit RecorderPacket(uint32_t frame_id, uint32_t symbol_id,
                          uint32_t cell_id, uint32_t ant_id)
  : frame_id_(frame_id),
    symbol_id_(symbol_id),
    cell_id_(cell_id),
    ant_id_(ant_id) { }
  explicit RecorderPacket(const RecorderPacket &input) {
    frame_id_ = input.frame_id_;
    symbol_id_ = input.symbol_id_;
    cell_id_ = input.cell_id_;
    ant_id_ = input.ant_id_;
  }
  virtual ~RecorderPacket() = default;

  // Disallow copy
  RecorderPacket &operator=(const RecorderPacket &) = delete;

  inline virtual bool Empty() const = 0;
  inline virtual void Use() = 0;
  inline virtual void Free() = 0;

  inline virtual EventType GetEventType() const = 0;

  inline uint32_t GetFrameId() {
    return frame_id_;
  }

  inline uint32_t GetSymbolId() {
    return symbol_id_;
  }

  inline uint32_t GetCellId() {
    return cell_id_;
  }

  inline uint32_t GetAntId() {
    return ant_id_;
  }

 private:
  uint32_t frame_id_;
  uint32_t symbol_id_;
  uint32_t cell_id_;
  uint32_t ant_id_;
};

union recorder_tag_t {
  RecorderPacket *recorder_packet_;
  size_t tag_;

  static_assert(sizeof(RecorderPacket *) >= sizeof(size_t),
                "RecorderPacket pointer must fit inside a size_t value");

  explicit recorder_tag_t(RecorderPacket &recorder_packet) : recorder_packet_(&recorder_packet) {}
  explicit recorder_tag_t(RecorderPacket *recorder_packet) : recorder_packet_(recorder_packet) {}
  explicit recorder_tag_t(size_t tag) : tag_(tag) {}
};
static_assert(sizeof(recorder_tag_t) == sizeof(size_t));

}

#endif /* AGORA_RECORDER_PACKET_H_ */