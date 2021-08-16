/**
 * @file rx_memory.h
 * @brief Header file rx packet memory interface
 */
#ifndef RX_MEMORY_H_
#define RX_MEMORY_H_

#include <atomic>
#include <vector>

#include "buffer.h"

namespace AgoraNetwork {

template <class PacketType>
class RxMemory : public MemoryAPI::Memory {
 public:
  RxMemory() : references_(0) { packet_ = nullptr; }
  explicit RxMemory(PacketType *in) : references_(0) { Set(in); }
  explicit RxMemory(const RxMemory &copy) : packet_(copy.packet_) {
    references_.store(copy.references_.load());
  }
  ~RxMemory() = default;

  inline bool Set(PacketType *in_pkt) {
    if (references_.load() == 0) {
      packet_ = in_pkt;
      return true;
    } else {
      throw std::runtime_error("RxMemory loaded when reference count > 0");
      return false;
    }
  }

  inline PacketType const *RawPacket() const { return packet_; }
  inline PacketType *RawPacket() { return packet_; }

  //Public API implementation
  virtual inline bool Empty() const { return references_.load() == 0; }
  virtual inline unsigned Alloc() { return references_.fetch_add(1); }
  virtual inline unsigned Free() {
    unsigned value = references_.fetch_sub(1);
    if (value == 0) {
      throw std::runtime_error("RxMemory free called when memory was empty");
    } else if (value == 1) {
      // Garbage collect if neccessary
    }
    return value;
  }

  //Member variables
 private:
  std::atomic<unsigned> references_;
  PacketType *packet_;
};  // class RxMemory

using RxPacket = RxMemory<Packet>;

// Event data tag for RX events
union rx_tag_t {
  RxPacket *rx_packet_;
  size_t tag_;

  static_assert(sizeof(RxPacket *) >= sizeof(size_t),
                "RxPacket pounter must fit inside a size_t value");

  explicit rx_tag_t(RxPacket &rx_packet) : rx_packet_(&rx_packet) {}
  explicit rx_tag_t(RxPacket *rx_packet) : rx_packet_(rx_packet) {}
  explicit rx_tag_t(size_t tag) : tag_(tag) {}
};
static_assert(sizeof(rx_tag_t) == sizeof(size_t));

// Event data tag for FFT task requests
using fft_req_tag_t = rx_tag_t;
}  // namespace AgoraNetwork

#endif  // RX_MEMORY_H_
