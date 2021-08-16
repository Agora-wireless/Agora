/**
 * @file mac_packet.h
 * @brief Header file mac packet structures
 */
#ifndef MAC_PACKET_H_
#define MAC_PACKET_H_

#include <atomic>
#include <vector>

#include "buffer.h"
#include "rx_memory.h"

namespace AgoraNetwork {
struct MacPacket {
  // The packet's data starts at kOffsetOfData bytes from the start
  static constexpr size_t kOffsetOfData = 16 + sizeof(RBIndicator);

  uint16_t frame_id_;
  uint16_t symbol_id_;
  uint16_t ue_id_;
  uint16_t datalen_;  // length of payload in bytes or array data[]
  // 16 bits CRC over calculated for the data[] array (should be 24 bit for packets > 3824 bits)
  uint16_t crc_;
  uint16_t rsvd_[3u];         // reserved for future use
  RBIndicator rb_indicator_;  // RAN scheduling details for PHY
  char data_[];               // Mac packet payload data
  MacPacket(int f, int s, int u, int d,
            int cc)  // TODO: Should be unsigned integers
      : frame_id_(f), symbol_id_(s), ue_id_(u), datalen_(d), crc_(cc) {}

  std::string ToString() const {
    std::ostringstream ret;
    ret << "[Frame seq num " << frame_id_ << ", symbol ID " << symbol_id_
        << ", user ID " << ue_id_ << "]";
    return ret.str();
  }
};

// Event data tag for Mac RX events
union rx_mac_tag_t {
  struct {
    size_t tid_ : 8;      // ID of the socket thread that received the packet
    size_t offset_ : 56;  // Offset in the socket thread's RX buffer
  };
  size_t tag_;

  rx_mac_tag_t(size_t tid, size_t offset) : tid_(tid), offset_(offset) {}
  explicit rx_mac_tag_t(size_t _tag) : tag_(_tag) {}
};
static_assert(sizeof(rx_mac_tag_t) == sizeof(size_t));

}  // namespace AgoraNetwork

#endif  // MAC_PACKET_H_
