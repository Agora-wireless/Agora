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

using RxMacPacket = RxMemory<MacPacket>;

// Event data tag for Mac RX events
union rx_mac_tag_t {
  RxMacPacket *rx_packet_;
  size_t tag_;

  static_assert(sizeof(RxMacPacket *) >= sizeof(size_t),
                "RxPacket pounter must fit inside a size_t value");

  explicit rx_mac_tag_t(RxMacPacket &rx_packet) : rx_packet_(&rx_packet) {}
  explicit rx_mac_tag_t(RxMacPacket *rx_packet) : rx_packet_(rx_packet) {}
  explicit rx_mac_tag_t(size_t tag) : tag_(tag) {}
};
static_assert(sizeof(rx_mac_tag_t) == sizeof(size_t));

}  // namespace AgoraNetwork

#endif  // MAC_PACKET_H_
