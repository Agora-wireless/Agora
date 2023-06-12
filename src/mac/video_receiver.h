/**
 * @file video_receiver.h
 * @brief Declaration file for the VideoReceiver class
 */
#ifndef VIDEO_RECEIVER_H_
#define VIDEO_RECEIVER_H_

#include <array>

#include "mac_data_receiver.h"
#include "udp_server.h"

/**
 * @brief The Video Receiver class creates a UDP server to receive a video
 * stream tested with VLC streaming application
 */
class VideoReceiver : public MacDataReceiver {
 public:
  // Video stream specific variables
  static constexpr size_t kVideoStreamRxPort = 1350u;
  // Typical rx size for a UDP data packet coming from VLC udp streamer
  static constexpr size_t kVideoStreamRxSize = (1316u);
  static constexpr size_t kVideoStreamSocketRxBufSize =
      (kVideoStreamRxSize * 1000u);
  // Oversize the potential receive size
  static constexpr size_t kVideoStreamMaxRxSize = 2048u;
  static constexpr size_t kVideoStreamLocalRxBufSize =
      kVideoStreamMaxRxSize * 10;

  explicit VideoReceiver(uint16_t port);
  ~VideoReceiver() override = default;

  size_t Load(unsigned char *destination, size_t requested_bytes) final;

 private:
  UDPServer udp_video_receiver_;
  std::array<std::byte, VideoReceiver::kVideoStreamLocalRxBufSize>
      local_rx_buffer_;

  size_t data_available_;
  size_t data_start_offset_;
};

#endif  // VIDEO_RECEIVER_H_
