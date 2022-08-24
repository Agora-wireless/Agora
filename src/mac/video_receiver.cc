/**
 * @file video_receiver.cc
 * @brief Implementation file for the VideoReceiver class
 */

#include "video_receiver.h"

#include <cstring>
#include <string>

#include "logger.h"

static constexpr size_t kMaxRxAttempts = 25u;
//Receive on all addresses
static const std::string kRxAddress = "";

VideoReceiver::VideoReceiver(uint16_t port)
    : udp_video_receiver_(kRxAddress, port,
                          VideoReceiver::kVideoStreamSocketRxBufSize),
      data_available_(0),
      data_start_offset_(0) {}

size_t VideoReceiver::Load(unsigned char *destination, size_t requested_bytes) {
  size_t rx_attempts = 0u;

  if (requested_bytes > data_available_) {
    // Check for potential local buffer wrap-around
    if ((data_available_ + data_start_offset_ +
         VideoReceiver::kVideoStreamMaxRxSize) > local_rx_buffer_.size()) {
      std::memcpy(&local_rx_buffer_.at(0u),
                  &local_rx_buffer_.at(data_start_offset_), data_available_);
      data_start_offset_ = 0u;
    }

    while ((data_available_ < requested_bytes) &&
           (rx_attempts < kMaxRxAttempts)) {
      rx_attempts++;
      ssize_t rcv_ret = udp_video_receiver_.Recv(
          &local_rx_buffer_.at(data_start_offset_ + data_available_),
          VideoReceiver::kVideoStreamMaxRxSize);

      if (rcv_ret < 0) {
        throw std::runtime_error("[VideoReceiver] Receive error");
      } else if (static_cast<size_t>(rcv_ret) >
                 VideoReceiver::kVideoStreamMaxRxSize) {
        throw std::runtime_error(
            "[VideoReceiver] Received packet larger than max receive size -- "
            "inspect");
      } else if (rcv_ret > 0) {
        AGORA_LOG_INFO("[VideoReceiver] data received: %zd\n", rcv_ret);
      }
      data_available_ += rcv_ret;
    }
  }

  // Copy what is available
  size_t loaded_bytes = std::min(data_available_, requested_bytes);
  std::memcpy(destination, &local_rx_buffer_.at(data_start_offset_),
              loaded_bytes);
  AGORA_LOG_FRAME("[VideoReceiver] data loaded: %zu : %zu %zu @ %zu offset\n",
                  loaded_bytes, requested_bytes, data_available_,
                  data_start_offset_);
  data_start_offset_ += loaded_bytes;
  data_available_ -= loaded_bytes;
  loaded_bytes = loaded_bytes;

  if (data_available_ < requested_bytes) {
    AGORA_LOG_FRAME(
        "[VideoReceiver] not enough data to service request %zu:%zu in %zu "
        "attempts\n",
        loaded_bytes, requested_bytes, rx_attempts);
  }
  return loaded_bytes;
}