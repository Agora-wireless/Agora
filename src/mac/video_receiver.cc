/**
 * @file video_receiver.cc
 * @brief Implementation file for the VideoReceiver class
 */

#include "video_receiver.h"

#include <cstring>

VideoReceiver::VideoReceiver(size_t port)
    : udp_video_receiver_(port, VideoReceiver::kVideoStreamSocketRxBufSize),
      data_availble_(0),
      data_start_offset_(0) { udp_video_receiver_.MakeBlocking(1); }

void VideoReceiver::Load(char *destination, size_t num_load_bytes) {
  if (num_load_bytes > data_availble_) {
    //Check for potential local buffer wrap-around
    if ((data_availble_ + data_start_offset_ +
         VideoReceiver::kVideoStreamMaxRxSize) > local_rx_buffer_.size()) {
      memcpy(&local_rx_buffer_.at(0), &local_rx_buffer_.at(data_start_offset_),
             data_availble_);
      data_start_offset_ = 0;
    }

    while (data_availble_ < num_load_bytes) {
      ssize_t rcv_ret = udp_video_receiver_.Recv(
          &local_rx_buffer_.at(data_start_offset_ + data_availble_),
          VideoReceiver::kVideoStreamMaxRxSize);

      if (rcv_ret < 0) {
        throw std::runtime_error("[VideoReceiver] Receive error");
      } else if (static_cast<size_t>(rcv_ret) >
                 VideoReceiver::kVideoStreamMaxRxSize) {
        throw std::runtime_error(
            "[VideoReceiver] Received packet larger than max receive size -- "
            "inspect");
      }
      else if (rcv_ret > 0){
        std::printf("[VideoReceiver] data received: %zd\n", rcv_ret);
      }
      data_availble_ += rcv_ret;
    }
  }

  //Copy data from local buffer to requested memory location
  memcpy(destination, &local_rx_buffer_.at(data_start_offset_), num_load_bytes);
  std::printf("[VideoReceiver] Data loaded: %zu %zu %zu\n", num_load_bytes, data_availble_, data_start_offset_);
  data_start_offset_ += num_load_bytes;
  data_availble_ -= num_load_bytes;
}