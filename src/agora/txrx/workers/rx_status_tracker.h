/**
 * @file rx_status_tracker.h
 * @brief Implementation of RxStatusTracker helper class
 */
#ifndef RX_STATUS_TRACKER_H_
#define RX_STATUS_TRACKER_H_

#include <complex>
#include <cstddef>
#include <vector>

#include "buffer.h"

namespace TxRxWorkerRx {

class RxStatusTracker {
 public:
  struct RxStatusPerChannelTracker {
    RxPacket* rx_packet_memory_ = nullptr;
    size_t sample_offset_ = 0;
  };

  explicit RxStatusTracker(size_t number_channels)
      : samples_available_(0),
        sample_start_rx_time_(0),
        tracking_(number_channels) {
    for (auto& tracker : tracking_) {
      tracker.sample_offset_ = 0;
      tracker.rx_packet_memory_ = nullptr;
    }
  }

  void Reset(std::vector<RxPacket*> new_packets) {
    samples_available_ = 0;
    sample_start_rx_time_ = 0;
    for (size_t i = 0; i < tracking_.size(); i++) {
      auto& tracker = tracking_.at(i);
      tracker.sample_offset_ = 0;
      tracker.rx_packet_memory_ = new_packets.at(i);
    }
  }

  //Reset, reusing the same rx packet memory locations
  void Reset() {
    std::vector<RxPacket*> new_packets;
    for (auto& tracker : tracking_) {
      new_packets.emplace_back(tracker.rx_packet_memory_);
    }
    Reset(new_packets);
  }

  const std::vector<RxPacket*> GetRxPackets() const {
    const size_t num_packets = tracking_.size();
    std::vector<RxPacket*> rx_packets;
    rx_packets.reserve(num_packets);
    for (const auto& rx_channel : tracking_) {
      rx_packets.emplace_back(rx_channel.rx_packet_memory_);
    }
    return rx_packets;
  }

  void Update(size_t new_samples, long long sample_rx_start) {
    if (samples_available_ == 0) {
      RtAssert(
          sample_start_rx_time_ == 0,
          "RxStatusTracker::Update - Expected samples start time to be 0\n");
      sample_start_rx_time_ = sample_rx_start;
    }

    //Update (incrememt) the rx memory locations by the number of new samples
    for (auto& tracker : tracking_) {
      tracker.sample_offset_ += new_samples;
      RtAssert(tracker.rx_packet_memory_ != nullptr,
               "RxStatusTracker::Update - Rx packet memory to be assigned\n");
    }
    samples_available_ += new_samples;
  }

  inline size_t SamplesAvailable() const { return samples_available_; }
  inline size_t NumChannels() const { return tracking_.size(); }
  inline long long StartTime() const { return sample_start_rx_time_; }
  //Get memory locations for Rx calls
  inline const std::vector<void*> GetRxPtrs() const {
    const size_t num_locations = tracking_.size();
    std::vector<void*> rx_locations;
    rx_locations.reserve(num_locations);
    for (const auto& rx_channel : tracking_) {
      rx_locations.emplace_back(
          static_cast<void*>(&reinterpret_cast<std::complex<int16_t>*>(
              rx_channel.rx_packet_memory_->RawPacket()
                  ->data_)[rx_channel.sample_offset_]));
    }
    return rx_locations;
  }

 private:
  size_t samples_available_ = 0;
  long long sample_start_rx_time_ = 0;
  // For each channel
  std::vector<RxStatusPerChannelTracker> tracking_;
};
}  // namespace TxRxWorkerRx

#endif  // RX_STATUS_TRACKER_H_