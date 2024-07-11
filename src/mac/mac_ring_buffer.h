#include <array>

#include "logger.h"
#include "message.h"
#include "symbols.h"

// Maximum capacity of Mac packet buffer
static constexpr size_t kMacBuffSizeMax = 1024 * 1024 * 64;

class MacMultiRingBuffer {
 public:
  MacMultiRingBuffer() {}
  void Push(std::byte* item, size_t n_items, size_t buf_id) {
    if (IsFull(n_items, buf_id) == true) {
      AGORA_LOG_ERROR("Buffer Id %zu is full. Push failed!\n", buf_id);
      return;
    }
    size_t& tail = tail_.at(buf_id);
    std::memcpy(&r_buff_.at(buf_id).at(tail), item, n_items);
    tail = (tail + n_items) % kMacBuffSizeMax;
  }

  bool Pop(std::byte* item, size_t n_items, size_t buf_id) {
    if (IsEmpty(n_items, buf_id) == true) {
      AGORA_LOG_ERROR("Buffer Id %zu is empty. Pop failed!\n", buf_id);
      return false;
    }
    size_t& head = head_.at(buf_id);
    std::memcpy(item, &r_buff_.at(buf_id).at(head), n_items);
    //r_buff_.at(buf_id).at(head) = empty_;
    head = (head + n_items) % kMacBuffSizeMax;
    return true;
  }

  size_t BuffSize(size_t buf_id) {
    size_t tail = tail_.at(buf_id);
    size_t head = head_.at(buf_id);
    return (tail >= head) ? (tail - head) : (kMacBuffSizeMax - head + tail);
  }

  bool IsEmpty(size_t n_items, size_t buf_id) {
    /*size_t tail = tail_.at(buf_id);
    size_t head = head_.at(buf_id);
    return (head == tail);*/
    return (this->BuffSize(buf_id) < n_items);
  }

  bool IsFull(size_t n_items, size_t buf_id) {
    /*size_t tail = tail_.at(buf_id);
    size_t head = head_.at(buf_id);
    return ((tail + 1) % kMacBuffSizeMax == head);*/
    return ((this->BuffSize(buf_id) + n_items) > kMacBuffSizeMax);
  }

 private:
  std::array<std::array<std::byte, kMacBuffSizeMax>, kMaxUEs> r_buff_;
  std::array<size_t, kMaxUEs> head_;
  std::array<size_t, kMaxUEs> tail_;
  //char* empty_;
};
