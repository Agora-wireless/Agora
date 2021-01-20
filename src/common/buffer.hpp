#ifndef BUFFER_HEAD
#define BUFFER_HEAD

#include <sstream>
#include <vector>

#include "Symbols.hpp"
#include "memory_manage.h"
#include "ran_config.h"

/* boost is required for aligned memory allocation (for SIMD instructions) */
#include <boost/align/aligned_allocator.hpp>

#include "common_typedef_sdk.h"

// Event data tag for RX events
union rx_tag_t {
  struct {
    size_t tid : 8;      // ID of the socket thread that received the packet
    size_t offset : 56;  // Offset in the socket thread's RX buffer
  };
  size_t _tag;

  rx_tag_t(size_t tid, size_t offset) : tid(tid), offset(offset) {}

  rx_tag_t(size_t _tag) : _tag(_tag) {}
};

// Event data tag for FFT task requests
using fft_req_tag_t = rx_tag_t;

// A generic tag type for Agora tasks. The tag for a particular task will
// have only a subset of the fields initialized.
union gen_tag_t {
  static constexpr size_t kInvalidSymbolId = (1ull << 13) - 1;
  static_assert(kMaxSymbols < ((1ull << 13) - 1), "");
  static_assert(kMaxUEs < UINT16_MAX, "");
  static_assert(kMaxAntennas < UINT16_MAX, "");
  static_assert(kMaxDataSCs < UINT16_MAX, "");

  enum TagType { kCodeblocks, kUsers, kAntennas, kSubcarriers, kNone };

  struct {
    uint32_t frame_id;
    uint16_t symbol_id : 13;
    TagType tag_type : 3;
    union {
      uint16_t cb_id;  // code block
      uint16_t ue_id;
      uint16_t ant_id;
      uint16_t sc_id;
    };
  };

  size_t _tag;
  gen_tag_t(size_t _tag) : _tag(_tag) {}

  // Return a string representation of this tag
  std::string to_string() {
    std::ostringstream ret;
    ret << "[Frame ID " << std::to_string(frame_id) << ", symbol ID "
        << std::to_string(symbol_id);
    switch (tag_type) {
      case kCodeblocks:
        ret << ", code block ID " << std::to_string(cb_id) << "]";
        break;
      case kUsers:
        ret << ", user ID " << std::to_string(ue_id) << "]";
        break;
      case kAntennas:
        ret << ", antenna ID " << std::to_string(ant_id) << "]";
        break;
      case kSubcarriers:
        ret << ", subcarrier ID " << std::to_string(sc_id) << "]";
        break;
      case kNone:
        ret << "] ";
        break;
    }
    return ret.str();
  }

  // Generate a tag with code block ID, frame ID, and symbol ID bits set and
  // other fields blank
  static gen_tag_t frm_sym_cb(size_t frame_id, size_t symbol_id, size_t cb_id) {
    gen_tag_t ret(0);
    ret.frame_id = frame_id;
    ret.symbol_id = symbol_id;
    ret.tag_type = TagType::kCodeblocks;
    ret.cb_id = cb_id;
    return ret;
  }

  // Generate a tag with user ID, frame ID, and symbol ID bits set and
  // other fields blank
  static gen_tag_t frm_sym_ue(size_t frame_id, size_t symbol_id, size_t ue_id) {
    gen_tag_t ret(0);
    ret.frame_id = frame_id;
    ret.symbol_id = symbol_id;
    ret.tag_type = TagType::kUsers;
    ret.ue_id = ue_id;
    return ret;
  }

  // Generate a tag with frame ID, symbol ID, and subcarrier ID bits set and
  // other fields blank
  static gen_tag_t frm_sym_sc(size_t frame_id, size_t symbol_id, size_t sc_id) {
    gen_tag_t ret(0);
    ret.frame_id = frame_id;
    ret.symbol_id = symbol_id;
    ret.tag_type = TagType::kSubcarriers;
    ret.sc_id = sc_id;
    return ret;
  }

  // Generate a tag with antenna ID, frame ID, and symbol ID bits set and
  // other fields blank
  static gen_tag_t frm_sym_ant(size_t frame_id, size_t symbol_id,
                               size_t ant_id) {
    gen_tag_t ret(0);
    ret.frame_id = frame_id;
    ret.symbol_id = symbol_id;
    ret.tag_type = TagType::kAntennas;
    ret.ant_id = ant_id;
    return ret;
  }

  // Generate a tag with frame ID and subcarrier ID bits set, and other fields
  // blank
  static gen_tag_t frm_sc(size_t frame_id, size_t sc_id) {
    gen_tag_t ret(0);
    ret.frame_id = frame_id;
    ret.symbol_id = kInvalidSymbolId;
    ret.tag_type = TagType::kSubcarriers;
    ret.sc_id = sc_id;
    return ret;
  }

  // Generate a tag with frame ID and symbol ID bits set, and other fields
  // blank
  static gen_tag_t frm_sym(size_t frame_id, size_t symbol_id) {
    gen_tag_t ret(0);
    ret.frame_id = frame_id;
    ret.symbol_id = symbol_id;
    ret.tag_type = TagType::kNone;
    return ret;
  }
};
static_assert(sizeof(gen_tag_t) == sizeof(size_t), "");

/**
 * Agora uses these event messages for communication between threads. Each
 * tag encodes information about a task.
 */
struct Event_data {
  static constexpr size_t kMaxTags = 7;
  EventType event_type;
  uint32_t num_tags;
  size_t tags[7];

  // Initialize and event with only the event type field set
  Event_data(EventType event_type) : event_type(event_type), num_tags(0) {}

  // Create an event with one tag
  Event_data(EventType event_type, size_t tag)
      : event_type(event_type), num_tags(1) {
    tags[0] = tag;
  }

  Event_data() : num_tags(0) {}
};
static_assert(sizeof(Event_data) == 64, "");

struct Packet {
  // The packet's data starts at kOffsetOfData bytes from the start
  static constexpr size_t kOffsetOfData = 64;

  uint32_t frame_id;
  uint32_t symbol_id;
  uint32_t cell_id;
  uint32_t ant_id;
  uint32_t fill[12];  // Padding for 64-byte alignment needed for SIMD
  short data[];       // Elements sent by antennae are two bytes (I/Q samples)
  Packet(int f, int s, int c, int a)  // TODO: Should be unsigned integers
      : frame_id(f), symbol_id(s), cell_id(c), ant_id(a) {}

  std::string to_string() const {
    std::ostringstream ret;
    ret << "[Frame seq num " << frame_id << ", symbol ID " << symbol_id
        << ", cell ID " << cell_id << ", antenna ID " << ant_id << ", "
        << sizeof(fill) << " empty bytes]";
    return ret.str();
  }
};

struct MacPacket {
  // The packet's data starts at kOffsetOfData bytes from the start
  static constexpr size_t kOffsetOfData = 16 + sizeof(RBIndicator);

  uint16_t frame_id;
  uint16_t symbol_id;
  uint16_t ue_id;
  uint16_t datalen;          // length of payload in bytes or array data[]
  uint16_t crc;              // 16 bits CRC over calculated for the data[] array
  uint16_t rsvd[3];          // reserved for future use
  RBIndicator rb_indicator;  // RAN scheduling details for PHY
  char data[];               // Mac packet payload data
  MacPacket(int f, int s, int u, int d,
            int cc)  // TODO: Should be unsigned integers
      : frame_id(f), symbol_id(s), ue_id(u), datalen(d), crc(cc) {}

  std::string to_string() const {
    std::ostringstream ret;
    ret << "[Frame seq num " << frame_id << ", symbol ID " << symbol_id
        << ", user ID " << ue_id << "]";
    return ret.str();
  }
};

class RxCounters {
 public:
  // num_pkt[i] is the total number of packets we've received for frame i
  std::array<size_t, kFrameWnd> num_pkts;

  // num_pilot_pkts[i] is the total number of pilot packets we've received
  // for frame i
  std::array<size_t, kFrameWnd> num_pilot_pkts;

  // num_rc_pkts[i] is the total number of reciprocity pilot packets we've
  // received for frame i
  std::array<size_t, kFrameWnd> num_reciprocity_pkts;

  // Number of packets we'll receive per frame on the uplink
  size_t num_pkts_per_frame;

  // Number of pilot packets we'll receive per frame
  size_t num_pilot_pkts_per_frame;

  // Number of reciprocity pilot packets we'll receive per frame
  size_t num_reciprocity_pkts_per_frame;

  RxCounters() {
    num_pkts.fill(0);
    num_pilot_pkts.fill(0);
    num_reciprocity_pkts.fill(0);
  }
};

/**
 * @brief This class stores the counters corresponding to a frame.
 * Specifically, it contains a) the number of symbols per frame
 * and b) the number of tasks per symbol, per frame.
 */
class FrameCounters {
 public:
  void Init(size_t max_symbol_count, size_t max_task_count = 0) {
    this->max_symbol_count_ = max_symbol_count;
    this->max_task_count_ = max_task_count;
    this->symbol_count_.fill(0);
  }

  void Reset(size_t frame_id) {
    const size_t frame_slot = (frame_id % kFrameWnd);
    this->symbol_count_.at(frame_slot) = 0;
    this->task_count_.at(frame_slot).fill(0);
  }

  /**
   * @brief Increments and checks the symbol count for input frame
   * @param frame_id The frame id of the symbol to increment
   */
  bool CompleteSymbol(size_t frame_id) {
    const size_t frame_slot = (frame_id % kFrameWnd);
    this->symbol_count_.at(frame_slot)++;
    return this->IsLastSymbol(frame_slot);
  }

  /**
   * @brief Increments the task count for input frame and symbol
   * @param frame_slot The frame index to increment
   * @param symbol_id The symbol id of the task to increment
   */
  bool CompleteTask(size_t frame_id, size_t symbol_id) {
    const size_t frame_slot = (frame_id % kFrameWnd);
    this->task_count_.at(frame_slot).at(symbol_id)++;
    return this->IsLastTask(frame_id, symbol_id);
  }

  /**
   * @brief Increments the symbol count for input frame
   * @param symbol_id The symbol id to increment
   */
  bool CompleteTask(size_t frame_id) { return this->CompleteSymbol(frame_id); }

  /**
   * @brief Check whether the symbol is the last symbol for a given frame
   * @param frame id The frame id of the symbol to check
   */
  bool IsLastSymbol(size_t frame_id) const {
    const size_t frame_slot = (frame_id % kFrameWnd);
    bool is_last;
    size_t symbol_count = this->symbol_count_.at(frame_slot);
    if (symbol_count == this->max_symbol_count_) {
      is_last = true;
    } else if (symbol_count < this->max_symbol_count_) {
      is_last = false;
    } else {
      is_last = true;
      /* This should never happen */
      assert(false);
    }
    return is_last;
  }

  /**
   * @brief Check whether the task is the last task for a given frame
   * while simultaneously incrementing the task count.
   * This is used for tasks performed once per frame (e.g., ZF)
   * @param frame_id The frame id to check
   */
  bool IsLastTask(size_t frame_id) const { return IsLastSymbol(frame_id); }

  /**
   * @brief Check whether the task is the last task for a given frame and
   * @param frame_id The frame id to check
   * @param symbol_id The symbol id to check
   */
  bool IsLastTask(size_t frame_id, size_t symbol_id) const {
    const size_t frame_slot = frame_id % kFrameWnd;
    bool is_last;
    size_t task_count = this->task_count_.at(frame_slot).at(symbol_id);
    if (task_count == this->max_task_count_) {
      is_last = true;
    } else if (task_count < this->max_task_count_) {
      is_last = false;
    } else {
      is_last = true;
      /* This should never happen */
      assert(false);
    }
    return is_last;
  }

  size_t GetSymbolCount(size_t frame_id) const {
    return this->symbol_count_.at(frame_id % kFrameWnd);
  }

  size_t GetTaskCount(size_t frame_id) const {
    return this->GetSymbolCount(frame_id);
  }

  size_t GetTaskCount(size_t frame_id, size_t symbol_id) const {
    return this->task_count_.at(frame_id % kFrameWnd).at(symbol_id);
  }

  inline size_t max_symbol_count(void) const { return this->max_symbol_count_; }
  inline size_t max_task_count(void) const { return this->max_task_count_; }

 private:
  // task_count[i][j] is the number of tasks completed for
  // frame (i % kFrameWnd) and symbol j
  std::array<std::array<size_t, kMaxSymbols>, kFrameWnd> task_count_;
  // symbol_count[i] is the number of symbols completed for
  // frame (i % kFrameWnd)
  std::array<size_t, kFrameWnd> symbol_count_;

  // Maximum number of symbols in a frame
  size_t max_symbol_count_;
  // Maximum number of tasks in a symbol
  size_t max_task_count_;
};

#endif
