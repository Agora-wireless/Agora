/**
 * @file message.h
 * @brief Self defined functions for message storage and passing
 */
#ifndef MESSAGE_H_
#define MESSAGE_H_

#include <array>
#include <atomic>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <string>

#include "ran_config.h"
#include "symbols.h"

// A generic tag type for Agora tasks. The tag for a particular task will
// have only a subset of the fields initialized.
union gen_tag_t {
  static constexpr size_t kInvalidSymbolId = (1ull << 13) - 1;
  static_assert(kMaxSymbols < ((1ull << 13) - 1));
  static_assert(kMaxUEs < UINT16_MAX);
  static_assert(kMaxAntennas < UINT16_MAX);
  static_assert(kMaxDataSCs < UINT16_MAX);

  enum TagType { kCodeblocks, kUsers, kAntennas, kSubcarriers, kNone };

  struct {
    uint32_t frame_id_;
    uint16_t symbol_id_ : 13;
    TagType tag_type_ : 3;
    union {
      uint16_t cb_id_;  // code block
      uint16_t ue_id_;
      uint16_t ant_id_;
      uint16_t sc_id_;
    };
  };

  size_t tag_;
  explicit gen_tag_t(size_t _tag) : tag_(_tag) {}

  // Return a string representation of this tag
  std::string ToString() const {
    std::ostringstream ret;
    ret << "[Frame ID " << std::to_string(frame_id_) << ", symbol ID "
        << std::to_string(symbol_id_);
    switch (tag_type_) {
      case kCodeblocks:
        ret << ", code block ID " << std::to_string(cb_id_) << "]";
        break;
      case kUsers:
        ret << ", user ID " << std::to_string(ue_id_) << "]";
        break;
      case kAntennas:
        ret << ", antenna ID " << std::to_string(ant_id_) << "]";
        break;
      case kSubcarriers:
        ret << ", subcarrier ID " << std::to_string(sc_id_) << "]";
        break;
      case kNone:
        ret << "] ";
        break;
    }
    return ret.str();
  }

  // Generate a tag with code block ID, frame ID, and symbol ID bits set and
  // other fields blank
  static gen_tag_t FrmSymCb(size_t frame_id, size_t symbol_id, size_t cb_id) {
    gen_tag_t ret(0);
    ret.frame_id_ = frame_id;
    ret.symbol_id_ = symbol_id;
    ret.tag_type_ = TagType::kCodeblocks;
    ret.cb_id_ = cb_id;
    return ret;
  }

  // Generate a tag with user ID, frame ID, and symbol ID bits set and
  // other fields blank
  static gen_tag_t FrmSymUe(size_t frame_id, size_t symbol_id, size_t ue_id) {
    gen_tag_t ret(0);
    ret.frame_id_ = frame_id;
    ret.symbol_id_ = symbol_id;
    ret.tag_type_ = TagType::kUsers;
    ret.ue_id_ = ue_id;
    return ret;
  }

  // Generate a tag with frame ID, symbol ID, and subcarrier ID bits set and
  // other fields blank
  static gen_tag_t FrmSymSc(size_t frame_id, size_t symbol_id, size_t sc_id) {
    gen_tag_t ret(0);
    ret.frame_id_ = frame_id;
    ret.symbol_id_ = symbol_id;
    ret.tag_type_ = TagType::kSubcarriers;
    ret.sc_id_ = sc_id;
    return ret;
  }

  // Generate a tag with antenna ID, frame ID, and symbol ID bits set and
  // other fields blank
  static gen_tag_t FrmSymAnt(size_t frame_id, size_t symbol_id, size_t ant_id) {
    gen_tag_t ret(0);
    ret.frame_id_ = frame_id;
    ret.symbol_id_ = symbol_id;
    ret.tag_type_ = TagType::kAntennas;
    ret.ant_id_ = ant_id;
    return ret;
  }

  // Generate a tag with frame ID and subcarrier ID bits set, and other fields
  // blank
  static gen_tag_t FrmSc(size_t frame_id, size_t sc_id) {
    gen_tag_t ret(0);
    ret.frame_id_ = frame_id;
    ret.symbol_id_ = kInvalidSymbolId;
    ret.tag_type_ = TagType::kSubcarriers;
    ret.sc_id_ = sc_id;
    return ret;
  }

  // Generate a tag with frame ID and symbol ID bits set, and other fields
  // blank
  static gen_tag_t FrmSym(size_t frame_id, size_t symbol_id) {
    gen_tag_t ret(0);
    ret.frame_id_ = frame_id;
    ret.symbol_id_ = symbol_id;
    ret.tag_type_ = TagType::kNone;
    return ret;
  }
};
static_assert(sizeof(gen_tag_t) == sizeof(size_t));

/**
 * Agora uses these event messages for communication between threads. Each
 * tag encodes information about a task.
 */
struct EventData {
  static constexpr size_t kMaxTags = 7;
  EventType event_type_;
  uint32_t num_tags_{0};
  std::array<size_t, kMaxTags> tags_;

  // Initialize an event with only the event type field set
  explicit EventData(EventType event_type) : event_type_(event_type) {
    tags_.fill(0);
  }

  // Create an event with one tag
  EventData(EventType event_type, size_t tag)
      : event_type_(event_type), num_tags_(1) {
    tags_.fill(0);
    tags_.at(0) = tag;
  }

  EventData() = default;
};
static_assert(sizeof(EventData) == 64);

struct Packet {
  // The packet's data starts at kOffsetOfData bytes from the start
  static constexpr size_t kOffsetOfData = 64;

  uint32_t frame_id_;
  uint32_t symbol_id_;
  uint32_t cell_id_;
  uint32_t ant_id_;
  uint32_t fill_[12];  // Padding for 64-byte alignment needed for SIMD
  short data_[];       // Elements sent by antennae are two bytes (I/Q samples)
  Packet(int f, int s, int c, int a)  // TODO: Should be unsigned integers
      : frame_id_(f), symbol_id_(s), cell_id_(c), ant_id_(a) {}

  std::string ToString() const {
    std::ostringstream ret;
    ret << "[Frame seq num " << frame_id_ << ", symbol ID " << symbol_id_
        << ", cell ID " << cell_id_ << ", antenna ID " << ant_id_ << ", "
        << sizeof(fill_) << " empty bytes]";
    return ret.str();
  }
};

class RxPacket {
 private:
  std::atomic<unsigned> references_;
  Packet *packet_;

  inline virtual void GcPacket() {}

 public:
  RxPacket() : references_(0) { packet_ = nullptr; }
  explicit RxPacket(Packet *in) : references_(0) { Set(in); }
  RxPacket(const RxPacket &copy) : packet_(copy.packet_) {
    references_.store(copy.references_.load());
  }
  virtual ~RxPacket() = default;

  // Disallow copy
  RxPacket &operator=(const RxPacket &) = delete;

  inline bool Set(Packet *in_pkt) {
    if (references_.load() == 0) {
      packet_ = in_pkt;
      return true;
    } else {
      return false;
    }
  }

  inline Packet *RawPacket() { return packet_; }
  inline bool Empty() const { return references_.load() == 0; }
  inline void Use() { references_.fetch_add(1); }
  inline void Free() {
    unsigned value = references_.fetch_sub(1);
    if (value == 0) {
      throw std::runtime_error("RxPacket free called when memory was empty");
    } else if (value == 1) {
      GcPacket();
    }
  }
};

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

#pragma pack(push, 1)
struct MacPacketHeaderPacked {
 public:
  inline uint16_t Frame() const { return frame_id_; }
  inline uint16_t Symbol() const { return symbol_id_; }
  inline uint16_t Ue() const { return ue_id_; }
  inline uint16_t Crc() const { return crc_; }
  inline uint16_t PayloadLength() const { return datalen_; }

  // Modifiers
  inline void Set(const uint16_t &f, const uint16_t &s, const uint16_t &u,
                  const uint16_t &d, const uint16_t &cc) {
    frame_id_ = f;
    symbol_id_ = s;
    ue_id_ = u;
    datalen_ = d;
    crc_ = cc;
  }
  inline void Crc(const uint16_t &crc) { crc_ = crc; }

 private:
  uint16_t frame_id_;
  uint16_t symbol_id_;
  uint16_t ue_id_;
  uint16_t datalen_;  // length of payload in bytes or array data[]
  uint16_t crc_;      // 16 bits CRC over calculated for the data[] array
#if defined(ENABLE_RB_IND)
  RBIndicator rb_indicator_;  // RAN scheduling details for PHY
#endif
};

struct MacPacketPacked {
 public:
  static constexpr size_t kHeaderSize = sizeof(MacPacketHeaderPacked);

  inline uint16_t Frame() const { return header_.Frame(); }
  inline uint16_t Symbol() const { return header_.Symbol(); }
  inline uint16_t Ue() const { return header_.Ue(); }
  inline uint16_t Crc() const { return header_.Crc(); }
  inline uint16_t PayloadLength() const { return header_.PayloadLength(); }
  inline const unsigned char *Data() const { return data_; };

  // Modifiers
  inline void Set(const uint16_t &f, const uint16_t &s, const uint16_t &u,
                  const uint16_t &data_size) {
    header_.Set(f, s, u, data_size, 0);
  }
  inline void LoadData(const unsigned char *src_data) {
    std::memcpy(this->data_, src_data, this->PayloadLength());
  }
  inline void Crc(const uint16_t &crc) { header_.Crc(crc); }
  inline unsigned char *DataPtr() { return data_; };

 private:
  MacPacketHeaderPacked header_;
  unsigned char data_[];  // Mac packet payload data
};
#pragma pack(pop)

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

class RxCounters {
 public:
  // num_pkt[i] is the total number of packets we've received for frame i
  std::array<size_t, kFrameWnd> num_pkts_;

  // num_pilot_pkts[i] is the total number of pilot packets we've received
  // for frame i
  std::array<size_t, kFrameWnd> num_pilot_pkts_;

  // num_rc_pkts[i] is the total number of reciprocity pilot packets we've
  // received for frame i
  std::array<size_t, kFrameWnd> num_reciprocity_pkts_;

  // Number of packets we'll receive per frame on the uplink
  size_t num_rx_pkts_per_frame_;

  // Number of pilot packets we'll receive per frame
  size_t num_pilot_pkts_per_frame_;

  // Number of reciprocity pilot packets we'll receive per frame
  size_t num_reciprocity_pkts_per_frame_;

  RxCounters() {
    num_pkts_.fill(0);
    num_pilot_pkts_.fill(0);
    num_reciprocity_pkts_.fill(0);
  }
};

/**
 * @brief This class stores the counters corresponding to a frame.
 * Specifically, it contains a) the number of symbols per frame
 * and b) the number of tasks per symbol, per frame.
 */
class FrameCounters {
 public:
  FrameCounters() : task_count_(), symbol_count_() {}

  void Init(size_t max_symbol_count, size_t max_task_count = 0) {
    this->max_symbol_count_ = max_symbol_count;
    this->max_task_count_ = max_task_count;
    this->symbol_count_.fill(0);
    for (auto &frame : task_count_) {
      frame.fill(0);
    }
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
    const size_t symbol_count = symbol_count_.at(frame_slot);
    bool is_last;
    if (symbol_count == max_symbol_count_) {
      is_last = true;
    } else if (symbol_count < max_symbol_count_) {
      is_last = false;
    } else {
      /* This should never happen */
      is_last = true;
      std::printf(
          "Unexpected result in IsLastSymbol: Symbol Count %zu,  Max Count "
          "%zu, Frame %zu\n",
          symbol_count, max_symbol_count_, frame_id);
      assert(false);
      throw std::runtime_error("IsLastSymbol error!");
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
    const size_t task_count = this->task_count_.at(frame_slot).at(symbol_id);
    bool is_last;
    if (task_count == this->max_task_count_) {
      is_last = true;
    } else if (task_count < this->max_task_count_) {
      is_last = false;
    } else {
      // This should never happen
      is_last = true;
      std::printf(
          "Unexpected result in IsLastTask: Task Count %zu,  Max Count %zu, "
          "Frame %zu, Symbol %zu\n",
          task_count, this->max_task_count_, frame_id, symbol_id);
      assert(false);
      throw std::runtime_error("IsLastTask error!");
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

  inline size_t MaxSymbolCount() const { return this->max_symbol_count_; }
  inline size_t MaxTaskCount() const { return this->max_task_count_; }

 private:
  // task_count[i][j] is the number of tasks completed for
  // frame (i % kFrameWnd) and symbol j
  std::array<std::array<size_t, kMaxSymbols>, kFrameWnd> task_count_;
  // symbol_count[i] is the number of symbols completed for
  // frame (i % kFrameWnd)
  std::array<size_t, kFrameWnd> symbol_count_;

  // Maximum number of symbols in a frame
  size_t max_symbol_count_{0};
  // Maximum number of tasks in a symbol
  size_t max_task_count_{0};
};

#endif  // MESSAGE_H_
