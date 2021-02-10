#ifndef BUFFER_HEAD
#define BUFFER_HEAD

#include "memory_manage.h"
#include "ran_config.h"
#include "symbols.h"
#include <sstream>
#include <vector>

/* boost is required for aligned memory allocation (for SIMD instructions) */
#include "common_typedef_sdk.h"
#include <boost/align/aligned_allocator.hpp>

// Event data tag for RX events
union rx_tag_t {
    struct {
        size_t tid_ : 8; // ID of the socket thread that received the packet
        size_t offset_ : 56; // Offset in the socket thread's RX buffer
    };
    size_t tag_;

    rx_tag_t(size_t tid, size_t offset)
        : tid_(tid)
        , offset_(offset)
    {
    }

    rx_tag_t(size_t _tag)
        : tag_(_tag)
    {
    }
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
        uint32_t frame_id_;
        uint16_t symbol_id_ : 13;
        TagType tag_type_ : 3;
        union {
            uint16_t cb_id_; // code block
            uint16_t ue_id_;
            uint16_t ant_id_;
            uint16_t sc_id_;
        };
    };

    size_t tag_;
    gen_tag_t(size_t _tag)
        : tag_(_tag)
    {
    }

    // Return a string representation of this tag
    std::string ToString() const
    {
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
    static gen_tag_t FrmSymCb(size_t frame_id, size_t symbol_id, size_t cb_id)
    {
        gen_tag_t ret(0);
        ret.frame_id_ = frame_id;
        ret.symbol_id_ = symbol_id;
        ret.tag_type_ = TagType::kCodeblocks;
        ret.cb_id_ = cb_id;
        return ret;
    }

    // Generate a tag with user ID, frame ID, and symbol ID bits set and
    // other fields blank
    static gen_tag_t FrmSymUe(size_t frame_id, size_t symbol_id, size_t ue_id)
    {
        gen_tag_t ret(0);
        ret.frame_id_ = frame_id;
        ret.symbol_id_ = symbol_id;
        ret.tag_type_ = TagType::kUsers;
        ret.ue_id_ = ue_id;
        return ret;
    }

    // Generate a tag with frame ID, symbol ID, and subcarrier ID bits set and
    // other fields blank
    static gen_tag_t FrmSymSc(size_t frame_id, size_t symbol_id, size_t sc_id)
    {
        gen_tag_t ret(0);
        ret.frame_id_ = frame_id;
        ret.symbol_id_ = symbol_id;
        ret.tag_type_ = TagType::kSubcarriers;
        ret.sc_id_ = sc_id;
        return ret;
    }

    // Generate a tag with antenna ID, frame ID, and symbol ID bits set and
    // other fields blank
    static gen_tag_t FrmSymAnt(size_t frame_id, size_t symbol_id, size_t ant_id)
    {
        gen_tag_t ret(0);
        ret.frame_id_ = frame_id;
        ret.symbol_id_ = symbol_id;
        ret.tag_type_ = TagType::kAntennas;
        ret.ant_id_ = ant_id;
        return ret;
    }

    // Generate a tag with frame ID and subcarrier ID bits set, and other fields
    // blank
    static gen_tag_t FrmSc(size_t frame_id, size_t sc_id)
    {
        gen_tag_t ret(0);
        ret.frame_id_ = frame_id;
        ret.symbol_id_ = kInvalidSymbolId;
        ret.tag_type_ = TagType::kSubcarriers;
        ret.sc_id_ = sc_id;
        return ret;
    }

    // Generate a tag with frame ID and symbol ID bits set, and other fields
    // blank
    static gen_tag_t FrmSym(size_t frame_id, size_t symbol_id)
    {
        gen_tag_t ret(0);
        ret.frame_id_ = frame_id;
        ret.symbol_id_ = symbol_id;
        ret.tag_type_ = TagType::kNone;
        return ret;
    }
};
static_assert(sizeof(gen_tag_t) == sizeof(size_t), "");

/**
 * Agora uses these event messages for communication between threads. Each
 * tag encodes information about a task.
 */
struct EventData {
    static constexpr size_t kMaxTags = 7;
    EventType event_type_;
    uint32_t num_tags_;
    size_t tags_[7];

    // Initialize and event with only the event type field set
    EventData(EventType event_type)
        : event_type_(event_type)
        , num_tags_(0)
    {
    }

    // Create an event with one tag
    EventData(EventType event_type, size_t tag)
        : event_type_(event_type)
        , num_tags_(1)
    {
        tags_[0] = tag;
    }

    EventData()
        : num_tags_(0)
    {
    }
};
static_assert(sizeof(EventData) == 64, "");

struct Packet {
    // The packet's data starts at kOffsetOfData bytes from the start
    static constexpr size_t kOffsetOfData = 64;

    uint32_t frame_id_;
    uint32_t symbol_id_;
    uint32_t cell_id_;
    uint32_t ant_id_;
    uint32_t fill_[12]; // Padding for 64-byte alignment needed for SIMD
    short data_[]; // Elements sent by antennae are two bytes (I/Q samples)
    Packet(int f, int s, int c, int a) // TODO: Should be unsigned integers
        : frame_id_(f)
        , symbol_id_(s)
        , cell_id_(c)
        , ant_id_(a)
    {
    }

    std::string ToString() const
    {
        std::ostringstream ret;
        ret << "[Frame seq num " << frame_id_ << ", symbol ID " << symbol_id_
            << ", cell ID " << cell_id_ << ", antenna ID " << ant_id_ << ", "
            << sizeof(fill_) << " empty bytes]";
        return ret.str();
    }
};

struct MacPacket {
    // The packet's data starts at kOffsetOfData bytes from the start
    static constexpr size_t kOffsetOfData = 16 + sizeof(RBIndicator);

    uint16_t frame_id_;
    uint16_t symbol_id_;
    uint16_t ue_id_;
    uint16_t datalen_; // length of payload in bytes or array data[]
    uint16_t crc_; // 16 bits CRC over calculated for the data[] array
    uint16_t rsvd_[3]; // reserved for future use
    RBIndicator rb_indicator_; // RAN scheduling details for PHY
    char data_[]; // Mac packet payload data
    MacPacket(int f, int s, int u, int d,
        int cc) // TODO: Should be unsigned integers
        : frame_id_(f)
        , symbol_id_(s)
        , ue_id_(u)
        , datalen_(d)
        , crc_(cc)
    {
    }

    std::string ToString() const
    {
        std::ostringstream ret;
        ret << "[Frame seq num " << frame_id_ << ", symbol ID " << symbol_id_
            << ", user ID " << ue_id_ << "]";
        return ret.str();
    }
};

class RxCounters {
public:
    // num_pkt[i] is the total number of packets we've received for frame i
    std::array<size_t, kFrameWnd> num_pkts_;

    // num_pilot_pkts[i] is the total number of pilot packets we've received
    // for frame i
    std::array<size_t, kFrameWnd> num_pilot_pkts_;

    // num_rc_pkts[i] is the total number of reciprocity pilot packets we've received
    // for frame i
    std::array<size_t, kFrameWnd> num_reciprocity_pkts_;

    // Number of packets we'll receive per frame on the uplink
    size_t num_pkts_per_frame_;

    // Number of pilot packets we'll receive per frame
    size_t num_pilot_pkts_per_frame_;

    // Number of reciprocity pilot packets we'll receive per frame
    size_t num_reciprocity_pkts_per_frame_;

    RxCounters()
    {
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
    // Maximum number of symbols in a frame
    size_t max_symbol_count_;
    // Maximum number of tasks in a symbol
    size_t max_task_count_;

    void Init(size_t max_symbol_count, size_t max_task_count = 0)
    {
        this->max_symbol_count_ = max_symbol_count;
        this->max_task_count_ = max_task_count;
        symbol_count_.fill(0);
    }

    /**
     * @brief Check whether the symbol is the last symbol for a given frame 
     * while simultaneously incrementing the symbol count.
     * @param frame id The frame id to check
     */
    bool LastSymbol(size_t frame_id)
    {
        const size_t frame_slot = frame_id % kFrameWnd;
        if (++symbol_count_[frame_slot] == max_symbol_count_) {
            // If the symbol is the last symbol, reset count to 0
            symbol_count_[frame_slot] = 0;
            return true;
        }
        return false;
    }

    /**
     * @brief Check whether the task is the last task for a given frame and 
     * symbol while simultaneously incrementing the task count.
     * @param frame_id The frame id to check
     * @param symbol_id The symbol id to check
     */
    bool LastTask(size_t frame_id, size_t symbol_id)
    {
        const size_t frame_slot = frame_id % kFrameWnd;
        if (++task_count_[frame_slot][symbol_id] == max_task_count_) {
            // If the task is the last task, reset count is to 0
            task_count_[frame_slot][symbol_id] = 0;
            return true;
        }
        return false;
    }

    /**
     * @brief Check whether the task is the last task for a given frame 
     * while simultaneously incrementing the task count.
     * This is used for tasks performed once per frame (e.g., ZF)
     * @param frame_id The frame id to check
     */
    bool LastTask(size_t frame_id)
    {
        const size_t frame_slot = frame_id % kFrameWnd;
        // Number of tasks is stored as number of symbols
        if (++symbol_count_[frame_slot] == max_symbol_count_) {
            // If the task is the last task, reset count to 0
            symbol_count_[frame_slot] = 0;
            return true;
        }
        return false;
    }

    size_t GetSymbolCount(size_t frame_id)
    {
        return symbol_count_[frame_id % kFrameWnd];
    }

    size_t GetTaskCount(size_t frame_id, size_t symbol_id)
    {
        return task_count_[frame_id % kFrameWnd][symbol_id];
    }

    size_t GetTaskCount(size_t frame_id)
    {
        return symbol_count_[frame_id % kFrameWnd];
    }

private:
    // task_count[i][j] is the number of tasks completed for
    // frame (i % kFrameWnd) and symbol j
    std::array<std::array<size_t, kMaxSymbols>, kFrameWnd> task_count_;
    // symbol_count[i] is the number of symbols completed for
    // frame (i % kFrameWnd)
    std::array<size_t, kFrameWnd> symbol_count_;
};

#endif
