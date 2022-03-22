#ifndef BUFFER_HEAD
#define BUFFER_HEAD

#include "Symbols.hpp"
#include "concurrentqueue.h"
#include "memory_manage.h"
#include "ran_config.h"
#include "utils.h"
#include <mutex>
#include <sstream>
#include <vector>

/* boost is required for aligned memory allocation (for SIMD instructions) */
#include "common_typedef_sdk.h"
#include <boost/align/aligned_allocator.hpp>

// Event data tag for RX events
union rx_tag_t {
    struct {
        size_t tid : 8; // ID of the socket thread that received the packet
        size_t offset : 56; // Offset in the socket thread's RX buffer
    };
    size_t _tag;

    rx_tag_t(size_t tid, size_t offset)
        : tid(tid)
        , offset(offset)
    {
    }

    rx_tag_t(size_t _tag)
        : _tag(_tag)
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
        uint32_t frame_id;
        uint16_t symbol_id : 13;
        TagType tag_type : 3;
        union {
            uint16_t cb_id; // code block
            uint16_t ue_id;
            uint16_t ant_id;
            uint16_t sc_id; // subcarrier id, the base of subcarrier range.
        };
    };

    size_t _tag;
    gen_tag_t(size_t _tag)
        : _tag(_tag)
    {
    }

    // Return a string representation of this tag
    std::string to_string()
    {
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
    static gen_tag_t frm_sym_cb(size_t frame_id, size_t symbol_id, size_t cb_id)
    {
        gen_tag_t ret(0);
        ret.frame_id = frame_id;
        ret.symbol_id = symbol_id;
        ret.tag_type = TagType::kCodeblocks;
        ret.cb_id = cb_id;
        return ret;
    }

    // Generate a tag with user ID, frame ID, and symbol ID bits set and
    // other fields blank
    static gen_tag_t frm_sym_ue(size_t frame_id, size_t symbol_id, size_t ue_id)
    {
        gen_tag_t ret(0);
        ret.frame_id = frame_id;
        ret.symbol_id = symbol_id;
        ret.tag_type = TagType::kUsers;
        ret.ue_id = ue_id;
        return ret;
    }

    // Generate a tag with frame ID, symbol ID, and subcarrier ID bits set and
    // other fields blank
    static gen_tag_t frm_sym_sc(size_t frame_id, size_t symbol_id, size_t sc_id)
    {
        gen_tag_t ret(0);
        ret.frame_id = frame_id;
        ret.symbol_id = symbol_id;
        ret.tag_type = TagType::kSubcarriers;
        ret.sc_id = sc_id;
        return ret;
    }

    // Generate a tag with antenna ID, frame ID, and symbol ID bits set and
    // other fields blank
    static gen_tag_t frm_sym_ant(
        size_t frame_id, size_t symbol_id, size_t ant_id)
    {
        gen_tag_t ret(0);
        ret.frame_id = frame_id;
        ret.symbol_id = symbol_id;
        ret.tag_type = TagType::kAntennas;
        ret.ant_id = ant_id;
        return ret;
    }

    // Generate a tag with frame ID and subcarrier ID bits set, and other fields
    // blank
    static gen_tag_t frm_sc(size_t frame_id, size_t sc_id)
    {
        gen_tag_t ret(0);
        ret.frame_id = frame_id;
        ret.symbol_id = kInvalidSymbolId;
        ret.tag_type = TagType::kSubcarriers;
        ret.sc_id = sc_id;
        return ret;
    }

    // Generate a tag with frame ID and symbol ID bits set, and other fields
    // blank
    static gen_tag_t frm_sym(size_t frame_id, size_t symbol_id)
    {
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

/// A struct pair containing a concurrent event queue
/// and a pointer to that queue's producer token.
struct sched_info_t {
    moodycamel::ConcurrentQueue<EventData> concurrent_q_;
    moodycamel::ProducerToken* ptok_;

    sched_info_t() = default;

    sched_info_t(moodycamel::ConcurrentQueue<EventData> conq)
        : concurrent_q_(std::move(conq)) // moodycamel queue can't be copied
    {
        ptok_ = new moodycamel::ProducerToken(concurrent_q_);
    }
};

struct Packet {
    // The packet's data starts at kOffsetOfData bytes from the start
    static constexpr size_t kOffsetOfData = 64;

    enum class PktType {
        kInvalid,
        kTimeIQ, // A packet containing time-domain IQ samples from the RRU
        kFreqIQ, // A packet containing frequency-domain IQ samples from the RRU or servers
        kIQFromServer, // A packet containing IQ samples from the server
        kDemod, // A packet generated after the demodulation stage
        kEncode, // A packet generated after the encode stage
        kFFT,
        kPostZF
    };

    static const char* PktTypeStr(PktType pkt_type)
    {
        switch (pkt_type) {
        case PktType::kFreqIQ:
            return "[Freq IQ from RRU]";
        case PktType::kDemod:
            return "[Demodulated data]";
        case PktType::kTimeIQ:
            return "[Time IQ from RRU]";
        case PktType::kEncode:
            return "[Encoded data]";
        case PktType::kFFT:
            return "[Post FFT data]";
        case PktType::kPostZF:
            return "[Post ZF data]";
        default:
            return "[Invalid]";
        }
    }

    PktType pkt_type_;
    uint32_t frame_id_;
    uint32_t symbol_id_;
    uint32_t cell_id_;
    uint32_t ue_id_;
    uint32_t ant_id_;
    uint32_t server_id_;
    uint32_t sc_id_;
    uint32_t sc_len_;
    uint32_t data_off_;
    uint32_t data_len_;
    uint32_t fill_[5]; // Padding for 64-byte alignment needed for SIMD
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
        ret << "[Packet type " << PktTypeStr(pkt_type_) << ", Frame seq num "
            << frame_id_ << ", symbol ID " << symbol_id_ << ", cell ID "
            << cell_id_ << ", antenna ID " << ant_id_ << ", " << sizeof(fill_)
            << " empty bytes]";
        return ret.str();
    }

    void PrintContent(size_t length) const
    {
        for (size_t i = 0; i < length; i ++) {
            printf("%02x ", ((uint8_t*)data_)[i]);
        }
        printf("\n");
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

#endif
