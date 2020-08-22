/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 *
 */

#ifndef BUFFER_HEAD
#define BUFFER_HEAD

#include "Symbols.hpp"
#include "concurrentqueue.h"
#include "memory_manage.h"
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

// A generic tag type for Millipede tasks. The tag for a particular task will
// have only a subset of the fields initialized.
union gen_tag_t {
    static constexpr size_t kInvalidSymbolId = (1ull << 13) - 1;
    static_assert(kMaxSymbolsPerFrame < ((1ull << 13) - 1), "");
    static_assert(kMaxUEs < UINT16_MAX, "");
    static_assert(kMaxAntennas < UINT16_MAX, "");
    static_assert(k5GMaxSubcarriers < UINT16_MAX, "");

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
 * Millipede uses these event messages for communication between threads. Each
 * tag encodes information about a task.
 */
struct Event_data {
    static constexpr size_t kMaxTags = 7;
    EventType event_type;
    uint32_t num_tags;
    size_t tags[7];

    // Initialize and event with only the event type field set
    Event_data(EventType event_type)
        : event_type(event_type)
        , num_tags(0)
    {
    }

    // Create an event with one tag
    Event_data(EventType event_type, size_t tag)
        : event_type(event_type)
        , num_tags(1)
    {
        tags[0] = tag;
    }

    Event_data()
        : num_tags(0)
    {
    }
};
static_assert(sizeof(Event_data) == 64, "");

/// A struct pair containing a concurrent event queue
/// and a pointer to that queue's producer token.
struct sched_info_t {
    moodycamel::ConcurrentQueue<Event_data> concurrent_q;
    moodycamel::ProducerToken* ptok;

    sched_info_t() = default;

    sched_info_t(moodycamel::ConcurrentQueue<Event_data> conq)
        : concurrent_q(std::move(conq)) // moodycamel queue can't be copied
    {
        ptok = new moodycamel::ProducerToken(concurrent_q);
    }
};

struct Packet {
    // The packet's data starts at kOffsetOfData bytes from the start
    static constexpr size_t kOffsetOfData = 64;

    uint32_t frame_id;
    uint32_t symbol_id;
    uint32_t cell_id;
    uint32_t ant_id;
    uint32_t fill[12]; // Padding for 64-byte alignment needed for SIMD
    short data[]; // Elements sent by antennae are two bytes (I/Q samples)
    Packet(int f, int s, int c, int a) // TODO: Should be unsigned integers
        : frame_id(f)
        , symbol_id(s)
        , cell_id(c)
        , ant_id(a)
    {
    }

    std::string to_string()
    {
        std::ostringstream ret;
        ret << "[Frame seq num " << frame_id << ", symbol ID " << symbol_id
            << ", cell ID " << cell_id << ", antenna ID " << ant_id << "]";
        return ret.str();
    }
};

struct MacPacket {
    // The packet's data starts at kOffsetOfData bytes from the start
    static constexpr size_t kOffsetOfData = 16;

    uint16_t frame_id;
    uint16_t symbol_id;
    uint16_t ue_id;
    uint16_t datalen; // length of payload in bytes or array data[]
    uint16_t crc; // 16 bits CRC over calculated for the data[] array
    uint16_t rsvd[3]; // reserved for future use
    char data[]; // Mac packet payload data
    MacPacket(int f, int s, int u, int d,
        int cc) // TODO: Should be unsigned integers
        : frame_id(f)
        , symbol_id(s)
        , ue_id(u)
        , datalen(d)
        , crc(cc)
    {
    }

    std::string to_string()
    {
        std::ostringstream ret;
        ret << "[Frame seq num " << frame_id << ", symbol ID " << symbol_id
            << ", user ID " << ue_id << "]";
        return ret.str();
    }
};

class RxCounters {
public:
    // num_pkt[i] is the total number of packets we've received for frame i
    std::array<size_t, TASK_BUFFER_FRAME_NUM> num_pkts;

    // num_pilot_pkts[i] is the total number of pilot packets we've received
    // for frame i
    std::array<size_t, TASK_BUFFER_FRAME_NUM> num_pilot_pkts;

    // num_rc_pkts[i] is the total number of reciprocity pilot packets we've received
    // for frame i
    std::array<size_t, TASK_BUFFER_FRAME_NUM> num_reciprocity_pkts;

    // Number of packets we'll receive per frame on the uplink
    size_t num_pkts_per_frame;

    // Number of pilot packets we'll receive per frame
    size_t num_pilot_pkts_per_frame;

    // Number of reciprocity pilot packets we'll receive per frame
    size_t num_reciprocity_pkts_per_frame;

    RxCounters()
    {
        num_pkts.fill(0);
        num_pilot_pkts.fill(0);
        num_reciprocity_pkts.fill(0);
    }
};

// We use one RxStatus object to track packet reception status.
// This object is shared between socket threads and subcarrier workers.
class RxStatus {
public:
    // num_pkts[i % TASK_BUFFER_FRAME_NUM] is the total number of packets
    // received for frame i (may be removed if not used)
    std::array<std::atomic<size_t>, TASK_BUFFER_FRAME_NUM> num_pkts;

    // num_pilot_pkts[i % TASK_BUFFER_FRAME_NUM] is the total number of pilot
    // packets received for frame i
    std::array<std::atomic<size_t>, TASK_BUFFER_FRAME_NUM> num_pilot_pkts;

    // num_data_pkts[i % TASK_BUFFER_FRAME_NUM][j] is the total number of data
    // packets received for frame i and symbol j
    std::array<std::array<std::atomic<size_t>, kMaxNumSymbolsPerFrame>,
        TASK_BUFFER_FRAME_NUM>
        num_data_pkts;

    // Millipede processes one frame at a time (roughly speaking). cur_frame is
    // the frame currently being processed.
    size_t cur_frame = 0;

    // The max frame number for which socket I/O threads have received any packet
    size_t latest_frame = 0;

    // The first data symbol for which the socket I/O threads have not received
    // all packets
    // size_t next_data_symbol = 0;

    // Atomic counter for # completed decode tasks
    // cur_frame will be incremented in all tasks are completed
    size_t num_decode_tasks_completed;
    std::mutex decode_mutex;

    // Copies of Config variables
    const size_t num_pilot_pkts_per_frame;
    const size_t num_pilot_symbols_per_frame;
    const size_t num_data_symbol_per_frame;
    const size_t num_pkts_per_symbol;
    const size_t num_decode_tasks_per_frame;

    RxStatus(size_t _num_pilot_pkts_per_frame,
        size_t _num_pilot_symbols_per_frame, size_t _num_data_symbol_per_frame,
        size_t _num_pkts_per_symbol, size_t _num_decode_tasks_per_frame)
        : num_pilot_pkts_per_frame(_num_pilot_pkts_per_frame)
        , num_pilot_symbols_per_frame(_num_pilot_symbols_per_frame)
        , num_data_symbol_per_frame(_num_data_symbol_per_frame)
        , num_pkts_per_symbol(_num_pkts_per_symbol)
        , num_decode_tasks_per_frame(_num_decode_tasks_per_frame)
    {
        // num_pkts.fill(0);
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            num_pkts[i].store(0);
        }
        // num_pilot_pkts.fill(0);
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            num_pilot_pkts[i].store(0);
        }
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            // num_data_pkts[i].fill(0);
            for (size_t j = 0; j < kMaxNumSymbolsPerFrame; j++) {
                num_data_pkts[i][j].store(0);
            }
        }
        // next_data_symbol = num_pilot_symbols_per_frame;
    }

    bool add_new_packet(size_t frame_id, size_t symbol_id)
    {
        if (frame_id >= cur_frame + TASK_BUFFER_FRAME_NUM) {
            std::cout << "Error: Received packet for future frame beyond "
                         "frame "
                      << "window. This can happen if Millipede is running "
                      << "slowly, e.g., in debug mode\n";
            return false;
        }
        size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        num_pkts[frame_slot]++;
        if (num_pkts[frame_slot].load()
            == num_pkts_per_symbol
                * (num_pilot_symbols_per_frame + num_data_symbol_per_frame)) {
            printf(
                "Main thread: received all packets in frame: %lu\n", frame_id);
        }
        // TODO
        // 1. Move those statements into functions in RxStatus
        // 2. Check invalid packets
        if (symbol_id < num_pilot_symbols_per_frame) {
            num_pilot_pkts[frame_slot]++;
            if (num_pilot_pkts[frame_slot].load() == num_pilot_pkts_per_frame) {
                printf("Main thread: received all pilots in frame: %zu\n",
                    frame_id);
            }
        } else {
            num_data_pkts[frame_slot][symbol_id]++;
        }
        if (frame_id > latest_frame) {
            // NOTE: race condition could happen here but the impact is small
            latest_frame = frame_id;
        }
        // if (frame_id == cur_frame && symbol_id == next_data_symbol) {
        //     while (
        //         num_data_pkts[frame_slot][symbol_id] == num_pkts_per_symbol) {
        //         num_data_pkts[frame_slot][symbol_id] = 0;
        //         symbol_id++;
        //         if (symbol_id == num_data_symbol_per_frame) {
        //             break;
        //         }
        //     }
        //     next_data_symbol = symbol_id;
        // }
        return true;
    }

    bool is_pilot_ready(size_t frame_id)
    {
        // if (frame_id == 1) {
        //     printf("check pilot in frame 1 %lu %lu cur_frame %lu\n",
        //         num_pilot_pkts[1], num_pilot_pkts_per_frame, cur_frame);
        // }
        if (frame_id < cur_frame
            || frame_id >= cur_frame + TASK_BUFFER_FRAME_NUM) {
            return false;
        }
        return num_pilot_pkts[frame_id % TASK_BUFFER_FRAME_NUM].load()
            == num_pilot_pkts_per_frame;
    }

    bool is_demod_ready(size_t frame_id, size_t symbol_id)
    {
        // return frame_id == cur_frame && next_data_symbol > symbol_id;
        if (frame_id < cur_frame
            || frame_id >= cur_frame + TASK_BUFFER_FRAME_NUM) {
            return false;
        }
        return num_data_pkts[frame_id % TASK_BUFFER_FRAME_NUM][symbol_id].load()
            == num_pkts_per_symbol;
    }

    void decode_done(size_t frame_id)
    {
        rt_assert(frame_id == cur_frame, "Wrong completed decode task!");
        decode_mutex.lock();
        num_decode_tasks_completed++;
        if (num_decode_tasks_completed == num_decode_tasks_per_frame) {
            cur_frame++;
            num_decode_tasks_completed = 0;
            size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
            num_pkts[frame_slot].store(0);
            num_pilot_pkts[frame_slot].store(0);
            // num_data_pkts[frame_slot].fill(0);
            for (size_t j = 0; j < kMaxNumSymbolsPerFrame; j++) {
                num_data_pkts[frame_slot][j].store(0);
            }
            printf("Main thread: Decode done frame: %lu\n", cur_frame - 1);
        }
        decode_mutex.unlock();
    }
};

// We use DecodeStatus to track # completed demul tasks for each symbol
// This object is shared between all dosubcarriers and dodecoders
class DemulStatus {
public:
    // num_demul_tasks_completed[i % TASK_BUFFER_FRAME_NUM][j] is
    // the number of subcarriers completed for demul tasks in
    // frame i and symbol j
    std::array<std::array<size_t, kMaxNumSymbolsPerFrame>,
        TASK_BUFFER_FRAME_NUM>
        num_demul_tasks_completed;

    // # subcarriers required to demul for each symbol
    const size_t num_demul_tasks_required;

    // Create a mutex for each symbol
    // to synchronize the update for each dosubcarrier
    std::array<std::mutex, kMaxNumSymbolsPerFrame> mutex_list;

    size_t cur_frame;
    std::mutex cur_frame_mutex;

    DemulStatus(size_t _num_demul_tasks_required)
        : num_demul_tasks_required(_num_demul_tasks_required)
    {
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++) {
            num_demul_tasks_completed[i].fill(0);
        }
        cur_frame = 0;
    }

    // A dosubcarrier launch this function to notify some subcarriers
    // are completed for demul tasks in a symbol of a frame
    void demul_complete(size_t frame_id, size_t symbol_id, size_t num_tasks)
    {
        rt_assert(
            frame_id < cur_frame + TASK_BUFFER_FRAME_NUM, "Decode too slow!");
        mutex_list[symbol_id].lock();
        num_demul_tasks_completed[frame_id % TASK_BUFFER_FRAME_NUM][symbol_id]
            += num_tasks;
        mutex_list[symbol_id].unlock();
    }

    // A dodecode checks whether all subcarriers in a symbol complete their
    // demul tasks so that it could start to decode
    bool ready_to_decode(size_t frame_id, size_t symbol_id)
    {
        if (frame_id > cur_frame) {
            cur_frame_mutex.lock();
            if (frame_id > cur_frame) {
                num_demul_tasks_completed[cur_frame % TASK_BUFFER_FRAME_NUM]
                    .fill(0);
                cur_frame++;
            }
            cur_frame_mutex.unlock();
        }
        return num_demul_tasks_completed[frame_id % TASK_BUFFER_FRAME_NUM]
                                        [symbol_id]
            == num_demul_tasks_required;
    }
};

class CSI_stats {
public:
    size_t num_tasks;
    bool last_task(size_t frame_id)
    {
        rt_assert(
            frame_id >= min_frame_id, "Invalid CSI task from previous frame!");
        rt_assert(frame_id < min_frame_id + TASK_BUFFER_FRAME_NUM,
            "CSI task buffer overflow!");
        const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        if (++task_count[frame_slot] == num_tasks) {
            task_count[frame_slot] = 0;
            rt_assert(
                frame_id == min_frame_id, "Future CSI tasks complete first!");
            min_frame_id++;
            return true;
        }
        return false;
    }

    void init(size_t _num_tasks)
    {
        num_tasks = _num_tasks;
        min_frame_id = 0;
        task_count.fill(0);
    }

private:
    std::array<size_t, TASK_BUFFER_FRAME_NUM> task_count;
    size_t min_frame_id;
};

class Frame_stats {
public:
    size_t max_symbol_count;
    bool last_symbol(int frame_id)
    {
        const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        if (++symbol_count[frame_slot] == max_symbol_count) {
            symbol_count[frame_slot] = 0;
            return true;
        }
        return false;
    }

    void init(int _max_symbol_count)
    {
        symbol_count.fill(0);
        max_symbol_count = _max_symbol_count;
    }

    size_t get_symbol_count(size_t frame_id)
    {
        return symbol_count[frame_id % TASK_BUFFER_FRAME_NUM];
    }

private:
    std::array<size_t, TASK_BUFFER_FRAME_NUM> symbol_count;
};

class ZF_stats : public Frame_stats {
public:
    size_t coded_frame;
    size_t& max_task_count;
    ZF_stats(void)
        : max_task_count(max_symbol_count)
    {
    }
    void init(int max_tasks)
    {
        Frame_stats::init(max_tasks);
        coded_frame = SIZE_MAX;
    }
};

class Data_stats : public Frame_stats {
public:
    size_t max_task_count;

    void init(int _max_task_count, int max_symbols, int max_data_symbol)
    {
        Frame_stats::init(max_symbols);
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
            task_count[i] = new size_t[max_data_symbol]();
        max_task_count = _max_task_count;
    }
    void fini()
    {
        for (size_t i = 0; i < TASK_BUFFER_FRAME_NUM; i++)
            delete[] task_count[i];
    }
    bool last_task(size_t frame_id, size_t data_symbol_id)
    {
        const size_t frame_slot = frame_id % TASK_BUFFER_FRAME_NUM;
        if (++task_count[frame_slot][data_symbol_id] == max_task_count) {
            task_count[frame_slot][data_symbol_id] = 0;
            return true;
        }
        return false;
    }

    size_t get_task_count(size_t frame_id, size_t symbol_id)
    {
        return task_count[frame_id % TASK_BUFFER_FRAME_NUM][symbol_id];
    }

private:
    size_t* task_count[TASK_BUFFER_FRAME_NUM];
};

class FFT_stats : public Data_stats {
public:
    size_t max_symbol_data_count;
    std::array<size_t, TASK_BUFFER_FRAME_NUM> symbol_rc_count;
    size_t max_symbol_rc_count;

    // cur_frame_for_symbol[i] is the current frame for the symbol whose
    // index in the frame's uplink symbols is i
    std::vector<size_t> cur_frame_for_symbol;
};

class RC_stats {
public:
    size_t max_task_count;
    size_t last_frame;
    RC_stats(void)
        : max_task_count(1)
        , last_frame(SIZE_MAX)
    {
    }
};

#endif
