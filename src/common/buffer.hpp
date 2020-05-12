/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 *
 */

#ifndef BUFFER_HEAD
#define BUFFER_HEAD

#include "Symbols.hpp"
#include "memory_manage.h"
#include <sstream>
#include <vector>

/* boost is required for aligned memory allocation (for SIMD instructions) */
#include <boost/align/aligned_allocator.hpp>
#ifdef USE_LDPC
#include "common_typedef_sdk.h"
#else
struct complex_float {
    float re;
    float im;
};
#endif

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
    static constexpr size_t kInvalidSymbolId = (1ull << 14) - 1;
    static_assert(kMaxSymbolsPerFrame < ((1ull << 14) - 1), "");
    static_assert(kMaxUEs < UINT16_MAX, "");
    static_assert(kMaxAntennas < UINT16_MAX, "");
    static_assert(k5GMaxSubcarriers < UINT16_MAX, "");

    enum TagType { kUEs, kAntennas, kSubcarriers, kNone };

    struct {
        uint32_t frame_id;
        uint16_t symbol_id : 14;
        TagType tag_type : 2;
        union {
            uint16_t ue_id;
            uint16_t ant_id;
            uint16_t sc_id;
        };
    };

    size_t _tag;
    gen_tag_t(size_t _tag)
        : _tag(_tag)
    {
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

struct Packet {
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

class RX_stats {
public:
    std::array<size_t, TASK_BUFFER_FRAME_NUM> task_count;
    std::array<size_t, TASK_BUFFER_FRAME_NUM> task_pilot_count;
    std::array<size_t, TASK_BUFFER_FRAME_NUM> task_rc_count;
    size_t max_task_count; // Max packets per frame
    size_t max_task_pilot_count; // Max pilot packets per frame
    size_t max_task_rc_count; // Max reciprocity packets per frame
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
        coded_frame = -1;
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
    std::array<size_t, TASK_BUFFER_FRAME_NUM> symbol_cal_count;
    size_t max_symbol_cal_count;

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

/* TODO: clean up the legency code below */
struct FFTBuffer {
    // Data before IFFT
    // record TASK_BUFFER_FRAME_NUM entire frames
    Table<complex_float> FFT_inputs;
    Table<complex_float> FFT_outputs;
};

struct IFFTBuffer {
    // Data before IFFT
    // record TASK_BUFFER_FRAME_NUM entire frames
    Table<complex_float> IFFT_inputs;
    Table<complex_float> IFFT_outputs;
};

inline size_t generateOffset2d(
    size_t max_dim1, size_t max_dim2, size_t dim1_id, size_t dim2_id)
{
    dim1_id = dim1_id % max_dim1;
    return dim1_id * max_dim2 + dim2_id;
}

inline size_t generateOffset3d(size_t max_dim1, size_t max_dim2,
    size_t max_dim3, size_t dim1_id, size_t dim2_id, size_t dim3_id)
{
    dim1_id = dim1_id % max_dim1;
    size_t dim2d_id = dim1_id * max_dim2 + dim2_id;
    return dim2d_id * max_dim3 + dim3_id;
}

inline void interpretOffset2d(
    size_t max_dim2, size_t offset, size_t* dim1_id, size_t* dim2_id)
{
    *dim2_id = offset % max_dim2;
    *dim1_id = offset / max_dim2;
}

inline void interpretOffset3d(size_t max_dim2, size_t max_dim3, size_t offset,
    size_t* dim1_id, size_t* dim2d_id, size_t* dim2_id, size_t* dim3_id)
{
    *dim3_id = offset % max_dim3;
    *dim2d_id = offset / max_dim3;
    *dim2_id = (*dim2d_id) % max_dim2;
    *dim1_id = (*dim2d_id) / max_dim2;
}

#endif
