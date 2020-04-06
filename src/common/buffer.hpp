/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 *
 */

#ifndef BUFFER_HEAD
#define BUFFER_HEAD

#include "Symbols.hpp"
#include "memory_manage.h"

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

struct complex_short {
    short re;
    short im;
};

// Event data tag for RX events
union rx_tag_t {
    struct {
        uint32_t tid : 4;
        uint32_t offset : 28;
    };
    int _tag;

    rx_tag_t(uint32_t tid, uint32_t offset)
        : tid(tid)
        , offset(offset)
    {
    }

    rx_tag_t(int _tag)
        : _tag(_tag)
    {
    }
};

// Event data tag for FFT task requests
using fft_req_tag_t = rx_tag_t;

// Event data tag for FFT responses responses
union fft_resp_tag_t {
    struct {
        uint32_t frame_id : 16;
        uint32_t subframe_id : 16;
    };
    int _tag;

    fft_resp_tag_t(uint32_t frame_id, uint32_t subframe_id)
        : frame_id(frame_id)
        , subframe_id(subframe_id)
    {
    }

    fft_resp_tag_t(int _tag)
        : _tag(_tag)
    {
    }
};

/**
 * Millipede uses these event messages for communication between threads
 *
 * @data is used for events with only a single offset
 *
 * num_offsets and offsets are used for events with multiple offsets
 *     num_offsets: number of offsets in an event
 *     offsets: the values of offsets
 */
struct Event_data {
    // TODO: @data can be removed and replaced with num_offsets and offsets
    // TODO: offsets should be called "data" or "tags" to avoid confusing with
    // offsets into memory buffers
    EventType event_type;
    int data;
    int num_offsets;
    int offsets[13];

    Event_data(EventType event_type, int data)
        : event_type(event_type)
        , data(data)
        , num_offsets(0)
    {
    }

    Event_data() { num_offsets = 0; }
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
};

struct RX_stats {
    int* task_count;
    int* task_pilot_count;
    // int frame_count = 0;
    int max_task_count;
    int max_task_pilot_count;
};

struct Frame_stats {
    int frame_count;
    int* symbol_count;
    int max_symbol_count;
    bool last_symbol(int frame_id)
    {
        if (++symbol_count[frame_id] == max_symbol_count) {
            symbol_count[frame_id] = 0;
            return (true);
        }
        return (false);
    }
    void init(int max_symbols, int max_frame, int align)
    {
        frame_count = 0;
        alloc_buffer_1d(&symbol_count, max_frame, align, 1);
        max_symbol_count = max_symbols;
    }
    void fini() { free_buffer_1d(&symbol_count); }
    void update_frame_count(void)
    {
        if (++frame_count == 1e9)
            frame_count = 0;
    }
};

struct ZF_stats : public Frame_stats {
    int coded_frame;
    int& max_task_count;
    ZF_stats(void)
        : max_task_count(max_symbol_count)
    {
    }
    void init(int max_tasks, int max_frame, int align)
    {
        Frame_stats::init(max_tasks, max_frame, align);
        coded_frame = -1;
    }
    void fini() { Frame_stats::fini(); }
};

struct Data_stats : public Frame_stats {
    Table<int> task_count;
    int max_task_count;

    void init(int max_tasks, int max_symbols, int max_frame,
        int max_data_subframe, int align)
    {
        Frame_stats::init(max_symbols, max_frame, align);
        task_count.calloc(max_frame, max_data_subframe, align);
        max_task_count = max_tasks;
    }
    void fini()
    {
        task_count.free();
        Frame_stats::fini();
    }
    bool last_task(int frame_id, int data_subframe_id)
    {
        if (++task_count[frame_id][data_subframe_id] == max_task_count) {
            task_count[frame_id][data_subframe_id] = 0;
            return (true);
        }
        return (false);
    }
};

struct FFT_stats : public Data_stats {
    int max_symbol_data_count;
    int* symbol_cal_count;
    int max_symbol_cal_count;
    int* cur_frame_for_symbol;
};

struct RC_stats {
    int frame_count;
    int max_task_count;
    RC_stats(void)
        : frame_count(0)
        , max_task_count(1)
    {
    }
    void update_frame_count(void)
    {
        if (++frame_count == 1e9)
            frame_count = 0;
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
