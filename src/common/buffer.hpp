/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */

#ifndef BUFFER_HEAD
#define BUFFER_HEAD

#include "memory_manage.h"

// boost is required for aligned memory allocation (for SIMD instructions)
#include <boost/align/aligned_allocator.hpp>

// size: 8 bytes
#ifdef USE_LDPC
#include "common_typedef_sdk.h"
#else
struct complex_float {
    float re;
    float im;
};
#endif

// structure for event
struct Event_data {
    int event_type;
    int data;
    // int more_data;
};

struct Packet {
    uint32_t frame_id;
    uint32_t symbol_id;
    uint32_t cell_id;
    uint32_t ant_id;
    short data[];
    Packet(int f, int s, int c, int a)
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
    int* fft_created_count;
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
    void fini()
    {
        free_buffer_1d(&symbol_count);
    }
    void update_frame_count(void)
    {
        if (++frame_count == 1e9)
            frame_count = 0;
    }
};

struct ZF_stats : public Frame_stats {
    bool* precoder_exist_in_frame;
    int& max_task_count;
    ZF_stats(void)
        : max_task_count(max_symbol_count)
    {
    }
    void init(int max_tasks, int max_frame, int align)
    {
        Frame_stats::init(max_tasks, max_frame, align);
        alloc_buffer_1d(&precoder_exist_in_frame, max_frame, align, 1);
    }
    void fini()
    {
        free_buffer_1d(&precoder_exist_in_frame);
        Frame_stats::fini();
    }
};

struct Data_stats : public Frame_stats {
    int frame_count;
    int* symbol_count;
    int max_symbol_count;

    Table<int> task_count;
    int max_task_count;

    void init(int max_tasks, int max_symbols, int max_frame, int max_data_subframe, int align)
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
    int* symbol_data_count;
    int max_symbol_data_count;
    Table<bool> data_exist_in_symbol;
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

inline size_t generateOffset2d(size_t max_dim1, size_t max_dim2, size_t dim1_id, size_t dim2_id)
{
    dim1_id = dim1_id % max_dim1;
    return dim1_id * max_dim2 + dim2_id;
}

inline size_t generateOffset3d(size_t max_dim1, size_t max_dim2, size_t max_dim3, size_t dim1_id, size_t dim2_id, size_t dim3_id)
{
    dim1_id = dim1_id % max_dim1;
    size_t dim2d_id = dim1_id * max_dim2 + dim2_id;
    return dim2d_id * max_dim3 + dim3_id;
}

inline void interpretOffset2d(size_t max_dim2, size_t offset, size_t* dim1_id, size_t* dim2_id)
{
    *dim2_id = offset % max_dim2;
    *dim1_id = offset / max_dim2;
}

inline void interpretOffset3d(size_t max_dim2, size_t max_dim3, size_t offset, size_t* dim1_id, size_t* dim2d_id, size_t* dim2_id, size_t* dim3_id)
{
    *dim3_id = offset % max_dim3;
    *dim2d_id = offset / max_dim3;
    *dim2_id = (*dim2d_id) % max_dim2;
    *dim1_id = (*dim2d_id) / max_dim2;
}

#endif
