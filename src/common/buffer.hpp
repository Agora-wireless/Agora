/**
 * Author: Peiyao Zhao
 * E-Mail: pdszpy19930218@163.com
 * 
 */


#ifndef BUFFER_HEAD
#define BUFFER_HEAD

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
    int *task_count;
    int *task_pilot_count;
    int *fft_created_count;
    // int frame_count = 0;
    int max_task_count;
    int max_task_pilot_count;
};

struct FFT_stats {
    int **task_count;
    int *symbol_pilot_count;
    int *symbol_data_count;
    bool **data_exist_in_symbol;
    int frame_count = 0;
    int max_task_count;
    int max_symbol_pilot_count;
    int max_symbol_data_count;
};

struct ZF_stats {
    int *task_count;
    int frame_count = 0;
    int max_task_count;
    bool *precoder_exist_in_frame;
};

struct Data_stats {
    int **task_count;
    int *symbol_count;
    int frame_count = 0;
    int max_task_count;
    int max_symbol_count;
};

// struct Decode_stats {
//     int **block_count;
//     int *count_symbols;
//     int frame_count = 0;
//     int count_block_max;
//     int max_symbol_count;
// };


struct FFTBuffer
{
    // Data before IFFT
    // record TASK_BUFFER_FRAME_NUM entire frames
    complex_float **FFT_inputs;
    complex_float **FFT_outputs;
};

struct IFFTBuffer
{
    // Data before IFFT
    // record TASK_BUFFER_FRAME_NUM entire frames
    complex_float **IFFT_inputs;
    complex_float **IFFT_outputs;
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

inline void interpretOffset2d(size_t max_dim2, size_t offset, size_t *dim1_id, size_t *dim2_id)
{
    *dim2_id = offset % max_dim2;
    *dim1_id = offset / max_dim2;
}

inline void interpretOffset3d(size_t max_dim2, size_t max_dim3, size_t offset, size_t *dim1_id, size_t *dim2d_id, size_t *dim2_id, size_t *dim3_id)
{
    *dim3_id = offset % max_dim3;
    *dim2d_id = offset / max_dim3;
    *dim2_id = (*dim2d_id) % max_dim2;
    *dim1_id = (*dim2d_id) / max_dim2;
}


#endif
