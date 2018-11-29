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
struct complex_float {
    float real;
    float imag;
};

//typedef std::vector<complex_float> myVec;
typedef std::vector<complex_float, boost::alignment::aligned_allocator<complex_float, 64>> myVec;

// structure for event
struct Event_data
{
    int event_type;
    int data;
};

// buffer of each socket thread
struct SocketBuffer
{
    // Received data
    char *buffer;
    int *buffer_status;
    // std::vector<char> buffer;
    // std::vector<int> buffer_status;
};

struct FFTBuffer
{
    // Data before IFFT
    // record TASK_BUFFER_FRAME_NUM entire frames
    complex_float ** FFT_inputs;
    complex_float ** FFT_outputs;
};

struct IFFTBuffer
{
    // Data before IFFT
    // record TASK_BUFFER_FRAME_NUM entire frames
    complex_float ** IFFT_inputs;
    complex_float ** IFFT_outputs;
};

struct CSIBuffer
{
    // CSI symbols after IFFT
    // record TASK_BUFFER_FRAME_NUM entire frames
    // inner vector is a BS_ANT_NUM * UE_NUM matrix
    // std::vector<myVec> CSI;
    complex_float ** CSI;
};

struct DataBuffer
{
    // Data symbols after IFFT    
    // record TASK_BUFFER_FRAME_NUM entire frames
    complex_float ** data;
    // std::vector<myVec> data;
};

struct PrecoderBuffer
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    // inner vector is a UE_NUM * BS_ANT_NUM matrix
    // std::vector<myVec> precoder;
    complex_float ** precoder;
};

struct EqualBuffer
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    // std::vector<myVec> data;
    complex_float ** data;
};

struct DemulBuffer
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    // std::vector<std::vector<long long>> data;
    long long ** data;
};

struct DLSocketBuffer
{
    std::vector<char> buffer;
};

// struct RawDataBuffer
// {
//     // Raw data symbols before modulation    
//     // record TASK_BUFFER_FRAME_NUM entire frames
//     std::vector<int> buffer;
//     std::vector<int> buffer_status;
// };


#endif
