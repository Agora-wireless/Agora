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
typedef std::vector<complex_float, boost::alignment::aligned_allocator<complex_float, 32>> myVec;

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
    std::vector<char> buffer;
    std::vector<int> buffer_status;
};

struct FFTBuffer
{
    // Data before IFFT
    // record TASK_BUFFER_FRAME_NUM entire frames
    complex_float ** FFT_inputs;
    complex_float ** FFT_outputs;
};

struct CSIBuffer
{
    // CSI symbols after IFFT
    // record TASK_BUFFER_FRAME_NUM entire frames
    // inner vector is a BS_ANT_NUM * UE_NUM matrix
    std::vector<myVec> CSI;
};

struct DataBuffer
{
    // Data symbols after IFFT    
    // record TASK_BUFFER_FRAME_NUM entire frames
    std::vector<myVec> data;
};

struct PrecoderBuffer
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    // inner vector is a UE_NUM * BS_ANT_NUM matrix
    std::vector<myVec> precoder;
};

struct DemulBuffer
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    std::vector<myVec> data;
};

struct DemulBuffer2
{
    // record TASK_BUFFER_FRAME_NUM entire frames
    std::vector<std::vector<int>> data;
};


#endif
