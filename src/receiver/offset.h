/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef OFFSET_H   
#define OFFSET_H


#include "Symbols.hpp"

inline int generateOffset2d(int unit_total_num, int frame_id, int unit_id) 
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * unit_total_num + unit_id;
};

inline int generateOffset3d(int unit_total_num, int frame_id, int current_data_subframe_id, int unit_id)
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    int total_data_subframe_id = frame_id * data_subframe_num_perframe + current_data_subframe_id;
    return total_data_subframe_id * unit_total_num + unit_id;
};

inline void interpreteOffset2d(int unit_total_num, int offset, int *frame_id, int *unit_id)
{
    *unit_id = offset % unit_total_num;
    *frame_id = offset / unit_total_num;
};

inline void interpreteOffset3d(int unit_total_num, int offset, int *frame_id, int *total_data_subframe_id, int *current_data_subframe_id, int *unit_id)
{
    *unit_id = offset % unit_total_num;
    *total_data_subframe_id = offset /unit_total_num;
    *current_data_subframe_id = (*total_data_subframe_id) % data_subframe_num_perframe;
    *frame_id = (*total_data_subframe_id) / data_subframe_num_perframe;
};


inline int getFFTBufferIndex(int frame_id, int subframe_id, int ant_id) 
{
    frame_id = frame_id % TASK_BUFFER_FRAME_NUM;
    return frame_id * (BS_ANT_NUM * subframe_num_perframe) + subframe_id * BS_ANT_NUM + ant_id;
};


inline void splitFFTBufferIndex(int FFT_buffer_target_id, int *frame_id, int *subframe_id, int *ant_id)
{
    (*frame_id) = FFT_buffer_target_id / (BS_ANT_NUM * subframe_num_perframe);
    FFT_buffer_target_id = FFT_buffer_target_id - (*frame_id) * (BS_ANT_NUM * subframe_num_perframe);
    (*subframe_id) = FFT_buffer_target_id / BS_ANT_NUM;
    (*ant_id) = FFT_buffer_target_id - *subframe_id * BS_ANT_NUM;
};


#endif