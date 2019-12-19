/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef DOPRECODE
#define DOPRECODE

#include <iostream>
#include <stdio.h>  /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>
#include <armadillo>
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "Symbols.hpp"
#include "gettime.h"
#include "offset.h"
#include "modulation.hpp"
#include "config.hpp"
#include "memory_manage.h"
#include "stats.hpp"
// #include "mkl_dfti.h"
class Consumer;

class DoPrecode
{
public:
    DoPrecode(Config *cfg, int in_tid, int in_demul_block_size, int in_transpose_block_size,
	      Consumer &in_consumer,
	Table<complex_float> &in_dl_modulated_buffer, Table<complex_float> &in_precoder_buffer,
	Table<complex_float> &in_dl_precoded_data_buffer, Table<complex_float> &in_dl_ifft_buffer,
#ifdef USE_LDPC
	Table<int8_t> &in_dl_encoded_data,
#else
	Table<int8_t> &in_dl_IQ_data,
#endif
        Stats *in_stats_manager);
    ~DoPrecode();

    /**
     * Do demodulation task for a block of subcarriers (demul_block_size)
     * @param tid: task thread index, used for selecting spm_buffer and task ptok
     * @param offset: offset of the first subcarrier in the block in data_buffer_
     * Buffers: data_buffer_, spm_buffer_, precoder_buffer_, equal_buffer_, demul_hard_buffer_
     *     Input buffer: data_buffer_, precoder_buffer_
     *     Output buffer: demul_hard_buffer_
     *     Intermediate buffer: spm_buffer, equal_buffer_
     * Offsets: 
     *     data_buffer_: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: transpose block index * block size * # of antennas + antenna index * block size
     *     spm_buffer: 
     *         dim1: task thread index
     *         dim2: antenna index
     *     precoder_buffer_: 
     *         dim1: frame index * FFT size + subcarrier index in the current frame
     *     equal_buffer_, demul_buffer: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: subcarrier index * # of users
     * Event offset: offset
     * Description: 
     *     1. for each subcarrier in the block, block-wisely copy data from data_buffer_ to spm_buffer_
     *     2. perform equalization with data and percoder matrixes
     *     3. perform demodulation on equalized data matrix   
     *     4. add an event to the message queue to infrom main thread the completion of this task
     */
    void Precode(int offset);
    
 
private:
    Config *config_;
    int BS_ANT_NUM, UE_NUM;
    int OFDM_DATA_NUM;
    int OFDM_DATA_START;
    int data_subframe_num_perframe;

    int tid;
    int transpose_block_size;
    int demul_block_size;
    Consumer &consumer_;
    
    Table<complex_float> &dl_modulated_buffer_;
    Table<complex_float> &precoder_buffer_;
    Table<complex_float> &dl_precoded_data_buffer_;
    Table<complex_float> &dl_ifft_buffer_;
    Table<int8_t> &dl_IQ_data;
    Table<float> qam_table;

    Table<double> &Precode_task_duration;
    int *Precode_task_count;

    complex_float *modulated_buffer_temp;
    complex_float *precoded_buffer_temp;


};


#endif
