/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef DOFFT
#define DOFFT

#include <iostream>
#include <stdio.h>  /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "Symbols.hpp"
#include "gettime.h"
#include "offset.h"
#include "mkl_dfti.h"
#include "mufft/fft.h"
#include "config.hpp"
#include "stats.hpp"
class Consumer;

class DoFFT
{
public:
    DoFFT(Config *cfg, int in_tid, Consumer &in_consumer,
	Table<char> &in_socket_buffer, Table<int> &in_socket_buffer_status, Table<complex_float> &in_data_buffer, Table<complex_float> &in_csi_buffer,
        Table<complex_float> &in_dl_ifft_buffer, char *in_dl_socket_buffer, 
        Stats *in_stats_manager);
    ~DoFFT();

    /**
     * Do FFT task for one OFDM symbol 
     * @param tid: task thread index, used for selecting muplans and task ptok
     * @param offset: offset of the OFDM symbol in socket_buffer_
     * Buffers: socket_buffer_, fft_buffer_, csi_buffer_, data_buffer_ 
     *     Input buffer: socket_buffer_
     *     Output buffer: csi_buffer_ if subframe is pilot
     *                    data_buffer_ if subframe is data
     *     Intermediate buffer: fft_buffer_ (FFT_inputs, FFT_outputs)
     * Offsets: 
     *     socket_buffer_: 
     *         dim1: socket thread index: (offset / # of OFDM symbols per thread)
     *         dim2: OFDM symbol index in this socket thread (offset - # of subframes in previous threads)
     *     FFT_inputs, FFT_outputs: 
     *         dim1: frame index * # of OFDM symbols per frame + subframe index * # of atennas + antenna index
     *         dim2: subcarrier index
     *     csi_buffer_: 
     *         dim1: frame index * FFT size + subcarrier index in the current frame
     *         dim2: user index * # of antennas + antenna index
     *     data_buffer_: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: transpose block index * block size * # of antennas + antenna index * block size
     * Event offset: frame index * # of subframe per frame + subframe index
     * Description: 
     *     1. copy received data (one OFDM symbol) from socket_buffer to fft_buffer_.FFT_inputs (remove CP)
     *     2. perform FFT on fft_buffer_.FFT_inputs and store results in fft_buffer_.FFT_outputs
     *     3. if subframe is pilot, do channel estimation from fft_buffer_.FFT_outputs to csi_buffer_
     *        if subframe is data, copy data from fft_buffer_.FFT_outputs to data_buffer_ and do block transpose     
     *     4. add an event to the message queue to infrom main thread the completion of this task
     */
    void FFT(int offset);


    /**
     * Do modulation and ifft tasks for one OFDM symbol
     * @param tid: task thread index, used for selecting task ptok
     * Buffers: dl_IQ_data_long
     *     Input buffer: dl_IQ_data_long
     *     Output buffer: dl_iffted_data_buffer_
     *     Intermediate buffer: dl_ifft_buffer_
     * Offsets: 
     *     dl_IQ_data_long_: 
     *         dim1: data subframe index in the current frame * # of users + user index
     *         dim2: subcarrier index
     *     dl_ifft_buffer_: 
     *         dim1: frame index * # of data subframes per frame * # of users + data subframe index * # of users + user index
     *         dim2: subcarrier index 
     *     dl_iffted_data_buffer_: 
     *         dim1: frame index * # of data subframes per frame + data subframe index
     *         dim2: transpose block index * block size * # of UEs + user index * block size
     * Event offset: offset
     * Description: 
     *     1. for each OFDM symbol, perform modulation and then ifft
     *     2. perform block-wise transpose on IFFT outputs and store results in dl_iffted_data_buffer_
     *     2. add an event to the message queue to infrom main thread the completion of this task
     */
    void IFFT(int offset);

    
private:
    Config *config_;
    int BS_ANT_NUM, PILOT_NUM;
    int OFDM_CA_NUM;
    int OFDM_DATA_NUM;
    int OFDM_DATA_START;
    int OFDM_PREFIX_LEN;
    int subframe_num_perframe, data_subframe_num_perframe;
    int packet_length;
    int buffer_subframe_num_;

    int tid;

    Consumer &consumer_;

    Table<char> &socket_buffer_;
    Table<int> &socket_buffer_status_;
    Table<complex_float> &data_buffer_;
    Table<complex_float> &csi_buffer_;

    char *dl_socket_buffer_;;
    Table<complex_float> &dl_ifft_buffer_;


    // int packet_length;
    Table<double> *FFT_task_duration;
    Table<double> *CSI_task_duration;
    int *FFT_task_count;
    int *CSI_task_count;
    Table<double> *IFFT_task_duration;
    int *IFFT_task_count;

    FFTBuffer fft_buffer_;

    DFTI_DESCRIPTOR_HANDLE mkl_handle;
    MKL_LONG mkl_status;

    DFTI_DESCRIPTOR_HANDLE mkl_handle_dl;
    MKL_LONG mkl_status_dl;

    

    // mufft_plan_1d *muplans_;


};


#endif
