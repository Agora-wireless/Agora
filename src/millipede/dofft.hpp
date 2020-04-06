/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef DOFFT
#define DOFFT

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "fft.h"
#include "gettime.h"
#include "mkl_dfti.h"
#include "stats.hpp"
#include "utils.h"
#include <iostream>
#include <stdio.h> /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>

class DoFFT : public Doer {
public:
    DoFFT(Config* in_config, int in_tid,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        Consumer& in_consumer, Table<char>& in_socket_buffer,
        Table<int>& in_socket_buffer_status,
        Table<complex_short>& in_data_buffer,
        Table<complex_short>& in_csi_buffer,
        Table<complex_short>& in_calib_buffer, Stats* in_stats_manager);
    ~DoFFT();

    /**
     * Do FFT task for one OFDM symbol
     *
     * @param tag is an event data tag of type fft_req_tag_t
     *
     * Buffers: socket_buffer_, fft_buffer_, csi_buffer_, data_buffer_
     *     Input buffer: socket_buffer_
     *     Output buffer: csi_buffer_ if subframe is pilot
     *                    data_buffer_ if subframe is data
     *     Intermediate buffer: fft_buffer_ (FFT_inputs, FFT_outputs)
     * Offsets:
     *     socket_buffer_:
     *         dim1: socket thread index: (offset / # of OFDM symbols per
     * thread) dim2: OFDM symbol index in this socket thread (offset - # of
     * subframes in previous threads) FFT_inputs, FFT_outputs: dim1: frame index
     * * # of OFDM symbols per frame + subframe index * # of atennas + antenna
     * index dim2: subcarrier index csi_buffer_: dim1: frame index * FFT size +
     * subcarrier index in the current frame dim2: user index * # of antennas +
     * antenna index data_buffer_: dim1: frame index * # of data subframes per
     * frame + data subframe index dim2: transpose block index * block size * #
     * of antennas + antenna index * block size Event offset: frame index * # of
     * subframe per frame + subframe index Description:
     *     1. copy received data (one OFDM symbol) from socket_buffer to
     * fft_buffer_.FFT_inputs (remove CP)
     *     2. perform FFT on fft_buffer_.FFT_inputs and store results in
     * fft_buffer_.FFT_outputs
     *     3. if subframe is pilot, do channel estimation from
     * fft_buffer_.FFT_outputs to csi_buffer_ if subframe is data, copy data
     * from fft_buffer_.FFT_outputs to data_buffer_ and do block transpose
     *     4. add an event to the message queue to infrom main thread the
     * completion of this task
     */
    Event_data launch(int tag);

    void simd_store_to_buf(
        float* fft_buf, float*& out_buf, size_t ant_id, SymbolType symbol_type);
    void simd_store_to_buf_short(
        short* fft_buf, short*& out_buf, size_t ant_id, SymbolType symbol_type);

private:
    Table<char>& socket_buffer_;
    Table<int>& socket_buffer_status_;
    Table<complex_short>& data_buffer_;
    Table<complex_short>& csi_buffer_;
    Table<complex_short>& calib_buffer_;
    Table<double>* FFT_task_duration;
    int* FFT_task_count;
    Table<double>* CSI_task_duration;
    int* CSI_task_count;
    DFTI_DESCRIPTOR_HANDLE mkl_handle;
    FFTBuffer fft_buffer_;
    short* fft_buf_temp;
};

class DoIFFT : public Doer {
public:
    DoIFFT(Config* in_config, int in_tid,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        Consumer& in_consumer, Table<complex_float>& in_dl_ifft_buffer,
        char* in_dl_socket_buffer, Stats* in_stats_manager);
    ~DoIFFT();

    /**
     * Do modulation and ifft tasks for one OFDM symbol
     * @param tid: task thread index, used for selecting task ptok
     * Buffers: dl_IQ_data_long
     *     Input buffer: dl_IQ_data_long
     *     Output buffer: dl_iffted_data_buffer_
     *     Intermediate buffer: dl_ifft_buffer_
     * Offsets:
     *     dl_IQ_data_long_:
     *         dim1: data subframe index in the current frame * # of users +
     * user index dim2: subcarrier index dl_ifft_buffer_: dim1: frame index * #
     * of data subframes per frame * # of users + data subframe index * # of
     * users + user index dim2: subcarrier index dl_iffted_data_buffer_: dim1:
     * frame index * # of data subframes per frame + data subframe index dim2:
     * transpose block index * block size * # of UEs + user index * block size
     * Event offset: offset
     * Description:
     *     1. for each OFDM symbol, perform modulation and then ifft
     *     2. perform block-wise transpose on IFFT outputs and store results in
     * dl_iffted_data_buffer_
     *     2. add an event to the message queue to infrom main thread the
     * completion of this task
     */
    Event_data launch(int offset);

private:
    Table<complex_float>& dl_ifft_buffer_;
    char* dl_socket_buffer_;
    Table<double>* task_duration;
    int* task_count;
    DFTI_DESCRIPTOR_HANDLE mkl_handle;
};

#endif
