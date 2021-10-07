#ifndef DOFFT
#define DOFFT

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "mkl_dfti.h"
#include "phy_stats.hpp"
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

class DoFFT : public Doer {
public:
    DoFFT(Config* config, int tid, double freq_ghz, Range ant_range,
        Table<char>& time_domain_iq_buffer, 
        Table<complex_float>& frequency_domain_iq_buffer_to_send,
        PhyStats* in_phy_stats,
        Stats* stats_manager, SharedState* shared_state_);
    ~DoFFT();

    /**
     * Do FFT task for one OFDM symbol
     *
     * @param tag is an event data tag of type fft_req_tag_t
     *
     * Buffers: socket_buffer_, fft_buffer_, csi_buffer_, data_buffer_
     *     Input buffer: socket_buffer_
     *     Output buffer: csi_buffer_ if symbol is pilot
     *                    data_buffer_ if symbol is data
     *     Intermediate buffer: fft_buffer_ (FFT_inputs, FFT_outputs)
     * Offsets:
     *     socket_buffer_:
     *         dim1: socket thread index: (offset / # of OFDM symbols per
     * thread) dim2: OFDM symbol index in this socket thread (offset - # of
     * symbols in previous threads) FFT_inputs, FFT_outputs: dim1: frame index
     * * # of OFDM symbols per frame + symbol index * # of atennas + antenna
     * index dim2: subcarrier index csi_buffer_: dim1: frame index * FFT size +
     * subcarrier index in the current frame dim2: user index * # of antennas +
     * antenna index data_buffer_: dim1: frame index * # of data symbols per
     * frame + data symbol index dim2: transpose block index * block size * #
     * of antennas + antenna index * block size Event offset: frame index * # of
     * symbol per frame + symbol index Description:
     *     1. copy received data (one OFDM symbol) from socket_buffer to
     * fft_buffer_.FFT_inputs (remove CP)
     *     2. perform FFT on fft_buffer_.FFT_inputs and store results in
     * fft_buffer_.FFT_outputs
     *     3. if symbol is pilot, do channel estimation from
     * fft_buffer_.FFT_outputs to csi_buffer_ if symbol is data, copy data
     * from fft_buffer_.FFT_outputs to data_buffer_ and do block transpose
     *     4. add an event to the message queue to infrom main thread the
     * completion of this task
     */
    void launch(size_t frame_id, size_t symbol_id, size_t ant_id);

    void start_work();

private:
    Table<char>& time_domain_iq_buffer_;
    Table<char>& frequency_domain_iq_buffer_to_send_;
    DFTI_DESCRIPTOR_HANDLE mkl_handle;
    complex_float* fft_inout; // Buffer for both FFT input and output
    DurationStat* duration_stat_fft;
    DurationStat* duration_stat_csi;
    PhyStats* phy_stats;

    struct Range ant_range_;
    moodycamel::ConcurrentQueue<Event_data> dummy_conq_;

    size_t cur_frame_ = 0;
    size_t cur_idx_ = 0;
    
    SharedState *shared_state_;
};

class DoIFFT : public Doer {
public:
    DoIFFT(Config* in_config, int in_tid, double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& in_task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<complex_float>& in_dl_ifft_buffer, char* in_dl_socket_buffer,
        Stats* in_stats_manager);
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
     *         dim1: data symbol index in the current frame * # of users +
     * user index dim2: subcarrier index dl_ifft_buffer_: dim1: frame index * #
     * of data symbols per frame * # of users + data symbol index * # of
     * users + user index dim2: subcarrier index dl_iffted_data_buffer_: dim1:
     * frame index * # of data symbols per frame + data symbol index dim2:
     * transpose block index * block size * # of UEs + user index * block size
     * Event offset: offset
     * Description:
     *     1. for each OFDM symbol, perform modulation and then ifft
     *     2. perform block-wise transpose on IFFT outputs and store results in
     * dl_iffted_data_buffer_
     *     2. add an event to the message queue to infrom main thread the
     * completion of this task
     */
    Event_data launch(size_t tag);

private:
    Table<complex_float>& dl_ifft_buffer_;
    char* dl_socket_buffer_;
    DurationStat* duration_stat;
    DFTI_DESCRIPTOR_HANDLE mkl_handle;
};

#endif
