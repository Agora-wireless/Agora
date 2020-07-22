/**
 * Author: Kevin Boos
 * Email: kevinaboos@gmail.com
 *
 * A worker class that handles all forms of subcarrier-parallel processing tasks.
 * Currently it comprises DoZF, DoDemul, and DoPrecode functionality.
 */
#ifndef DOSUBCARRIER_HPP
#define DOSUBCARRIER_HPP

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "modulation.hpp"
#include "stats.hpp"
#include "phy_stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h> /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>
// #include "mkl_dfti.h"

using namespace arma;

/**
 * @brief 
 * 
 * Ownership of buffers is as follows:
 * @li Accepts as input a reference to an externally-owned data buffer.
 * @li Owns the output buffer, i.e., demod_soft_buffer
 * 
 */
class DoSubcarrier {
public:
    DoSubcarrier(
        Config* config,
        int tid,
        double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        Table<complex_float>& data_buffer, 
        // NOTE: The below `demod_soft_buffer` is currently a table owned by this class,
        //       but if we wanted to share it differently we could accept a reference to an externally-owned instance.
        // Table<int8_t>& demod_soft_buffer,  
        PhyStats* in_phy_stats,
        Stats* in_stats_manager
    ) : // TODO: finish up initializing the other cfg and event queue stuff
        DoZf(...),
        DoDemul(...),
        DoPrecode(...)
    {
        auto& cfg = config_;
        const size_t task_buffer_symbol_num_ul = cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

        // Create the various buffers owned by this class. 
        demod_hard_buffer_.malloc(task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        equal_buffer_.malloc(task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        size_t mod_type = config_->mod_type;
        demod_soft_buffer_.malloc(task_buffer_symbol_num_ul, mod_type * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        ue_spec_pilot_buffer_.calloc(TASK_BUFFER_FRAME_NUM, cfg->UL_PILOT_SYMS * cfg->UE_NUM, 64);
        ul_zf_buffer_.malloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM, 64);
    }


    ~DoSubcarrier() {
        demod_hard_buffer_.free();
        equal_buffer_.free();
        demod_soft_buffer_.free();
        ue_spec_pilot_buffer.free();
        ul_zf_buffer_.free();
    }


private:
    DoZF computeZF;
    DoDemul computeDemul;
    DoPrecode computePrecode; 

    /// The sole input buffer for subcarrier processing stages,
    /// which holds the data symbols after the FFT stage.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of antennas * number of OFDM data subcarriers
    ///
    /// The 2nd dimension's data order: 32 blocks each with 32 subcarriers each:
    /// subcarrier 1 -- 32 of antennas, subcarrier 33 -- 64 of antennas, ...,
    /// subcarrier 993 -- 1024 of antennas.
    Table<complex_float>& data_buffer_;

    /// The output buffer from after the soft demodulation stage.
    Table<int8_t> demod_soft_buffer_;

    /// An internal buffer
    /// TODO: I'm fairly certain this is used only in DoDemul, so we can move it directly into that class.
    Table<complex_float> ue_spec_pilot_buffer_;

    /// Data after equalization
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    /// TODO: I'm fairly certain this is used only in DoDemul, so we can move it directly into that class.
    Table<complex_float> equal_buffer_;

    /// Calculated uplink zeroforcing detection matrices.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of OFDM data subcarriers
    /// @li 2nd dimension: number of antennas * number of UEs
    ///
    /// This is an internal buffer.
    /// This is written to by DoZF as its output and then fed into DoPrecode as input.
    /// In DoPrecode, it is currently `(in)_precoder_buffer`.
    ///
    /// TODO: I'm not 100% confident that this belongs here. I think it's intended to be shared by DoDemul and DoZF,
    ///       but DoZF currently appears to hold its own separate instance of this buffer. TBD...
    Table<complex_float> ul_zf_buffer_;

    /// The internal buffer for data after hard demodulation.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<uint8_t> demod_hard_buffer_;

};

#endif // DOSUBCARRIER_HPP
