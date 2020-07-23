/**
 * Author: Kevin Boos
 * Email: kevinaboos@gmail.com
 *
 * @see DoSubcarrier
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
 * @brief A worker class that handles all forms of subcarrier-parallel processing tasks.
 * 
 * Currently, this worker class contains the following functionality:
 * @li DoZF
 * @li DoDemul
 * @li DoPrecode
 * @li Reciprocity (?? TBD)
 * 
 * ## General Usage ##
 * One instance of this class should handle the computation for one `block_size` group of subcarrier frequencies,
 * so we should spawn `num_events` instances of this class. 
 * For example, see `config->demul_events_per_symbol` and `config->demul_block_size`,
 * or the similar ones for zeroforcing (`zf_events_per_symbol` and `zf_block_size`).
 * 
 * It then executes the sequential processing stages that are required for each subcarrier block,
 * consisting of the following stages:
 * FIXME: ensure this is right
 * @li zeroforcing (`DoZF`)
 * @li demodulation (`DoDemul`) for uplink, or precoding (`DoPrecode`) for downlink.
 * 
 * 
 * FIXME: The biggest issue is how buffers are going to be allocated, shared, and accessed. 
 *        Currently, the rest of Millipede expects a single instance of all buffers,
 *        but with this redesign, we are allocating per-DoSubcarrier buffers. 
 *        While this probably works just fine for intermediate internal buffers,
 *        perhaps we should require (at least initially) that all input and output buffers
 *        are shared across all `DoSubcarrier` instances. 
 * 
 * 
 * ## Buffer ownership and management ##
 * The general buffer ownership policy is to accept *references* to input buffers, 
 * and to own both intermediate buffers for internal usage
 * as well as output buffers that are shared with others. 
 * This means that the constructor/destructor of this class is responsible
 * for allocating/deallocating the intermediate buffers and output buffers 
 * but not the input buffers.
 *
 * FIXME: Currently, however, some of the output buffers are still owned
 *        by the core `Millipede` class instance. We'll gradually move them into here.
 * 
 */
class DoSubcarrier: public Doer {
public:
    /**
     * @brief Construct a new Do Subcarrier object
     * 
     * In general, 
     * NOTE: The following buffers are owned by this class, but if we wanted to 
     *       share them differently, we could accept a reference to an externally-owned instance.
     *       - `demod_soft_buffer`
     *       - `dl_ifft_buffer` (TODO, currently still owned by `Millipede` class)
     * 
     */
    DoSubcarrier(
        Config* config,
        int tid,
        double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        // input buffers below
        Table<complex_float>& csi_buffer,
        Table<complex_float>& recip_buffer,
        Table<int8_t>&        dl_encoded_buffer,
        Table<complex_float>& data_buffer, 
        // output buffers below
        Table<complex_float>& dl_ifft_buffer,
        // Table<int8_t>& demod_soft_buffer,  
        PhyStats* phy_stats,
        Stats* stats
    ) : Doer(config, tid, freq_ghz, task_queue, complete_task_queue, worker_producer_token),
        csi_buffer_(csi_buffer),
        recip_buffer_(recip_buffer),
        data_buffer_(data_buffer),
        dl_ifft_buffer_(dl_ifft_buffer),
        dl_encoded_buffer_(dl_encoded_buffer)
    {
        const size_t task_buffer_symbol_num_ul = cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

        // Create the various buffers owned by this class. 
        /*
         * TODO: Currently, these buffers are created to be huge enough to store data 
         *       for *ALL* subcarrier ranges, system-wide. 
         *       We may want to scale them down to be sufficient for just a single subcarrier range;
         *       although, then we'd need to perform some kind of "gather" operation at the end
         *       before handing them off to the encoder stage.
         */

        demod_hard_buffer_   .malloc(task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        equal_buffer_        .malloc(task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        demod_soft_buffer_   .malloc(task_buffer_symbol_num_ul, cfg->mod_type * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        ue_spec_pilot_buffer_.calloc(TASK_BUFFER_FRAME_NUM, cfg->UL_PILOT_SYMS * cfg->UE_NUM, 64);
        ul_zf_buffer_        .malloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM, 64);
        dl_zf_buffer_        .calloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, cfg->UE_NUM * cfg->BS_ANT_NUM, 64);


        // Create the requisite Doers
        computeZF = new DoZF(this->cfg, tid, freq_ghz, 
            this->task_queue_, this->complete_task_queue, this->worker_producer_token, 
            csi_buffer_, recip_buffer_, ul_zf_buffer_, dl_zf_buffer_, stats
        );
        computeDemul = new DoDemul(this->cfg, tid, freq_ghz, 
            this->task_queue_, this->complete_task_queue, this->worker_producer_token, 
            data_buffer_, ul_zf_buffer_, ue_spec_pilot_buffer_, equal_buffer_,
            demod_hard_buffer_, demod_soft_buffer_, phy_stats, stats
        );
        computePrecode = new DoPrecode(this->cfg, tid, freq_ghz, 
            this->task_queue_, this->complete_task_queue, this->worker_producer_token, 
            dl_zf_buffer_, dl_ifft_buffer_, dl_encoded_buffer_, stats
        );
        // TODO: I believe we need the Reciprocity doer too, since it's used for calculating `dl_zf_buffer`
    }


    ~DoSubcarrier() {
        demod_hard_buffer_.free();
        equal_buffer_.free();
        demod_soft_buffer_.free();
        ue_spec_pilot_buffer_.free();
        ul_zf_buffer_.free();
        dl_zf_buffer_.free();

        delete computeZF;
        delete computeDemul;
        delete computePrecode;
        // TODO: delete recip;
    }


    /// TODO: here's where the real work happens
    Event_data launch(size_t tag) {
        /*
         * Here we need to directly invoke `compute[ZF|Demul|Precoder].launch()`
         * so we prevent those `Doer`s from enqueueing their individual events types 
         * onto the `complete_task_queue`. 
         * Instead, we'll enqueue a single `EventType::kSubcarrier` event once all stages are complete.
         */

        /*
         * Based on the given `tag`, we need to run the following stages:
         * TODO: if necessary, call computeZF.launch()
         * TODO: if necessary, call computeDemul.launch()
         * TODO: if necessary, call computePrecode.launch()
         */


        return Event_data(EventType::kSubcarrier, tag);
    }

    /// An accessor function to expose the internal `demod_hard_buffer_`
    /// for debugging/GUI purposes.
    Table<uint8_t>& get_demod_hard_buffer() {
        return demod_hard_buffer_;
    }

    /// An accessor function to expose the internal `equal_buffer_`
    /// for debugging/GUI purposes.
    Table<complex_float>& get_equal_buffer() {
        return equal_buffer_;
    }


private:
    /*
     * TODO: I'd like to use owned objects here instead of pointers,
     *       but I don't think we can create them in initializer lists because
     *       the other buffers (`Table`s) need to be allocated first via calls to `malloc`.
     *       In other news, C++ initializer lists are complete and utter garbage.
     */
    DoZF* computeZF;
    DoDemul* computeDemul;
    DoPrecode* computePrecode; 


    ///////////////////////////////////////////////////
    ////////////////// Input Buffers //////////////////
    ///////////////////////////////////////////////////

    /// An input buffer of channel state information (CSI), the output of the FFT stage.
    Table<complex_float>& csi_buffer_;

    /// TODO: I'm not 100% sure what this is used for. It's an input buffer into doZF...
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM
    /// @li 2nd dimension: number of OFDM data subcarriers * number of antennas
    Table<complex_float>& recip_buffer_;

    /// An input buffer of encoded data coming from the encoder (e.g., LDPC),
    /// which is used only during downlink. 
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<int8_t>& dl_encoded_buffer_;

    /// The main input buffer for subcarrier processing stages,
    /// which holds the data symbols after the FFT stage.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of antennas * number of OFDM data subcarriers
    ///
    /// The 2nd dimension's data order: 32 blocks each with 32 subcarriers each:
    /// subcarrier 1 -- 32 of antennas, subcarrier 33 -- 64 of antennas, ...,
    /// subcarrier 993 -- 1024 of antennas.
    Table<complex_float>& data_buffer_;


    ///////////////////////////////////////////////////
    ////////////////// Output Buffers /////////////////
    ///////////////////////////////////////////////////

    /// The main output buffer, which comes from the soft demodulation stage.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<int8_t> demod_soft_buffer_;

    /// An output buffer (TODO: currently a reference) holding data destined for IFFT,
    /// only useful for downlink.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of antennas * number of data symbols per frame
    /// @li 2nd dimension: number of OFDM carriers (including non-data carriers)
    Table<complex_float> dl_ifft_buffer_;


    ///////////////////////////////////////////////////
    ///////// Internal / Intermediate Buffers /////////
    ///////////////////////////////////////////////////

    /// An internal buffer
    /// TODO: I'm fairly certain this is used only in DoDemul, so we can move it directly into that class.
    Table<complex_float> ue_spec_pilot_buffer_;

    /// Data after equalization
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    /// TODO: I'm fairly certain this is used only in DoDemul, so we can move it directly into that class.
    ///       The only concern with that is that it's accessed externally by the Python GUI, so we'd need to export it.
    Table<complex_float> equal_buffer_;

    /// Calculated uplink zeroforcing detection matrices.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of OFDM data subcarriers
    /// @li 2nd dimension: number of antennas * number of UEs
    ///
    /// This is an internal buffer that is written to by DoZF as output
    /// and then fed into `DoDemul` as input.
    Table<complex_float> ul_zf_buffer_;

    /// Calculated zeroforcing precoders for downlink beamforming.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * number of OFDM data subcarriers
    /// @li 2nd dimension: number of antennas * number of UEs
    ///
    /// This is an internal buffer that is written to by DoZF as output
    /// and then fed into `DoPrecode` as input.
    Table<complex_float> dl_zf_buffer_;

    /// The internal buffer for data after hard demodulation.
    /// @li 1st dimension: TASK_BUFFER_FRAME_NUM * uplink data symbols per frame
    /// @li 2nd dimension: number of OFDM data subcarriers * number of UEs
    Table<uint8_t> demod_hard_buffer_;

};

#endif // DOSUBCARRIER_HPP
