/**
 * Author: Kevin Boos
 * Email: kevinaboos@gmail.com
 *
 * @see SubcarrierManager
 */
#ifndef SUBCARRIER_MANAGER_HPP
#define SUBCARRIER_MANAGER_HPP

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "modulation.hpp"
#include "stats.hpp"
#include "phy_stats.hpp"
// #include <armadillo>
#include <iostream>
#include <stdio.h> /* for fprintf */
#include <string.h> /* for memcpy */
#include <vector>
// #include "mkl_dfti.h"

// using namespace arma;

/**
 * @brief The singleton manager of all `DoSubcarrier` instances.
 * 
 * This class is essentially a wrapper around the many `DoSubcarrier` instances,
 * aka subcarrier workers.
 * It encapsulates (and owns) the buffers shared by those `DoSubcarrier` instances
 * and exposes interfaces to communicate with them, e.g., by using event queues. 
 * 
 * 
 * There is at least one `DoSubcarrier` instance per subcarrier range,
 * typically just one but potentially more redundant replicates for 
 * purposes of scalability and reliability.
 * This class is responsible for owning the internal buffers that are used
 * by the subcarrier workers, 
 * and serves as a wrapper interface to export subcarrier-parallel functionality. 
 * 
 */
class SubcarrierManager {
public:
    
    SubcarrierManager(
        Config* config,
        int tid,
        double freq_ghz,
        // moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        // moodycamel::ProducerToken* worker_producer_token,
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
    ) : cfg(config), 
        freq_ghz_(freq_ghz),
        complete_task_queue(complete_task_queue), 
        csi_buffer_(csi_buffer),
        recip_buffer_(recip_buffer),
        data_buffer_(data_buffer),
        dl_ifft_buffer_(dl_ifft_buffer),
        dl_encoded_buffer_(dl_encoded_buffer)
    {
        const size_t task_buffer_symbol_num_ul = cfg->ul_data_symbol_num_perframe * TASK_BUFFER_FRAME_NUM;

        /*
         * Create the various buffers owned by the subcarrier manager. 
         * These buffers are shared across all subcarrier doers.
         * 
         * TODO: Currently, these buffers are created to be huge enough to store data 
         *       for *ALL* subcarrier ranges, system-wide. 
         *       We may want to scale them down to be sufficient for just a single subcarrier range;
         *       although, then we'd need to perform some kind of "gather" operation at the end
         *       before handing them off to the encoder stage.
         */

        equal_buffer_        .malloc(task_buffer_symbol_num_ul, cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        demod_soft_buffer_   .malloc(task_buffer_symbol_num_ul, cfg->mod_type * cfg->OFDM_DATA_NUM * cfg->UE_NUM, 64);
        ue_spec_pilot_buffer_.calloc(TASK_BUFFER_FRAME_NUM, cfg->UL_PILOT_SYMS * cfg->UE_NUM, 64);
        ul_zf_buffer_        .malloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, cfg->BS_ANT_NUM * cfg->UE_NUM, 64);
        dl_zf_buffer_        .calloc(cfg->OFDM_DATA_NUM * TASK_BUFFER_FRAME_NUM, cfg->UE_NUM * cfg->BS_ANT_NUM, 64);
    }


    ~DoSubcarrier() {
        // First, free all of the buffers we own.
        equal_buffer_.free();
        demod_soft_buffer_.free();
        ue_spec_pilot_buffer_.free();
        ul_zf_buffer_.free();
        dl_zf_buffer_.free();

        // Destroy all of the Subcarrier doers
        for (DoSubcarrier*& dsc : subcarrier_doers_) {
            delete dsc;
        }
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


    /// An accessor function to expose the internal `equal_buffer_`
    /// for debugging/GUI purposes.
    Table<complex_float>& get_equal_buffer() {
        return equal_buffer_;
    }


private:
    
    /// The list of subcarrier doers, of which there should be
    /// `demul_events_per_symbol` elements in total. 
    /// FIXME: ensure this is correct
    ///
    /// TODO: we could also include the subcarrier range in this vector,
    ///       e.g., using a std::pair -- `vector<pair<int, DoSubcarrier*>>`
    std::vector<DoSubcarrier*> subcarrier_doers_;

  

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


    ///////////////////////////////////////////////////
    ///////// Basic data items from Millipede /////////
    ///////////////////////////////////////////////////

    /// Millipede-wide configuration values.
    Config* cfg;
    /// RDTSC frequency in GHz.
    double freq_ghz_;

    /// TODO: there should be one task_queue per SubcarrierDoer.
    /// Millipede pushes events onto this queue to indicate
    /// to worker threads that various tasks are available to work on.
    // moodycamel::ConcurrentQueue<Event_data>& task_queue_;

    /// The singleton task completion queue. 
    /// Push events onto this queue to notify Millipede's master thread
    /// that a given task has been completed. 
    moodycamel::ConcurrentQueue<Event_data>& complete_task_queue;

};

#endif // SUBCARRIER_MANAGER_HPP
