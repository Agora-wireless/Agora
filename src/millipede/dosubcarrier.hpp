/// Author: Kevin Boos
/// Email: kevinaboos@gmail.com
///
/// @see DoSubcarrier

#ifndef DOSUBCARRIER_HPP
#define DOSUBCARRIER_HPP

// [Kevin]: unsure if all of these includes are needed.
#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "doprecode.hpp"
#include "dodemul.hpp"
#include "dozf.hpp"
#include "reciprocity.hpp"

#include "gettime.h"
#include "modulation.hpp"
#include "stats.hpp"
#include "phy_stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

using namespace arma;

/// @brief A worker class that handles all forms of subcarrier-parallel processing tasks.
/// 
/// Currently, this worker class contains the following functionality:
/// @li DoZF
/// @li DoDemul
/// @li DoPrecode
/// @li Reciprocity (?? TBD)
/// 
/// ## General Usage ##
/// One instance of this class should handle the computation for one `block_size` range of subcarrier frequencies,
/// so we should spawn `num_events` instances of this class. 
/// For example, see `config->demul_events_per_symbol` and `config->demul_block_size`,
/// or the similar ones for zeroforcing (`zf_events_per_symbol` and `zf_block_size`).
/// 
/// It then executes the sequential processing stages that are required for each subcarrier block,
/// consisting of the following stages:
/// FIXME: ensure this is right
/// @li zeroforcing (`DoZF`)
/// @li demodulation (`DoDemul`) for uplink, or precoding (`DoPrecode`) for downlink.
/// 
/// 
/// FIXME: The biggest issue is how buffers are going to be allocated, shared, and accessed. 
///        Currently, the rest of Millipede expects a single instance of all buffers,
///        but with this redesign, we are allocating per-DoSubcarrier buffers. 
///        While this probably works just fine for intermediate internal buffers,
///        perhaps we should require (at least initially) that all input and output buffers
///        are shared across all `DoSubcarrier` instances. 
/// 
/// 
/// ## Buffer ownership and management ##
/// The general buffer ownership policy is to accept *references* to input buffers, 
/// and to own both intermediate buffers for internal usage
/// as well as output buffers that are shared with others. 
/// This means that the constructor/destructor of this class is responsible
/// for allocating/deallocating the intermediate buffers and output buffers 
/// but not the input buffers.
///
/// FIXME: Currently, the output buffers are still owned by the core `Millipede` instance.
///        We eventually should move them into here.
///
class DoSubcarrier: public Doer {
public:
    /// @brief Construct a new Do Subcarrier object
    /// 
    /// In general, 
    /// NOTE: The following buffers are owned by this class, but if we wanted to 
    ///       share them differently, we could accept a reference to an externally-owned instance.
    ///       - `demod_soft_buffer`
    ///       - `dl_ifft_buffer` (TODO, currently still owned by `Millipede` class)
    ///
    DoSubcarrier(
        Config* config,
        int tid,
        double freq_ghz,
        moodycamel::ConcurrentQueue<Event_data>& task_queue,
        moodycamel::ConcurrentQueue<Event_data>& complete_task_queue,
        moodycamel::ProducerToken* worker_producer_token,
        /// The range of subcarriers handled by this subcarrier doer.
        struct Range subcarrier_range,
        // input buffers
        Table<complex_float>& csi_buffer,
        Table<complex_float>& recip_buffer,
        Table<complex_float>& calib_buffer,
        Table<int8_t>&        dl_encoded_buffer,
        Table<complex_float>& data_buffer, 
        // output buffers
        Table<int8_t>&        demod_soft_buffer,  
        Table<complex_float>& dl_ifft_buffer,
        // intermediate buffers owned by SubcarrierManager
        Table<complex_float>& ue_spec_pilot_buffer,
        Table<complex_float>& equal_buffer,
        Table<complex_float>& ul_zf_buffer,
        Table<complex_float>& dl_zf_buffer,
        PhyStats* phy_stats,
        Stats* stats
    )   : Doer(config, tid, freq_ghz, task_queue,
            complete_task_queue, worker_producer_token)
        , subcarrier_range_(subcarrier_range)
        , csi_buffer_(csi_buffer)
        , recip_buffer_(recip_buffer)
        , calib_buffer_(calib_buffer)
        , dl_encoded_buffer_(dl_encoded_buffer)
        , data_buffer_(data_buffer)
        , demod_soft_buffer_(demod_soft_buffer)
        , dl_ifft_buffer_(dl_ifft_buffer)
        , ue_spec_pilot_buffer_(ue_spec_pilot_buffer)
        , equal_buffer_(equal_buffer)
        , ul_zf_buffer_(ul_zf_buffer)
        , dl_zf_buffer_(dl_zf_buffer)
    {

        // Create the requisite Doers
        computeZF_ = new DoZF(this->cfg, tid, freq_ghz, 
            this->task_queue_, this->complete_task_queue, this->worker_producer_token, 
            csi_buffer_, recip_buffer_, ul_zf_buffer_, dl_zf_buffer_, stats
        );
        computeDemul_ = new DoDemul(this->cfg, tid, freq_ghz, 
            this->task_queue_, this->complete_task_queue, this->worker_producer_token, 
            data_buffer_, ul_zf_buffer_, ue_spec_pilot_buffer_, equal_buffer_,
            demod_soft_buffer_, phy_stats, stats
        );
        computePrecode_ = new DoPrecode(this->cfg, tid, freq_ghz, 
            this->task_queue_, this->complete_task_queue, this->worker_producer_token, 
            dl_zf_buffer_, dl_ifft_buffer_, dl_encoded_buffer_, stats
        );
        // computeReciprocity_ = new Reciprocity(this->cfg, tid, freq_ghz,
        //     this->task_queue_, this->complete_task_queue, this->worker_producer_token,
        //     calib_buffer_, recip_buffer_, stats);
    }


    ~DoSubcarrier() {
        delete computeZF_;
        delete computeDemul_;
        delete computePrecode_;
        // delete computeReciprocity_;
    }


    Event_data launch(size_t tag, EventType event_type) {
        switch (event_type) {
        case EventType::kZF: {
            return computeZF_->launch(tag, event_type);
        } break;
        case EventType::kDemul: {
            return computeDemul_->launch(tag, event_type);
        } break;
        case EventType::kPrecode: {
            return computePrecode_->launch(tag, event_type);
        } break;
        /// TODO: when reciprocity is moved into Subcarrier doers, add this back in.
        // case EventType::kRc: {
        //     return computeReciprocity_->launch(tag, event_type);
        // } break;

        // all other cases are handled below
        default: break;
        }

        std::cerr << "[DoSubcarrier::launch] error, unexpected event type " << (int)event_type << std::endl;
        rt_assert(false, "[DoSubcarrier::launch] error, unexpected event type.");

        return Event_data();
    }


private:

    /// The subcarrier range handled by this subcarrier doer.
    struct Range subcarrier_range_;

    // TODO: I'd like to use owned objects here instead of pointers,
    //       but I don't think we can create them in initializer lists because
    //       the other buffers (`Table`s) need to be allocated first via calls to `malloc`.
    //       In other news, C++ initializer lists are complete and utter garbage.

    DoZF*         computeZF_;
    DoDemul*      computeDemul_;
    DoPrecode*    computePrecode_;
    // Reciprocity*  computeReciprocity_;

    // For the following buffers, see the `SubcarrierManager`'s documentation.

    ///////////////////////////////////////////////////
    ////////////////// Input Buffers //////////////////
    ///////////////////////////////////////////////////

    Table<complex_float>& csi_buffer_;
    Table<complex_float>& recip_buffer_;
    Table<complex_float>& calib_buffer_;
    Table<int8_t>&        dl_encoded_buffer_;
    Table<complex_float>& data_buffer_;


    ///////////////////////////////////////////////////
    ////////////////// Output Buffers /////////////////
    ///////////////////////////////////////////////////
    
    Table<int8_t>&        demod_soft_buffer_;
    Table<complex_float>& dl_ifft_buffer_;


    ///////////////////////////////////////////////////
    ///////// Internal / Intermediate Buffers /////////
    ///////////////////////////////////////////////////
    
    Table<complex_float>& ue_spec_pilot_buffer_;
    Table<complex_float>& equal_buffer_;
    Table<complex_float>& ul_zf_buffer_;
    Table<complex_float>& dl_zf_buffer_;

};

#endif // DOSUBCARRIER_HPP
