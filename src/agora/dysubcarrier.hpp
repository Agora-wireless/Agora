#pragma once

#include "Symbols.hpp"
#include "buffer.hpp"
#include "comms-lib.h"
#include "concurrentqueue.h"
#include "config.hpp"
#include "datatype_conversion.h"
#include "dydemul.hpp"
#include "doer.hpp"
#include "doprecode.hpp"
#include "dyzf.hpp"
#include "gettime.h"
#include "modulation.hpp"
#include "phy_stats.hpp"
#include "shared_counters.hpp"
#include "signalHandler.hpp"
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

using namespace arma;

/**
 * @brief A worker class that handles all subcarrier-parallel processing tasks.
 *
 * Currently, this worker class contains the following functionality:
 * @li DoZF
 * @li DoDemul
 * @li DoPrecode
 * @li Reciprocity (?? TBD)
 *
 * ## General usage ##
 * One instance of this class should handle the computation for one `block_size`
 * range of subcarrier frequencies, so we should spawn `num_events` instances of
 * this class.
 * For example, see `Config::demul_events_per_symbol and
 * `Config::demul_block_size`, or the similar ones for zeroforcing,
 * `zf_events_per_symbol` and `zf_block_size`.
 *
 * Upon receiving an event, it executes the specific doer for that event type,
 * consisting of one of the following:
 * @li zeroforcing (`DoZF`)
 * @li demodulation (`DoDemul`) for uplink
 * @li precoding (`DoPrecode`) for downlink.
 *
 * FIXME: The biggest issue is how buffers are going to be allocated, shared,
 * and accessed. Currently, the rest of Agora expects single instance of all
 * buffers, but with this redesign, we are allocating per-DySubcarrier buffers.
 * While this probably is okay for intermediate internal buffers, perhaps we
 * should require (at least initially) that all input and output buffers are
 * shared across all `DySubcarrier` instances.
 *
 * ## Buffer ownership and management ##
 * The general buffer ownership policy is to accept *references* to input
 * buffers, and to own both intermediate buffers for internal usage as well as
 * output buffers that are shared with others.
 * This means that the constructor/destructor of this class is responsible for
 * allocating/deallocating the intermediate buffers and output buffers but not
 * the input buffers.
 * 
 * FIXME: Currently, the output buffers are still owned by the core `Agora`
 * instance. We should eventually move them into here.
 */
class DySubcarrier : public Doer {
public:
    /// Construct a new Do Subcarrier object
    DySubcarrier(Config* config, int tid, double freq_ghz,
        /// The range of subcarriers handled by this subcarrier doer.
        Range sc_range,
        // input buffers
        Table<char>& freq_domain_iq_buffer,
        PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffer,
        Table<complex_float>& calib_buffer, Table<int8_t>& dl_encoded_buffer,
        // output buffers
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_to_send,
        Table<complex_float>& dl_ifft_buffer,
        // intermediate buffers owned by SubcarrierManager
        Table<complex_float>& equal_buffer,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
        PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
        std::vector<std::vector<ControlInfo>>& control_info_table,
        std::vector<size_t>& control_idx_list,
        SharedState* shared_state);

    ~DySubcarrier();

    void StartWork();
    void StartWorkCentral();


    void runCsi(size_t frame_id, size_t base_sc_id);

    inline bool shouldSleep(std::vector<ControlInfo>& control_list);

    DyZF* do_zf_;
    DyDemul* do_demul_;
    DoPrecode* do_precode_;

    // Input buffers
private:
    /// The subcarrier range handled by this subcarrier doer.
    struct Range sc_range_;

    Table<char>& freq_domain_iq_buffer_;

    PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffer_;
    Table<complex_float>& calib_buffer_;
    Table<int8_t>& dl_encoded_buffer_;

    // Output buffers

    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffer_to_send_;
    Table<complex_float>& dl_ifft_buffer_;

    // Intermediate buffers

    Table<complex_float>& equal_buffer_;
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_;
    PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_;

    // Shared states with TXRX threads
    SharedState* shared_state_;

    // Internal CSI states
    size_t csi_cur_frame_ = 0;

    // Internal ZF states
    size_t zf_cur_frame_ = 0; // Current frame waiting for CSI matrix
    size_t n_zf_tasks_done_ = 0;

    // Internal Demul states
    size_t demul_cur_frame_; // Current frame waiting for ZF matrix
    size_t demul_cur_sym_ul_ = 0; // Current data symbol wait to process
    size_t n_demul_tasks_done_ = 0;

    // Internal Precode states
    size_t precode_cur_frame_ = 0;
    size_t precode_cur_sym_dl_ = 0;
    size_t n_precode_tasks_done_ = 0;

    // Control info
    std::vector<std::vector<ControlInfo>>& control_info_table_;
    std::vector<size_t>& control_idx_list_;
};
