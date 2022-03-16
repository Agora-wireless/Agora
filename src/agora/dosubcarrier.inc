/**
 * @file dosubcarrier.inc
 * @brief Self-Contained Implmentation file implmenting a worker class that
 * handles all subcarrier-parallel processing tasks.
 */
#ifndef DOSUBCARRIER_H_
#define DOSUBCARRIER_H_

#include <armadillo>
#include <iostream>
#include <vector>

#include "buffer.h"
#include "comms-lib.h"
#include "concurrentqueue.h"
#include "config.h"
#include "datatype_conversion.h"
#include "dodemul.h"
#include "doer.h"
#include "doprecode.h"
#include "dozf.h"
#include "gettime.h"
#include "modulation.h"
#include "phy_stats.h"
#include "shared_counters.hpp"
#include "signal_handler.h"
#include "stats.h"
#include "symbols.h"

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
 * buffers, but with this redesign, we are allocating per-DoSubcarrier buffers.
 * While this probably is okay for intermediate internal buffers, perhaps we
 * should require (at least initially) that all input and output buffers are
 * shared across all `DoSubcarrier` instances.
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
class DoSubcarrier : public Doer {
 public:
  /// Construct a new Do Subcarrier object
  DoSubcarrier(Config* config, int tid, double freq_ghz,
               moodycamel::ConcurrentQueue<Event_data>&,
               moodycamel::ConcurrentQueue<Event_data>&,
               moodycamel::ProducerToken*,
               /// The range of subcarriers handled by this subcarrier doer.
               struct Range subcarrier_range,
               // input buffers
               Table<char>& socket_buffer, Table<int>& socket_buffer_status,
               PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
               Table<complex_float>& calib_buffer,
               Table<int8_t>& dl_encoded_buffer,
               Table<complex_float>& data_buffer,
               // output buffers
               PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
               Table<complex_float>& dl_ifft_buffer,
               // intermediate buffers owned by SubcarrierManager
               Table<complex_float>& ue_spec_pilot_buffer,
               Table<complex_float>& equal_buffer,
               PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices,
               PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices,
               PhyStats* phy_stats, Stats* stats, RxStatus* rx_status = nullptr,
               DemulStatus* demul_status = nullptr)
      : Doer(config, tid, freq_ghz, dummy_conq_, dummy_conq_,
             nullptr /* tok */),
        sc_range_(subcarrier_range),
        socket_buffer_(socket_buffer),
        socket_buffer_status_(socket_buffer_status),
        csi_buffers_(csi_buffers),
        calib_buffer_(calib_buffer),
        dl_encoded_buffer_(dl_encoded_buffer),
        data_buffer_(data_buffer),
        demod_buffers_(demod_buffers),
        dl_ifft_buffer_(dl_ifft_buffer),
        ue_spec_pilot_buffer_(ue_spec_pilot_buffer),
        equal_buffer_(equal_buffer),
        ul_zf_matrices_(ul_zf_matrices),
        dl_zf_matrices_(dl_zf_matrices),
        rx_status_(rx_status),
        demul_status_(demul_status) {
    // Create the requisite Doers
    do_zf_ = new DoZF(this->cfg, tid, freq_ghz, dummy_conq_, dummy_conq_,
                      nullptr /* ptok */, csi_buffers_, calib_buffer,
                      ul_zf_matrices_, dl_zf_matrices_, stats);

    do_demul_ = new DoDemul(this->cfg, tid, freq_ghz, dummy_conq_, dummy_conq_,
                            nullptr /* ptok */, data_buffer_, ul_zf_matrices_,
                            ue_spec_pilot_buffer_, equal_buffer_,
                            demod_buffers_, phy_stats, stats, &socket_buffer_);

    do_precode_ = new DoPrecode(
        this->cfg, tid, freq_ghz, dummy_conq_, dummy_conq_, nullptr /* ptok */,
        dl_zf_matrices_, dl_ifft_buffer_, dl_encoded_buffer_, stats);

    // Init internal states
    demul_cur_sym_ = cfg->Frame().NumPilotSyms();
  }

  ~DoSubcarrier() {
    delete do_zf_;
    delete do_demul_;
    delete do_precode_;
    // delete computeReciprocity_;
  }

  // Returns the range of subcarrier IDs handled by this subcarrier doer.
  Range& subcarrier_range() { return sc_range_; }

  void start_work() {
    const size_t n_zf_tasks_reqd =
        (sc_range_.end - sc_range_.start) / cfg->ZfBlockSize();
    const size_t n_demul_tasks_reqd =
        (sc_range_.end - sc_range_.start) / cfg->DemulBlockSize();

    while (cfg->running && !SignalHandler::gotExitSignal()) {
      if (rx_status_->received_all_pilots(csi_cur_frame_)) {
        run_csi(csi_cur_frame_, sc_range_.start);
        std::printf(
            "Main thread: pilot frame: %lu, finished CSI for all pilot "
            "symbols\n",
            csi_cur_frame_);
        csi_cur_frame_++;
      }

      if (csi_cur_frame_ > zf_cur_frame_) {
        do_zf_->Launch(
            gen_tag_t::frm_sym_sc(
                zf_cur_frame_, 0,
                sc_range_.start + n_zf_tasks_done_ * cfg->ZfBlockSize())
                ._tag);
        n_zf_tasks_done_++;
        if (n_zf_tasks_done_ == n_zf_tasks_reqd) {
          n_zf_tasks_done_ = 0;
          std::printf("Main thread: ZF done frame: %lu\n", zf_cur_frame_);
          zf_cur_frame_++;
        }
      }

      if (zf_cur_frame_ > demul_cur_frame_ &&
          rx_status_->is_demod_ready(demul_cur_frame_, demul_cur_sym_)) {
        do_demul_->independent_launch(
            demul_cur_frame_, demul_cur_sym_ - cfg->Frame().NumPilotSyms(),
            sc_range_.start + (n_demul_tasks_done_ * cfg->DemulBlockSize));

        n_demul_tasks_done_++;
        if (n_demul_tasks_done_ == n_demul_tasks_reqd) {
          /*
          // Debug printing: TODO, should use AGORA_LOG_TRACE
          for (size_t i = 0; i < 4; i++) {
              usleep(tid * 3000);
              int8_t* demul_ptr = demod_buffers_[demul_cur_frame_
                  % kFrameWnd][demul_cur_sym_
                  - cfg->Frame().NumPilotSyms()][i];
              std::printf("UE %zu: ", i);
              for (size_t i = 0; i < cfg->OFDM_DATA_NUM; i++) {
                  if (i % 20 == 0) {
                      std::printf(
                          "%d ", demul_ptr[i * cfg->ModOrderBits()]);
                  }
              }
              std::printf("\n");
          }
          */

          n_demul_tasks_done_ = 0;
          demul_status_->demul_complete(demul_cur_frame_, demul_cur_sym_,
                                        n_demul_tasks_reqd);
          demul_cur_sym_++;
          if (demul_cur_sym_ == cfg->frame().NumTotalSyms()) {
            demul_cur_sym_ = cfg->frame().NumPilotSyms();
            std::printf(
                "Main thread: Demodulation done frame: %lu "
                "(%lu UL symbols)\n",
                demul_cur_frame_,
                cfg->frame().NumTotalSyms() - cfg->frame().NumPilotSyms());
            demul_cur_frame_++;
          }
        }
      }
    }
  }

 private:
  void run_csi(size_t frame_id, size_t base_sc_id) {
    const size_t frame_slot = frame_id % kFrameWnd;
    rt_assert(base_sc_id == sc_range_.start, "Invalid SC in run_csi!");

    complex_float converted_sc[kSCsPerCacheline];

    for (size_t i = 0; i < cfg->Frame().NumPilotSyms(); i++) {
      for (size_t j = 0; j < cfg->BsAntNum(); j++) {
        auto* pkt = reinterpret_cast<Packet*>(
            socket_buffer_[j] +
            (frame_slot * cfg->frame().NumTotalSyms() * cfg->PacketLength()) +
            i * cfg->PacketLength());

        // Subcarrier ranges should be aligned with kTransposeBlockSize
        for (size_t block_idx = sc_range_.start / kTransposeBlockSize;
             block_idx < sc_range_.end / kTransposeBlockSize; block_idx++) {
          const size_t block_base_offset =
              block_idx * (kTransposeBlockSize * cfg->BsAntNum());

          for (size_t sc_j = 0; sc_j < kTransposeBlockSize;
               sc_j += kSCsPerCacheline) {
            const size_t sc_idx = (block_idx * kTransposeBlockSize) + sc_j;

            simd_convert_float16_to_float32(
                reinterpret_cast<float*>(converted_sc),
                reinterpret_cast<float*>(pkt->data +
                                         (cfg->subcarrier_start + sc_idx) * 2),
                kSCsPerCacheline * 2);

            const complex_float* src = converted_sc;
            complex_float* dst = csi_buffers_[frame_slot][i] +
                                 block_base_offset + (j * kTransposeBlockSize) +
                                 sc_j;

            // With either of AVX-512 or AVX2, load one cacheline =
            // 16 float values = 8 subcarriers = kSCsPerCacheline
            // TODO: AVX512 complex multiply support below
            size_t pilots_sgn_offset =
                cfg->server_addr_idx * cfg->get_num_sc_per_server();

            __m256 fft_result0 =
                _mm256_load_ps(reinterpret_cast<const float*>(src));
            __m256 fft_result1 =
                _mm256_load_ps(reinterpret_cast<const float*>(src + 4));
            __m256 pilot_tx0 = _mm256_set_ps(
                cfg->PilotsSgn()[sc_idx + 3 + pilots_sgn_offset].im,
                cfg->PilotsSgn()[sc_idx + 3 + pilots_sgn_offset].re,
                cfg->PilotsSgn()[sc_idx + 2 + pilots_sgn_offset].im,
                cfg->PilotsSgn()[sc_idx + 2 + pilots_sgn_offset].re,
                cfg->PilotsSgn()[sc_idx + 1 + pilots_sgn_offset].im,
                cfg->PilotsSgn()[sc_idx + 1 + pilots_sgn_offset].re,
                cfg->PilotsSgn()[sc_idx + pilots_sgn_offset].im,
                cfg->PilotsSgn()[sc_idx + pilots_sgn_offset].re);
            fft_result0 = CommsLib::__m256_complex_cf32_mult(fft_result0,
                                                             pilot_tx0, true);

            __m256 pilot_tx1 = _mm256_set_ps(
                cfg->PilotsSgn()[sc_idx + 7 + pilots_sgn_offset].im,
                cfg->PilotsSgn()[sc_idx + 7 + pilots_sgn_offset].re,
                cfg->PilotsSgn()[sc_idx + 6 + pilots_sgn_offset].im,
                cfg->PilotsSgn()[sc_idx + 6 + pilots_sgn_offset].re,
                cfg->PilotsSgn()[sc_idx + 5 + pilots_sgn_offset].im,
                cfg->PilotsSgn()[sc_idx + 5 + pilots_sgn_offset].re,
                cfg->PilotsSgn()[sc_idx + 4 + pilots_sgn_offset].im,
                cfg->PilotsSgn()[sc_idx + 4 + pilots_sgn_offset].re);
            fft_result1 = CommsLib::__m256_complex_cf32_mult(fft_result1,
                                                             pilot_tx1, true);
            _mm256_stream_ps(reinterpret_cast<float*>(dst), fft_result0);
            _mm256_stream_ps(reinterpret_cast<float*>(dst + 4), fft_result1);
          }
        }
      }
    }
  }

  /// An unused queue used for constructing Doers
  moodycamel::ConcurrentQueue<Event_data> dummy_conq_;

  /// The subcarrier range handled by this subcarrier doer.
  struct Range sc_range_;

  DoZF* do_zf_;
  DoDemul* do_demul_;
  DoPrecode* do_precode_;

  // Input buffers

  Table<char>& socket_buffer_;
  Table<int>& socket_buffer_status_;

  PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers_;
  Table<complex_float>& calib_buffer_;
  Table<int8_t>& dl_encoded_buffer_;
  Table<complex_float>& data_buffer_;

  // Output buffers

  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_;
  Table<complex_float>& dl_ifft_buffer_;

  // Intermediate buffers

  Table<complex_float>& ue_spec_pilot_buffer_;
  Table<complex_float>& equal_buffer_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& ul_zf_matrices_;
  PtrGrid<kFrameWnd, kMaxDataSCs, complex_float>& dl_zf_matrices_;

  // Shared states with TXRX threads
  RxStatus* rx_status_;

  // Internal CSI states
  size_t csi_cur_frame_ = 0;

  // Internal ZF states
  size_t zf_cur_frame_ = 0;  // Current frame waiting for CSI matrix
  size_t n_zf_tasks_done_ = 0;

  // Internal Demul states
  size_t demul_cur_frame_;    // Current frame waiting for ZF matrix
  size_t demul_cur_sym_ = 0;  // Current data symbol wait to process
  size_t n_demul_tasks_done_ = 0;

  // Shared status with Decode threads
  DemulStatus* demul_status_;
};

#endif  // DOSUBCARRIER_H_
