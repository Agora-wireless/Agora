/**
 * @file dofft.h
 * @brief Declaration file for the DoFFT class.
 */
#ifndef DOFFT_H_
#define DOFFT_H_

#include <complex>
#include <cstdint>

#include "common_typedef_sdk.h"
#include "config.h"
#include "doer.h"
#include "memory_manage.h"
#include "message.h"
#include "mkl_dfti.h"
#include "phy_stats.h"
#include "stats.h"
#include "symbols.h"

class DoFFT : public Doer {
 public:
  DoFFT(Config* config, size_t tid, Table<complex_float>& data_buffer,
        PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers,
        Table<complex_float>& calib_dl_buffer,
        Table<complex_float>& calib_ul_buffer, PhyStats* in_phy_stats,
        Stats* stats_manager);
  ~DoFFT() override;

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
  EventData Launch(size_t tag) override;

  /**
   * Fill-in the partial transpose of the computed FFT for this antenna into
   * out_buf.
   *
   * The fully-transposed matrix after FFT is a subcarriers x antennas matrix
   * that should look like so (using the notation subcarrier/antenna, and
   * assuming kTransposeBlockSize = 16)
   *
   * 0/0, 0/1, ........................................................., 0/63
   * 1/0, 0/1, ........................................................ , 1/63
   * ...
   * 15/0, 15/1, ......................................................, 15/63
   * ...
   * 1199/0, 1199/1, ............................................... , 1199/63
   *
   *
   * The partially-tranposed matrix looks like so:
   * 0/0 1/0 ... 15/0  0/1 1/1 ... 15/1 .................... 0/3 1/3 .... 15/3
   * 0/4 1/4 ... 15/4  0/5 1/5 ... 15/5 .................... 0/7 1/5 .... 15/7
   * ...
   * ...........................................................0/63 ... 15/63
   * <end of partial transpose block>
   * 16/0 17/0 ... 31/0 ....................................16/3 17/3 ... 31/3
   * 16/4 17/4 ... 31/4 ....................................16/7 17/7 ... 31/7
   *
   *
   * Each partially-transposed block is identical to the corresponding block
   * of the fully-transposed matrix, but laid out in memory in column-major
   * order.
   */
  void PartialTranspose(complex_float* out_buf, size_t ant_id,
                        SymbolType symbol_type) const;

 private:
  Table<complex_float>& data_buffer_;
  PtrGrid<kFrameWnd, kMaxUEs, complex_float>& csi_buffers_;
  Table<complex_float>& calib_dl_buffer_;
  Table<complex_float>& calib_ul_buffer_;
  DFTI_DESCRIPTOR_HANDLE mkl_handle_;
  complex_float* fft_inout_;      // Buffer for both FFT input and output
  complex_float* fft_shift_tmp_;  // Buffer for both FFT input and output

  // Buffer for store 16-bit IQ converted from 12-bit IQ
  uint16_t* temp_16bits_iq_;
  std::complex<float>* rx_samps_tmp_;  // Temp buffer for received samples

  DurationStat* duration_stat_fft_;
  DurationStat* duration_stat_csi_;
  PhyStats* phy_stats_;
};

#endif  // DOFFT_H_
