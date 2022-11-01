/**
 * @file doifft_client.h
 * @brief Declaration file for the DoIFFTClient class.
 */
#ifndef DOIFFT_CLIENT_H_
#define DOIFFT_CLIENT_H_

#include "common_typedef_sdk.h"
#include "config.h"
#include "doer.h"
#include "memory_manage.h"
#include "mkl_dfti.h"
#include "stats.h"
#include "symbols.h"

class DoIFFTClient : public Doer {
 public:
  DoIFFTClient(Config* in_config, int in_tid,
               Table<complex_float>& in_ifft_buffer, char* in_socket_buffer,
               Stats* in_stats_manager);
  ~DoIFFTClient() override;

  /**
   * Do modulation and ifft tasks for one OFDM symbol
   * @param tid: task thread index, used for selecting task ptok
   * Buffers: IQ_data_long
   *     Input buffer: IQ_data_long
   *     Output buffer: iffted_data_buffer_
   *     Intermediate buffer: ifft_buffer_
   * Offsets:
   *     dl_IQ_data_long_:
   *         dim1: data symbol index in the current frame * # of users +
   * user index dim2: subcarrier index ifft_buffer_: dim1: frame index * #
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
  EventData Launch(size_t tag) override;

 private:
  Table<complex_float>& ifft_buffer_;
  char* socket_buffer_;
  DurationStat* duration_stat_;
  DFTI_DESCRIPTOR_HANDLE mkl_handle_;
  float* ifft_out_;  // Buffer for IFFT output
  complex_float* ifft_shift_tmp_;
  float ifft_scale_factor_;
};

#endif  // DOIFFT_CLIENT_H_
