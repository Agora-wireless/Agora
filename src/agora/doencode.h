/**
 * @file doencode.h
 * @brief Declaration file for the Docoding class.  Includes the DoEncode and
 * DoDecode classes.
 */

#ifndef DOENCODE_H_
#define DOENCODE_H_

#include <memory>

#include "buffer.h"
#include "config.h"
#include "doer.h"
#include "memory_manage.h"
#include "scrambler.h"
#include "stats.h"

class DoEncode : public Doer {
 public:
  DoEncode(Config* in_config, int in_tid, Table<int8_t>& in_raw_data_buffer,
           Table<int8_t>& in_encoded_buffer, size_t in_mac_frame_wnd,
           size_t in_mac_bytes_perframe, Stats* in_stats_manager);
  ~DoEncode() override;

  EventData Launch(size_t tag) override;

 private:
  size_t mac_frame_wnd_;
  size_t mac_bytes_perframe_;
  // References to buffers allocated pre-construction
  Table<int8_t>& mac_data_buffer_;
  Table<int8_t>& encoded_buffer_;

  // Intermediate buffer to hold LDPC encoding parity
  int8_t* parity_buffer_;

  // Intermediate buffer to hold LDPC encoding output
  int8_t* encoded_buffer_temp_;

  // Intermediate buffer to hold pre/post scrambled data
  int8_t* scrambler_buffer_;

  DurationStat* duration_stat_;
  std::unique_ptr<AgoraScrambler::Scrambler> scrambler_;
};

#endif  // DOENCODE_H_
