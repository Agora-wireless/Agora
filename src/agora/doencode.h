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
           Table<int8_t>& in_encoded_buffer, Stats* in_stats_manager);
  ~DoEncode() override;

  EventData Launch(size_t tag) override;

 private:
  Table<int8_t>& raw_data_buffer_;
  int8_t* parity_buffer_;  // Intermediate buffer to hold LDPC encoding parity

  // Intermediate buffer to hold LDPC encoding output
  int8_t* encoded_buffer_temp_;
  Table<int8_t>& encoded_buffer_;
  DurationStat* duration_stat_;
  int8_t* scrambler_buffer_;

  std::unique_ptr<AgoraScrambler::Scrambler> scrambler_;
};

#endif  // DOENCODE_H_
