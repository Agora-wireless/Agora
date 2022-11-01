/**
 * @file doencode.h
 * @brief Declaration file for the Docoding class.  Includes the DoEncode and
 * DoDecode classes.
 */

#ifndef DOENCODE_H_
#define DOENCODE_H_

#include <cstdint>
#include <memory>

#include "config.h"
#include "doer.h"
#include "memory_manage.h"
#include "message.h"
#include "scrambler.h"
#include "stats.h"

class DoEncode : public Doer {
 public:
  DoEncode(Config* in_config, int in_tid, Direction dir,
           Table<int8_t>& in_raw_data_buffer, size_t in_buffer_rollover,
           Table<int8_t>& in_mod_bits_buffer, Stats* in_stats_manager);
  ~DoEncode() override;

  EventData Launch(size_t tag) override;

 private:
  Direction dir_;

  // References to buffers allocated pre-construction
  Table<int8_t>& raw_data_buffer_;
  size_t raw_buffer_rollover_;
  Table<int8_t>& mod_bits_buffer_;

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
