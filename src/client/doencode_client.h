/**
 * @file doencode_client.h
 * @brief Declaration file for the DoEncodeClient class.
 */

#ifndef DOENCODE_CLIENT_H_
#define DOENCODE_CLIENT_H_

#include <memory>

#include "buffer.h"
#include "config.h"
#include "doer.h"
#include "memory_manage.h"
#include "scrambler.h"
#include "stats.h"

class DoEncodeClient : public Doer {
 public:
  DoEncodeClient(Config* in_config, size_t in_tid,
                 Table<int8_t>& in_raw_data_buffer,
                 Table<int8_t>& out_encoded_buffer, Stats* in_stats_manager);
  ~DoEncodeClient() override;

  EventData Launch(size_t tag) override;

 private:
  // References to buffers allocated pre-construction
  Table<int8_t>& raw_data_buffer_;
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

#endif  // DOENCODE_CLIENT_H_
