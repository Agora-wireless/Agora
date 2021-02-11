#ifndef DOCODING
#define DOCODING

#include <armadillo>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <vector>

#include "buffer.h"
#include "concurrentqueue.h"
#include "config.h"
#include "doer.h"
#include "encoder.h"
#include "gettime.h"
#include "iobuffer.h"
#include "memory_manage.h"
#include "modulation.h"
#include "phy_stats.h"
#include "stats.h"
#include "symbols.h"
#include "utils_ldpc.h"

class DoEncode : public Doer {
 public:
  DoEncode(Config* in_config, int in_tid, Table<int8_t>& in_raw_data_buffer,
           Table<int8_t>& in_encoded_buffer, Stats* in_stats_manager);
  ~DoEncode();

  EventData Launch(size_t tag);

 private:
  Table<int8_t>& raw_data_buffer_;
  int8_t* parity_buffer_;  // Intermediate buffer to hold LDPC encoding parity

  // Intermediate buffer to hold LDPC encoding output
  int8_t* encoded_buffer_temp_;
  Table<int8_t>& encoded_buffer_;
  DurationStat* duration_stat_;
};

class DoDecode : public Doer {
 public:
  DoDecode(Config* in_config, int in_tid,
           PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
           PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
           PhyStats* in_phy_stats, Stats* in_stats_manager);
  ~DoDecode();

  EventData Launch(size_t tag);

 private:
  int16_t* resp_var_nodes_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers_;
  PhyStats* phy_stats_;
  DurationStat* duration_stat_;
};

#endif
