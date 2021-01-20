#ifndef DOCODING
#define DOCODING

#include <stdio.h>
#include <string.h>

#include <armadillo>
#include <iostream>
#include <vector>

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "encoder.hpp"
#include "gettime.h"
#include "iobuffer.hpp"
#include "memory_manage.h"
#include "modulation.hpp"
#include "phy_stats.hpp"
#include "stats.hpp"
#include "utils_ldpc.hpp"

class DoEncode : public Doer {
 public:
  DoEncode(Config* in_config, int in_tid, Table<int8_t>& in_raw_data_buffer,
           Table<int8_t>& in_encoded_buffer, Stats* in_stats_manager);
  ~DoEncode();

  Event_data launch(size_t tag);

 private:
  Table<int8_t>& raw_data_buffer_;
  int8_t* parity_buffer;  // Intermediate buffer to hold LDPC encoding parity

  // Intermediate buffer to hold LDPC encoding output
  int8_t* encoded_buffer_temp;
  Table<int8_t>& encoded_buffer_;
  DurationStat* duration_stat;
};

class DoDecode : public Doer {
 public:
  DoDecode(Config* in_config, int in_tid,
           PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
           PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers,
           PhyStats* in_phy_stats, Stats* in_stats_manager);
  ~DoDecode();

  Event_data launch(size_t tag);

 private:
  int16_t* resp_var_nodes;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& decoded_buffers_;
  PhyStats* phy_stats;
  DurationStat* duration_stat;
};

#endif
