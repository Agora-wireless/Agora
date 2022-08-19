/**
 * @file dodecode.h
 * @brief Declaration file for the DoDecode class.
 */

#ifndef DODECODE_H_
#define DODECODE_H_

#include <cstdint>
#include <memory>

#include "buffer.h"
#include "config.h"
#include "doer.h"
#include "message.h"
#include "phy_stats.h"
#include "scrambler.h"
#include "stats.h"

class DoDecode : public Doer {
 public:
  DoDecode(Config* in_config, int in_tid, AgoraBuffer* buffer,
           PhyStats* in_phy_stats, Stats* in_stats_manager);
  ~DoDecode() override;

  EventData Launch(size_t tag) override;

 private:
  int16_t* resp_var_nodes_;
  AgoraBuffer* buffer_;
  PhyStats* phy_stats_;
  DurationStat* duration_stat_;
  std::unique_ptr<AgoraScrambler::Scrambler> scrambler_;
};

#endif  // DODECODE_H_
