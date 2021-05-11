/**
 * @file dodecode_client.h
 * @brief Declaration file for the DoDecode class.
 */

#ifndef DODECODE_CLIENT_H_
#define DODECODE_CLIENT_H_

#include <memory>

#include "config.h"
#include "doer.h"
#include "memory_manage.h"
#include "phy_stats.h"
#include "scrambler.h"
#include "stats.h"

class DoDecodeClient : public Doer {
 public:
  DoDecodeClient(
      Config* in_config, int in_tid,
      PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
      PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffers,
      PhyStats* in_phy_stats, Stats* in_stats_manager);
  ~DoDecodeClient() override;

  EventData Launch(size_t tag) override;

 private:
  int16_t* resp_var_nodes_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_;
  PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& decoded_buffers_;
  PhyStats* phy_stats_;
  DurationStat* duration_stat_;
  std::unique_ptr<AgoraScrambler::Scrambler> scrambler_;
};

#endif  // DODECODE_CLIENT_H_
