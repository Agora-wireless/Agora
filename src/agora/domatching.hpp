#ifndef DOMATCHING
#define DOMATCHING

#include "Symbols.hpp"
#include "buffer.hpp"
#include "concurrentqueue.h"
#include "config.hpp"
#include "doer.hpp"
#include "gettime.h"
#include "memory_manage.h"
#include "modulation.hpp"
#include "phy_stats.hpp"
#include "stats.hpp"
#include <armadillo>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "iobuffer.hpp"

class DoRMatching : public Doer {
public:
    DoRMatching(Config* in_config, int in_tid, Table<int8_t>& in_encoded_data_buffer,
        Table<int8_t>& in_rmatched_buffer, Stats* in_stats_manager);
    ~DoRMatching();

    Event_data launch(size_t tag);

private:
    Table<int8_t>& encoded_data_buffer_;

    // Intermediate buffer to hold Rate Matching output
    Table<int8_t>& rmatching_buffer_;
    DurationStat* duration_stat;
};

class DoDMatching : public Doer {
public:
    DoDMatching(Config* in_config, int in_tid,
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers,
        PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& dematched_buffers,
        PhyStats* in_phy_stats, Stats* in_stats_manager);
    ~DoDMatching();

    Event_data launch(size_t tag);

private:
    int16_t* resp_var_nodes;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, int8_t>& demod_buffers_;
    PtrCube<kFrameWnd, kMaxSymbols, kMaxUEs, uint8_t>& dematched_buffers_;
    PhyStats* phy_stats;
    DurationStat* duration_stat;
};

#endif
