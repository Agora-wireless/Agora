/*
 * Channel Model
 *
 *
 *
 */

#ifndef SIM_CHAN_MODEL
#define SIM_CHAN_MODEL

#include "Symbols.hpp"
#include "buffer.hpp"
#include "config.hpp"
#include "utils.h"
#include "gettime.h"
#include "memory_manage.h"
#include "signalHandler.hpp"
#include <algorithm>
#include <armadillo>
#include <assert.h>
#include <ctime>
#include <math.h>
#include <numeric>
#include <iomanip>

using namespace arma;

class Channel {

public:


    Channel (Config* bscfg, Config* uecfg);
    ~Channel ();
    void apply_chan(const cx_fmat& fmat_src, cx_fmat& mat_dst, const bool is_downlink, const bool is_newChan);
    void awgn(const cx_fmat& fmat_src, cx_fmat& fmat_dst);
    void lte_3gpp(const cx_fmat& fmat_src, cx_fmat& fmat_dst);

private:

    Config* bscfg;
    Config* uecfg;

    Channel* channel;
    size_t bs_ant;
    size_t ue_ant;
    size_t n_samps;

    cx_fmat H;
};

#endif /* SIM_CHAN_MODEL */
