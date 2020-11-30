/*
 * Channel Model
 *
 *
 *
 */

#include "channel.hpp"
#include "sim_utils.hpp"

static constexpr bool kPrintChannelOutput = true;


Channel::Channel(Config* config_bs, Config* config_ue)
  : bscfg(config_bs)
  , uecfg(config_ue)
{
    bs_ant = bscfg->BS_ANT_NUM;
    ue_ant = uecfg->UE_ANT_NUM;
}

Channel::~Channel()
{

}

void Channel::apply_chan(const cx_fmat& fmat_src, cx_fmat& fmat_dst)
{
    /* 
     *
     *
     * Dimensions of fmat_src: ( bscfg->sampsPerSymbol, uecfg->UE_ANT_NUM )
     */
    
    // After applying channel
    cx_fmat fmat_src_chan;

    switch(bscfg->chan_model)
    {
        case Config::AWGN:
            awgn(fmat_src, fmat_dst);
	    break;

        case Config::RAYLEIGH:
	    rayleigh(fmat_src, fmat_src_chan);
	    awgn(fmat_src_chan, fmat_dst);
	    break;

        case Config::RAN_3GPP:
	    lte_3gpp(fmat_src, fmat_dst);
	    break;

        case Config::NONE:
	    {
		// Mostly for debugging...
		cx_fmat H(ones<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM),
                          zeros<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM));
                fmat_dst = fmat_src * H;
	    }
	    break;
    }

    if (kPrintChannelOutput)
        print_cxmat(fmat_dst);
     
}

void Channel::awgn(const cx_fmat& fmat_src, cx_fmat& fmat_dst)
{
    /*
     * Additive White Gaussian Noise 
     */
    // Dimensions of fmat_src: ( bscfg->sampsPerSymbol, uecfg->UE_ANT_NUM )
    int n_row = fmat_src.n_rows;
    int n_col = fmat_src.n_cols;
    float snr_lin = pow(10, bscfg->sim_snr_db/10);

    // Power spectral density of noise
    fmat fmat_src_sq = square(abs(fmat_src));
    frowvec pwr_vec = sum(fmat_src_sq, 0) / n_row;  //pwr = sum(abs(samps)ˆ2)/length(samps)
    frowvec n0 = pwr_vec / snr_lin;
    frowvec n = sqrt(n0 / 2);

    // Generate noise
    cx_fmat noise(randn<fmat>(n_row, n_col), randn<fmat>(n_row, n_col));
    fmat n0_mat = repmat(n0, n_row, 1);
    noise = noise % n0_mat;  // Element-wise multiplication

    // Add noise to signal
    cx_fmat s_n = fmat_src + noise;
    fmat_dst = s_n;
    //printf("S+N: \n");
    //print_cxmat(s_n);
}

void Channel::rayleigh(const cx_fmat& fmat_src, cx_fmat& fmat_dst)
{
    /*
     * Simple Uncorrelated Rayleigh Channel
     * - Flat fading (single tap)
     */

    cx_fmat H(randn<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM),
        randn<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM));
    H = (1/sqrt(2)) * H;
    fmat_dst = fmat_src * H;
}

void Channel::lte_3gpp(const cx_fmat& fmat_src, cx_fmat& fmat_dst)
{
    /*
     * From "Study on 3D-channel model for Elevation Beamforming
     *       and FD-MIMO studies for LTE"
     *       3GPP TR 36.873 V12.7.0 (2017-12)
     * Note: FD-MIMO Stands for Full-Dimension MIMO
     * Currenlty implements the 3D-UMa scenario only. This
     * corresponds to an Urban Macro cell with high UE density
     * indoor and outdoor. This scenario assumes Base Stations
     * are above surrounding buildings.
     *
     */

    // FIXME
}
