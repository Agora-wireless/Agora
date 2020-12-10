/*
 * Channel Model
 *
 *
 *
 */

#include "channel.hpp"

static constexpr bool kPrintChannelOutput = false;
using namespace arma;

Channel::Channel(Config* config_bs, Config* config_ue)
  : bscfg(config_bs)
  , uecfg(config_ue)
{
    bs_ant = bscfg->BS_ANT_NUM;
    ue_ant = uecfg->UE_ANT_NUM;
    n_samps = bscfg->sampsPerSymbol;
}


Channel::~Channel()
{

}

void Channel::apply_chan(const cx_fmat& fmat_src, cx_fmat& fmat_dst, const bool is_downlink)
{
    /* 
     *
     *
     * Dimensions of fmat_src:
     * ( bscfg->sampsPerSymbol, uecfg->UE_ANT_NUM )
     */
    
    // After applying channel
    cx_fmat fmat_H;
    fmat rmat(ue_ant, bs_ant, fill::ones);
    fmat imat(ue_ant, bs_ant, fill::zeros);
    cx_fmat H = cx_fmat(rmat, imat);

    switch(bscfg->chan_model)
    {
        case Config::AWGN:
            if (is_downlink)
                fmat_H = fmat_src * H.st();
            else
                fmat_H = fmat_src * H;
            awgn(fmat_H, fmat_dst);
	    break;

        case Config::RAYLEIGH:
	    rayleigh(fmat_src, fmat_H, is_downlink);
	    awgn(fmat_H, fmat_dst);
	    break;

        case Config::RAN_3GPP:
	    lte_3gpp(fmat_src, fmat_dst);
	    break;

        case Config::NONE:
            // Mostly for debugging...
            if (is_downlink)
                fmat_dst = fmat_src * H.st();
            else
                fmat_dst = fmat_src * H;
	    break;
    }

    if (kPrintChannelOutput)
        Utils::print_mat(fmat_dst);
}

void Channel::awgn(const cx_fmat& src, cx_fmat& dst)
{
    /*
     * Additive White Gaussian Noise 
     */
    // Dimensions of src: ( bscfg->sampsPerSymbol, uecfg->UE_ANT_NUM )

    int n_row = src.n_rows;
    int n_col = src.n_cols;
    float snr_lin = pow(10, bscfg->sim_snr_db/10);

    // Power spectral density of noise
    fmat src_sq = square(abs(src));
    frowvec pwr_vec = mean(src_sq, 0); //pwr = sum(abs(samps)ˆ2)/length(samps)
    frowvec n0 = pwr_vec / snr_lin;
    frowvec n = sqrt(n0 / 2);

    // Generate noise
    cx_fmat noise(randn<fmat>(n_row, n_col), randn<fmat>(n_row, n_col));
    // Supposed to be faster
    //fmat x(n_row, n_col, fill::randn);
    //fmat y(n_row, n_col, fill::randn);
    //cx_fmat noise = cx_fmat(x, y);

    fmat n_mat = repmat(n, n_row, 1);
    noise = noise % n_mat;  // Element-wise multiplication

    // Add noise to signal
    dst = src + noise;

    printf("Inside: \n");
    Utils::print_mat(dst);
    // CHECK!!
    fmat noise_sq = square(abs(noise));
    frowvec noise_vec = mean(noise_sq, 0);
    frowvec snr = 10 * log10(pwr_vec / noise_vec);
    printf("SNR: ");
    std::cout << snr << std::endl;
}

void Channel::rayleigh(const cx_fmat& fmat_src, cx_fmat& fmat_dst, const bool is_downlink)
{
    /*
     * Simple Uncorrelated Rayleigh Channel
     * - Flat fading (single tap)
     */

    cx_fmat H(randn<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM),
        randn<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM));
    H = (1/sqrt(2)) * H;

    if (is_downlink)
        fmat_dst = fmat_src * H.st();
    else
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

    // TODO - In progress (Use Rayleigh for now...)
    cx_fmat H(randn<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM),
        randn<fmat>(uecfg->UE_ANT_NUM, bscfg->BS_ANT_NUM));
    H = (1/sqrt(2)) * H;
    fmat_dst = fmat_src * H;
}
