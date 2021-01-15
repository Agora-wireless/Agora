#include "channel.hpp"

static constexpr bool kPrintChannelOutput = false;
using namespace arma;

Channel::Channel(Config* config_bs, Config* config_ue,
    std::string in_channel_type, double in_channel_snr)
    : bscfg(config_bs)
    , uecfg(config_ue)
    , sim_chan_model(in_channel_type)
    , channel_snr_db(in_channel_snr)
{
    bs_ant = bscfg->bs_ant_num();
    ue_ant = uecfg->ue_ant_num();
    n_samps = bscfg->samps_per_symbol();

    if (sim_chan_model == "AWGN")
        chan_model = AWGN;
    else if (sim_chan_model == "RAYLEIGH")
        chan_model = RAYLEIGH;
    else if (sim_chan_model == "RAN_3GPP") {
        chan_model = RAN_3GPP;
        printf("3GPP Model in progress, setting to RAYLEIGH channel \n");
        chan_model = RAYLEIGH;
    } else
        chan_model = AWGN;
}

Channel::~Channel() {}

void Channel::apply_chan(const cx_fmat& fmat_src, cx_fmat& fmat_dst,
    const bool is_downlink, const bool is_newChan)
{
    cx_fmat fmat_H;

    if (is_newChan) {
        switch (chan_model) {
        case AWGN: {
            fmat rmat(ue_ant, bs_ant, fill::ones);
            fmat imat(ue_ant, bs_ant, fill::zeros);
            H = cx_fmat(rmat, imat);
            // H = H / abs(H).max();
        } break;

        case RAYLEIGH:
            // Simple Uncorrelated Rayleigh Channel - Flat fading (single tap)
            {
                fmat rmat(ue_ant, bs_ant, fill::randn);
                fmat imat(ue_ant, bs_ant, fill::randn);
                H = cx_fmat(rmat, imat);
                H = (1 / sqrt(2)) * H;
                // H = H / abs(H).max();
            }
            break;

        case RAN_3GPP:
            lte_3gpp(fmat_src, fmat_dst);
            break;
        }
    }
    if (is_downlink)
        fmat_H = fmat_src * H.st() / std::sqrt(bscfg->bs_ant_num());
    else
        fmat_H = fmat_src * H;

    // Add noise
    awgn(fmat_H, fmat_dst);

    if (kPrintChannelOutput)
        Utils::print_mat(H);
}

void Channel::awgn(const cx_fmat& src, cx_fmat& dst)
{
    int n_row = src.n_rows;
    int n_col = src.n_cols;
    float snr_lin = pow(10, channel_snr_db / 10);

    // Power spectral density of noise
    fmat src_sq = square(abs(src));
    // pwr = sum(abs(samps)ˆ2)/length(samps)
    frowvec pwr_vec = mean(src_sq, 0);
    frowvec n0 = pwr_vec / snr_lin;
    frowvec n = sqrt(n0 / 2);

    // Generate noise
    cx_fmat noise(randn<fmat>(n_row, n_col), randn<fmat>(n_row, n_col));
    // Supposed to be faster
    // fmat x(n_row, n_col, fill::randn);
    // fmat y(n_row, n_col, fill::randn);
    // cx_fmat noise = cx_fmat(x, y);

    fmat n_mat = repmat(n, n_row, 1);
    // Element-wise multiplication
    noise = noise % n_mat;
    // Add noise to signal
    dst = src + noise;

    // Check SNR
    /*
    fmat noise_sq = square(abs(noise));
    frowvec noise_vec = mean(noise_sq, 0);
    frowvec snr = 10 * log10(pwr_vec / noise_vec);
    printf("SNR: ");
    std::cout << snr << std::endl;
    */
}

void Channel::lte_3gpp(const cx_fmat& fmat_src, cx_fmat& fmat_dst)
{
    // TODO - In progress (Use Rayleigh for now...)
    cx_fmat H(randn<fmat>(uecfg->ue_ant_num(), bscfg->bs_ant_num()),
        randn<fmat>(uecfg->ue_ant_num(), bscfg->bs_ant_num()));
    H = (1 / sqrt(2)) * H;
    fmat_dst = fmat_src * H;
}
