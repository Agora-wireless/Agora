import numpy as np
from scipy import signal

def extract_pilots_data(cmpx_pilots, seq_conj, seq_num, seq_len, frame):
    # normalized matched filter
    a = 1
    l_fc = len(seq_conj)
    unos = np.ones(l_fc)
    v0 = signal.lfilter(seq_conj, a, cmpx_pilots, axis=2)
    v1 = signal.lfilter(unos, a, (abs(cmpx_pilots)**2), axis=2)
    #supress invalid floating point operation error
    np.seterr(invalid='ignore')
    m_filt = (np.abs(v0)**2)/v1

    # clean up nan samples: replace nan with -1
    nan_indices = np.argwhere(np.isnan(m_filt))
    m_filt[np.isnan(m_filt)] = -0.5  # the only negative value in m_filt

    #if write_to_file:
    #    # write the nan_indices into a file
    #    np.savetxt("nan_indices.txt", nan_indices, fmt='%i')

    #if debug:
    #    print("Shape of truncated complex pilots: {} , l_lts_fc = {}, v0.shape = {}, v1.shape = {}, m_filt.shape = {}".
    #          format(cmpx_pilots.shape, l_lts_fc, v0.shape, v1.shape, m_filt.shape))

    rho_max = np.amax(m_filt, axis=2)         # maximum peak per SF per antenna
    rho_min = np.amin(m_filt, axis=2)        # minimum peak per SF per antenna
    ipos = np.argmax(m_filt, axis=2)          # positons of the max peaks
    sf_start = ipos - l_fc + 1             # start of every received SF
    # get rid of negative indices in case of an incorrect peak
    sf_start = np.where(sf_start < 0, 0, sf_start)

    # get the pilot samples from the cmpx_pilots array and reshape for seq_num LTS pilots:
    n_ue = cmpx_pilots.shape[0]         # no. of UEs
    n_ant = cmpx_pilots.shape[1]        # no. of BS antennas
    pilots_rx_t = np.empty(
        [n_ue, n_ant, seq_num * seq_len], dtype='complex64')

    for k in range(n_ue):
        for l in range(n_ant):
            pilots_rx_t[k, l, :] = cmpx_pilots[k, l, sf_start[k, l]:  sf_start[k, l] + (seq_num * seq_len)]

    return frame, pilots_rx_t, m_filt, sf_start

if __name__ == '__main__':
    from generate_sequence import *
    seq_tmp,_ = generate_training_seq(preamble_type='zadoff-chu', seq_length=336, cp=32, upsample=1, reps=[])
    rep = 1
    seq_orig = np.tile(seq_tmp, rep)
    sig = np.concatenate((128 * [0], seq_orig))
    sig = np.concatenate((sig, 128 * [0]))
    seq = seq_orig[::-1]  # flip
    # conjugate the local sequence
    seq_conj = np.conjugate(seq)
    sig3d = np.empty((1, 1, len(sig)), dtype='complex64')
    sig3d[0, 0, :] = sig
    frame, pilot_rx_t, m_filt, sf_start = extract_pilots_data(sig3d, seq_conj, rep, len(seq_tmp), 0)
    print(sf_start)
    fig = plt.figure()
    ax1 = fig.add_subplot(2, 1, 1)
    ax1.grid(True)
    ax1.plot(np.abs(pilot_rx_t[0, 0, :]))
    ax2 = fig.add_subplot(2, 1, 2)
    ax2.grid(True)
    ax2.plot(np.abs(m_filt[0, 0, :]))
    #ax2.scatter(pilot_pks, 2 * np.ones(len(pilot_pks)))
    plt.show()