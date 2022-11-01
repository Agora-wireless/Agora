import numpy as np
import matplotlib
import matplotlib.pyplot as plt

# Plots IQ samples in one single frame as well as all frames for
# selected cell,  BS ant, and user
def plot_iq_samps(samps, amps, frm_st, frame_i, users, ants, data_str="Pilots"):
    # Samps Dimensions: (Frame, Cell, User, Antenna, Sample)
    for u in users:
        for a in ants:
            user_i = u
            ant_i = a
            fig, axes = plt.subplots(nrows=3, ncols=1, squeeze=False, figsize=(10, 8))
            axes[0, 0].set_title(data_str + " IQ - Antenna %d - User %d"%(ant_i, user_i))
            axes[0, 0].set_ylabel('Frame %d (IQ)' %( (frame_i + frm_st)) )
            axes[0, 0].plot(np.real(samps[frame_i, user_i, ant_i, :]))
            axes[0, 0].plot(np.imag(samps[frame_i, user_i, ant_i, :]))

            axes[1, 0].set_ylabel('All Frames (IQ)')
            axes[1, 0].plot(np.real(samps[:, user_i, ant_i, :]).flatten())
            axes[1, 0].plot(np.imag(samps[:, user_i, ant_i, :]).flatten())

            axes[2, 0].set_ylabel('magtinute (ant %d)' % (0))
            for i in range(amps.shape[1]):
                label_str = 'ant %d'%i if "Downlink" in data_str else 'user %d'%i
                axes[2, 0].plot(amps[:, i].flatten(), label=label_str)
            axes[2, 0].set_xlabel('frame')
            axes[2, 0].legend(frameon=False)

def plot_csi(csi, bs_nodes, good_frames, frame_i, ant_i, subcarrier_i, offset, data_str="Uplink"):
    fig, axes = plt.subplots(nrows=2, ncols=1, squeeze=False, figsize=(10, 8))
    axes[0, 0].set_title(data_str + " Pilot CSI Stats Across Frames- Antenna %d - Subcarrier %d" % (ant_i, subcarrier_i))
    axes[0, 0].set_ylabel('Magnitude')
    for i in range(csi.shape[1]):
        axes[0, 0].plot(np.abs(csi[:, i, ant_i, subcarrier_i]).flatten(), label="user %d" % bs_nodes[i])
    axes[0, 0].set_xlabel('Frame')

    axes[1, 0].set_ylabel('Phase')
    for i in range(csi.shape[1]):
        axes[1, 0].plot(np.angle(csi[:, i, ant_i, subcarrier_i]).flatten(), label="user %d" % bs_nodes[i])
    axes[1, 0].set_ylim(-np.pi, np.pi)
    axes[1, 0].set_xlabel('Frame')

    lines, labels = axes[-1, 0].get_legend_handles_labels()
    fig.legend(lines, labels, loc = 'upper right', frameon=False)

    fig, axes = plt.subplots(nrows=4, ncols=1, squeeze=False, figsize=(10, 8))
    axes[0, 0].set_title(data_str + " Pilot CSI Stats Across Subcarriers - Cell 0 - Frame %d - Ant %d" % (frame_i, ant_i))
    axes[0, 0].set_ylabel('Magnitude user 0')
    axes[0, 0].plot(np.abs(csi[frame_i, 0, ant_i, :]).flatten())
    axes[0, 0].set_ylim(0, 1)
    axes[0, 0].set_xlabel('Subcarrier')

    axes[1, 0].set_ylabel('Phase user 0')
    axes[1, 0].plot(np.angle(csi[frame_i, 0, ant_i, :].flatten()))
    axes[1, 0].set_ylim(-np.pi, np.pi)
    axes[1, 0].set_xlabel('Subcarrier')

    axes[2, 0].set_ylabel('Magnitude')
    for i in range(csi.shape[1]):
        axes[2, 0].plot(np.abs(csi[frame_i, i, ant_i, :]).flatten(), label="user %d" % i)
    axes[0, 0].set_ylim(0, 1)
    axes[2, 0].set_xlabel('Subcarrier')
    axes[2, 0].legend(loc='lower right', frameon=False)

    axes[3, 0].set_ylabel('Phase')
    for i in range(csi.shape[1]):
        axes[3, 0].plot(np.angle(csi[frame_i, i, ant_i, :]).flatten(), label="user %d" % i)
    axes[3, 0].set_xlabel('Subcarrier')
    axes[3, 0].set_ylim(-np.pi, np.pi)
    axes[3, 0].legend(loc='lower right', frameon=False)

def plot_constellation_stats(evm, evm_snr, ser, ul_data, txdata, frame_i, ul_slot_i, data_str = "Uplink"):
    n_users = ul_data.shape[2]
    plt_x_len = int(np.ceil(np.sqrt(n_users)))
    plt_y_len = int(np.ceil(n_users / plt_x_len))
    fig5, axes5 = plt.subplots(nrows=plt_y_len, ncols=plt_x_len, squeeze=False, figsize=(10, 8))
    fig5.suptitle(data_str+" User Constellations (ZF) - Frame %d - UL SF %d" % (frame_i, ul_slot_i))
    fig6, axes6 = plt.subplots(nrows=3, ncols=1, squeeze=False, figsize=(10, 8))
    fig6.suptitle('Uplink EVM/SNR/SER - UL SF %d' % (ul_slot_i))
    axes6[0, 0].set_ylabel('EVM (%)')
    axes6[1, 0].set_ylabel('EVM-SNR (dB)')
    axes6[2, 0].set_ylabel('Symbol Error Rate')
    axes6[2, 0].set_xlabel('Frame Number')
    for i in range(n_users):
        y_i = int(i // plt_x_len)
        x_i = i % plt_x_len
        tx_max = np.max(np.real(txdata[frame_i, ul_slot_i, i, :]))
        tx_color = (tx_max - np.real(txdata[frame_i, ul_slot_i, i, :])) * 2 * tx_max + (tx_max - np.imag(txdata[frame_i, ul_slot_i, i, :]))
        axes5[y_i, x_i].set_title('User %d'%(i))
        axes5[y_i, x_i].scatter(np.real(ul_data[frame_i, ul_slot_i, i, :]), np.imag(ul_data[frame_i, ul_slot_i, i, :]), c=tx_color, cmap='rainbow')
        axes5[y_i, x_i].scatter(np.real(txdata[frame_i, ul_slot_i, i, :]), np.imag(txdata[frame_i, ul_slot_i, i, :]), marker='*', s=300, c=tx_color, cmap='rainbow')

        axes6[0, 0].plot(range(evm.shape[0]), 100 * evm[:, ul_slot_i, i], label='User %d'%(i))
        axes6[1, 0].plot(range(evm.shape[0]), evm_snr[:, ul_slot_i, i], label='User %d'%(i))
        axes6[2, 0].plot(range(ser.shape[0]), ser[:, ul_slot_i, i], label='User %d'%(i))
    axes6[0, 0].legend(loc='upper right', frameon=False)

def plot_snr_map(snr, n_frm_st, n_frm_end, n_ant, sub_sample=1):
    n_ue = snr.shape[1]
    fig, axes = plt.subplots(nrows=n_ue, ncols=1, squeeze=False)
    c = []
    fig.suptitle('SNR Map')
    for n_u in range(n_ue):
        c.append(
            axes[n_u, 0].imshow(snr[:, n_u, :].T, vmin=np.min(snr), vmax=np.max(snr), cmap='Blues',
                                  interpolation='nearest',
                                  extent=[n_frm_st, n_frm_end, n_ant, 0],
                                  aspect="auto"))
        axes[n_u, 0].set_title('UE {}'.format(n_u))
        axes[n_u, 0].set_ylabel('Antenna #')
        axes[n_u, 0].set_xlabel('Frame #')
        axes[n_u, 0].set_xticks(np.arange(n_frm_st, n_frm_end, sub_sample), minor=True)
        axes[n_u, 0].set_yticks(np.arange(0, n_ant, 1), minor=True)
        axes[n_u, 0].grid(which='minor', color='0.75', linestyle='-', linewidth=0.05)
    cbar = plt.colorbar(c[-1], ax=axes.ravel().tolist(), ticks=np.linspace(0, np.max(snr), 10),
                        orientation='horizontal')

