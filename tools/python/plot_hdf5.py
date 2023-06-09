#!/usr/bin/python3
"""
 plot_hdf5.py
 Plotting from HDF5 file
 Script to analyze recorded hdf5 file from channel sounding (see Sounder/).
 Usage format is:
    ./plot_hdf5.py <hdf5_file_name>
 Example:
    ./plot_hdf5.py ../Sounder/logs/test-hdf5.py
---------------------------------------------------------------------
 Copyright © 2018-2022. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import numpy as np
import h5py
import matplotlib.pyplot as plt
import collections
import time
from optparse import OptionParser
import hdf5_lib
from hdf5_lib import *
import matplotlib
from plot_lib import *
#matplotlib.use("Agg")

def verify_hdf5(hdf5, frame_i=100, cell_i=0, ofdm_sym_i=0, ant_i=0,
                user_i=0, ul_slot_i=0, dl_slot_i=0, subcarrier_i=10, 
                offset=-1, thresh=0.001, deep_inspect=False, 
                corr_thresh=0.00, exclude_bs_nodes=[], demod=""):
    """Plot data in the hdf5 file to verify contents.
    Args:
        hdf5: An hdf5_lib object.
        frame_i: The index of the frame to be plotted.
        cell_i: The index of the hub where base station is connected.
        ofdm_sym_i: The index of the reference ofdm symbol in a pilot.
        ant_i: The index of the reference base station antenna.
        user_i: The index of the reference user.
    """
    plt.close("all")

    # Retrieve attributes
    n_frm_end = hdf5.n_frm_end
    n_frm_st = hdf5.n_frm_st
    metadata = hdf5.metadata
    if 'SYMBOL_LEN' in metadata: # to support older datasets
        samps_per_slot = int(metadata['SYMBOL_LEN'])
    elif 'SLOT_SAMP_LEN' in metadata:
        samps_per_slot = int(metadata['SLOT_SAMP_LEN'])
    num_pilots = int(metadata['PILOT_NUM'])
    num_cl = int(metadata['CL_NUM'])
    prefix_len = int(metadata['PREFIX_LEN'])
    postfix_len = int(metadata['POSTFIX_LEN'])
    z_padding = prefix_len + postfix_len
    if offset < 0: # if no offset is given use prefix from HDF5
        offset = int(prefix_len)
    fft_size = int(metadata['FFT_SIZE'])
    cp = int(metadata['CP_LEN'])
    rate = int(metadata['RATE'])
    pilot_type = metadata['PILOT_SEQ_TYPE'].astype(str)[0]
    nonzero_sc_size = fft_size
    if 'DATA_SUBCARRIER_NUM' in metadata:
        nonzero_sc_size = metadata['DATA_SUBCARRIER_NUM']
    ofdm_pilot_f = np.array(metadata['OFDM_PILOT_F'])
    ofdm_len = fft_size + cp
    symbol_per_slot = (samps_per_slot - z_padding) // ofdm_len
    if 'UL_SYMS' in metadata:
        ul_slot_num = int(metadata['UL_SYMS'])
    elif 'UL_SLOTS' in metadata:
        ul_slot_num = int(metadata['UL_SLOTS'])
    slot_num = int(metadata['BS_FRAME_LEN'])
    timestep = samps_per_slot*slot_num/rate
    print("samps_per_slot = {}, offset = {}, cp = {}, prefix_len = {}, postfix_len = {}, z_padding = {}, pilot_rep = {}, timestep = {}".format(samps_per_slot, offset, cp, prefix_len, postfix_len, z_padding, symbol_per_slot, timestep))

    pilot_data_avail = len(hdf5.pilot_samples) > 0
    ul_data_avail = len(hdf5.uplink_samples) > 0

    chunk_size = 10000
    ue_frame_offset = [0]*num_cl

    if pilot_data_avail:
        pilot_samples = hdf5.pilot_samples[:, cell_i, :, :, :]
        num_bs_ants = pilot_samples.shape[2]
        print("Number of antennas in dataset %d"%num_bs_ants)
        all_bs_nodes = set(range(num_bs_ants))
        plot_bs_nodes = list(all_bs_nodes - set(exclude_bs_nodes))
        pilot_samples = pilot_samples[:, :, plot_bs_nodes, :]

        # Verify frame_i does not exceed max number of collected frames
        ref_frame = min(frame_i, pilot_samples.shape[0])

        # pilot_samples dimensions:
        # ( #frames, #cells, #pilot subframes or cl ant sending pilots, #bs nodes or # bs ant, #samps per frame * 2 for IQ )
        num_frames = pilot_samples.shape[0]
        num_ues = pilot_samples.shape[1]
        num_ants = pilot_samples.shape[2]
        print("num_frames %d, num_ues %d, num_ants %d"%(num_frames, num_ues, num_ants))
        print(pilot_samples.shape)

        samps_mat = np.reshape(
                pilot_samples, (num_frames, num_ues, num_ants, samps_per_slot, 2))
        samps = (samps_mat[:, :, :, :, 0] +
                samps_mat[:, :, :, :, 1]*1j)*2**-15
        del samps_mat
        gc.collect
        # Correlation (Debug plot useful for checking sync)
        good_ants = []
        insp_ants = [] # antennas to be inspected
        if ant_i > num_ants - 1:
            insp_ants = range(num_ants)
        else:
            insp_ants = [ant_i]
        for i in insp_ants:
            amps = np.mean(np.abs(samps[:, user_i, i, :]), axis=1)
            pilot_frames = [i for i in range(len(amps)) if amps[i] > thresh]
            if len(pilot_frames) > 0:
                good_ants = good_ants + [i]
            else:
                print("no valid frames where found in antenna %d. Decision threshold (average pilot amplitude) was %f" % (i, thresh))
        if len(good_ants) == 0:
            print("no valid frames found in data belonging to user %d. Exitting ..." % user_i)
            return

        # Find the frame number at which each UE starts sending pilots+data
        for u in range(num_ues):
            amps = np.mean(np.abs(samps[:, u, ant_i, :]), axis=1)
            for i in range(1, len(amps)):
                if amps[i] > thresh and amps[i-1] < thresh:
                    ue_frame_offset[u] = i
                    break
            #pilot_frames[u] = [i for i in range(1, len(amps)) if amps[i] > thresh and amps[i-1] < thresh]
        print("Starting frame offset for each UE:")
        print(ue_frame_offset)


        # Plotter
        # Plot pilots
        # Compute CSI from IQ samples
        # Samps: #Frames, #Users, #Antennas, #Samples
        # CSI:   #Frames, #Users, #Pilot Rep, #Antennas, #Subcarrier
        # For correlation use a fft size of 64
        print("*verify_hdf5(): Calling samps2csi with fft_size = {}, offset = {}, bound = {}, cp = {} *".format(fft_size, offset, z_padding, cp))
        csi, _ = hdf5_lib.samps2csi_large(pilot_samples, num_pilots, chunk_size, samps_per_slot, fft_size=fft_size,
                                        offset=offset, bound=z_padding, cp=cp, pilot_f=ofdm_pilot_f)

        if corr_thresh > 0.0:
            bad_nodes = find_bad_nodes(csi, corr_thresh=corr_thresh,
                                       user=user_i)
            if bad_nodes:
                print(">>> Warning! List of bad nodes (1-based): {bad_nodes}".
                      format(bad_nodes=bad_nodes))
            else:
                print(">>> All Iris nodes are good!")

        if ofdm_sym_i >= symbol_per_slot:  # if out of range index, do average
            userCSI = np.mean(csi, 2)
        else:
            userCSI = csi[:, :, ofdm_sym_i, :, :]

        for i in insp_ants:
            user_amps = np.mean(np.abs(samps[:, :, i, :]), axis=2)
            plot_iq_samps(samps, user_amps, n_frm_st, ref_frame, [user_i], [i])
        csi_to_plot = userCSI
        plot_csi(csi_to_plot, plot_bs_nodes, pilot_frames, ref_frame, ant_i, subcarrier_i, offset)

    # Plot UL data symbols
    if ul_data_avail > 0:
        # UL Samps: #Frames, #Uplink Symbol, #Antennas, #Samples
        uplink_samples = hdf5.uplink_samples[:, cell_i, :, :, :]
        all_bs_nodes = set(range(hdf5.uplink_samples.shape[3]))
        plot_bs_nodes = list(all_bs_nodes - set(exclude_bs_nodes))
        uplink_samples = uplink_samples[:, :, plot_bs_nodes, :]
        ref_frame = frame_i #min(frame_i - n_frm_st, uplink_samples.shape[0])
        samps_mat = np.reshape(
                uplink_samples, (uplink_samples.shape[0], uplink_samples.shape[1], uplink_samples.shape[2], samps_per_slot, 2))
        ul_samps = (samps_mat[:, :, :, :, 0] +
                samps_mat[:, :, :, :, 1]*1j)*2**-15

        user_amps = np.mean(np.abs(ul_samps[:, :, ant_i, :]), axis=2)
        plot_iq_samps(ul_samps, user_amps, n_frm_st, ref_frame, [ul_slot_i], [ant_i], data_str="Uplink Data")

        if demod=='zf' or demod=='conj' or demod=='mmse':
            noise_f = None
            tx_data = hdf5_lib.load_tx_data(metadata, hdf5.dirpath, userCSI.shape[1])
            equalized_symbols, demod_symbols, tx_symbols, slot_evm, slot_evm_snr, slot_ser = \
                hdf5_lib.demodulate(ul_samps, userCSI, tx_data, metadata, ue_frame_offset, offset, ul_slot_i, noise_f, demod)
            plot_constellation_stats(slot_evm, slot_evm_snr, slot_ser, equalized_symbols, tx_symbols, ref_frame, ul_slot_i)

    plt.show()

def main():
    # Tested with inputs: ./data_in/Argos-2019-3-11-11-45-17_1x8x2.hdf5 300  (for two users)
    #                     ./data_in/Argos-2019-3-30-12-20-50_1x8x1.hdf5 300  (for one user) 
    parser = OptionParser()
    parser.add_option("--show-metadata", action="store_true", dest="show_metadata", help="Displays hdf5 metadata", default=False)
    parser.add_option("--deep-inspect", action="store_true", dest="deep_inspect", help="Run script without analysis", default=False)
    parser.add_option("--demodulate", type="string", dest="demodulate", help="Demodulate method for uplink data", default="zf")
    parser.add_option("--ref-frame", type="int", dest="ref_frame", help="Frame number to plot", default=1000)
    parser.add_option("--ref-ul-slot", type="int", dest="ref_ul_slot", help="UL slot number to plot", default=0)
    parser.add_option("--ref-dl-slot", type="int", dest="ref_dl_slot", help="DL slot number to plot", default=0)
    parser.add_option("--ref-cell", type="int", dest="ref_cell", help="Cell number to plot", default=0)
    parser.add_option("--ref-ant", type="int", dest="ref_ant", help="Reference antenna", default=0)
    parser.add_option("--ants", type="string", dest="bs_nodes", help="Bs antennas to be included in plotting", default="")
    parser.add_option("--exclude-bs-ants", type="string", dest="exclude_bs_nodes", help="Bs antennas to be excluded in plotting", default="")
    parser.add_option("--ref-ofdm-sym", type="int", dest="ref_ofdm_sym", help="Reference ofdm symbol within a pilot", default=0)
    parser.add_option("--ref-user", type="int", dest="ref_user", help="Reference User", default=0)
    parser.add_option("--ref-subcarrier", type="int", dest="ref_subcarrier", help="Reference subcarrier", default=0)
    parser.add_option("--signal-offset", type="int", dest="signal_offset", help="signal offset from the start of the time-domain symbols", default=-1)
    parser.add_option("--n-frames", type="int", dest="n_frames", help="Number of frames to inspect", default=2000)
    parser.add_option("--sub-sample", type="int", dest="sub_sample", help="Sub sample rate", default=1)
    parser.add_option("--thresh", type="float", dest="thresh", help="Ampiltude Threshold for valid frames", default=0.001)
    parser.add_option("--frame-start", type="int", dest="fr_strt", help="Starting frame. Must have set n_frames first and make sure fr_strt is within boundaries ", default=0)
    parser.add_option("--verify-trace", action="store_true", dest="verify", help="Run script without analysis", default=True)
    parser.add_option("--corr-thresh", type="float", dest="corr_thresh",
                      help="Correlation threshold to exclude bad nodes",
                      default=0.00)
    (options, args) = parser.parse_args()

    show_metadata = options.show_metadata
    deep_inspect = options.deep_inspect
    demodulate = options.demodulate
    n_frames = options.n_frames
    ref_frame = options.ref_frame
    ref_cell = options.ref_cell
    ref_ofdm_sym = options.ref_ofdm_sym
    ref_ant = options.ref_ant
    ref_user = options.ref_user
    ref_subcarrier = options.ref_subcarrier
    ref_ul_slot = options.ref_ul_slot
    ref_dl_slot = options.ref_dl_slot
    signal_offset = options.signal_offset
    thresh = options.thresh
    fr_strt = options.fr_strt
    verify = options.verify
    sub_sample = options.sub_sample
    corr_thresh = options.corr_thresh
    bs_nodes_str = options.bs_nodes
    exclude_bs_nodes_str = options.exclude_bs_nodes

    filename = sys.argv[1]
    scrpt_strt = time.time()

    if n_frames == 0:
        print("WARNING: No frames_to_inspect given. Will process the whole dataset.") 

    if (ref_frame > n_frames):
        print("WARNING: Attempted to inspect a frame at an index larger than the no. of requested frames +  or at an index smaller than the required start of the frames: ref_frame:{} > n_frames:{} or ref_frame:{} <  fr_strt:{}. ".format(
                ref_frame, n_frames, ref_frame, fr_strt))
        print("Setting the frame to inspect/plot to {}".format(fr_strt))
        ref_frame = 0

    print(">> frame to plot = {}, ref. ant = {}, ref. user = {}, ref ofdm_sym = {}, no. of frames to inspect = {}, starting frame = {} <<".format(ref_frame, ref_ant, ref_user, ref_ofdm_sym, n_frames, fr_strt))

    # Instantiate
    hdf5 = hdf5_lib(filename, n_frames, fr_strt, sub_sample)
    pilot_samples = hdf5.pilot_samples
    uplink_samples = hdf5.uplink_samples

    # Check which data we have available
    pilots_avail = len(pilot_samples) > 0
    ul_data_avail = len(uplink_samples) > 0
    exclude_bs_nodes = []
    if pilots_avail:
        num_bs_ants = pilot_samples.shape[4]
        if len(bs_nodes_str) > 0:
            ant_ids = bs_nodes_str.split(',')
            bs_nodes = [int(i) for i in ant_ids]
            exclude_bs_nodes = list(set(range(num_bs_ants)) - set(bs_nodes))
        else:
            exclude_bs_nodes = []
            if len(exclude_bs_nodes_str) > 0:
                exclude_ant_ids = exclude_bs_nodes_str.split(',')
                exclude_bs_nodes = [int(i) for i in exclude_ant_ids]
        print("HDF5 pilot data size:")
        print(pilot_samples.shape)
    if ul_data_avail:
        print("HDF5 uplink data size:")
        print(uplink_samples.shape)

    if show_metadata:
        print(hdf5.metadata)
    else:
        verify_hdf5(hdf5, ref_frame, ref_cell, ref_ofdm_sym, ref_ant,
                    ref_user, ref_ul_slot, ref_dl_slot, ref_subcarrier,
                    signal_offset, thresh, deep_inspect,
                    corr_thresh, exclude_bs_nodes, demodulate)
    scrpt_end = time.time()
    print(">>>> Script Duration: time: %f \n" % ( scrpt_end - scrpt_strt) )


if __name__ == '__main__':
    main()
