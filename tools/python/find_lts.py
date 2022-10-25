#!/usr/bin/python3
"""
 find_lts.py
 Find LTS sequence
---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import numpy as np
from generate_sequence import *
import matplotlib
import matplotlib.pyplot as plt
import scipy.io as sio  # For .mat format


def find_zc_pilot(iq, thresh=0.8, us=1, seq_length=512, cp=32, pilot_seq=[]):
    debug = False
    pilot_seq = np.asarray(pilot_seq)
    if pilot_seq.size == 0:
        pilot, _ = generate_training_seq(preamble_type='zadoff-chu', seq_length=seq_length, cp=cp, upsample=us, reps=[])
        fft_length = int(np.power(2, np.ceil(np.log2(seq_length))))
        peak_spacing = fft_length + cp
    else:
        pilot = pilot_seq
        peak_spacing = pilot_seq.size

    pilot_flip = pilot[::-1]
    pilot_flip_conj = np.conjugate(pilot_flip)
    sign_fct = iq/abs(iq)			  	# Equivalent to Matlab's sign function (X/abs(X))
    sign_fct = np.nan_to_num(sign_fct)			# Replace NaN values
    pilot_corr = np.abs(np.convolve(pilot_flip_conj, iq))
    pilot_pks = np.where(pilot_corr > (thresh * np.max(pilot_corr)))
    pilot_pks = np.squeeze(pilot_pks)

    if not pilot_pks.any():
        if debug:
            print("NO PILOT FOUND!")
        best_pk = []
    else:
        # best_pk = pilot_pks[second_peak_idx[0]]  # Grab only the first packet we have received
        best_pk = np.argmax(pilot_corr)

    if debug:
        fig = plt.figure()
        ax1 = fig.add_subplot(2, 1, 1)
        ax1.grid(True)
        ax1.plot(np.abs(iq))
        ax2 = fig.add_subplot(2, 1, 2)
        ax2.grid(True)
        ax2.plot(np.abs(pilot_corr))
        #ax2.scatter(pilot_pks, 2 * np.ones(len(pilot_pks)))
        plt.show()
    return best_pk, pilot_pks, pilot_corr


def find_lts(iq, thresh=0.8, us=1, cp=32, flip=False, lts_seq=[]):
	"""
		Find the indices of LTSs in the input "iq" signal (upsampled by a factor of "up").
		"thresh" sets sensitivity.
		Inputs:
			iq: IQ samples
			thresh: threshold to detect peak
			us: upsampling factor, needed for generate_training_seq() function
			cp: cyclic prefix
			flip: Flag to specify order or LTS sequence.
			lts_seq: if transmitted lts sequence is provided, use it, otherwise generate it
		Returns:
			best_pk: highest LTS peak,
			lts_pks: the list of all detected LTSs, and
			lts_corr: the correlated signal, multiplied by itself delayed by 1/2 an LTS
	"""
	debug = False

	# Ignore warnings
	np.seterr(divide='ignore', invalid='ignore')

	# If original signal not provided, generate LTS
	lts_seq = np.asarray(lts_seq)
	if lts_seq.size == 0:
		# full lts contains 2.5 64-sample-LTS sequences, we need only one symbol
		lts, lts_f = generate_training_seq(preamble_type='lts', cp=cp, upsample=us)
		peak_spacing = 64
	else:
		# If provided...
		lts = lts_seq
		peak_spacing = 80

		# Deprecated
		#if len(lts_seq) == 80:
		#	peak_spacing = 80
		#else:
		#	peak_spacing = 64

	lts_tmp = lts[-64:]
	if flip:
		lts_flip = lts_tmp[::-1]
	else:
		lts_flip = lts_tmp

	lts_flip_conj = np.conjugate(lts_flip)
	sign_fct = iq/abs(iq)									  	# Equivalent to Matlab's sign function (X/abs(X))
	sign_fct = np.nan_to_num(sign_fct)							# Replace NaN values
	lts_corr = np.abs(np.convolve(lts_flip_conj, sign_fct))

	lts_pks = np.where(lts_corr[:len(iq)] > (thresh * np.max(lts_corr)))
	lts_pks = np.squeeze(lts_pks)
	x_vec, y_vec = np.meshgrid(lts_pks, lts_pks)

	# second_peak_idx, y = np.where((y_vec - x_vec) == len(lts_tmp))
	second_peak_idx, y = np.where((y_vec - x_vec) == peak_spacing)

	# To save mat files
	# sio.savemat('rx_iq_pilot.mat', {'iq_pilot': iq})

	if not second_peak_idx.any():
		if debug:
			print("NO LTS FOUND!")
		best_pk = []
	else:
		best_pk = lts_pks[second_peak_idx[0]]  # Grab only the first packet we have received

	if debug:
		# print("LTS: {}, BEST: {}".format(lts_pks, lts_pks[second_peak_idx]))
		if lts_pks.size > 1:
			fig = plt.figure()
			ax1 = fig.add_subplot(2, 1, 1)
			ax1.grid(True)
			ax1.plot(np.abs(iq))
			ax2 = fig.add_subplot(2, 1, 2)
			ax2.grid(True)
			ax2.stem(np.abs(lts_corr))
			ax2.scatter(lts_pks, 2 * np.ones(len(lts_pks)))
			plt.show()

	return best_pk, lts_pks, lts_corr

def pilot_finder(samples, pilot_type, flip=False, pilot_seq=[], seq_length=64,cp=0):
    """
    Find pilots from clients to each of the base station antennas
    Input:
        samples    - Raw samples from pilots and data.
                     Dimensions: vector [1 x num samples]
        pilot_type - Type of TX pilot (e.g., 802.11 LTS)
        flip       - Needed for finding LTS function
    Output:
        pilot     - Received pilot (from multiple clients)
        tx_pilot  - Transmitted pilot (same pilot sent by all clients)
    """

    if pilot_type.find('lts') != -1:
        # LTS-based pilot
        lts_thresh = 0.8
        best_pk, pilot_pks, pilot_corr = find_lts(samples, thresh=lts_thresh, flip=flip, lts_seq=pilot_seq)

        # full lts contains 2.5 64-sample-LTS sequences, we need only one symbol
        lts, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)

        if not (pilot_seq.size == 0):
            # pilot provided, overwrite the one returned above
            lts = pilot_seq

        pilot_thresh = lts_thresh * np.max(pilot_corr)
        # We'll need the transmitted version of the pilot (for channel estimation, for example)
        tx_pilot = [lts, lts_f]

    elif pilot_type.find('zadoff-chu') != -1:
        best_pk, pilot_pks, pilot_corr = find_zc_pilot(samples, seq_length=seq_length, cp=cp, pilot_seq=pilot_seq)
        pilot, pilot_f = generate_training_seq(preamble_type='zadoff-chu', seq_length=seq_length, cp=cp, upsample=1, reps=1)
        tx_pilot = [pilot, pilot_f]
        pilot_thresh = 0.8 * np.max(pilot_corr)

    else:
        raise Exception("Only LTS & Zadoff-Chu Pilots are currently supported!")

    return tx_pilot, pilot_pks, pilot_corr, pilot_thresh, best_pk


if __name__ == '__main__':

	#lts, lts_f = generate_training_seq(preamble_type='lts', cp=32, upsample=1)
	#find_lts(lts)
	zcpilot,_ = generate_training_seq(preamble_type='zadoff-chu', seq_length=336, cp=0, upsample=1, reps=1)
	find_zc_pilot(iq=zcpilot,seq_length=336,cp=0)