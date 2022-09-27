#!/usr/bin/python3
# -*- coding: UTF-8 -*-
"""
 generate_sequence.py
 Library for generating different training sequences
---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import os
import sys
import math
import matplotlib.pyplot as plt
import numpy as np


def generate_training_seq(preamble_type='lts', seq_length=0, cp=32, upsample=1, reps=10):
	"""
	Generate any of the following preambles/training sequences.
	802.11 STS, 802.11 LTS, Zadoff-Chu, and Gold Sequence (ifft)
	ARGS:
		preamble_type: sequence name
		seq_length: length of sequence, doesn't apply to all types
		cp: cyclic prefix
		upsample: upsampling factor
		reps: number of symbol repetition. Specific to STS
	RETURNS:
		- For STS, LTS, and Zadoff-Chu: Training Sequence
		- For Gold Sequences: Matrix where each row is an individual preamble.
			Row are ordered such that top row has the highest auto-correlation
			highest_peak_to_second_ratio, and corresponding index
	** MORE INFO ON EACH SEQUENCE TYPE **
	STS:
		Generate a time-domain 802.11 STS with:
		#reps repetitions of one STS symbol
		All other input arguments are ignored
		Example:
			generate_training_seq(preamble_type='sts', seq_length=[], cp=[], upsample=[], reps=10)
	LTS:
		Generate a time-domain 802.11 LTS with:
		Cyclic prefix of "cp" (32)
		Upsampled by a factor of "upsample" (1)
		Seq_length is ignored for LTS
		Reps is ignored for STS
		Example:
			generate_training_seq(preamble_type='lts', seq_length=[], cp=32, upsample=1, reps=[])
	lte_zadoffchu_seq:
		Generate a 3GPP-based Zadoff-Chu sequence of complex symbols.
		SEQ = LTEZADOFFCHUSEQ(R, N) generates a Zadoff-Chu sequence
		of length N as per LTE specifications. The output SEQ is an N-length
		column vector of complex symbols.
		Example:
			Generate the 25th root length-63 Zadoff-Chu sequence
			seq = lteZadoffChuSeq(25, 63);
		Reference:
		3rd Generation Partnership Project, Technical Specification Group Radio
		Access Network, Evolved Universal Terrestrial Radio Access (E-UTRA),
		Physical channels and modulation, Release 10, 3GPP TS 36.211, v10.0.0,
		2010-12.
		See also comm.PNSequence.
		Copyright 2012 The MathWorks, Inc.
		$Revision: 1.1.6.1 $ $Date: 2012/03/13 07:13:00 $
	gold_ifft:
		Generates a Gold sequence and constructs a preamble with it
		Example:
			generate_training_seq(preamble_type='gold_ifft', seq_length=128, cp=0, upsample=1)
	"""
	preamble_type = preamble_type.lower()
	highest_peak_to_second_ratio = []

	if preamble_type == 'sts':
		# STS symbols for the preamble
		use_802_11_structure = 1

		if use_802_11_structure:
			# 802.11-2012 STRUCTURE
			sts_f_low = np.array(
				[0, 0, 0, 0, 0, 0, 0, 0, 1 + 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, -1 - 1j,
				 0, 0, 0, 1 + 1j, 0, 0, 0, 0])
			sts_f_up = np.array(
				[0, 0, 0, -1 - 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, 1 + 1j,
				 0, 0, 0, 0, 0, 0, 0])
			sts_f = np.sqrt(13 / 6) * np.concatenate(
				(sts_f_low, sts_f_up))  # Norm. avg. power (12 out of 52 subcarriers)
			sts_t = np.fft.ifft(np.fft.ifftshift(sts_f))
			sts_t = sts_t[0:16]
			sts_t = np.tile(sts_t, reps).astype(np.complex64)
		else:
			# WARPLAB STRUCTURE
			sts_f_wl = np.zeros(64).astype(np.complex)
			sts_f_wl[0:27] = np.array(
				[0, 0, 0, 0, -1 - 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, 1 + 1j, 0, 0,
				 0, 1 + 1j, 0, 0])
			sts_f_wl[38:64] = np.array(
				[0, 0, 1 + 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, 1 + 1j, 0, 0, 0, -1 - 1j, 0, 0, 0, -1 - 1j, 0, 0, 0,
				 1 + 1j, 0, 0, 0])
			sts_t_wl = np.fft.ifft(
				np.fft.ifftshift(sts_f_wl * np.sqrt(13 / 6)))  # Norm. avg. power (12 out of 52 subcarriers)
			sts_t_wl = sts_t_wl[0:16]
			sts_t_wl = np.tile(sts_t_wl, reps).astype(np.complex64)
			sts_t = sts_t_wl

		return sts_t

	elif preamble_type == 'lts':
		# Generate 802.11 LTS preamble
		lts_freq = np.array([
			0, 0, 0, 0, 0, 0, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1, 1, 1, 1, 0,
			1, -1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1, -1, -1, 1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1, 1, 0, 0, 0, 0, 0])
		up_zeros = np.zeros(len(lts_freq) // 2 * (upsample - 1))
		lts_freq_up = np.concatenate((up_zeros, lts_freq, up_zeros))
		signal = np.fft.ifft(np.fft.ifftshift(lts_freq_up))
		# signal = signal / np.absolute(signal).max()  # normalize - move... do it later

		# Now affix the cyclic prefix
		sequence = np.concatenate((signal[len(signal) - cp:], signal, signal))  # could use tile...
		return sequence, lts_freq

	elif preamble_type == "zadoff-chu":
                u = 1
                v = 0

                prime = np.array([ 2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43,
                    47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 101, 103, 107, 109, 113,
                    127, 131, 137, 139, 149, 151, 157, 163, 167, 173, 179, 181, 191,
                    193, 197, 199, 211, 223, 227, 229, 233, 239, 241, 251, 257, 263,
                    269, 271, 277, 281, 283, 293, 307, 311, 313, 317, 331, 337, 347,
                    349, 353, 359, 367, 373, 379, 383, 389, 397, 401, 409, 419, 421,
                    431, 433, 439, 443, 449, 457, 461, 463, 467, 479, 487, 491, 499,
                    503, 509, 521, 523, 541, 547, 557, 563, 569, 571, 577, 587, 593,
                    599, 601, 607, 613, 617, 619, 631, 641, 643, 647, 653, 659, 661,
                    673, 677, 683, 691, 701, 709, 719, 727, 733, 739, 743, 751, 757,
                    761, 769, 773, 787, 797, 809, 811, 821, 823, 827, 829, 839, 853,
                    857, 859, 863, 877, 881, 883, 887, 907, 911, 919, 929, 937, 941,
                    947, 953, 967, 971, 977, 983, 991, 997, 1009, 1013, 1019, 1021,
                    1031, 1033, 1039, 1049, 1051, 1061, 1063, 1069, 1087, 1091, 1093,
                    1097, 1103, 1109, 1117, 1123, 1129, 1151, 1153, 1163, 1171, 1181,
                    1187, 1193, 1201, 1213, 1217, 1223, 1229, 1231, 1237, 1249, 1259,
                    1277, 1279, 1283, 1289, 1291, 1297, 1301, 1303, 1307, 1319, 1321,
                    1327, 1361, 1367, 1373, 1381, 1399, 1409, 1423, 1427, 1429, 1433,
                    1439, 1447, 1451, 1453, 1459, 1471, 1481, 1483, 1487, 1489, 1493,
                    1499, 1511, 1523, 1531, 1543, 1549, 1553, 1559, 1567, 1571, 1579,
                    1583, 1597, 1601, 1607, 1609, 1613, 1619, 1621, 1627, 1637, 1657,
                    1663, 1667, 1669, 1693, 1697, 1699, 1709, 1721, 1723, 1733, 1741,
                    1747, 1753, 1759, 1777, 1783, 1787, 1789, 1801, 1811, 1823, 1831,
                    1847, 1861, 1867, 1871, 1873, 1877, 1879, 1889, 1901, 1907, 1913,
                    1931, 1933, 1949, 1951, 1973, 1979, 1987, 1993, 1997, 1999, 2003,
                    2011, 2017, 2027, 2029, 2039]) # array of primes < 2038 of size 309
                M = prime[308]
                for i in range(308):
                    if prime[i] < seq_length and prime[i + 1] > seq_length:
                        M = prime[i]
                        break
                qh = M * (u + 1) / 31
                q = np.floor(qh + 0.5) + v * np.power(-1, np.floor(2 * qh))

		# m = (0:N-1).'
                m = np.mod(np.arange(seq_length).transpose(), M)
                p = np.exp(-1j * np.pi * q * m * (m + 1) / M)
                fft_length = int(np.power(2, np.ceil(np.log2(seq_length))))
                start_index = int((fft_length - seq_length) // 2)
                stop_index = int(start_index + seq_length)
                zadoff_freq = np.zeros(fft_length, dtype=complex);
                zadoff_freq[start_index:stop_index] = p;

                up_zeros = np.zeros(len(zadoff_freq) // 2 * (upsample - 1))
                zadoff_freq_up = np.concatenate((up_zeros, zadoff_freq, up_zeros))
                signal = np.fft.ifft(zadoff_freq_up)
                # signal = signal / np.absolute(signal).max()  # normalize - move... do it later

                # Now affix the cyclic prefix
                sequence = np.concatenate((signal[len(signal) - cp:], signal))
                return sequence, zadoff_freq

	elif preamble_type == 'gold_ifft':
		# Generate IFFT GoldCode sequences
		preambles = np.empty((seq_length, 2 * seq_length * upsample + cp), dtype='complex64')
		integ = np.round(np.log2(seq_length))
		if 2 ** integ - seq_length == 0:
			for i in range(seq_length):
				thisP = preamble_generator(int(round(np.log2(seq_length))), i, cp, 0, upsample=upsample)
				thisP = (1 - 2 ** -15) * thisP / np.amax(abs(thisP))
				p_short = thisP[cp + seq_length:]
				auto_corr = np.correlate(p_short, p_short, mode="full")
				idx = abs(auto_corr).argsort()
				auto_corr = auto_corr[idx]
				auto_corr = auto_corr[::-1]
				abs_p = np.square(abs(p_short))
				PAPR = (max(abs_p) / (sum(abs_p) / len(abs_p)))
				# print (float((sum(abs_p)/len(abs_p))))  #error seems to be that the 108th term seems to be truncated   32 bit vs 64 bit precision
				highest_peak_to_second_ratio.append(PAPR)
				preambles[i, :] = thisP
		else:
			print('not here')

		# Order them according to highest autocorrelation Peak
		highest_peak_to_second_ratio_idx = np.argsort(highest_peak_to_second_ratio)
		# temporary fix needed if indexing is wrong due to precision error
		# if seq_length == 128 and preamble_type == 'gold_ifft':
		#    y = highest_peak_to_second_ratio_idx[-2]
		#    highest_peak_to_second_ratio_idx[-2] = highest_peak_to_second_ratio_idx[-3]
		#    highest_peak_to_second_ratio_idx[-3] = y
		preambles = (1 - 2 ** -15) * preambles[highest_peak_to_second_ratio_idx]
		return preambles

	else:
		raise Exception("Preamble sequence not supported")


def preamble_generator(N, index=0, CP=0, bpsk=0, shift=0, upsample=1):
	# example usage preamble = preamble_generator(7, 1, 0);
	numsyms = 2 ** N
	seq1 = read_precomp_code(N, index)

	quadseq = seq1 + seq1 * 1j  # you can shift the imaginary component...
	quadseq = np.concatenate(
		[quadseq[0:int(np.ceil(len(quadseq)) / 2)], [0], quadseq[int(np.ceil(len(quadseq)) / 2):len(quadseq)]])

	# Enable second line for BPSK
	if bpsk:
		#sequence = np.concatenate((quadseq, quadseq), axis=1).transpose()
		sequence = np.concatenate((quadseq, quadseq)).transpose()
	# todo: enable upsampling here
	else:
		symseq = np.zeros(2 * numsyms, dtype=complex)
		symseq[0:2 * (numsyms) - 1:2] = quadseq
		up_zeros = np.zeros(len(symseq) // 2 * (upsample - 1))
		symseq_up = np.concatenate((up_zeros, symseq, up_zeros))
		sequence = np.fft.ifft((np.fft.ifftshift(symseq_up)))

	if CP:
		# Notice that we are adding a cyclic prefix of 1/4 of the signature txlen
		if CP == 1:
			# legacy use -- this is way too long of a preamble for N > 5
			sequence = np.concatenate([sequence[3 / 4 * (numsyms) * 2:(numsyms) * 2], sequence])
		else:
			sequence = np.concatenate([sequence[len(sequence) - CP: len(sequence)], sequence])
	scale = max(max(abs(sequence.real)), max(abs(sequence.imag)))
	preamble = sequence * (1 - 2 ** -15) / scale

	return preamble


def read_precomp_code(N, index=0):
	# This code generates gold or kasami sequences of length 2^N.
	numsyms = 2 ** N - 1
	path = os.path.dirname(os.path.realpath(__file__))

	if numsyms == 255 or numsyms == 63:
		filename = path + '/codebooks/kasamilarge-' + str(numsyms)
		# sequence = readFileSignature( filename, index );
		File = open(filename, 'r')
		# d = textread(filename, '%d')
		d = [list(map(int, line.split())) for line in File]
		d = np.array(d)
		out = d
		out = d[index, :]
		File.close()
		if 0:
			seq1 = d[index, :]
			seq2 = np.roll(seq1, (numsyms + 1) / 2 + 1)
			quadseq = [0] + seq1 + sqrt(-1) * seq2
			symseq = np.zeros(2 * (numsyms + 1), 1)
			symseq[1:2 * (numsyms + 1) - 1:2] = quadseq
			sequence = np.fft.ifft(symseq)
			# Notice that we are adding a cyclic prefix of 1/5 of the signature txlen
			sequence = np.concatonate(sequence[3 / 4 * (numsyms + 1) * 2 + 1:(numsyms + 1) * 2].transpose(),
									  sequence.transpose(), axis=1).transpose()  # .' transpose (' conj transpose)
			offset = (numsyms + 1) * 2 - 3 / 4 * (numsyms + 1) * 2 + 1

	elif numsyms == 127 or numsyms == 511:
		filename = path + '/codebooks/gold-' + str(numsyms)
		# sequence = readFileSignature( filename, index );
		File = open(filename, 'r')
		d = [list(map(int, line.split())) for line in File]
		d = np.array(d)
		out = d[index, :]
		File.close()
		if 0:
			seq1 = d[index, :]
			seq2 = np.roll(seq1, (numsyms + 1) / 2 + 1)
			quadseq = [0] + seq1 + sqrt(-1) * seq2
			symseq = np.zeros(2 * (numsyms + 1), 1)
			symseq[1:2 * (numsyms + 1) - 1:2] = quadseq
			sequence = np.fft.ifft(symseq)
			# Notice that we are adding a cyclic prefix of 1/5 of the signature txlen
			sequence = np.concatonate(sequence[3 / 4 * (numsyms + 1) * 2 + 1:(numsyms + 1) * 2].transpose(),
									  sequence.transpose(), axis=1).transpose()  # .' transpose (' conj transpose)
			offset = (numsyms + 1) * 2 - 3 / 4 * (numsyms + 1) * 2 + 1
			len = numsyms + 1

	else:
		print('Length not supported')

	return out


if __name__ == '__main__':

	"""
	Example on how to generate the different sequences
	"""
	sequence_sts = generate_training_seq('sts', reps=10)
	sequence_lts, lts_f = generate_training_seq('lts', cp=32, upsample=1)
	sequence_zadoff = generate_training_seq('lte_zadoffchu_seq', seq_length=63, root=25)
	sequence_goldIfft = generate_training_seq('gold_ifft', seq_length=128, cp=0, upsample=1)

	print("SIZE GOLD SEQ.: {},{}".format(len(sequence_goldIfft), len(sequence_goldIfft[0])))

	l_re = list(np.real(sequence_goldIfft[0][0:128]))
	l_im = list(np.imag(sequence_goldIfft[0][0:128]))

	with open('./test_re.txt', 'w') as f:
		for idx, item in enumerate(l_re):
			f.write("%s, " % item)
			if ((idx+1) % 5) == 0:
				f.write("\n")
	with open('./test_im.txt', 'w') as f:
		for idx, item in enumerate(l_im):
			f.write("%s, " % item)
			if ((idx+1) % 5) == 0:
				f.write("\n")

	plt.figure()
	plt.subplot(4, 1, 1)
	plt.plot(np.abs(sequence_sts))
	plt.subplot(4, 1, 2)
	plt.plot(np.abs(sequence_lts))
	plt.subplot(4, 1, 3)
	plt.plot(np.real(sequence_zadoff))
	plt.subplot(4, 1, 4)
	plt.plot(np.abs(sequence_goldIfft[0]))
	plt.show()