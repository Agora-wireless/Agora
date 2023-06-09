#!/usr/bin/python3
"""
 ofdmTxRx.py
 OFDM Modulation/Demodulation library
---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import numpy as np
import numpy.matlib
import random
import time
import datetime
import matplotlib.pyplot as plt


class ofdmTxRx:
    """
    OFDM Library
    """

    def __init__(self):
        self.n_ofdm_syms = 100
        self.mod_order = 4
        self.cp_length = 16

    def bpsk_mod(self, val):
        """
        BPSK Modulation
        ARGS
            - val: Integer value (data symbol) between 0 and 1
        RETURNS:
            - iq: Complex data symbol
        """
        mod_vec = [-1, 1]
        a = mod_vec[val]
        b = 0
        iq = np.complex(a, b)

        iq = 1 / np.sqrt(2) * iq
        return iq

    def qpsk_mod(self, val):
        """
        QPSK Modulation
        ARGS
            - val: Integer value (data symbol) between 0 and 3 (i.e., 0:1:3)
        RETURNS:
            - iq: Complex data symbol
        """
        mod_vec = [-1, 1]
        a = mod_vec[val >> 1]
        b = mod_vec[val % 2]
        iq = np.complex(a, b)

        iq = 1 / np.sqrt(2) * iq
        return iq

    def qam16_mod(self, val):
        """
        16QAM Modulation
        ARGS
            - val: Integer value (data symbol) between 0 and 15 (i.e., 0:1:15)
        RETURNS:
            - iq: Complex data symbol
        """
        mod_vec = [-3, -1, 3, 1]
        a = mod_vec[val >> 2]
        b = mod_vec[val % 4]
        iq = np.complex(a, b)

        iq = 1 / np.sqrt(10) * iq
        return iq

    def qam64_mod(self, val):
        """
        64QAM Modulation
        ARGS
            - val: Integer value (data symbol) between 0 and 63 (i.e., 0:1:63)
        RETURNS:
            - iq: Complex data symbol
        """
        mod_vec = [-7, -5, -1, -3, 7, 5, 1, 3]
        a = mod_vec[val >> 3]
        b = mod_vec[val % 8]
        iq = np.complex(a, b)

        iq = 1 / np.sqrt(43) * iq
        return iq

    def bpsk_dem(self, iq):
        """
        BPSK Demodulation
        ARGS
            - iq: Complex data symbol
        RETURNS:
            - val: Integer value (data symbol) between 0 and 1
        """
        val = np.double(np.real(iq) > 0)
        return val

    def qpsk_dem(self, iq):
        """
        QPSK Demodulation
        ARGS
            - iq: Complex data symbol
        RETURNS:
            - val: Integer value (data symbol) between 0 and 3 (i.e., 0:1:3)
        """
        val = np.double(2 * (np.real(iq) > 0) + 1 * (np.imag(iq) > 0))
        return val

    def qam16_dem(self, iq):
        """
        16QAM Demodulation
        ARGS
            - iq: Complex data symbol
        RETURNS:
            - val: Integer value (data symbol) between 0 and 15 (i.e., 0:1:15)
        """
        thresh = 2 / np.sqrt(10)
        val = (8 * (np.real(iq) > 0)) + \
              (4 * (abs(np.real(iq)) < thresh)) + \
              (2 * (np.imag(iq) > 0)) + \
              (1 * (abs(np.imag(iq)) < thresh))
        return val

    def qam64_dem(self, iq):
        """
        64QAM Demodulation
        ARGS
            - iq: Complex data symbol
        RETURNS:
            - val: Integer value (data symbol) between 0 and 63 (i.e., 0:1:63)
        """
        thresh1 = 2 / np.sqrt(43)
        thresh2 = 4 / np.sqrt(43)
        thresh3 = 6 / np.sqrt(43)
        val = (32 * (np.real(iq) > 0)) + \
              (16 * (abs(np.real(iq)) < thresh2)) + \
              (8 * ((abs(np.real(iq)) < thresh3) and (abs(np.real(iq)) > thresh1))) + \
              (4 * (np.imag(iq) > 0)) + \
              (2 * (abs(np.imag(iq)) < thresh2)) + \
              (1 * ((abs(np.imag(iq)) < thresh3) and (abs(np.imag(iq)) > thresh1)))
        return val

    def generate_data(self, n_ofdm_syms=100, mod_order=4, cp_length=16, datastream=[]):
        """
        Generate random data stream of n_ofdm_syms number of symbols,
        and modulate according to mod_order.
        ARGS:
            - n_ofdm_syms: Number of OFDM symbols
            - mod_order: Modulation Order
                2 - BPSK
                4 - QPSK
                16 - 16QAM
                64 - 64QAM
            - cp_length: Length of cyclic prefix
        RETURNS:
            - signal: Time domain signal after IFFT and added Cyclic Prefix
            - data: Modulated data stream (Constellation)
            - data_i: Random data to be transmitted
            - sc_idx_all: Indexes for both data and pilot subcarriers
        """
        # Data and Pilot Subcarriers
        # data_sc = [1:6, 8:20, 22:26, 38:42, 44:56, 58:63];
        # pilot_sc = [7, 21, 43, 57]
        num_subcarriers = 64
        data_subcarriers = list(range(1, 7)) + list(range(8, 21)) + list(range(22, 27)) + \
                           list(range(38, 43)) + list(range(44, 57)) + list(range(58, 64))
        pilot_subcarriers = [7, 21, 43, 57]
        n_data_syms = n_ofdm_syms * len(data_subcarriers)  # One data sym per data-subcarrier per ofdm symbol

        if not datastream:
            data_i = [random.randint(0, mod_order-1) for i in range(n_data_syms)]  # includes end-points
        else:
            # Prepend datastream to rest of randomly generated data symbols
            data_i_tmp = [random.randint(0, mod_order-1) for i in range(n_data_syms-len(datastream))]
            data_i = np.concatenate((datastream, data_i_tmp))

        data = np.zeros(len(data_i), dtype=complex)

        for x in range(len(data_i)):
            if mod_order == 2:
                data[x] = self.bpsk_mod(data_i[x])
            elif mod_order == 4:
                data[x] = self.qpsk_mod(data_i[x])
            elif mod_order == 16:
                data[x] = self.qam16_mod(data_i[x])
            elif mod_order == 64:
                data[x] = self.qam64_mod(data_i[x])
            else:
                raise Exception("Modulation Order Not Supported. Valid orders: 2/4/16/64 for BPSK, QPSK, 16QAM, 64QAM")

        # Data
        data_matrix = np.reshape(data, (len(data_subcarriers), n_ofdm_syms), order="F")

        # Pilots
        pilots = np.array([1, 1, -1, 1]).reshape(4, 1, order="F")
        pilots_matrix = np.matlib.repmat(pilots, 1, n_ofdm_syms)

        # Full Matrix
        ifft_matrix = np.zeros((num_subcarriers, n_ofdm_syms)).astype(complex)
        ifft_matrix[pilot_subcarriers] = pilots_matrix
        ifft_matrix[data_subcarriers] = data_matrix

        # IFFT
        signal = np.fft.ifft(ifft_matrix, n=num_subcarriers, axis=0)

        # Add Cyclic Prefix
        cp = np.array([])
        if cp_length > 0:
            cp = signal[-cp_length:, :]

        signal = np.vstack([cp, signal]) if cp.size else signal

        # Matrix --> Vector
        signal = np.squeeze(np.reshape(signal, (1, signal.size), order="F"))

        # Subcarriers
        sc_idx_all = [data_subcarriers, pilot_subcarriers]

        return signal, data_matrix, data_i, sc_idx_all, pilots_matrix

    def modulation(self, in_data, mod_order):
        """
        Demodulate data stream of n_ofdm_syms number of symbols, according to mod_order.
        """
        data_i = [int(a) for a in in_data]
        data = np.zeros(len(data_i), dtype='complex64')
        for x in range(len(data_i)):
            if mod_order == 2:
                data[x] = self.bpsk_mod(data_i[x])
            elif mod_order == 4:
                data[x] = self.qpsk_mod(data_i[x])
            elif mod_order == 16:
                data[x] = self.qam16_mod(data_i[x])
            elif mod_order == 64:
                data[x] = self.qam64_mod(data_i[x])
            else:
                raise Exception("Modulation Order Not Supported. Valid orders: 2/4/16/64 for BPSK, QPSK, 16QAM, 64QAM")

        return data

    def demodulation(self, in_data, mod_order):
        """
        Demodulate data stream of n_ofdm_syms number of symbols, according to mod_order.
        """
        data_i = in_data
        data = np.zeros(len(data_i))
        for x in range(len(data_i)):
            if mod_order == 2:
                data[x] = self.bpsk_dem(data_i[x])
            elif mod_order == 4:
                data[x] = self.qpsk_dem(data_i[x])
            elif mod_order == 16:
                data[x] = self.qam16_dem(data_i[x])
            elif mod_order == 64:
                data[x] = self.qam64_dem(data_i[x])
            else:
                raise Exception("Modulation Order Not Supported. Valid orders: 2/4/16/64 for BPSK, QPSK, 16QAM, 64QAM")

        return data

    def cfo_correction(self, rxSignal, lts_start, lts_syms_len, fft_offset):
        """
        Apply Carrier Frequency Offset
        Input:
            rxSignal     - Received IQ Signal
            lts_start    - Sample where LTS begins
            lts_syms_len - Length of LTS Symbol
            fft_offset   - Number of CP samples for FFT
        Output:
            coarse_cfo_est - CFO estimate
        """
        # Get LTS
        lts = rxSignal[lts_start: lts_start + lts_syms_len]

        # Verify number of samples
        if len(lts) != 160:
            coarse_cfo_est = 0
            return coarse_cfo_est

        lts_1 = lts[-64 + -fft_offset + np.array(range(97, 161))]
        lts_2 = lts[-fft_offset + np.array(range(97, 161))]
        # Compute CFO
        tmp = np.unwrap(np.angle(lts_2 * np.conjugate(lts_1)))
        coarse_cfo_est = np.mean(tmp)
        coarse_cfo_est = coarse_cfo_est / (2 * np.pi * 64)
        return coarse_cfo_est

    def sfo_correction(self, rxSig_freq_eq, pilot_sc, pilots_matrix, n_ofdm_syms):
        """
        Apply Sample Frequency Offset
        Input:
            rxSig_freq_eq - Equalized, frequency domain received signal
            pilot_sc      - Pilot subcarriers (indexes)
            pilots_matrix - Pilots in matrix form
            n_ofdm_syms   - Number of OFDM symbols
        Output:
            rxSig_freq_eq - Frequency domain signal after SFO correction
        """
        # Extract pilots and equalize them by their nominal Tx values
        pilot_freq = rxSig_freq_eq[pilot_sc, :]
        pilot_freq_corr = pilot_freq * pilots_matrix
        # Compute phase of every RX pilot
        pilot_phase = np.angle(np.fft.fftshift(pilot_freq_corr))
        pilot_phase_uw = np.unwrap(pilot_phase)
        # Slope of pilot phase vs frequency of OFDM symbol
        pilot_shift = np.fft.fftshift(pilot_sc)
        pilot_shift_diff = np.diff(pilot_shift)
        pilot_shift_diff_mod = np.remainder(pilot_shift_diff, 64).reshape(len(pilot_shift_diff), 1)
        pilot_delta = np.matlib.repmat(pilot_shift_diff_mod, 1, n_ofdm_syms)
        pilot_slope = np.mean(np.diff(pilot_phase_uw, axis=0) / pilot_delta, axis=0)
        # Compute SFO correction phases
        tmp = np.array(range(-32, 32)).reshape(len(range(-32, 32)), 1)
        tmp2 = tmp * pilot_slope
        pilot_phase_sfo_corr = np.fft.fftshift(tmp2, 1)
        pilot_phase_corr = np.exp(-1j * pilot_phase_sfo_corr)
        # Apply correction per symbol
        rxSig_freq_eq = rxSig_freq_eq * pilot_phase_corr
        return rxSig_freq_eq

    def phase_correction(self, rxSig_freq_eq, pilot_sc, pilots_matrix):
        """
        Apply Phase Correction
        Input:
            rxSig_freq_eq - Equalized, time domain received signal
            pilot_sc      - Pilot subcarriers (indexes)
            pilots_matrix - Pilots in matrix form
        Output:
            phase_error   - Computed phase error
        """
        # Extract pilots and equalize them by their nominal Tx values
        pilot_freq = rxSig_freq_eq[pilot_sc, :]
        pilot_freq_corr = pilot_freq * pilots_matrix
        # Calculate phase error for each symbol
        phase_error = np.angle(np.mean(pilot_freq_corr, axis=0))
        return phase_error

    def demult(self, csi, data, noise=None, method='zf'):
        # TODO include cell dimension for both csi and data and symbol num for data
        """csi: Frame, User, Antenna, Subcarrier"""
        """data: Frame, Uplink Syms, Antenna, Subcarrier"""
        # Compute beamweights based on the specified frame.
        demult_start = time.time()
        demul_data = np.empty(
            (csi.shape[0], data.shape[1], csi.shape[1], csi.shape[3]), dtype='complex64')
        bmf_w = np.empty(
            (csi.shape[0], csi.shape[3], csi.shape[2], csi.shape[1]), dtype='complex64')
        data_tp = np.transpose(data, (0, 3, 1, 2))
        # TODO: use multi processing to accelerate code
        for sc in range(csi.shape[3]):
            for frame in range(csi.shape[0]):
                if method == 'zf':
                    bmf_w[frame, sc, :, :] = np.linalg.pinv(csi[frame, :, :, sc])
                elif method == 'mmse' and noise is not None:
                    sigma = np.mean(np.mean(np.power(np.abs(noise[frame, :, :, sc]), 2), axis=0))
                    H = csi[frame, :, :, sc]
                    w_mmse = np.matmul(np.linalg.inv(np.matmul(H, np.transpose(np.conj(H))) + sigma*np.eye(H.shape[0])), H)
                    bmf_w[frame, sc, :, :] = np.transpose(np.conj(w_mmse))
                else:
                    csi_conj = np.conj(csi[frame, :, :, sc])
                    w_scale = 1 / np.sum(np.multiply(csi_conj, csi[frame, :, :, sc]), 1);
                    bmf_w[frame, sc, :, :] = np.transpose(np.matmul(np.diag(w_scale), csi_conj), (1, 0))
        demul_data = np.transpose(np.matmul(data_tp, bmf_w), (0, 2, 3, 1))
        print("demult time: %f"%(time.time() - demult_start))
        return demul_data
    
    def calcBW(self, csi, noise=None, method='zf'):
        bmf_w = np.empty(
            (csi.shape[2], csi.shape[1], csi.shape[0]), dtype='complex64')
        for sc in range(csi.shape[2]):
            if method == 'zf':
                bmf_w[sc, :, :] = np.linalg.pinv(csi[:, :, sc])
            elif method == 'mmse' and noise is not None:
                sigma = np.mean(np.mean(np.power(np.abs(noise[:, :, sc]), 2), axis=0))
                H = csi[:, :, sc]
                w_mmse = np.matmul(H, np.linalg.inv(np.matmul(np.transpose(np.conj(H)), H) + sigma*np.eye(H.shape[1])))
                bmf_w[sc, :, :] = np.conj(w_mmse)
            else:
                csi_conj = np.conj(csi[:, :, sc])
                w_scale = np.sum(np.multiply(csi_conj, csi[:, :, sc]), 1);
                bmf_w[sc, :, :] = np.transpose(np.diag(w_scale) * csi_conj, (1, 0))
        return bmf_w


if __name__ == '__main__':
    """
    Example on how to generate and modulate a data stream consisting of random values
    """
    ofdm_obj = ofdmTxRx()

    n_ofdm_syms = 100
    mod_order = 2
    cp_length = 16
    sig_t_bpsk, data_bpsk, tx_data_1, sc_idx_1, x0 = ofdm_obj.generate_data(n_ofdm_syms, mod_order, cp_length)
    mod_order = 4
    sig_t_qpsk, data_qpsk, tx_data_2, sc_idx_2, x1 = ofdm_obj.generate_data(n_ofdm_syms, mod_order, cp_length)
    mod_order = 16
    sig_t_16qam, data_16qam, tx_data_3, sc_idx_3, x2 = ofdm_obj.generate_data(n_ofdm_syms, mod_order, cp_length)
    mod_order = 64
    sig_t_64qam, data_64qam, tx_data_4, sc_idx_4, x3 = ofdm_obj.generate_data(n_ofdm_syms, mod_order, cp_length)

    plt.figure(1)
    plt.subplot(2, 2, 1)
    plt.scatter(np.real(data_bpsk), np.imag(data_bpsk))
    plt.subplot(2, 2, 2)
    plt.scatter(np.real(data_qpsk), np.imag(data_qpsk))
    plt.subplot(2, 2, 3)
    plt.scatter(np.real(data_16qam), np.imag(data_16qam))
    plt.subplot(2, 2, 4)
    plt.scatter(np.real(data_64qam), np.imag(data_64qam))
    plt.show()

    plt.figure(2)
    plt.subplot(2, 2, 1)
    plt.plot(np.abs(sig_t_bpsk))
    plt.subplot(2, 2, 2)
    plt.plot(np.abs(sig_t_qpsk))
    plt.subplot(2, 2, 3)
    plt.plot(np.abs(sig_t_16qam))
    plt.subplot(2, 2, 4)
    plt.plot(np.abs(sig_t_64qam))
    plt.show()
