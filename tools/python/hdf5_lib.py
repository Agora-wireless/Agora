#!/usr/bin/python3
"""
 hdf5_lib.py
 Library handling recorded hdf5 file from channel sounding (see Sounder/).
---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import struct
import gc
import numpy as np
import h5py
import matplotlib.pyplot as plt
import collections
import time
from optparse import OptionParser
from ofdmtxrx import ofdmTxRx

class hdf5_lib:
    def __init__(self, filename, n_frames_to_inspect=0, n_fr_insp_st = 0, sub_sample = 0):
        self.h5file = None
        self.filename = filename
        self.dirpath = '/'.join(filename.split('/')[:-1])
        self.h5struct = []
        self.data = []
        self.metadata = {}
        self.pilot_samples = []
        self.uplink_samples = []
        self.n_frm_st = n_fr_insp_st                                # index of last frame
        self.n_frm_end = self.n_frm_st + n_frames_to_inspect    # index of last frame in the range of n_frames_to_inspect
        self.sub_sample = sub_sample
        self.open_hdf5()
        self.get_data()
        self.get_metadata()

    def open_hdf5(self):
        """
        Get the most recent log file, open it if necessary.
        """
        if (not self.h5file) or (self.filename != self.h5file.filename):
            # if it's closed, e.g. for the C version, open it
            print('Opening %s...' % self.filename)
            try:
                self.h5file = h5py.File(self.filename, 'r')
            except OSError:
                print("File not found. Terminating program now")
                sys.exit(0)
        # return self.h5file

    def get_data(self):
        """
        Parse file to retrieve metadata and data.
        HDF5 file has been written in DataRecorder.cpp (in Sounder folder)
        Output:
            Data (hierarchy):
                -Path
                -Pilot_Samples
                    --Samples
                -UplinkData
                    --Samples
        Dimensions of input sample data (as shown in DataRecorder.cpp in Sounder):
            - Pilots
                dims_pilot[0] = maxFrame
                dims_pilot[1] = number of cells
                dims_pilot[2] = number of UEs
                dims_pilot[3] = number of antennas (at BS)
                dims_pilot[4] = samples per symbol * 2 (IQ)
            - Uplink Data
                dims_data[0] = maxFrame
                dims_data[1] = number of cells
                dims_data[2] = uplink symbols per frame
                dims_data[3] = number of antennas (at BS)
                dims_data[4] = samples per symbol * 2 (IQ)
        """

        self.data = self.h5file['Data']

        if 'Pilot_Samples' in self.data:
            if self.n_frm_st == self.n_frm_end:
                # Consider the entire dataset (for demos etc)
                self.pilot_samples = self.data['Pilot_Samples']
            else:
                self.pilot_samples = self.data['Pilot_Samples'][self.n_frm_st:self.n_frm_end:self.sub_sample, ...]

        if 'UplinkData' in self.data:
            if self.n_frm_st == self.n_frm_end:
                # Consider the entire dataset (for demos etc)
                self.uplink_samples = self.data['UplinkData']
            else:
                self.uplink_samples = self.data['UplinkData'][self.n_frm_st:self.n_frm_end:self.sub_sample, ...]

        return self.data

    def get_metadata(self):
        """
                -Attributes
                        {FREQ, RATE, PREFIX_LEN, POSTFIX_LEN, SLOT_SAMP_LEN, FFT_SIZE, CP_LEN,
                        BEACON_SEQ_TYPE, PILOT_SEQ_TYPE, BS_HUB_ID, BS_SDR_NUM_PER_CELL, BS_SDR_ID, BS_NUM_CELLS,
                        BS_CH_PER_RADIO, BS_FRAME_SCHED, BS_RX_GAIN_A, BS_TX_GAIN_A, BS_RX_GAIN_B, BS_TX_GAIN_B,
                        BS_BEAMSWEEP, BS_BEACON_ANT, BS_NUM_ANT, BS_FRAME_LEN, CL_NUM, CL_CH_PER_RADIO, CL_AGC_EN,
                        CL_RX_GAIN_A, CL_TX_GAIN_A, CL_RX_GAIN_B, CL_TX_GAIN_B, CL_FRAME_SCHED, CL_SDR_ID,
                        CL_MODULATION, UL_SLOTS}
        """

        # Retrieve attributes, translate into python dictionary
        #data = self.data
        self.metadata = dict(self.h5file['Data'].attrs)
        if "CL_SDR_ID" in self.metadata.keys():
            cl_present = True
        else:
            cl_present = False
            print('Client information not present. It is likely the client was run separately')

        bs_id = self.metadata['BS_SDR_ID'].astype(str)
        if bs_id.size == 0:
            raise Exception('Base Station information not present')

        # Data cleanup
        # In OFDM_DATA_CLx and OFDM_PILOT, we have stored both real and imaginary in same vector
        # (i.e., RE1,IM1,RE2,IM2...REm,IM,)

        # Freq-domain Pilot
        if "OFDM_PILOT_F" in self.metadata.keys():
            pilot_vec = self.metadata['OFDM_PILOT_F']
            # some_list[start:stop:step]
            I = pilot_vec[0::2]
            Q = pilot_vec[1::2]
            pilot_complex = I + Q * 1j
            self.metadata['OFDM_PILOT_F'] = pilot_complex

        # Time-domain Pilot
        if "OFDM_PILOT" in self.metadata.keys():
            pilot_vec = self.metadata['OFDM_PILOT']
            # some_list[start:stop:step]
            I = pilot_vec[0::2]
            Q = pilot_vec[1::2]
            pilot_complex = I + Q * 1j
            self.metadata['OFDM_PILOT'] = pilot_complex

        if 'UL_SYMS' in self.metadata:
            n_ul_slots = self.metadata['UL_SYMS']
        elif 'UL_SLOTS' in self.metadata:
            n_ul_slots = self.metadata['UL_SLOTS']
        if n_ul_slots > 0:
            # Phase-Tracking Pilot Subcarrier
            pilot_sc_vals = self.metadata['OFDM_PILOT_SC_VALS']
            pilot_sc_vals_complex = pilot_sc_vals[0::2] + 1j * pilot_sc_vals[1::2]
            self.metadata['OFDM_PILOT_SC_VALS'] = pilot_sc_vals_complex

        return self.metadata

    @staticmethod
    def samps2csi_large(samps, num_users, chunk_size=1000, samps_per_user=224, fft_size=64, offset=0, bound=94, cp=0, pilot_f=[], legacy=False):
        """Wrapper function for samps2csi_main for to speed up large logs by leveraging data-locality. Chunk_size may need to be adjusted based on your computer."""

        print("starting samps2csi")
        samps2csi_large_start = time.time()
        if samps.shape[0] > chunk_size:
                    # rather than memmap let's just increase swap... should be just as fast.
                    #csi = np.memmap(os.path.join(_here,'temp1.mymemmap'), dtype='complex64', mode='w+', shape=(samps.shape[0], num_users, 2, samps.shape[1],52))
                    #iq = np.memmap(os.path.join(_here,'temp2.mymemmap'), dtype='complex64', mode='w+', shape=(samps.shape[0], num_users, 2, samps.shape[1],64))
            chunk_num = samps.shape[0] // chunk_size
            csi0, SNR0 = hdf5_lib.samps2csi(
                    samps[:chunk_size, :, :, :], num_users, samps_per_user, fft_size, offset, bound, cp, pilot_f, legacy, 0)
            csi = np.empty(
                (samps.shape[0], csi0.shape[1], csi0.shape[2], csi0.shape[3], csi0.shape[4]), dtype='complex64')
            csi[:chunk_size] = csi0
            SNR = np.empty(
                (samps.shape[0], csi0.shape[1], csi0.shape[2], csi0.shape[3]), dtype='complex64')
            for i in range(1, chunk_num):
                csi[i*chunk_size:i*chunk_size+chunk_size], SNR[i*chunk_size:i*chunk_size+chunk_size] = hdf5_lib.samps2csi(
                        samps[i*chunk_size:(i*chunk_size+chunk_size), :, :, :], num_users, samps_per_user, fft_size, offset, bound, cp, pilot_f, legacy, i)
            if samps.shape[0] > chunk_num*chunk_size:
                csi[chunk_num*chunk_size:], SNR[chunk_num*chunk_size:] = hdf5_lib.samps2csi(
                    samps[chunk_num*chunk_size:, :, :, :], num_users, samps_per_user, fft_size, offset, bound, cp, pilot_f, legacy, chunk_num)
        else:
            csi, SNR = hdf5_lib.samps2csi(
                samps, num_users, samps_per_user, fft_size, offset, bound, cp, pilot_f, legacy)

        print("samps2csi_large took %f seconds" % (time.time() - samps2csi_large_start))
        return csi, SNR

    @staticmethod
    def samps2csi(samps, num_users, samps_per_user=224, fft_size=64, offset=0, bound=94, cp=0, pilot_f=[], legacy=False, chunk_id=-1):
        """Convert an Argos HDF5 log file with raw IQ in to CSI.
        Asumes 802.11 style LTS used for trace collection.
    
        Args:
            samps: The h5py or numpy array containing the raw IQ samples,
                dims = [Frame, Cell, User, Antenna, Sample].
            num_users: Number of users used in trace collection. (Last 'user' is noise.)
            samps_per_user: Number of samples allocated to each user in each frame.
     
        Returns:
            csi: Complex numpy array with [Frame, Cell, User, Pilot Rep, Antenna, Subcarrier]
            iq: Complex numpy array of raw IQ samples [Frame, Cell, User, Pilot Rep, Antenna, samples]
     
        Example:
            h5log = h5py.File(filename,'r')
            csi,iq = samps2csi(h5log['Pilot_Samples'], h5log.attrs['num_mob_ant']+1, h5log.attrs['samples_per_user'])
        """
        debug = False
        chunkstart = time.time()
        samps2csi_start = time.time()

        usersamps = np.reshape(
            samps, (samps.shape[0], num_users, samps.shape[2], samps_per_user, 2))
        ofdm_len = fft_size+cp
        pilot_rep = (samps_per_user-bound)//ofdm_len
        #pilot_rep = min([(samps_per_user-bound)//(fft_size+cp), 2]) # consider min. 2 pilot reps
        iq = np.empty((samps.shape[0], num_users, samps.shape[2], pilot_rep, fft_size), dtype='complex64')
        if debug:
            print("chunkstart = {}, usersamps.shape = {}, samps.shape = {}, samps_per_user = {}, iq.shape = {}".format(
                chunkstart, usersamps.shape, samps.shape, samps_per_user, iq.shape))
        for i in range(pilot_rep):
            iq[:, :, :, i, :] = (usersamps[:, :, :, offset + cp + i*ofdm_len:offset+(i+1)*ofdm_len, 0] +
                                    usersamps[:, :, :, offset + cp + i*ofdm_len:offset+(i+1)*ofdm_len, 1]*1j)*2**-15

        iq = iq.swapaxes(2, 3)
        if debug:
            print("iq.shape after axes swapping: {}".format(iq.shape))

        fftstart = time.time()
        csi = np.empty(iq.shape, dtype='complex64')
        zero_sc = np.where(pilot_f == 0)[0]
        nonzero_sc_size = len(pilot_f) - len(zero_sc)

        fft_length = int(np.power(2, np.ceil(np.log2(nonzero_sc_size))))
        if fft_length != fft_size:
            print("Expected fft size %d, given %d"%(fft_length, fft_size))
            return None, iq
        #start_i = int((fft_length - nonzero_sc_size) // 2)
        #stop_i = int(start_i + nonzero_sc_size)
        nonzero_sc = np.setdiff1d(range(fft_size), zero_sc)
        iq_fft = np.fft.fftshift(np.fft.fft(iq, fft_size, 4), 4)
        seq_freq_inv = 1 / pilot_f[nonzero_sc]
        csi = iq_fft[:, :, :, :, nonzero_sc] * seq_freq_inv
        if len(zero_sc) == 0:
            SNR = None
        else:
            noise = iq_fft[:, :, :, :, zero_sc]
            noise_power = np.mean(np.power(np.abs(noise), 2), 4) * len(nonzero_sc)
            signal_power = np.sum(np.power(np.abs(iq_fft[:, :, :, :, nonzero_sc]), 2), 4)
            SNR = signal_power / noise_power

        endtime = time.time()
        if debug:
            print("csi.shape:{} lts_freq.shape: {}, pre_csi.shape = {}".format(
                csi.shape, lts_freq.shape, pre_csi.shape))
        if debug:
            print("chunk time: %f fft time: %f" %
                  (fftstart - chunkstart, endtime - fftstart))
        if debug:
            print("csi.shape:{}".format(csi.shape))
        if chunk_id == -1:
            print("samps2csi took %f seconds" % (time.time() - samps2csi_start))
        else:
            print("samps2csi chunk %d took %f seconds" % (chunk_id, time.time() - samps2csi_start))
        del iq
        gc.collect()
        return csi, SNR

    @staticmethod
    def load_tx_data(metadata, dirpath, n_users):
        if 'SYMBOL_LEN' in metadata: # to support older datasets
            samps_per_slot = int(metadata['SYMBOL_LEN'])
        elif 'SLOT_SAMP_LEN' in metadata:
            samps_per_slot = int(metadata['SLOT_SAMP_LEN'])
        prefix_len = int(metadata['PREFIX_LEN'])
        postfix_len = int(metadata['POSTFIX_LEN'])
        z_padding = prefix_len + postfix_len
        fft_size = int(metadata['FFT_SIZE'])
        cp = int(metadata['CP_LEN'])
        ofdm_len = fft_size + cp
        if 'UL_SYMS' in metadata:
            ul_slot_num = int(metadata['UL_SYMS'])
        elif 'UL_SLOTS' in metadata:
            ul_slot_num = int(metadata['UL_SLOTS'])
        cl_ch_num = int(metadata['CL_CH_PER_RADIO'])
        num_cl = int(metadata['CL_NUM'])
        data_sc_ind = np.array(metadata['OFDM_DATA_SC'])
        pilot_sc_ind = np.array(metadata['OFDM_PILOT_SC'])
        pilot_sc_val = np.array(metadata['OFDM_PILOT_SC_VALS'])
        zero_sc_ind = np.setdiff1d(range(fft_size), data_sc_ind)
        zero_sc_ind = np.setdiff1d(zero_sc_ind, pilot_sc_ind)
        nonzero_sc_ind = np.setdiff1d(range(fft_size), zero_sc_ind)
        ul_data_frame_num = int(metadata['UL_DATA_FRAME_NUM'])
        tx_file_names = metadata['TX_FD_DATA_FILENAMES'].astype(str)
        txdata = np.empty((ul_data_frame_num, ul_slot_num, num_cl,
                     fft_size), dtype='complex64')
        read_size = 2 * ul_data_frame_num * ul_slot_num * cl_ch_num * fft_size
        cl = 0
        for fn in tx_file_names:
            if dirpath == "":
                tx_file_path = fn
            else:
                tx_file_path = dirpath + '/' + fn
            print('Opening source TX data file %s'%tx_file_path)
            with open(tx_file_path, mode='rb') as f:
                txdata0 = list(struct.unpack('f'*read_size, f.read(4*read_size)))
                I = np.array(txdata0[0::2])
                Q = np.array(txdata0[1::2])
                IQ = I + Q * 1j
                # frames, users, ul_slots, sym_per_slots, fft_size
                txdata[:, :, cl:cl+cl_ch_num, :] = np.reshape(IQ, (ul_data_frame_num, ul_slot_num,
                    cl_ch_num, fft_size))
            cl = cl + cl_ch_num
        txdata = np.fft.fftshift(txdata, 3)
        
        return txdata

    @staticmethod
    def demodulate(ul_samps, csi, txdata, metadata, ue_frame_offset, offset, ul_slot_i, noise_samps_f=None, method='zf'):
        if method.lower() == 'mmse' and noise_samps_f is None:
            print("%s requires noise samples"%(method))
            return None
        if 'SYMBOL_LEN' in metadata: # to support older datasets
            samps_per_slot = int(metadata['SYMBOL_LEN'])
        elif 'SLOT_SAMP_LEN' in metadata:
            samps_per_slot = int(metadata['SLOT_SAMP_LEN'])
        prefix_len = int(metadata['PREFIX_LEN'])
        postfix_len = int(metadata['POSTFIX_LEN'])
        z_padding = prefix_len + postfix_len
        modulation = metadata['CL_MODULATION'].astype(str)[0]
        fft_size = int(metadata['FFT_SIZE'])
        cp = int(metadata['CP_LEN'])
        ofdm_len = fft_size + cp
        if 'UL_SYMS' in metadata:
            ul_slot_num = int(metadata['UL_SYMS'])
        elif 'UL_SLOTS' in metadata:
            ul_slot_num = int(metadata['UL_SLOTS'])
        if 'UL_PILOT_SLOTS' in metadata:
            ul_pilot_slot_num = int(metadata['UL_PILOT_SLOTS'])
        else:
            ul_pilot_slot_num = 2
            print('missing UL_PILOT_SLOTS in metadata; using default', ul_pilot_slot_num)
        ul_data_slot_num = ul_slot_num - ul_pilot_slot_num
        data_sc_ind = np.array(metadata['OFDM_DATA_SC'])
        pilot_sc_ind = np.array(metadata['OFDM_PILOT_SC'])
        pilot_sc_val = np.array(metadata['OFDM_PILOT_SC_VALS'])
        zero_sc_ind = np.setdiff1d(range(fft_size), data_sc_ind)
        zero_sc_ind = np.setdiff1d(zero_sc_ind, pilot_sc_ind)
        nonzero_sc_ind = np.setdiff1d(range(fft_size), zero_sc_ind)
        ofdm_obj = ofdmTxRx()
        data_sc_len = len(data_sc_ind)

        M = 2
        if modulation == 'QPSK':
            M = 4
        elif modulation == '16QAM':
            M = 16
        elif modulation == '64QAM':
            M = 64

        # UL Samps: #Frames, #Uplink SLOTS, #Antennas, #Samples
        n_frames = ul_samps.shape[0]
        n_slots = ul_samps.shape[1]
        n_ants = ul_samps.shape[2]
        n_users = csi.shape[1]
        ul_syms = np.empty((n_frames, n_slots, n_ants, fft_size), dtype='complex64')

        # process tx data
        rep = n_frames // txdata.shape[0]
        tx_symbols = np.tile(txdata, (rep, 1, 1, 1))
        frac_fr = n_frames % txdata.shape[0]
        #frac_fr = 0
        if frac_fr > 0:
            frac = txdata[:frac_fr, :, :, :]
            tx_symbols = frac if rep == 0 else np.concatenate((tx_symbols, frac), axis=0)
        # frames, users, ul_slots, sym_samps
        tx_syms = tx_symbols[:, :, :, data_sc_ind]
        useful_frame_num = tx_syms.shape[0]
        if txdata.shape[0] > 1:
            useful_frame_num = useful_frame_num - max(ue_frame_offset)
        min_ue_offset = min(ue_frame_offset)
        slot_evm = np.zeros((useful_frame_num, ul_data_slot_num, n_users))
        slot_evm_snr = np.zeros((useful_frame_num, ul_data_slot_num, n_users))
        slot_ser = np.zeros((useful_frame_num, ul_data_slot_num, n_users))

        # UL Syms: #Frames, #Uplink SLOTS, #Antennas, #OFDM_Samples
        ul_syms = ul_samps[:, :, :, offset+cp:offset+ofdm_len]
        ul_syms_f = np.fft.fftshift(np.fft.fft(ul_syms, fft_size, 3), 3)
        ul_syms_f_tp = ul_syms_f[:, :, :, nonzero_sc_ind]
        # UL DEMULT: #Frames, #OFDM Symbols, #User, #Sample (DATA + PILOT SCs)
        # We assume the first two (or more) slots are pilots used for phase tracking
        # Slot(s) onwards in Agora are data
        for i in range(ul_pilot_slot_num):
            ul_demult = ofdm_obj.demult(csi, ul_syms_f_tp[:, i:i+1, :, :], noise_samps_f, method=method)
            # the first ul_pilot_slot_num slots are pilots used for phase tracking
            phase_shift_cur = np.angle(np.sum(ul_demult * np.conj(tx_syms[0, i, :, :] ), axis=3))
            if i > 0:
                phase_shift_diff = phase_shift_cur - phase_shift_prev # Only using the diff between the last two pilots
            if i < ul_pilot_slot_num - 1:
                phase_shift_prev = phase_shift_cur

        ul_equal_syms = np.zeros((n_frames, ul_data_slot_num, n_users, data_sc_len), dtype='complex64')
        ul_demod_syms = np.empty(ul_equal_syms.shape, dtype="int")
        for i in range(ul_data_slot_num):
            phase_shift_cur += phase_shift_diff
            ul_demult = ofdm_obj.demult(csi, ul_syms_f_tp[:, ul_pilot_slot_num + i:, :, :], noise_samps_f, method=method)
            phase_comp = np.exp(-1j * phase_shift_cur)
            phase_comp_ext = np.tile(np.expand_dims(phase_comp, axis=3), (1, 1, 1, data_sc_len))
            ul_equal_syms[:, i:, :, :] = np.multiply(ul_demult, phase_comp_ext)

        tx_data_syms = tx_syms[:, ul_pilot_slot_num:, :, :]
        for j in range(ul_data_slot_num):
            for k in range(n_users):
                frame_start = 0 if txdata.shape[0] == 1 else min_ue_offset
                frame_end = frame_start + useful_frame_num
                slot_evm[:, j, k] = np.linalg.norm(ul_equal_syms[frame_start:frame_end, j, k, :] - tx_data_syms[:useful_frame_num, j, k, :], 2, axis=1) / data_sc_len
                for i in range(n_frames):
                    ul_demod_syms[i, j, k, :] = ofdm_obj.demodulation(ul_equal_syms[i, j, k, :], M)
                for i in range(frame_start, frame_end):
                    new_i = i - frame_start
                    ul_tx_syms = ofdm_obj.demodulation(tx_data_syms[new_i, j, k, :], M)
                    res = [m for m, l in zip(list(ul_demod_syms[i, j, k, :]), list(ul_tx_syms)) if m == l]
                    slot_ser[new_i, j, k] = (data_sc_len - len(list(res))) / data_sc_len
        slot_evm_snr = 10 * np.log10(1 / slot_evm)

        return ul_equal_syms, ul_demod_syms, tx_data_syms, slot_evm, slot_evm_snr, slot_ser
