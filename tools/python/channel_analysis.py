"""
 channel_analysis.py
 CSI analysis API file
 Author(s): Clay Shepard: cws@rice.edu
            Rahman Doost-Mohamamdy: doost@rice.edu
            Oscar Bejarano: obejarano@rice.edu
---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
---------------------------------------------------------------------
"""

import sys
import struct
import numpy as np
import os
import math
import time
import datetime
from scipy import signal
import multiprocessing as mp
import matplotlib.pyplot as plt
from generate_sequence import *


def calCond(userCSI):
    """Calculate the standard matrix condition number.
    Args:
            userCSI: Complex numpy array with [Frame, User, BS Ant, Subcarrier]
    Returns:
            condNumber_ave: The average condition number across all users and subcarriers.
            condNumber: Numpy array of condition number [Frame, Subcarrier]. 
    """
    condNumber = np.empty(
        (userCSI.shape[0], userCSI.shape[3]), dtype='float32')
    for sc in range(userCSI.shape[3]):
        condNumber[:, sc] = np.linalg.cond(
            userCSI[:, :, :, sc])
    condNumber_ave = np.average(condNumber)
    return condNumber_ave, condNumber


def calDemmel(userCSI):
    """Calculate the Demmel condition number.
    Args:
            userCSI: Complex numpy array with [Frame, User, BS Ant, Subcarrier]
    Returns:
            demmelNumber_ave: The average condition number across all users and subcarriers.
            demmelNumber: Numpy array of condition number [Frame, Subcarrier].
    """
    demmelNumber = np.empty(
        (userCSI.shape[0], userCSI.shape[3]), dtype='float32')
    for sc in range(userCSI.shape[3]):

        # covariance matrix
        cov = np.matmul(userCSI[:, :, :, sc], np.transpose(
            userCSI[:, :, :, sc], [0, 2, 1]).conj())
        eigenvalues = np.abs(np.linalg.eigvals(cov))
        demmelNumber[:, sc] = np.sum(
            eigenvalues, axis=1)/np.min(eigenvalues, axis=1)
    demmelNumber_ave = np.average(demmelNumber)
    return demmelNumber_ave, demmelNumber


def calCapacity(userCSI, noise, beamweights, downlink=False):
    """Calculate the capacity of a trace with static beamweights.
    Apply a set of beamweights to a set of wideband user channels and calculate the shannon capacity of the resulting channel for every Frame.
    Note that if the beamweights are calculated with a frame from the trace, that frame will have unrealistic capacity since it will correlate noise as signal.
    Args:
            userCSI: Complex numpy array with [Frame, User, BS Ant, Subcarrier]
            noise: Complex numpy array with [Frame, BS Ant, Subcarrier]
            beamweights: Set of beamweights to apply to userCSI [BS Ant, User, Subcarrier]
            downlink: (Boolean) Compute downlink capacity if True, else Uplink
    Returns:
            cap_total: Total capacity across all users averaged over subarriers in bps/hz [Frame]
            cap_u: Capacity per user across averaged over subcarriers in bps/hz [Frame, User]
            cap_sc: Capacity per user and subcarrier in bps/hz [Frame, User, Subcarrier]
            SINR: Signtal to interference and noise ratio for each frame user and subcarrier [Frame, User, Subcarrier]
            cap_su_sc: Single user (no interference) capacity per subcarrier in bps/hz  [Frame, User, Subcarrier]
            cap_su_u: Single user (no interference) capacity averaged over subcarriers in bps/hz [Frame, User]
            SNR: Signtal to noise ratio for each frame user and subcarrier [Frame, User, Subcarrier]
    """
    noise_bs_sc = np.mean(np.mean(np.abs(noise), 0),
                          0)  # average over time and the two ltss
    sig_intf = np.empty(
        (userCSI.shape[0], userCSI.shape[1], userCSI.shape[1], userCSI.shape[3]), dtype='float32')
    noise_sc_u = np.empty(
        (userCSI.shape[1], userCSI.shape[3]), dtype='float32')
    for sc in range(userCSI.shape[3]):
        # TODO: can we get rid of the for loop?
        sig_intf[:, :, :, sc] = np.square(
            np.abs(np.dot(userCSI[:, :, :, sc], beamweights[:, :, sc])))
        # noise is uncorrelated, and all we have is average power here (Evan wants to do it per frame, but I think that's a bad idea)
        noise_sc_u[:, sc] = np.dot(
            np.square(noise_bs_sc[:, sc]), np.square(np.abs(beamweights[:, :, sc])))

    # noise_sc_u *= 4 #fudge factor since our noise doesn't include a lot of noise sources

    sig_sc = np.diagonal(sig_intf, axis1=1, axis2=2)
    sig_sc = np.swapaxes(sig_sc, 1, 2)
    # remove noise from signal power (only matters in low snr really...)
    sig_sc = sig_sc - noise_sc_u
    sig_sc[sig_sc < 0] = 0  # can't have negative power (prevent errors)
    intf_sc = np.sum(sig_intf, axis=1+int(downlink)) - sig_sc
    SINR = sig_sc/(noise_sc_u+intf_sc)

    cap_sc = np.log2(1+SINR)
    cap_u = np.mean(cap_sc, axis=2)
    cap_total = np.sum(cap_u, axis=1)

    SNR = sig_sc/noise_sc_u
    cap_su_sc = np.log2(1+SNR)
    cap_su_u = np.mean(cap_su_sc, axis=2)

    return cap_total, cap_u, cap_sc, SINR, cap_su_sc, cap_su_u, SNR


def calContCapacity(csi, conj=True, downlink=False, offset=1):
    """Calculate the capacity of a trace with continuous beamforming.
    For every frame in a trace, calculate beamweights (either conjugate or ZF),
    apply them to a set of wideband user channels either from the same frame or some constant offset (delay),
    then calculate the shannon capacity of the resulting channel.
    The main difference in uplink/downlink is the source of interference (and power allocation).
    In uplink the intended user's interference is a result of every other user's signal passed through that user's beamweights.
    In downlink the inteded user's interference is a result of every other user's signal passed through their beamweights (applied to the intended user's channel).
    Note that every user has a full 802.11 LTS, which is a repitition of the same symbol.
    This method uses the first half of the LTS to make beamweights, then applies them to the second half.
    Otherwise, noise is correlated, resulting in inaccurate results.
    Args:
            csi: Full complex numpy array with separate LTSs and noise [Frame, User, BS Ant, Subcarrier] (noise is last user)
            conj: (Boolean) If True use conjugate beamforming, else use zeroforcing beamforming.
            downlink: (Boolean) Compute downlink capacity if True, else Uplink
            offset: Number of frames to delay beamweight application.
    Returns:
            cap_total: Total capacity across all users averaged over subarriers in bps/hz [Frame]
            cap_u: Capacity per user across averaged over subcarriers in bps/hz [Frame, User]
            cap_sc: Capacity per user and subcarrier in bps/hz [Frame, User, Subcarrier]
            SINR: Signtal to interference and noise ratio for each frame user and subcarrier [Frame, User, Subcarrier]
            cap_su_sc: Single user (no interference) capacity per subcarrier in bps/hz  [Frame, User, Subcarrier]
            cap_su_u: Single user (no interference) capacity averaged over subcarriers in bps/hz [Frame, User]
            SNR: Signtal to noise ratio for each frame user and subcarrier [Frame, User, Subcarrier]
    """
    csi_sw = np.transpose(
        csi, (0, 4, 1, 3, 2))  # hack to avoid for loop (matmul requires last two axes to be matrix) #frame, sc, user, bsant, lts
    # noise is last set of data. #frame, sc, bsant, lts
    noise = csi_sw[:, :, -1, :, :]
    # don't include noise, use first LTS for CSI #frame, sc, user, bsant, lts
    userCSI_sw = csi_sw[:, :, :-1, :, 0]

    # average over time and the two ltss
    noise_sc_bs = np.mean(np.mean(np.abs(noise), 3), 0)

    if conj:
        '''Calculate weights as conjugate.'''
        beamweights = np.transpose(
            np.conj(csi_sw[:, :, :-1, :, 1]), (0, 1, 3, 2))
    else:
        '''Calculate weights using zeroforcing.'''
        beamweights = np.empty(
            (userCSI_sw.shape[0], userCSI_sw.shape[1], userCSI_sw.shape[3], userCSI_sw.shape[2]), dtype='complex64')
        for frame in range(userCSI_sw.shape[0]):
            for sc in range(userCSI_sw.shape[1]):
                # * np.linalg.norm(csi[frame,:4,0,:,sc]) #either this, or the noise power has to be scaled back accordingly
                beamweights[frame, sc, :, :] = np.linalg.pinv(
                    csi_sw[frame, sc, :-1, :, 1])
    if offset > 0:
        # delay offset samples
        beamweights = np.roll(beamweights, offset, axis=0)

    sig_intf = np.square(
        np.abs(np.matmul(userCSI_sw[offset:, :, :, :], beamweights[offset:, :, :, :])))

    noise_sc_u = np.transpose(np.sum(np.square(
        noise_sc_bs)*np.square(np.abs(np.transpose(beamweights, (0, 3, 1, 2)))), 3), (0, 2, 1))
    noise_sc_u = noise_sc_u[offset:]
    # noise_sc_u *= 4 #fudge factor since our noise doesn't include a lot of noise sources.  this should probably be justified/measured or removed

    sig_sc = np.diagonal(sig_intf, axis1=2, axis2=3)
    # remove noise from signal power (only matters in low snr really...)
    sig_sc = sig_sc - noise_sc_u
    sig_sc[sig_sc < 0] = 0  # can't have negative power (prevent errors)
    # lazy hack -- just sum then subtract the intended signal.
    intf_sc = np.sum(sig_intf, axis=2+int(downlink)) - sig_sc
    SINR = sig_sc/(noise_sc_u+intf_sc)

    cap_sc = np.log2(1+SINR)
    cap_u = np.mean(cap_sc, axis=1)
    cap_total = np.sum(cap_u, axis=1)

    SNR = sig_sc/noise_sc_u
    cap_su_sc = np.log2(1+SNR)
    cap_su_u = np.mean(cap_su_sc, axis=1)

    return cap_total, cap_u, cap_sc, SINR, cap_su_sc, cap_su_u, SNR


def calExpectedCapacity(csi, user=0, max_delay=100, conj=True, downlink=False):
    """Calculate the expected capacity for beamweights calculated with delayed stale CSI.
    Args:
            csi: Full complex numpy array with separate LTSs and noise [Frame, User, BS Ant, Subcarrier] (noise is last user)
            user: Index of user to compute for (note that other users still affect capacity due to their interference)
            max_delay: Maximum delay (in frames) to delay the beamweight computation.
            conj: (Boolean) If True use conjugate beamforming, else use zeroforcing beamforming.
            downlink: (Boolean) Compute downlink capacity if True, else Uplink
    Returns:
            cap: Average capacity across all frames for a given delay (in frames) in bps/hz [Delay]
    """
    cap = []
    for d in range(max_delay):
        # print([d,time.time()])
        delayed = calContCapacity(
            csi, conj=conj, downlink=downlink, offset=d)
        cap.append(np.mean(delayed[1][:, user]))

    return cap


def find_bad_nodes(csi, corr_thresh=0.32, user=0):
    """Find bad Iris nodes.
	
    Find bad nodes based on the correlation between a reference node 
    and every other node. If all nodes are 'bad' then the reference 
    node is assumed to be bad, and a new reference node is chosen. The 
    reference node is picked sequentially from the list of BS antennas.
    
    Note: 
        csi should be a channel trace taken in a stable environment. 
        If AGC is used, csi should be taken after AGC has settled.
	
	Note:
        By only correlating one node with another, the decorrelation 
        is maximized when there is a sync issue. (If you use more 
        antennas for reference, the impact of the time sync is less. 
        If you use fewer, the bad antennas, which have all of the same 
        phase shift, dominate correlation.)
        
        An important trick is to normalize amplitudes of the CSI so 
        that the phase shift always has the same impact, regardless of 
        channel gains (i.e. if the reference node has low or high 
        signal strength relative to the bad node it would decrease the 
        impact of a time sync issue). 
        
        This also allows a fixed threshold to work well as a single 
        sample shift of 4 antennas out of 8 relatively consistently 
        causes a decorrelation of .25. If half the samples (worst 
        case), this is a .125 deviation from the mean.
	
	Args:
        csi: Complex array [Frame, User, Pilot Rep, BS Ant, Subcarrier]
		thresh: Lower threshold means more sensitivity to deviation in 
            correlation. Higher shall give more false negatives, lower 
            shall give more false positives.
		user: The index of the user to use for correlation.
		
	Returns:
		bad_nodes: A list of indices of nodes determined to be bad.
	"""
    userCSI = np.mean(csi[1:csi.shape[0]:,:,:,:,:], 2)
    num_nodes = userCSI.shape[2]
    print(">>> User {user}: Total {num_nodes} reference Iris nodes: ".format(
        user=user, num_nodes=num_nodes))
    if num_nodes == 1:
        node_good = [True]
    else:
        corr = [None] * num_nodes
        ref_good = False
        ref = num_nodes//2  # 0
        node_good = [True] * num_nodes
        # Must normalize amplitude so that the phase shift always has 
        # the same effect on correlation.
        userCSI_abs = np.abs(userCSI)
        userCSI_abs[userCSI_abs == 0] = 1
        userCSI /= userCSI_abs  # Handles the division by zero case.
        print(">>> corr_thresh = {corr_thresh}\n>>> corr:".format(
            corr_thresh=corr_thresh))

        while not ref_good:
            for n in range(num_nodes):
                if n != ref:
                    sl = list(range(ref,ref+1)) + list(range(n,n+1))
                    corr_vec = np.conj(userCSI[0,:,sl,:])
                    c = userCSI[:,:,sl,:]
                    corr[n] = calCorr(c, corr_vec)[0][:,user]
                    v = np.max(np.abs(corr[n] - np.mean(corr[n])))
                    print(n+1, v)
                    if  v > corr_thresh:
                        node_good[n] = False
            if np.sum(node_good) > 1:  # It is good reference antenna.
                ref_good = True
                print(">>> Node {ref} is a good reference.".format(ref=ref+1))
            else:  # It may have the same timing error as other nodes.
                print(">>> Warning! Node {ref} chosen for reference appears "
                    "to be bad! Trying the next node ...".format(ref=ref))
                ref +=1
                node_good = [True] * num_nodes
                if ref == num_nodes:
                        print(">>> No good nodes found! If there are only 2 "
                            "nodes, perhaps just 1 is bad.")
                        ref_good = True  # Used to exit the while loop.
                        node_good = [False] * num_nodes
                
    bad_nodes = [i+1 for i, x in enumerate(node_good) if not x]
    return bad_nodes


def calCorr(userCSI, corr_vec):
    """Calculate instantaneous correlation with a correlation vector.
    Sub-samples userCSI in the time/user/subcarrier slice before 
    passing it. If slicing just one user, dimensionality has to be 
    maintained, i.e. slice like userCSI[:,[2],:,:].
    Args:
        userCSI: Complex numpy array [Frame, User, BS Ant, Subcarrier].
        corr_vec: Vector to correlate with [BS Ant, User, Subcarrier].
    Returns:
        corr_total: Average correlation across subcarriers [Frame,User].
        sig_sc: Correlation on every [Frame, User, Subcarrier].
    Typical usage example:
        corr_total,sig_sc = calCorr(userCSI, 
            np.transpose(np.conj(userCSI[frame,:,:,:]),(1,0,2)))
    """
    sig_intf = np.empty((userCSI.shape[0], userCSI.shape[1], 
                         userCSI.shape[1], userCSI.shape[3]), dtype='float32')

    for sc in range(userCSI.shape[3]):
        sig_intf[:, :, :, sc] = np.abs(np.dot(userCSI[:,:,:,sc],
                                       corr_vec[:,:,sc])) / \
                                np.dot(np.abs(userCSI[:,:,:,sc]),
                                       np.abs(corr_vec[:,:,sc]))

    # Get correlation of subcarriers for each user across BS antennas
    sig_sc = np.diagonal(sig_intf, axis1=1, axis2=2)
    sig_sc = np.swapaxes(sig_sc, 1, 2)
    corr_total = np.mean(sig_sc, axis=2)  # Averaging corr across users

    return corr_total, sig_sc


def demult(csi, data, noise=None, method='zf'):
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

def calcBW(csi, noise=None, method='zf'):
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

### This function requires Gurobi Optimization package installed (requires a license)
def mlDetector(csi_f, ul_syms_f, mod_syms):
    from ml_solver import mlSolver
    mod_syms_lin = list(set(np.real(mod_syms)))
    n_frames = csi_f.shape[0]
    n_ants = csi_f.shape[1]
    n_users = csi_f.shape[2]
    sc_per_slot = csi_f.shape[3]

    # convert complex csi to real
    csi_f_real = np.zeros((csi_f.shape[0], 2 * csi_f.shape[1], 2 * csi_f.shape[2], csi_f.shape[3]))
    csi_f_real[:, :n_ants, :n_users, :] = np.real(csi_f)
    csi_f_real[:, n_ants:, :n_users, :] = np.imag(csi_f)
    csi_f_real[:, :n_ants, n_users:, :] = -np.imag(csi_f)
    csi_f_real[:, n_ants:, n_users:, :] = np.real(csi_f)

    # convert complex receive signal to real
    ul_syms_f_real = np.zeros((n_frames, 2 * n_ants, sc_per_slot))
    ul_syms_f_real = np.concatenate((np.real(ul_syms_f), np.imag(ul_syms_f)), axis=1)

    indexing_start = time.time()
    demod_sc_real = np.zeros((n_frames, 2 * n_users, sc_per_slot), dtype='complex64')
    for sc in range(sc_per_slot):
        demod_sc_real[:, :, sc] = mlSolver(csi_f_real[:, :, :, sc], ul_syms_f_real[:, :, sc], mod_syms_lin)
    ## TODO: use multi processing to solve Maximum Likelihood detector
    indexing_end = time.time()
    print("ML Solver time: %f \n" % (indexing_end - indexing_start))

    ul_demod_syms = demod_sc_real[:, :n_users, :] + 1j*demod_sc_real[:, n_users:, :]
    return ul_demod_syms