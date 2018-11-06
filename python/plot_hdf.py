#!/usr/bin/python
import sys
import struct
import numpy as np
#import configparser 
import os
import math
import h5py
import time
import datetime
import logging
import matplotlib.pyplot as plt
#import lts
import pdb
#from detect_peaks import detect_peaks
from channel_analysis import *

class plot_hdf:

    def __init__(self, filename, uplink_samples):
        self.plot_h5file = None
        self.filename = filename
        self.uplink = uplink_samples
    def get_h5(self):
        '''Get the most recent log file, open it if necessary.'''
        if (not self.plot_h5file) or (self.filename != self.plot_h5file.filename): #if it's closed, e.g. for the C version, open it
            print('Opening %s...' % self.filename)
            self.plot_h5file = h5py.File(self.filename,'r')
        return self.plot_h5file
	
    def plot_frame(self,samps=None,bs_ant=None,frame=None):
        '''Debug plot of real part of one frame that is useful for checking sync and AGC.'''
        if (samps is None):
            h5f = self.get_h5()
            samps = h5f['Pilot_Samples']
 
        ncell = samps.shape[1]
        frame = frame if frame is not None else samps.shape[0]//2 #random frame -- let's just choose one in the middle.
        fig,axes = plt.subplots(nrows=ncell,ncols=1)
        c = 0
        if ncell == 1: axes = [axes]
        for ax1 in axes: #for c in range(ncell):
            #ax1 = fig.add_subplot(1, ncell, c+1)
            ax1.set_ylabel('Real Part of Frame %d' % frame)
            ax1.set_title('Cell %d'%c)
            if bs_ant is None: bs_ant = range(samps.shape[2])
            for bs in bs_ant:
                #corr = np.correlate(samps[frame,c,bs,:,0]+samps[frame,c,bs,:,1]*1j, lts.genLTS(cp=0), 'full')
                ax1.plot(samps[frame,c,bs,:,0])
                #ax1.plot(corr)
            if c == ncell - 1:
                ax1.set_xlabel('Sample')
            c += 1
        fig.show()
		

    def plot_corr(self,samps=None,frame=0, length=512+94):
    	'''Debug plot that is useful for checking sync.'''
        if (samps is None or user is None):
            h5f = self.get_h5()
            pilots = h5f['/Data/Pilot_Samples']
        csi, iq = samps2csi(pilots,1,length,64, offset=320)
        amps = np.mean(np.abs(iq[:,0,0,0,0,:]), axis=1)
        pilot_frames = [i for i in range(len(amps)) if amps[i] > 0.01]
        print("good frames len %d" % len(pilot_frames))
        ref_frame = pilot_frames[len(pilot_frames)//2] 
        #print("amp of reference frame %f", np.mean(np.abs(iq[ref_frame,0,0,0,0,:])))
        print("amp of reference frame %d is %f"% (ref_frame, amps[ref_frame]))
    	ncell = iq.shape[1]
    	c = 0
    	fig,axes = plt.subplots(nrows=ncell,ncols=1)
    	if ncell == 1: axes = [axes]
    	for ax1 in axes: #c in range(ncell):
    		#cellCSI = np.squeeze(csi[:,c,:,:,:,:], axis=1)
                cellCSI = csi[:,0,:,:,:,:]
                #cellCSI = np.delete(cellCSI,[2,3],3)
    		userCSI = np.mean(cellCSI[:,:,:,:,:],2)
    		corr_total,sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[ref_frame,:,:,:]),(1,0,2)))
                best_frames = [i for i in  pilot_frames if corr_total[i,0]>0.99]
                good_frames = [i for i in  pilot_frames if corr_total[i,0]>0.95]
                bad_frames = [i for i in   pilot_frames if corr_total[i,0]>0.9 and corr_total[i,0]<=0.94]
                worst_frames = [i for i in pilot_frames if corr_total[i,0]<0.9]
                print("num of best frames %d" % len(best_frames))
                print("num of good frames %d" % len(good_frames))
                print("num of bad frames   %d" % len(bad_frames))
                print("num of worst frames   %d" % len(worst_frames))
    		ax1.set_ylabel('Correlation with Frame %d' % ref_frame)
    		ax1.set_ylim([0,1.1])
    		ax1.set_title('Cell %d'%c)
    		#ax1.plot(corr_total[:,0])
    		ax1.plot(corr_total[pilot_frames,0])
    		ax1.set_xlabel('Frame')		
    		c += 1
    	plt.show()

    def plot_csi(self,samps=None, subcarrier=1):       
        if (samps is None):
            h5f = self.get_h5()
            pilots = h5f['/Data/Pilot_Samples']
        csi, iq, ofst, good_frames = samps2csi_findLTS(pilots,1,512+94)
        valid_idx = abs(csi[:,0,0,0,0,5])>=0.08 #check valid frames
        samples = np.arange(1,csi.shape[0]+1)
        fig,axes = plt.subplots(nrows=4,ncols=1)
        axes[0].set_title('Channel amplitude over time')
        axes[0].set_ylabel('Amplitude')
        axes[0].set_xlabel('Sample index')		
        axes[0].plot(np.abs(csi[valid_idx,0,0,0,:,subcarrier]))
		
        axes[1].set_title('Channel phase relative to ant 0')
        axes[1].set_ylabel('Phase')
        axes[1].set_xlabel('Sample index')		
        axes[1].plot(np.angle(csi[valid_idx,0,0,0,:,subcarrier]*np.conj(csi[valid_idx,0,0,0,0,subcarrier][:,np.newaxis])))		

        plt.show()

    def plot_csi_new(self,samps=None, frame=0, length=512+94, subcarrier=1):       
        if (samps is None):
            h5f = self.get_h5()
            pilots = h5f['/Data/Pilot_Samples']
        csi, iq = samps2csi(pilots,1,length,64, offset=256)
        amps = np.mean(np.abs(iq[:,0,0,0,0,:]), axis=1)
        pilot_frames = [i for i in range(len(amps)) if amps[i] > 0.01]
        ref_frame = pilot_frames[len(pilot_frames)//2] 
        cellCSI = csi[:,0,:,:,:,:]
    	userCSI = np.mean(cellCSI[:,:,:,:,:],2)
    	corr_total,sig_sc = calCorr(userCSI, np.transpose(np.conj(userCSI[ref_frame,:,:,:]),(1,0,2)))
        valid_idx = [i for i in  pilot_frames if corr_total[i,0]>0.95]
        print("num of good frames %d" % len(valid_idx))
        fig,axes = plt.subplots(nrows=2,ncols=1)
        axes[0].set_title('Channel amplitude over time')
        axes[0].set_ylabel('Amplitude')
        axes[0].set_xlabel('Sample index')		
        axes[0].plot(np.abs(csi[valid_idx,0,0,0,:,subcarrier]))
        axes[0].legend(fontsize=10)	
        axes[1].set_title('Channel phase relative to ant 0')
        axes[1].set_ylabel('Phase')
        axes[1].set_xlabel('Sample index')		
        axes[1].plot(np.angle(csi[valid_idx,0,0,0,:,subcarrier]*np.conj(csi[valid_idx,0,0,0,0,subcarrier][:,np.newaxis])))		

        plt.show()

    def analyze_frames(self,samps=None, frame1=0, frame2=1):
        plt.close("all")
        if (samps is None):
            h5f = self.get_h5()
            pilots = h5f['/Data/Pilot_Samples']
        csi, samps = samps2csi(pilots,1,512+94,512+94, offset=0,bound=0)
        offset = 0
	gold = lts.genLTS(upsample=1, cp=0)[:64]
        pilot1 = samps[frame1,0,0,0,:,:]
        pilot2 = samps[frame2,0,0,0,:,:]
        best, actual_ltss, peaks = lts.findLTS(pilot1[0,:])
        peaks_loc = detect_peaks(np.abs(peaks),mph=max(abs(peaks))/2)
        print(peaks_loc[0])
        best, actual_ltss, peaks = lts.findLTS(pilot1[1,:])
        peaks_loc = detect_peaks(np.abs(peaks),mph=max(abs(peaks))/2)
        print(peaks_loc[0])
        best, actual_ltss, peaks = lts.findLTS(pilot2[0,:])
        peaks_loc = detect_peaks(np.abs(peaks),mph=max(abs(peaks))/2)
        print(peaks_loc[0])
        best, actual_ltss, peaks = lts.findLTS(pilot2[1,:])
        peaks_loc = detect_peaks(np.abs(peaks),mph=max(abs(peaks))/2)
        print(peaks_loc[0])
        fig, axes = plt.subplots(nrows=4,ncols=1)
        axes[0].set_title('Cell 0')
        axes[0].set_ylabel('Real Part of Frame %d ant 0' % frame1)
        axes[0].plot(np.real(pilot1[0,:]))
        #axes[0].plot(np.real(np.correlate(pilot1[0,:],gold,'full') ))

        axes[1].set_ylabel('Real Part of Frame %d ant 1' % frame1)
        axes[1].plot(np.real(pilot1[1,:]))

        axes[2].set_ylabel('Real Part of Frame %d ant 0' % frame2)
        axes[2].plot(np.real(pilot2[0,:]))
        #axes[2].plot(np.real(np.correlate(pilot2[0,:],gold,'full') ))

        axes[3].set_ylabel('Real Part of Frame %d ant 1' % frame2)
        axes[3].plot(np.real(pilot2[1,:]))

        axes[3].set_xlabel('Sample')
        plt.show()
		
    def plot_debug(self,samps=None, frame=100, length=512+94+30):
        plt.close("all")
        if (samps is None):
            h5f = self.get_h5()
            pilots = h5f['/Data/Pilot_Samples']
        csi, samps = samps2csi(pilots,1,length,length, offset=0, bound=0)
        fig,axes = plt.subplots(nrows=4,ncols=1)
        axes[0].set_title('Cell 0')
        axes[0].set_ylabel('Real Part of Frame %d ant 0' % frame)
        axes[0].plot(np.real(samps[frame,0,0,0,0,:]))

        axes[1].set_ylabel('Real Part of Frame %d ant 1' % frame)
        axes[1].plot(np.real(samps[frame,0,0,0,1,:]))

        axes[2].set_ylabel('Real Part of All Frames ant 0')
        axes[2].plot(np.real(samps[:,0,0,0,0,:]).flatten())

        axes[3].set_ylabel('Real Part of All Frames ant 1')
        axes[3].plot(np.real(samps[:,0,0,0,1,:]).flatten())

        axes[3].set_xlabel('Sample')
        plt.show()
		
if __name__ == '__main__':
    starttime = time.time()	
    show_plots = True	
    zoom = 0  #samples to zoom in around frame (to look at local behavior), 0 to disable
    pl = 0
    if len(sys.argv) > 1:
        filename = sys.argv[1]
        uplink = int(sys.argv[2])
        fnum = int(sys.argv[3])
        cc = plot_hdf(filename,uplink) # worst
        #cc.analyze_frames(frame1=103961,frame2=fnum)
        #cc.plot_debug(frame=fnum)
        #cc.plot_good_frames()
        #cc.plot_corr(frame=fnum, length=512)
        #cc.plot_csi_new(frame=fnum, subcarrier=2)
        frame_length = 512+150
        cc.plot_debug(frame=fnum, length=frame_length)
        cc.plot_corr(frame=fnum, length=frame_length)
        #cc.plot_csi_new(length=frame_length, subcarrier=2)
    else:
        #cc = plot_hdf('test_4ant_3.hdf5', 0)
        frame_length = 512+94+30
        cc = plot_hdf('test.hdf5', 0)
        cc.plot_debug(frame=0, length=frame_length)
        cc.plot_corr(frame=0, length=frame_length)
        cc.plot_csi_new(length=frame_length, subcarrier=2)
