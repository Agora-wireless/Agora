#!/usr/bin/python
import sys
sys.path.append('python/')
from pyqtgraph.Qt import QtGui, QtCore
from UserWrapper import *
import numpy as np
import pyqtgraph as pg
import math
import threading
from threading import Thread



class readDataThread(QtCore.QThread):
    dataChanged = QtCore.pyqtSignal(np.ndarray)
    def __init__(self, parent=None):
        super(readDataThread, self).__init__(parent)
        # QtCore.QThread.__init__(self, *args, **kwargs)
        self.userNum = parent.userNum
        self.FFT_len = parent.FFT_len
        self.mode = parent.mode
        self.parent = parent
        self.userClass = parent.userClass

    def run(self):
        samps = np.zeros((self.userNum, self.FFT_len), np.complex64)
        while self.parent.running:
            for ue_id in range(self.userNum):
                if self.mode == 'pc':
                    [equalData,datalen] = self.userClass.getEqualPCData(ue_id)
                else:
                    [equalData,datalen] = self.userClass.getEqualData(ue_id)
                for sc_id in range(self.FFT_len):
                    samps[ue_id][sc_id] = equalData[sc_id*2]+1j*equalData[sc_id*2+1]
            self.dataChanged.emit(samps)
            QtCore.QThread.msleep(250)

class MainWindow(QtGui.QMainWindow):
    def __init__(self, num_samps=4096, update_interval=250, userClass=None, userNum=1, FFT_len=64, mode=''):
        QtGui.QMainWindow.__init__(self)
        self.userClass = userClass
        self.userNum = userNum
        self.FFT_len = FFT_len
        self.mode = mode
        self.running = True
        self.start()
        self.m = 0

    def update_plot(self, samps):
        print("Reading: ",self.m)
        if self.m > 99:
            self.xall = np.zeros((self.userNum,self.FFT_len), dtype=float)
            self.yall = np.zeros((self.userNum,self.FFT_len), dtype=float)
            self.m = 0
        else:
            self.xall = np.real(samps)
            self.yall = np.imag(samps)
            for ue_id in range(np.shape(self.xall)[0]):
                self.Const_data[ue_id].setPoints(x=self.xall[ue_id], y=self.yall[ue_id])
            self.m += 1

    def start(self):
        thread = readDataThread(self)
        thread.dataChanged.connect(self.update_plot)
        thread.start()
        num_cols = 1
        num_rows = 1
        plt_scale = .6
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('Downlink Constellations')
        self.win.resize(500,300*self.userNum)
        
        self.Const_plots = [None]*self.userNum
        self.Const_data = [None]*self.userNum
        self.xall = np.zeros((self.userNum,self.FFT_len), dtype=float)
        self.yall = np.zeros((self.userNum,self.FFT_len), dtype=float)
        for plt in range(self.userNum):
            if plt % num_cols == 0:
                self.win.nextRow()
            
            self.Const_plots[plt] = self.win.addPlot(title='Constellation %i' % (plt+1))
            self.Const_plots[plt].setTitle('<span style="font-size: 22pt;">User %i</span>' % (plt+1))
            # Const_plots[plt].setRange(xRange=[-.25,.25],yRange=[-.25,.25],disableAutoRange=True)
            self.Const_plots[plt].setRange(xRange=[-1.5,1.5],yRange=[-1.5,1.5],disableAutoRange=True)
            self.Const_plots[plt].setAspectLocked(lock=True, ratio=1)
            self.Const_data[plt] = pg.ScatterPlotItem(size=6, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))
            self.Const_plots[plt].addItem(self.Const_data[plt])

    def closeEvent(self, event):
        self.userClass.stopUserProc()
        #self.userClass.destroyUserProc()
        self.running = False
        event.accept()

		
if __name__ == '__main__':
    filename = "files/experiment/ue-dl-hw.json"
    nusers = 4
    data_len = 52
    mode = ''
    isDownlink = True
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    with open(filename) as json_file:
        data = json.load(json_file)
        if 'ue_num' not in data:
            sched = data['frames']
            nusers = sched[0].count('P')
            isDownlink = sched[0].count('D') > 2
        else:
            nusers = data['ue_num']
        data_len = int(data['ofdm_data_num'])
    print('nusers %d, data_len %d' %(nusers, data_len))
    if not isDownlink:
        print('No downlink symbols. Exiting...')
        sys.exit(0)
    ue = UserClass(filename)
    app = QtGui.QApplication(sys.argv)
    w = MainWindow(userClass=ue, userNum=nusers, FFT_len=data_len, mode=mode, update_interval=1000)
    w.show()
    Thread(target = ue.startUserProc).start()
    sys.exit(app.exec_())
