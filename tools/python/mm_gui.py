#!/usr/bin/python
import sys
sys.path.append('./python')
from pyqtgraph.Qt import QtGui, QtCore
from BeamformerWrapper import *
import numpy as np
import pyqtgraph as pg
import math
import json
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
        self.comp = parent.comp
        self.parent = parent

    def run(self):
        samps = np.zeros((self.userNum,self.FFT_len), np.complex64)
        while self.parent.running:
            if self.mode == 'pc':
                [equalData,datalen] = self.comp.getEqualPCData()
            else:
                [equalData,datalen] = self.comp.getEqualData()
            for ue_id in range(self.userNum):
                for sc_id in range(self.FFT_len):
                    samps[ue_id][sc_id] = equalData[sc_id*self.userNum*2+ue_id*2]+1j*equalData[sc_id*self.userNum*2+ue_id*2+1]
                # print(np.max(np.abs(samps[:,0])),np.mean(np.abs(samps[:,0])))
                # print(samps[0,:])
            self.dataChanged.emit(samps)
            QtCore.QThread.msleep(250)

class MainWindow(QtGui.QMainWindow):
    def __init__(self, num_samps=4096, update_interval=250, comp=None, userNum=1, FFT_len=64, mode=''):
        QtGui.QMainWindow.__init__(self)
        self.comp = comp
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
        num_cols = max(self.userNum/4, 1)
        num_rows = 1
        plt_scale = .6
        self.win = pg.GraphicsWindow()
        self.win.setWindowTitle('Uplink demo')
        #self.win.resize(500,300*self.userNum)

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
            self.Const_plots[plt].setRange(xRange=[-4.5,4.5],yRange=[-4.5,4.5],disableAutoRange=True)
            self.Const_plots[plt].setAspectLocked(lock=True, ratio=1)
            self.Const_data[plt] = pg.ScatterPlotItem(size=6, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))
            self.Const_plots[plt].addItem(self.Const_data[plt])

    def closeEvent(self, event):
        self.comp.stopCoMP()
        #self.comp.destroyCoMP();
        self.running = False
        event.accept()
        #self.deleteLater()         


if __name__ == '__main__':

    filename = "files/experiment/bs-ul-hw.json"
    #cfg = Config(filename)
    nusers = 4
    data_len = 48
    mode = ''
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    with open(filename) as json_file:
        data = json.load(json_file)
        if 'ue_num' not in data:
            sched = data['frames']
            nusers = sched[0].count('P')
        else:
            nusers = data['ue_num']
        data_len = int(data['ofdm_data_num'])
    print('nusers %d, data_len %d' %(nusers, data_len))
    comp = CoMP(filename)
    app = QtGui.QApplication(sys.argv)
    w = MainWindow(comp=comp, userNum=nusers, FFT_len=data_len, mode=mode, update_interval=1000)
    w.show()
    # Thread(target = init_gui(comp=comp, userNum=1, FFT_len=64, update_interval=1000)).start()
    # comp.startCC()
    # mGUI = QT_GUI(comp=comp, userNum=1, FFT_len=64, update_interval=1000)
    # mGUI.win.show()
    Thread(target = comp.startCoMP).start()

    sys.exit(app.exec_())

    


