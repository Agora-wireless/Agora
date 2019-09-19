#!/usr/bin/python
import sys
sys.path.append('./python')
from pyqtgraph.Qt import QtGui, QtCore
from BeamformerWrapper import *
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
		self.comp = parent.comp
                self.parent = parent

	def run(self):
		samps = np.zeros((self.userNum,self.FFT_len), np.complex64)
		while self.parent.running:
			print("update()")
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
		num_cols = 1
		num_rows = 1
		plt_scale = .6
		self.win = pg.GraphicsWindow()
		self.win.setWindowTitle('Uplink demo')
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


#class QT_GUI:
#	'''
#		This is probably not appropriate QT style, but it works fine for now.
#		
#		This class just keeps track of the graphic elements in pyqtgraph.
#		It setups up a grid of plots (1 column if 2 radios, otherwise 2 columns),
#		then simply plots the received IQ samples of each RX radio on every update in
#		the corresponding plot.
#	'''
#
#	class myWin(pg.GraphicsWindow):
#		'''Handle key presses.'''
#		def keyPressEvent(self, event):
#			#print(event.key())
#			if event.key() == QtCore.Qt.Key_Escape:
#				self.showNormal()
#			elif event.key() == ord('f') or event.key() == ord('F'):
#				self.showFullScreen() 
#			elif event.key() == ord('q') or event.key() == ord('Q'):
#				self.timer.stop() #todo: timer and cleanup should be passed in the constructor, not set after creation.
#				self.cleanup() #weird, this crashes things -- it's like Soapy doesn't detect it's already closed and tries to free it again.
#				sys.exit(self.app.exec_())
#				#self.closeEvent(event)
#			return super(QT_GUI.myWin, self).keyPressEvent(event)
#		def closeEvent(self, evnt):
#			self.timer.stop() #todo: timer and cleanup should be passed in the constructor, not set after creation.
#			#self.cleanup() #weird, this crashes things -- it's like Soapy doesn't detect it's already closed and tries to free it again.
#			super(QT_GUI.myWin, self).closeEvent(evnt)
#	def __init__(self, num_samps=4096, update_interval=250, comp=None, userNum=1, FFT_len=64):
#		self.comp = comp
#		self.userNum = userNum
#		self.FFT_len = FFT_len
#		
#		# self.cc = cc
#		#QtGui.QApplication.setGraphicsSystem('raster')
#		app = QtGui.QApplication([])
#		#mw = QtGui.QMainWindow()
#		#mw.resize(800,800)
#
#		'''  Window arrangement settings '''
#
#		#num_plots = 2 #len(sdr_serials) #this is /2 since have are tx, then *2 since each has 2 ant
#		# num_cols = 2 
#		# num_rows = int(math.ceil(userNum/num_cols))
#		num_cols = 1
#		num_rows = 1
#		plt_scale = .6
#
#		global win, vb
#		win = self.myWin(title="Argos Zero Forcing Demo")
#		# win.cleanup = self.cc.close
#
#		#win.resize(1000,600)
#		#win.showMaximized()
#		win.show()
#		# win.showFullScreen() #To return from full-screen mode, call showNormal().
#		# Enable antialiasing for prettier plots
#		pg.setConfigOptions(antialias=True)
#		win.ci.layout.setRowMaximumHeight(0,80)
#
#
#		vb = win.addViewBox(col=0, colspan=num_cols, lockAspect=True, enableMouse=False, invertY=True) #, border='00ff00'
#		vb.setBackgroundColor('ffffff')
#		#vb.setStyle() #todo:make rounded corners This takes a QStyle
#		img = QtGui.QGraphicsPixmapItem(QtGui.QPixmap('RiceLogo.jpg'))
#		vb.addItem(img)
#		#vb.scaleBy(4)
#
#		Const_plots = [None]*userNum
#		Const_data = [None]*userNum
#		for plt in range(userNum):
#			if plt % num_cols == 0:
#				win.nextRow()
#
#			Const_plots[plt] = win.addPlot(title='Constellation %i' % (plt+1))
#			Const_plots[plt].setTitle('<span style="font-size: 22pt;">User %i</span>' % (plt+1))
#			# Const_plots[plt].setRange(xRange=[-.25,.25],yRange=[-.25,.25],disableAutoRange=True)
#			Const_plots[plt].setRange(xRange=[-2,2],yRange=[-2,2],disableAutoRange=True)
#			Const_plots[plt].setAspectLocked(lock=True, ratio=1)
#			Const_data[plt] = pg.ScatterPlotItem(size=6, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))
#			Const_plots[plt].addItem(Const_data[plt])
#
#		self.Const_plots = Const_plots
#		self.Const_data = Const_data
#
#		timer = QtCore.QTimer()
#		timer.timeout.connect(self.update)
#		timer.start(update_interval)
#		win.timer = timer
#
#		#this is critical!  (otherwise it breaks, since if we don't keep a reference they are deleted: 
#		#http://stackoverflow.com/questions/5339062/python-pyside-internal-c-object-already-deleted)
#		self.timer = timer
#		self.app = app
#		self.win = win
#		self.win.app = app
#		win.show()
#	def update(self):
#		
#		samps = np.zeros((64,self.userNum), np.complex64)
#		print("update()")
#		equalData = self.comp.getEqualData()
#		for ue_id in range(self.userNum):
#			for sc_id in range(self.FFT_len):
#				samps[sc_id][ue_id] = equalData[sc_id*self.userNum*2+ue_id*2]+1j*equalData[sc_id*self.userNum*2+ue_id*2+1]
#			# print(np.max(np.abs(samps[:,0])),np.mean(np.abs(samps[:,0])))
#			print(samps[:,0])
#			self.Const_data[ue_id].setPoints(x=np.real(samps[:,ue_id]), y=np.imag(samps[:,ue_id]))
#

		
if __name__ == '__main__':

	filename = "data/tddconfig.json"
	#cfg = Config(filename)
	comp = CoMP(filename)
        nusers = 4
        data_len = 48
	mode = ''
        if len(sys.argv) > 2:
            nusers = int(sys.argv[1])
            data_len = int(sys.argv[2])
            if len(sys.argv) == 4: mode = (sys.argv[3])
	app = QtGui.QApplication(sys.argv)
	w = MainWindow(comp=comp, userNum=nusers, FFT_len=data_len, mode=mode, update_interval=1000)
	w.show()
	# Thread(target = init_gui(comp=comp, userNum=1, FFT_len=64, update_interval=1000)).start()
	# comp.startCC()
	# mGUI = QT_GUI(comp=comp, userNum=1, FFT_len=64, update_interval=1000)
	# mGUI.win.show()
	Thread(target = comp.startCoMP).start()

	sys.exit(app.exec_())

	


