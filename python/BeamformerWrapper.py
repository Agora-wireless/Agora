from ctypes import *
import numpy as np
import matplotlib.pyplot as plt
import json


lib = cdll.LoadLibrary('./build/libcomp.so')


class Config(object):
    def __init__(self, val):  	
        lib.Config_new.argtypes = [c_char_p]
        lib.Config_new.restype = c_void_p
        self.obj = lib.Config_new(c_char_p(val))

class CoMP(object):
    def __init__(self, configfile):
        conf = Config(configfile)
        lib.Agora_new.argtypes = [c_void_p]
        lib.Agora_new.restype = c_void_p
        lib.Agora_start.argtypes = [c_void_p]
        lib.Agora_start.restype = c_void_p
        lib.Agora_getEqualData.argtypes = [c_void_p,POINTER(POINTER(c_float)),POINTER(c_int)]
        lib.Agora_getEqualData.restype = c_void_p
        self.obj = lib.Agora_new(conf.obj)

    def startCoMP(self):
    	lib.Agora_start(self.obj)

    def stopCoMP(self):
        lib.Agora_stop()

    def destroyCoMP(self):
        lib.Agora_destroy(self.obj)

    def getEqualData(self):
        mem = POINTER(c_float)()
        size = c_int(0)
        lib.Agora_getEqualData(self.obj, mem, size)
        return mem,size
    

# filename = "../tddconfig.json"
# cfg = Config(filename)
# comp = CoMP(cfg.obj)
# cfg=Config('../tddconfig.json')
# comp = CoMP(cfg)
# comp.startCC()

        # self.readEqualData_C = self.lib.getEqualData
        # self.readEqualData_C.argtypes = [POINTER(POINTER(c_float)),POINTER(c_int)]          

        # self.readDemulData_C = self.lib.getDemulData
        # self.readDemulData_C.argtypes = [POINTER(POINTER(c_longlong)),POINTER(c_int)]  
        
    # def startCC(self):
    #     self.lib.start()

    # def readEqualData(self):
	   #  mem = POINTER(c_float)()
	   #  size = c_int(0)
	   #  self.readEqualData_C(byref(mem),byref(size))
	   #  return mem     

    # def readDemulData(self):
	   #  mem = POINTER(c_longlong)()
	   #  size = c_int(0)
	   #  self.readDemulData_C(byref(mem),byref(size))
	   #  return mem 


