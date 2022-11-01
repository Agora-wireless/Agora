from ctypes import *
import numpy as np
import matplotlib.pyplot as plt
import json


lib = cdll.LoadLibrary('./build/libue_phy.so')


class Config(object):
    def __init__(self, val):  	
        lib.Config_new.argtypes = [c_char_p]
        lib.Config_new.restype = c_void_p
        self.obj = lib.Config_new(c_char_p(val))

class UserClass(object):
    def __init__(self, configfile):
        conf = Config(configfile) 
        lib.Phy_UE_new.argtypes = [c_void_p]
        #lib.Phy_UE_new.restype = [c_void_p]
        lib.Phy_UE_new.restype = c_void_p
        lib.Phy_UE_start.argtypes = [c_void_p]
        lib.Phy_UE_start.restype = c_void_p
        lib.Phy_UE_getEqualData.argtypes = [c_void_p,POINTER(POINTER(c_float)),POINTER(c_int), c_int]
        lib.Phy_UE_getEqualData.restype = c_void_p
        lib.Phy_UE_getEqualPCData.argtypes = [c_void_p,POINTER(POINTER(c_float)),POINTER(c_int), c_int]
        lib.Phy_UE_getEqualPCData.restype = c_void_p
        lib.Phy_UE_getDemulData.argtypes = [c_void_p,POINTER(POINTER(c_longlong)),POINTER(c_int), c_int]
        lib.Phy_UE_getDemulData.restype = c_void_p
        self.obj = lib.Phy_UE_new(conf.obj)

    def startUserProc(self):
    	lib.Phy_UE_start(self.obj)

    def stopUserProc(self):
    	#lib.Phy_UE_stop(self.obj)
    	lib.Phy_UE_stop()

    def destroyUserProc(self):
    	lib.Phy_UE_destroy(self.obj)

    def getEqualData(self, ue_id):
        mem = POINTER(c_float)()
        size = c_int(0)
        lib.Phy_UE_getEqualData(self.obj, mem, size, ue_id)
        return mem,size
    
    def getEqualPCData(self, ue_id):
        mem = POINTER(c_float)()
        size = c_int(0)
        lib.Phy_UE_getEqualPCData(self.obj, mem, size, ue_id)
        return mem,size
    
    def getDemulData(self):
        mem = POINTER(c_longlong)()
        size = c_int(0)
        lib.Phy_UE_getDemulData(self.obj, byref(mem),byref(size))
        return mem,size

