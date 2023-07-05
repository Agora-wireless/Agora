
"""
Copyright IBM

This code is Contributed IP and is being provided  according to the limited 
rights accorded in the IBM-Columbia sub-contract agreement for the COSMOS 
Effort of the NSF PAWR program

This is a collection of utility functions used by the IBM Phased Array API
"""
import numpy as np
import itertools
import spectrum

def ps2therm(phase_sh):
    """Internally used function to generate a mapping from phase shifts to the thermometer codes used in the IC. Users don't need to use this function"""
    if phase_sh <0:
        phase_sh=0
    elif phase_sh>42:
        phase_sh=42
    phase_sh_therm_list=np.concatenate((np.zeros((1,42-phase_sh),'int'),np.ones((1,phase_sh),'int')),axis=1).tolist()
    phase_sh_therm=''.join(str(x) for x in phase_sh_therm_list[0])
    return phase_sh_therm

def  closest(array, value):
    """Internally used function to quantize gain values from ideal ones to ones realizable by the IC. Users don't need to use this function"""
    f = array
    tmp = np.abs(f-value)
    idx = np.argmin(tmp) # index of closest value
    val_closest = f[0,idx] # closest value, commented by Jenny, seems that Python index 1D array with 2 numbers 
    index_closest = idx
    return [val_closest, index_closest]

def phase_repos(Npsx0, pi0):
    """Internally used function to remap a phase_shifter+phase_inverter data. Users don't need to use this function"""
    # 00 (00,0) -> 0
    # 01 (01,0) -> 4.87
    # 42 (42,0) -> 204.54
    # 43 (07,1) -> 209.41
    # 73 (37,1) -> 355.51
    # 74 (00,0) -> 0
    if Npsx0 > 73:
        Npsx = Npsx0 - 74
        pi_x = pi0
    elif Npsx0 > 42:
        Npsx = Npsx0 - 43 + 7
        pi_x = not(pi0)
    elif Npsx0 < 0:
        Npsx = Npsx0 + 74 - 43 + 7
        pi_x = ~pi0
    else:
        Npsx = Npsx0
        pi_x = pi0
    return Npsx, pi_x

def generate_taper(N, SLL):
    """Internally used function to generate Taylor tapering data. Users don't need to use this function"""
    W = spectrum.Window(N, 'taylor', sll=SLL).data
    H = W.reshape([N,1])
    V = W.reshape([1,N])
    M = H*V
    M = M/M.max()
    return 20*np.log10(M)

conf_taper_N2d = 64
conf_taper_SLL = -25
taper_data = generate_taper(8, -25)

def get_taper_data(N2d, SLL):
    """Function used by PhasedArrayControl.steer_beam() to generate Taylor tapering data for the 8x8 IBM Phased arrays

    This function caches the most recently computed outputs in order to speed up function call

    Users can use this function as a baseline to create custom tapering functions for use with PhasedArrayControl.set_arbitrary_beam()

    This function generates valid 8x8 arrays that can be used as inputs to the PhasedArrayControl.set_arbitrary_beam() method

    Parameters
    ----------
    N2d : {16, 64} integer
        Number of elements in a 2D array. If N2d=16, the tapering map is repeated to generate a 8x8 output needed for PhasedArrayControl.set_arbitrary_beam()
    SLL : float
        Side lobe level for a Taylor tapering window. Meaningful values are -13 and lower. Typical value that works for the IBM Phased Array is -20

    """
    global conf_taper_SLL
    global conf_taper_N2d
    global taper_data
    if (N2d == conf_taper_N2d) and (SLL == conf_taper_SLL):
        return taper_data
    elif N2d == 16:
        conf_taper_N2d = 16
        conf_taper_SLL = SLL
        taper_data = np.pad(generate_taper(4, SLL), (4,0), mode='wrap')
        return taper_data
    elif N2d==64:
        conf_taper_N2d = 64
        conf_taper_SLL = SLL
        taper_data = generate_taper(8, SLL)
        return taper_data
    else:
        return np.zeros([8,8]) - 3

x_mat = np.array([-3.5,-2.5,-1.5,-0.5,0.5,1.5,2.5,3.5]*8)
x_mat = x_mat.reshape([8,8])
y_mat = np.array([3.5,2.5,1.5,0.5,-0.5,-1.5,-2.5,-3.5]*8)
y_mat = y_mat.reshape([8,8]).transpose()
def phase_map(theta,phi, d=0.5506666666666666666):
    """Function used by PhasedArrayControl.steer_beam() to generate a phase map for the 8x8 IBM Phased arrays

    Users can use this function as a baseline to create custom phase map functions for use with PhasedArrayControl.set_arbitrary_beam()

    This function generates valid 8x8 arrays that can be used as inputs to the PhasedArrayControl.set_arbitrary_beam() method

    Parameters
    ----------
    theta : float
        Polar angle (degrees) in the range of (-60, 60) is supported by the array
    phi : float
        Azimuthal angle (degrees) in the range of (-90, 90) is supported by the array

    Additional optional parameters
    ------------------------------
    d : (optional) float
        Default is 0.5506666666666666666
        This is the fractional spacing of each element in the IBM Phased array in units of Lambda (wavelength)
    
    """
    temp = d*360*np.sin(np.deg2rad(theta))
    dx = temp*np.cos(np.deg2rad(phi))
    dy = temp*np.sin(np.deg2rad(phi))
    return dx*x_mat + dy*y_mat

