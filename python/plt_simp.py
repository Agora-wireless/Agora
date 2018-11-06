import pickle
import numpy as np
import scipy.io as sio
import matplotlib.pyplot as plt
def cfloat2uint32(arr):
        arr_i = (np.real(arr) * 32767).astype(np.uint16)
        arr_q = (np.imag(arr) * 32767).astype(np.uint16)
        return np.bitwise_or(arr_i ,np.left_shift(arr_q.astype(np.uint32), 16))

def uint32tocfloat(arr):
	arr_i = ((np.right_shift(arr, 16).astype(np.int16))/32768.0)
	arr_q = (np.bitwise_and(arr, 0xFFFF).astype(np.int16))/32768.0
	return (arr_i + 1j*arr_q).astype(np.complex64)

def write_to_file(name,arr,num_bits=12):
    """Save complex numpy array val to files prefixed with name in binary twos-complement binary format with num_bits."""
    fi = open(name+'.bin', 'wb')
    for a in arr:
        #fi.write(np.binary_repr(a,width=num_bits))
        pickle.dump(a,fi)
    fi.close()

def read_from_file(name,leng,offset,num_bits=12):
    """Save complex numpy array val to files prefixed with name in binary twos-complement binary format with num_bits."""
    fi = open(name+'.bin', 'rb')
    for k in range(offset):
        pickle.load(fi)
    arr = np.array([0]*leng, np.uint32)
    for a in range(leng):
        #fi.write(np.binary_repr(a,width=num_bits))
        arr[a] = pickle.load(fi)
    fi.close()
    return arr

fig = plt.figure(figsize=(20, 8), dpi=100)
ax1 = fig.add_subplot(2, 1, 1)
ax2 = fig.add_subplot(2, 1, 2)
pilot = uint32tocfloat(read_from_file("pilots", 1024, 0))#256*100))
#pilot = uint32tocfloat(read_from_file("rxpilot", 256*100, 0))#256*100))
rxdata = mp.zeros(1024) #uint32tocfloat(read_from_file("rxdata", 256*100, 0))#256*100))



sio.savemat('rxTest.mat', {'pilot':pilot, 'rxdata':rxdata})

ax1.plot(np.real(pilot), label='pilot i')
ax1.plot(np.imag(pilot), label='pilot q')
ax2.plot(np.real(rxdata), label='rx data i')
ax2.plot(np.imag(rxdata), label='rx data q')
plt.show()
 
