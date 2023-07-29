#Creates a Rayleigh fading dataset
import numpy as np
import h5py

frames = 500
sub_carrier_num = 512

ues = 4
bss = 32

dataset_file = h5py.File('rayleigh_flat_dataset.hdf5', 'w')

#Dataset shape MUST be (Frames,SCs,BSs,UEs)
H_r_dataset = np.empty(shape=(frames,sub_carrier_num,bss,ues))
H_i_dataset = np.empty(shape=(frames,sub_carrier_num,bss,ues))

for frame in range( frames ):
    for sub_carrier in range( sub_carrier_num ):
        H_r_dataset[frame,sub_carrier] = np.random.randn( bss,ues )
        H_i_dataset[frame,sub_carrier] = np.random.randn( bss,ues )

dataset_file.create_dataset('H_r', data=H_r_dataset)
dataset_file.create_dataset('H_i', data=H_i_dataset)

dataset_file.close()