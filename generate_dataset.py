#Creates a Rayleigh fading dataset
import numpy as np
import h5py

flat_fading = True
frames = 50
sub_carrier_num = 512

ues = 4
bss = 32

dataset_file = h5py.File('rayleigh_flat_dataset.hdf5', 'w')

#Dataset shape MUST be (Frames,SCs,BSs,UEs)
H_r_dataset = np.empty(shape=(frames,sub_carrier_num,bss,ues))
H_i_dataset = np.empty(shape=(frames,sub_carrier_num,bss,ues))

for frame in range( frames ):
    if flat_fading:
        H_r = np.random.randn( bss,ues )
        H_i = np.random.randn( bss,ues )

    for sub_carrier in range( sub_carrier_num ):
        if not flat_fading:
            H_r = np.random.randn( bss,ues )
            H_i = np.random.randn( bss,ues )
        H_r_dataset[frame,sub_carrier] = H_r
        H_i_dataset[frame,sub_carrier] = H_i

print( H_i_dataset.shape )

dataset_file.create_dataset('H_r', data=H_r_dataset)
dataset_file.create_dataset('H_i', data=H_i_dataset)

dataset_file.close()