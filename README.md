# CoMP
Mac version of CoMP

## Instruction:

1. Compile
* mkdir build
* cd build
* cmake ..
* make -j36 (36 is the number of cores, on the server it can be 36)

2. Run
* In one terminal, run "./CoMP" to start the receiver
* In another terminal, run "./sender 20 1 20" to start the sender, the three arguments are: # of sockets, # of threads, and offset of CPU core index

3. Other information
* CoMP.cpp is the file that controls most things (performs FFT, ZF, and demodulation). 
* I added some debug information settings in Symbols.hpp
* Thread number settings are in CoMP.hpp
