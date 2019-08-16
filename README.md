# Millipede

## Instruction:

1. Installation

Required packages LAPACK, BLAS, Boost, Doxygen

	sudo apt-get install liblapack-dev, libblas-dev, libboost-all-dev
	sudo apt-get install doxygen

Install Armadillo:

	wget http://sourceforge.net/projects/arma/files/armadillo-9.300.2.tar.xz .
	tar xf armadillo-9.300.2.tar.xz
	cd armadillo-9.300.2
	cmake .
	make -j4
	sudo make install

Install muFFT:

	git clone https://github.com/Themaister/muFFT.git
	cd muFFT
	make
	sudo make install

Install aff3ct:

	git clone https://github.com/aff3ct/my_project_with_aff3ct.git .
	cd my_project_with_aff3ct
	git submodule update --init --recursive
	cd lib/aff3ct
	mkdir build
	cd build
	cmake .. -G"Unix Makefiles" -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-funroll-loops -march=native" -DAFF3CT_COMPILE_EXE="OFF" -DAFF3CT_COMPILE_STATIC_LIB="ON" -DAFF3CT_COMPILE_SHARED_LIB="ON"
	make -j4
	
Go back to the Millipede repository

	mkdir cmake && mkdir cmake/Modules
	cp ../../my_project_with_aff3ct/lib/aff3ct/build/lib/cmake/aff3ct-*/* cmake/Modules

Compile Millipede:

	cd millipede
	mkdir build
	cd build
	cmake ..
	make -j36 (36 is the number of cores, on the server it can be 36)

2. Run
* In one terminal, run "./millipede" to start the receiver
* In another terminal, run "./sender 4 4 1 20" to start the sender, the four arguments are: # of sockets, # of threads (should be same as # of sockets), offset of CPU core index (change the value according to which socket the NIC is installed), and delay between symbols

3. Other information
* CoMP.cpp is the file that controls most things (performs FFT, ZF, and demodulation). 
* I added some debug information settings in Symbols.hpp
* Thread number settings are in CoMP.hpp
* test_matrix.cpp is for unit tests of matrix operations
* test_mufft.c is for unit tests of FFT and IFFT

To compile test_matrix.cpp:

	g++ -I/opt/intel/vtune_amplifier/include -o test_matrix ../test_matrix.cpp ../cpu_attach.cpp -std=c++11 -w -O3 -mavx2 -mavx -g -larmadillo -lpthread /opt/intel/vtune_amplifier/lib64/libittnotify.a -ldl

To compile test_mufft.c:

	gcc -o test_mufft ../test_mufft.c /usr/local/lib/libmufft.a -lm -Wl,-no-undefined

