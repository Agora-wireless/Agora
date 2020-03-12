# Millipede

## Instruction:

1. Installation

Required packages LAPACK, BLAS, Boost, Doxygen

	sudo apt-get install liblapack-dev libblas-dev libboost-all-dev
	sudo apt-get install doxygen nlohmann-json-dev

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
    git checkout 6d716ab
	make
	sudo make install

Intel MKL and compiler can be installed by installing Parallel Studio XE:

* Available at https://software.intel.com/en-us/parallel-studio-xe/choose-download/student-linux-fortran

Install Intel FlexRAN (optional, only used for LDPC):

* Available at https://software.intel.com/en-us/articles/flexran-lte-and-5g-nr-fec-software-development-kit-modules
* Requires gtest Google Test 1.7.0: https://github.com/google/googletest/releases/tag/release-1.7.0


Set environment vairables for Intel libraries before compiling:

	source path_to/compilervars.sh intel64

E.g., 

	source /opt/intel/compilers_and_libraries_2019.0.117/linux/bin/compilervars.sh intel64


Compile Millipede:

	cd Millipede
	mkdir build
	cd build
	cmake ..
	make -j 

2. Run
* First, run "./data_generator data/tddconfig-sim-ul.json" to generate required data files for Millipede
* In one terminal, run "./millipede data/tddconfig-sim-ul.json" to start Millipede with Uplink configuration 
* In another terminal, run "./sender 4 2 5000 data/tddconfig-sim-ul.json" to start the sender with Uplink configuration, the four arguments are: # of threads, offset of CPU core index (change the value according to which socket the NIC is installed), frame duration in microseconds, config filename

3. Other information
* millipede.cpp is the file that controls most things (performs FFT, ZF, and demodulation). 
* Eebug information settings are in Symbols.hpp
* test/test_millipede is used for correctness test
  * The sender sends 1 frame, Millipede processes it and compares results with gound truth data.
  * Gound truth data is produced by MATLAB file generate_data_dl.m. 

To compile and run Millipede test:

	cd test/test_millipede
	cmake .
	make -j
	./test_millipede.sh

* test_matrix.cpp is for unit tests of matrix operations (inversion and multiplication)
* test_mufft.c is for unit tests of FFT and IFFT

To compile test_matrix.cpp:

	g++ -o test_matrix test_matrix.cpp cpu_attach.cpp -std=c++11 -w -O3 -mavx2 -g -larmadillo -lpthread -lm -ldl 

To compile test_mufft.c:

	gcc -o test_mufft ../test_mufft.c /usr/local/lib/libmufft.a -lm -Wl,-no-undefined

