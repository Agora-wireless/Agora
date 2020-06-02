# Millipede

## Instruction:

1. Installation

Required packages:

`sudo apt -y install liblapack-dev libblas-dev libboost-all-dev doxygen nlohmann-json-dev python-numpy python-pyqt5`

Install Armadillo: `./scripts/install_armadillo.sh`

Install the latest version of SoapySDR from https://github.com/pothosware/SoapySDR

Intel MKL and compiler can be installed by installing Parallel Studio XE:

* Available at https://software.intel.com/en-us/parallel-studio-xe/choose-download/student-linux-fortran

Install Intel FlexRAN (optional, only used for LDPC):

* Available at https://software.intel.com/en-us/articles/flexran-lte-and-5g-nr-fec-software-development-kit-modules
* Requires gtest Google Test 1.7.0: https://github.com/google/googletest/releases/tag/release-1.7.0


Set environment vairables for Intel libraries before compiling:

	source path_to/compilervars.sh intel64

E.g., 

	source /opt/intel/compilers_and_libraries_2019.0.117/linux/bin/compilervars.sh intel64

2. Build

Compile Millipede:

	cd Millipede
	mkdir build
	cd build
	cmake ..
	make -j 

* Note: to be able run Millipede with Faros/Iris hardware, set `USE_ARGOS` variable in CMakeLists.txt to `TRUE`.

2. Run

2.1. Simulation Mode

* First, run `./data_generator data/tddconfig-sim-ul.json` to generate required data files for Millipede
* In one terminal, run `./millipede data/tddconfig-sim-ul.json` to start Millipede with Uplink configuration 
* In another terminal, run `./sender 4 2 5000 data/tddconfig-sim-ul.json` to start the sender with Uplink configuration, the four arguments are: # of threads, offset of CPU core index (change the value according to which socket the NIC is installed), frame duration in microseconds, config filename

2.2. Hardware Mode

Uplink Demo:

Flash the *client* Iris device with proper image:

* Download the image bundle from https://files.sklk.us/release/universal_2020.04.0.1-3-c9adc42.tar.gz
* Unpack the tarball and the one inside: bootfiles-iris030_ue-2020.04.0.1-3-c9adc42.tar.gz
* Copy BOOT.BIN and image.ub files to the SD card of you Iris.
* Alternatively, you can transfer the files over the network (with Iris on and discoverable):

	scp BOON.BIN image.up sklk@IrisIPAdress:~
	ssh sklk@IrisIPAddress
	sudo mount /boot
	sudo cp BOON.BIN image.ub /boot
	sudo sync
	sudo umount /boot
	sudo reboot

**Note:** Default login password to Iris is `sklk`

Running Client App on Server 2:

* Re-build code with `DEBUG_UPLINK` enabled in `src/common/Symbols.hpp`
* Modify `data/user-iris-serials.txt` by adding 2 client Iris serials in your setup.
* Run `./build/user data/ue-ul-hw.json`

Running Millipede on Server 1:

* Re-build code with `kExportConstellation` enabled in `src/common/Symbols.hpp`
* Modify `data/bs-iris-serials.txt` and `data/bs-hub-serial.txt` by adding Iris serials in your Faros RRHs.
* Run `python mm_gui.py data/bs-ul-hw.json`
 

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
