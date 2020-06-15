Millipede is a high-performance system for massive-MIMO baseband processing.

## Requirements
 * Toolchain: A C++11 compiler and CMake 2.8+. Enabling LDPC decoding requires
   an Intel C++ compiler (`icpc`).
 * Required packages
   * `sudo apt -y install liblapack-dev libblas-dev libboost-all-dev doxygen nlohmann-json-dev python-numpy python-pyqt5 libgflags-dev`
   * Install Intel MKL (see [instructions](https://software.intel.com/content/www/us/en/develop/articles/installing_intel_free_libs_and_python_apt_repo.html))
   * Install Armadillo: `./scripts/install_armadillo.sh`
   * Install the latest version of SoapySDR from https://github.com/pothosware/SoapySDR
   * Download Intel FlexRAN to `/opt` (does not need to be compiled)
     * Download [link](https://software.intel.com/en-us/articles/flexran-lte-and-5g-nr-fec-software-development-kit-modules)
   * Optional: Install Intel compiler
     * Intel MKL and compiler can be installed by installing Parallel Studio XE
     * Set environment vairables by sourcing `compilervars.sh`, e.g.,
     `source /opt/intel/compilers_and_libraries_2019.0.117/linux/bin/compilervars.sh intel64`


## Millipede quickstart

 * Run the tests
    ```
    cd test/test_millipede
    cmake .
    make -j
    ./test_millipede.sh 100 out % Runs the test for 100 iterations
    ```

 * Build Millipede:
    ```
    cd Millipede
    mkdir build
    cd build
    cmake ..
    make -j
    ```

 * Run with simulated client traffic
   * First, run `./data_generator data/tddconfig-sim-ul.json` to generate
     data files
   * In one terminal, run `./millipede data/tddconfig-sim-ul.json` to start 
     Millipede with uplink configuration 
   * In another terminal, run  `./sender --num_threads=2 --core_offset=0
     --delay=5000 --enable_slow_start=false
     --conf_file=data/tddconfig-sim-ul.json` to start the sender with uplink
     configuration

 * Run with real traffic from Faros/Iris hardware UEs: See the Hardware Mode
   section below

 * Before contributing, please go over CONTRIBUTING.md.

2.2. Hardware Mode


* Note 1 : to run Millipede with Faros/Iris hardware, set `USE_ARGOS` variable 
in CMakeLists.txt to `True`.
* Note 2 : to run Client with Iris Hardware, set `ENABLE_MAC` in CMakeLists.txt 
to `True`.

Uplink Demo:

* Flash the *client* Iris device with proper image:

  * Download the image bundle from 
  https://files.sklk.us/release/universal_2020.04.0.1-3-c9adc42.tar.gz
  * Unpack the tarball and the one inside: 
  `bootfiles-iris030_ue-2020.04.0.1-3-c9adc42.tar.gz`
  * Copy `BOOT.BIN` and `image.ub` files to the SD card of you Iris.
  * Alternatively, you can transfer the files over the network (with Iris on 
  and discoverable):

	scp BOON.BIN image.up sklk@IrisIPAdress:~
	ssh sklk@IrisIPAddress
	sudo mount /boot
	sudo cp BOON.BIN image.ub /boot
	sudo sync
	sudo umount /boot
	sudo reboot

**Note:** Default login password to Iris is `sklk`

Running Client App on Server 2:

* Disable `kConnectUDP` in `src/common/Symbols.hpp`
* Modify `data/user-iris-serials.txt` by adding 2 client Iris serials in your 
setup.
* Run `./data_generator data/ue-ul-hw.json` to generate required data files
* Run `./build/user data/ue-ul-hw.json`
* Run `./build/macuser 2 5000 data/ue-ul-hw.json`

Running Millipede on Server 1:

* Re-build code with `kExportConstellation` enabled in `src/common/Symbols.hpp`
* Modify `data/bs-iris-serials.txt` and `data/bs-hub-serial.txt` by adding Iris 
serials in your Faros RRHs.
* scp over the generated file `data/orig_data_512_ant2.bin` from server 2 to 
`data` directory.
* Run `python mm_gui.py data/bs-ul-hw.json`
 

3. Other information

* millipede.cpp is the file that controls most things (performs FFT, ZF, and 
demodulation). 
* Debug information settings are in Symbols.hpp
* test/test_millipede is used for correctness test
  * The sender sends 1 frame, Millipede processes it and compares results with 
  ground truth data.
  * Gound truth data is produced by MATLAB file generate_data_dl.m. 
