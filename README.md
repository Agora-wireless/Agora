Millipede is a high-performance system for massive-MIMO baseband processing.

## Requirements
 * Toolchain: A C++11 compiler and CMake 2.8+. Enabling LDPC decoding requires
   an Intel C++ compiler (`icpc`).
 * Required packages
   * `sudo apt -y install liblapack-dev libblas-dev libboost-all-dev doxygen nlohmann-json-dev python-numpy python-pyqt5 libgflags-dev`
   * Install Intel MKL (see [instructions](https://software.intel.com/content/www/us/en/develop/articles/installing-intel-free-libs-and-python-apt-repo.html))
   * Install Armadillo: `./scripts/install_armadillo.sh`
   * Install the latest version of SoapySDR: `./scripts/install_soapysdr.sh`
   * Download Intel FlexRAN's LDPC SDK to `/opt` (does not need to be compiled)
     * Download [link](https://software.intel.com/en-us/articles/flexran-lte-and-5g-nr-fec-software-development-kit-modules)
     * Compile FlexRAN's LDPC SDK:
     ```
     % First, change ownership of /opt/FlexRAN_FEC_SDK_19_04 to your Linux user
     cd /opt/FlexRAN_FEC_SDK_19_04/sdk/
     ./create-makefiles-linux.sh
     sed -i '/add_compile_options("-Wall")/a \ \ add_compile_options("-fPIC")' cmake/intel-compile-options.cmake
     cd build-avx512-icc % or build-avx2-icc
     make
     ```
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

 * Build Millipede
    ```
    cd Millipede
    mkdir build
    cd build
    cmake ..
    make -j
    ```

 * To include LDPC in the build, 
   * Make sure to enable Intel compiler as instructed above.
   * Instead of `cmake ..` above, run `cmake -DCMAKE_C_COMPILER=icc -DCMAKE_CXX_COMPILER=icpc -DUSE_LDPC=1 ..`.

 * Run Millipede with simulated client traffic
   * First, run `./data_generator data/tddconfig-sim-ul.json` to generate data
     files.
   * In one terminal, run `./millipede data/tddconfig-sim-ul.json` to start
     Millipede with uplink configuration.
   * In another terminal, run  `./sender --num_threads=2 --core_offset=0
     --delay=5000 --enable_slow_start=false
     --conf_file=data/tddconfig-sim-ul.json` to start the simulated traffic
     sender with uplink configuration.

 * To run with real wireless traffic from Faros/Iris hardware UEs, see the
   "Hardware mode" section below.

 * Before contributing, please go over CONTRIBUTING.md

## Running with hardware UEs

This section provides instructions for generating and processing real wireless
traffic with hardware UEs (e.g., Iris devices)

### Prepare the Iris UE devices

 * Flash the client Iris UE device with the proper image
   * Download the image
     [bundle]https://files.sklk.us/release/universal_2020.04.0.1-3-c9adc42.tar.gz
   * Unpack the tarball `bootfiles-iris030_ue-2020.04.0.1-3-c9adc42.tar.gz` and
     the one inside.
   * Copy `BOOT.BIN` and `image.ub` files to the SD card of you Iris.
     Alternatively, you can transfer the files over the network with Iris on
     and discoverable.
   * The IPv6 address of the Iris looks like `fe80::3b3b:21ee:fd81:687%2`.
     Default username and password to Iris devices is {`sklk`, `sklk`}.

   * In the Iris UE device
    ```
      scp BOOT.BIN image.up sklk@IrisIPAdress:~
      ssh sklk@IrisIPAddress
      sudo mount /boot
      sudo cp BOOT.BIN image.ub /boot
      sudo sync
      sudo umount /boot
      sudo reboot
    ```

### Run the uplink demo

 * Run the client on a machine connected to the Iris UEs
   * Rebuild the code
     * Set `kConnectUDP = false` in `src/common/Symbols.hpp`
     * Pass `-DENABLE_MAC=on` to cmake
   * Modify `data/user-iris-serials.txt` by adding serials of two client Irises
     from your setup.
   * Run `./build/data_generator data/ue-ul-hw.json` to generate required data files
   * Run `./build/user data/ue-ul-hw.json`
   * Run `./build/macuser 2 5000 data/ue-ul-hw.json`

 * Run Millipede on the server
   * scp over the generated file `data/orig_data_512_ant2.bin` from the client
     machine to the server's `data` directory.
   * Rebuild the code
     * Set `kExportConstellation = true` in `src/common/Symbols.hpp`
     * Pass `-DUSE_ARGOS=on` to cmake
   * Modify `data/bs-iris-serials.txt` and `data/bs-hub-serial.txt` by adding
    serials of your RRU Irises and hub, respectively. Iris serials in your
    Faros RRHs.
   * Run `python mm_gui.py data/bs-ul-hw.json`
