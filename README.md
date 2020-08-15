Millipede is a high-performance system for massive-MIMO baseband processing.

## Requirements
 * Toolchain: A C++11 compiler and CMake 2.8+.
 * Required packages
   * `sudo apt -y install liblapack-dev libblas-dev libboost-all-dev doxygen nlohmann-json-dev python-numpy python-pyqt5 libgflags-dev ibverbs-providers`
   * Install Intel MKL (see [instructions](https://software.intel.com/content/www/us/en/develop/articles/installing-intel-free-libs-and-python-apt-repo.html))
   * Install Armadillo: `./scripts/install_armadillo.sh`
   * Install the latest version of SoapySDR: `./scripts/install_soapysdr.sh`
   * Download Intel FlexRAN's FEC SDK for LDPC decoding to `/opt`.
     [Link](https://software.intel.com/en-us/articles/flexran-lte-and-5g-nr-fec-software-development-kit-modules).
   * Getting FlexRAN FEC SDK libraries:
     * Compiling FlexRAN requires an Intel compiler. For Millipede developers:
       please ask internally for precompiled FlexRAN libraries to use from `gcc`.
     * After instaling `icc 19.04` (see instructions below), compile FlexRAN:
     ```
     sudo chmod -R a+rwX FlexRAN-FEC-SDK-19-04/ % Allow all users read-write access
     cd /opt/FlexRAN-FEC-SDK-19-04/sdk/
     sed -i '/add_compile_options("-Wall")/a \ \ add_compile_options("-ffreestanding")' cmake/intel-compile-options.cmake
     ./create-makefiles-linux.sh
     cd build-avx512-icc % or build-avx2-icc
     make
     ```

   * Optional: Install Intel compiler
     * Intel compiler version 19.0.4 is required for compiling FlexRAN. Newer
       versions will not work. Please reach out to one of the current Millipede
       developers to learn how to get the correct versions of Intel Parallel
       Studio XE or Intel System Studio.

     * Set required environment vairables by sourcing `compilervars.sh`. For
       example, if Intel compiler is in `/opt`, run `source $(find 2>/dev/null
       /opt -name compilervars.sh) intel64`. After running this command, ensure
       that `icc --version` reports 19.0.4.

   * Optinal: DPDK 
     * [DPDK](http://core.dpdk.org/download/) verison 20.02.1 is tested with
       Intel 40 GbE and Mellanox 100 GbE NICs in Millipede.
     * To install it, run `sudo make install T=x86_64-native-linuxapp-gcc
       DESTDIR=/usr -j`

## Millipede quickstart

 * Build Millipede
    ```
    cd Millipede
    mkdir build
    cd build
    cmake ..
    make -j
    ```

 * Run end-to-end tests
    ```
    ./test/test_millipede/test_millipede.sh 100 out % Runs test for 100 iterations
    ```

 * Run Millipede with simulated client traffic
   * First, return to the base directory (`cd ..`), then run
     `./build/data_generator data/tddconfig-sim-ul.json` to generate data
     files.
   * In one terminal, run `./build/millipede data/tddconfig-sim-ul.json` to
     start Millipede with uplink configuration.
   * In another terminal, run  `./build/sender --num_threads=2 --core_offset=0
     --delay=5000 --enable_slow_start=true
     --conf_file=data/tddconfig-sim-ul.json` to start the simulated traffic
     sender with uplink configuration.
   * Note: make sure Millipede and sender are using different set of cores, 
     otherwise there will be performance slow down.

 * Run Millipede with DPDK
   * Run `cmake -DUSE_DPDK=1` to enable DPDK in the build.
   * For Intel NICs, run `cmake -DUSE_DPDK=1 -DUSE_MLX_NIC=0` to exclude
     Mellanox libraries in the build.
   * When running the sender with DPDK, it is required to set the MAC address
     of the NIC used by Millipede. To do this, pass `--server_mac_addr=` to
     `./build/sender`.

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
     * Pass `-DENABLE_MAC=on` to cmake
   * Modify `data/user-iris-serials.txt` by adding serials of two client Irises
     from your setup.
   * Run `./build/data_generator data/ue-ul-hw.json` to generate required data files
   * Run `./build/user data/ue-ul-hw.json`
   * Run `./build/macuser 2 5000 data/ue-ul-hw.json`

 * Run Millipede on the server
   * Recompile FlexRAN with `-fPIC` to allow using from Python
     ```
     cd /opt/FlexRAN-FEC-SDK-19-04/sdk/
     sed -i '/add_compile_options("-Wall")/a \ \ add_compile_options("-fPIC")' cmake/intel-compile-options.cmake
     ./create-makefiles-linux.sh
     cd build-avx512-icc % or build-avx2-icc
     make
     ```
   * scp over the generated file `data/orig_data_512_ant2.bin` from the client
     machine to the server's `data` directory.
   * Rebuild the code
     * Set `kExportConstellation = true` in `src/common/Symbols.hpp`
     * Pass `-DUSE_ARGOS=on` to cmake
   * Modify `data/bs-iris-serials.txt` and `data/bs-hub-serial.txt` by adding
     serials of your RRU Irises and hub, respectively. Iris serials in your
     Faros RRHs.
   * Run `python mm_gui.py data/bs-ul-hw.json`
