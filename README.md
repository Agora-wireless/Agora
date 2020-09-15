Agora is a high-performance system for massive-MIMO baseband processing.

## Requirements
 * Toolchain: A C++11 compiler and CMake 2.8+.
 * Required packages
   * `sudo apt -y install liblapack-dev libblas-dev libboost-all-dev doxygen
     nlohmann-json-dev python-numpy python-pyqt5 libgflags-dev`
     * If `nlohmann-json-dev` package can't be found, get code from
       [here](https://github.com/nlohmann/json) and build from source.

   * Install Armadillo: `./scripts/install_armadillo.sh`.
   * Install the latest version of SoapySDR: `./scripts/install_soapysdr.sh`.
   * Download and install Intel libraries:
     * Install Intel MKL - See
       [instructions](https://software.intel.com/content/www/us/en/develop/articles/installing-intel-free-libs-and-python-apt-repo.html).
     * Download [Intel FlexRAN's FEC
       SDK](https://software.intel.com/en-us/articles/flexran-lte-and-5g-nr-fec-software-development-kit-modules)
       for LDPC decoding to `/opt`.
     * Compiling FlexRAN requires an Intel compiler.
        * For Agora developers: Please ask internally for precompiled
          FlexRAN libraries to use from `gcc`.
        * For Agora users:
          * Intel compiler version 19.0.4 is required for compiling FlexRAN.
            Newer versions will not work. Please reach out to the current
            Agora developers to learn how to get the correct versions of
            Intel Parallel Studio XE or Intel System Studio.
          * Set required environment variables by sourcing `compilervars.sh`.
            For example, if Intel compiler is in `/opt`, run `source $(find
            2>/dev/null/opt -name compilervars.sh) intel64`. After running this
            command, ensure that `icc --version` reports 19.0.4.
          * After instaling `icc 19.04`, compile FlexRAN as follows:
          ```
          sudo chmod -R a+rwX FlexRAN-FEC-SDK-19-04/ % Allow all
          users read-write access cd /opt/FlexRAN-FEC-SDK-19-04/sdk/ sed -i
          '/add_compile_options("-Wall")/a \ \
          add_compile_options("-ffreestanding")'
          cmake/intel-compile-options.cmake ./create-makefiles-linux.sh cd
          build-avx512-icc % or build-avx2-icc make
          ```

   * Optional: DPDK
      * [DPDK](http://core.dpdk.org/download/) verison 20.02.1 is tested with
        Intel 40 GbE and Mellanox 100 GbE NICs in Agora.
      * To install it, run `sudo make install T=x86_64-native-linuxapp-gcc
        DESTDIR=/usr -j`

## Agora quickstart

 * Build Agora
    ```
    cd Agora
    mkdir build
    cd build
    cmake ..
    make -j
    ```

 * Run end-to-end tests
    ```
    ./test/test_agora/test_agora.sh 100 out % Runs test for 100 iterations
    ```

 * Run Agora with simulated client traffic
   * First, return to the base directory (`cd ..`), then run
     `./build/data_generator data/tddconfig-sim-ul.json` to generate data
     files.
   * In one terminal, run `./build/agora data/tddconfig-sim-ul.json` to
     start Agora with uplink configuration.
   * In another terminal, run  `./build/sender --num_threads=2 --core_offset=0
     --delay=5000 --enable_slow_start=true
     --conf_file=data/tddconfig-sim-ul.json` to start the simulated traffic
     sender with uplink configuration.
   * Note: make sure Agora and sender are using different set of cores, 
     otherwise there will be performance slow down.

 * Run Agora with DPDK
   * Run `cmake -DUSE_DPDK=1` to enable DPDK in the build.
   * For Intel NICs, run `cmake -DUSE_DPDK=1 -DUSE_MLX_NIC=0` to exclude
     Mellanox libraries in the build.
   * When running the sender with DPDK, it is required to set the MAC address
     of the NIC used by Agora. To do this, pass `--server_mac_addr=` to
     `./build/sender`.

 * Run Agora with channel simulator and clients
   * First, return to the base directory (`cd ..`), then run
     `./build/data_generator data/bs-ul-sim.json` to generate data files.
   * In one terminal, run `./build/user data/ue-ul-sim.json` to start clients with
     uplink configuration.
   * In another terminal, run  `./build/chsim --bs_threads 1 --ue_threads 1
     --worker_threads 2 --core_offset 24 --bs_conf_file data/bs-ul-sim.json
     --ue_conf_file data/ue-ul-sim.json`
   * In another terminal, run `./build/agora data/bs-ul-sim.json` to start
     Agora with uplink configuration.
   * Note: make sure Agora and sender are using different set of cores,
     otherwise there will be performance slow down.

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
     * Pass `DUSE_ARGOS=on` and `-DENABLE_MAC=on` to cmake
   * Modify `data/user-iris-serials.txt` by adding serials of two client Irises
     from your setup.
   * Run `./build/data_generator data/ue-ul-hw.json` to generate required data files.
   * Start client app `./python/client_app.py`.
   * Run `./build/user data/ue-ul-hw.json`.

 * Run Agora on the server
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
     * Pass `-DUSE_ARGOS=on` and `-DENABLE_MAC=on` to cmake
   * Modify `data/bs-iris-serials.txt` and `data/bs-hub-serial.txt` by adding
     serials of your RRU Irises and hub, respectively. Iris serials in your
     Faros RRHs.
   * Run BS app `./python/bs_app.py`.
   * Run `./build/agora data/bs-ul-hw.json`.

## Acknowledgment
Agora was funded in part by NSF Grant #1518916 and by the NSF PAWR project.

## Documentation
Technical details and performance results can be found in
 * Jian Ding, Rahman Doost-Mohammady, Anuj Kalia, and Lin Zhong, "Agora: Software-based real-time massive MIMO baseband," to appear in Proc. of ACM CoNEXT, November 2020.
 
## Contact
Jian Ding (jian.ding@yale.edu)
