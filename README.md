Agora is a high-performance system for real-time massive MIMO baseband processing. 

Some highlights:

* Agora currently supports baseband processing of up to 64 RRU antennas, 16 UEs, 20 MHz bandwidth, and 64QAM modulation. A 36-core server with AVX512 support is sufficient to run Agora under this maximum configuration.
* Agora can support vaious configurations: different numbers of RRU antennas and UEs, different bandwidth, different moduation orders, different LDPC code rates.
* Agora can work without real hardware. A high-performance packet generator is implemented to emulate a massive MIMO RRU.
* Agora has been tested with real RRU with up to 64 antennas and up to 8 UEs. The RRU and UE devices are available from 
[Skylark Wireless](https://skylarkwireless.com). 

Before contributing, please go over [CONTRIBUTING.md](CONTRIBUTING.md)

## Contents

 * [Requirements for building Agora](#requirements-for-building-agora)
 * [Agora with emulated RRU](#agora-with-emulated-rru)
   * [Building and running Agora](#building-and-running-agora)
   * [Server setup for performance test](#server-setup-for-performance-tests)
   * [Running performance test](#running-performance-test)
 * [Agora with real RRU and UEs](#agora-with-real-rru-and-ues)
   * [Running the uplink demo](#running-the-uplink-demo)
 * [Acknowledgment](#acknowledgment)
 * [Dodumentation](#documentation)
 * [Contact](#contact)
 
 
## Requirements for building Agora
Agora can be built with the following setup.

 * Toolchain: A C++11 compiler and CMake 2.8+.
 * Operating system: Linux (Ubuntu 16.04 and 18.04 are tested)
 * Required packages
   * Install required Ubuntu libraries, Armadillo, nlohmann json-dev and SoapySDR: ./scripts/install/ubuntu.sh.
   * Download and install Intel libraries:
     * Install Intel MKL - See
       [instructions](https://software.intel.com/content/www/us/en/develop/articles/installing-intel-free-libs-and-python-apt-repo.html).
       * MKL can also be installed from Intel Parallel Studio XE, please reach out to the current
         Agora developers to learn how to get the correct version. 
       * Note: MKL version after 2019 update 3 is required to enable JIT acceleration applied for matrix multiplication in the code.
     * Install Intel FlexRAN's FEC SDK for LDPC encoding and decoding
        * Download [Intel FlexRAN's FEC
           SDK](https://software.intel.com/en-us/articles/flexran-lte-and-5g-nr-fec-software-development-kit-modules)
           for LDPC decoding to `/opt`.
        * Compiling FlexRAN requires an Intel compiler. Intel compiler version <= 19.0.4 is required for compiling FlexRAN.
          Newer versions will not work. 
          * Please reach out to the current
          Agora developers to learn how to get the correct versions of
          Intel Parallel Studio XE or Intel System Studio. Version 2019 initial 
          release of Intel Prallel Studio XE has been tested to work. 
          * If you are using a newer version of icc, please reach out to the current
          Agora developers to get the patch for resolving conflicts with FlexRAN.
        * Set required environment variables by sourcing `compilervars.sh`.
          For example, if Intel compiler is in `/opt`, run `source $(find
          2>/dev/null/opt -name compilervars.sh) intel64`. After running this
          command, ensure that `icc --version` reports 19.0.4.
        * After instaling `icc 19.04`, compile FlexRAN as follows:
        ```
        sudo chmod -R a+rwX FlexRAN-FEC-SDK-19-04/ # Allow all users read-write access 
        cd /opt/FlexRAN-FEC-SDK-19-04/sdk/ 
        sed -i '/add_compile_options("-Wall")/a \ \ add_compile_options("-ffreestanding")' cmake/intel-compile-options.cmake 
        ./create-makefiles-linux.sh 
        cd build-avx512-icc # or build-avx2-icc 
        make -j
        ```

   * Optional: DPDK
      * [DPDK](http://core.dpdk.org/download/) verison 20.02.1 is tested with
        Intel 40 GbE and Mellanox 100 GbE NICs in Agora.
      * To install it, run `sudo make install T=x86_64-native-linuxapp-gcc
        DESTDIR=/usr -j`

## Agora with emulated RRU
We provide a high performance [packet generator](simulator) to emulate the RRU. This generator allows Agora to run and be tested without actual RRU hardware. The following are steps to set up both Agora and the packet generator.

### Builing and running Agora
 * Build Agora. This step also builds the sender, a data generator that generates random input data files, an end-to-end test that checks correctness of end results for both uplink and downlink, and several unit tests for testing either performance or correctness of invididual functions.
    ```
    cd Agora
    mkdir build
    cd build
    cmake ..
    make -j
    ```

 * Run end-to-end test (uplink and downlik tests should both pass if everything is set up correctly).
    ```
    ./test/test_agora/test_agora.sh 100 out % Runs test for 100 iterations
    ```

 * Run Agora with simulated RRU traffic
   * First, return to the base directory (`cd ..`), then run
     `./build/data_generator --conf_file data/tddconfig-sim-ul.json` to generate data
     files.
   * In one terminal, run `./build/agora data/tddconfig-sim-ul.json` to
     start Agora with uplink configuration.
   * In another terminal, run  `./build/sender --num_threads=2 --core_offset=0
     --frame_duration=5000 --enable_slow_start=1 
     --conf_file=data/tddconfig-sim-ul.json` to start the simulated RRU 
     with uplink configuration.
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
     `./build/data_generator --conf_file data/bs-ul-sim.json` to generate data files.
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
   "Agora with real RRU and UEs" section below.
 
 
### Server setup for performance test
To test the performance of Agora, we recommend using two servers 
(one for Agora and another for the sender) and DPDK for networking. 
In our experiments, we use 2 servers each with 4 Intel Xeon Gold 6130 CPUs. 
The servers are connected by 40 GbE Intel XL710 dual-port NICs. 
We use both ports with DPDK to get enough throughput for the traffic of 64 antennas. 
We did the following server configurations for the server that runs Agora
  * Disable Turbo Boost to reduce performance variation by running 
   `echo "0" | sudo tee /sys/devices/system/cpu/cpufreq/boost`
  * Set CPU scaling to performance by running 
    `sudo cpupower frequency-set -g performance`, 
    where cpupower can be installed through
    `sudo apt-get install -y linux-tools-$(uname -r)`
  * Turn off hyper-threading. We provide an example bash script 
	 (scripts/tune_hyperthread.sh), where the core indices are machine dependent.
  * Set IRQ affinity to direct OS interrupts away from Agora's cores. 
    We direct all the interrupts to core 0 in our experiments.  
	  We provide an example bash script (scripts/system/set_smp_affinity.sh), 
    where the IRQ indices are machine dependent.
  * Compile code with `cmake -DUSE_DPDK=1 -DUSE_MLX_NIC=0 ..; make -j`.
    
### Running performance test
In this section, we provide the instruction to collect and analyze timestamp traces for emulated RRU. 
  * We use data/tddconfig-sim-ul.json for uplink experiments and data/tddconfig-sim-dl.json for downlink experiments. 
    In our [paper](#documentation), we change “antenna_num”,  “ue_num” and “symbol_num_perframe” 
    to different values to collect different data points in the figures. 
  * Run Agora as a real-time process (to prevent OS from doing context swithes) using 
    `sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} chrt -rr 99 ./build/agora data/tddconfig-sim-ul.json`. 
    (Note: using a process priority 99 is dangerous. Before running it, 
    make sure you have direct OS interrupts used by Agora's cores.)
  * Run emulated RRU using `sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} ./build/sender --server_mac_addr=00:00:00:00:00:00 --num_threads=2 --core_offset=0 --conf_file=data/tddconfig-sim-ul.json --delay=1000 --enable_slow_start=$2`. 
  * The timestamps will be saved in data/timeresult.txt after Agora finishes processing. We can then use a [MATLAB script](matlab/parsedata_ul.m) to process the timestamp trace. 
  * We also provide MATLAB scripts for [uplink](matlab/parse_multi_file_ul) and [downlink](matlab/parse_multi_file_dl) that are able to process multiple timestamp files and generate figures reported in our [paper](#documentation).

## Agora with real RRU and UEs

Currently Agora suports a 64-antenna 
Faros base station as RRU and Iris UE devices. Both are commercially available from 
[Skylark Wireless](https://skylarkwireless.com) and are used in the [POWER-RENEW PAWR testbed](https://powderwireless.net/).
Both Faros and Iris have their roots in the [Argos massive MIMO base station](https://www.yecl.org/argos/), especially [ArgosV3](https://www.yecl.org/argos/pubs/Shepard-MobiCom17-Demo.pdf). Agora also supports USRP-based RRU and UEs. 
We use command line variables of cmake to switch between emulated RRU and real RRU. 
We use `-DUSE_AGROS` for Faros RRU and Iris UEs, and `-DUSE_UHD` for USRP-based RRU and UEs. 

Below we describe how to get it to work with Faros RRU and Iris UEs.

### Running the uplink demo

 * Run the UE code on a machine connected to the Iris UEs
   * Rebuild the code
     * Pass `-DUSE_ARGOS=on -DUSE_UHD=off -DENABLE_MAC=on` to cmake
     * For USRP-based RRU and UEs, pass `-DUSE_ARGOS=off -DUSE_UHD=on -DENABLE_MAC=on` to cmake 
   * Modify `data/user-iris-serials.txt` by adding serials of two client Irises
     from your setup.
   * Run `./build/data_generator --conf_file data/ue-ul-hw.json` to generate required data files.
   * Start client app `./python/client_app.py`.
   * Run `./build/user data/ue-ul-hw.json`.

 * Run Agora on the server
   * Recompile FlexRAN with `-fPIC` to allow using from Python
     ```
     cd /opt/FlexRAN-FEC-SDK-19-04/sdk/
     sed -i '/add_compile_options("-Wall")/a \ \ add_compile_options("-fPIC")' cmake/intel-compile-options.cmake
     ./create-makefiles-linux.sh
     cd build-avx512-icc # or build-avx2-icc
     make -j
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
Check out [Agora Wiki](https://github.com/jianding17/Agora/wiki) for 
Agora's design overview and flow diagram that maps massive MIMO baseband processing 
to the actual code structure. Technical details and performance results can be found in
 * Jian Ding, Rahman Doost-Mohammady, Anuj Kalia, and Lin Zhong, "Agora: Software-based real-time massive MIMO baseband," to appear in Proc. of ACM CoNEXT, November 2020.
 
## Contact
Jian Ding (jian.ding@yale.edu)
