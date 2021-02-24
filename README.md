[![Build Status](https://falcon.ecg.rice.edu:443/buildStatus/icon?job=github_public_agora%2Fdevelop)](https://falcon.ecg.rice.edu:443/job/github_public_agora/job/develop/)

Agora is a complete software realization of real-time massive MIMO baseband processing. 

Some highlights:

* Agora currently supports 64x16 MU-MIMO (64 RRU antennas and 16 UEs) with 20 MHz bandwidth and 64QAM modulation, on a 36-core server with AVX512 support. 
* Agora is configurable in terms of numbers of RRU antennas and UEs, bandwidth, moduation orders, LDPC code rates.
* Agora supports an emulated RRU and UEs with a high-performance packet generator.
* Agora has been tested with real RRUs with up to 64 antennas and up to 8 UEs. The RRU and UE devices are available from 
[Skylark Wireless](https://skylarkwireless.com). 

## Contents
 * [Building Agora](#building-agora)
   * [Setting up the build environment](#setting-up-the-build-environment)
   * [Building and running with emulated RRU](#building-and-running-with-emulated-rru)
   * [Building and running with real RRU](#building-and-running-with-real-rru)
   * [Running performance test](#running-performance-test)
 * [Contributing to Agora](#contributing-to-agora)
 * [Acknowledgment](#acknowledgment)
 * [Documentation](#documentation)
 * [Contact](#contact)
 
 
# Building Agora
  Agora currently only builds and runs on Linux, and has been tested on Ubuntu 16.04, 18.04, and 20.04. 
  Agora requires CMake 2.8+ and works with both GNU and Intel compilers with C++17 support. 
## Setting up the build environment
  * Setup CI: run `./config_ci.sh`
     * Note for developers: You must run this command before checking out your new feature brach. Do not use `_` in your branch name. Use `-` instead.  
    
  * See `scripts/ubuntu.sh` for required packages, including Linux packages, gtest, Armadillo, nlohmann json-dev and SoapySDR, and the corresponding versions. Run `./scripts/ubuntu.sh` to install these packages.
  * Download and install Intel libraries:
     * Install Intel FlexRAN's FEC SDK for LDPC encoding and decoding
        * Download [Intel FlexRAN's FEC
           SDK](https://software.intel.com/en-us/articles/flexran-lte-and-5g-nr-fec-software-development-kit-modules)
           for LDPC decoding to `/opt`.
        * Compiling FlexRAN requires an Intel compiler. Intel compiler version <= 19.0.4 is required for compiling FlexRAN.
          Newer versions will not work. 
          * Intel compiler can be installed from Intel Parallel Studio XE or Intel System Studio. 
	  We have tested version 2019 initial release of Intel Prallel Studio XE. 
          * Newer versions of Intel compiler can also work, but require a patch for resolving conflicts with FlexRAN. 
	  Please [contact](#contact) the current Agora developers to get the patch.
        * Set required environment variables by sourcing `compilervars.sh`.
          For example, if Intel compiler is in `/opt`, run `source $(find
          2>/dev/null /opt -name compilervars.sh) intel64`. After running this
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
    * Install Intel MKL - See
       [instructions](https://software.intel.com/content/www/us/en/develop/articles/installing-intel-free-libs-and-python-apt-repo.html).
       * MKL can also be installed from Intel Parallel Studio XE. Agora has been tested with 2019 and 2020 versions. 
       * **NOTE**: To enable JIT acceleration applied for matrix multiplication in the code, MKL version after 2019 update 3 is required.

    * Optional: DPDK
       * [DPDK](http://core.dpdk.org/download/) verison 20.02.1 is tested with
        Intel 40 GbE and Mellanox 100 GbE NICs in Agora.
       * To install it, run `sudo make install T=x86_64-native-linuxapp-gcc
        DESTDIR=/usr -j`

## Building and running with emulated RRU
We provide a high performance [packet generator](simulator) to emulate the RRU. This generator allows Agora to run and be tested without actual RRU hardware. The following are steps to set up both Agora and the packet generator.

 * Build Agora. This step also builds the sender, a data generator that generates random input data files, an end-to-end test that checks correctness of end results for both uplink and downlink, and several unit tests for testing either performance or correctness of invididual functions.
    ```
    cd Agora
    mkdir build
    cd build
    cmake ..
    make -j
    ```

 * Run end-to-end test to check correctness (uplink, downlink and combined tests should all pass if everything is set up correctly).
    ```
    ./test/test_agora/test_agora.sh 10 out % Runs test for 10 iterations
    ```

 * Run Agora with emulated RRU traffic
   * **NOTE**: We recommend running Agora and the emulated RRU on two different machines. 
   If you are running them on the same machine, make sure Agora and the emulated RRU are using different set of cores, 
     otherwise there will be performance slow down.
   * First, return to the base directory (`cd ..`), then run
     `./build/data_generator --conf_file data/tddconfig-sim-ul.json` to generate data
     files.
   * In one terminal, run `./build/agora data/tddconfig-sim-ul.json` to
     start Agora with uplink configuration.
   * In another terminal, run  `./build/sender --num_threads=2 --core_offset=1 --frame_duration=5000 --enable_slow_start=1 --conf_file=data/tddconfig-sim-ul.json` to start the emulated RRU 
     with uplink configuration.
   * The above steps use Linux networking stack for packet I/O. Agora also supports using DPDK
   to bypass the kernel for packet I/O. To enable DPDK, run `cmake -DUSE_DPDK=1 ..; make -j` to 
   rebuild code for Mellanox NICs; for Intel NICs, run `cmake -DUSE_DPDK=1 -DUSE_MLX_NIC=0 ..; make -j` 
   to exclude Mellanox libraries in the build.
   When running the emulated RRU with DPDK, it is required to set the MAC address
     of the NIC used by Agora. To do this, pass `--server_mac_addr=` to
     `./build/sender`.
   * To test the real-time performance of Agora, see the [Running performance test](#running-performance-test) section below.

 * Run Agora with channel simulator and clients
   * First, return to the base directory (`cd ..`), then run
     `./build/data_generator --conf_file data/bs-sim.json` to generate data files.
   * In one terminal, run `./build/user data/ue-sim.json` to start clients with
     uplink configuration.
   * In another terminal, run  `./build/chsim --bs_threads 1 --ue_threads 1 --worker_threads 2 --core_offset 24 --bs_conf_file data/bs-sim.json --ue_conf_file data/ue-sim.json`
   * In another terminal, run `./build/agora data/bs-sim.json` to start
     Agora with uplink configuration.
   * Note: make sure Agora and sender are using different set of cores,
     otherwise there will be performance slow down.

 * To run with real wireless traffic from Faros/Iris hardware UEs, see the
   [Agora with real RRU](#agora-with-real-rru) section below.

## Building and running with real RRU
Agora suports a 64-antenna 
Faros base station as RRU and Iris UE devices. Both are commercially available from 
[Skylark Wireless](https://skylarkwireless.com) and are used in the [POWER-RENEW PAWR testbed](https://powderwireless.net/).
Both Faros and Iris have their roots in the [Argos massive MIMO base station](https://www.yecl.org/argos/), especially [ArgosV3](https://www.yecl.org/argos/pubs/Shepard-MobiCom17-Demo.pdf). Agora also supports USRP-based RRU and UEs.

We use command line variables of cmake to switch between emulated RRU and real RRU. 
We use `-DUSE_AGROS` for Faros RRU and Iris UEs, and `-DUSE_UHD` for USRP-based RRU and UEs. 

Currently, Agora only supports uplink with real RRU and UEs. 
We recommend using one server for controlling the RRU and running Agora, 
and another server for controlling the UEs and running the UE code.
Below we describe how to get the uplink demo work.
 * Rebuild the code on both servers for RRU side the UE side.
    * For Faros RRU and Iris UEs, pass `-DUSE_ARGOS=on -DUSE_UHD=off` to cmake
    * For USRP-based RRU and UEs, pass `-DUSE_ARGOS=off -DUSE_UHD=on` to cmake 
    * Run `make -j` to recompile the code.
 * Run the UE code on the server connected to the Iris UEs
   * Modify `data/user-iris-serials.txt` by adding serials of two client Irises
     from your setup.
   * Run `./build/data_generator --conf_file data/ue-ul-hw.json` to generate required data files.
   * Run `./build/user data/ue-ul-hw.json`.
 * Run Agora on the server connected to the Faros RRU
   * scp over the generated file `data/orig_data_512_ant2.bin` from the client
     machine to the server's `data` directory.
   * Rebuild the code
     * Set `kPrintPhyStats = true` in `src/common/Symbols.hpp`, if you wish to see uplink BER results.
     * Run `make -j` to recompile the code.
   * Modify `data/bs-iris-serials.txt` and `data/bs-hub-serial.txt` by adding
     serials of your RRU Irises and hub, respectively.
   * Run `./build/agora data/bs-ul-hw.json`.

## Running performance test
To test the real-time performance of Agora for processing 64x16 MU-MIMO with 20 MHz bandwidth and 64QAM modulation,
we recommend using two servers 
(one for Agora and another for the emulated RRU) and DPDK for networking. 
In our experiments, we use 2 servers each with 4 Intel Xeon Gold 6130 CPUs. 
The servers are connected by 40 GbE Intel XL710 dual-port NICs. 

* **NOTE**: We recommend using at least 10 GbE NIC and a server with more than 10 cores 
for testing real-time performance of 8x8 MU-MIMO. 
For 8x8 MU-MIMO, our test on a machine with AVX-512 and CPU frequency of 2.3 GHz support shows that at least 7 worker cores are required to achieve real-time performance. Additionally, Agora requires one core for the manager thread and at least 1 core for network threads. We change "worker_thread_num" and "socket_thread_num" to change the number cores assigned to of worker threads and network threads in the json files, e.g., data/tddconfig-sim-ul.json. 
If you do not have a powerful server or high throughput NICs, 
we recommend increasing the value of `--frame_duration` when you run `./build/sender`, 
which will increase frame duration and reduce throughput.

To process 64x16 MU-MIMO in real-time, we use both ports of 40 GbE Intel XL710 NIC with DPDK
to get enough throughput for the traffic of 64 antennas. 
(**NOTE**: For 100 GbE NIC, we just need to use one port to get enough thoughput.)

To reduce performance variations, we did the following configurations for the server that runs Agora:
  * **NOTE**: These steps are not strictly required if you just wanted to try out Agora and do not care about performance variations.
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
	  We provide an example bash script (scripts/set_smp_affinity.sh), 
    where the IRQ indices are machine dependent.
    
The steps to collect and analyze timestamp traces are as follows:
  * For Intel NICs, recompile code with `cmake -DUSE_DPDK=1 -DUSE_MLX_NIC=0 ..; make -j`; 
  For Mallenox NICs, recompile code with `cmake -DUSE_DPDK=1 -DUSE_MLX_NIC=1 ..; make -j`.
  * We use data/tddconfig-sim-ul.json for uplink experiments and data/tddconfig-sim-dl.json for downlink experiments. 
    In our [paper](#documentation), we change “antenna_num”,  “ue_num” and “symbol_num_perframe” 
    to different values to collect different data points in the figures. 
  * Gerenrate source data files by running
     `./build/data_generator --conf_file data/tddconfig-sim-ul.json`.
  * Run Agora as a real-time process (to prevent OS from doing context swithes) using 
    `sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} chrt -rr 99 ./build/agora data/tddconfig-sim-ul.json`. 
    (**NOTE**: Using a process priority 99 is dangerous. Before running it, 
    make sure you have directed OS interrupts away from cores used by Agora. If you have not done so,
    run `sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} ./build/agora data/tddconfig-sim-ul.json` instead to run Agora as a normal process.)
  * Run the emulated RRU using `sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} ./build/sender --server_mac_addr=00:00:00:00:00:00 --num_threads=2 --core_offset=0 --conf_file=data/tddconfig-sim-ul.json --delay=1000 --enable_slow_start=$2`. 
  * The timestamps will be saved in data/timeresult.txt after Agora finishes processing. We can then use a [MATLAB script](matlab/parsedata_ul.m) to process the timestamp trace. 
  * We also provide MATLAB scripts for [uplink](matlab/parse_multi_file_ul) and [downlink](matlab/parse_multi_file_dl) that are able to process multiple timestamp files and generate figures reported in our [paper](#documentation).

## Contributing to Agora
Agora is open-source and open to your contributions. Before contributing, please read [this](CONTRIBUTING.md).

## Acknowledgment
Agora was funded in part by NSF Grant #1518916 and by the NSF PAWR project.

## Documentation
Check out [Agora Wiki](https://github.com/jianding17/Agora/wiki) for 
Agora's design overview and flow diagram that maps massive MIMO baseband processing 
to the actual code structure. Technical details and performance results can be found in
 * Jian Ding, Rahman Doost-Mohammady, Anuj Kalia, and Lin Zhong, "Agora: Real-time massive MIMO baseband processing in software," in Proc. of ACM CoNEXT, December 2020 ([PDF](https://www.yecl.org/publications/ding2020conext.pdf), [video](https://dl.acm.org/doi/abs/10.1145/3386367.3431296)).
 
## Contact
Jian Ding (jian.ding@yale.edu)
