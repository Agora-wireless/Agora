[![Build Status](https://falcon.ecg.rice.edu:443/buildStatus/icon?job=github_public_agora%2Fdevelop)](https://falcon.ecg.rice.edu:443/job/github_public_agora/job/develop/)

Agora is a complete software realization of real-time massive MIMO baseband processing. 

Some highlights:

* Agora currently supports 64x16 MU-MIMO (64 RRU antennas and 16 UEs) with 20 MHz bandwidth and 64QAM modulation, on a 36-core server with AVX512 support. 
* Agora is configurable in terms of numbers of RRU antennas and UEs, bandwidth, modulation orders, LDPC code rates.
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
  * Setup CI: run
    <pre>
    $ ./config_ci.sh
    </pre>
     * Note for developers: You must run this command before checking out your new feature branch. Do not use `_` in your branch name. Use `-` instead.  
    
  * See `scripts/ubuntu.sh` for required packages, including Linux packages, gtest, Armadillo, and SoapySDR, and the corresponding versions. Run `./scripts/ubuntu.sh` to install these packages.
  * Download and install Intel libraries:
     * Install Intel compiler and MKL, refer to [INTELLIB_README.md](INTELLIB_README.md).

     * Set required environment variables by sourcing `setvars.sh`. If oneAPI is installed in `/opt`,
     run `source /opt/intel/oneapi/setvars.sh`.   

     * Install [Intel FlexRAN's FEC SDK](https://software.intel.com/en-us/articles/flexran-lte-and-5g-nr-fec-software-development-kit-modules) for LDPC encoding and decoding:
        * **NOTE**: Compiling FlexRAN requires Intel compiler with version <= 19.0.4.
          Newer versions of Intel compiler can also work, but require a patch for resolving conflicts with FlexRAN.\
          Please [contact](#contact) the current Agora developers to get the patch.
        * Download Intel FlexRAN's FEC SDK to `/opt`.
        * Compile FlexRAN as follows:
        <pre>
        $ sudo chmod -R a+rwX FlexRAN-FEC-SDK-19-04/ # Allow all users read-write access 
        $ cd /opt/FlexRAN-FEC-SDK-19-04/sdk/ 
        $ sed -i '/add_compile_options("-Wall")/a \ \ add_compile_options("-ffreestanding")' cmake/intel-compile-options.cmake 
        $ ./create-makefiles-linux.sh 
        $ cd build-avx512-icc # or build-avx2-icc 
        $ make -j
        </pre>

    * Optional: DPDK
       * Refer to [DPDK_README.md](DPDK_README.md) for configuration and installation instructions.

## Building and running with emulated RRU
We provide a high performance [packet generator](simulator) to emulate the RRU. This generator allows Agora to run and be tested without actual RRU hardware.\
The following are steps to set up both Agora and the packet generator:

 * Build Agora. This step also builds the emulated RRU, a data generator that generates random input data files, an end-to-end test that checks correctness of end results for both uplink and downlink,\
 and several unit tests for testing either performance or correctness of individual functions.
    <pre>
    $ cd Agora
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make -j
    </pre>

 * Run end-to-end test to check correctness (uplink, downlink and combined tests should all pass if everything is set up correctly).
    <pre>
    $ ./test/test_agora/test_agora.sh 10 out # Runs test for 10 iterations
    </pre>

#### Run Agora with emulated RRU traffic
   * **NOTE**: We recommend running Agora and the emulated RRU on two different machines.\
   If you are running them on the same machine, make sure Agora and the emulated RRU are using different set of cores,
     otherwise there will be performance slow down. 
     
   When running Agora and the emulated RRU on two different machines, the following steps use Linux networking stack for packet I/O.\
     Agora also supports using DPDK to bypass the kernel for packet I/O. 
     See [DPDK_README.md](DPDK_README.md) for instructions of running emulated RRU and Agora with DPDK. 
   
   * First, return to the base directory (`cd ..`), then run
   <pre>
   $ ./build/data_generator --conf_file files/config/ci/tddconfig-sim-ul.json
   </pre>
     to generate data files.
   * In one terminal, run 
   <pre>
   $ ./build/agora --conf_file files/config/ci/tddconfig-sim-ul.json
   </pre>
    to start Agora with uplink configuration.
   * In another terminal, run
   <pre>
   $ ./build/sender --num_threads=2 --core_offset=1 --frame_duration=5000 --enable_slow_start=1 --conf_file=files/config/ci/tddconfig-sim-ul.json
   </pre>
   to start the emulated RRU with uplink configuration.
   * To test the real-time performance of Agora, see the [Running performance test](#running-performance-test) section below.

#### Run Agora with channel simulator and clients
   * First, return to the base directory (`cd ..`), then run
   <pre>
   $ ./build/data_generator --conf_file files/config/ci/chsim.json
   </pre>
    to generate data files.
   * In one terminal, run
   <pre>
   $ ./build/user --conf_file files/config/ci/chsim.json
   </pre>
     to start clients with
     combined uplink & downlink configuration.
   * In another terminal, run
   <pre>
   $ ./build/chsim --bs_threads 1 --ue_threads 1 --worker_threads 2 --core_offset 24 --conf_file files/config/ci/chsim.json
   </pre>
   * In another terminal, run
   <pre>
   $ ./build/agora --conf_file files/config/ci/chsim.json
   </pre>
   to start Agora with the combined configuration.
   * Note: make sure Agora and sender are using different set of cores, otherwise there will be performance slow down.

#### Run Agora with channel simulator, clients, and mac enabled
   * Compile the code with
   <pre>
   $ cmake .. -DENABLE_MAC=true
   </pre>
   * Uplink Testing (`--conf_file mac-ul-sim.json`)
   * Downlink Testing  (`--conf_file mac-dl-sim.json`)
   * Combined Testing  (`--conf_file mac-sim.json`)
     * Terminal 1:
     <pre>
       $./build/data_generator --conf_file files/config/examples/mac-sim.json
     </pre>
       to generate data files.
     <pre>
       $./build/user --conf_file files/config/examples/mac-sim.json
     </pre>
       to start users.
     * Terminal 2:
     <pre>
     $ ./build/chsim --bs_threads 1 --ue_threads 1 --worker_threads 2 --core_offset 28 --conf_file files/config/examples/mac-sim.json
     </pre>
       to run the channel simulator
     * Terminal 3:
     <pre>
       $ ./build/macuser --enable_slow_start 1 --conf_file files/config/examples/mac-sim.json
     </pre>
      to run to user mac app.  Specify `--data_file ""` to generate patterned data and `--conf_file` options as necessary.
     * Terminal 4:
     <pre>
     $ ./build/agora --conf_file files/config/examples/mac-sim.json
     </pre>
      run agora before running macbs.  Run macuser -> agora -> macbs in quick succession. 
     * Terminal 5:
     <pre>
     $ ./build/macbs --enable_slow_start 1 --conf_file files/config/examples/mac-sim.json
     </pre>
     to run to base station mac app. specify `--data_file ""` to generate patterned data and `--conf_file` options as necessary.
   * Note: make sure agora / user / chsim / macuser / macbs are using different set of cores, otherwise there will be performance slow down.

## Building and running with real RRU
Agora supports a 64-antenna Faros base station as RRU and Iris UE devices. Both are commercially available from [Skylark Wireless](https://skylarkwireless.com) and are used in the [POWER-RENEW PAWR testbed](https://powderwireless.net/).\
Both Faros and Iris have their roots in the [Argos massive MIMO base station](https://www.yecl.org/argos/), especially [ArgosV3](https://www.yecl.org/argos/pubs/Shepard-MobiCom17-Demo.pdf).
Agora also supports USRP-based RRU and UEs.

We recommend using one server for controlling the RRU and running Agora,
and another server for controlling the UEs and running the UE code.
 
Agora supports both uplink and downlink with real RRU and UEs. For downlink, a reference node outside the array (and synchronized) is required for reciprocity calibration.\
**Note:** Faros RRU and Iris UEs can be discovered using the [pyfaros](https://github.com/skylarkwireless/pyfaros) tool. You can use this tool to find the topology of the hardware connected to the server.

We describe how to get the uplink and downlink demos working. Below XX can be replaced with either `ul` and `dl`.
 * Rebuild the code on both servers for RRU side the UE side.
    * For Faros RRU and Iris UEs, pass `-DRADIO_TYPE=SOAPY_IRIS` to cmake
    * For USRP-based RRU and UEs, pass `-DRADIO_TYPE=SOAPY_UHD` to cmake
    * Run `make -j` to recompile the code.
 * Run the UE code on the server connected to the Iris UEs
   * For Iris UEs, run the pyfaros tool in the `files/topology` directory as follows:
     <pre>
     $ python3 -m pyfaros.discover --json-out
     </pre>
     This will output a file named `topology.json` with all the discoverable serial IDs included.
   * Modify `files/topology/topology.json` by adding/removing serials of client Irises you'd like to include
     from your setup.
   * For USRP-based RRU and UEs, modify the existing `files/topology/topology.json` and enter the appropriate IDs.
   * Run `./build/data_generator --conf_file files/config/XX-hw.json` to generate required data files.
   * Run `./build/user --conf_file files/config/XX-hw.json`.
 * Run Agora on the server connected to the Faros RRU
   * scp over the generated file `files/experiment/LDPC_orig_XX_data_512_ant2.bin` from the client
     machine to the server's `files/experiment` directory.
   * Rebuild the code
     * Run `make -j` to compile the code.
   * For Faros RRU, use the pyfaros tool the same as with the UEs to generate a new `files/topology/topology.json`
   * Modify `files/topology/topology.json` by adding/removing serials of your RRU Irises, and the hub.
   * Run `./build/agora --conf_file files/config/XX-hw.json`.

## Running performance test
To test the real-time performance of Agora for processing 64x16 MU-MIMO with 20 MHz bandwidth and 64QAM modulation, we recommend using two servers 
(one for Agora and another for the emulated RRU) and DPDK\
for networking. 
In our experiments, we use 2 servers each with 4 Intel Xeon Gold 6130 CPUs. 
The servers are connected by 40 GbE Intel XL710 dual-port NICs. 

* **NOTE**: We recommend using at least 10 GbE NIC and a server with more than 10 cores for testing real-time performance of 8x8 MU-MIMO. For 8x8 MU-MIMO, our test on a machine with AVX-512 and CPU frequency\
of 2.3 GHz support shows that at least 7 worker cores are required to achieve real-time performance. Additionally, Agora requires one core for the manager thread and at least 1 core for network threads.\

We change "worker_thread_num" and "socket_thread_num" to change the number cores assigned to of worker threads and network threads in the json files, e.g., files/config/ci/tddconfig-sim-ul.json.\
If you do not have a powerful server or high throughput NICs, we recommend increasing the value of `--frame_duration` when you run `./build/sender`, which will increase frame duration and reduce throughput.

To process 64x16 MU-MIMO in real-time, we use both ports of 40 GbE Intel XL710 NIC with DPDK (see [DPDK_README.md](DPDK_README.md))
to get enough throughput for the traffic of 64 antennas. \
(**NOTE**: For 100 GbE NIC, we just need to use one port to get enough thoughput.)

To reduce performance variations, we did the following configurations for the server that runs Agora:
  * **NOTE**: These steps are not strictly required if you just wanted to try out Agora and do not care about performance variations.
  * Disable Turbo Boost to reduce performance variation by running 
    <pre>
    $ echo "0" | sudo tee /sys/devices/system/cpu/cpufreq/boost
    </pre>
  * Set CPU scaling to performance by running 
    <pre>
    $ sudo cpupower frequency-set -g performance
    </pre>
    where cpupower can be installed through
    <pre>
    $ sudo apt-get install -y linux-tools-$(uname -r)
    </pre>
  * Turn off hyper-threading. We provide an example bash script 
	 (scripts/tune_hyperthread.sh), where the core indices are machine dependent.
  * Set IRQ affinity to direct OS interrupts away from Agora's cores. 
    We direct all the interrupts to core 0 in our experiments.  
	  We provide an example bash script (scripts/set_smp_affinity.sh), 
    where the IRQ indices are machine dependent.
    
The steps to collect and analyze timestamp traces are as follows:
  * Enable DPDK in Agora.  Make sure it is compiled / configured for supporting your specific hardware NICs (see [DPDK_README.md](DPDK_README.md)).
  * We use files/config/ci/tddconfig-sim-ul.json for uplink experiments and files/config/ci/tddconfig-sim-dl.json for downlink experiments.\
    In our [paper](#documentation), we change “antenna_num”,  “ue_num” and “symbol_num_perframe” 
    to different values to collect different data points in the figures. 
  * Generate source data files by running
    <pre>
    $ ./build/data_generator --conf_file files/config/ci/tddconfig-sim-ul.json
    </pre>
  * Run Agora as a real-time process (to prevent OS from doing context switches) using 
    <pre>
    $ sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} chrt -rr 99 ./build/agora --conf_file files/config/ci/tddconfig-sim-ul.json
    </pre>

    (**NOTE**: Using a process priority 99 is dangerous. Before running it, 
    make sure you have directed OS interrupts away from cores used by Agora. If you have not done so, run
    <pre>
    $ sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} ./build/agora --conf_file files/config/ci/tddconfig-sim-ul.json
    </pre>
    instead to run Agora as a normal process.)
  * Run the emulated RRU using
    <pre>
    $ sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} ./build/sender --num_threads=2 --core_offset=0 \
      --conf_file=files/config/ci/tddconfig-sim-ul.json --frame_duration=5000 --enable_slow_start=1
    </pre>
    For DPDK, add `--server_mac_addr=` and set it to the MAC address of the NIC used by Agora. 
  * The timestamps will be saved in files/experiment/timeresult.txt after Agora finishes processing. We can then use a [MATLAB script](matlab/parsedata_ul.m) to process the timestamp trace. 
  * We also provide MATLAB scripts for [uplink](matlab/parse_multi_file_ul) and [downlink](matlab/parse_multi_file_dl) that are able to process multiple timestamp files and generate figures reported in our [paper](#documentation).

Log and plot PHY stats:
  * Compile the code with
    <pre>
    $ cmake .. -DENABLE_CSV_LOG=True
    </pre>
  * Run test with desired config; log files will be created in a directory named with timestamp under the files/log/ folder
  * Run plot_csv.py with csv file input
    <pre>
    $ python3 tools/python/plot_csv.py [max_frames] [X_label] [Y_label] [legend_name] < path/to/log/log-xyz.csv
    With optional paramters, e.g.,
    $ python3 tools/python/plot_csv.py 1000 Frame EVM UE < files/log/2022-10-25-15-46-55/log-evm-BS.csv
    or set max_frames to 0 to plot all frames, e.g.,
    $ python3 tools/python/plot_csv.py 0 Frame EVM UE < files/log/2022-10-25-15-46-55/log-evm-BS.csv
    or without any paramter to plot as default, e.g.,
    $ python3 tools/python/plot_csv.py < files/log/2022-10-25-15-46-55/log-evm-BS.csv
    Note the < operator is required.
    </pre>
  * (Optional) Run plot_csv.py with UDP input
    Set log listener IP address and port in config file, e.g.,
    <pre>
    "log_listener_addr": "127.0.0.1",
    "log_listener_port": 33300
    </pre>
    Before start, run the command on the listener machine (which has the specified IP address):
    <pre>
    $ nc -u -l [port_number] | python3 tools/python/plot_csv.py [max_frames] [X_label] [Y_label] [legend_name]
    port_number (required) is log_listener_port + log_id (defined in csv_logger.h);
    max_frames (required) is a positive integer no greater than the maximum transfered frames.
    For example,
    $ nc -u -l 33303 | python3 tools/python/plot_csv.py 1000 Frame EVM UE
    Repeat with multiple ports for more logs if desired.
    </pre>
    Run test; plots will be shown when max_frames is reached.

## Contributing to Agora
Agora is open-source and open to your contributions. Before contributing, please read [this](CONTRIBUTING.md).

## Acknowledgment
Agora was funded in part by NSF Grant #1518916 and by the NSF PAWR project.

## Documentation
Check out [Agora Wiki](https://github.com/jianding17/Agora/wiki) for 
Agora's design overview and flow diagram that maps massive MIMO baseband processing 
to the actual code structure. Technical details and performance results can be found in
 * Jian Ding, Rahman Doost-Mohammady, Anuj Kalia, and Lin Zhong, "Agora: Real-time massive MIMO baseband processing in software," in Proc. of ACM CoNEXT, December 2020 ([PDF](https://www.yecl.org/publications/ding2020conext.pdf), [video](https://dl.acm.org/doi/abs/10.1145/3386367.3431296)).

Doxygen documentation generation for Agora can be initiated by running the following command from the repository root directory:
`doxygen Agora_doxygen.conf`
The latest hosted output is located at [Agora Doxygen](https://renew-wireless.org/agora-doxy/html/index.html) 

Other community resources can be found at the [RENEW Wireless Wiki](https://wiki.renew-wireless.org/)

## Contact
Jian Ding (jian.ding@yale.edu)
