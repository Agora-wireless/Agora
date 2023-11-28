[![Build Status](https://falcon.ecg.rice.edu:443/buildStatus/icon?job=github_public_agora%2Fdevelop)](https://falcon.ecg.rice.edu:443/job/github_public_agora/job/develop/)

# M3A
M3A: Multipath Multicarrier Misinformation to Adversaries.
M3A is a multi-antenna multicarrier OFDM/OFDMA transmission system that allows the sender (Alice) to deliver data symbols to legitimate users (Bob) while simultaneously sending misinformation to eavesdroppers (Eve). As a result, the decoded sysbols at Eve are positioned randomly across the I-Q plane. Meanwhile, the data symbol integrity at Bob is still successfully retained. To achieve these, M3A adopts a novel digital baseband algorithm and exploits rich multipath characteristics of physical channels within the Sub-6GHz frequeny range.
Check our [paper at MobiCom 23'](https://dl.acm.org/doi/10.1145/3570361.3613282) for more details.

[M3A Code](https://github.com/Agora-wireless/Agora/tree/subset-modulation_fftshift) is implemented by using Agora, a complete softwarized baseband processing for Massive MIMO.
More detailed installation and running instructions are detailed below.

Some highlights:
* M3A does not require either the physical location or CSI (exact or statistical) of Eve.
* M3A can thwart passive eavesdroppers from decoding data symbols effectively, even in wavelength-scale eavesdropping proximity in practical indoor multipath environment.
* M3A retains reliability at Bob under diverse channel conditions.
* M3A can be implemented in multi-antenna 5G and beyond base stations and does not require any modification in the UE.

## Project Dataset
This dataset was collected by [Zhecun Liu](mailto:zl83@rice.edu) from Rice University, in a typical lab room that consists of multiple objects, namely
chairs, tables, and numerous other objects which create a natural multipath environment. 
The setup is shown in figure below, which contains two user nodes (UE) and one base-station (BS). 
The BS consists of four main components: a remote radio head (RRH), a hub unit (HUB), a reference node (REF), and a multi-core server (SVR).
The BS and two Iris UEs are configured to be Alice, Bob, and Eve respectively. 
A reference node outside the array (and synchronized) has been included for reciprocity calibration.

<img src="https://github.com/Agora-wireless/Agora/blob/M3A/images/bs_2ue.png" width="45%"/>

The dataset compares the reliability and security performance of M3A and its variants against downlink conjugate beamforming as the baseline, over 20 locations for different Alice-Bob/Eve channels.
In test topology shown below, there is an obstacle near location 20, deteriorating signal strength between Alice and Bob there.

<img src="https://github.com/Agora-wireless/Agora/blob/M3A/images/test_topology.png" width="35%"/>

We use a fixed 16-QAM modulation during the experiments.
Operational parameters such as amplifier gains, modulation order, and the number of subcarriers, are configured using JSON files.
Alice adopts a TDD-based transmission protocol, as illustrated in timeline figure below.
Note that we configure Bob and Eve so that they send their uplink pilot, for the CSIT acquisition, in different times (indicated by the transmission slot `P`).
Since Alice is configured to activate her reception window (highlighted below) during only Bob's `P` slot that , both Alice and Bob essentially are agnostic to the presence of Eve.
Finally, Bob and Eve both are in receive state, when Alice sending downlink signals during the three `D` slots. 
The first `D` slot carries a preamble symbol, allowing a receiver (Bob and Eve) calculate the CSIR and equalize the channel.

<img src="https://github.com/Agora-wireless/Agora/blob/M3A/images/timeline.png" width="50%"/>

## Dataset Description
* Directory
`M3A-data/Data-reliabilityExp/XX/loc#` represents the collected PHY status per TDD frame using beamforming scheme XX when Bob is at location # (`XX` can be either `BF`, `FASM`, `M3A`, or `M3Alc` and `#` can be any integer from 1 to 20).
Here, we solely consider Alice and Bob to study the reliability at Bob, and we adopt Bob's BER as the performance metric.
At the end of log file within the directory, the BER is calculated based on 1,000 transmitted TDD frames.
At each location, we repeated such transmission consecutively five times to transmit 5,000 frames before moving to the next location.
We then switched to different beamforming strategies, after finishing all 20 locations.
* Directory
Now we introduce Eve into the system and evaluate how well does M3A protect against passive eavesdropping.
We adopt Eve’s BER values as a metric.
`M3A-data/Data-securityExp/security-M3A/LOG_FILES` contains collected PHY status per TDD frame when Eve is located at one of the 20 locations (while Bob is at location-8).
`M3A-data/Data-securityExp/security-M3A/TXT_FILES` reports BER metric directly.
In addition, we further collected HDF5 files which contains fine-grained PHY statistics (e.g., the receive constellations) in `M3A-data/Data-securityExp/security-M3A/H5_FILES`.
The detailed instruction on how to analyze them using scripts are given in later section.
As a comparison, we repeated the experiment using BF; the data is stored in `M3A-data/Data-securityExp/security-M3A/`.
* Directory
`M3A-data/Data-wavelengthExp/dir/M3A/xlam` contains Eve's log file and .h5 file when Eve is of `x` unit of wavelengths distance away from Bob along direction `dir`.
There are six different directions in this measurement, and see below a 3D view of experimental setup with distances normalized with respect to carrier wavelength.

<img src="https://github.com/Agora-wireless/Agora/blob/M3A/images/3dView.png" width="35%"/>


## Contents
 * [Build M3A using Agora](#Build-M3A-using-Agora)
   * [Setting up the build environment](#setting-up-the-build-environment)
   * [Building and running with real RRU](#building-and-running-with-real-rru)
 * [Acknowledgment](#acknowledgment)
 * [Documentation](#documentation)
 * [Contact](#contact)
 
 
# Build M3A using Agora
Agora is a complete software realization of real-time MaMIMO baseband. 
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
     
## Building and running with real RRU
M3A is evaluated using an indoor 64-antenna [Argos massive MIMO base station](https://www.yecl.org/argos/), also commercially available from [Skylark Wireless](https://skylarkwireless.com) and are used in the [POWER-RENEW PAWR testbed](https://powderwireless.net/).
The base-station contains four linear antennna arrays, we reserved the top array to perform our experiment (8 radios in total).
The BS and two Iris UEs are configured to be Alice, Bob, and Eve respectively. The hardware we used are illustrated below. 

<img src="https://github.com/Agora-wireless/Agora/blob/M3A/images/hw-zoomin.png" width="39%"/>

**NOTE**: We recommend using one server for controlling the RRU and running Agora, and another server for controlling the UEs and running the UE code.
If you are running them on the same machine, make sure Agora and UEs are using different set of cores, otherwise there will be performance slow down. 
 
**Note:** Faros RRU and Iris UEs can be discovered using the [pyfaros](https://github.com/skylarkwireless/pyfaros) tool. You can use this tool to find the topology of the hardware connected to the server.

 * Rebuild the code on both servers for RRU side the UE side.
    * For Faros RRU and Iris UEs (M3A uses this option), pass `-DRADIO_TYPE=SOAPY_IRIS` to cmake. M3A uses this option, and passes `-DENABLE_HDF5=true` as well to enable HDF5 files collection.
    * For USRP-based RRU and UEs, pass `-DRADIO_TYPE=SOAPY_UHD` to cmake
    * M3A used uncoded transmission to explore the scrambling of constellations at Eve. To do so, in file `symbols.h`, set variable `static constexpr bool kDownlinkHardDemod` to true.
    * There are three different beamformers implemented in M3A. To toggle between them, go to file `dozf.cc` and change `static constexpr enum M3A_Version kM3A_Version`. To set number of antennnas that are off, change `static constexpr size_t N_OFF`.
    * Run `make -j` to recompile the code.
 * Run the UE code on the server connected to the Iris UEs
   * For Iris UEs, run the pyfaros tool in the `data` directory as follows:
     <pre>
     $ python3 -m pyfaros.discover --json-out
     </pre>
     This will output a file named `topology.json` with all the discoverable serial IDs included.
   * Modify `data/topology.json` by adding/removing serials of client Irises you'd like to include
     from your setup. In M3A experimnets we used two UEs (Bob and Eve), and the two json files we used are `topology-vulture.json` and `topology-vulture-listener.json` for Bob and Eve respectively.
   * For USRP-based RRU and UEs, modify the existing `data/topology.json` and enter the appropriate IDs.
   * Run `./build/data_generator --conf_file data/examples/dl-vulture.json` to generate required data files. **Note:** This step does not give Eve any information about sent bits, but merely letting us check if decoded bits at Eve are correct.
   * Run `./build/user --conf_file data/examples/dl-vulture.json`, in order to configure Bob. This is the file where we set carrier frequency, trasmit/receive gains, sample rate, mcs information and so forth.
   * Run `./build/user --conf_file data/examples/dl-vulture-listener.json`, in order to configure Eve. Notice that the only difference in Bob's vs Eve's json is that Eve had a different `frame_schedule: BGCLGGGPGDDDG`. 
  Consequnetly, Alice does single-user transmission while Eve overhears all the downlink signals.
 * Run Agora on the server connected to the Faros RRU
   * scp over the generated file `data/LDPC_orig_dl_data_512_ant1.bin` `data/LDPC_orig_dl_data_512_ant9.bin` `data/LDPC_rx_data_512_ant9.bin` and `data/orig_dl_data_512_ant1.bin`  from the client machine to the server's `data` directory. 
   * Run `make -j` to compile the code.
   * For Faros RRU, use the pyfaros tool the same as with the UEs to generate a new `data/topology.json`; modify `data/topology.json` by adding/removing serials of your RRU Irises, and the hub. M3A experiments are conducted using `dl-vulture.json`.
   * Run `./build/agora --conf_file data/examples/dl-vulture.json`

After this step, the two log files will be generated automatically, which contains physical layer statistics of the configured transmission frame-by-frame.

## HDF5 files analysis using MATLAB Scripts
In this section, we provide some examples for using MATLAB scripts to analyze and plot PHY data in HDF5 files.
  * Run function `inspect_single_frame(dataset_filename, inspect_frame, verbose)` in MATLAB command window; where the first argument is the name of h5 file to analyze, second argument the specific frame number, and verbose should be `true` to disply h5 file attributes
  * Below shows example output figures by running `inspect_single_frame("UeRxData-loc10.h5", 999, "false")`, including the receive constellation plot at Eve over two downlink transmissions, receive time-domain waveform, symbol detection error matrices, and the magnitude and phase of the effective CSIR.
<img src="https://github.com/Agora-wireless/Agora/blob/M3A/images/sampleMatlab.png" width="89%"/>


## Acknowledgment
This work was supported by Cisco, Intel, NSF grants CNS-2148132, CNS-2211618, CNS-1955075, and DOD: Army Research Laboratory grant W911NF-19-2-0269.

## Documentation
Our dataset has been released and can be found at the [RENEW Wireless Wiki](https://wiki.renew-wireless.org/)
## Contact
Corresponding author: Zhecun Liu (zl83@rice.edu)
