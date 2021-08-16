Agora is compatable with and accelerated by the DPDK library.
* Agora has been tested with dpdk version 20.02.1 and 20.11.2 with both Intel 40 GbE and Mellanox 100 GbE NICs

## Contents
 * [Building Agora](#building-agora)
   * [Setting up the build environment](#setting-up-the-build-environment)
   * [Building and running with emulated RRU](#building-and-running-with-emulated-rru)

# Building Agora
  Agora currently only builds and runs on Linux, and has been tested on Ubuntu 16.04, 18.04, and 20.04. 
  Agora requires CMake 2.8+ and works with both GNU and Intel compilers with C++17 support. 
## Setting up the build environment
  * DPDK
    * [DPDK](http://core.dpdk.org/download/) version 20.02.1 and 20.11.2 has been tested with
      Intel 40 GbE and Mellanox 100 GbE NICs in Agora.
    * It is required you enable hugepage support and run agora under sudo permissions (LD_LIBRARY_PATH=${LD_LIBRARY_PATH}).
      <pre>
      $ sudo sh -c "echo 1024 > /sys/devices/system/node/node0/hugepages/hugepages-2048kB/nr_hugepages"
      $ sudo sh -c "echo 1024 > /sys/devices/system/node/node1/hugepages/hugepages-2048kB/nr_hugepages"
      $ sudo sh -c "echo 4 > /sys/devices/system/node/node0/hugepages/hugepages-1048576kB/nr_hugepages"
      $ sudo sh -c "echo 4 > /sys/devices/system/node/node1/hugepages/hugepages-1048576kB/nr_hugepages"
      </pre>
      Make memory available for dpdk
      <pre>
      $ mkdir /mnt/huge
      $ mount -t hugetlbfs nodev /mnt/huge
      </pre>
      Check hugepage usage
      <pre>
      $ cat /proc/meminfo
      </pre>
      [Building] https://doc.dpdk.org/guides/linux_gsg/sys_reqs.html#building-dpdk-applications

    * Mellanox dpdk support depends on libibverbs / libmlx5 / ibverbs-utils 
    * Intel NICs may require the user to bring down the interface and load a dpdk compatible driver (vfio / pci_generic)
    * To install version 20.02.1 of dpdk, run `sudo make install T=x86_64-native-linuxapp-gcc
      DESTDIR=/usr -j` (CONFIG_RTE_LIBRTE_MLX5_PMD=y in config/common_base to enabled MLX5 poll mode driver)
    * To install version 20.11.1 
      <pre>
      $ meson build && cd build && ninja
      $ sudo ninja install
      $ sudo ldconfig
      </pre>
      the MLX poll mode driver will be autodetected and installed if required

## Building and running with emulated RRU
We provide a high performance [packet generator](simulator) to emulate the RRU. This generator allows Agora to run and be tested without actual RRU hardware. The following are steps to set up both Agora and the packet generator.

 * Build Agora. This step also builds the sender, a data generator that generates random input data files, an end-to-end test that checks correctness of end results for both uplink and downlink, and several unit tests for testing either performance or correctness of individual functions.
    <pre>
    $ cd Agora
    $ mkdir build
    $ cd build
    $ cmake .. -DUSE_DPDK=1
    $ make -j
    </pre>
 * Most DPDK installations will require you to run under sudo permissions

 * Run end-to-end test to check correctness (uplink, downlink and combined tests should all pass if everything is set up correctly).
    <pre>
    $ sudo ./test/test_agora/test_agora.sh 10 out # Runs test for 10 iterations
    </pre>

 * Run Agora with emulated RRU traffic
   * **NOTE**: We recommend running Agora and the emulated RRU on two different machines. 
   If you are running them on the same machine, make sure Agora and the emulated RRU are using different set of cores, 
     otherwise there will be performance slow down.
   * First, return to the base directory (`cd ..`), then run
   <pre>
   $ ./build/data_generator --conf_file data/tddconfig-sim-ul.json
   </pre>
     to generate data files.
   * In one terminal, run 
   <pre>
   $ sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} ./build/agora --conf_file data/tddconfig-sim-ul.json
   </pre>
    to start Agora with uplink configuration.
   * In another terminal, run
   <pre>
   $ sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} ./build/sender --num_threads=2 --core_offset=1 --frame_duration=5000 --enable_slow_start=1 --conf_file=data/tddconfig-sim-ul.json
   </pre>
   to start the emulated RRU with uplink configuration.
   
   When running the emulated RRU with DPDK, it is required to set the MAC address of the NIC used by Agora. To do this, pass `--server_mac_addr=` to `sender`.
