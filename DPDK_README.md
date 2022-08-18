Agora is compatable with and accelerated by the DPDK library.
* Agora has been tested with dpdk version 20.02.1 and 20.11.2 with both Intel 40 GbE and Mellanox 100 GbE NICs

## DPDK installation
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

  * Mellanox's DPDK support depends on libibverbs / libmlx5 / ibverbs-utils. For DPDK version 20.02.1, 
    set `CONFIG_RTE_LIBRTE_MLX5_PMD=y` in `config/common_base` file to enable MLX5 poll mode driver before compiling DPDK.
  * Intel NICs may require the user to bring down the interface and load a DPDK compatible driver (vfio / pci_generic)
  * To install version 20.02.1 of DPDK, run `sudo make install T=x86_64-native-linuxapp-gcc
    DESTDIR=/usr -j`. 
  * To install version 20.11.1, run
    <pre>
    $ meson build && cd build && ninja
    $ sudo ninja install
    $ sudo ldconfig
    </pre>
    the MLX poll mode driver will be autodetected and installed if required

## Building and running Agora and emulated RRU with DPDK
 * Build Agora and emulated RRU with DPDK enabled.
    <pre>
    $ cd Agora
    $ mkdir build
    $ cd build
    $ cmake -DUSE_DPDK=1 ..
    $ make -j
    </pre>
  * For Intel NICs, Mellanox libraries need to be excluded in the build. To do this, run
    <pre>
    $ cmake -DUSE_DPDK=1 -DUSE_MLX_NIC=0 ..; make -j
    </pre>

 * Run Agora with emulated RRU traffic with DPDK 
   * **NOTE**: For DPDK test, we run Agora and the emulated RRU on two different machines.
     Most DPDK installations will require you to run under sudo permissions. 
   * To generate data file, first return to the base directory (`cd ..`), then run
   <pre>
   $ ./build/data_generator --conf_file files/config/ci/tddconfig-sim-ul.json
   </pre>
   * To start Agora with uplink configuration, on one machine, run 
   <pre>
   $ sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} ./build/agora --conf_file files/config/ci/tddconfig-sim-ul.json
   </pre>
    
   * To start the emulated RRU with uplink configuration, on another machine, run
   <pre>
   $ sudo LD_LIBRARY_PATH=${LD_LIBRARY_PATH} ./build/sender --num_threads=2 --core_offset=1 --frame_duration=5000 --enable_slow_start=1 --conf_file=files/config/ci/tddconfig-sim-ul.json --server_mac_addr=00:00:00:00:00:00
   </pre>
   Change the MAC address in `--server_mac_addr=` to the MAC address of the NIC used by Agora. 
   
