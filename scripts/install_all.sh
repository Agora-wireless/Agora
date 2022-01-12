#! /bin/bash

source $(dirname $0)/utils.sh
source $(dirname $0)/ubuntu.sh

PROJECT_ROOT=~/project

echo "source ~/.bashrc" >> ~/.bash_profile

# Install Intel libraries
echocyan "Install Intel libraries"
mkdir -p ${PROJECT_ROOT}/tmp
cd ${PROJECT_ROOT}/tmp
wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18236/l_BaseKit_p_2021.4.0.3422.sh
bash l_BaseKit_p_2021.4.0.3422.sh -a --silent --eula accept --components intel.oneapi.lin.ipp.devel:intel.oneapi.lin.mkl.devel --install-dir ${PROJECT_ROOT}/intel/oneapi
wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18211/l_HPCKit_p_2021.4.0.3347.sh
bash l_HPCKit_p_2021.4.0.3347.sh -a --silent --eula accept --components intel.oneapi.lin.dpcpp-cpp-compiler-pro --install-dir /users/junzhig/project/intel/oneapi/
rm l_BaseKit_p_2021.4.0.3422.sh l_HPCKit_p_2021.4.0.3347.sh
echo "source ~/project/intel/oneapi/setvars.sh" >> ~/.bash_profile
source ~/project/intel/oneapi/setvars.sh

# Install FlexRAN SDK
echocyan "Install FlexRAN SDK"
cd ${PROJECT_ROOT}/Agora
sudo cp -r deps/FlexRAN-FEC-SDK-19-04 /opt/
sudo chmod 777 /opt/FlexRAN-FEC-SDK-19-04

# Install meson
echocyan "Install meson"
sudo apt-get install -y python3-pip
sudo pip3 install meson

# Install rdma-core
echocyan "Install rdma-core"
cd ${PROJECT_ROOT}
git clone https://github.com/linux-rdma/rdma-core.git
sudo apt-get install build-essential cmake gcc libudev-dev libnl-3-dev libnl-route-3-dev ninja-build pkg-config valgrind python3-dev cython3 python3-docutils pandoc -y
cd rdma-core
bash build.sh
echo "export LIBRARY_PATH=\${LIBRARY_PATH}:~/project/rdma-core/build/lib" >> ~/.bash_profile
echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:\${LIBRARY_PATH}" >> ~/.bash_profile
export LIBRARY_PATH=${LIBRARY_PATH}:~/project/rdma-core/build/lib
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${LIBRARY_PATH}

# Install DPDK
echocyan "Install DPDK"
echo "export RTE_SDK=~/project/dpdk-stable-20.11.3" >> ~/.bash_profile
export RTE_SDK=~/project/dpdk-stable-20.11.3
cd ${PROJECT_ROOT}
wget http://fast.dpdk.org/rel/dpdk-20.11.3.tar.xz
tar xf dpdk-20.11.3.tar.xz
rm dpdk-20.11.3.tar.xz
cd dpdk-stable-20.11.3
meson build
cd build
ninja
DESTDIR=./install ninja install

# Build Agora
echocyan "Build Agora"
sudo apt-get install -y python3-numpy jq
cd ${PROJECT_ROOT}/Agora
mkdir build
cd build
cmake .. -DLOG_LEVEL=warn
make -j

# Setup Hugepages and modprobe
cd ${PROJECT_ROOT}/dpdk-stable-20.11.3
sudo ./usertools/dpdk-hugepages.py --setup 22G
sudo modprobe -a ib_uverbs mlx5_core mlx5_ib