#! /bin/bash

source $(dirname $0)/utils.sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
PROJECT_ROOT=~/project

# Install Intel libraries
# echocyan "Install Intel libraries"
# mkdir -p ${PROJECT_ROOT}/tmp
# cd ${PROJECT_ROOT}/tmp
# wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18236/l_BaseKit_p_2021.4.0.3422.sh
# bash l_BaseKit_p_2021.4.0.3422.sh -a --silent --eula accept --components intel.oneapi.lin.ipp.devel:intel.oneapi.lin.mkl.devel --install-dir ${PROJECT_ROOT}/intel/oneapi
# wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18211/l_HPCKit_p_2021.4.0.3347.sh
# bash l_HPCKit_p_2021.4.0.3347.sh -a --silent --eula accept --components intel.oneapi.lin.dpcpp-cpp-compiler-pro --install-dir /users/junzhig/project/intel/oneapi/
# rm l_BaseKit_p_2021.4.0.3422.sh l_HPCKit_p_2021.4.0.3347.sh
# echo "source ~/project/intel/oneapi/setvars.sh" >> ~/.bashrc

# # Install FlexRAN SDK
echocyan "Install FlexRAN SDK"
cd ${DIR}/..
sudo cp -r deps/FlexRAN-FEC-SDK-19-04 /opt/
sudo chmod 777 /opt/FlexRAN-FEC-SDK-19-04

# # Install meson
# echocyan "Install meson"
# sudo apt-get install -y python3-pip
# pip3 install --user meson
# echo "export PATH=\${PATH}:~/.local/bin" >> ~/.bashrc
# source ~/.bashrc

# # Install rdma-core
# echocyan "Install rdma-core"
# cd ${PROJECT_ROOT}
# git clone https://github.com/linux-rdma/rdma-core.git
# sudo apt-get install build-essential cmake gcc libudev-dev libnl-3-dev libnl-route-3-dev ninja-build pkg-config valgrind python3-dev cython3 python3-docutils pandoc -y
# cd rdma-core
# bash build.sh
# echo "export LIBRARY_PATH=\${LIBRARY_PATH}:~/project/rdma-core/build/lib" >> ~/.bashrc
# echo "export LD_LIBRARY_PATH=\${LD_LIBRARY_PATH}:\${LIBRARY_PATH}" >> ~/.bashrc

# # Install DPDK
# echocyan "Install DPDK"
# echo "export RTE_SDK=~/project/downloads/dpdk-stable-20.11.3" >> ~/.bashrc
# source ~/.bashrc
cd ${PROJECT_ROOT}
# wget http://fast.dpdk.org/rel/dpdk-20.11.3.tar.xz
# tar xf dpdk-20.11.3.tar.xz
# rm dpdk-20.11.3.tar.xz
# cd dpdk-stable-20.11.3
# meson build
# cd build
# ninja
# DESTDIR=./install ninja install

# Build Agora
echocyan "Build Agora"
sudo apt-get install -y python3-numpy
cd ${DIR}/..
mkdir build
cd build
cmake ..
make -j
