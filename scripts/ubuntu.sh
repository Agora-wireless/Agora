#!/bin/bash
#
# Set up a fresh Ubuntu 18.04 box with packages for building Agora.
# This does not include installing Intel compilers and FlexRAN

sudo apt update

# Toolchain
sudo apt -y install g++ cmake make

# General libs
sudo apt -y install liblapack-dev libblas-dev libboost-all-dev doxygen \
  libnuma-dev libgflags-dev libgtest-dev swig

# These libraries may not exist on newer kernel versions
# If they fail to be installed, try python3 version instead
sudo apt -y python-numpy python-pyqt5 libpython-dev
# sudo apt -y python3-numpy python3-pyqt5 libpython3-dev

# GTest needs special compilation
(cd /usr/src/gtest && sudo cmake . && sudo make && sudo mv libg* /usr/lib/)

# Install Armadillo from source
wget http://sourceforge.net/projects/arma/files/armadillo-10.7.1.tar.xz .
tar xf armadillo-10.7.1.tar.xz
(cd armadillo-10.7.1; cmake .; make -j; sudo make install)
rm -rf armadillo*

# Install nlohmann json-dev from the GitHub repo
cd `mktemp -d`
git clone --depth=1 https://github.com/nlohmann/json.git
cd json
mkdir build
cd build
cmake ..
make -j
sudo make install
sudo ldconfig

# Install SoapySDR from the GitHub repo
cd `mktemp -d`
git clone --depth=1 https://github.com/pothosware/SoapySDR.git
cd SoapySDR
mkdir build
cd build
cmake ..
make -j
sudo make install
sudo ldconfig

# DPDK drivers
# Latest DPDK: sudo make install T=x86_64-native-linuxapp-gcc DESTDIR=/usr
# sudo apt -y install dpdk libdpdk-dev dpdk-igb-uio-dkms
