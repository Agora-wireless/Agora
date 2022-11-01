#!/bin/bash
#
# Set up a fresh Ubuntu 20.04 box with packages for building Agora.
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
wget http://sourceforge.net/projects/arma/files/armadillo-11.2.3.tar.xz .
tar -xf armadillo-11.2.3.tar.xz
(cd armadillo-11.2.3; cmake -DALLOW_OPENBLAS_MACOS=ON .; make -j4; sudo make install)
rm -rf armadillo*
sudo ldconfig
cd ../

# Install SoapySDR from the GitHub repo
cd `mktemp -d`
git clone --branch soapy-sdr-0.8.1 --depth 1 --single-branch https://github.com/pothosware/SoapySDR.git
cd SoapySDR
mkdir -p build
cd build
cmake ../
make -j`nproc`
sudo make install
cd ../..
sudo ldconfig
