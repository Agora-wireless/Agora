#!/bin/bash
#
# Set up a fresh Ubuntu 20.04 box with packages for running Agora.
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

# GTest needs special compilation
(cd /usr/src/gtest && sudo cmake . && sudo make && sudo mv libg* /usr/lib/)

# Install Armadillo from source
source install_armadillo.sh

# Install SoapySDR from the GitHub repo
source install_soapysdr.sh
