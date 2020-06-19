#!/bin/bash
# Install SoapySDR from the GitHub repo

sudo apt -y install cmake g++ libpython-dev python-numpy swig

cd `mktemp -d`
git clone --depth=1 https://github.com/pothosware/SoapySDR.git
cd SoapySDR
mkdir build
cd build
cmake ..
make -j8
sudo make install
sudo ldconfig
