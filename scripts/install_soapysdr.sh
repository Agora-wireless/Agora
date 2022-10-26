#!/bin/bash
# Install SoapySDR from the GitHub repo

sudo apt -y install cmake g++ libpython-dev python-numpy swig

git clone --branch soapy-sdr-0.8.1 --depth 1 --single-branch https://github.com/pothosware/SoapySDR.git
cd SoapySDR
mkdir -p build
cd build
cmake ../
make -j`nproc`
sudo make install
cd ../..
sudo ldconfig