#!/bin/bash
# Install SoapySDR from the GitHub repo
cd `mktemp -d`
git clone --depth=1 https://github.com/nlohmann/json.git
cd json
mkdir build
cd build
cmake ..
make -j8
sudo make install
sudo ldconfig
