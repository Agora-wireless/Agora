#!/bin/bash
# Install dpdk from source
# Tested on Ubuntu (requires meson / ninja / pip3 install pyelftools)
cd /opt/
wget https://fast.dpdk.org/rel/dpdk-21.11.1.tar.xz .
tar -xf https://fast.dpdk.org/rel/dpdk-21.11.1.tar.xz
cd dpdk-stable-21.11.1
meson build
cd build
ninja
sudo ninja install
sudo ldconfig
cd ../