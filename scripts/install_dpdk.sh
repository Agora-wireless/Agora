#!/bin/bash
# Install dpdk from source
# Tested on Ubuntu (requires meson / ninja / pip3 install pyelftools)
#Install dpdk 20.11. lts
cd /opt/
sudo wget https://fast.dpdk.org/rel/dpdk-21.11.2.tar.xz
sudo tar -xf dpdk-21.11.2.tar.xz
sudo rm -rf dpdk-21.11.2.tar.xz
sudo chmod -R a+rwX dpdk-stable-21.11.2/
cd dpdk-stable-21.11.2/
meson build
cd build
ninja
sudo ninja install
ninja -t clean
sudo ldconfig
