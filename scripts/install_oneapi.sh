#!/bin/bash
# Install Intel one api from apt package

#Install inteloneAPI
# use wget to fetch the Intel repository public key
cd /tmp
wget https://apt.repos.intel.com/intel-gpg-keys/GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
# add to your apt sources keyring so that archives signed with this key will be trusted.
sudo apt-key add GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB
# remove the public key
rm GPG-PUB-KEY-INTEL-SW-PRODUCTS.PUB

sudo add-apt-repository "deb https://apt.repos.intel.com/oneapi all main"
sudo apt install intel-hpckit
