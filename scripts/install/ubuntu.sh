#!/bin/bash
#
# Set up a fresh Ubuntu 18.04 box with packages for building Agora.
# This does not include installing Intel compilers and FlexRAN

if [ -n ${SYSTEM_INSTALL} ]; then
  if [ ${SYSTEM_INSTALL} == "1" ]; then
    sudo apt-get update
    sudo apt-get install -y g++ cmake make liblapack-dev libblas-dev libboost-all-dev \
      libnuma-dev libgflags-dev libgtest-dev swig python-numpy python-pyqt5 \
      libpython-dev python3-pip build-essential gcc libudev-dev libnl-3-dev \
      libnl-route-3-dev ninja-build pkg-config valgrind python3-dev \
      cython3 python3-docutils pandoc jq rsync
    sudo pip3 install meson
  fi
fi

systemPkgs=(g++ cmake make liblapack-dev libblas-dev libboost-all-dev \
  libnuma-dev libgflags-dev libgtest-dev swig python-numpy python-pyqt5 \
  libpython-dev python3-pip build-essential gcc libudev-dev libnl-3-dev \
  libnl-route-3-dev ninja-build pkg-config valgrind python3-dev \
  cython3 python3-docutils pandoc jq rsync)

for pkg in ${systemPkgs[@]}; do
  checkpkg ${pkg}
  if [ ${checkpkg_res} == "0" ]; then
    echocyan "Required packages:"
    echo -en "\t"
    for pkg1 in ${systemPkgs[@]}; do
      echo -n "${pkg1} "
    done
    echo
    exit
  fi
done

res=$(meson --version | grep "0.")
if [ -z ${res} ]; then
  echored "Error: meson is required"
  echo -e "\tRun pip3 install meson"
  exit
fi

# sudo apt update

# # Toolchain
# sudo apt -y install g++ cmake make

# # General libs
# sudo apt -y install liblapack-dev libblas-dev libboost-all-dev doxygen \
#   libnuma-dev libgflags-dev libgtest-dev swig

# # These libraries may not exist on newer kernel versions
# # If they fail to be installed, try python3 version instead
# sudo apt -y python-numpy python-pyqt5 libpython-dev

# GTest needs special compilation
# (cd /usr/src/gtest && sudo cmake . && sudo make && sudo mv libg* /usr/lib/)

mkdir -p ~/project/tmp

# Install Armadillo from source
cd ~/project/tmp
wget http://sourceforge.net/projects/arma/files/armadillo-10.7.1.tar.xz
tar xf armadillo-10.7.1.tar.xz
(cd armadillo-10.7.1; cmake . -DCMAKE_INSTALL_PREFIX:PATH=~/project/armadillo/; make -j; make install)
rm -rf armadillo*

# Install nlohmann json-dev from the GitHub repo
git clone --depth=1 https://github.com/nlohmann/json.git
cd json
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/project/json/
make -j
make install
cd ~/project/tmp
rm -rf json
# sudo ldconfig

# Install SoapySDR from the GitHub repo
git clone --depth=1 https://github.com/pothosware/SoapySDR.git
cd SoapySDR
mkdir build
cd build
cmake .. -DCMAKE_INSTALL_PREFIX:PATH=~/project/SoapySDR/
make -j
make install
cd ~/project/tmp
rm -rf SoapySDR
# sudo ldconfig

# DPDK drivers
# Latest DPDK: sudo make install T=x86_64-native-linuxapp-gcc DESTDIR=/usr
# sudo apt -y install dpdk libdpdk-dev dpdk-igb-uio-dkms
