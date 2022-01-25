#! /bin/bash 

# Run this script on the server running Hydra app
# This script install Hydra and dependent packages

set -e

# Find the root directory of Hydra app
script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

# Set the root directory of Hydra remote runner
HYDRA_RUNNER_ROOT="~/HydraRemoteRunner"

checkpkg jq
if [ ${checkpkg_res} == "0" ]; then
    exit
fi

# Read HYDRA_RUNNER_ROOT from file config/config.json
hydra_master_config_json=${hydra_root_dir}/config/config.json
if [ ! -f ${hydra_master_config_json} ]; then
    echored "ERROR: config file ${hydra_master_config_json} does not exist"
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_runner_root' | tr -d '"')
if [ "${res}" != "null" ]; then
    HYDRA_RUNNER_ROOT=${res}
fi

# Check whether system packages are installed
# If not, report en error
function check_sys_pkgs() {
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
} 

# Install all system-level packages if INSTALL_HYDRA_PKGS_SYSTEM_LEVEL is set to 1
function install_sys_pkgs() {
    echocyan "Installing system packages"
    sudo apt-get update
    sudo apt-get install -y g++ cmake make liblapack-dev libblas-dev libboost-all-dev \
        libnuma-dev libgflags-dev libgtest-dev swig python-numpy python-pyqt5 \
        libpython-dev python3-pip build-essential gcc libudev-dev libnl-3-dev \
        libnl-route-3-dev ninja-build pkg-config valgrind python3-dev \
        cython3 python3-docutils pandoc jq rsync
    sudo pip3 install meson
}

function install_armadillo() {
    echocyan "Installing Armadillo"
    eval "mkdir -p ${HYDRA_RUNNER_ROOT}/tmp"
    eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
    wget http://sourceforge.net/projects/arma/files/armadillo-10.7.1.tar.xz
    tar xf armadillo-10.7.1.tar.xz
    eval "(cd armadillo-10.7.1; cmake . -DCMAKE_INSTALL_PREFIX:PATH=${HYDRA_RUNNER_ROOT}/armadillo/; make -j; make install)"
    rm -rf armadillo*
}

function install_nlohmann() {
    echocyan "Installing nlohmann json"
    eval "mkdir -p ${HYDRA_RUNNER_ROOT}/tmp"
    eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
    git clone --depth=1 https://github.com/nlohmann/json.git
    cd json
    mkdir build
    cd build
    eval "cmake .. -DCMAKE_INSTALL_PREFIX:PATH=${HYDRA_RUNNER_ROOT}/json/"
    make -j
    make install
    eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
    rm -rf json
}

function install_soapysdr() {
    echocyan "Installing Soapy SDR"
    eval "mkdir -p ${HYDRA_RUNNER_ROOT}/tmp"
    eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
    git clone --depth=1 https://github.com/pothosware/SoapySDR.git
    cd SoapySDR
    mkdir build
    cd build
    eval "cmake .. -DCMAKE_INSTALL_PREFIX:PATH=${HYDRA_RUNNER_ROOT}/SoapySDR/"
    make -j
    make install
    eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
    rm -rf SoapySDR
}

# Check whether DPDK lib is installed on this server
#    by checking RTE_SDK variable and necessary files in RTE_SDK directory
function check_dpdk() {
    dpdk_detected=1
    if [ -z "${RTE_SDK}" ]; then
        dpdk_detected=0
    fi
    res=$(ls ${RTE_SDK}/build/install/usr/local/include/rte_common.h) || :
    if [ "$?" != "0" ]; then
        dpdk_detected=0
    fi
    res=$(ls ${RTE_SDK}/build/install/usr/local/lib/x86_64-linux-gnu/librte_common_mlx5.a) || :
    if [ "$?" != "0" ]; then
        dpdk_detected=0
    fi
    res=$(cat ${RTE_SDK}/VERSION) || :
    if [ "$?" != "0" ]; then
        dpdk_detected=0
    fi
    if [ "${res}" != "20.11.3" ]; then
        echocyan "WARNING: Recommend DPDK version 20.11.3"
    fi
}

function install_dpdk() {
    echocyan "Installing DPDK"
    eval "export RTE_SDK=${HYDRA_RUNNER_ROOT}/dpdk-stable-20.11.3"
    eval "cd ${HYDRA_RUNNER_ROOT}"
    wget http://fast.dpdk.org/rel/dpdk-20.11.3.tar.xz
    tar xf dpdk-20.11.3.tar.xz
    rm dpdk-20.11.3.tar.xz
    cd dpdk-stable-20.11.3
    meson build
    cd build
    ninja
    DESTDIR=./install ninja install
}

function install_intel_lib() {
    echocyan "Installing Intel libraries"
    eval "mkdir -p ${HYDRA_RUNNER_ROOT}/tmp"
    eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
    wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18236/l_BaseKit_p_2021.4.0.3422.sh
    eval "bash l_BaseKit_p_2021.4.0.3422.sh -a --silent --eula accept --components \
        intel.oneapi.lin.ipp.devel:intel.oneapi.lin.mkl.devel --install-dir ${HYDRA_RUNNER_ROOT}/intel/oneapi"
    wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18211/l_HPCKit_p_2021.4.0.3347.sh
    eval "bash l_HPCKit_p_2021.4.0.3347.sh -a --silent --eula accept --components \
        intel.oneapi.lin.dpcpp-cpp-compiler-pro --install-dir ${HYDRA_RUNNER_ROOT}/intel/oneapi/"
    rm l_BaseKit_p_2021.4.0.3422.sh l_HPCKit_p_2021.4.0.3347.sh
    eval "source ${HYDRA_RUNNER_ROOT}/intel/oneapi/setvars.sh"
}

function install_rdma_core() {
    echocyan "Installing rdma-core"
    eval "cd ${HYDRA_RUNNER_ROOT}"
    git clone https://github.com/linux-rdma/rdma-core.git
    cd rdma-core
    bash build.sh
    eval "export LIBRARY_PATH=${LIBRARY_PATH}:${HYDRA_RUNNER_ROOT}/rdma-core/build/lib"
    export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${LIBRARY_PATH}
}

function install_hydra() {
    echocyan "Building Hydra"
    eval "cd ${HYDRA_RUNNER_ROOT}/Agora"
    mkdir build
    cd build
    cmake .. -DLOG_LEVEL=warn
    make -j
}

# Install system-level packages if INSTALL_HYDRA_PKGS_SYSTEM_LEVEL is set to 1
if [ -n ${INSTALL_HYDRA_PKGS_SYSTEM_LEVEL} ]; then
    if [ ${INSTALL_HYDRA_PKGS_SYSTEM_LEVEL} == "1" ]; then
        install_sys_pkgs
    fi
fi

# Verify that system-level packages are installed
check_sys_pkgs

# Install armadillo, nlohmann JSON, Soapy SDR, Intel lib, and RDMA core
install_armadillo
install_nlohmann
install_soapysdr
install_intel_lib
install_rdma_core

# Check whether DPDK is installed
# If yes, then directly use the DPDK installed
# Otherwise, install DPDK
check_dpdk
if [ "${dpdk_detected}" == "0" ]; then
    install_dpdk
fi

# Build and install Hydra app
install_hydra
