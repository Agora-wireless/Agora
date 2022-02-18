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
  echo "[$(hostname)] ERROR: config file ${hydra_master_config_json} does not exist"
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_runner_root' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_RUNNER_ROOT=${res}
fi

mkdir -p /tmp/hydra
rm -f /tmp/hydra/install.log || :
touch /tmp/hydra/install.log

# Check whether system packages are installed
# If not, report en error
function check_sys_pkgs() {
  systemPkgs=(g++ cmake make liblapack-dev libblas-dev \
    libboost-all-dev libnuma-dev python3-pip build-essential \
    gcc libudev-dev libnl-3-dev libnl-route-3-dev ninja-build \
    pkg-config valgrind python3-dev cython3 python3-docutils \
    pandoc jq rsync)
  for pkg in ${systemPkgs[@]}; do
    checkpkg ${pkg}
    if [ ${checkpkg_res} == "0" ]; then
      echo -n "[$(hostname)] Required packages: "
      for pkg1 in ${systemPkgs[@]}; do
        echo -n "${pkg1} "
      done
      echo
      exit
    fi
  done
  res=$(meson --version | grep "0.")
  if [ -z ${res} ]; then
    echo "[$(hostname)] Error: meson is required, please run 'pip3 install meson'"
    exit
  fi
} 

# Install all system-level packages if INSTALL_HYDRA_PKGS_SYSTEM_LEVEL is set to 1
function install_sys_pkgs() {
  echo "[$(hostname)] Downloading required system packages"
  sudo apt-get update >> /tmp/hydra/install.log 2>&1
  sudo apt-get install -y g++ cmake make liblapack-dev libblas-dev \
    libboost-all-dev libnuma-dev python3-pip build-essential gcc \
    libudev-dev libnl-3-dev libnl-route-3-dev ninja-build \
    pkg-config valgrind python3-dev cython3 python3-docutils pandoc \
    jq rsync >> /tmp/hydra/install.log 2>&1
  sudo pip3 install meson >> /tmp/hydra/install.log 2>&1
}

function install_armadillo() {
  echo "[$(hostname)] Downloading and building Armadillo"
  eval "mkdir -p ${HYDRA_RUNNER_ROOT}/tmp"
  eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
  wget http://sourceforge.net/projects/arma/files/armadillo-10.7.1.tar.xz >> /tmp/hydra/install.log 2>&1
  tar xf armadillo-10.7.1.tar.xz >> /tmp/hydra/install.log 2>&1
  eval "(cd armadillo-10.7.1; cmake . -DCMAKE_INSTALL_PREFIX:PATH=${HYDRA_RUNNER_ROOT}/armadillo/; make -j; make install)" \
    >> /tmp/hydra/install.log 2>&1
  rm -rf armadillo*
}

function install_nlohmann() {
  echo "[$(hostname)] Downloading and building nlohmann json"
  eval "mkdir -p ${HYDRA_RUNNER_ROOT}/tmp"
  eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
  git clone --depth=1 https://github.com/nlohmann/json.git >> /tmp/hydra/install.log 2>&1
  cd json
  mkdir -p build
  cd build
  eval "cmake .. -DCMAKE_INSTALL_PREFIX:PATH=${HYDRA_RUNNER_ROOT}/json/" >> /tmp/hydra/install.log 2>&1
  make -j >> /tmp/hydra/install.log 2>&1
  make install >> /tmp/hydra/install.log 2>&1
  eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
  rm -rf json
}

# Check whether DPDK lib is installed on this server
#    by checking RTE_SDK variable and necessary files in RTE_SDK directory
function check_dpdk() {
  dpdk_detected=1
  if [ -z "${RTE_SDK}" ]; then
    dpdk_detected=0
  fi
  res=$(ls ${RTE_SDK}/build/install/usr/local/include/rte_common.h 2> /dev/null) || :
  if [ "$?" != "0" ]; then
    dpdk_detected=0
  fi
  res=$(ls ${RTE_SDK}/build/install/usr/local/lib/x86_64-linux-gnu/librte_common_mlx5.a 2> /dev/null) || :
  if [ "$?" != "0" ]; then
    dpdk_detected=0
  fi
  res=$(cat ${RTE_SDK}/VERSION 2> /dev/null) || :
  if [ "$?" != "0" ]; then
    dpdk_detected=0
  fi
  if [ "${res}" != "20.11.3" ]; then
    echo "[$(hostname)] WARNING: Recommend DPDK version 20.11.3"
  fi
}

function install_dpdk() {
  echo "[$(hostname)] Downloading and building DPDK"
  eval "export RTE_SDK=${HYDRA_RUNNER_ROOT}/dpdk-stable-20.11.3"
  eval "cd ${HYDRA_RUNNER_ROOT}"
  wget http://fast.dpdk.org/rel/dpdk-20.11.3.tar.xz >> /tmp/hydra/install.log 2>&1
  tar xf dpdk-20.11.3.tar.xz >> /tmp/hydra/install.log 2>&1
  rm dpdk-20.11.3.tar.xz
  cd dpdk-stable-20.11.3
  meson build >> /tmp/hydra/install.log 2>&1
  cd build
  ninja >> /tmp/hydra/install.log 2>&1
  DESTDIR=${RTE_SDK}/build/install ninja install >> /tmp/hydra/install.log 2>&1
}

function install_intel_lib() {
  echo "[$(hostname)] Downloading Intel libraries for MKL (this could take several minutes)"
  eval "mkdir -p ${HYDRA_RUNNER_ROOT}/tmp"
  eval "cd ${HYDRA_RUNNER_ROOT}/tmp"
  wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18236/l_BaseKit_p_2021.4.0.3422.sh \
    >> /tmp/hydra/install.log 2>&1
  eval "bash l_BaseKit_p_2021.4.0.3422.sh -a --silent --eula accept --components \
    intel.oneapi.lin.ipp.devel:intel.oneapi.lin.mkl.devel --install-dir ${HYDRA_RUNNER_ROOT}/intel/oneapi" \
    >> /tmp/hydra/install.log 2>&1
  wget https://registrationcenter-download.intel.com/akdlm/irc_nas/18211/l_HPCKit_p_2021.4.0.3347.sh \
    >> /tmp/hydra/install.log 2>&1
  eval "bash l_HPCKit_p_2021.4.0.3347.sh -a --silent --eula accept --components \
    intel.oneapi.lin.dpcpp-cpp-compiler-pro --install-dir ${HYDRA_RUNNER_ROOT}/intel/oneapi/" \
    >> /tmp/hydra/install.log 2>&1
  rm l_BaseKit_p_2021.4.0.3422.sh l_HPCKit_p_2021.4.0.3347.sh
  eval "source ${HYDRA_RUNNER_ROOT}/intel/oneapi/setvars.sh" >> /tmp/hydra/install.log 2>&1
}

function install_rdma_core() {
  echo "[$(hostname)] Downloading and building rdma-core"
  eval "cd ${HYDRA_RUNNER_ROOT}"
  git clone https://github.com/linux-rdma/rdma-core.git >> /tmp/hydra/install.log 2>&1
  cd rdma-core
  bash build.sh >> /tmp/hydra/install.log 2>&1
  eval "export LIBRARY_PATH=${LIBRARY_PATH}:${HYDRA_RUNNER_ROOT}/rdma-core/build/lib"
  export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${LIBRARY_PATH}
}

function install_hydra() {
  echo "[$(hostname)] Building Hydra"
  eval "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh" >> /tmp/hydra/install.log 2>&1
  eval "cd ${HYDRA_RUNNER_ROOT}/Agora"
  mkdir -p build
  cd build
  cmake .. -DLOG_LEVEL=warn >> /tmp/hydra/install.log 2>&1
  make -j >> /tmp/hydra/install.log 2>&1
}

function save_progress() {
  echo "armadillo=${installed_armadillo}" > ${hydra_root_dir}/progress/progress.txt
  echo "json=${installed_json}" >> ${hydra_root_dir}/progress/progress.txt
  echo "mkl=${installed_mkl}" >> ${hydra_root_dir}/progress/progress.txt
  echo "rdma=${installed_rdma}" >> ${hydra_root_dir}/progress/progress.txt
  echo "dpdk=${installed_dpdk}" >> ${hydra_root_dir}/progress/progress.txt
}

# Install system-level packages if INSTALL_HYDRA_PKGS_SYSTEM_LEVEL is set to 1
if [ -n ${INSTALL_HYDRA_PKGS_SYSTEM_LEVEL} ]; then
  if [ ${INSTALL_HYDRA_PKGS_SYSTEM_LEVEL} == "1" ]; then
    install_sys_pkgs
  fi
fi

# Verify that system-level packages are installed
check_sys_pkgs

# Load the installation progress
source ${hydra_root_dir}/scripts/install/load_progress.sh

# Install armadillo, nlohmann JSON, Intel lib, and RDMA core
if [ "${installed_armadillo}" == "none" ]; then
  install_armadillo
  installed_armadillo="local"
  save_progress
else
  echo "[$(hostname)] Armadillo is already installed"
fi

if [ "${installed_json}" == "none" ]; then
  install_nlohmann
  installed_json="local"
  save_progress
else
  echo "[$(hostname)] nlohmann JSON is already installed"
fi

if [ "${installed_mkl}" == "none" ]; then
  install_intel_lib
  installed_mkl="local"
  save_progress
else
  echo "[$(hostname)] Intel MKL is already installed"
fi

if [ "${installed_rdma}" == "none" ]; then
  install_rdma_core
  installed_rdma="local"
  save_progress
else
  echo "[$(hostname)] RDMA core is already installed"
fi

if [ "${installed_dpdk}" == "none" ]; then
  install_dpdk
  installed_dpdk="local"
  save_progress
else
  echo "[$(hostname)] DPDK is already installed"
fi

# Build and install Hydra app
install_hydra
