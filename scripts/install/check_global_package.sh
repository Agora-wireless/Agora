#! /bin/bash

# Find the root directory of Hydra app
script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

function check_global_armadillo() {
  res=$(ldconfig -p | grep libarmadillo) || :
  if [ -n "${res}" ]; then
    installed_armadillo="global"
  fi
}

function check_global_json() {
  if [ -f "/usr/local/include/nlohmann/json.hpp" ]; then
    installed_json="global"
  fi
}

function check_global_mkl() {
  res=$(ldconfig -p | grep libmkl) || :
  if [ -n "${res}" ]; then
    installed_mkl="global"
    return
  fi
  if [ -f "/opt/intel/system_studio_2019/bin/compilervars.sh" ]; then
    installed_mkl="custom1"
    return
  fi
  if [ -f "/opt/intel/oneapi/setvars.sh" ]; then
    installed_mkl="custom2"
    return
  fi
}

function check_global_dpdk() {
  if [ -n "${RTE_SDK}" ]; then
    installed_dpdk="global"
  fi
}