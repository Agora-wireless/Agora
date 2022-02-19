#! /bin/bash

# Find the root directory of Hydra app
script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

# Set the root directory of Hydra remote runner
HYDRA_RUNNER_ROOT="~/HydraRemoteRunner"

# Read HYDRA_RUNNER_ROOT from file config/config.json
hydra_master_config_json=${hydra_root_dir}/config/config.json
if [ ! -f ${hydra_master_config_json} ]; then
  echored "ERROR: config file ${hydra_master_config_json} does not exist"
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_runner_root' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_RUNNER_ROOT=${res}
fi

source ${hydra_root_dir}/scripts/install/load_progress.sh

if [ "${installed_mkl}" == "custom1" ]; then
  eval "source /opt/intel/system_studio_2019/bin/compilervars.sh intel64"
elif [ "${installed_mkl}" == "custom2" ]; then
  eval "source /opt/intel/oneapi/setvars.sh --force"
elif [ "${installed_mkl}" == "local" ]; then
  eval "source ${HYDRA_RUNNER_ROOT}/intel/oneapi/setvars.sh --force"
fi

eval "export LIBRARY_PATH=${LIBRARY_PATH}:${HYDRA_RUNNER_ROOT}/rdma-core/build/lib"
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${LIBRARY_PATH}

if [ "${installed_dpdk}" == "local" ]; then
  eval "export RTE_SDK=${HYDRA_RUNNER_ROOT}/dpdk-stable-20.11.3"
fi