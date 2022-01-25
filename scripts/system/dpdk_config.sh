#! /bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

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

# Setup Hugepages and modprobe
eval "cd ${HYDRA_RUNNER_ROOT}/dpdk-stable-20.11.3"
sudo ./usertools/dpdk-hugepages.py --setup 22G
sudo modprobe -a ib_uverbs mlx5_core mlx5_ib