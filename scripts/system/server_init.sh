#! /bin/bash

# Please ignore this script
# You don't need this

set -e

# Find the root directory of Hydra project
script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

HYDRA_RUNNER_ROOT="~/HydraRemoteRunner"

sudo apt-get update
sudo apt-get install -y jq

# Read HYDRA_RUNNER_ROOT from file config/config.json
hydra_master_config_json=${hydra_root_dir}/config/config.json
if [ ! -f ${hydra_master_config_json} ]; then
  echo "ERROR: config file ${hydra_master_config_json} does not exist"
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_runner_root' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_RUNNER_ROOT=${res}
fi

sudo apt-get update
sudo apt-get install -y byobu
eval "mkdir -p ${HYDRA_RUNNER_ROOT}"
sudo mkfs -t ext3 /dev/sda4 <<< y
eval "sudo mount -t auto /dev/sda4 ${HYDRA_RUNNER_ROOT}"
eval "sudo chown junzhig ${HYDRA_RUNNER_ROOT}"
eval "sudo chgrp opensketch-PG0 ${HYDRA_RUNNER_ROOT}"

echo "1" | sudo tee /sys/devices/system/cpu/intel_pstate/no_turbo
sudo apt-get install -y linux-tools-$(uname -r)
sudo cpupower frequency-set -g performance
${hydra_root_dir}/scripts/system/set_smp_affinity.sh
