#! /bin/bash

# This script runs on your laptop
# This script sync all changes from your laptop to all remote servers

script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

checkpkg jq
if [ ${checkpkg_res} == "0" ]; then
  exit
fi

HYDRA_SERVER_LIST_JSON=${hydra_root_dir}/config/platform.json
HYDRA_RUNNER_ROOT="~/HydraRemoteRunner"

# Read HYDRA_SERVER_LIST_JSON and HYDRA_RUNNER_ROOT from file config/config.json
hydra_master_config_json=${hydra_root_dir}/config/config.json
if [ ! -f ${hydra_master_config_json} ]; then
  echored "ERROR: config file ${hydra_master_config_json} does not exist"
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_server_list_json' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_SERVER_LIST_JSON=${hydra_root_dir}/${res}
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_runner_root' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_RUNNER_ROOT=${res}
fi

# Initialize the info of the platform:
# app_name, servers, NIC info
hydra_server_num=$(cat ${HYDRA_SERVER_LIST_JSON} | jq '. | length')

for (( i=0; i<${hydra_server_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --argjson i $i '. | keys | .[$i]')
  hostname=$(hostname)
  echo "Copying ${hydra_root_dir}, ${hostname} -> $(echo ${server_name} | tr -d '"') at ${HYDRA_RUNNER_ROOT}/Agora/"
  eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/src $(echo ${server_name} | tr -d '"'):${HYDRA_RUNNER_ROOT}/Agora/"
  eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/simulator $(echo ${server_name} | tr -d '"'):${HYDRA_RUNNER_ROOT}/Agora/"
  eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/scripts $(echo ${server_name} | tr -d '"'):${HYDRA_RUNNER_ROOT}/Agora/"
  eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/config $(echo ${server_name} | tr -d '"'):${HYDRA_RUNNER_ROOT}/Agora/"
  eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/data $(echo ${server_name} | tr -d '"'):${HYDRA_RUNNER_ROOT}/Agora/"
  eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/test $(echo ${server_name} | tr -d '"'):${HYDRA_RUNNER_ROOT}/Agora/"
  eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/tool $(echo ${server_name} | tr -d '"'):${HYDRA_RUNNER_ROOT}/Agora/"
  eval "rsync -a ${hydra_root_dir}/CMakeLists.txt $(echo ${server_name} | tr -d '"'):${HYDRA_RUNNER_ROOT}/Agora/"
done
