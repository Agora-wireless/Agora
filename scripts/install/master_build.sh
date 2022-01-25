#! /bin/bash

# Run this script on your laptop
# This script helps you to run install_hydra.sh on all remote servers
#   listed on $hydra_platform_fn

set -e

# Find the root directory of Hydra app
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

HYDRA_SERVER_LIST_JSON=${hydra_root_dir}/config/platform.json
HYDRA_RUNNER_ROOT="~/HydraRemoteRunner"

checkpkg jq
if [ ${checkpkg_res} == "0" ]; then
    exit
fi

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

hydra_server_num=$(cat ${HYDRA_SERVER_LIST_JSON} | jq '. | length')

for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
    ssh -oStrictHostKeyChecking=no ${server_name} "cd ${HYDRA_RUNNER_ROOT}/Agora; \
        ./scripts/install/install_hydra.sh" &
done

wait