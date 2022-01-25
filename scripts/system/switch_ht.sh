#! /bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

# Enable or disable hyperthreading (0 for disable, 1 for enable)
mode=0

while getopts "h?:e" opt; do
    case "$opt" in
        h|\?)
            echo "Help"
            echo -e "\t-h\tShow this infomation"
            echo -e "\t-e\tEnable hyperthreading"
            exit 0
            ;;
        e)
            mode=1
            ;;
    esac
done

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

# Initialize the info of the platform:
# app_name, servers, NIC info
hydra_server_num=$(cat ${HYDRA_SERVER_LIST_JSON} | jq '. | length')

for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
    hostname=$(hostname)
    if [ "$hostname" == "$(echo ${server_name} | tr -d '"')" ]; then
        core_per_socket=$(lscpu | grep 'Core' | awk '{ print $4 }')
        num_socket=$(lscpu | grep 'Socket' | awk '{ print $2 }')
        num_core=$(( core_per_socket*num_socket ))
        core_high=$(( num_core*2 ))
        if [ "$mode" -eq "0" ]; then
            echocyan "Disable hyperthreading on ${server_name}"
            sudo ${hydra_root_dir}/scripts/system/disable_ht.sh ${num_core} ${core_high} 0
        else
            echocyan "Enable hyperthreading on ${server_name}"
            sudo ${hydra_root_dir}/scripts/system/disable_ht.sh ${num_core} ${core_high} 1
        fi
    else
        core_per_socket=$(ssh -oStrictHostKeyChecking=no ${server_name} "lscpu | grep 'Core'" | awk '{ print $4 }')
        num_socket=$(ssh -oStrictHostKeyChecking=no ${server_name} "lscpu | grep 'Socket'" | awk '{ print $2 }')
        num_core=$(( core_per_socket*num_socket ))
        core_high=$(( num_core*2 ))
        if [ "$mode" -eq "0" ]; then
            echocyan "Disable hyperthreading on ${server_name}"
            ssh -oStrictHostKeyChecking=no ${server_name} "cd ${HYDRA_RUNNER_ROOT}/Agora; \
                sudo ./scripts/system/disable_ht.sh ${num_core} ${core_high} 0"
        else
            echocyan "Enable hyperthreading on ${server_name}"
            ssh -oStrictHostKeyChecking=no ${server_name} "cd ${HYDRA_RUNNER_ROOT}/Agora; \
                sudo ./scripts/system/disable_ht.sh ${num_core} ${core_high} 1"
        fi
    fi
done