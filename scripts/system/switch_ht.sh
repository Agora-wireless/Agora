#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR=$( cd ${DIR}/../.. >/dev/null 2>&1 && pwd )

source ${ROOT_DIR}/scripts/utils/utils.sh

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

# Initialize the info of the platform:
# app_name, servers, NIC info
hydra_platform_fn=${ROOT_DIR}/config/platform.json
hydra_server_num=$(cat ${hydra_platform_fn} | jq '. | length')

for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
    hostname=$(hostname)
    if [ "$hostname" == "$(echo ${server_name} | tr -d '"')" ]; then
        core_per_socket=$(lscpu | grep 'Core' | awk '{ print $4 }')
        num_socket=$(lscpu | grep 'Socket' | awk '{ print $2 }')
        num_core=$(( core_per_socket*num_socket ))
        core_high=$(( num_core*2 ))
        if [ "$mode" -eq "0" ]; then
            echocyan "Disable hyperthreading on ${server_name}"
            sudo ${ROOT_DIR}/scripts/system/disable_ht.sh ${num_core} ${core_high} 0
        else
            echocyan "Enable hyperthreading on ${server_name}"
            sudo ${ROOT_DIR}/scripts/system/disable_ht.sh ${num_core} ${core_high} 1
        fi
    else
        core_per_socket=$(ssh -oStrictHostKeyChecking=no ${server_name} "lscpu | grep 'Core'" | awk '{ print $4 }')
        num_socket=$(ssh -oStrictHostKeyChecking=no ${server_name} "lscpu | grep 'Socket'" | awk '{ print $2 }')
        num_core=$(( core_per_socket*num_socket ))
        core_high=$(( num_core*2 ))
        if [ "$mode" -eq "0" ]; then
            echocyan "Disable hyperthreading on ${server_name}"
            ssh -oStrictHostKeyChecking=no ${server_name} "cd ~/project/Agora; sudo ./scripts/system/disable_ht.sh ${num_core} ${core_high} 0"
        else
            echocyan "Enable hyperthreading on ${server_name}"
            ssh -oStrictHostKeyChecking=no ${server_name} "cd ~/project/Agora; sudo ./scripts/system/disable_ht.sh ${num_core} ${core_high} 1"
        fi
    fi
done