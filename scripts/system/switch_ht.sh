#! /bin/bash

source $(dirname $0)/../utils/utils.sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR=${DIR}/..

mode=0

if [ "$#" -eq 0 ]; then
    echocyan "Disable hyperthreading on servers"
elif [ "$#" -eq 1 ]; then
    if [ "$1" == "enable" ]; then
        echocyan "Enable hyperthreading on servers"
        mode=1
    else
        echored "Invalid mode"
        exit
    fi
else
    echored "Invalid number of argumemts"
    exit
fi

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