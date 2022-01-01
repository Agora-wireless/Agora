#! /bin/bash

source $(dirname $0)/utils.sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR=${DIR}/..

# Initialize the info of the platform:
# app_name, servers, NIC info
hydra_app_name="agora"
hydra_rru_app_name="dynamic_sender"
hydra_deploy_fn=${ROOT_DIR}/config/deploy.json
hydra_rru_num=$(cat ${hydra_deploy_fn} | jq '.rru_servers | length')
hydra_num=$(cat ${hydra_deploy_fn} | jq '.hydra_servers | length')

# Kill all RRU processes
echocyan "Kill all RRU processes..."
for (( i=0; i<${hydra_rru_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    hostname=$(hostname)
    echo "Kill ${hydra_rru_app_name} process on ${server_name}"
    if [ "$hostname" == "${server_name}" ]; then
        sudo pkill ${hydra_rru_app_name} 1> /dev/null 2> /dev/null
    else
        ssh -oStrictHostKeyChecking=no ${server_name} "sudo pkill ${hydra_rru_app_name} 1> /dev/null 2> /dev/null" 
    fi
done

# Kill all Hydra processes
echocyan "Kill all Hydra processes..."
for (( i=0; i<${hydra_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    hostname=$(hostname)
    echo "Kill ${hydra_app_name} process on ${server_name}"
    if [ "$hostname" == "${server_name}" ]; then
        sudo pkill ${hydra_app_name} 1> /dev/null 2> /dev/null
    else
        ssh -oStrictHostKeyChecking=no ${server_name} "sudo pkill ${hydra_app_name} 1> /dev/null 2> /dev/null" 
    fi
done

# Check running processes
# echocyan "Check all processes..."
for (( i=0; i<${hydra_rru_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    hostname=$(hostname)
    # echo "Check process on ${server_name}"
    if [ "$hostname" == "${server_name}" ]; then
        ret=$(pgrep -x  ${hydra_rru_app_name})
        if [ -n "${ret}" ]; then
            echored "${hydra_rru_app_name} still runs on ${server_name}"
        fi
    else
        ret=$(ssh -oStrictHostKeyChecking=no ${server_name} "pgrep -x ${hydra_rru_app_name}")
        if [ -n "${ret}" ]; then
            echored "${hydra_rru_app_name} still runs on ${server_name}"
        fi
    fi
done
for (( i=0; i<${hydra_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    hostname=$(hostname)
    # echo "Check process on ${server_name}"
    if [ "$hostname" == "${server_name}" ]; then
        ret=$(pgrep -x  ${hydra_app_name})
        if [ -n "${ret}" ]; then
            echored "${hydra_app_name} still runs on ${server_name}"
        fi
    else
        ret=$(ssh -oStrictHostKeyChecking=no ${server_name} "pgrep -x ${hydra_app_name}")
        if [ -n "${ret}" ]; then
            echored "${hydra_app_name} still runs on ${server_name}"
        fi
    fi
done