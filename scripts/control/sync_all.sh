#! /bin/bash

source $(dirname $0)/../utils/utils.sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR=${DIR}/..

# Initialize the info of the platform:
# app_name, servers, NIC info
hydra_platform_fn=${ROOT_DIR}/config/platform.json
hydra_server_num=$(cat ${hydra_platform_fn} | jq '. | length')

for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson i $i '. | keys | .[$i]')
    hostname=$(hostname)
    if [ "$hostname" == "$(echo ${server_name} | tr -d '"')" ]; then
        continue
    fi
    echo "Run rsync $hostname->$(echo ${server_name} | tr -d '"')"
    rsync -a --exclude '*.bin' ${ROOT_DIR}/src $(echo ${server_name} | tr -d '"'):~/project/Agora/
    rsync -a --exclude '*.bin' ${ROOT_DIR}/simulator $(echo ${server_name} | tr -d '"'):~/project/Agora/
    rsync -a --exclude '*.bin' ${ROOT_DIR}/scripts $(echo ${server_name} | tr -d '"'):~/project/Agora/
    rsync -a --exclude '*.bin' ${ROOT_DIR}/config $(echo ${server_name} | tr -d '"'):~/project/Agora/
done
