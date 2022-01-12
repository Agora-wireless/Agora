#! /bin/bash

if [ "$#" -ne 1 ]; then
    echo "illegal number of parameters (require 1 argument: interface name)"
    exit
fi

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR=${DIR}/..

hydra_platform_fn=${ROOT_DIR}/config/platform.json
hydra_server_num=$(cat ${hydra_platform_fn} | jq '. | length')

for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
    ssh -oStrictHostKeyChecking=no ${server_name} "ifconfig $1 | grep ether" | awk '{ print $2 }'
done