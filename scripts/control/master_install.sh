#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR=${DIR}/..

hydra_platform_fn=${ROOT_DIR}/config/platform.json
hydra_server_num=$(cat ${hydra_platform_fn} | jq '. | length')
for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
    rsync -a --exclude '*.bin' . ${server_name}:~/Agora
    ssh -oStrictHostKeyChecking=no ${server_name} "cd Agora; ./scripts/system/server_init.sh; cd ~/project/Agora; ./scripts/install/install_all.sh" &
done

wait