#! /bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
ROOT_DIR=$( cd ${DIR}/../.. >/dev/null 2>&1 && pwd )

inits=0
target="null"
all=0

while getopts "h?:is:a" opt; do
    case "$opt" in
        h|\?)
            echo "Help"
            echo -e "\t-h\tShow this infomation"
            echo -e "\t-i\tRun server init"
            exit 0
            ;;
        i)
            inits=1
            ;;
        s)
            target="${OPTARG}"
            ;;
        a)
            all=1
            ;;
    esac
done

hydra_platform_fn=${ROOT_DIR}/config/platform.json
hydra_server_num=$(cat ${hydra_platform_fn} | jq '. | length')
if [ ${target} == "null" ]; then
    for (( i=0; i<${hydra_server_num}; i++ )) do
        server_name=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
        rsync -a --exclude '*.bin' . ${server_name}:~/Agora
        if [ "${inits}" == 1 ]; then
            ssh -oStrictHostKeyChecking=no ${server_name} "cd Agora; ./scripts/system/server_init.sh" 
        fi
        ssh -oStrictHostKeyChecking=no ${server_name} "mkdir -p ~/project; cd ~/project; cp -r ~/Agora ./; rm -rf ~/Agora"
        ssh -oStrictHostKeyChecking=no ${server_name} "cd ~/project/Agora; SYSTEM_INSTALL=${all} ./scripts/install/install_all.sh" &
    done
else
    server_name=${target}
    rsync -a --exclude '*.bin' . ${server_name}:~/Agora
    if [ "${inits}" == 1 ]; then
        ssh -oStrictHostKeyChecking=no ${server_name} "cd Agora; ./scripts/system/server_init.sh" 
    fi
    ssh -oStrictHostKeyChecking=no ${server_name} "mkdir -p ~/project; cd ~/project; cp -r ~/Agora ./; rm -rf ~/Agora"
    ssh -oStrictHostKeyChecking=no ${server_name} "cd ~/project/Agora; SYSTEM_INSTALL=${all} ./scripts/install/install_all.sh" &
fi

wait