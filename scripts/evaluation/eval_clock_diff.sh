#! /bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

mkdir -p /tmp/hydra
rm -f /tmp/hydra/clock_diff.txt

master_node=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.hydra_servers[0]' | tr -d '"')
ssh ${master_node} "cd ${HYDRA_RUNNER_ROOT}/Agora; source scripts/install/setvars.sh; \
    sudo LD_LIBRARY_PATH=\${LD_LIBRARY_PATH} ./build/test_clock_receiver" > /dev/null 2>&1 &

sleep 2

for (( i=1; i<${hydra_app_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    echo "Evaluate clock diff between ${master_node} and ${server_name}"
    ssh ${server_name} "cd ${HYDRA_RUNNER_ROOT}/Agora; source scripts/install/setvars.sh > /dev/null 2>&1;
        sudo LD_LIBRARY_PATH=\${LD_LIBRARY_PATH} ./build/test_clock_sender -r 0" >> /tmp/hydra/clock_diff.txt 2> /dev/null
done

ssh ${master_node} "sudo pkill test_clock"

cat /tmp/hydra/clock_diff.txt