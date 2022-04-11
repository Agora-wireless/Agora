#! /bin/bash

set -e

# Run all Hydra servers
for (( i=0; i<${hydra_app_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
  echo "Run hydra server ${server_name}"
  hostname=$(hostname)
  (ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh;\
    mkdir -p /tmp/hydra; echo 0 > /tmp/hydra/ready; cd ${HYDRA_RUNNER_ROOT}/Agora; \
    sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/agora -c config/run.json" \
    > /tmp/hydra/log_${server_name}.txt 2>&1 || echo 1 > /tmp/hydra/done_${server_name}.txt; echo 1 > /tmp/hydra/done_${server_name}.txt) & \
    # echo 1 > /tmp/hydra/done_${server_name}.txt) &
done