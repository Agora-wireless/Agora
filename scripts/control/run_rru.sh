#! /bin/bash

set -e

# Run all RRU servers
for (( i=1; i<${hydra_rru_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
  echocyan "Run RRU server ${server_name}"
  hostname=$(hostname)
  (ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
    cd ${HYDRA_RUNNER_ROOT}/Agora; \
    sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender -t 6 \
    -c ./config/run.json -o 1" > /tmp/hydra/log_${server_name}.txt 2>&1 || echo 1 > /tmp/hydra/done_${server_name}.txt; \
    echo 1 > /tmp/hydra/done_${server_name}.txt) &
done

sleep 5

server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.rru_servers[0]' | tr -d '"')
echocyan "Run RRU server ${server_name}"
hostname=$(hostname)
if [ "${HYDRA_RUN_MODE}" == 0 ]; then
  (ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
    cd ${HYDRA_RUNNER_ROOT}/Agora; \
    sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender -t 6 \
    -c ./config/run.json -o 1" > /tmp/hydra/log_${server_name}.txt 2>&1 || echo 1 > /tmp/hydra/done_${server_name}.txt; \
    echo 1 > /tmp/hydra/done_${server_name}.txt) &
else
  ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
    cd ${HYDRA_RUNNER_ROOT}/Agora; \
    sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender -t 6 \
    -c ./config/run.json -o 1 -f" > /tmp/hydra/log_${server_name}.txt 2>&1 &
fi