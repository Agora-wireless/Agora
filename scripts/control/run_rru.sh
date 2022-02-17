#! /bin/bash

set -e

# Run all RRU servers
for (( i=1; i<${hydra_rru_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
  echocyan "Run RRU server ${server_name}"
  hostname=$(hostname)
  if [ "${hostname}" == "${server_name}" ]; then
    sudo env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 ${hydra_root_dir}/build/dynamic_sender -t 6 \
      -c ${hydra_root_dir}/config/run.json \
      -d $SLOT_US -o 0 > /tmp/Hydra/log_${server_name}.txt 2>&1 &
  else
    ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
      cd ${HYDRA_RUNNER_ROOT}/Agora; \
      sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender -t 6 \
      -c ./config/run.json -d ${SLOT_US} -o 0" > /tmp/Hydra/log_${server_name}.txt 2>&1 &
  fi
done

server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.rru_servers[0]' | tr -d '"')
echocyan "Run RRU server ${server_name}"
hostname=$(hostname)
if [ "${hostname}" == "${server_name}" ]; then
  sudo env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 ${hydra_root_dir}/build/dynamic_sender -t 6 \
    -c ${hydra_root_dir}/config/run.json \
    -d $SLOT_US -o 0 > /tmp/Hydra/log_${server_name}.txt 2>&1 &
else
  ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
    cd ${HYDRA_RUNNER_ROOT}/Agora; \
    sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender -t 6 \
    -c ./config/run.json -d ${SLOT_US} -o 0" > /tmp/Hydra/log_${server_name}.txt 2>&1 &
fi