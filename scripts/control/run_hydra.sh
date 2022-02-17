#! /bin/bash

set -e

# Run all Hydra servers
for (( i=0; i<${hydra_app_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
  echo "Run hydra server ${server_name}"
  hostname=$(hostname)
  if [ "${hostname}" == "${server_name}" ]; then
    sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 \
      ${hydra_root_dir}/build/agora -c ${hydra_root_dir}/config/run.json > /tmp/Hydra/log_${server_name}.txt 2>&1 &
  else
    ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh;\
      cd ${HYDRA_RUNNER_ROOT}/Agora; \
      sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/agora -c config/run.json" \
      > /tmp/Hydra/log_${server_name}.txt 2>&1 &
  fi
done