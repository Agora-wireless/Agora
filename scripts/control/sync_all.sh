#! /bin/bash

# This script runs on your laptop
# This script sync all changes from your laptop to all remote servers

script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

checkpkg jq
if [ ${checkpkg_res} == "0" ]; then
  exit
fi

HYDRA_SERVER_DEPLOY_JSON=${hydra_root_dir}/config/deploy.json
HYDRA_RUNNER_ROOT="~/HydraRemoteRunner"

# Read HYDRA_SERVER_DEPLOY_JSON and HYDRA_RUNNER_ROOT from file config/config.json
hydra_master_config_json=${hydra_root_dir}/config/config.json
if [ ! -f ${hydra_master_config_json} ]; then
  echored "ERROR: config file ${hydra_master_config_json} does not exist"
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_server_deploy_json' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_SERVER_DEPLOY_JSON=${hydra_root_dir}/${res}
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_runner_root' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_RUNNER_ROOT=${res}
fi

# Initialize the info of the platform:
# app_name, servers, NIC info
rru_server_num=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.rru_servers | length')
hydra_server_num=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.hydra_servers | length')

for (( i=0; i<${rru_server_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
  ( echo "Copying ${hydra_root_dir} to ${server_name} at ${HYDRA_RUNNER_ROOT}/Agora/"; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/src ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/simulator ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/scripts ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/config ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/data ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/test ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/tool ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a ${hydra_root_dir}/CMakeLists.txt ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue ) &
done

for (( i=0; i<${hydra_server_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
  ( echo "Copying ${hydra_root_dir} to ${server_name} at ${HYDRA_RUNNER_ROOT}/Agora/"; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/src ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/simulator ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/scripts ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/config ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/data ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/test ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a --exclude '*.bin' ${hydra_root_dir}/tool ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue; \
    eval "rsync -a ${hydra_root_dir}/CMakeLists.txt ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/" || continue ) &
done

wait

echo "Copying ${hydra_root_dir} to all remote servers done"