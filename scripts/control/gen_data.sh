#! /bin/bash

set -e

# Generate control data and traffic data
server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.rru_servers[0]' | tr -d '"')
echocyan "Run control and config generator on ${server_name}"
hostname=$(hostname)
num_antennas=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.antenna_num')
num_users=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.ue_num')
if [ "${hostname}" == "${server_name}" ]; then
  sudo ${hydra_root_dir}/build/control_generator -c ${hydra_root_dir}/config/run.json > /dev/null
  if [ "${USE_MATLAB_GEN_RAYLEIGH}" == 1 ]; then
    sudo ${hydra_root_dir}/build/dynamic_generator -c ${hydra_root_dir}/config/run.json -m prechannel > /dev/null
    cd ${hydra_root_dir}/matlab; matlab -batch "generate_uplink(${num_users}, ${num_antennas})" > /dev/null
    sudo ${hydra_root_dir}/build/dynamic_generator -c ${hydra_root_dir}/config/run.json -m postchannel > /dev/null
  else
    sudo ${hydra_root_dir}/build/dynamic_generator -c ${hydra_root_dir}/config/run.json > /dev/null
  fi
else
  ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
    cd ${HYDRA_RUNNER_ROOT}/Agora; \
    ./build/control_generator -c ./config/run.json" > /dev/null
  if [ "${USE_MATLAB_GEN_RAYLEIGH}" == 1 ]; then
    ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
      cd ${HYDRA_RUNNER_ROOT}/Agora; \
      ./build/dynamic_generator -c ./config/run.json -m prechannel" > /dev/null
    scp -oStrictHostKeyChecking=no ${server_name}:/tmp/Hydra/matlab_input.txt /tmp/Hydra/ > /dev/null
    cd ${hydra_root_dir}/matlab; matlab -batch "generate_uplink(${num_users}, ${num_antennas})" > /dev/null
    scp -oStrictHostKeyChecking=no /tmp/Hydra/matlab_output.txt ${server_name}:/tmp/Hydra/ > /dev/null
    ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
      cd ${HYDRA_RUNNER_ROOT}/Agora; \
      ./build/dynamic_generator -c ./config/run.json -m postchannel" > /dev/null
  else
    ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
      cd ${HYDRA_RUNNER_ROOT}/Agora; \
      ./build/dynamic_generator -c ./config/run.json" > /dev/null
  fi
fi
eval "scp -oStrictHostKeyChecking=no ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/control_ue_template.bin ${hydra_root_dir}/data/" > /dev/null
eval "scp -oStrictHostKeyChecking=no ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/control_ue.bin ${hydra_root_dir}/data/" > /dev/null
eval "scp -oStrictHostKeyChecking=no ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${hydra_root_dir}/data/" > /dev/null

# Deploy data to Hydra servers
for (( i=0; i<${hydra_app_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  if [ "${hostname}" != "${server_name}" ]; then
    eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/control_ue_template.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" > /dev/null
    eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/control_ue.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" > /dev/null
    eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" > /dev/null
  fi
done

# Deploy data to RRU servers
for (( i=0; i<${hydra_rru_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  if [ "${hostname}" != "${server_name}" ]; then
    eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/control_ue_template.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" > /dev/null
    eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/control_ue.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" > /dev/null
    eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" > /dev/null
  fi
done