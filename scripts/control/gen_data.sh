#! /bin/bash

set -e

# Generate control data and traffic data
server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.rru_servers[0]' | tr -d '"')
echocyan "Run control and config generator on ${server_name}"
num_antennas=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.antenna_num')
num_users=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.ue_num')
echo "********** Running control generator **********" >> /tmp/hydra/install.log
ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
  cd ${HYDRA_RUNNER_ROOT}/Agora; \
  ./build/control_generator -c ./config/run.json" >> /tmp/hydra/install.log 2>&1 || \
  { echo "Failed to run control generator on ${server_name}; Please check /tmp/hydra/install.log for more details";
    exit 1; }
echo "********** Running traffic data generator **********" >> /tmp/hydra/install.log
if [ "${USE_MATLAB_GEN_RAYLEIGH}" == 1 ]; then
  # ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
  #   cd ${HYDRA_RUNNER_ROOT}/Agora; \
  #   ./build/dynamic_generator -c ./config/run.json -m prechannel" >> /tmp/hydra/install.log 2>&1 || \
  #   { echo "Failed to run traffic data generator (prechannel mode) on ${server_name}"; exit 1; }
  # scp -oStrictHostKeyChecking=no ${server_name}:/tmp/hydra/matlab_input.txt /tmp/hydra/ >> /tmp/hydra/install.log
  # cd ${hydra_root_dir}/matlab; matlab -batch "generate_uplink(${num_users}, ${num_antennas}, \
  #   '/tmp/hydra/matlab_input.txt', '/tmp/hydra/matlab_output.txt')" >> /tmp/hydra/install.log
  # scp -oStrictHostKeyChecking=no /tmp/hydra/matlab_output.txt ${server_name}:/tmp/hydra/ >> /tmp/hydra/install.log
  # ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
  #   cd ${HYDRA_RUNNER_ROOT}/Agora; \
  #   ./build/dynamic_generator -c ./config/run.json -m postchannel" >> /tmp/hydra/install.log 2>&1 || \
  #   { echo "Failed to run traffic data generator (postchannel mode) on ${server_name}"; exit 1; }
  cp ${hydra_root_dir}/data/matlab_data/LDPC_rx_data_2048_ant${num_antennas}_ue${num_users}.bin \
    ${hydra_root_dir}/data/LDPC_rx_data_2048_ant${num_antennas}.bin
else
  ssh -oStrictHostKeyChecking=no ${server_name} "source ${HYDRA_RUNNER_ROOT}/Agora/scripts/install/setvars.sh; \
    cd ${HYDRA_RUNNER_ROOT}/Agora; \
    ./build/dynamic_generator -c ./config/run.json" >> /tmp/hydra/install.log 2>&1 || \
    { echo "Failed to run traffic data generator on ${server_name}"; exit 1; }
fi
echo "********** Copying generated data back to master **********" >> /tmp/hydra/install.log
eval "scp -oStrictHostKeyChecking=no ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/control_ue_template.bin ${hydra_root_dir}/data/" >> /tmp/hydra/install.log
eval "scp -oStrictHostKeyChecking=no ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/control_ue.bin ${hydra_root_dir}/data/" >> /tmp/hydra/install.log
if [ "${USE_MATLAB_GEN_RAYLEIGH}" != 1 ]; then
  eval "scp -oStrictHostKeyChecking=no ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${hydra_root_dir}/data/" >> /tmp/hydra/install.log
fi

# Deploy data to Hydra servers
echo "********** Deploying data to Hydra servers **********" >> /tmp/hydra/install.log
for (( i=0; i<${hydra_app_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
  eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/control_ue_template.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" >> /tmp/hydra/install.log
  eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/control_ue.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" >> /tmp/hydra/install.log
  eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" >> /tmp/hydra/install.log
done

# Deploy data to RRU servers
echo "********** Deploying data to RRU servers **********" >> /tmp/hydra/install.log
for (( i=0; i<${hydra_rru_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
  eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/control_ue_template.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" >> /tmp/hydra/install.log
  eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/control_ue.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" >> /tmp/hydra/install.log
  eval "scp -oStrictHostKeyChecking=no ${hydra_root_dir}/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/" >> /tmp/hydra/install.log
done