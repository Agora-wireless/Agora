#! /bin/bash

if [ -z "${hydra_root_dir}" ]; then
  echo "hydra_root_dir variable not set"
  exit
fi

hydra_app_name="agora"
hydra_rru_app_name="dynamic_sender"

HYDRA_SERVER_LIST_JSON=${hydra_root_dir}/config/platform.json
HYDRA_SERVER_DEPLOY_JSON=${hydra_root_dir}/config/deploy.json
HYDRA_SYSTEM_CONFIG_JSON=${hydra_root_dir}/config/template.json
HYDRA_RUNNER_ROOT="~/HydraRemoteRunner"

# Read HYDRA_SERVER_LIST_JSON and HYDRA_RUNNER_ROOT from file config/config.json
hydra_master_config_json=${hydra_root_dir}/config/config.json
if [ ! -f ${hydra_master_config_json} ]; then
  echored "ERROR: config file ${hydra_master_config_json} does not exist"
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_server_list_json' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_SERVER_LIST_JSON=${hydra_root_dir}/${res}
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_server_deploy_json' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_SERVER_DEPLOY_JSON=${hydra_root_dir}/${res}
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_system_config_json' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_SYSTEM_CONFIG_JSON=${hydra_root_dir}/${res}
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_runner_root' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_RUNNER_ROOT=${res}
fi

hydra_server_num=$(cat ${HYDRA_SERVER_LIST_JSON} | jq '. | length')

for (( i=0; i<${hydra_server_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
  pcie=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --arg name ${server_name} '.[$name] | .pcie')
  mac=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --arg name ${server_name} '.[$name] | .mac')
  ip=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --arg name ${server_name} '.[$name] | .ip')
  hydra_server_list[$i]=${server_name}
  hydra_pcie_list[$i]=${pcie}
  hydra_mac_list[$i]=${mac}
  hydra_ip_list[$i]=${ip}
done

hydra_rru_num=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.rru_servers | length')
hydra_app_num=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.hydra_servers | length')
slots_to_test=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.frames_to_test')
slot_us=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.slot_us')