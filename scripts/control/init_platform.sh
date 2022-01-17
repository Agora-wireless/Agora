#! /bin/bash

if [ -z "${ROOT_DIR}" ]; then
    echo "ROOT_DIR variable not set"
    exit
fi

hydra_app_name="agora"
hydra_platform_fn=${ROOT_DIR}/config/platform.json
hydra_server_num=$(cat ${hydra_platform_fn} | jq '. | length')

for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
    pcie=$(cat ${ROOT_DIR}/config/platform.json | jq --arg name ${server_name} '.[$name] | .pcie')
    mac=$(cat ${ROOT_DIR}/config/platform.json | jq --arg name ${server_name} '.[$name] | .mac')
    ip=$(cat ${ROOT_DIR}/config/platform.json | jq --arg name ${server_name} '.[$name] | .ip')
    hydra_server_list[$i]=${server_name}
    hydra_pcie_list[$i]=${pcie}
    hydra_mac_list[$i]=${mac}
    hydra_ip_list[$i]=${ip}
done

hydra_deploy_fn=${ROOT_DIR}/config/deploy.json
hydra_rru_num=$(cat ${hydra_deploy_fn} | jq '.rru_servers | length')
hydra_num=$(cat ${hydra_deploy_fn} | jq '.hydra_servers | length')
hydra_template_fn=${ROOT_DIR}/config/template.json