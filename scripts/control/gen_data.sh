#! /bin/bash

# Generate control data and traffic data
server_name=$(cat ${hydra_deploy_fn} | jq '.rru_servers[0]' | tr -d '"')
echocyan "Run control and config generator on ${server_name}"
hostname=$(hostname)
if [ "${hostname}" == "${server_name}" ]; then
    sudo ${ROOT_DIR}/build/control_generator --conf_file ${ROOT_DIR}/config/run.json
    sudo ${ROOT_DIR}/build/data_generator --conf_file ${ROOT_DIR}/config/run.json
else
    ssh -oStrictHostKeyChecking=no ${server_name} "source ~/.bash_profile; \
        cd ~/project/Agora; \
        ./build/control_generator --conf_file ./config/run.json; \
        ./build/dynamic_generator --conf_file ./config/run.json;"
fi
num_antennas=$(cat ${hydra_template_fn} | jq '.antenna_num')
scp -oStrictHostKeyChecking=no ${server_name}:~/project/Agora/data/control_ue_template.bin ${ROOT_DIR}/data/
scp -oStrictHostKeyChecking=no ${server_name}:~/project/Agora/data/control_ue.bin ${ROOT_DIR}/data/
scp -oStrictHostKeyChecking=no ${server_name}:~/project/Agora/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${ROOT_DIR}/data/

# Deploy data to Hydra servers
for (( i=0; i<${hydra_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    hostname=$(hostname)
    if [ "${hostname}" == "${server_name}" ]; then
        sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 \
            ./build/agora --conf_file config/run.json &
    else
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue_template.bin ${server_name}:~/project/Agora/data/
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue.bin ${server_name}:~/project/Agora/data/
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${server_name}:~/project/Agora/data/
    fi
done

# Deploy data to RRU servers
for (( i=0; i<${hydra_rru_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    hostname=$(hostname)
    if [ "${hostname}" == "${server_name}" ]; then
        sudo env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 --conf_file=${ROOT_DIR}/config/run.json \
            --frame_duration=$slot_us --core_offset=0 > /tmp/Hydra/log_${server_name}.txt &
    else
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue_template.bin ${server_name}:~/project/Agora/data/
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue.bin ${server_name}:~/project/Agora/data/
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${server_name}:~/project/Agora/data/
    fi
done