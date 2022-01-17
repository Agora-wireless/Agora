#! /bin/bash

# Run all RRU servers
for (( i=1; i<${hydra_rru_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    echocyan "Run RRU server ${server_name}"
    hostname=$(hostname)
    if [ "${hostname}" == "${server_name}" ]; then
        sudo env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 --conf_file=${ROOT_DIR}/config/run.json \
            --frame_duration=$slot_us --core_offset=0 > /tmp/Hydra/log_${server_name}.txt &
    else
        ssh -oStrictHostKeyChecking=no ${server_name} "source ~/.bash_profile; \
            cd ~/project/Agora; \
            sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 \
                --conf_file=./config/run.json --frame_duration=${slot_us} --core_offset=0" > /tmp/Hydra/log_${server_name}.txt &
    fi
done

server_name=$(cat ${hydra_deploy_fn} | jq '.rru_servers[0]' | tr -d '"')
echocyan "Run RRU server ${server_name}"
hostname=$(hostname)
if [ "${hostname}" == "${server_name}" ]; then
    sudo env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 --conf_file=${ROOT_DIR}/config/run.json \
        --frame_duration=$slot_us --core_offset=0 > /tmp/Hydra/log_${server_name}.txt &
else
    ssh -oStrictHostKeyChecking=no ${server_name} "source ~/.bash_profile; \
        cd ~/project/Agora; \
        sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 \
            --conf_file=./config/run.json --frame_duration=${slot_us} --core_offset=0" > /tmp/Hydra/log_${server_name}.txt &
fi