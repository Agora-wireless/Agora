#! /bin/bash

# Run all Hydra servers (TODO: complete)
for (( i=0; i<${hydra_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    echo "Run hydra server ${server_name}"
    hostname=$(hostname)
    if [ "${hostname}" == "${server_name}" ]; then
        sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 \
            ./build/agora --conf_file config/run.json &
    else
        ssh -oStrictHostKeyChecking=no ${server_name} "source ~/.bash_profile;\
            cd ~/project/Agora; \
            sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/agora --conf_file config/run.json" > /tmp/Hydra/log_${server_name}.txt &
    fi
done