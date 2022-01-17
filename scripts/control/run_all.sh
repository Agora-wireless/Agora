#! /bin/bash

source $(dirname $0)/../utils/utils.sh

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
ROOT_DIR=${DIR}/..

# Run modes
mode=0
slot_us=1000

if [ "$#" -eq 0 ]; then
    echocyan "Default mode: running the whole system"
elif [ "$#" -eq 1 ]; then
    if [ "$1" == "rru" ]; then
        echocyan "Running in RRU mode"
        mode=1
    else
        echored "Invalid mode"
        exit
    fi
else
    echored "Invalid number of argumemts"
    exit
fi

# Initialize the info of the platform:
# app_name, servers, NIC info
hydra_app_name="agora"
hydra_platform_fn=${ROOT_DIR}/config/platform.json
hydra_server_num=$(cat ${hydra_platform_fn} | jq '. | length')

for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson i $i '. | keys | .[$i]')
    pcie=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson name ${server_name} '.[$name] | .pcie')
    mac=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson name ${server_name} '.[$name] | .mac')
    ip=$(cat ${ROOT_DIR}/config/platform.json | jq --argjson name ${server_name} '.[$name] | .ip')
    hydra_server_list[$i]=${server_name}
    hydra_pcie_list[$i]=${pcie}
    hydra_mac_list[$i]=${mac}
    hydra_ip_list[$i]=${ip}
done

hydra_deploy_fn=${ROOT_DIR}/config/deploy.json
hydra_rru_num=$(cat ${hydra_deploy_fn} | jq '.rru_servers | length')
hydra_num=$(cat ${hydra_deploy_fn} | jq '.hydra_servers | length')

# Check list length
sc_num_list_len=$(cat ${hydra_deploy_fn} | jq '.subcarrier_num_list | length')
sc_block_list_len=$(cat ${hydra_deploy_fn} | jq '.subcarrier_block_list | length')
dc_thread_list_len=$(cat ${hydra_deploy_fn} | jq '.decode_thread_num | length')
if [ "$hydra_num" -ne "$sc_num_list_len" ]; then
    echored "ERROR: hydra_servers != subcarrier_num_list"
    exit
fi
if [ "$hydra_num" -ne "$sc_block_list_len" ]; then
    echored "ERROR: hydra_servers != subcarrier_block_list"
    exit
fi
if [ "$hydra_num" -ne "$dc_thread_list_len" ]; then
    echored "ERROR: hydra_servers != decode_thread_num"
    exit
fi

# Check ofdm_data_num
hydra_template_fn=${ROOT_DIR}/config/template.json
sc_num=$(cat ${hydra_template_fn} | jq '.ofdm_data_num')
ue_num=$(cat ${hydra_template_fn} | jq '.ue_num')
sc_ue_mol=$(( ${sc_num}%${ue_num} ))
if [ "${sc_ue_mol}" -ne "0" ]; then
    echored "ERROR: ofdm_data_num % ue_num != 0"
    exit
fi
sc_ue_mol=$(( ${sc_num}%8 ))
if [ "${sc_ue_mol}" -ne "0" ]; then
    echored "ERROR: ofdm_data_num % 8 != 0"
    exit
fi

# Check subcarrier_num_list
sc_num_check=0
for (( i=0; i<${hydra_num}; i++ )) do
    sc_num_cur=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.subcarrier_num_list[$i]')
    sc_ue_mol=$(( ${sc_num_cur}%${ue_num} ))
    if [ "${sc_ue_mol}" -ne "0" ]; then
        echored "ERROR: subcarrier_num_list[$i] % ue_num != 0"
        exit
    fi
    sc_ue_mol=$(( ${sc_num_cur}%8 ))
    if [ "${sc_ue_mol}" -ne "0" ]; then
        echored "ERROR: subcarrier_num_list[$i] % 8 != 0"
        exit
    fi
    sc_num_check=$(( ${sc_num_check}+${sc_num_cur} ))
done
if [ "${sc_num}" -ne "${sc_num_check}" ]; then
    echored "ERROR: the sum of subcarrier_num_list != ofdm_data_num"
    exit
fi

# Check subcarrier_block_list
for (( i=0; i<${hydra_num}; i++ )) do
    sc_block_cur=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.subcarrier_block_list[$i]')
    sc_ue_mol=$(( ${sc_block_cur}%8 ))
    if [ "${sc_ue_mol}" -ne "0" ]; then
        echored "ERROR: subcarrier_block_list[$i] % 8 != 0"
        exit
    fi
    if [ "${sc_block_cur}" == "0" ]; then
        echored "ERROR: subcarrier_block_list[$i] == 0"
        exit
    fi
done

# Create config for RRU
sc_num_list=$(cat ${hydra_deploy_fn} | jq '.subcarrier_num_list')
sc_block_list=$(cat ${hydra_deploy_fn} | jq '.subcarrier_block_list')
dc_thread_list=$(cat ${hydra_deploy_fn} | jq '.decode_thread_num')
for (( i=0; i<${hydra_rru_num}; i++ )) do
    cat ${hydra_template_fn} | jq --argjson list "${sc_num_list}" '.subcarrier_num_list=$list' | \
        jq --argjson list "${sc_block_list}" '.subcarrier_block_list=$list' | \
        jq --argjson list "${dc_thread_list}" '.decode_thread_num=$list' > tmp_0.json
    for (( j=0; j<${hydra_rru_num}; j++ )) do
        server_name=$(cat ${hydra_deploy_fn} | jq --argjson j $j '.rru_servers[$j]')
        unset list_idx
        for (( k=0; k<${hydra_server_num}; k++ )) do
            if [ "${hydra_server_list[$k]}" == $server_name ]; then
                list_idx=$k
            fi
        done
        if [ -z ${list_idx} ]; then
            echored "ERROR: invalid server name (rru_servers[$j])"
            exit
        fi
        cat tmp_$j.json | jq --argjson ip "${hydra_ip_list[${list_idx}]}" --argjson j $j '.bs_rru_addr_list[$j]=$ip' | \
            jq --argjson mac "${hydra_mac_list[${list_idx}]}" --argjson j $j '.bs_rru_mac_list[$j]=$mac' > tmp_$(( $j+1 )).json
    done
    mv tmp_${hydra_rru_num}.json tmp_0.json
    for (( j=0; j<${hydra_num}; j++ )) do
        server_name=$(cat ${hydra_deploy_fn} | jq --argjson j $j '.hydra_servers[$j]')
        unset list_idx
        for (( k=0; k<${hydra_server_num}; k++ )) do
            if [ "${hydra_server_list[$k]}" == ${server_name} ]; then
                list_idx=$k
            fi
        done
        if [ -z ${list_idx} ]; then
            echored "ERROR: invalid server name (hydra_servers[$j]=${server_name})"
            exit
        fi
        cat tmp_$j.json | jq --argjson ip "${hydra_ip_list[${list_idx}]}" --argjson j $j '.bs_server_addr_list[$j]=$ip' | \
            jq --argjson mac "${hydra_mac_list[${list_idx}]}" --argjson j $j '.bs_server_mac_list[$j]=$mac' > tmp_$(( $j+1 )).json
    done
    mv tmp_${hydra_num}.json tmp.json
    rm tmp_*.json
    cat tmp.json | jq --argjson idx $i '.bs_rru_addr_idx=$idx' > tmp_0.json
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.rru_servers[$i]')
    echo "Create config file to RRU ${server_name}"
    pci_addr=$(cat ${hydra_platform_fn} | jq --argjson node ${server_name} '.[$node].pcie' | tr -d '"')
    cat tmp_0.json | jq --arg pci ${pci_addr} '.pci_addr=$pci' > tmp_1.json
    hostname=$(hostname)
    if [ "$hostname" == "$(echo ${server_name} | tr -d '"')" ]; then
        cp tmp_1.json ${ROOT_DIR}/config/run.json
    else
        scp -oStrictHostKeyChecking=no tmp_1.json $(echo ${server_name} | tr -d '"'):~/project/Agora/config/run.json 
    fi
    rm tmp.json tmp_0.json tmp_1.json
done

# Create config for Hydra servers
for (( i=0; i<${hydra_num}; i++ )) do
    cat ${hydra_template_fn} | jq --argjson list "${sc_num_list}" '.subcarrier_num_list=$list' | \
        jq --argjson list "${sc_block_list}" '.subcarrier_block_list=$list' | \
        jq --argjson list "${dc_thread_list}" '.decode_thread_num=$list' > tmp_0.json
    for (( j=0; j<${hydra_rru_num}; j++ )) do
        server_name=$(cat ${hydra_deploy_fn} | jq --argjson j $j '.rru_servers[$j]')
        unset list_idx
        for (( k=0; k<${hydra_server_num}; k++ )) do
            if [ "${hydra_server_list[$k]}" == $server_name ]; then
                list_idx=$k
            fi
        done
        if [ -z ${list_idx} ]; then
            echored "ERROR: invalid server name (rru_servers[$j])"
            exit
        fi
        cat tmp_$j.json | jq --argjson ip "${hydra_ip_list[${list_idx}]}" --argjson j $j '.bs_rru_addr_list[$j]=$ip' | \
            jq --argjson mac "${hydra_mac_list[${list_idx}]}" --argjson j $j '.bs_rru_mac_list[$j]=$mac' > tmp_$(( $j+1 )).json
    done
    mv tmp_${hydra_rru_num}.json tmp_0.json
    for (( j=0; j<${hydra_num}; j++ )) do
        server_name=$(cat ${hydra_deploy_fn} | jq --argjson j $j '.hydra_servers[$j]')
        unset list_idx
        for (( k=0; k<${hydra_server_num}; k++ )) do
            if [ "${hydra_server_list[$k]}" == ${server_name} ]; then
                list_idx=$k
            fi
        done
        if [ -z ${list_idx} ]; then
            echored "ERROR: invalid server name (hydra_servers[$j]=${server_name})"
            exit
        fi
        cat tmp_$j.json | jq --argjson ip "${hydra_ip_list[${list_idx}]}" --argjson j $j '.bs_server_addr_list[$j]=$ip' | \
            jq --argjson mac "${hydra_mac_list[${list_idx}]}" --argjson j $j '.bs_server_mac_list[$j]=$mac' > tmp_$(( $j+1 )).json
    done
    mv tmp_${hydra_num}.json tmp.json
    rm tmp_*.json
    cat tmp.json | jq --argjson idx $i '.bs_server_addr_idx=$idx' > tmp_0.json
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.hydra_servers[$i]')
    echo "Create config file to hydra server ${server_name}"
    pci_addr=$(cat ${hydra_platform_fn} | jq --argjson node ${server_name} '.[$node].pcie' | tr -d '"')
    cat tmp_0.json | jq --arg pci ${pci_addr} '.pci_addr=$pci' > tmp_1.json
    hostname=$(hostname)
    if [ "$hostname" == "$(echo ${server_name} | tr -d '"')" ]; then
        cp tmp_1.json ${ROOT_DIR}/config/run.json
    else
        scp -oStrictHostKeyChecking=no tmp_1.json $(echo ${server_name} | tr -d '"'):~/project/Agora/config/run.json 
    fi
    rm tmp.json tmp_0.json tmp_1.json
done

mkdir -p /tmp/Hydra

# Prepare for the data
server_name=$(cat ${hydra_deploy_fn} | jq '.rru_servers[0]' | tr -d '"')
echocyan "Run control and config generator on ${server_name}"
hostname=$(hostname)
if [ "$hostname" == "${server_name}" ]; then
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

# Run all Hydra servers (TODO: complete)
for (( i=0; i<${hydra_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    echo "Run hydra server ${server_name}"
    hostname=$(hostname)
    if [ "$hostname" == "${server_name}" ]; then
        sudo -E env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 \
            ./build/agora --conf_file config/run.json &
    else
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue_template.bin ${server_name}:~/project/Agora/data/
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue.bin ${server_name}:~/project/Agora/data/
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${server_name}:~/project/Agora/data/
        ssh -oStrictHostKeyChecking=no ${server_name} "source ~/.bash_profile;\
            cd ~/project/Agora; \
            sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/agora --conf_file config/run.json" > /tmp/Hydra/log_${server_name}.txt &
    fi
done

# Give Hydra servers 5 seconds to initialize
sleep 5

# Run all RRU servers
for (( i=1; i<${hydra_rru_num}; i++ )) do
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    echocyan "Run RRU server ${server_name}"
    hostname=$(hostname)
    if [ "$hostname" == "${server_name}" ]; then
        sudo env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 --conf_file=${ROOT_DIR}/config/run.json \
            --frame_duration=$slot_us --core_offset=0 > /tmp/Hydra/log_${server_name}.txt &
    else
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue_template.bin ${server_name}:~/project/Agora/data/
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue.bin ${server_name}:~/project/Agora/data/
        scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${server_name}:~/project/Agora/data/
        ssh -oStrictHostKeyChecking=no ${server_name} "source ~/.bash_profile; \
            cd ~/project/Agora; \
            sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 \
                --conf_file=./config/run.json --frame_duration=$slot_us --core_offset=0" > /tmp/Hydra/log_${server_name}.txt &
    fi
done

server_name=$(cat ${hydra_deploy_fn} | jq '.rru_servers[0]' | tr -d '"')
echocyan "Run RRU server ${server_name}"
hostname=$(hostname)
if [ "$hostname" == "${server_name}" ]; then
    sudo env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 --conf_file=${ROOT_DIR}/config/run.json \
        --frame_duration=$slot_us --core_offset=0 > /tmp/Hydra/log_${server_name}.txt &
else
    scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue_template.bin ${server_name}:~/project/Agora/data/
    scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/control_ue.bin ${server_name}:~/project/Agora/data/
    scp -oStrictHostKeyChecking=no ${ROOT_DIR}/data/LDPC_rx_data_2048_ant${num_antennas}.bin ${server_name}:~/project/Agora/data/
    ssh -oStrictHostKeyChecking=no ${server_name} "source ~/.bash_profile; \
        cd ~/project/Agora; \
        sudo -E env LD_LIBRARY_PATH=\$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 \
            --conf_file=./config/run.json --frame_duration=$slot_us --core_offset=0" > /tmp/Hydra/log_${server_name}.txt &
fi

wait