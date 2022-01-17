#! /bin/bash

# Create config for RRU
sc_num_list=$(cat ${hydra_deploy_fn} | jq '.subcarrier_num_list')
sc_block_list=$(cat ${hydra_deploy_fn} | jq '.subcarrier_block_list')
dc_thread_list=$(cat ${hydra_deploy_fn} | jq '.decode_thread_num')
for (( i=0; i<${hydra_rru_num}; i++ )) do
    cat ${hydra_template_fn} | jq --argjson list "${sc_num_list}" '.subcarrier_num_list=$list' | \
        jq --argjson list "${sc_block_list}" '.subcarrier_block_list=$list' | \
        jq --argjson list "${dc_thread_list}" '.decode_thread_num=$list' > tmp_0.json
    for (( j=0; j<${hydra_rru_num}; j++ )) do
        server_name=$(cat ${hydra_deploy_fn} | jq --argjson j $j '.rru_servers[$j]' | tr -d '"')
        unset list_idx
        for (( k=0; k<${hydra_server_num}; k++ )) do
            if [ "${hydra_server_list[$k]}" == "${server_name}" ]; then
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
        server_name=$(cat ${hydra_deploy_fn} | jq --argjson j $j '.hydra_servers[$j]' | tr -d '"')
        unset list_idx
        for (( k=0; k<${hydra_server_num}; k++ )) do
            if [ "${hydra_server_list[$k]}" == "${server_name}" ]; then
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
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    echo "Create config file to RRU ${server_name}"
    pci_addr=$(cat ${hydra_platform_fn} | jq --arg node "${server_name}" '.[$node].pcie' | tr -d '"')
    cat tmp_0.json | jq --arg pci ${pci_addr} '.pci_addr=$pci' > tmp_1.json
    hostname=$(hostname)
    if [ "${hostname}" == "${server_name}" ]; then
        cp tmp_1.json ${ROOT_DIR}/config/run.json
    else
        scp -oStrictHostKeyChecking=no tmp_1.json ${server_name}:~/project/Agora/config/run.json 
    fi
    rm tmp.json tmp_0.json tmp_1.json
done

# Create config for Hydra servers
for (( i=0; i<${hydra_num}; i++ )) do
    cat ${hydra_template_fn} | jq --argjson list "${sc_num_list}" '.subcarrier_num_list=$list' | \
        jq --argjson list "${sc_block_list}" '.subcarrier_block_list=$list' | \
        jq --argjson list "${dc_thread_list}" '.decode_thread_num=$list' > tmp_0.json
    for (( j=0; j<${hydra_rru_num}; j++ )) do
        server_name=$(cat ${hydra_deploy_fn} | jq --argjson j $j '.rru_servers[$j]' | tr -d '"')
        unset list_idx
        for (( k=0; k<${hydra_server_num}; k++ )) do
            if [ "${hydra_server_list[$k]}" == "${server_name}" ]; then
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
        server_name=$(cat ${hydra_deploy_fn} | jq --argjson j $j '.hydra_servers[$j]' | tr -d '"')
        unset list_idx
        for (( k=0; k<${hydra_server_num}; k++ )) do
            if [ "${hydra_server_list[$k]}" == "${server_name}" ]; then
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
    server_name=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    echo "Create config file to hydra server ${server_name}"
    pci_addr=$(cat ${hydra_platform_fn} | jq --arg node "${server_name}" '.[$node].pcie' | tr -d '"')
    cat tmp_0.json | jq --arg pci ${pci_addr} '.pci_addr=$pci' > tmp_1.json
    hostname=$(hostname)
    if [ "${hostname}" == "${server_name}" ]; then
        cp tmp_1.json ${ROOT_DIR}/config/run.json
    else
        scp -oStrictHostKeyChecking=no tmp_1.json ${server_name}:~/project/Agora/config/run.json 
    fi
    rm tmp.json tmp_0.json tmp_1.json
done