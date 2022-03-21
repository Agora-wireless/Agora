#! /bin/bash

set -e

if [ "${bigstation_mode}" == "true" ]; then
  num_fft_workers=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.num_fft_workers')
  num_zf_workers=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.num_zf_workers')
  num_demul_workers=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.num_demul_workers')
  num_decode_workers=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.num_decode_workers')
  for (( i=0; i<${hydra_rru_num}; i++ )) do
    cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson list "${num_fft_workers}" '.num_fft_workers=$list' | \
      jq --argjson list "${num_zf_workers}" '.num_zf_workers=$list' | \
      jq --argjson list "${num_demul_workers}" '.num_demul_workers=$list' | \
      jq --argjson list "${num_decode_workers}" '.num_decode_workers=$list' > tmp_0.json
    for (( j=0; j<${hydra_rru_num}; j++ )) do
      server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson j $j '.rru_servers[$j]' | tr -d '"')
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
    for (( j=0; j<${hydra_app_num}; j++ )) do
      server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson j $j '.hydra_servers[$j]' | tr -d '"')
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
    mv tmp_${hydra_app_num}.json tmp.json
    rm tmp_*.json
    cat tmp.json | jq --argjson idx $i '.bs_rru_addr_idx=$idx' > tmp_0.json
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    echo "Create config file to RRU ${server_name}"
    pci_addr=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --arg node "${server_name}" '.[$node].pcie' | tr -d '"')
    cat tmp_0.json | jq --arg pci ${pci_addr} '.pci_addr=$pci' > tmp_1.json
    hostname=$(hostname)
    eval "scp -oStrictHostKeyChecking=no tmp_1.json ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/config/run.json" > /dev/null 2>&1
    rm tmp.json tmp_0.json tmp_1.json
  done

  # Create config for Hydra servers
  for (( i=0; i<${hydra_app_num}; i++ )) do
    cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson list "${num_fft_workers}" '.num_fft_workers=$list' | \
      jq --argjson list "${num_zf_workers}" '.num_zf_workers=$list' | \
      jq --argjson list "${num_demul_workers}" '.num_demul_workers=$list' | \
      jq --argjson list "${num_decode_workers}" '.num_decode_workers=$list' > tmp_0.json
    for (( j=0; j<${hydra_rru_num}; j++ )) do
      server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson j $j '.rru_servers[$j]' | tr -d '"')
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
    for (( j=0; j<${hydra_app_num}; j++ )) do
      server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson j $j '.hydra_servers[$j]' | tr -d '"')
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
    mv tmp_${hydra_app_num}.json tmp.json
    rm tmp_*.json
    cat tmp.json | jq --argjson idx $i '.bs_server_addr_idx=$idx' > tmp_0.json
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    echo "Create config file to hydra server ${server_name}"
    pci_addr=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --arg node "${server_name}" '.[$node].pcie' | tr -d '"')
    cat tmp_0.json | jq --arg pci ${pci_addr} '.pci_addr=$pci' > tmp_1.json
    hostname=$(hostname)
    eval "scp -oStrictHostKeyChecking=no tmp_1.json ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/config/run.json" > /dev/null 2>&1
    rm tmp.json tmp_0.json tmp_1.json
  done
else
  # Create config for servers running RRU traffic generator
  sc_num_list=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.subcarrier_num_list')
  sc_block_list=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.subcarrier_block_list')
  cd_thread_list=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.coding_thread_num')
  for (( i=0; i<${hydra_rru_num}; i++ )) do
    cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson list "${sc_num_list}" '.subcarrier_num_list=$list' | \
      jq --argjson list "${sc_block_list}" '.subcarrier_block_list=$list' | \
      jq --argjson list "${cd_thread_list}" '.coding_thread_num=$list' > tmp_0.json
    for (( j=0; j<${hydra_rru_num}; j++ )) do
      server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson j $j '.rru_servers[$j]' | tr -d '"')
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
    for (( j=0; j<${hydra_app_num}; j++ )) do
      server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson j $j '.hydra_servers[$j]' | tr -d '"')
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
    mv tmp_${hydra_app_num}.json tmp.json
    rm tmp_*.json
    cat tmp.json | jq --argjson idx $i '.bs_rru_addr_idx=$idx' > tmp_0.json
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    echo "Create config file to RRU ${server_name}"
    pci_addr=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --arg node "${server_name}" '.[$node].pcie' | tr -d '"')
    cat tmp_0.json | jq --arg pci ${pci_addr} '.pci_addr=$pci' > tmp_1.json
    hostname=$(hostname)
    eval "scp -oStrictHostKeyChecking=no tmp_1.json ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/config/run.json" > /dev/null 2>&1
    rm tmp.json tmp_0.json tmp_1.json
  done

  # Create config for Hydra servers
  for (( i=0; i<${hydra_app_num}; i++ )) do
    cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson list "${sc_num_list}" '.subcarrier_num_list=$list' | \
      jq --argjson list "${sc_block_list}" '.subcarrier_block_list=$list' | \
      jq --argjson list "${cd_thread_list}" '.coding_thread_num=$list' > tmp_0.json
    for (( j=0; j<${hydra_rru_num}; j++ )) do
      server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson j $j '.rru_servers[$j]' | tr -d '"')
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
    for (( j=0; j<${hydra_app_num}; j++ )) do
      server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson j $j '.hydra_servers[$j]' | tr -d '"')
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
    mv tmp_${hydra_app_num}.json tmp.json
    rm tmp_*.json
    cat tmp.json | jq --argjson idx $i '.bs_server_addr_idx=$idx' > tmp_0.json
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    echo "Create config file to hydra server ${server_name}"
    pci_addr=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --arg node "${server_name}" '.[$node].pcie' | tr -d '"')
    cat tmp_0.json | jq --arg pci ${pci_addr} '.pci_addr=$pci' > tmp_1.json
    hostname=$(hostname)
    eval "scp -oStrictHostKeyChecking=no tmp_1.json ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/config/run.json" > /dev/null 2>&1
    rm tmp.json tmp_0.json tmp_1.json
  done
fi