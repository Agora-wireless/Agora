#! /bin/bash

set -e

# Checking hugepage amount on Hydra servers
for (( i=0; i<${hydra_app_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  if [ "${hostname}" == "${server_name}" ]; then
    hugepage_sz=$(cat /proc/meminfo | grep Hugepagesize | awk '{ print $2 }')
    hugepage_num=$(cat /proc/meminfo | grep HugePages_Free | awk '{ print $2 }')
    total_sz=$(( hugepage_sz*hugepage_num ))
    if [ "${total_sz}" -le "8388608" ]; then
        echored "ERROR: Server ${server_name} does not have 8GB hugepage!"
        exit
    fi
  else
    hugepage_sz=$(ssh ${server_name} "cat /proc/meminfo | grep Hugepagesize | awk '{ print \$2 }'")
    hugepage_num=$(ssh ${server_name} "cat /proc/meminfo | grep HugePages_Free | awk '{ print \$2 }'")
    total_sz=$(( hugepage_sz*hugepage_num ))
    if [ "${total_sz}" -le "8388608" ]; then
        echored "ERROR: Server ${server_name} does not have 8GB hugepage!"
        exit
    fi
  fi
done

# Checking hugepage amount on RRU servers
for (( i=0; i<${hydra_rru_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  if [ "${hostname}" == "${server_name}" ]; then
    hugepage_sz=$(cat /proc/meminfo | grep Hugepagesize | awk '{ print $2 }')
    hugepage_num=$(cat /proc/meminfo | grep HugePages_Free | awk '{ print $2 }')
    total_sz=$(( hugepage_sz*hugepage_num ))
    if [ "${total_sz}" -le "8388608" ]; then
        echored "ERROR: Server ${server_name} does not have 8GB hugepage!"
        exit
    fi
  else
    hugepage_sz=$(ssh ${server_name} "cat /proc/meminfo | grep Hugepagesize | awk '{ print \$2 }'")
    hugepage_num=$(ssh ${server_name} "cat /proc/meminfo | grep HugePages_Free | awk '{ print \$2 }'")
    total_sz=$(( hugepage_sz*hugepage_num ))
    if [ "${total_sz}" -le "8388608" ]; then
        echored "ERROR: Server ${server_name} does not have 8GB hugepage!"
        exit
    fi
  fi
done