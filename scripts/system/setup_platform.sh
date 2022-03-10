#! /bin/bash

set -e

script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

HYDRA_SERVER_LIST_JSON=${hydra_root_dir}/config/platform.json

node_list_str=$(cat ${hydra_root_dir}/scripts/system/node_list)
addr_list_str=$(cat ${hydra_root_dir}/scripts/system/addr_list)

node_id=0
for node in ${node_list_str}
do
    node_list[${node_id}]=$node
    node_id=$(( node_id+1 ))
done
node_id=0
for addr in ${addr_list_str}
do
    addr_list[${node_id}]=$addr
    node_id=$(( node_id+1 ))
done

echo '{}' > ${HYDRA_SERVER_LIST_JSON}

node_id=1
for node in ${node_list_str}
do
    cat ${HYDRA_SERVER_LIST_JSON} | jq --arg node ${node} --arg ip \
        "10.10.1.${node_id}" '. + {($node): {"pcie": "0000:03:00.1", "ip": $ip, "mac": "ff:ff:ff:ff:ff:ff"}}' \
        > tmp_platform
    mv tmp_platform ${HYDRA_SERVER_LIST_JSON}
    node_id=$(( node_id+1 ))
done 

mac_list_str=$(${hydra_root_dir}/scripts/system/query_mac.sh ens1f1)
node_id=0
for mac in ${mac_list_str}
do
    mac_list[${node_id}]=${mac}
    node_id=$(( node_id+1 ))
done

node_id=0
for node in ${node_list_str}
do
    cat ${HYDRA_SERVER_LIST_JSON} | jq --arg node ${node} --arg mac "${mac_list[${node_id}]}" \
        '.[$node].mac |= $mac' \
        > tmp_platform
    mv tmp_platform ${HYDRA_SERVER_LIST_JSON}
    node_id=$(( node_id+1 ))
done 