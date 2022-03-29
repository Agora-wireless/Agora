#! /bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

rm ${hydra_root_dir}/data/frame_latency_all.txt

for (( i=0; i<${hydra_app_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    eval "scp ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/frame_latency.txt ${hydra_root_dir}/data/frame_latency_$i.txt"
done

python ${hydra_root_dir}/scripts/evaluation/merge_latency.py ${hydra_app_num} ${hydra_root_dir}/data
cat ${hydra_root_dir}/data/frame_latency_all.txt | awk 'NR > 200 { print $6 }' >> ${hydra_root_dir}/data/useful_latency_data.txt 