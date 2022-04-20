#! /bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

rm ${hydra_root_dir}/data/frame_latency_all.txt

valid=1
j=0
for (( i=0; i<${hydra_app_num}; i++ )) do
    if [ "${i}" == "1" ]; then
        continue
    fi
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    eval "scp ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/data/frame_latency.txt ${hydra_root_dir}/data/frame_latency_$i.txt"
    num_lines=$(cat ${hydra_root_dir}/data/frame_latency_$j.txt | wc -l )
    if [ "${num_lines}" != "${slots_to_test}" ]; then
        valid=0
    fi
    j=$(( ${j}+1 ))
done

if [ "${valid}" == "1" ]; then
    python ${hydra_root_dir}/scripts/evaluation/merge_latency.py $(( ${hydra_app_num}-1 )) ${hydra_root_dir}/data
    cp ${hydra_root_dir}/data/frame_latency_all.txt ${hydra_root_dir}/data/frame_latency_all_$1.txt
else
    python ${hydra_root_dir}/scripts/evaluation/merge_latency.py $(( ${hydra_app_num}-1 )) ${hydra_root_dir}/data
    cp ${hydra_root_dir}/data/frame_latency_all.txt ${hydra_root_dir}/data/frame_latency_all_error.txt
fi
# cat ${hydra_root_dir}/data/frame_latency_all.txt | awk 'NR > 1000 { print $6 }' >> ${hydra_root_dir}/data/useful_latency_data.txt 