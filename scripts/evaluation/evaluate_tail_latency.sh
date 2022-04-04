#! /bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

# ant_num=$1
# ue_num=$2
# cat ${hydra_root_dir}/config/config.json | jq --arg ant ${ant_num} --arg ue ${ue_num} \
#     '.hydra_system_config_json="config/template_cloudlab/template_cloudlab_hydra_$ant_$ue_ul.json"' > tmp_0.json
# cat tmp_0.json | jq --arg ant ${ant_num} --arg ue ${ue_num} \
#     '.hydra_server_deploy_json="onfig/deploy_cloudlab/deploy_cloudlab_hydra_$ant_$ue_ul.json"' > ${hydra_root_dir}/config/config.json

target_line_num=200000
# latency_file=${hydra_root_dir}/data/useful_latency_data.txt
# line_num=$(cat ${latency_file} | wc -l )
line_num=0
run_time=0
while [ ${line_num} -lt ${target_line_num} ]; do
    echo "Run Hydra for the ${run_time}-th time"
    ${hydra_root_dir}/scripts/control/run_all.sh -x || continue
    if [ "$?" == "0" ]; then
        ${hydra_root_dir}/scripts/evaluation/latency_analysis.sh ${run_time}
        if [ ! -f ${hydra_root_dir}/data/frame_latency_all_${run_time}.txt ]; then
            echo "Run Hydra failed this time"
            continue
        fi
    fi
    # line_num=$(cat ${latency_file} | wc -l )
    new_line_num=$(cat ${hydra_root_dir}/data/frame_latency_all_${run_time}.txt | wc -l )
    line_num=$(( ${line_num}+${new_line_num} ))
    echo "We got ${line_num} lines to use"
    run_time=$(( ${run_time}+1 ))
done

