#! /bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

ant_list=(150)
ue_list=(32)

# for i in ${!ant_list[@]}; do
#     ant_num=${ant_list[$i]}
#     ue_num=${ue_list[$i]}
#     for (( c=120; c>=40; c-=20 )) do
#         echo "Run tail latency evaluation for ant ${ant_num} ue ${ue_num} core ${c}"
#         HYDRA_SERVER_DEPLOY_JSON=config/deploy_cloudlab/deploy_cloudlab_hydra_${ant_num}_${ue_num}_dl_${c}c.json
#         HYDRA_SYSTEM_CONFIG_JSON=config/template_cloudlab/template_cloudlab_hydra_${ant_num}_${ue_num}_dl_${c}c.json
#         cat ${hydra_master_config_json} | jq --arg deploy "${HYDRA_SERVER_DEPLOY_JSON}" '.hydra_server_deploy_json=$deploy' > tmp.json
#         cat tmp.json | jq --arg template "${HYDRA_SYSTEM_CONFIG_JSON}" '.hydra_system_config_json=$template' > ${hydra_master_config_json}
#         rm tmp.json
#         mkdir -p ${hydra_root_dir}/latency/cloudlab_hydra_${ant_num}_${ue_num}_dl_${c}c 
#         ${hydra_root_dir}/scripts/evaluation/evaluate_tail_latency.sh
#         mv ${hydra_root_dir}/data/frame_latency_all_*.txt ${hydra_root_dir}/latency/cloudlab_hydra_${ant_num}_${ue_num}_dl_${c}c/
#     done
# done

# for i in ${!ant_list[@]}; do
#     ant_num=${ant_list[$i]}
#     ue_num=${ue_list[$i]}
#     echo "Run tail latency evaluation for ant ${ant_num} ue ${ue_num} uplink"
#     HYDRA_SERVER_DEPLOY_JSON=config/deploy_cloudlab/deploy_cloudlab_hydra_${ant_num}_${ue_num}_ul.json
#     HYDRA_SYSTEM_CONFIG_JSON=config/template_cloudlab/template_cloudlab_hydra_${ant_num}_${ue_num}_ul.json
#     cat ${hydra_master_config_json} | jq --arg deploy "${HYDRA_SERVER_DEPLOY_JSON}" '.hydra_server_deploy_json=$deploy' > tmp.json
#     cat tmp.json | jq --arg template "${HYDRA_SYSTEM_CONFIG_JSON}" '.hydra_system_config_json=$template' > ${hydra_master_config_json}
#     rm tmp.json
#     mkdir -p ${hydra_root_dir}/latency/cloudlab_hydra_${ant_num}_${ue_num}_ul
#     ${hydra_root_dir}/scripts/evaluation/evaluate_tail_latency.sh
#     mv ${hydra_root_dir}/data/frame_latency_all_*.txt ${hydra_root_dir}/latency/cloudlab_hydra_${ant_num}_${ue_num}_ul/
# done

# for i in ${!ant_list[@]}; do
#     ant_num=${ant_list[$i]}
#     ue_num=${ue_list[$i]}
#     echo "Run tail latency evaluation for ant ${ant_num} ue ${ue_num} downlink"
#     HYDRA_SERVER_DEPLOY_JSON=config/deploy_cloudlab/deploy_cloudlab_hydra_${ant_num}_${ue_num}_dl.json
#     HYDRA_SYSTEM_CONFIG_JSON=config/template_cloudlab/template_cloudlab_hydra_${ant_num}_${ue_num}_dl.json
#     cat ${hydra_master_config_json} | jq --arg deploy "${HYDRA_SERVER_DEPLOY_JSON}" '.hydra_server_deploy_json=$deploy' > tmp.json
#     cat tmp.json | jq --arg template "${HYDRA_SYSTEM_CONFIG_JSON}" '.hydra_system_config_json=$template' > ${hydra_master_config_json}
#     rm tmp.json
#     mkdir -p ${hydra_root_dir}/latency/cloudlab_hydra_${ant_num}_${ue_num}_dl
#     ${hydra_root_dir}/scripts/evaluation/evaluate_tail_latency.sh
#     mv ${hydra_root_dir}/data/frame_latency_all_*.txt ${hydra_root_dir}/latency/cloudlab_hydra_${ant_num}_${ue_num}_dl/
# done

# for i in ${!ant_list[@]}; do
#     ant_num=${ant_list[$i]}
#     ue_num=${ue_list[$i]}
#     for (( c=80; c>=20; c-=10 )) do
#         echo "Run tail latency evaluation for ant ${ant_num} ue ${ue_num} core ${c}"
#         HYDRA_SERVER_DEPLOY_JSON=config/deploy_msr/deploy_msr_${ant_num}_${ue_num}_${c}c.json
#         HYDRA_SYSTEM_CONFIG_JSON=config/template_msr/template_msr_${ant_num}_${ue_num}_${c}c.json
#         cat ${hydra_master_config_json} | jq --arg deploy "${HYDRA_SERVER_DEPLOY_JSON}" '.hydra_server_deploy_json=$deploy' > tmp.json
#         cat tmp.json | jq --arg template "${HYDRA_SYSTEM_CONFIG_JSON}" '.hydra_system_config_json=$template' > ${hydra_master_config_json}
#         rm tmp.json
#         mkdir -p ${hydra_root_dir}/latency/msr_hydra_${ant_num}_${ue_num}_ul_${c}c 
#         ${hydra_root_dir}/scripts/evaluation/evaluate_tail_latency.sh
#         mv ${hydra_root_dir}/data/frame_latency_all_*.txt ${hydra_root_dir}/latency/msr_hydra_${ant_num}_${ue_num}_ul_${c}c/
#     done
# done

# for i in ${!ant_list[@]}; do
#     ant_num=${ant_list[$i]}
#     ue_num=${ue_list[$i]}
#     for (( c=80; c>=20; c-=10 )) do
#         echo "Run tail latency evaluation for ant ${ant_num} ue ${ue_num} core ${c}"
#         HYDRA_SERVER_DEPLOY_JSON=config/deploy_msr_dl/deploy_msr_${ant_num}_${ue_num}_${c}c.json
#         HYDRA_SYSTEM_CONFIG_JSON=config/template_msr_dl/template_msr_${ant_num}_${ue_num}_${c}c.json
#         cat ${hydra_master_config_json} | jq --arg deploy "${HYDRA_SERVER_DEPLOY_JSON}" '.hydra_server_deploy_json=$deploy' > tmp.json
#         cat tmp.json | jq --arg template "${HYDRA_SYSTEM_CONFIG_JSON}" '.hydra_system_config_json=$template' > ${hydra_master_config_json}
#         rm tmp.json
#         mkdir -p ${hydra_root_dir}/latency/msr_hydra_${ant_num}_${ue_num}_dl_${c}c 
#         ${hydra_root_dir}/scripts/evaluation/evaluate_tail_latency.sh
#         mv ${hydra_root_dir}/data/frame_latency_all_*.txt ${hydra_root_dir}/latency/msr_hydra_${ant_num}_${ue_num}_dl_${c}c/
#     done
# done

for i in ${!ant_list[@]}; do
    ant_num=${ant_list[$i]}
    ue_num=${ue_list[$i]}
    echo "Run tail latency evaluation for ant ${ant_num} ue ${ue_num} core ${c}"
    HYDRA_SERVER_DEPLOY_JSON=config/deploy_msr_dl/deploy_msr_${ant_num}_${ue_num}.json
    HYDRA_SYSTEM_CONFIG_JSON=config/template_msr_dl/template_msr_${ant_num}_${ue_num}.json
    cat ${hydra_master_config_json} | jq --arg deploy "${HYDRA_SERVER_DEPLOY_JSON}" '.hydra_server_deploy_json=$deploy' > tmp.json
    cat tmp.json | jq --arg template "${HYDRA_SYSTEM_CONFIG_JSON}" '.hydra_system_config_json=$template' > ${hydra_master_config_json}
    rm tmp.json
    mkdir -p ${hydra_root_dir}/latency/msr_hydra_${ant_num}_${ue_num}_dl 
    ${hydra_root_dir}/scripts/evaluation/evaluate_tail_latency.sh
    mv ${hydra_root_dir}/data/frame_latency_all_*.txt ${hydra_root_dir}/latency/msr_hydra_${ant_num}_${ue_num}_dl/
done