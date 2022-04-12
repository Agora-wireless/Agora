#! /bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

ant_list=(64 64 64 128 128 150)
ue_list=(8 16 32 16 32 32)

# for i in ${!ant_list[@]}; do
#     ant_num=${ant_list[$i]}
#     ue_num=${ue_list[$i]}
#     for (( c=60; c>=20; c-=10 )) do
#         echo "test min slot size for ant ${ant_num} ue ${ue_num} core ${c}"
#         HYDRA_SERVER_DEPLOY_JSON=config/deploy_cloudlab/deploy_cloudlab_hydra_${ant_num}_${ue_num}_dl_${c}c.json
#         HYDRA_SYSTEM_CONFIG_JSON=config/template_cloudlab/template_cloudlab_hydra_${ant_num}_${ue_num}_dl_${c}c.json
#         cat ${hydra_master_config_json} | jq --arg deploy "${HYDRA_SERVER_DEPLOY_JSON}" '.hydra_server_deploy_json=$deploy' > tmp.json
#         cat tmp.json | jq --arg template "${HYDRA_SYSTEM_CONFIG_JSON}" '.hydra_system_config_json=$template' > ${hydra_master_config_json}
#         bash ${hydra_root_dir}/scripts/evaluation/test_min_slot.sh
#     done
# done

for i in ${!ant_list[@]}; do
    ant_num=${ant_list[$i]}
    ue_num=${ue_list[$i]}
    for (( c=80; c>=20; c-=10 )) do
        echo "test min slot size for ant ${ant_num} ue ${ue_num} core ${c}"
        HYDRA_SERVER_DEPLOY_JSON=config/deploy_msr/deploy_msr_${ant_num}_${ue_num}_${c}c.json
        HYDRA_SYSTEM_CONFIG_JSON=config/template_msr/template_msr_${ant_num}_${ue_num}_${c}c.json
        cat ${hydra_master_config_json} | jq --arg deploy "${HYDRA_SERVER_DEPLOY_JSON}" '.hydra_server_deploy_json=$deploy' > tmp.json
        cat tmp.json | jq --arg template "${HYDRA_SYSTEM_CONFIG_JSON}" '.hydra_system_config_json=$template' > ${hydra_master_config_json}
        bash ${hydra_root_dir}/scripts/evaluation/test_min_slot.sh ${ant_num} ${ue_num} ${c}
    done
done