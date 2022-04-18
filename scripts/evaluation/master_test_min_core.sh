#! /bin/bash
set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

ant_list=(64 64 64 128 128 128 128 240 240 240)
ue_list=(24 40 48 8 24 40 48 8 16 24)

for i in ${!ant_list[@]}; do
    ant_num=${ant_list[$i]}
    ue_num=${ue_list[$i]}
    HYDRA_SYSTEM_CONFIG_JSON=config/template_msr/template_msr_${ant_num}_${ue_num}.json
    HYDRA_SERVER_DEPLOY_JSON=config/deploy_msrn/deploy_msr_${ant_num}_${ue_num}.json
    cat ${hydra_master_config_json} | jq --arg template ${HYDRA_SYSTEM_CONFIG_JSON} '.hydra_system_config_json=$template' | \
        jq --arg deploy ${HYDRA_SERVER_DEPLOY_JSON} '.hydra_server_deploy_json=$deploy' > tmp.json
    ${hydra_root_dir}/scripts/evaluation/test_min_core.sh
done