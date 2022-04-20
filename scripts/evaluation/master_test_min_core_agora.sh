#! /bin/bash
set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

ant_list=(64 64 128 128)
ue_list=(16 32 16 32)

rm -f result.txt

for i in ${!ant_list[@]}; do
    ant_num=${ant_list[$i]}
    ue_num=${ue_list[$i]}
    echo "Test ant ${ant_num} ue ${ue_num}"
    HYDRA_SYSTEM_CONFIG_JSON=config/template_msr/template_msr_${ant_num}_${ue_num}_agora.json
    HYDRA_SERVER_DEPLOY_JSON=config/deploy_msr/deploy_msr_${ant_num}_${ue_num}_agora.json
    cat ${hydra_master_config_json} | jq --arg template ${HYDRA_SYSTEM_CONFIG_JSON} '.hydra_system_config_json=$template' | \
        jq --arg deploy ${HYDRA_SERVER_DEPLOY_JSON} '.hydra_server_deploy_json=$deploy' > tmp.json
    mv tmp.json ${hydra_master_config_json}
    ${hydra_root_dir}/scripts/evaluation/test_min_core_agora.sh
done