#! /bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

ant_list=(64 64 128 128 150)
ue_list=(16 32 16 32 32)

for i in ${!ant_list[@]}; do
    ant_num=${ant_list[$i]}
    ue_num=${ue_list[$i]}
    for (( c=120; c>=40; c-=20 )) do
        python ${hydra_root_dir}/scripts/evaluation/create_deploy.py ${c} \
            ${hydra_root_dir}/config/deploy_cloudlab/deploy_cloudlab_hydra_${ant_num}_${ue_num}_ul.json \
            ${hydra_root_dir}/config/template_cloudlab/template_cloudlab_hydra_${ant_num}_${ue_num}_ul.json
    done
done