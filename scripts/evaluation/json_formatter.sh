#! /bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

files=$(ls ${hydra_root_dir}/config/deploy_msr)
for f in ${files}; do
    jq . ${hydra_root_dir}/config/deploy_msr/${f} > tmp.json
    mv tmp.json ${hydra_root_dir}/config/deploy_msr/${f}
done