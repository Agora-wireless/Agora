#! /bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

for (( i=2; i<=5; i++ )) do
    # ${hydra_root_dir}/scripts/evaluation/master_test_min_core_bigstation.sh ${i}
    ${hydra_root_dir}/scripts/evaluation/master_test_min_core_bigstation_dl.sh ${i}
done