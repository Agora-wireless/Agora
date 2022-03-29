#! /bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

source ${hydra_root_dir}/scripts/evaluation/init_settings.sh

server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.hydra_servers[0]' | tr -d '"')

rm -f ${hydra_root_dir}/data/benchmark_data.txt

for (( i=0; i<${#ant_array[@]}; i++ )) do
  res=$(ssh ${server_name} "cd ${HYDRA_RUNNER_ROOT}/Agora; source scripts/install/setvars.sh > /dev/null 2>&1; \
    ./build/benchmark -a ${ant_array[$i]} -u ${ue_array[$i]}" | tail -6 | awk '{ print $5 }' | cut -c2- )
  echo ${res} >> ${hydra_root_dir}/data/benchmark_data.txt
done
