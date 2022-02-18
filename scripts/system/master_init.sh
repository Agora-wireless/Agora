#! /bin/bash

set -e

# Find the root directory of Hydra project
script_dir=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

# The JSON file describing all remote servers to install Hydra and dependencies
# User can specify this parameter using 'hydra_server_list_json' in config/config.json
HYDRA_SERVER_LIST_JSON=${hydra_root_dir}/config/platform.json

ONLY_SINGLE_SERVER="all"

# Parse input arguments
# Run "master_install.sh -h" to check all argument options
while getopts "h?:s:" opt; do
  case "$opt" in
    h|\?)
      echo "Help"
      echo -e "\t-h\tShow this infomation"
      echo -e "\t-s [hostname]\tSpecify the only remote server to install packages and Hydra app"
      exit 0
      ;;
    s)
      ONLY_SINGLE_SERVER="${OPTARG}"
      ;;
  esac
done

mkdir -p /tmp/hydra

# Read HYDRA_SERVER_LIST_JSON and HYDRA_RUNNER_ROOT from file config/config.json
hydra_master_config_json=${hydra_root_dir}/config/config.json
if [ ! -f ${hydra_master_config_json} ]; then
  echored "ERROR: config file ${hydra_master_config_json} does not exist"
fi
res=$(cat ${hydra_master_config_json} | jq '.hydra_server_list_json' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_SERVER_LIST_JSON=${hydra_root_dir}/${res}
fi

if [ "${ONLY_SINGLE_SERVER}" == "all" ]; then
  # Parse the JSON file describing remote servers using jq tool
  hydra_server_num=$(cat ${HYDRA_SERVER_LIST_JSON} | jq '. | length')
  echocyan "Initialize server setup on all remote servers"
  for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
    hydra_tmp_dir[$i]=$(ssh -oStrictHostKeyChecking=no ${server_name} "mktemp -d")
    (rsync -a --exclude '*.bin' --exclude '*.git/*' ${hydra_root_dir} ${server_name}:${hydra_tmp_dir[$i]}/ \
      > /dev/null 2>&1 || (echo "Copying source code to ${server_name} failed"; exit); \
    echo "${server_name} rsync complete") &
  done
  wait
  for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_LIST_JSON} | jq --argjson i $i '. | keys | .[$i]' | tr -d '"')
    (ssh -oStrictHostKeyChecking=no ${server_name} "cd ${hydra_tmp_dir[$i]}/Agora; \
      ./scripts/system/server_init.sh" > /tmp/hydra/init_${server_name}.log 2>&1; \
      echo "${server_name} initialization complete";) &
  done
  wait
else
  server_list=$(echo ${ONLY_SINGLE_SERVER} | tr "," "\n")
  i=0
  for server_name in ${server_list}
  do
    hydra_tmp_dir[$i]=$(ssh -oStrictHostKeyChecking=no ${server_name} "mktemp -d")
    (rsync -a --exclude '*.bin' --exclude '*.git/*' ${hydra_root_dir} ${server_name}:${hydra_tmp_dir[$i]}/ \
      > /dev/null 2>&1 || (echo "Copying source code to ${server_name} failed"; exit); \
    echo "${server_name} rsync complete") &
    i=$((i+1))
  done
  wait
  i=0
  for server_name in ${server_list}
  do
    echocyan "Initialize server setup on ${server_name}"
    (ssh -oStrictHostKeyChecking=no ${server_name} "cd ${hydra_tmp_dir[$i]}/Agora; \
      ./scripts/system/server_init.sh" > /tmp/hydra/init_${server_name}.log 2>&1; \
      echo "${server_name} initialization complete";) &
    i=$((i+1))
  done
  wait
fi