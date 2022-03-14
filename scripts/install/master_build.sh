#! /bin/bash

# Run this script on your laptop
# This script helps you to run install_hydra.sh on all remote servers
#   listed on $hydra_platform_fn

set -e

# Find the root directory of Hydra app
script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

HYDRA_SERVER_DEPLOY_JSON=${hydra_root_dir}/config/deploy.json
HYDRA_RUNNER_ROOT="~/HydraRemoteRunner"

BUILD_MODE="local"

while getopts "h?:r" opt; do
  case "$opt" in
    h|\?)
      echo "Help"
      echo -e "\t-h\tShow this infomation"
      echo -e "\t-r\tBuild Hydra app on remote servers"
      exit 0
      ;;
    r)
      BUILD_MODE="remote"
      ;;
  esac
done

checkpkg jq
if [ ${checkpkg_res} == "0" ]; then
  exit
fi

# Read HYDRA_SERVER_LIST_JSON and HYDRA_RUNNER_ROOT from file config/config.json
hydra_master_config_json=${hydra_root_dir}/config/config.json
if [ ! -f ${hydra_master_config_json} ]; then
  echored "ERROR: config file ${hydra_master_config_json} does not exist"
fi
echo "Parsing 1"
res=$(cat ${hydra_master_config_json} | jq '.hydra_server_deploy_json' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_SERVER_DEPLOY_JSON=${hydra_root_dir}/${res}
fi
echo "Parsing 2"
res=$(cat ${hydra_master_config_json} | jq '.hydra_runner_root' | tr -d '"')
if [ "${res}" != "null" ]; then
  HYDRA_RUNNER_ROOT=${res}
fi

echo "Parsing 3"
rru_server_num=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.rru_servers | length')
echo "Parsing 4"
hydra_server_num=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.hydra_servers | length')

mkdir -p /tmp/hydra

if [ "${BUILD_MODE}" == "remote" ]; then
  echocyan "Sync Hydra source codes to remote servers"
  source ${hydra_root_dir}/scripts/control/sync_all.sh
  echocyan "Build Hydra app on remote servers"
  for (( i=0; i<${rru_server_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    ssh -oStrictHostKeyChecking=no ${server_name} "cd ${HYDRA_RUNNER_ROOT}/Agora; \
      ./scripts/install/install_hydra.sh" > /tmp/hydra/install_${server_name}.log || \
      echored "Server ${server_name} failed to build hydra. Please check /tmp/hydra/install_${server_name}.log for details" &
  done
  for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    ssh -oStrictHostKeyChecking=no ${server_name} "cd ${HYDRA_RUNNER_ROOT}/Agora; \
      ./scripts/install/install_hydra.sh" > /tmp/hydra/install_${server_name}.log || \
      echored "Server ${server_name} failed to build hydra. Please check /tmp/hydra/install_${server_name}.log for details" &
  done
else
  echocyan "Build Hydra app on the local server"
  rm -rf ${hydra_root_dir}/build || :
  mkdir -p ${hydra_root_dir}/build
  cd ${hydra_root_dir}/build
  echocyan "Building Hydra app..."
  cmake .. -DLOG_LEVEL=warn > /tmp/hydra/install.log || \
    { echored "Failed to build hydra. Please check /tmp/hydra/install.log for details" && exit 1; }
  make -j >> /tmp/hydra/install.log || \
    { echored "Failed to build hydra. Please check /tmp/hydra/install.log for details" && exit 1; }
  echocyan "Copying Hydra binaries to ${HYDRA_RUNNER_ROOT} on remote servers"
  for (( i=0; i<${rru_server_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
    scp -r -oStrictHostKeyChecking=no ${hydra_root_dir}/build/* ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/build/ >> /tmp/hydra/install.log || \
      echored "Copying binaries to server ${server_name} failed. Please check /tmp/hydra/install.log for details" &
  done
  for (( i=0; i<${hydra_server_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    scp -r -oStrictHostKeyChecking=no ${hydra_root_dir}/build/* ${server_name}:${HYDRA_RUNNER_ROOT}/Agora/build/ >> /tmp/hydra/install.log || \
      echored "Copying binaries to server ${server_name} failed. Please check /tmp/hydra/install.log for details" &
  done
fi

wait

echocyan "Hydra app is built successfully"