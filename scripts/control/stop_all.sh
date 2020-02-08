#! /bin/bash
# Kill Hydra RRU sender and Hydra worker processes on remote servers

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

# Kill all RRU processes
echocyan "Killing all ${hydra_rru_num} RRU processes"
for (( i=0; i<${hydra_rru_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  echo "Killing ${hydra_rru_app_name} process on ${server_name}"
  if [ "$hostname" == "${server_name}" ]; then
    sudo pkill ${hydra_rru_app_name} 1> /dev/null 2> /dev/null
  else
    ssh -oStrictHostKeyChecking=no ${server_name} "sudo pkill ${hydra_rru_app_name} 1> /dev/null 2> /dev/null" 
  fi
done

# Kill all Hydra processes
echocyan "Killing all ${hydra_app_num} Hydra processes..."
for (( i=0; i<${hydra_app_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  echo "Killing ${hydra_app_name} process on ${server_name}"
  if [ "$hostname" == "${server_name}" ]; then
    sudo pkill ${hydra_app_name} 1> /dev/null 2> /dev/null
  else
    ssh -oStrictHostKeyChecking=no ${server_name} "sudo pkill ${hydra_app_name} 1> /dev/null 2> /dev/null" 
  fi
done

# Check running processes
echocyan "Checking if any Hydra processes are still running"
for (( i=0; i<${hydra_rru_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  echo "Checking on ${server_name}"
  if [ "$hostname" == "${server_name}" ]; then
    ret=$(pgrep -x  ${hydra_rru_app_name})
    if [ -n "${ret}" ]; then
      echored "${hydra_rru_app_name} still runs on ${server_name}"
    fi
  else
    ret=$(ssh -oStrictHostKeyChecking=no ${server_name} "pgrep -x ${hydra_rru_app_name}")
    if [ -n "${ret}" ]; then
      echored "${hydra_rru_app_name} still runs on ${server_name}"
    fi
  fi
done
for (( i=0; i<${hydra_app_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  echo "Checking on ${server_name}"
  if [ "$hostname" == "${server_name}" ]; then
    ret=$(pgrep -x  ${hydra_app_name})
    if [ -n "${ret}" ]; then
      echored "${hydra_app_name} still runs on ${server_name}"
    fi
  else
    ret=$(ssh -oStrictHostKeyChecking=no ${server_name} "pgrep -x ${hydra_app_name}")
    if [ -n "${ret}" ]; then
      echored "${hydra_app_name} still runs on ${server_name}"
    fi
  fi
done