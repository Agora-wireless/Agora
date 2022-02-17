#! /bin/bash

is_running=0
for (( i=0; i<${hydra_rru_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.rru_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  if [ "$hostname" == "${server_name}" ]; then
    ret=$(pgrep -x ${hydra_rru_app_name})
    if [ -n "${ret}" ]; then
      echored "${hydra_rru_app_name} still runs on ${server_name}"
      is_running=1
    fi
  else
    ret=$(ssh -oStrictHostKeyChecking=no ${server_name} "pgrep -x ${hydra_rru_app_name}")
    if [ -n "${ret}" ]; then
      echored "${hydra_rru_app_name} still runs on ${server_name}"
      is_running=1
    fi
  fi
done
for (( i=0; i<${hydra_app_num}; i++ )) do
  server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
  hostname=$(hostname)
  if [ "$hostname" == "${server_name}" ]; then
    ret=$(pgrep -x ${hydra_app_name})
    if [ -n "${ret}" ]; then
      echored "${hydra_app_name} still runs on ${server_name}"
      is_running=1
    fi
  else
    ret=$(ssh -oStrictHostKeyChecking=no ${server_name} "pgrep -x ${hydra_app_name}")
    if [ -n "${ret}" ]; then
      echored "${hydra_app_name} still runs on ${server_name}"
      is_running=1
    fi
  fi
done