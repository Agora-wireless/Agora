#! /bin/bash

check_done=0
while [ "${check_done}" == "0" ]
do 
  is_ready=0
  for (( i=0; i<${hydra_app_num}; i++ )) do
    server_name=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.hydra_servers[$i]' | tr -d '"')
    ret=$(cat /tmp/hydra/log_${server_name}.txt | grep "Master thread core") || :
    if [ -n "${ret}" ]; then
      is_ready=$(( is_ready+1 ))
    fi
  done
  if [ "${is_ready}" == "${hydra_app_num}" ]; then
    check_done=1
  fi
  sleep 1
done