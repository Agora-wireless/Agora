#! /bin/bash

set -e

# Check necessary env vars exist
if [ -z "${hydra_root_dir}" ]; then
  echo "hydra_root_dir variable not set"
  exit
fi
if [ -z "${HYDRA_SERVER_DEPLOY_JSON}" ]; then
  echo "HYDRA_SERVER_DEPLOY_JSON variable not set"
  exit
fi
if [ -z "${hydra_server_num}" ]; then
  echo "hydra_server_num variable not set"
  exit
fi
if [ -z "${HYDRA_SYSTEM_CONFIG_JSON}" ]; then
  echo "HYDRA_SYSTEM_CONFIG_JSON variable not set"
  exit
fi
if [ -z "${hydra_rru_num}" ]; then
  echo "hydra_rru_num variable not set"
  exit
fi
if [ -z "${hydra_app_num}" ]; then
  echo "hydra_app_num variable not set"
  exit
fi

# Check the length of 
#  * subcarrier_num_list
#  * subcarrier_block_list
#  * coding_thread_num
# are equal 
sc_num_list_len=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.subcarrier_num_list | length')
sc_block_list_len=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.subcarrier_block_list | length')
dc_thread_list_len=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.coding_thread_num | length')
if [ "${hydra_app_num}" -ne "${sc_num_list_len}" ]; then
  echored "ERROR: hydra_servers != subcarrier_num_list"
  exit
fi
if [ "${hydra_app_num}" -ne "${sc_block_list_len}" ]; then
  echored "ERROR: hydra_servers != subcarrier_block_list"
  exit
fi
if [ "${hydra_app_num}" -ne "${dc_thread_list_len}" ]; then
  echored "ERROR: hydra_servers != coding_thread_num"
  exit
fi

# Check ofdm_data_num is a multiple of 8 and ue_num
sc_num=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.ofdm_data_num')
ue_num=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.ue_num')
sc_ue_mol=$(( ${sc_num}%${ue_num} ))
if [ "${sc_ue_mol}" -ne "0" ]; then
  echored "ERROR: ofdm_data_num % ue_num != 0"
  exit
fi
sc_ue_mol=$(( ${sc_num}%8 ))
if [ "${sc_ue_mol}" -ne "0" ]; then
  echored "ERROR: ofdm_data_num % 8 != 0"
  exit
fi

# Check subcarrier_num_list
# * The sum of the list should be equal to ofdm_data_num
# * Each item in this list should be a multiple of ue_num and 8
sc_num_check=0
for (( i=0; i<${hydra_app_num}; i++ )) do
  sc_num_cur=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.subcarrier_num_list[$i]')
  sc_ue_mol=$(( ${sc_num_cur}%${ue_num} ))
  if [ "${sc_ue_mol}" -ne "0" ]; then
    echored "ERROR: subcarrier_num_list[$i] % ue_num != 0"
    exit
  fi
  sc_ue_mol=$(( ${sc_num_cur}%8 ))
  if [ "${sc_ue_mol}" -ne "0" ]; then
    echored "ERROR: subcarrier_num_list[$i] % 8 != 0"
    exit
  fi
  sc_num_check=$(( ${sc_num_check}+${sc_num_cur} ))
done
if [ "${sc_num}" -ne "${sc_num_check}" ]; then
  echored "ERROR: the sum of subcarrier_num_list != ofdm_data_num"
  exit
fi

# Check subcarrier_block_list
# * Each item in this list should be a multiple of 8
# * Each item should be larger than 0
for (( i=0; i<${hydra_app_num}; i++ )) do
  sc_block_cur=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i $i '.subcarrier_block_list[$i]')
  # sc_ue_mol=$(( ${sc_block_cur}%8 ))
  # if [ "${sc_ue_mol}" -ne "0" ]; then
  #   echored "ERROR: subcarrier_block_list[$i] % 8 != 0"
  #   exit
  # fi
  if [ "${sc_block_cur}" == "0" ]; then
    echored "ERROR: subcarrier_block_list[$i] == 0"
    exit
  fi
done