#! /bin/bash

if [ -z "${ROOT_DIR}" ]; then
    echo "ROOT_DIR variable not set"
    exit
fi
if [ -z "${hydra_deploy_fn}" ]; then
    echo "hydra_deploy_fn variable not set"
    exit
fi
if [ -z "${hydra_num}" ]; then
    echo "hydra_num variable not set"
    exit
fi
if [ -z "${hydra_template_fn}" ]; then
    echo "hydra_template_fn variable not set"
    exit
fi
if [ -z "${hydra_rru_num}" ]; then
    echo "hydra_rru_num variable not set"
    exit
fi

# Check list length
sc_num_list_len=$(cat ${hydra_deploy_fn} | jq '.subcarrier_num_list | length')
sc_block_list_len=$(cat ${hydra_deploy_fn} | jq '.subcarrier_block_list | length')
dc_thread_list_len=$(cat ${hydra_deploy_fn} | jq '.decode_thread_num | length')
if [ "$hydra_num" -ne "$sc_num_list_len" ]; then
    echored "ERROR: hydra_servers != subcarrier_num_list"
    exit
fi
if [ "$hydra_num" -ne "$sc_block_list_len" ]; then
    echored "ERROR: hydra_servers != subcarrier_block_list"
    exit
fi
if [ "$hydra_num" -ne "$dc_thread_list_len" ]; then
    echored "ERROR: hydra_servers != decode_thread_num"
    exit
fi

# Check ofdm_data_num
sc_num=$(cat ${hydra_template_fn} | jq '.ofdm_data_num')
ue_num=$(cat ${hydra_template_fn} | jq '.ue_num')
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
sc_num_check=0
for (( i=0; i<${hydra_num}; i++ )) do
    sc_num_cur=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.subcarrier_num_list[$i]')
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
for (( i=0; i<${hydra_num}; i++ )) do
    sc_block_cur=$(cat ${hydra_deploy_fn} | jq --argjson i $i '.subcarrier_block_list[$i]')
    sc_ue_mol=$(( ${sc_block_cur}%8 ))
    if [ "${sc_ue_mol}" -ne "0" ]; then
        echored "ERROR: subcarrier_block_list[$i] % 8 != 0"
        exit
    fi
    if [ "${sc_block_cur}" == "0" ]; then
        echored "ERROR: subcarrier_block_list[$i] == 0"
        exit
    fi
done