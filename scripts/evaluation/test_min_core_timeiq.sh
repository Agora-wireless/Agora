#! /bin/bash
set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

echo "Checking out ${HYDRA_SERVER_DEPLOY_JSON}"
git checkout ${HYDRA_SERVER_DEPLOY_JSON}

fft_thread=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.fft_thread_num[0]')
sc_block_sz=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.subcarrier_block_list[0]')
coding_thread=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.coding_thread_num[0]')
fft_tx_thread=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.fft_tx_thread_num' )
rx_thread=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.rx_thread_num' )

rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt

${hydra_root_dir}/scripts/control/run_all.sh -x || continue
${hydra_root_dir}/scripts/evaluation/latency_analysis.sh 0
if [ -f ${hydra_root_dir}/data/frame_latency_all_0.txt ]; then
    echo "Succeed this time"
else
    echo "Failed in the beginning"
    exit
fi

cur_fft_thread=$(( ${fft_thread}-2 ))
over=0
while [ "${over}" == "0" ]; do
    rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt
    echo "Run Hydra for fft thread num ${cur_fft_thread}, subcarrier block size ${cur_sc_block_sz}, and coding thread num ${coding_thread}"
    for (( i=0; i<${hydra_app_num}; i++ )) do
        cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i ${i} --argjson num ${cur_fft_thread} '.fft_thread_num[$i]=$num' > tmp.json
        mv tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
    done
    ${hydra_root_dir}/scripts/control/run_all.sh -x || continue
    ${hydra_root_dir}/scripts/evaluation/latency_analysis.sh 0
    if [ -f ${hydra_root_dir}/data/frame_latency_all_0.txt ]; then
        echo "Succeed this time"
        fft_thread=${cur_fft_thread}
        cur_fft_thread=$(( ${cur_fft_thread}-2 ))
        continue
    fi
    over=1
done

for (( i=0; i<${hydra_app_num}; i++ )) do
    cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i ${i} --argjson num ${fft_thread} '.fft_thread_num[$i]=$num' > tmp.json
    mv tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
done

cur_sc_block_sz=$(( ${sc_block_sz}+2 ))
over=0
while [ "${over}" != "2" ]; do
    rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt
    echo "Run Hydra for fft thread num ${cur_fft_thread}, subcarrier block size ${cur_sc_block_sz}, and coding thread num ${coding_thread}"
    for (( i=0; i<${hydra_app_num}; i++ )) do
        cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i ${i} --argjson num ${cur_sc_block_sz} '.subcarrier_block_list[$i]=$num' > tmp.json
        mv tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
    done
    ${hydra_root_dir}/scripts/control/run_all.sh -x || continue
    ${hydra_root_dir}/scripts/evaluation/latency_analysis.sh 0
    if [ -f ${hydra_root_dir}/data/frame_latency_all_0.txt ]; then
        echo "Succeed this time"
        sc_block_sz=${cur_sc_block_sz}
        cur_sc_block_sz=$(( ${sc_block_sz}+2 ))
        continue
    else
        cur_sc_block_sz=$(( ${cur_sc_block_sz}+2 ))
    fi
    over=$(( ${over}+1 ))
done

for (( i=0; i<${hydra_app_num}; i++ )) do
    cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i ${i} --argjson num ${sc_block_sz} '.subcarrier_block_list[$i]=$num' > tmp.json
    mv tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
done

cur_coding_thread=$(( ${coding_thread}-2 ))
over=0
while [ "${over}" == "0" ]; do
    rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt
    echo "Run Hydra for fft thread num ${cur_fft_thread}, subcarrier block size ${cur_sc_block_sz}, and coding thread num ${coding_thread}"
    for (( i=0; i<${hydra_app_num}; i++ )) do
        cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i ${i} --argjson num ${cur_coding_thread} '.coding_thread_num[$i]=$num' > tmp.json
        cp tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
    done
    ${hydra_root_dir}/scripts/control/run_all.sh -x || continue
    ${hydra_root_dir}/scripts/evaluation/latency_analysis.sh 0
    if [ -f ${hydra_root_dir}/data/frame_latency_all_0.txt ]; then
        echo "Succeed this time"
        coding_thread=${cur_coding_thread}
        cur_coding_thread=$(( ${coding_thread}-2 ))
        continue
    fi
    over=1
done

for (( i=0; i<${hydra_app_num}; i++ )) do
    cat ${HYDRA_SERVER_DEPLOY_JSON} | jq --argjson i ${i} --argjson num ${coding_thread} '.coding_thread_num[$i]=$num' > tmp.json
    mv tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
done

cur_fft_tx_thread=$(( ${fft_tx_thread}-1 ))
over=0
while [ "${over}" == "0" ]; do
    rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt
    echo "Run Hydra for fft tx thread ${cur_fft_tx_thread}"
    cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson num ${cur_fft_tx_thread} '.fft_tx_thread_num=$num' > tmp.json
    cp tmp.json ${HYDRA_SYSTEM_CONFIG_JSON}
    ${hydra_root_dir}/scripts/control/run_all.sh -x || continue
    ${hydra_root_dir}/scripts/evaluation/latency_analysis.sh 0
    if [ -f ${hydra_root_dir}/data/frame_latency_all_0.txt ]; then
        echo "Succeed this time"
        fft_tx_thread=${cur_fft_tx_thread}
        cur_fft_tx_thread=$(( ${cur_fft_tx_thread}-1 ))
        continue
    fi
    over=1
done

cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson num ${fft_tx_thread} '.fft_tx_thread_num=$num' > tmp.json
cp tmp.json ${HYDRA_SYSTEM_CONFIG_JSON}

cur_rx_thread=$(( ${rx_thread}-1 ))
over=0
while [ "${over}" == "0" ]; do
    rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt
    echo "Run Hydra for rx thread ${cur_rx_thread}"
    cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson num ${cur_rx_thread} '.rx_thread_num=$num' > tmp.json
    cp tmp.json ${HYDRA_SYSTEM_CONFIG_JSON}
    ${hydra_root_dir}/scripts/control/run_all.sh -x || continue
    ${hydra_root_dir}/scripts/evaluation/latency_analysis.sh 0
    if [ -f ${hydra_root_dir}/data/frame_latency_all_0.txt ]; then
        echo "Succeed this time"
        rx_thread=${cur_rx_thread}
        cur_rx_thread=$(( ${cur_rx_thread}-1 ))
        continue
    fi
    over=1
done

cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson num ${rx_thread} '.rx_thread_num=$num' > tmp.json
cp tmp.json ${HYDRA_SYSTEM_CONFIG_JSON}

echo "Final setup: fft thread num ${fft_thread}, subcarrier block size ${sc_block_sz}, coding thread num ${coding_thread}, fft tx thread ${fft_tx_thread}, rx thread ${rx_thread}"
echo "Final setup (${HYDRA_SERVER_DEPLOY_JSON}): fft thread num ${fft_thread}, subcarrier block size ${sc_block_sz}, coding thread num ${coding_thread}, fft tx thread ${fft_tx_thread}, rx thread ${rx_thread}" >> result.txt
