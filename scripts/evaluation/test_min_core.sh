#! /bin/bash
set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

echo "Checking out ${HYDRA_SERVER_DEPLOY_JSON}"
git checkout ${HYDRA_SERVER_DEPLOY_JSON}

sc_block_sz=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.subcarrier_block_list[0]')
coding_thread=$(cat ${HYDRA_SERVER_DEPLOY_JSON} | jq '.coding_thread_num[0]')

rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt

${hydra_root_dir}/scripts/control/run_all.sh -x || continue
${hydra_root_dir}/scripts/evaluation/latency_analysis.sh 0
if [ -f ${hydra_root_dir}/data/frame_latency_all_0.txt ]; then
    echo "Succeed this time"
else
    echo "Failed in the beginning"
    exit
fi

cur_sc_block_sz=$(( ${sc_block_sz}+2 ))
over=0
while [ "${over}" != "2" ]; do
    rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt
    echo "Run Hydra for subcarrier block size ${cur_sc_block_sz} and coding thread num ${coding_thread}"
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
    echo "Run Hydra for subcarrier block size ${sc_block_sz} and coding thread num ${cur_coding_thread}"
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

echo "Final setup: subcarrier block size ${sc_block_sz}, coding thread num ${coding_thread}"
echo "Final setup (${HYDRA_SERVER_DEPLOY_JSON}): subcarrier block size ${sc_block_sz}, coding thread num ${coding_thread}" >> result.txt
