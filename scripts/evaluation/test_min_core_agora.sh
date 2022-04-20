#! /bin/bash
set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

echo "Checking out ${HYDRA_SERVER_DEPLOY_JSON}"
git checkout ${HYDRA_SERVER_DEPLOY_JSON}

worker_thread=$(cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq '.worker_thread_num')

rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt

${hydra_root_dir}/scripts/control/run_all.sh -x || continue
${hydra_root_dir}/scripts/evaluation/latency_analysis.sh 0
if [ -f ${hydra_root_dir}/data/frame_latency_all_0.txt ]; then
    echo "Succeed this time"
else
    echo "Failed in the beginning"
    exit
fi

cur_worker_thread=$(( ${worker_thread}-2 ))
over=0
while [ "${over}" == "0" ]; do
    rm -f ${hydra_root_dir}/data/frame_latency_all_0.txt
    echo "Run Hydra for worker thread num ${cur_worker_thread}"
    cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson num ${cur_worker_thread} '.worker_thread_num=$num' > tmp.json
    cp tmp.json ${HYDRA_SERVER_DEPLOY_JSON}
    ${hydra_root_dir}/scripts/control/run_all.sh -x || continue
    ${hydra_root_dir}/scripts/evaluation/latency_analysis.sh 0
    if [ -f ${hydra_root_dir}/data/frame_latency_all_0.txt ]; then
        echo "Succeed this time"
        worker_thread=${cur_worker_thread}
        cur_worker_thread=$(( ${cur_worker_thread}-2 ))
        continue
    fi
    over=1
done

cat ${HYDRA_SYSTEM_CONFIG_JSON} | jq --argjson num ${cur_worker_thread} '.worker_thread_num=$num' > tmp.json
cp tmp.json ${HYDRA_SERVER_DEPLOY_JSON}

echo "Final setup: worker thread num ${worker_thread}"
echo "Final setup (${HYDRA_SERVER_DEPLOY_JSON}): worker thread num ${worker_thread}" >> result.txt
