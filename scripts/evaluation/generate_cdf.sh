#! /bin/bash

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

dir_name=$1
FILES="${hydra_root_dir}/latency/${dir_name}/frame_latency_all_*.txt"

rm -f cumulate_data.txt
for f in ${FILES}
do
    cat ${f} | awk 'NR>1000 { print $6;  }' >> cumulate_data.txt
done

cat cumulate_data.txt | ./cdf -m 0 > cdf_latency.txt
# python fig_cdf_latency.py
# python fig_quant_latency.py