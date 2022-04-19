#! /bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh
source ${hydra_root_dir}/scripts/control/init_platform.sh

if [ ! -f ${hydra_root_dir}/scripts/evaluation/cdf ]; then
    g++ -o ${hydra_root_dir}/scripts/evaluation/cdf ${hydra_root_dir}/scripts/evaluation/cdf.cpp
fi

for (( i=0; i<12; i++ ))
do
    cat ${hydra_root_dir}/latency/msr_hydra_64_16_ul/frame_latency_all_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ${hydra_root_dir}/scripts/evaluation/cdf -m 0 -x 2.5 > tmp1.txt
rm tmp0.txt

for (( i=0; i<12; i++ ))
do
    cat ${hydra_root_dir}/latency/msr_hydra_128_32_ul/frame_latency_all_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ${hydra_root_dir}/scripts/evaluation/cdf -m 0 -x 2.5 > tmp2.txt
rm tmp0.txt

for (( i=0; i<12; i++ ))
do
    cat ${hydra_root_dir}/latency/msr_hydra_150_32_ul/frame_latency_all_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ${hydra_root_dir}/scripts/evaluation/cdf -m 0 -x 2.5 > tmp3.txt
rm tmp0.txt

for (( i=0; i<12; i++ ))
do
    cat ${hydra_root_dir}/latency/msr_hydra_192_32_ul/frame_latency_all_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ${hydra_root_dir}/scripts/evaluation/cdf -m 0 -x 2.5 > tmp4.txt
rm tmp0.txt

for (( i=0; i<12; i++ ))
do
    cat ${hydra_root_dir}/latency/msr_hydra_240_32_ul/frame_latency_all_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ${hydra_root_dir}/scripts/evaluation/cdf -m 0 -x 2.5 > tmp5.txt
rm tmp0.txt

paste tmp1.txt tmp2.txt tmp3.txt tmp4.txt tmp5.txt -d " " > ${hydra_root_dir}/data/measurement/fig_cdf_latency.txt
rm tmp*.txt