#!/bin/bash

set -e

script_dir="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
hydra_root_dir=$( cd ${script_dir}/../.. >/dev/null 2>&1 && pwd )

source ${hydra_root_dir}/scripts/utils/utils.sh

core_low=0
core_high=0
mode=0

if [ "$#" -eq 3 ]; then
    core_low=$1
    core_high=$2
    mode=$3
else
    echored "Invalid number of argumemts"
    exit
fi

if [ "${mode}" -eq 0 ]; then
    for (( i=${core_low}; i<${core_high}; i++ )); do
        echo "Disabling logical HT core $i."
        echo 0 > /sys/devices/system/cpu/cpu${i}/online;
    done
else
    for (( i=${core_low}; i<${core_high}; i++ )); do
        echo "Enbaling logical HT core $i."
        echo 1 > /sys/devices/system/cpu/cpu${i}/online;
    done
fi