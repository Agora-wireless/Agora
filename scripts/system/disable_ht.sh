#!/bin/bash

source $(dirname $0)/../utils/utils.sh

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