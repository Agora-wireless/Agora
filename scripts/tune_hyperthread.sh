#!/bin/bash
echo 'Current status'
cat /sys/devices/system/cpu/cpu*/online

for i in {36..71};
do
    #echo ${i}
    file=/sys/devices/system/cpu/cpu${i}/online
    echo ${file}
    sudo sh -c "echo ${1} > ${file}"
    cat /sys/devices/system/cpu/cpu${i}/online
done
