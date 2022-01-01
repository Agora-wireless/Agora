#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

servers=(roce83 roce82 roce81)

num_servers=$(cat $DIR/data/tddconfig-sim-ul-distributed.json | jq '.bs_server_addr_list | length')
for (( i=0; i<$num_servers; i++ ))
do
    ssh ${servers[$i]} taskset 0x1 mpstat -P 0-31 1 100 > $DIR/data/cpu_usage_$i.txt &
done

read var

for (( i=0; i<$num_servers; i++ ))
do
    ssh ${servers[$i]} sudo pkill mpstat
done