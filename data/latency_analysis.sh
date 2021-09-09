#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

rm $DIR/frame_latency_all.txt

servers=(roce83 roce82 roce82)

num_servers=$(cat $DIR/tddconfig-sim-ul-distributed.json | jq '.bs_server_addr_list | length')
for (( i=0; i<$num_servers; i++ ))
do
    scp ${servers[$i]}:$DIR/frame_latency.txt $DIR/frame_latency_$i.txt
done

python $DIR/merge_latency.py $num_servers