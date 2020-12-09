#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

servers=(roce82)

if [ $# -eq 1 ]
then
    cat $DIR/data/tddconfig-sim-dl-distributed.json | jq --argjson num_test $1 '.frames_to_test=$num_test' > tmp.json
    mv tmp.json $DIR/data/tddconfig-sim-dl-distributed.json
fi

$DIR/build/data_generator --conf_file $DIR/data/tddconfig-sim-dl-distributed.json

num_servers=$(cat $DIR/data/tddconfig-sim-dl-distributed.json | jq '.bs_server_addr_list | length')
for (( i=0; i<$num_servers; i++ ))
do
    cat $DIR/data/tddconfig-sim-dl-distributed.json | jq --argjson i $i '.bs_server_addr_idx=$i' > $DIR/data/tddconfig-sim-dl-distributed_$i.json
    scp $DIR/data/tddconfig-sim-dl-distributed_$i.json junzhi@${servers[$i]}:$DIR/data/tddconfig-sim-dl-distributed.json
    ssh junzhi@${servers[$i]} cd Agora; ./build/data_generator --conf_file ./data/tddconfig-sim-dl-distributed.json
done
