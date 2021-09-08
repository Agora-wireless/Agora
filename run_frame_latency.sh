#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

num_ant=(64 64 64 128 128 150)
num_ue=(8 16 32 16 32 32)

for (( i=5; i<6; i++ ))
do
    ant=${num_ant[$i]}
    ue=${num_ue[$i]}
    for (( c=30; c<=90; c+=10 ))
    do
        echo "run evaluation for ant:$ant ue:$ue core:$c"
        cp $DIR/data/config/tddconfig-sim-ul-distributed-$ant-$ue-1ms-${c}c.json $DIR/data/tddconfig-sim-ul-distributed.json
        ./run_distributed_ul_dynamic.sh 10 $c 1
    done
done