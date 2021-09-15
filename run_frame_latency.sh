#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

num_ant=(64 64 128)
num_ue=(16 32 16)

for (( i=0; i<3; i++ ))
do
    ant=${num_ant[$i]}
    ue=${num_ue[$i]}
    for (( c=20; c<=90; c+=10 ))
    do
        echo "run evaluation for ant:$ant ue:$ue core:$c"
        cp $DIR/data/config/tddconfig-sim-ul-distributed-$ant-$ue-0.5ms-${c}c-hydra.json $DIR/data/tddconfig-sim-ul-distributed.json
        ./run_distributed_ul_dynamic.sh 10 $c "0.5"
    done
done