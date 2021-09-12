#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

servers=(roce83 roce82 roce81)
pci=('"37:00.1"' '"37:00.0"' '"37:00.0"')

batch_mode=0

num_antennas=$(cat $DIR/data/tddconfig-sim-ul-distributed.json | jq '.antenna_num')
num_ue=$(cat $DIR/data/tddconfig-sim-ul-distributed.json | jq '.ue_num')
slot_size="0.5"
num_cores=80
num_runs=10

if (( $# >= 1 )); then
    if (( $# != 3 )); then
        echo Illegal number of arguments!
        echo $# $1 $2 $3
        exit
    fi
    batch_mode=1
    num_runs=$1
    num_cores=$2
    slot_size=$3
fi

if [ $# -eq 1 ]
then
    cat $DIR/data/tddconfig-sim-ul-distributed.json | jq --argjson num_test $1 '.frames_to_test=$num_test' > tmp.json
    mv tmp.json $DIR/data/tddconfig-sim-ul-distributed.json
fi

# $DIR/build/data_generator --conf_file $DIR/data/tddconfig-sim-ul-distributed.json
$DIR/build/control_generator --conf_file $DIR/data/tddconfig-sim-ul-distributed.json
$DIR/build/dynamic_generator --conf_file $DIR/data/tddconfig-sim-ul-distributed.json

num_servers=$(cat $DIR/data/tddconfig-sim-ul-distributed.json | jq '.bs_server_addr_list | length')
for (( i=0; i<$num_servers; i++ ))
do
    cat $DIR/data/tddconfig-sim-ul-distributed.json | jq --argjson i $i '.bs_server_addr_idx=$i' | jq --argjson pci_addr ${pci[$i]} '.pci_addr=$pci_addr' > $DIR/data/tddconfig-sim-ul-distributed_$i.json
    scp $DIR/data/tddconfig-sim-ul-distributed_$i.json ${servers[$i]}:$DIR/data/tddconfig-sim-ul-distributed.json
    # ssh ${servers[$i]} cd Agora; ./build/data_generator --conf_file ./data/tddconfig-sim-ul-distributed.json
    scp $DIR/data/control_ue_template.bin ${servers[$i]}:$DIR/data/control_ue_template.bin
    scp $DIR/data/control_ue.bin ${servers[$i]}:$DIR/data/control_ue.bin
done

slot_us=$( cat $DIR/data/tddconfig-sim-ul-distributed.json | jq '.slot_size' )
if [ "$slot_us" = "null" ]
then
    slot_us=500
fi

if (( $batch_mode == 0 )); then
    exit
fi

for (( T=0; T<$num_runs; T++ ))
# for T in 9
do
    echo Start to run servers $T
    for (( i=0; i<$num_servers; i++ ))
    do
        ssh ${servers[$i]} "cd Agora; sudo env LD_LIBRARY_PATH=$LD_LIBRARY_PATH bash run_agora.sh > /dev/null" &
    done
    sleep 10
    sudo env LD_LIBRARY_PATH=$LD_LIBRARY_PATH nice -20 chrt -r 99 ./build/dynamic_sender --num_threads=6 --conf_file=./data/tddconfig-sim-ul-distributed.json --frame_duration=$slot_us --core_offset=4 > /dev/null
    sleep 10
    cd data
    bash latency_analysis.sh
    cp frame_latency_all.txt frame_latency/frame_latency_all_${num_antennas}_${num_ue}_${slot_size}ms_${num_cores}c_${T}_hydra.txt
    cd ..
done