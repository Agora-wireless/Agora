#! /bin/bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

servers=(roce81 roce82 roce83)

num_servers=$(cat $DIR/data/tddconfig-sim-ul-distributed.json | jq '.bs_server_addr_list | length')
for (( i=0; i<$num_servers; i++ ))
do
    scp ${servers[$i]}:$DIR/data/frame_latency.txt $DIR/tmp.txt
    cat $DIR/tmp.txt | awk '{ print $1 $2 $2+$4 $2+$4+$5 $2+$4+$5+$6 $2+$4+$5+$6+$7 }' > $DIR/data/frame_latency_$i.txt
done

n_frames=2000
new_lines=()
for (( i=0; i<$n_frames; i++ ))
do
    min_t1=$(sed "${i}q;d" $DIR/data/frame_latency_0.txt | awk '{ print $2 }')
    max_t2=$(sed "${i}q;d" $DIR/data/frame_latency_0.txt | awk '{ print $3 }')
    max_t3=$(sed "${i}q;d" $DIR/data/frame_latency_0.txt | awk '{ print $4 }')
    min_t4=$(sed "${i}q;d" $DIR/data/frame_latency_0.txt | awk '{ print $5 }')
    max_t5=$(sed "${i}q;d" $DIR/data/frame_latency_0.txt | awk '{ print $6 }')
    for (( j=1; j<$num_servers; j++ ))
    do
        t1=$(sed "${i}q;d" $DIR/data/frame_latency_$j.txt | awk '{ print $2 }')
        t2=$(sed "${i}q;d" $DIR/data/frame_latency_$j.txt | awk '{ print $3 }')
        t3=$(sed "${i}q;d" $DIR/data/frame_latency_$j.txt | awk '{ print $4 }')
        t4=$(sed "${i}q;d" $DIR/data/frame_latency_$j.txt | awk '{ print $5 }')
        t5=$(sed "${i}q;d" $DIR/data/frame_latency_$j.txt | awk '{ print $6 }')
        if [ "$min_t1" -gt "$t1" ]; then
            min_t1=$t1
        fi
        if [ "$max_t2" -lt "$t2" ]; then
            max_t2=$t2
        fi
        if [ "$max_t3" -lt "$t3" ]; then
            max_t3=$t3
        fi
        if [ "$min_t4" -gt "$t4" ]; then
            min_t4=$t4
        fi
        if [ "$max_t5" -lt "$t5" ]; then
            max_t5=$t5
        fi
    done
    d12=$max_t2-$min_t1
    d23=$max_t3-$max_t2
    d34=$min_t4-$max_t3
    d45=$max_t5-$min_t4
    dall=$max_t5-$min_t1
    echo "$i $d12 $d23 $d34 $d45 $dall" >> $DIR/data/frame_latency_all.txt
done
