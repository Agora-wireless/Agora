#! /bin/bash

for (( i=0; i<10; i++ ))
do
    cat frame_latency/frame_latency_all_64_8_1ms_70c_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ./cdf -m 0 -x 2.5 > tmp1.txt
rm tmp0.txt

for (( i=0; i<10; i++ ))
do
    cat frame_latency/frame_latency_all_64_16_1ms_70c_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ./cdf -m 0 -x 2.5 > tmp2.txt
rm tmp0.txt

for (( i=0; i<10; i++ ))
do
    cat frame_latency/frame_latency_all_64_32_1ms_70c_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ./cdf -m 0 -x 2.5 > tmp3.txt
rm tmp0.txt

for (( i=0; i<10; i++ ))
do
    cat frame_latency/frame_latency_all_128_16_1ms_70c_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ./cdf -m 0 -x 2.5 > tmp4.txt
rm tmp0.txt

for (( i=0; i<10; i++ ))
do
    cat frame_latency/frame_latency_all_128_32_1ms_70c_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ./cdf -m 0 -x 2.5 > tmp5.txt
rm tmp0.txt

for (( i=0; i<10; i++ ))
do
    cat frame_latency/frame_latency_all_150_32_1ms_70c_$i.txt | awk 'NR>1000 { print $6;  }' >> tmp0.txt
done
cat tmp0.txt | ./cdf -m 0 -x 2.5 > tmp6.txt
rm tmp0.txt

paste tmp1.txt tmp2.txt tmp3.txt tmp4.txt tmp5.txt tmp6.txt -d " " > measurement/fig_cdf_latency.txt
rm tmp*.txt