#! /bin/bash

cat frame_latency_all.txt | awk 'NR>1000 { print $6;  }' > frame_latencies.txt
cat frame_latencies.txt | ./cdf -m 0 > cdf_latency.txt
python fig_cdf_latency.py
python fig_quant_latency.py