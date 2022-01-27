#!/bin/bash

if [ $# -ne 2 ]; then
  echo "usage: $0 <input_log_file> <output_data_file>"
  exit
fi
echo "Frame,Symbol,User,EVM,SNR" > $2
grep 'Frame.*Symbol.*User.*EVM.*SNR.*' $1 | awk '{print $2 $4 $6 $8 $10}' | sed 's/%//g' >> $2
