#!/bin/bash

if [ $# -ne 1 ]; then
  echo 'usage: $0 <input_log_file>'
  exit
fi
echo 'Frame,User,EVM,SNR' > uedata-evmsnr.csv
grep 'Frame.*Symbol.*User.*EVM.*SNR.*' $1 | awk '{print $2 $6 $8 $10}' \
| sed 's/%//g' >> uedata-evmsnr.csv
echo 'Frame,User,SymbolErrors' > uedata-se.csv
grep 'Frame.*Symbol.*Ue.*symbol errors' $1 | awk '{print $2 "," $6 "," $7}' \
| sed 's/://g' >> uedata-se.csv
echo 'Frame,AntId,RFSNR' > uedata-rfsnr.csv
grep 'UeWorker: Fft Pilot(frame.*symbol 10 ant.*) sig offset.*SNR.*' $1 \
| awk '{print $4 "," $8 "," $13}' | sed 's/)//g' >> uedata-rfsnr.csv
python3 plot-ue.py
