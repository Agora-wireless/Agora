#!/bin/bash

if [ $# -eq 0 ]; then
  echo "usage: $0 <log_file_0> [<log_file_1> <log_file_2> ...]"
  exit
fi

pattern1="Frame (.+) Pilot SNR \(dB\) at UE Antenna (.+): \[ (.+) (.+) \]"
pattern2="Frame: (.+), Symbol: (.+), User: (.+), EVM: (.+), SNR: (.+)"
pattern3="Frame (.+) Symbol (.+) Ue (.+): (.+) symbol errors"
pattern4="UE (.+): Downlink bit errors \(BER\) (.+)/(.+) \((.+)\),"

echo 'Bit-Errors,Decoded-Bits,BER' > uedata-ber.csv
idx=0
for file in $@; do
  echo 'Frame,DL-Pilot-SNR' > "uedata-dlpsnr-$idx.csv"
  echo 'Frame,EVM,EVM-SNR' > "uedata-evmsnr-$idx.csv"
  echo 'Frame,Symbol-Errors' > "uedata-se-$idx.csv"
  while read -r line; do
    if [[ $line =~ $pattern1 ]]; then
      printf "%s,%s\n" ${BASH_REMATCH[1]} ${BASH_REMATCH[3]} \
          >> "uedata-dlpsnr-$idx.csv"
    elif [[ $line =~ $pattern2 ]]; then
      printf "%s,%s,%s\n" ${BASH_REMATCH[1]} ${BASH_REMATCH[4]} ${BASH_REMATCH[5]} \
          >> "uedata-evmsnr-$idx.csv"
    elif [[ $line =~ $pattern3 ]]; then
      printf "%s,%s\n" ${BASH_REMATCH[1]} ${BASH_REMATCH[4]} \
          >> "uedata-se-$idx.csv"
    elif [[ $line =~ $pattern4 ]]; then
      printf "%s,%s,%s\n" ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} ${BASH_REMATCH[4]} \
          >> "uedata-ber.csv"
    fi
  done < $file
  ((idx++))
done

python3 plot-ue.py $idx False