#!/bin/bash

if [ $# -eq 0 ]; then
  echo "usage: $0 <log_file_0> [<log_file_1> <log_file_2> ...]"
  exit
fi

pattern_dlpsnr="Frame (.+) Pilot SNR \(dB\) at UE Antenna 0: \[ (.+) (.+) \]"
pattern_dlpoff="Fft Pilot\(frame (.+) symbol 9 ant 0\) sig offset (.+)"
pattern_evmsnr="Frame: (.+), Symbol: 11, User: 0, EVM: (.+), SNR: (.+)"
pattern_se="Frame (.+) Symbol 11 Ue 0: (.+) symbol errors"
pattern_ber="UE (.+): Downlink bit errors \(BER\) (.+)/(.+) \((.+)\),"

echo 'Bit-Errors,Decoded-Bits,BER' > uedata-ber.csv
idx=0
for file in $@; do
  echo 'Frame,DL-Pilot-SNR-0' > "uedata-dlpsnr-$idx.csv"
  echo 'Frame,DL-Pilot-Offset-0' > "uedata-dlpoff-$idx.csv"
  echo 'Frame,EVM,EVM-SNR' > "uedata-evmsnr-$idx.csv"
  echo 'Frame,Symbol-Errors' > "uedata-se-$idx.csv"
  while read -r line; do
    if [[ $line =~ $pattern_dlpsnr ]]; then
      printf "%s,%s\n" ${BASH_REMATCH[1]} ${BASH_REMATCH[2]} \
          >> "uedata-dlpsnr-$idx.csv"
    elif [[ $line =~ $pattern_dlpoff ]]; then
      printf "%s,%s\n" ${BASH_REMATCH[1]} ${BASH_REMATCH[2]} \
          >> "uedata-dlpoff-$idx.csv"
    elif [[ $line =~ $pattern_evmsnr ]]; then
      printf "%s,%s,%s\n" ${BASH_REMATCH[1]} ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} \
          >> "uedata-evmsnr-$idx.csv"
    elif [[ $line =~ $pattern_se ]]; then
      printf "%s,%s\n" ${BASH_REMATCH[1]} ${BASH_REMATCH[2]} \
          >> "uedata-se-$idx.csv"
    elif [[ $line =~ $pattern_ber ]]; then
      printf "%s,%s,%s\n" ${BASH_REMATCH[2]} ${BASH_REMATCH[3]} ${BASH_REMATCH[4]} \
          >> "uedata-ber.csv"
      printf "BER[%d] = %s\n" $idx ${BASH_REMATCH[4]}
    fi
  done < $file
  ((idx++))
done

python3 plot-ue.py $idx False