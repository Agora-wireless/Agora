#!/bin/bash

if [ $# -eq 0 ]; then
	echo "usage: $0 <log_file_0> [<log_file_1> <log_file_2> ...]"
	exit
fi

echo 'Bit-Errors,Decoded-Bits,BER' > uedata-ber.csv
idx=0
for file in $@
do
	echo 'Frame,EVM,EVM-SNR' > "uedata-evmsnr-$idx.csv"
	grep 'Frame.*Symbol.*User: 0, EVM.*SNR.*' $file | awk '{print $2 $8 $10}' \
			| sed 's/%//g;/nan/d;/inf/d' >> "uedata-evmsnr-$idx.csv"
	echo 'Frame,Symbol-Errors' > "uedata-se-$idx.csv"
	grep 'Frame.*Symbol.*Ue 0:.*symbol errors' $file | awk '{print $2 "," $7}' \
			| sed 's/://g;/nan/d;/inf/d' >> "uedata-se-$idx.csv"
	echo 'Frame,RF-SNR' > "uedata-rfsnr-$idx.csv"
	grep 'UeWorker: Fft Pilot(frame.*symbol 10 ant 0) sig offset.*SNR.*' $file \
			| awk '{print $4 "," $13}' | sed 's/)//g;/nan/d;/inf/d' \
			>> "uedata-rfsnr-$idx.csv"
	grep 'UE 0: Downlink bit errors (BER).*' $file | awk '{print $7}' \
			| sed 's/\//,/g;s/(/,/g;s/),//g;/nan/d;/inf/d' >> uedata-ber.csv
	((idx++))
done

python3 plot-ue.py $idx
