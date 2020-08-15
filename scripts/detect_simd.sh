#!/bin/bash

# Auto detect CPU features
CPU_FEATURES_DETECT_AVX512=`cat /proc/cpuinfo | grep avx512 | wc -l`
CPU_FEATURES_DETECT_AVX2=`cat /proc/cpuinfo | grep avx2 | grep f16c | grep fma | grep bmi | wc -l`

if [ $CPU_FEATURES_DETECT_AVX512 -ne 0 ]
then
    ISA_SELECT="-DISA_AVX512=1"
elif [ $CPU_FEATURES_DETECT_AVX2 -ne 0 ]
then
    ISA_SELECT="-DISA_AVX2=1"
fi

# Uncomment to generate makefiles if needed
#rm -rf CMakeFiles/
#rm CMakeCache.txt
#cmake -G "Unix Makefiles" $ISA_SELECT . || exit 1
