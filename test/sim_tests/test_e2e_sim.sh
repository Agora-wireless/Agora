#!/bin/bash
#
# Usage:
#  * This script must be run from Agora's top-level directory.
#  * If a number is passed to the script, it is used as Pass/Fail threshold.
#  * Otherwise, the default 0.005 shall be used as Pass/Fail threshold.
###############################################################################

# Check that all required executables are present
exe_list="build/user build/data_generator build/chsim build/agora data/bs-sim.json"
for exe in ${exe_list}; do
  if [ ! -f ${exe} ]; then
      echo "${exe} not found. Exiting."
      exit
  fi
done


# Setup the config with the number of frames to test
cp data/bs-sim.json data/bs-sim-tmp.json
cp data/ue-sim.json data/ue-sim-tmp.json
sed -i '2i\ \ "frames_to_test": 1000,' data/bs-sim-tmp.json
sed -i '2i\ \ "frames_to_test": 1000,' data/ue-sim-tmp.json

echo "==========================================="
echo "Generating data for emulated RRU end-to-end test with channel simulator ......"
echo -e "===========================================\n"
./build/data_generator --conf_file data/bs-sim-tmp.json

echo "==========================================="
echo "Running emulated RRU end-to-end test with channel simulator ......"
echo -e "===========================================\n"
echo "Emulated RRU Test" > test_user_output.txt
echo "Emulated RRU Test" > test_agora_output.txt
echo "Emulated RRU Test" > test_chsim_output.txt
./build/user --conf_file data/ue-sim-tmp.json >> test_user_output.txt &
sleep 1; ./build/chsim --bs_threads 1 --ue_threads 1 --worker_threads 4 --core_offset 29 --bs_conf_file data/bs-sim-tmp.json --ue_conf_file data/ue-sim-tmp.json >> test_chsim_output.txt &
sleep 1; ./build/agora -conf_file data/bs-sim-tmp.json >> test_agora_output.txt

sleep 5;

# Agora and User is terminated automatically. Manually terminate chsim
rm data/bs-sim-tmp.json
rm data/ue-sim-tmp.json

pkill -INT chsim
pkill -INT user
pkill -INT agora

sleep 1
grep ".*bit errors (BER).*" test_user_output.txt > test_output.txt
grep ".*bit errors (BER).*" test_agora_output.txt >> test_output.txt
grep ".*bit errors (BER).*" test_output.txt
echo "=================================================="


# Decide Pass/Fail
let ue_res_count=$(grep -c "(BER)" test_output.txt)/2
ant_string=$(grep " UE antennas," test_user_output.txt)
read num1 num2 num_bs_ants num_ues num5 <<<${ant_string//[^0-9]/ }

if [ "$ue_res_count" != "$num_ues" ]; then
  echo "Failed the end-to-end test with channel simulator due to invalid number of records!"
  echo "=================================================="
  rm test_agora_output.txt
  rm test_user_output.txt
  rm test_output.txt
  rm test_chsim_output.txt
  exit
fi

THRESH=$1
if [ "$THRESH" == "" ]; then
  THRESH=0.005
fi
echo "BER test threshold is ${THRESH}"
while read -r line ; do
  ue_BER=$(printf %.15f $(echo $line | awk -F '[()]' '{FOO=$4; print $4}'))
  if [ "`echo "${ue_BER} > ${THRESH}" | bc`" -eq 1 ]; then
    echo "UE BER " ${ue_BER} "exceeds threshold"
    echo "Failed the end-to-end test with channel simulator!"
    echo "=================================================="
    rm test_agora_output.txt
    rm test_user_output.txt
    rm test_output.txt
    rm test_chsim_output.txt
    exit
  fi
done < <(grep ".*bit errors (BER)." test_output.txt)

echo "Passed the end-to-end test with channel simulator!"
echo "=================================================="
rm test_agora_output.txt
rm test_user_output.txt
rm test_output.txt
rm test_chsim_output.txt
exit