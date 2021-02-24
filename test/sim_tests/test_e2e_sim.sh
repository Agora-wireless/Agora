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
sed -i '2i\ \ "frames_to_test": 1000,' data/bs-sim-tmp.json

echo "==========================================="
echo "Generating data for emulated RRU end-to-end test with channel simulator ......"
echo -e "===========================================\n"
./build/data_generator --conf_file data/bs-sim-tmp.json

echo "==========================================="
echo "Running emulated RRU end-to-end test with channel simulator ......"
echo -e "===========================================\n"
./build/user data/ue-sim.json > test_output.txt &
sleep 1; ./build/chsim --bs_threads 1 --ue_threads 1 --worker_threads 2 --core_offset 24 --bs_conf_file data/bs-sim-tmp.json --ue_conf_file data/ue-sim.json &
sleep 1; ./build/agora data/bs-sim-tmp.json > test_output.txt


# Agora is terminated automatically. Manually terminate user and chsim
rm data/bs-sim-tmp.json
pkill -INT user >> test_output.txt
pkill chsim
sleep 1
grep "UE .*:" test_output.txt
echo "=================================================="


# Decide Pass/Fail
let ue_res_count=$(grep -c "(BER)" test_output.txt)/2
ant_string=$(grep " UE antennas," test_output.txt)
read num1 num2 num_bs_ants num_ues num5 <<<${ant_string//[^0-9]/ }
if [ "$ue_res_count" != "$num_ues" ]; then
  echo "Failed the end-to-end test with channel simulator!"
  echo "=================================================="
  rm test_output.txt
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
    echo "Failed the end-to-end test with channel simulator!"
    echo "=================================================="
    rm test_output.txt
    exit
  fi
done < <(grep "UE .*:" test_output.txt)

echo "Passed the end-to-end test with channel simulator!"
echo "=================================================="
rm test_output.txt
