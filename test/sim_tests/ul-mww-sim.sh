#!/bin/bash
#
# Usage:
#  * This script must be run from Agora's top-level directory.
###############################################################################

input_file="files/config/examples/ul-mww-sim.json"
output_filepath="files/log"

# Check that all required executables are present
exe_list="build/user build/data_generator build/chsim build/agora ${input_file}"
for exe in ${exe_list}; do
  if [ ! -f ${exe} ]; then
      echo "${exe} not found. Exiting."
      exit
  fi
done

echo "==========================================="
echo "Generating data for emulated RRU end-to-end test with channel simulator ......"
echo -e "===========================================\n"
./build/data_generator --conf_file ${input_file}

echo "==========================================="
echo "Running emulated RRU end-to-end test with channel simulator ......"
echo -e "===========================================\n"
./build/user --conf_file ${input_file} > ${output_filepath}/test_user_output.txt &
sleep 1; ./build/chsim --bs_threads 1 --ue_threads 1 --worker_threads 1 --core_offset 12 --conf_file ${input_file} > ${output_filepath}/test_chsim_output.txt &
sleep 1; ./build/agora --conf_file ${input_file} > ${output_filepath}/test_agora_output.txt

sleep 5
pkill -INT chsim
pkill -INT user
pkill -INT agora

sleep 1
grep ".*bit errors (BER).*" ${output_filepath}/test_user_output.txt
grep ".*bit errors (BER).*" ${output_filepath}/test_agora_output.txt
echo "=================================================="
