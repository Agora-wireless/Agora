#!/bin/bash
#
# Usage:
#  * This script must be run from Agora's top-level directory
#  * test_agora.sh: Run the test once
#  * test_agora.sh 5: Run the test five times
#  * test_agora.sh 5 out_file: Run the test five times and redirect test
#    outputs to out_file. Only print pass/fail summary statistics to screen.
input_filepath="files/config/ci"

# Check that all required executables are present
exe_list="build/test_agora build/data_generator build/sender"
for exe in ${exe_list}; do
  if [ ! -f ${exe} ]; then
      echo "${exe} not found. Exiting."
      exit
  fi
done

num_iters=1
out_file=/dev/stdout

# Check if the user supplied a number-of-iterations argument
if [ "$#" -ge 1 ]; then
  num_iters=$1
fi

# Check if the user supplied an output file argument
if [ "$#" -eq 2 ]; then
  out_file=$2
  echo "Redirecting stdout to file = ${out_file}"
  rm -f ${out_file}
  touch ${out_file}
fi

# Process out_file and print summary of pass/fails stats. This should be called
# only iff the user supplied out_file
analyse_out() {
  max_errs=5
  n_uplink_passed=`cat ${out_file} | grep -i "Passed uplink test" | wc -l`
  n_uplink_failed=`cat ${out_file} | grep -i "Failed uplink test" | wc -l`
  n_downlink_passed=`cat ${out_file} | grep -i "Passed downlink test" | wc -l`
  n_downlink_failed=`cat ${out_file} | grep -i "Failed downlink test" | wc -l`
  n_combined_passed=`cat ${out_file} | grep -i "Passed combined test" | wc -l`
  n_combined_failed=`cat ${out_file} | grep -i "Failed combined test" | wc -l`

  >&2 echo "Iteration $i/${num_iters}: Uplink: ${n_uplink_passed} passed,"\
    "${n_uplink_failed} failed. Downlink: ${n_downlink_passed} passed,"\
    "${n_downlink_failed} failed. Combined: ${n_combined_passed} passed,"\
    "${n_combined_failed} failed. Listing up to ${max_errs} errors:"

  # Print any errors or warnings
  cat ${out_file} | grep "WARNG" | head -${max_errs}
  cat ${out_file} | grep "ERROR" | head -${max_errs}
}

echo "Running tests for $num_iters iterations"

for i in `seq 1 $num_iters`; do
  # Everything in the braces below gets redirected to $out_file
  {
    echo "==========================================="
    echo "Generating data for uplink correctness test $i......"
    echo -e "===========================================\n"
    ./build/data_generator --conf_file ${input_filepath}/tddconfig-correctness-test-ul.json
    
    echo -e "-------------------------------------------------------\n\n\n"
    echo "==========================================="
    echo "Running uplink correctness test $i......"
    echo -e "===========================================\n"
    # We sleep before starting the sender to allow the Agora server to start
    ./build/test_agora --conf_file ${input_filepath}/tddconfig-correctness-test-ul.json &
    sleep 1; ./build/sender --num_threads 1 --core_offset 10 --conf_file ${input_filepath}/tddconfig-correctness-test-ul.json
    wait

    echo "==========================================="
    echo "Generating data for downlink correctness test $i......"
    echo -e "===========================================\n"
    ./build/data_generator --conf_file ${input_filepath}/tddconfig-correctness-test-dl.json

    echo -e "-------------------------------------------------------\n\n\n"
    echo "==========================================="
    echo "Running downlink correctness test $i......"
    echo -e "===========================================\n"
    ./build/test_agora --conf_file ${input_filepath}/tddconfig-correctness-test-dl.json &
    sleep 1; ./build/sender --num_threads 1 --core_offset 10 --conf_file ${input_filepath}/tddconfig-correctness-test-dl.json
    echo -e "-------------------------------------------------------\n\n\n"
    wait

    echo "==========================================="
    echo "Generating data for uplink downlink combined correctness test $i......"
    echo -e "===========================================\n"
    ./build/data_generator --conf_file ${input_filepath}/tddconfig-correctness-test-both.json

    echo -e "-------------------------------------------------------\n\n\n"
    echo "==========================================="
    echo "Running combined correctness test $i......"
    echo -e "===========================================\n"
    ./build/test_agora --conf_file ${input_filepath}/tddconfig-correctness-test-both.json &
    sleep 1; ./build/sender --num_threads 1 --core_offset 10 --conf_file ${input_filepath}/tddconfig-correctness-test-both.json
    echo -e "-------------------------------------------------------\n\n\n"
    wait
  } >> $out_file

  # If the user supplied an output file, print pass/fail summary analysis
  if [ "$#" -eq 2 ]; then
    analyse_out
  fi
done
