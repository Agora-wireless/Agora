#!/bin/bash
#
# Usage:
# ./test_millipede.sh: Run the test once
# ./test_millipede.sh 5: Run the test five times
# ./test_millipede.sh 5 out_file: Run the test five times and redirect test
#     outputs to out_file. Only print pass/fail summary statistics to screen.

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
  n_uplink_passed=`cat ${out_file} | grep -i "Passed uplink test" | wc -l`
  n_uplink_failed=`cat ${out_file} | grep -i "Failed uplink test" | wc -l`
  n_downlink_passed=`cat ${out_file} | grep -i "Passed downlink test" | wc -l`
  n_downlink_failed=`cat ${out_file} | grep -i "Failed downlink test" | wc -l`

  >&2 echo "Iteration $i/${num_iters}: Uplink: ${n_uplink_passed} passed, ${n_uplink_failed} failed. Downlink: ${n_downlink_passed} passed, ${n_downlink_failed} failed."
}

echo "Running tests for $num_iters iterations"

for i in `seq 1 $num_iters`; do
  # Everything in the braces below gets redirected to $out_file
  {
    echo "==========================================="
    echo "Generating data for uplink correctness test $i......"
    echo -e "===========================================\n"
    ./data_generator data/tddconfig-correctness-test-ul.json
    
    echo -e "-------------------------------------------------------\n\n\n"
    echo "======================================"
    echo "Running uplink correctness test $i......"
    echo -e "======================================\n"
    # We sleep before starting the sender to allow the Millipede server to start
    ./millipede data/tddconfig-correctness-test-ul.json &
    sleep 1; ./sender 4 10 5000 data/tddconfig-correctness-test-ul.json
    wait

    echo "==========================================="
    echo "Generating data for downlink correctness test $i......"
    echo -e "===========================================\n"
    ./data_generator data/tddconfig-correctness-test-dl.json

    echo -e "-------------------------------------------------------\n\n\n"
    echo "======================================"
    echo "Running downlink correctness test $i......"
    echo -e "======================================\n"
    ./millipede data/tddconfig-correctness-test-dl.json &
    sleep 1; ./sender 4 10 5000 data/tddconfig-correctness-test-dl.json
    echo -e "-------------------------------------------------------\n\n\n"
    wait
  } >> $out_file

  # If the user supplied an output file, print pass/fail summary analysis
  if [ "$#" -eq 2 ]; then
    analyse_out
  fi
done
