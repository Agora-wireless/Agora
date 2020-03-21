#!/bin/bash
num_iters=1

# Check if the user supplied a number-of-iterations argument
if [ "$#" -eq 1 ]; then
  num_iters=$1
fi

echo "Running tests for $num_iters iterations"

for i in `seq 1 $num_iters`; do
  echo "Generating data for correctness test"
  ./data_generator data/tddconfig-correctness-test-ul.json
  echo -e "-------------------------------------------------------\n\n\n"
  echo "==================================="
  echo "Running uplink correctness test......"
  echo -e "===================================\n"
  ./millipede data/tddconfig-correctness-test-ul.json &
  ./sender 4 10 5000 data/tddconfig-correctness-test-ul.json

  echo -e "-------------------------------------------------------\n\n\n"
  echo "==================================="
  echo "Running downlink correctness test......"
  echo -e "===================================\n"
  ./millipede data/tddconfig-correctness-test-dl.json &
  ./sender 4 10 5000 data/tddconfig-correctness-test-dl.json

done