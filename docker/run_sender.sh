#!/bin/bash

### This script first generates the simple simulated uplink data 
### and then runs the sender simulator process endlessly until it is killed.

# Before we do anything, we need to add the proper env vars for Intel MKL, etc.
# Even though the Dockerfile might do this too, the .bashrc file will not necessarily be `source`d 
# in a k8s container because the container's command will run before/without bash. 
source /opt/intel/compilers_and_libraries/linux/bin/compilervars.sh intel64

echo "__mkl_lib_dir env var is: \"$__mkl_lib_dir\""

cd /Agora
echo -e "Generating data for uplink simulation ...\n"
./build/data_generator ./data/tddconfig-sim-ul.json
echo -e "\nDone generating data.\n"

echo -e "\nContents of data file ./data/tddconfig-sim-ul.json:"
cat ./data/tddconfig-sim-ul.json
echo -e "\n"

echo "Starting the agora sender client."
./build/sender --num_threads=2 --core_offset=0 --delay=5000000 --enable_slow_start=true --conf_file=data/tddconfig-sim-ul.json 

echo "Sender exited with exit code $?"
