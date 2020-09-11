#!/bin/bash

### This script is for executing only the Agora server.
### It first generates the simple simulated uplink data and 
### then it runs a simple Agora server (receiver).

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

echo -e "\n------------------------------------------------"
echo "Starting the agora server."
./build/agora data/tddconfig-sim-ul.json

echo "Agora server exited with exit code $?"
