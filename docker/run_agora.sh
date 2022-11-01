#!/bin/bash

### This script is for executing only the Agora server.
### It first generates the simple simulated uplink data and 
### then it runs a simple Agora server (receiver).

# Before we do anything, we need to add the proper env vars for Intel MKL, etc.
# Even though the Dockerfile might do this too, the .bashrc file will not necessarily be `source`d 
# in a k8s container because the container's command will run before/without bash. 
source /opt/intel/oneapi/setvars.sh --config="/opt/intel/oneapi/renew-config.txt"
input_filepath="files/config/ci"

echo "__mkl_lib_dir env var is: \"$__mkl_lib_dir\""

cd /Agora
echo -e "Generating data for uplink simulation ...\n"
./build/data_generator ${input_filepath}/tddconfig-sim-ul.json
echo -e "\nDone generating data.\n"

echo -e "\nContents of data file ${input_filepath}/tddconfig-sim-ul.json:"
cat ${input_filepath}/tddconfig-sim-ul.json
echo -e "\n"

echo -e "\n------------------------------------------------"
echo "Starting the agora server."
./build/agora --conf_file ${input_filepath}/tddconfig-sim-ul.json

echo "Agora server exited with exit code $?"
