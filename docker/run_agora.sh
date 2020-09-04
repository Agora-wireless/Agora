#!/bin/bash

### This script is for executing a simple Agora demo from within the docker container. 
### It first generates the simple simulated uplink data, 
### then it runs a simple Agora server (receiver),
### and finally it runs the Agora sender process. 

# Before we do anything, we need to add the proper env vars for Intel MKL, etc.
# Even though the Dockerfile might do this too, the .bashrc file will not necessarily be `source`d 
# in a k8s container because the container's command will run before/without bash. 
source /opt/intel/compilers_and_libraries/linux/bin/compilervars.sh intel64

echo "__mkl_lib_dir env var is: \"$__mkl_lib_dir\""

cd /Agora
echo -n "Generating data for uplink simulation ... "
./build/data_generator ./data/tddconfig-sim-ul.json
echo "done"

echo "Starting the agora server."
./build/agora data/tddconfig-sim-ul.json &
sleep 3

echo "Starting the agora sender client."
./build/sender --num_threads=2 --core_offset=0 --delay=5000 --enable_slow_start=false --conf_file=data/tddconfig-sim-ul.json & 
sleep 20
wait

echo "Done with testing."
