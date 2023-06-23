#!/bin/bash

# This script runs the Agora in RRU mode with USRP.
#
# Copyright 2023 Chung-Hsuan Tung
#
# This script is used to test if the code can run.
# Thus, it does not look into if the processing is correct, nor is it meaningful
# to read the output numbers.
# What matters is the execution flow.
# The script will direct the output to a log file with a timestamp.

exe=./build/agora
config=./files/config/examples/ul-usrp.json
logfile=./log/$(date +"%Y-%m-%d_%H-%M-%S").log

################
# Compile
################
# cd build
# make -j16
# cd ..

################
# Run the test
################
script -q -c "$exe --conf_file $config" $logfile
# Use `cat log/2023-06-23_11-32-22.log | less -R` to read colored log file
