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

################
# Path & Param
################

exe=./build/agora
config=./files/config/examples/ul-usrp.json
logpath=./log
logfile=$logpath/$(date +"%Y-%m-%d_%H-%M-%S").log

################
# Functions
################

# Function to display help
function display_help {
    echo "Usage: ./mvp_test.sh [option]"
    echo "Options:"
    echo "  -b, --build   - Build the project"
    echo "  -x, --execute - Run the project"
    echo "  -r, --read    - Read the log of the latest test run"
    echo "  -c, --clean   - Clean the project"
    echo "  -h, --help    - Display this help message"
}

# Function to build the project
function build_project {
    echo "Building the project..."
    cd build
    make -j16
    cd ..
}

# Function to run the project
function exe_project {
    echo "Running the project..."
    script -q -c "$exe --conf_file $config" $logfile
    # Use `cat log/2023-06-23_11-32-22.log | less -R` to read colored log file
}

# Function to read the log
function read_log {
    # Find the latest log file
    latest_log=$(ls -t "$logpath"/*.log | head -1)

    # Read the contents
    if [ -f "$latest_log" ]; then
        echo "Reading the latest log: $latest_log"
        # tail "$latest_log"
        cat $latest_log | less -R
    else
        echo "No log files found in the directory"
    fi
}

# Function to clean the project
function clean_project {
    echo "Cleaning the project..."
    cd build
    make clean
    cd ..
}

################
# Handle Inputs
################

# Check the number of arguments
if [ $# -ne 1 ]; then
    display_help
    exit 1
fi

# Handle the argument
case "$1" in
    "-b" | "--build")
        build_project
        ;;
    "-x" | "--exe")
        exe_project
        ;;
    "-r" | "--read")
        read_log
        ;;
    "-c" | "--clean")
        clean_project
        ;;
    "-h" | "--help")
        display_help
        ;;
    *)
        echo "Invalid option."
        display_help
        ;;
esac
