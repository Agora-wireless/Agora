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
user=./build/sender
data_gen_exe=./build/data_generator
config=./files/config/ci/tddconfig-sim-ul-fr2.json
logpath=./log
logfile=$logpath/$(date +"%Y-%m-%d_%H-%M-%S").log

################
# Functions
################

# Function to display help
function display_help {
    echo "Usage: ./mvp_test.sh [option]"
    echo "Options:"
    echo "  -b, --build    - Build the project"
    echo "  -d, --debug    - Set the project in debug mode, need to rebuild"
    echo "  -n, --normal   - Set the project in normal mode, need to rebuild"
    echo "  -g, --datagen  - Generate the data from the config file"
    echo "  -x, --execute  - Run the basestation"
    echo "  -u, --user     - Run the user"
#     echo "  -v, --valgrind - Run the project with vlagrind"
    echo "  -r, --read     - Read the log of the latest test run"
    echo "  -c, --clean    - Clean the project"
    echo "  -h, --help     - Display this help message"
}

# Function to build the project
function build_project {
    echo "Building the project..."
    cd build
    make -j16
    cd ..
}

function set_debug {
    echo "Setting the project to debug mode..."
    cd build
    cmake .. -DDEBUG=true
    cd ..
}

function set_normal {
    echo "Setting the project to normal mode..."
    cd build
    cmake .. -DDEBUG=false
    cd ..
}

function gen_data {
    echo "Generating the data for simulation based on config file: $config"
    $data_gen_exe --conf_file $config
}

# Function to run the project
function exe_bs {
    echo "Running the basestation..."
    script -q -c "$exe --conf_file $config" $logfile
    # Use `cat log/2023-06-23_11-32-22.log | less -R` to read colored log file
}

function exe_user {
    echo "Running the user..."
    # $user --conf_file $config
    $user --conf_file $config   \
          --num_threads=2       \
          --core_offset=10      \
          --enable_slow_start=0
    # script -q -c "$user --conf_file $config" $logfile
}

# function valgrind_exe {
#     echo "Running the project with valgrind..."
#     script -q -c "valgrind --leak-check=full $exe --conf_file $config" $logfile
# }

# Function to read the log
function read_log {
    # Find the latest log file
    latest_log=$(ls -t "$logpath"/*.log | head -1)

    # Read the contents
    if [ -f "$latest_log" ]; then
        echo "Reading the latest log: $latest_log with"
        echo "cat \`$latest_log | less -R\` to read colored log file"
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
    "-d" | "--debug")
        set_debug
        ;;
    "-n" | "--normal")
        set_normal
        ;;
    "-g" | "--datagen")
        gen_data
        ;;
    "-x" | "--exe")
        exe_bs
        ;;
    "-u" | "--user")
        exe_user
        ;;
#     "-v" | "--valgrind")
#         valgrind_exe
#         ;;
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
