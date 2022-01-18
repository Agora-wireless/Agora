#! /bin/bash

source ~/project/intel/oneapi/setvars.sh
export LIBRARY_PATH=${LIBRARY_PATH}:~/project/rdma-core/build/lib
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:${LIBRARY_PATH}
export RTE_SDK=~/project/dpdk-stable-20.11.3
