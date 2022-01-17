#! /bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
ROOT_DIR=$( cd ${DIR}/../.. >/dev/null 2>&1 && pwd )

PROJECT_ROOT=~/project

# Setup Hugepages and modprobe
cd ${PROJECT_ROOT}/dpdk-stable-20.11.3
sudo ./usertools/dpdk-hugepages.py --setup 22G
sudo modprobe -a ib_uverbs mlx5_core mlx5_ib