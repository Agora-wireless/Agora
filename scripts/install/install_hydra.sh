#! /bin/bash

DIR=$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )
ROOT_DIR=$( cd ${DIR}/../.. >/dev/null 2>&1 && pwd )

# Build Agora
echocyan "Build Hydra"
cd ${ROOT_DIR}
mkdir build
cd build
cmake .. -DLOG_LEVEL=warn
make -j