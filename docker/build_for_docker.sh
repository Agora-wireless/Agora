#!/bin/bash
set -e

# Obtain the absolute directory where this script exists, which is the `./docker` directory.
DOCKER_DIR=$(dirname $(readlink -f ${BASH_SOURCE}))
MILLIPEDE_BASE_DIR=$(readlink -f ${DOCKER_DIR}/.. )
echo "DOCKER_DIR: ${DOCKER_DIR}"
echo "MILLIPEDE_BASE_DIR: ${MILLIPEDE_BASE_DIR}"

# Navigate to the `docker` directory, everything in this script will be executed within there.
cd ${DOCKER_DIR}

# Perform a regular build, and do so in a new "build" directory
echo "Performing regular build ..."
( mkdir -p build;  cd build;  cmake ${MILLIPEDE_BASE_DIR};  make -j12 )

# (Optional) Build the simple tests too
echo "Performing test build ..."
( cd ${MILLIPEDE_BASE_DIR}/test/test_millipede;  cmake . ;  make -j12 )

echo -n "Copying ./test ... "
rm -rf ./test/
cp -r  ${MILLIPEDE_BASE_DIR}/test/   ./test/
echo "done"

echo -n "Copying ./data ... "
rm -rf ./data/
cp -r  ${MILLIPEDE_BASE_DIR}/data/   ./data/
echo "done"


# Build the docker image
docker build -t boos:Dockerfile ./

# Tag it so we can more easily push it to the Azure container registry
docker tag boos:Dockerfile booscr.azurecr.io/samples/millipede
