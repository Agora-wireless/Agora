#!/bin/bash
set -e

# Obtain the absolute directory where this script exists, which is the `./docker` directory.
DOCKER_DIR=$(dirname $(readlink -f ${BASH_SOURCE}))
MILLIPEDE_BASE_DIR=$(readlink -f ${DOCKER_DIR}/.. )
FLEXRAN_DIR_NAME=FlexRAN-FEC-SDK-19-04
FLEXRAN_DIR=/opt/${FLEXRAN_DIR_NAME}
# echo "DOCKER_DIR: ${DOCKER_DIR}"
# echo "MILLIPEDE_BASE_DIR: ${MILLIPEDE_BASE_DIR}"


# Navigate to the `docker` directory, everything in this script will be executed within there.
cd ${DOCKER_DIR}

# Perform a regular build, and do so in a new `build` directory
echo "Performing regular build ..."
( mkdir -p build;  cd build;  cmake ${MILLIPEDE_BASE_DIR};  make -j12 )

# The build directory is now present, but we also need `data` and (optionally) `test`
echo -n "Copying ./data ... "
rm -rf ./data/
cp -r  ${MILLIPEDE_BASE_DIR}/data/   ./data/
echo "done"

echo -n "Copying ./test ... "
rm -rf ./test/
cp -r  ${MILLIPEDE_BASE_DIR}/test/   ./test/
echo "done"


# Millipede now requires the FlexRAN libs
echo -n "Copying ${FLEXRAN_DIR} ... "
cp -r  ${FLEXRAN_DIR}   ./${FLEXRAN_DIR_NAME}/
echo "done"


# Build the docker image
docker build -t boos:Dockerfile ./

# Tag it so we can more easily push it to the Azure container registry
docker tag boos:Dockerfile booscr.azurecr.io/samples/millipede
