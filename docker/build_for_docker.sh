#!/bin/bash
set -e

### This script builds Agora locally and creates a docker image
### that can execute Agora, either as a local container
### or as a remote container (e.g., part of a kubernetes cluster).
### 
### Pass "clean" as a command-line argument to this script to remove docker build files.
### Otherwise, all arguments will be passed into the `cmake` invocation.


# Obtain the absolute directory where this script exists, which is the `./docker` directory.
DOCKER_DIR=$(dirname $(readlink -f ${BASH_SOURCE}))
AGORA_BASE_DIR=$(readlink -f ${DOCKER_DIR}/.. )
FLEXRAN_DIR_NAME=FlexRAN-FEC-SDK-19-04
FLEXRAN_DIR=/opt/${FLEXRAN_DIR_NAME}
# echo "DOCKER_DIR: ${DOCKER_DIR}"
# echo "AGORA_BASE_DIR: ${AGORA_BASE_DIR}"


# Navigate to the `docker` directory, everything in this script will be executed within there.
cd ${DOCKER_DIR}

if [ "$1" = "clean" ]; then
    echo -n "Cleaning docker build files ... "
    rm -rf ./build/
    rm -rf ./data/
    rm -rf ./test/
    rm -rf ./${FLEXRAN_DIR_NAME}/
    echo "done."
    exit 0
fi

# Perform a regular build, and do so in a new `build` directory
echo "Performing regular build ..."
( mkdir -p build;  \
  cd build;  \
  cmake -DFORCE_BUILD_PATH=off ${AGORA_BASE_DIR}  $@;  \
  make -j12 \
)

# The build directory is now present, but we also need `data` and (optionally) `test`
echo -n "Copying ./data into docker context ... "
rm -rf ./data/
cp -rf  ${AGORA_BASE_DIR}/data/   ./data/
echo "done"

echo -n "Copying ./test into docker context ... "
rm -rf ./test/
cp -rf  ${AGORA_BASE_DIR}/test/   ./test/
echo "done"


# Agora now requires the FlexRAN libs
echo -n "Copying ${FLEXRAN_DIR} into docker context ... "
cp -rf  ${FLEXRAN_DIR}   ./${FLEXRAN_DIR_NAME}/
echo "done"


DOCKER_TAG="agora:Dockerfile"

# Build the docker image
docker build -t ${DOCKER_TAG} ./

# Add another tag so we can more easily push it to the Azure container registry
docker tag ${DOCKER_TAG} booscr.azurecr.io/samples/agora


echo -e "$(tput setaf 10)\nDocker build complete. You can run the container locally with:$(tput sgr0)"
echo "    docker run -ti ${DOCKER_TAG}"
