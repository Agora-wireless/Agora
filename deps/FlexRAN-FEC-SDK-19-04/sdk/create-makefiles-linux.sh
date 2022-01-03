#!/bin/bash

#######################################################################
#
# INTEL CONFIDENTIAL
# Copyright 2009-2019 Intel Corporation All Rights Reserved.
# 
# The source code contained or described herein and all documents related to the
# source code ("Material") are owned by Intel Corporation or its suppliers or
# licensors. Title to the Material remains with Intel Corporation or its
# suppliers and licensors. The Material may contain trade secrets and proprietary
# and confidential information of Intel Corporation and its suppliers and
# licensors, and is protected by worldwide copyright and trade secret laws and
# treaty provisions. No part of the Material may be used, copied, reproduced,
# modified, published, uploaded, posted, transmitted, distributed, or disclosed
# in any way without Intel's prior express written permission.
# 
# No license under any patent, copyright, trade secret or other intellectual
# property right is granted to or conferred upon you by disclosure or delivery
# of the Materials, either expressly, by implication, inducement, estoppel or
# otherwise. Any license under such intellectual property rights must be
# express and approved by Intel in writing.
# 
# Unless otherwise agreed by Intel in writing, you may not remove or alter this
# notice or any other notice embedded in Materials by Intel or Intel's suppliers
# or licensors in any way.
# 
#  version: SDK-jenkins-FlexRAN-SDK-REL-448-g3be238
#
#######################################################################

# This script creates the Makefiles needed to build the kernels.
# The makefiles are created in $DIR_WIRELESS_SDK_BUILD

# Set DIR_WIRELESS_SDK
export DIR_WIRELESS_SDK="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Checks
case "$GTEST_ROOT" in
"")
    echo "INFO:  Environment variable GTEST_ROOT not set, testing disabled"
    echo "       Expecting GTEST_ROOT=/your_path_to_gtest/gtest-1.7.0/"
    ;;
*)
    echo "INFO:  Environment variable GTEST_ROOT=$GTEST_ROOT"
    ;;
esac

case "$CMAKE_BUILD_TYPE" in
"Debug" | "Release" | "RelWithDebInfo" | "MinSizeRel")
    echo "INFO:  Environment variable CMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE"
    ;;
"")
    export CMAKE_BUILD_TYPE=Release
    echo "INFO:  Environment variable CMAKE_BUILD_TYPE not found, defaulting to $CMAKE_BUILD_TYPE"
    ;;
*)
    echo "ERROR: Environment variable CMAKE_BUILD_TYPE set to invalid value"
    echo "       Valid settings: Debug, Release, RelWithDebInfo, MinSizeRel"
    exit 1
    ;;
esac

case "$WIRELESS_SDK_TOOLCHAIN" in
"icc" | "gcc")
    echo "INFO:  Environment variable WIRELESS_SDK_TOOLCHAIN=$WIRELESS_SDK_TOOLCHAIN"
    ;;
"")
    export WIRELESS_SDK_TOOLCHAIN=icc
    echo "INFO:  Environment variable WIRELESS_SDK_TOOLCHAIN not found, defaulting to $WIRELESS_SDK_TOOLCHAIN"
    ;;
*)
    echo "ERROR: Environment variable WIRELESS_SDK_TOOLCHAIN set to invalid value"
    echo "       Valid settings: icc, gcc"
    exit 1
    ;;
esac

case "$WIRELESS_SDK_TARGET_ISA" in
"sse4_2" | "avx2" | "avx512")
    echo "INFO:  Environment variable WIRELESS_SDK_TARGET_ISA=$WIRELESS_SDK_TARGET_ISA"
    ;;
"")
    # Auto detect CPU features
    CPU_FEATURES_DETECT_AVX512=`cat /proc/cpuinfo | grep avx512 | wc -l`
    CPU_FEATURES_DETECT_AVX2=`cat /proc/cpuinfo | grep avx2 | grep f16c | grep fma | grep bmi | wc -l`

    if [ $CPU_FEATURES_DETECT_AVX512 -ne 0 ]
    then
        export WIRELESS_SDK_TARGET_ISA=avx512
    elif [ $CPU_FEATURES_DETECT_AVX2 -ne 0 ]
    then
        export WIRELESS_SDK_TARGET_ISA=avx2
    fi

    echo "INFO:  Environment variable WIRELESS_SDK_TARGET_ISA not found, auto-detecting $WIRELESS_SDK_TARGET_ISA "
    ;;

*)
    echo "ERROR: Environment variable WIRELESS_SDK_TARGET_ISA not set correctly"
    echo "       Valid settings: avx2, avx512"
    exit 1
    ;;
esac

case "$WIRELESS_SDK_STANDARD" in
"lte" | "5gnr" | "all")
    echo "INFO:  Environment variable WIRELESS_SDK_STANDARD=$WIRELESS_SDK_STANDARD"
    ;;
"")
    export WIRELESS_SDK_STANDARD=all
    echo "INFO:  Environment variable WIRELESS_SDK_STANDARD not found, defaulting to $WIRELESS_SDK_STANDARD"
    ;;

*)
    echo "ERROR: Environment variable WIRELESS_SDK_STANDARD not set correctly"
    echo "       Valid settings: lte, 5gnr, all"
    exit 1
    ;;
esac

case "$RTE_SDK" in
"")
    echo "INFO:  Environment variable RTE_SDK not set, building without DPDK huge page memory support."
    echo "       For best performance please build with DPDK by setting RTE_SDK environment variable:"
    echo "       RTE_SDK=/your_path_to_dpdk/dpdk-version"
    ;;
*)
    echo "INFO:  Environment variable RTE_SDK=$RTE_SDK"
    echo "       Note that the verification tests assume that huge pages of 1G have been set."
    echo "       Smaller huge page configurations will lead to page faults and degraded performance."
    ;;
esac

if [ "$RTE_SDK" != "" ]
then
    case "$RTE_TARGET" in
    "x86_64-native-linuxapp-icc" | "x86_64-native-linuxapp-gcc")
        echo "INFO:  Environment variable RTE_TARGET=$RTE_TARGET."
        ;;
    "")
        export RTE_TARGET=x86_64-native-linuxapp-icc
        echo "INFO:  Environment variable RTE_TARGET not found, defaulting to $RTE_TARGET"
        ;;
    *)
        echo "ERROR: Environment variable RTE_TARGET set to invalid value"
        echo "       Valid settings: x86_64-native-linuxapp-icc, x86_64-native-linuxapp-gcc"
        exit 1
        ;;
    esac
fi

# Do not support DESTDIR
case "$DESTDIR" in
"")
    ;;
*)
    echo "ERROR: Environment variable DESTDIR=$DESTDIR detected"
    echo "       Use of DESTDIR not supported, please delete from environment"
    exit 1
    ;;
esac

# Checks OK - select ISA and TOOLCHAIN
export DIR_WIRELESS_SDK_BUILD=build-$WIRELESS_SDK_TARGET_ISA-$WIRELESS_SDK_TOOLCHAIN

# define toolchain file based on $WIRELESS_SDK_TOOLCHAIN
if [ $WIRELESS_SDK_TOOLCHAIN == "icc" ]
then
    TOOLCHAIN_FILE=$DIR_WIRELESS_SDK/cmake/toolchain-intel-linux.cmake
elif [ $WIRELESS_SDK_TOOLCHAIN == "gcc" ]
then
    TOOLCHAIN_FILE=$DIR_WIRELESS_SDK/cmake/toolchain-gcc-linux.cmake
fi

# define ISA switches based on $WIRELESS_SDK_TARGET_ISA
if [ $WIRELESS_SDK_TARGET_ISA == "avx2" ]
then
    ISA_SELECT="-DISA_AVX2=1"
elif  [ $WIRELESS_SDK_TARGET_ISA == "avx512" ]
then
    ISA_SELECT="-DISA_AVX512=1"
fi

# define DPDK switch based on $RTE_SDK
if [ "$RTE_SDK" != "" ]
then
    DPDK_SETTINGS="-DRTE_SDK=$RTE_SDK -DRTE_TARGET=$RTE_TARGET"
else
    DPDK_SETTINGS=""
fi

# Create clean build directory
cd $DIR_WIRELESS_SDK
rm -rf $DIR_WIRELESS_SDK_BUILD
mkdir $DIR_WIRELESS_SDK_BUILD

# Generate makefiles
cd $DIR_WIRELESS_SDK_BUILD
cmake -G "Unix Makefiles" $ISA_SELECT -DCMAKE_BUILD_TYPE=$CMAKE_BUILD_TYPE -DCMAKE_TOOLCHAIN_FILE=$TOOLCHAIN_FILE $DPDK_SETTINGS .. || exit 1
cd ..
