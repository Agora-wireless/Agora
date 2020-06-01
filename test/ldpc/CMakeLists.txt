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

# Cmake 2.8.12 needed for "add_compile_options" command
cmake_minimum_required(VERSION 2.8.12)

find_program(ICPC_EXISTS "icpc")
if(NOT ICPC_EXISTS)
  message(FATAL_ERROR "Intel C++ compiler not found. Exiting.")
endif()

set(CMAKE_CXX_COMPILER icpc)  # Intel compiler required
set(CMAKE_CXX_FLAGS "-std=c++11 -O3 -mavx2 -mavx")

# Set Wireless SDK preprocessor definitions for target ISA
if(${ISA_SSE4_2})
  add_definitions("-D_BBLIB_SSE4_2_")
elseif(${ISA_AVX2})
  add_definitions("-D_BBLIB_SSE4_2_ -D_BBLIB_AVX2_")
elseif(${ISA_AVX512})
  add_definitions("-D_BBLIB_SSE4_2_ -D_BBLIB_AVX2_ -D_BBLIB_AVX512_")
endif()

if(RTE_SDK)
  add_definitions("-D_BBLIB_DPDK_")
endif()

set(FLEXRAN_FEC_SDK_DIR "/opt/FlexRAN-FEC-SDK-19-04/sdk")
set(FLEXRAN_FEC_LIB_DIR ${FLEXRAN_FEC_SDK_DIR}/build-avx2-icc)

include_directories(
  /opt/intel/mkl/include
  ${FLEXRAN_FEC_SDK_DIR}/test/phy/common
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_ldpc_decoder_5gnr
  ${FLEXRAN_FEC_SDK_DIR}/source/phy/lib_common
  ../../src/encoder)

set(SOURCES 
  ../../src/encoder/cyclic_shift.cpp
  ../../src/encoder/encoder.cpp
  ../../src/encoder/iobuffer.cpp)

add_executable(test_ldpc test_ldpc.cpp ${SOURCES})
target_link_libraries (test_ldpc
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a)

add_definitions(-DUSE_LDPC)
set(MOD_SOURCES ../../src/common/modulation.cpp)
include_directories(../../src/common)
add_executable(test_mod_ldpc test_mod_ldpc.cpp ${SOURCES} ${MOD_SOURCES})
target_link_libraries(test_mod_ldpc
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_ldpc_decoder_5gnr/libldpc_decoder_5gnr.a
  ${FLEXRAN_FEC_LIB_DIR}/source/phy/lib_common/libcommon.a)
