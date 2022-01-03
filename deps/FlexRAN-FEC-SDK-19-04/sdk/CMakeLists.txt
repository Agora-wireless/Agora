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

#
# Top level project label
#
project (WIRELESS_SDK CXX C)

#
# Compiler options
#
# Include compile options file for chosen toolchain
# Load compiler specific kernel lists
message("WIRELESS_SDK_TOOLCHAIN is $ENV{WIRELESS_SDK_TOOLCHAIN}")
message("WIRELESS_SDK_SOURCE_DIR is ${WIRELESS_SDK_SOURCE_DIR}")
if($ENV{WIRELESS_SDK_TOOLCHAIN} MATCHES "icc")
  include (${WIRELESS_SDK_SOURCE_DIR}/cmake/intel-compile-options.cmake)
  include (${WIRELESS_SDK_SOURCE_DIR}/kernel-list-icc.cmake)
else($ENV{WIRELESS_SDK_TOOLCHAIN} MATCHES "gcc")
  include (${WIRELESS_SDK_SOURCE_DIR}/cmake/gcc-compile-options.cmake)
  include (${WIRELESS_SDK_SOURCE_DIR}/kernel-list-gcc.cmake)
endif()

#
# Wireless SDK options
#
# Set Wireless SDK preprocessor definitions for target ISA
#
if(WIN32)
  if(${ISA_SSE4_2})
    add_definitions("/D_BBLIB_SSE4_2_")
  elseif(${ISA_AVX2})
    add_definitions("/D_BBLIB_SSE4_2_ /D_BBLIB_AVX2_")
  elseif(${ISA_AVX512})
    add_definitions("/D_BBLIB_SSE4_2_ /D_BBLIB_AVX2_ /D_BBLIB_AVX512_")
  endif()
  if(RTE_SDK)
    add_definitions("/D_BBLIB_DPDK_")
  endif()
else()
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
endif()


#
#  Cmake build options
#
# Tell Cmake not to prefix library names with "lib"
set(CMAKE_STATIC_LIBRARY_PREFIX "")

# The 'deliverable' output of the builds are collected together via 'make install'.
# Kernel libraries are installed in directories specified by CMAKE_INSTALL_PREFIX.
set(CMAKE_INSTALL_PREFIX install)
IF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  SET(CMAKE_INSTALL_PREFIX install CACHE PATH "Cmake install prefix" FORCE)
ENDIF(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

# Display compiler flags set through CMAKE_BUILD_TYPE
if(${CMAKE_BUILD_TYPE} MATCHES "Debug")
  message("CMAKE_CXX_FLAGS_DEBUG is ${CMAKE_CXX_FLAGS_DEBUG}")
elseif(${CMAKE_BUILD_TYPE} MATCHES "Release")
  message("CMAKE_CXX_FLAGS_RELEASE is ${CMAKE_CXX_FLAGS_RELEASE}")
elseif(${CMAKE_BUILD_TYPE} MATCHES "RelWithDebInfo")
  message("CMAKE_CXX_FLAGS_RELWITHDEBINFO is ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
elseif(${CMAKE_BUILD_TYPE} MATCHES "MinSizeRel")
  message("CMAKE_CXX_FLAGS_MINSIZEREL is ${CMAKE_CXX_FLAGS_MINSIZEREL}")
endif()

#
# Don't require modules to be built before installation
#
set(CMAKE_SKIP_INSTALL_ALL_DEPENDENCY true)

#
# The kernel code
#
add_subdirectory(source/phy)

#
# c++ unit testing
#
if(EXISTS "$ENV{GTEST_ROOT}")
  enable_testing()
  find_package(GTest REQUIRED)
  add_subdirectory(test/phy)
endif()

#
# kfw
#
if(DEFINED ENV{ENABLE_KFW})
  message("ENABLING KERNEL FRAMEWORK")
  add_subdirectory(kfw)
endif()


