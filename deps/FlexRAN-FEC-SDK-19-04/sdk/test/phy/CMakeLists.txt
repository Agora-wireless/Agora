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

project(PhyTests)

# Set CPU ID used for performance tests
set(PERF_CPU_ID 4)

# Options sepcific to all tests
# Call add_compile_options' for EACH option
if(WIN32)
  # windows options
  add_compile_options("")
  add_definitions("/DCPU_ID=${PERF_CPU_ID}")
else()
  # linux options
  add_compile_options("")
  add_definitions("-DCPU_ID=${PERF_CPU_ID}")
endif()

# Macro used in test directories
#
# KernelName = Name of kernel under test (eg lib_modulation)
# TestFiles  = One or more gtest test files (eg modulation_functional.cc)
# BinaryName = Output binary name (eg unittests)
#
macro(ADD_TEST_SUITE KernelName TestFiles BinaryName)

  # derive kernel library name
  string(REPLACE "lib_" "lib" KernelLib "${KernelName}")
  # derive kernel test dir name
  string(REPLACE "lib_" "test_" KernelTest "${KernelName}")

  if(RTE_SDK)
    # Define include directories
    include_directories(/opt/intel/mkl/include
                        ${GTEST_INCLUDE_DIRS}
                        ${RTE_SDK}/${RTE_TARGET}/include
                        ${CMAKE_SOURCE_DIR}/test/phy/common/
                        ${CMAKE_SOURCE_DIR}/source/phy/lib_common/
                        ${CMAKE_SOURCE_DIR}/source/phy/${KernelName}/)

    # Add DPDK lib driectory and define libraries
    link_directories(${RTE_SDK}/${RTE_TARGET}/lib)
    set( DPDK_LIB  "-Wl,--whole-archive -lrte_eal -lrte_kvargs -Wl,--no-whole-archive" )
    set( NUMA_LIB  "-Wl,-lnuma" )

    # Add unittessts executable target
    add_executable (${KernelTest}
                    ${TestFiles}
                    ../common/bblib_unit_test_main.cc
                   )

    # Libraries to use when linking unittests
    target_link_libraries (${KernelTest}
                          ${GTEST_BOTH_LIBRARIES}
                          ${NUMA_LIB}
                          ${DPDK_LIB}
                          ${CMAKE_BINARY_DIR}/source/phy/${KernelName}/${KernelLib}.a
                          ${CMAKE_BINARY_DIR}/test/phy/common/utility.a
                          ${CMAKE_BINARY_DIR}/source/phy/lib_common/libcommon.a)
  else()
    # Define include directories
    include_directories(/opt/intel/mkl/include
                        ${GTEST_INCLUDE_DIRS}
                        ${CMAKE_SOURCE_DIR}/test/phy/common/
                        ${CMAKE_SOURCE_DIR}/source/phy/lib_common/
                        ${CMAKE_SOURCE_DIR}/source/phy/${KernelName}/)

    # Add unittessts executable target
    add_executable (${KernelTest}
                    ${TestFiles}
                    ../common/bblib_unit_test_main.cc
                   )

    # Libraries to use when linking unittests
    target_link_libraries (${KernelTest}
                          ${GTEST_BOTH_LIBRARIES}
                          ${CMAKE_BINARY_DIR}/source/phy/${KernelName}/${KernelLib}.a
                          ${CMAKE_BINARY_DIR}/test/phy/common/utility.a
                          ${CMAKE_BINARY_DIR}/source/phy/lib_common/libcommon.a)
  endif()

  # Add test config, vectors and tables (needed at runtime)
  add_custom_command(TARGET ${KernelTest} POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_directory
                     ${CMAKE_SOURCE_DIR}/test/phy/${KernelTest}/test_vectors
                     $<TARGET_FILE_DIR:${KernelTest}>/test_vectors
                     COMMAND ${CMAKE_COMMAND} -E copy_directory
                     ${CMAKE_SOURCE_DIR}/test/phy/${KernelTest}/test_tables
                     $<TARGET_FILE_DIR:${KernelTest}>/test_tables
                     COMMAND ${CMAKE_COMMAND} -E copy
                     ${CMAKE_SOURCE_DIR}/test/phy/${KernelTest}/*conf*
                     $<TARGET_FILE_DIR:${KernelTest}>/
                    )

  set_target_properties(${KernelTest} PROPERTIES OUTPUT_NAME ${BinaryName})

  add_dependencies(${KernelTest} libcommon utility ${KernelLib})

endmacro()

# Macro used in test directories
#
# KernelName = Name of kernel under test (eg lib_modulation)
# LibraryPath  = Path to the kernel (the *.a file) that should be linked against the test binary
# DependencyName = Name of the kernel library (eg libcrc)
#
macro(ADD_DEPENDENCY KernelName LibraryPath DependencyName)

  # derive kernel library name
  string(REPLACE "lib_" "lib" KernelLib "${KernelName}")
  # derive kernel test dir name
  string(REPLACE "lib_" "test_" KernelTest "${KernelName}")

  # link the library
  target_link_libraries(${KernelTest} ${LibraryPath})

  # add dependency to allow parallel build
  add_dependencies(${KernelTest} ${DependencyName})

endmacro()

#
# Add commmon code
#
add_subdirectory(common)

# Add the test directories.
#
macro(ADD_TEST_DIRS KernelNames)
  foreach(kernel ${KernelNames})
    # check naming convention - source folder should exist named lib_kernelname
    if(EXISTS ${CMAKE_SOURCE_DIR}/source/phy/lib_${kernel})
      add_subdirectory ("test_${kernel}")
    else()
      message(FATAL_ERROR "test_${kernel} - invalid test directory name, source directory lib_${kernel} not found")
    endif()
  endforeach()
endmacro()

if($ENV{WIRELESS_SDK_STANDARD} MATCHES "lte")
  ADD_TEST_DIRS("${COMMON_KERNELS}")
  ADD_TEST_DIRS("${LTE_KERNELS}")
elseif($ENV{WIRELESS_SDK_STANDARD} MATCHES "5gnr")
  ADD_TEST_DIRS("${COMMON_KERNELS}")
  ADD_TEST_DIRS("${5GNR_KERNELS}")
else()
  ADD_TEST_DIRS("${COMMON_KERNELS}")
  ADD_TEST_DIRS("${LTE_KERNELS}")
  ADD_TEST_DIRS("${5GNR_KERNELS}")
  ADD_TEST_DIRS("${OTHER_KERNELS}")
endif()
