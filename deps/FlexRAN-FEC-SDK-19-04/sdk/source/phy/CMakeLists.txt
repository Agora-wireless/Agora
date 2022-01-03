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

project (Kernels CXX C)

# Common files
include_directories(lib_common)

# Macro helper function to remove compile option
macro(REMOVE_COMPILE_OPTION opt)
  get_directory_property( comp_opts DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMPILE_OPTIONS )
  list(REMOVE_ITEM comp_opts "${opt}")
  set_directory_properties(PROPERTIES COMPILE_OPTIONS "${comp_opts}")
endmacro()

# Macro helper function to create a kernel library.
macro(ADD_KERNEL KernelSrcs Headers)

  foreach(src ${KernelSrcs})
    list(APPEND KERNEL_SRC_LIST ${CMAKE_CURRENT_SOURCE_DIR}/${src})
  endforeach()

  # Derive library names from directory name.
  file(RELATIVE_PATH kernel_dir ${CMAKE_SOURCE_DIR}/source/phy ${CMAKE_CURRENT_SOURCE_DIR})

  # Remove underscore
  string(REPLACE "lib_" "lib" kernellib "${kernel_dir}")

  # Add libray
  add_library(${kernellib} ${KERNEL_SRC_LIST} ${COMMON_SRC_LIST})

  # Copy files needed at runtime to source folder (tables etc)
  file(GLOB BIN_FILES *.dat *.bin)
  file(COPY ${BIN_FILES} DESTINATION ${CMAKE_BINARY_DIR}/source/phy/${kernel_dir})

  # install
  install(FILES ${Headers} DESTINATION ${kernel_dir})
  install(TARGETS ${kernellib} DESTINATION ${kernel_dir} OPTIONAL)
  install(FILES ${BIN_FILES} DESTINATION ${kernel_dir} OPTIONAL)

endmacro()

# Pull in all the kernel directories.
macro(ADD_LIB_DIRS KernelNames)
  foreach(kernel ${KernelNames})
    # prefix lib_ onto kernel name
    add_subdirectory("lib_${kernel}")
  endforeach()
endmacro()

if($ENV{WIRELESS_SDK_STANDARD} MATCHES "lte")
  ADD_LIB_DIRS("${COMMON_KERNELS}")
  ADD_LIB_DIRS("${LTE_KERNELS}")
elseif($ENV{WIRELESS_SDK_STANDARD} MATCHES "5gnr")
  ADD_LIB_DIRS("${COMMON_KERNELS}")
  ADD_LIB_DIRS("${5GNR_KERNELS}")
else()
  ADD_LIB_DIRS("${COMMON_KERNELS}")
  ADD_LIB_DIRS("${LTE_KERNELS}")
  ADD_LIB_DIRS("${5GNR_KERNELS}")
  ADD_LIB_DIRS("${OTHER_KERNELS}")
endif()

