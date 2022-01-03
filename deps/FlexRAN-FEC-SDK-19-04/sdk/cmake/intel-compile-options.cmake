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

# This file contains the Intel compiler options for both Windows and Linux

# Settings are global to WIRELESS_SDK project

# TODO - cannot enable Werror /WX until warnings fixed


# diag-disable
#   :10397 Some error details are only displayed if optimisation reports are enabled, so they are
#          enabled. However this generates a load of
#          "icc: remark #10397: optimization reports are generated in *.optrpt files in the output location"
#          comments. 10397 disables this output from icc, and instead is replaced by a single cmake
#          message below
message("icc: remark #10397: optimization reports are generated in *.optrpt files in the output location")
# - 10382, cpu-dispatch Suppresses remarks about automatic cpu dispatch when xHost option is used
# - 13000 is suppressing a warning about locale settings for Windows.
# - 981 'operator evaluation unordered' - Intel recommend turning off for C++.
# - 869 Parameter unused. Commenting out the parameter everywhere is clunky.
# - 383 Reference to temporary - very common with STL so disable.
# - 2547 Include paths for MKL are set twice to same location. One is ignored anyway.
# - 11074
# - 11075
# - 11076
set (INTEL_DIAG_DISABLE "10397,10382,13000,981,869,383,2547,11074,11075,11076,cpu-dispatch")

if (WIN32)
  #
  # Windows
  #
  # Set CMAKE_BUILD_TYPE specific c++ compile flags (overrides CMake defaults)
  set(CMAKE_CXX_FLAGS_DEBUG  "/g")
  set(CMAKE_CXX_FLAGS_RELEASE  "/O3 /DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "/O2 /g /DNDEBUG")
  set(CMAKE_CXX_FLAGS_MINSIZEREL  "/Os /DNDEBUG")

  # Compile flags for all ISA and build types (do not get passed to linker)

  add_compile_options("/W5")

  # More extensive error checking disabled until SDK updates
  #add_compile_options("/Wcheck")
  #add_compile_options("/WX")

  add_compile_options("/EHsc")
  add_compile_options("/Qstd=c++11")
  add_compile_options("/Qrestrict")
  add_compile_options("/Qdiag-disable:${INTEL_DIAG_DISABLE}")
  add_compile_options("/Qmkl:sequential")

  # Set ISA specific compile flags (do not get passed to linker)
  if(${ISA_SSE4_2})
    # Compile flags / defintions for SSE4_2 (Windows)
    add_compile_options("/QxSSE4.2")
  elseif(${ISA_AVX2})
    # Compile flags / defintions for AVX2 (Windows)
    add_compile_options("/QxCORE-AVX2")
  elseif(${ISA_AVX512})
    # Compile flags / defintions for AVX512 (Windows)
    add_compile_options("/QxCORE-AVX512")
  endif()

else()
  #
  # Linux
  #
  # Set CMAKE_BUILD_TYPE specific c++ compile flags (overrides CMake defaults)
  set(CMAKE_CXX_FLAGS_DEBUG  "-O1 -g")
  set(CMAKE_CXX_FLAGS_RELEASE  "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO  "-O2 -g -DNDEBUG")
  set(CMAKE_CXX_FLAGS_MINSIZEREL  "-Os -DNDEBUG")

  # Compile flags for all ISA and build types (do not get passed to linker)

  add_compile_options("-Wall")
  add_compile_options("-ffreestanding")

  # More extensive error checking disabled until SDK updates
  #add_compile_options("-Wcheck")
  #add_compile_options("-Wremarks ")
  #add_compile_options("-Werror")

  add_compile_options("-std=c++11")
  add_compile_options("-restrict")
  add_compile_options("-diag-enable=all")
  add_compile_options("-diag-disable=${INTEL_DIAG_DISABLE}")
  add_compile_options("-qopt-report=4")
  add_compile_options("-qopt-report-phase=all")

  # Set ISA specific compile flags (do not get passed to linker)
  if(${ISA_SSE4_2})
    # Compile flags / defintions for SSE4_2 (Linux)
    add_compile_options("-xSSE4.2")
  elseif(${ISA_AVX2})
    # Compile flags / defintions for AVX2 (Linux)
   add_compile_options("-xCORE-AVX2")

  elseif(${ISA_AVX512})
    # Compile flags / defintions for AVX512 (Linux)
    add_compile_options("-xCORE-AVX512")
  endif()

  # linux linker flags for unittests executable
  set(CMAKE_EXE_LINKER_FLAGS "-lmkl_intel_lp64 -lmkl_core -lmkl_intel_thread -liomp5 -lpthread -lipps")

endif()
