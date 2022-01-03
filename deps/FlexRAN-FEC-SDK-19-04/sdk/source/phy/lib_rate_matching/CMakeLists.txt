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

# Kernel specific flags
# Call add_compile_options' for EACH option
if(WIN32)
  # windows options
  add_compile_options("")
else()
  # linux options
  add_compile_options("-fPIC")
  REMOVE_COMPILE_OPTION("-xCORE-AVX2")
  REMOVE_COMPILE_OPTION("-xCORE-AVX512")
endif()

# Kernel sources
set (KernelSrcs
  phy_rate_match.cpp
  phy_rate_match_avx2.cpp
  phy_rate_match_sse.cpp
  phy_rate_match_sse_short.cpp
  phy_rate_match_sse_k6144.cpp
  phy_de_rate_match_avx2.cpp
  phy_de_rate_match_avx512.cpp
)

# Kernel includes (public only)
set (KernelIncs
  phy_rate_match.h
)


# NOTE: Enclosing variables in quotes forms a single argument parameter to allow
# for more than one file to be present.
ADD_KERNEL("${KernelSrcs}" "${KernelIncs}")
