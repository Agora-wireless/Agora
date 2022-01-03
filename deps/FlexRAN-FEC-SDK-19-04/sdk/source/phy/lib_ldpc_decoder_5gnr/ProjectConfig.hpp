/**
*** INTEL CONFIDENTIAL
***
*** Copyright 2015-2016 Intel Corporation
***
***
*** The source code contained or described herein and all documents related to
*** the source code ("Material") are owned by Intel Corporation or its suppliers
*** or licensors. Title to the Material remains with Intel Corporation or its
*** suppliers and licensors. The Material contains trade secrets and proprietary
*** and confidential information of Intel or its suppliers and licensors. The
*** Material is protected by worldwide copyright and trade secret laws and treaty
*** provisions. No part of the Material may be used, copied, reproduced,
*** modified, published, uploaded, posted, transmitted, distributed, or disclosed
*** in any way without Intel's prior express written permission.
***
*** No license under any patent, copyright, trade secret or other intellectual
*** property right is granted to or conferred upon you by disclosure or delivery
*** of the Materials, either expressly, by implication, inducement, estoppel or
*** otherwise. Any license under such intellectual property rights must be
*** express and approved by Intel in writing.
***
**/

#pragma once

#include <stdint.h>

// This configuration file sets global constants and macros which are
// of general use throughout the project.

// All current IA processors of interest align their cache lines on
// this boundary. If the cache alignment for future processors changes
// then the most restrictive alignment should be set.
constexpr unsigned k_cacheByteAlignment = 64;

// Force the data to which this macro is applied to be aligned on a cache line.
// For example:
//
// CACHE_ALIGNED float data[64];
#define CACHE_ALIGNED alignas(k_cacheByteAlignment)

// Hint to the compiler that the data to which this macro is applied
// can be assumed to be aligned to a cache line. This allows the
// compiler to generate improved code by using aligned reads and
// writes.
#define ASSUME_CACHE_ALIGNED(data) __assume_aligned(data, k_cacheByteAlignment);

/// Intel compiler frequently complains about templates not being declared in an external
/// header. Templates are used throughout this project's source files to define local type-specific
/// versions of functions. Defining every one of these in a header is unnecessary, so the warnings
/// about this are turned off globally.
#pragma warning(disable:1418)
#pragma warning(disable:1419)

//#include "common/half.hpp"
