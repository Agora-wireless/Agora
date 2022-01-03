/**********************************************************************
*
* INTEL CONFIDENTIAL
* Copyright 2009-2019 Intel Corporation All Rights Reserved.
* 
* The source code contained or described herein and all documents related to the
* source code ("Material") are owned by Intel Corporation or its suppliers or
* licensors. Title to the Material remains with Intel Corporation or its
* suppliers and licensors. The Material may contain trade secrets and proprietary
* and confidential information of Intel Corporation and its suppliers and
* licensors, and is protected by worldwide copyright and trade secret laws and
* treaty provisions. No part of the Material may be used, copied, reproduced,
* modified, published, uploaded, posted, transmitted, distributed, or disclosed
* in any way without Intel's prior express written permission.
* 
* No license under any patent, copyright, trade secret or other intellectual
* property right is granted to or conferred upon you by disclosure or delivery
* of the Materials, either expressly, by implication, inducement, estoppel or
* otherwise. Any license under such intellectual property rights must be
* express and approved by Intel in writing.
* 
* Unless otherwise agreed by Intel in writing, you may not remove or alter this
* notice or any other notice embedded in Materials by Intel or Intel's suppliers
* or licensors in any way.
* 
*  version: SDK-jenkins-FlexRAN-SDK-REL-448-g3be238
*
**********************************************************************/

/*
 * @file
 * @brief  Source code of External API for LTE Rate Matching,
 * Dematching ( HARQ & deinterleaver ) functions in LTE
*/

#include <functional>
#include <cstdio>
#include <cstdint>

#include "phy_rate_match.h"
#include "phy_rate_match_internal.h"
#include "sdk_version.h"

typedef int32_t (*rate_match_dl_func)(const struct bblib_rate_match_dl_request *request,
    struct bblib_rate_match_dl_response *response);

typedef int32_t (*rate_match_ul_func)(const struct bblib_rate_match_ul_request *request,
    struct bblib_rate_match_ul_response *response);

typedef int32_t (*harq_combine_ul_func)(const struct bblib_harq_combine_ul_request *request,
    struct bblib_harq_combine_ul_response *response);

typedef int32_t (*deinterleave_ul_func)(const struct bblib_deinterleave_ul_request *request,
    struct bblib_deinterleave_ul_response *response);

typedef int32_t (*turbo_adapter_ul_func)(const struct bblib_turbo_adapter_ul_request *request,
    struct bblib_turbo_adapter_ul_response *response);

struct bblib_rate_match_init
{
    bblib_rate_match_init()
    {
#if !defined(_BBLIB_AVX2_) && !defined(_BBLIB_AVX512_)
        printf("__func__ rate_match cannot run with this CPU type, needs AVX2 or greater.\n");
        exit(-1);
#endif
        bblib_print_rate_match_version();
    }
};

bblib_rate_match_init do_constructor_rate_matching;

int16_t bblib_rate_match_version(char *version, int buffer_size) {
    /* The version string will be updated before the build process starts  by the
     *       jobs building the library and/or preparing the release packages.
     *       Do not edit the version string manually */
    const char *msg = "FlexRAN SDK bblib_lte_rate_matching version sdk-19.04-ea1-1-g3be2380";

    return (bblib_sdk_version(&version, &msg, buffer_size));
}

/** Return a pointer-to-function for a specific implementation */
static rate_match_dl_func bblib_rate_match_dl_select_on_isa() {
    /* Although there are both SSE and AVX2 version of the DL code
     * there is only AVX2 support for the UL code.  Therefore the default
     * for DL will also be AVX2.
     */
    return bblib_rate_match_dl_avx2;
}

/** Return a pointer-to-function for a specific implementation */
static rate_match_ul_func bblib_rate_match_ul_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_rate_match_ul_avx512;
#else
    return bblib_rate_match_ul_avx2;
#endif
}

/** Return a pointer-to-function for a specific implementation */
static harq_combine_ul_func bblib_harq_combine_ul_select_on_isa() {
#ifdef _BBLIB_AVX512_
    //return bblib_harq_combine_ul_avx512;
    return bblib_harq_combine_ul_avx2;
#else
    return bblib_harq_combine_ul_avx2;
#endif
}

/** Return a pointer-to-function for a specific implementation */
static deinterleave_ul_func bblib_deinterleave_ul_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_deinterleave_ul_avx512;
#else
    return bblib_deinterleave_ul_avx2;
#endif
}

/** Return a pointer-to-function for a specific implementation */
static turbo_adapter_ul_func bblib_turbo_adapter_ul_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_turbo_adapter_ul_avx512;
#else
    return bblib_turbo_adapter_ul_avx2;
#endif
}

static rate_match_dl_func default_dl_func = bblib_rate_match_dl_select_on_isa();
static rate_match_ul_func default_ul_func = bblib_rate_match_ul_select_on_isa();
static harq_combine_ul_func default_harq_func = bblib_harq_combine_ul_select_on_isa();
static deinterleave_ul_func default_deinterleave_func = bblib_deinterleave_ul_select_on_isa();
static turbo_adapter_ul_func default_turbo_adapter_func = bblib_turbo_adapter_ul_select_on_isa();

int32_t bblib_rate_match_dl(const struct bblib_rate_match_dl_request *request,
        struct bblib_rate_match_dl_response *response) {
    return default_dl_func(request, response);
}

int32_t bblib_rate_match_ul(const struct bblib_rate_match_ul_request *request,
        struct bblib_rate_match_ul_response *response) {
    return default_ul_func(request, response);
}

int32_t bblib_harq_combine_ul(const struct bblib_harq_combine_ul_request *request,
        struct bblib_harq_combine_ul_response *response) {
    return default_harq_func(request, response);
}

int32_t bblib_deinterleave_ul(const struct bblib_deinterleave_ul_request *request,
        struct bblib_deinterleave_ul_response *response) {
    return default_deinterleave_func(request, response);
}

int32_t bblib_turbo_adapter_ul(const struct bblib_turbo_adapter_ul_request *request,
        struct bblib_turbo_adapter_ul_response *response) {
    return default_turbo_adapter_func(request, response);
}

void bblib_print_rate_match_version() {
    static bool was_executed = false;
    if (!was_executed) {
        was_executed = true;
        char version[BBLIB_SDK_VERSION_STRING_MAX_LEN] = { };
        bblib_rate_match_version(version, sizeof(version));
        printf("%s\n", version);
    }
}
