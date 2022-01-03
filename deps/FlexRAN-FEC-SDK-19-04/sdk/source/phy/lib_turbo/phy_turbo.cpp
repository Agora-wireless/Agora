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
 * @file   phy_turbo.cpp
 * @brief  External API for LTE turbo coder/decoder
*/

#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <functional>

#include "phy_turbo.h"
#include "phy_turbo_internal.h"

#include "sdk_version.h"

_TurboInterleaver g_TurboInterleaver;

/* Turbo table */
extern __align(64) uint16_t g_TurboBufAddr[1068000];
extern __align(64) int32_t g_TurboBufAddrOffset[188];

/* TS36.212 Table 5.1.3-3, Turbo code internal interleaver parameters with XXXX */
extern int32_t g_Kidx_K_Nmaxrep_shuf_sdk[188][5];

typedef int32_t (*encode_function)(const bblib_turbo_encoder_request *request,
    bblib_turbo_encoder_response *response);

struct bblib_lte_turbo_init
{
    bblib_lte_turbo_init() {
#if !defined(_BBLIB_AVX2_) && !defined(_BBLIB_SSE4_2_) && !defined(_BBLIB_AVX512_)
        printf("__func__ bblib_scramble_init() cannot run with this CPU type, needs "
                       "AVX2 or SSE\n");
        exit(-1);
#endif
        /* Get SDK root folder from environment variable */
        char *pTablePath = getenv("DIR_WIRELESS_SDK");
        if (pTablePath == NULL) {
            printf("Need to setup environment variable 'DIR_WIRELESS_SDK' for turbo,"
                           " please set it up to folder where SDK is stored, with 'export"
                           " DIR_WIRELESS_SDK=...'\n");
            exit(-1);
        }

        bblib_lte_turbo_interleaver_initTable(pTablePath);
        init_turbo_decoder_interleaver_table(pTablePath, &g_TurboInterleaver);
        init_common_tables(pTablePath);
        get_turbo_buf_addr_table_new(g_TurboBufAddr, g_TurboBufAddrOffset,
                                     g_Kidx_K_Nmaxrep_shuf_sdk);
    }
};

bblib_lte_turbo_init do_constructor_turbo;

void
bblib_print_turbo_version()
{
    static bool was_executed = false;
    if(!was_executed) {
        was_executed = true;
        char version[BBLIB_SDK_VERSION_STRING_MAX_LEN] = { };
        bblib_lte_turbo_version(version, sizeof(version));
        printf("%s\n", version);
    }
}

int16_t bblib_lte_turbo_version(char *version, int buffer_size) {
    /* The version string will be updated before the build process starts by the
     *       jobs building the library and/or preparing the release packages.
     *       Do not edit the version string manually */
    const char *msg = "FlexRAN SDK bblib_lte_turbo version sdk-19.04-ea1-1-g3be2380";

    return(bblib_sdk_version(&version, &msg, buffer_size));
}

static encode_function
bblib_encoder_select_on_isa() {
#ifdef _BBLIB_AVX512_
    // AVX512 has problems (see d73f51264838d41656679123733764ee3f80cb0c)
    return bblib_lte_turbo_encoder_avx2;
#elif defined _BBLIB_AVX2_
    return bblib_lte_turbo_encoder_avx2;
#else
    return bblib_lte_turbo_encoder_sse;
#endif
}

static encode_function default_encode = bblib_encoder_select_on_isa();

int32_t bblib_turbo_encoder(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response) {

    return default_encode(request, response);
}

int32_t bblib_turbo_decoder(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response) {
#ifdef _BBLIB_AVX512_
    if ((request->k) % 16 != 0) {
        return bblib_lte_turbo_decoder_8windows_sse(request, response);
    }
    else if(request->k % 32 != 0) {
        if (0 == request->max_iter_num)
            return bblib_lte_turbo_decoder_16windows_3iteration_sse(request, response);
        else
            return bblib_lte_turbo_decoder_16windows_sse(request, response);
    }
    else if(request->k % 64 != 0) {
        return bblib_lte_turbo_decoder_32windows_avx2(request, response);
    }
    else {
        return bblib_lte_turbo_decoder_64windows_avx512(request, response);
    }
#elif defined _BBLIB_AVX2_
    if ((request->k) % 16 != 0) {
        return bblib_lte_turbo_decoder_8windows_sse(request, response);
    }
    else if(request->k % 32 != 0) {
        if (0 == request->max_iter_num)
            return bblib_lte_turbo_decoder_16windows_3iteration_sse(request, response);
        else
            return bblib_lte_turbo_decoder_16windows_sse(request, response);
    }
    else {
        return bblib_lte_turbo_decoder_32windows_avx2(request, response);
    }
#else
    if ((request->k) % 16 != 0) {
        return bblib_lte_turbo_decoder_8windows_sse(request, response);
    }
    else {
        if (0 == request->max_iter_num)
            return bblib_lte_turbo_decoder_16windows_3iteration_sse(request, response);
        else
            return bblib_lte_turbo_decoder_16windows_sse(request, response);
    }
#endif
}
