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
 * @file   phy_crc.cpp
 * @brief  Source code of External API for LTE CRC24A/CRC24B and the corresponding
 *         crc check functions
*/

#include <stdio.h>
#include <stdint.h>
#include <cstdlib>
#include <functional>

#include "phy_crc.h"
#include "phy_crc_internal.h"
#include "sdk_version.h"

typedef void (*crc_function)(bblib_crc_request *request, bblib_crc_response *response);

struct bblib_lte_crc_init
{
    bblib_lte_crc_init()
    {
#if !defined(_BBLIB_SSE4_2_)
        printf("__func__ bblib_init_crc cannot run with this CPU type, needs SSE\n");
        exit(-1);
#endif
    }
};

bblib_lte_crc_init do_constructor_crc;

void
bblib_print_crc_version()
{
    static bool was_executed = false;
    if(!was_executed) {
        was_executed = true;
        char version[BBLIB_SDK_VERSION_STRING_MAX_LEN] = { };
        bblib_lte_crc_version(version, sizeof(version));
        printf("%s\n", version);
    }
}

int16_t
bblib_lte_crc_version(char *version, int buffer_size)
{
    /* The version string will be updated before the build process starts  by the
     *       jobs building the library and/or preparing the release packages.
     *       Do not edit the version string manually */
    const char *msg = "FlexRAN SDK bblib_lte_crc version sdk-19.04-ea1-1-g3be2380";

    return(bblib_sdk_version(&version, &msg, buffer_size));
}


static crc_function
bblib_crc24a_gen_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc24a_gen_avx512;
#else
    return bblib_lte_crc24a_gen_sse;
#endif
}

static crc_function default_crc24a_gen = bblib_crc24a_gen_select_on_isa();

void bblib_lte_crc24a_gen(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc24a_gen(request, response);
}


static crc_function
bblib_crc24a_check_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc24a_check_avx512;
#else
    return bblib_lte_crc24a_check_sse;
#endif
}

static crc_function default_crc24a_check = bblib_crc24a_check_select_on_isa();

void bblib_lte_crc24a_check(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc24a_check(request, response);
}


static crc_function
bblib_crc24b_gen_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc24b_gen_avx512;
#else
    return bblib_lte_crc24b_gen_sse;
#endif
}

static crc_function default_crc24b_gen = bblib_crc24b_gen_select_on_isa();

void bblib_lte_crc24b_gen(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc24b_gen(request, response);
}


static crc_function
bblib_crc24b_check_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc24b_check_avx512;
#else
    return bblib_lte_crc24b_check_sse;
#endif
}

static crc_function default_crc24b_check = bblib_crc24b_check_select_on_isa();

void bblib_lte_crc24b_check(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc24b_check(request, response);
}


static crc_function
bblib_crc24c_gen_select_on_isa() {
    return bblib_lte_crc24c_gen_avx512;
}

static crc_function default_crc24c_gen = bblib_crc24c_gen_select_on_isa();

void bblib_lte_crc24c_gen(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc24c_gen(request, response);
}


static crc_function
bblib_crc24c_check_select_on_isa() {
    return bblib_lte_crc24c_check_avx512;
}

static crc_function default_crc24c_check = bblib_crc24c_check_select_on_isa();

void bblib_lte_crc24c_check(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc24c_check(request, response);
}


static crc_function
bblib_crc24c_1_gen_select_on_isa() {
    return bblib_lte_crc24c_1_gen_avx512;
}

static crc_function default_crc24c_1_gen = bblib_crc24c_1_gen_select_on_isa();

void bblib_lte_crc24c_1_gen(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc24c_1_gen(request, response);
}


static crc_function
bblib_crc24c_1_check_select_on_isa() {
    return bblib_lte_crc24c_1_check_avx512;
}

static crc_function default_crc24c_1_check = bblib_crc24c_1_check_select_on_isa();

void bblib_lte_crc24c_1_check(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc24c_1_check(request, response);
}


static crc_function
bblib_crc16_gen_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc16_gen_avx512;
#else
    return bblib_lte_crc16_gen_sse;
#endif
}

static crc_function default_crc16_gen = bblib_crc16_gen_select_on_isa();

void bblib_lte_crc16_gen(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc16_gen(request, response);
}


static crc_function
bblib_crc16_check_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc16_check_avx512;
#else
    return bblib_lte_crc16_check_sse;
#endif
}

static crc_function default_crc16_check = bblib_crc16_check_select_on_isa();

void bblib_lte_crc16_check(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc16_check(request, response);
}


static crc_function
bblib_crc11_gen_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc11_gen_avx512;
#else
    return bblib_lte_crc11_gen_sse;
#endif
}

static crc_function default_crc11_gen = bblib_crc11_gen_select_on_isa();

void bblib_lte_crc11_gen(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc11_gen(request, response);
}


static crc_function
bblib_crc11_check_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc11_check_avx512;
#else
    return bblib_lte_crc11_check_sse;
#endif
}

static crc_function default_crc11_check = bblib_crc11_check_select_on_isa();

void bblib_lte_crc11_check(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc11_check(request, response);
}


static crc_function
bblib_crc6_gen_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc6_gen_avx512;
#else
    return bblib_lte_crc6_gen_sse;
#endif
}

static crc_function default_crc6_gen = bblib_crc6_gen_select_on_isa();

void bblib_lte_crc6_gen(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc6_gen(request, response);
}


static crc_function
bblib_crc6_check_select_on_isa() {
#ifdef _BBLIB_AVX512_
    return bblib_lte_crc6_check_avx512;
#else
    return bblib_lte_crc6_check_sse;
#endif
}

static crc_function default_crc6_check = bblib_crc6_check_select_on_isa();

void bblib_lte_crc6_check(struct bblib_crc_request *request, struct bblib_crc_response *response)
{
    default_crc6_check(request, response);
}
