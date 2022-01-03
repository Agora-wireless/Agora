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
 * @file   bit_reverse.cpp
 * @brief  Source code of bit reverse.
*/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <malloc.h>

#include "common_typedef_sdk.h"
#include "bit_reverse.h"


int16_t bblib_bit_reverse(int8_t* output, int32_t num_data)
{
#if defined(_BBLIB_AVX512_)
    bblib_bit_reverse_avx512(output, num_data);
    return 0;
#elif defined(_BBLIB_AVX2_)
    bblib_bit_reverse_avx2(output, num_data);
    return 0;
#else
    bblib_bit_reverse_c(output, num_data);
    return 0;
#endif
}

