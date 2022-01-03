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
/*! @file   bit_reverse.h
    @brief  Source code of conversion between float and int16, with agc gain.
*/

#ifndef _BIT_REVERSE_H_
#define _BIT_REVERSE_H_

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <malloc.h>
#include <stdint.h>
#include <immintrin.h>

#ifdef __cplusplus
extern "C" {
#endif

//! @{
/*! \brief Bit Reversion.
    \param [out] inout Input and Output buffer
    \param [in] num_data Number of data in bits for conversion.
    \return Return 0 for success, and -1 for error.
    \note Input and output is aligned with 512 bits.
*/
int16_t
bblib_bit_reverse(int8_t* inout, int32_t num_data);

void bblib_bit_reverse_avx2(int8_t* inout, int32_t num_data);
void bblib_bit_reverse_c(int8_t* inout, int32_t num_data);
void bblib_bit_reverse_avx512(int8_t* inout, int32_t num_data);
//! @}



#ifdef __cplusplus
}
#endif

#endif  /* _BIT_REVERSE_H_ */
