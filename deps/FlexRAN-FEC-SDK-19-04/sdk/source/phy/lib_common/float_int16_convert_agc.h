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
/*! @file   float_int16_convert_agc.h
    @brief  Source code of conversion between float and int16, with agc gain.
*/

#ifndef _FLOAT_INT16_CONVERT_AGC_H_
#define _FLOAT_INT16_CONVERT_AGC_H_

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <malloc.h>

#include <immintrin.h>
#include "common_typedef_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

//! @{
/*! \brief Conversion from float to int16, with float gain.
    \param [in] input Input buffer for float.
    \param [in] num_data Number of data for conversion.
    \param [in] gain Gain for agc.
    \param [out] output Output buffer for int16.
    \return Return 0 for success, and -1 for error.
    \note Input and output is aligned with 512 bits.
    \note Also (input data*gain) should be in the rage of -32768~32767, for the range of int16.
*/
int16_t
bblib_float_to_int16_agc(int16_t* output, float* input, int32_t num_data, float gain);

void bblib_float_to_int16_agc_avx2(int16_t* output, float* input, int32_t num_data, float gain);
void bblib_float_to_int16_agc_c(int16_t* output, float* input, int32_t num_data, float gain);
void bblib_float_to_int16_agc_avx512(int16_t* output, float* input, int32_t num_data, float gain);
//! @}

//! @{
/*! \brief Conversion from float to int16, with float gain and int16 threshold.
    \param [in] input Input buffer for float.
    \param [in] num_data Number of data for conversion.
    \param [in] gain Gain for agc.
    \param [in] threshold Threshold after agc, which should be >=0.
    \param [out] output Output buffer for int16.
    \return Return 0 for success, and -1 for error.
    \note Input and output is aligned with 512 bits.
 */
int16_t
bblib_float_to_int16_agc_threshold(int16_t* output, float* input, int32_t num_data,
    float gain, int16_t threshold);

int16_t bblib_float_to_int16_agc_threshold_avx2(int16_t* output, float* input, int32_t num_data,
    float gain, int16_t threshold);
int16_t bblib_float_to_int16_agc_threshold_c(int16_t* output, float* input, int32_t num_data,
    float gain, int16_t threshold);
int16_t bblib_float_to_int16_agc_threshold_avx512(int16_t* output, float* input, int32_t num_data,
    float gain, int16_t threshold);
//! @}

//! @{
/*! \brief Conversion from int16 to float, with float gain.
    \param [in] input Input buffer for int16.
    \param [in] num_data Number of data for conversion.
    \param [in] gain Gain for agc.
    \param [out] output Output buffer for float.
    \return Return 0 for success, and -1 for error.
    \note Input and output is aligned with 512 bits.
*/
int16_t
bblib_int16_to_float_agc(float* output, int16_t* input, int32_t num_data, float gain);

void bblib_int16_to_float_agc_avx2(float* output, int16_t* input, int32_t num_data, float gain);
void bblib_int16_to_float_agc_c(float* output, int16_t* input, int32_t num_data, float gain);
void bblib_int16_to_float_agc_avx512(float* output, int16_t* input, int32_t num_data, float gain);
//! @}

//! @{
/*! \brief Conversion from int16 to int16, with float gain.
    \param [in] input Input buffer for int16.
    \param [in] num_data Number of data for conversion.
    \param [in] gain Gain for agc.
    \param [out] output Output buffer for int16.
    \return Return 0 for success, and -1 for error.
    \note Input and output is aligned with 512 bits.
*/
int16_t
bblib_int16_to_int16_agc(int16_t* output, int16_t* input, int32_t num_data, float gain);

void bblib_int16_to_int16_agc_avx2(int16_t* output, int16_t* input, int32_t num_data, float gain);
void bblib_int16_to_int16_agc_c(int16_t* output, int16_t* input, int32_t num_data, float gain);
void bblib_int16_to_int16_agc_avx512(int16_t* output, int16_t* input, int32_t num_data, float gain);
//! @}

//! @{
/*! \brief add fxp scale to int16
    \param[in] scaleIn scaling input
    \param[in] num_samples number of samples
    \param[in] scale16 scaling in int16
    \param[out] scaleOut scaling output
    \return none
*/
int16_t
bblib_int16_to_int16_fxp_scale(int16_t *scaleOut, int16_t *scaleIn, int32_t num_samples, int16_t scale16);

void bblib_int16_to_int16_fxp_scale_c(int16_t *scaleOut, int16_t *scaleIn, int32_t num_samples, int16_t scale16);
void bblib_int16_to_int16_fxp_scale_avx512(int16_t *scaleOut, int16_t *scaleIn, int32_t num_samples, int16_t scale16);
//! @}

#ifdef __cplusplus
}
#endif

#endif  /* _FLOAT_INT16_CONVERT_AGC_H_ */
