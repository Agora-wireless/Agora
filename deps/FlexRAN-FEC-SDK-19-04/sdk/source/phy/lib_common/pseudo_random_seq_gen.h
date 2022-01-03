// INTEL CONFIDENTIAL
// Copyright 2009-2019 Intel Corporation All Rights Reserved.
// 
// The source code contained or described herein and all documents related to the
// source code ("Material") are owned by Intel Corporation or its suppliers or
// licensors. Title to the Material remains with Intel Corporation or its
// suppliers and licensors. The Material may contain trade secrets and proprietary
// and confidential information of Intel Corporation and its suppliers and
// licensors, and is protected by worldwide copyright and trade secret laws and
// treaty provisions. No part of the Material may be used, copied, reproduced,
// modified, published, uploaded, posted, transmitted, distributed, or disclosed
// in any way without Intel's prior express written permission.
// 
// No license under any patent, copyright, trade secret or other intellectual
// property right is granted to or conferred upon you by disclosure or delivery
// of the Materials, either expressly, by implication, inducement, estoppel or
// otherwise. Any license under such intellectual property rights must be
// express and approved by Intel in writing.
// 
// Unless otherwise agreed by Intel in writing, you may not remove or alter this
// notice or any other notice embedded in Materials by Intel or Intel's suppliers
// or licensors in any way.
// 
//  version: SDK-jenkins-FlexRAN-SDK-REL-448-g3be238

#ifndef _PSEUDO_RANDOM_SEQ_GEN_
#define _PSEUDO_RANDOM_SEQ_GEN_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*!
    \enum bblib_prbs_parameters.
    \brief Pseudo random sequence generation parameters
*/
enum bblib_prbs_parameters
{
    k_maxSymsSlot = 14,  /*!< maximum symbols per slot */

    k_maxSubCarr = 3300, /*!< maximum number of subcarriers */

    /*! Assume packed into 8-bit words */
    k_maxLengthPrs = (k_maxSymsSlot * k_maxSubCarr)>>3 + 1 };

/*!
    \struct bblib_prbs_request.
    \brief Request structure for Pseudo random sequence generation
*/
struct bblib_prbs_request
{
    /*! cinit value- see TS38.211 5.2 for more details */
    uint32_t c_init;

    /*! Number of bits to output */
    uint16_t num_bits;

    /*! Gold code- Fixed for 4G and NR at 1600 */
    uint16_t gold_code_advance;
};

/*!
    \struct bblib_prbs_response.
    \brief Response structure for Pseudo random sequence generation
*/
struct bblib_prbs_response
{
    /*! Number of bits in the output sequence */
    uint16_t num_bits;

    /*! Output bit sequence of size num_bits (as defined in request),
        should be 64 byte aligned */
    uint8_t *bits;
};

/*!
    \brief Pseudo-random sequence generation as defined in TS38.211 section 5.2
    \param [in]     Request structure containing
    \param [out] Response structure containing
 */
void bblib_prbs_basic (const struct bblib_prbs_request* request, struct bblib_prbs_response* response);

#ifdef __cplusplus
}
#endif

#endif // _PSEUDO_RANDOM_SEQ_GEN_
