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
/*!
    \file   phy_ldpc_encoder_5gnr.h
    \brief  Source code of External API for 5GNR LDPC Encoder functions

    __Overview:__

    The bblib_ldpc_encoder_5gnr kernel is a 5G NR LDPC Encoder function.
    It is implemented as defined in TS38212 5.3.2

    __Requirements and Test Coverage:__

    BaseGraph 1(.2). Lifting Factor >176.

    __Algorithm Guidance:__

    The algorithm is implemented as defined in TS38212 5.3.2.
*/

#ifndef _PHY_LDPC_ENCODER_5GNR_H_
#define _PHY_LDPC_ENCODER_5GNR_H_

#include <stdint.h>

#include "common_typedef_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MAX_CB_BLOCK (32)

/*!
    \struct bblib_ldpc_encoder_5gnr_request
    \brief Structure for input parameters in API of LDPC Encoder for 5GNR.
    \note ... \n
*/
struct bblib_ldpc_encoder_5gnr_request {

    uint16_t Zc; /*!< Lifting factor Zc as defined in TS38212-5.2.1. */

    int32_t baseGraph; /*!< LDPC Base graph, which can be 1 or 2  as defined in TS38212-5.2.1. */

    int32_t nRows;  /*!< Number Rows being used for the encoding native code rate - Minimum 4 */

    int8_t numberCodeblocks;  /*!<
        Used to run several code blocks in one operation notably for low expansion factor
        All code blocks must use the same parameters above.
        numberCodeblocks * Zc must not exceed 512 */

    int8_t *input[MAX_CB_BLOCK]; /*!< 
        Pointer to input stream related to each code block
        This corresponds to the bit sequence c_k as defined in TS38.212-5.3.2.
        This includes therefore the filler bits set as 0. */
};

/*!
    \struct bblib_ldpc_encoder_5gnr_response
    \brief structure for outputs of LDPC encoder for 5GNR.
    \note
 */
struct bblib_ldpc_encoder_5gnr_response {
    int8_t *output[MAX_CB_BLOCK]; /*!< 
        Output buffer for data stream after LDPC Encoding  for each  CodeBlocks
        This corresponds to the parity bit sequence w_k as defined in TS38.212-5.3.2
        The actual length is limited to the number of rows requested.  */
};

//! @{
/*! \brief Encoder for LDPC in 5GNR.
    \param [in] request Structure containing configuration information and input data.
    \param [out] response Structure containing kernel outputs.
    \note bblib_ldpc_encoder_5gnr provides the most appropriate version for the available ISA,
          the _avx512 etc. version allow direct access to specific ISA implementations.
    \return Success: return 0, else: return -1.
*/
int32_t bblib_ldpc_encoder_5gnr(struct bblib_ldpc_encoder_5gnr_request *request, struct bblib_ldpc_encoder_5gnr_response *response);
int32_t bblib_ldpc_encoder_5gnr_avx512( struct bblib_ldpc_encoder_5gnr_request *request, struct bblib_ldpc_encoder_5gnr_response *response);
//! @}

/*! \brief Report the version number for the encoder library.
 */
void bblib_print_ldpc_encoder_5gnr_version(void);

#ifdef __cplusplus
}
#endif

#endif
