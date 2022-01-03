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
    \file   phy_ldpc_decoder_5gnr.h
    \brief  Source code of External API for 5GNR LDPC Encoder functions

    __Overview:__

    The bblib_ldpc_decoder_5gnr kernel is a 5G NR LDPC Encoder function.
    It is implemented as defined in TS38212 5.3.2

    __Requirements and Test Coverage:__

    BaseGraph 1(.2). Lifting Factor >176.

    __Algorithm Guidance:__

    The algorithm is implemented as defined in TS38212 5.3.2.
*/

#ifndef _PHY_LDPC_DECODER_5GNR_H_
#define _PHY_LDPC_DECODER_5GNR_H_

#include <stdint.h>

#include "common_typedef_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif


/*!
    \struct bblib_ldpc_decoder_5gnr_request
    \brief Structure for input parameters in API of LDPC Decoder for 5GNR.
    \note ... \n
*/
struct bblib_ldpc_decoder_5gnr_request {

    uint16_t Zc; /*!< Lifting factor Zc as defined in TS38212-5.2.1. */

    int32_t baseGraph; /*!< LDPC Base graph, which can be 1 or 2 as defined in TS38212-5.2.1. */

    int32_t nRows;  /*!< Number Rows in the LDPC being used for the encoding native code rate - Minimum 4 */

    int8_t* varNodes;
    /*!<
        Pointer to the buffer used to store the code word 8-bit integer LLRs into the
        top-level of the decoder to each code block
        This corresponds to the bit sequence d_k as defined in TS38.212-5.3.2.
        This should be z*22 + z*nRows - z*2 - numFillerBits in length for BG1
        This should be z*10 + z*nRows - z*2 - numFillerBits in length for BG2
        The filler bits NULL are not included
     */

    int16_t numChannelLlrs; /*!<The number of post rate-matched output LLR values*/

    int16_t numFillerBits; /*!< The number of filler bits used in the encoding */

    int16_t maxIterations;
    /*!<
     The maximum number of iterations that the decoder will perform before
     it is forced to terminate
     */

    bool enableEarlyTermination;
    /*!<
    When true, the decoder is allowed to terminate before maxIterations
    if the parity-check equations all pass
     */
};

/*!
    \struct bblib_ldpc_decoder_5gnr_response
    \brief structure for outputs of LDPC decoder for 5GNR.
    \note
 */
struct bblib_ldpc_decoder_5gnr_response {

    int16_t* varNodes;
    /*!<
     Pointer to the buffer used to store the code word 16-bit LLR outputs
      Space allocation *must* be  >= z*22 + z*nRows for BG1
      Space allocation *must* be  >= z*10 + z*nRows for BG2
     */

    int numMsgBits;    /*!<
     Output message stored as individual bits in a pre-allocated buffer.
     Number of bytes allocation *must* be  >= ceil((z*22 - numFillerBits)/8) for BG1
     Number of bytes allocation *must* be  >= ceil((z*10 - numFillerBits)/8) for BG2
    */

    uint8_t* compactedMessageBytes; /*!< Decoded Message Data */

    int iterationAtTermination; /*!< The number of iterations executed before termination. */

    bool parityPassedAtTermination;/*!<
      True if the parity checks all had passed at termination  (Always true if
      response.iterationAtTermination  < request.maxIterations).
     */

};

//! @{
/*! \brief Encoder for LDPC in 5GNR.
    \param [in] request Structure containing configuration information and input data.
    \param [out] response Structure containing kernel outputs.
    \note bblib_ldpc_decoder_5gnr provides the most appropriate version for the available ISA,
          the _avx512 etc. version allow direct access to specific ISA implementations.
    \return Success: return 0, else: return -1.
*/
int32_t bblib_ldpc_decoder_5gnr(struct bblib_ldpc_decoder_5gnr_request *request, struct bblib_ldpc_decoder_5gnr_response *response);
int32_t bblib_ldpc_decoder_5gnr_avx2( struct bblib_ldpc_decoder_5gnr_request *request, struct bblib_ldpc_decoder_5gnr_response *response);
int32_t bblib_ldpc_decoder_5gnr_avx512( struct bblib_ldpc_decoder_5gnr_request *request, struct bblib_ldpc_decoder_5gnr_response *response);
//! @}

/*! \brief Report the version number for the decoder library.
 */
void bblib_print_ldpc_decoder_5gnr_version(void);

#ifdef __cplusplus
}
#endif

#endif
