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
    \file   phy_LDPC_ratematch_5gnr.h
    \brief  Source code of External API for 5GNR LDPC ratematch functions

    __Overview:__

    The bblib_LDPC_ratematch_5gnr kernel is a 5G NR LDPC ratematch function.
    It is implemeneted as defined in TS38212 5.4.2

    __Requirements and Test Coverage:__

    BaseGraph 1/2; Rvidx 0-3; Modulation Type BPSK/QPSK/16QAM/64QAM/256QAM

    __Algorithm Guidance:__

    The algorithm is implemented as defined in TS38212 5.4.2.
*/

#ifndef _PHY_LDPC_RATEMATCH_5GNR_H_
#define _PHY_LDPC_RATEMATCH_5GNR_H_

#include <stdint.h>

#include "common_typedef_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
    \struct bblib_LDPC_ratematch_5gnr_request
    \brief Structure for input parameters in API of rate matching for 5GNR.
    \note input data alignment depends on modulation type.\n
          For BPSK, no alignment requirement.\n
          For QPSK, input should be aligned with 2 BITS.\n
          For 16QAM, input should be aligned with 4 BITS.\n
          For 64QAM, input should be aligned with 6 BITS.\n
          For 256QAM, input should be aligned with 1 Byte.\n
*/
struct bblib_LDPC_ratematch_5gnr_request {
    int32_t Ncb; /*!< Length of the circular buffer in bits. */

    int32_t Zc; /*!< Parameter defined in TS 38211-5.2.1. */

    int32_t E; /*!< Length of the output buffer in bits. Currently limited to 8448*8 bits */

    int32_t Qm; /*!< Modulation type, which can be 1/2/4/6/8. */

    int32_t rvidx; /*!< Redundancy version, which can be 0/1/2/3. */

    int32_t baseGraph; /*!< Base graph, which can be 1/2. */

    int32_t nullIndex; /*!< Position of starting null bits. -1 if no null bit */

    int32_t nLen; /*!< Length of null bits. 0 if no null bit */

    uint8_t *input; /*!< pointer to input stream. alignment depends on modulation type */
};

/*!
    \struct bblib_LDPC_ratematch_5gnr_response
    \brief structure for outputs of rate matching for 5GNR.
    \note output data alignment depends on modulation type.
          For BPSK, no alignment requirement
          For QPSK, input should be aligned with 2 BITS
          For 16QAM, input should be aligned with 4 BITS
          For 64QAM, input should be aligned with 6 BITS
          For 256QAM, input should be aligned with 1 Byte
 */
struct bblib_LDPC_ratematch_5gnr_response {
    uint8_t *output; /*!< Output buffer for data stream after rate matching. alignment depends on modulation type */
};

//! @{
/*! \brief rate matching for LDPC in 5GNR.
    \param [in] request Structure containing configuration information and input data.
    \param [out] response Structure containing kernel outputs.
    \note bblib_LDPC_ratematch_5gnr provides the most appropriate version for the available ISA,
          the _avx512 etc. version allow direct access to specific ISA implementations.
    \return Success: return 0, else: return -1.
*/
int32_t bblib_LDPC_ratematch_5gnr(const struct bblib_LDPC_ratematch_5gnr_request *request, struct bblib_LDPC_ratematch_5gnr_response *response);
int32_t bblib_LDPC_ratematch_5gnr_avx512(const struct bblib_LDPC_ratematch_5gnr_request *request, struct bblib_LDPC_ratematch_5gnr_response *response);
//! @}

/*! \brief Report the version number for the rate match library.
 */
void bblib_print_LDPC_ratematch_5gnr_version(void);

#ifdef __cplusplus
}
#endif

#endif
