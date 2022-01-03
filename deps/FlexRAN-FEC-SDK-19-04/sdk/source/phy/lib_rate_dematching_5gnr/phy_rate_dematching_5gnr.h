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
    \file phy_rate_dematching_5gnr.h
    \brief  External API for 5G rate dematching

    5G rate dematching module performs HARQ combine, De-Interleaver and De-Selection
    for data channel LDPC decoder.\n\n
    The lib_rate_dematching_5gnr kernel is a 5G NR Rate Dematching for LDPC code
    implemented by Harq Combine, bit de-interleaving and bit de-selection.
    For reducing the interference and matching the bearing of physical channel,
    the transfer site need to do bit collection, bit interleaving and bit repeat
    or puncture some bits in order to map to physical resource element.
    Vice-versa happens in receiver site to do bit de-interleaving,
    bit de-selection and HARQ combine for retransmission.\n\n
    It is implemented according to 3GPP TS 38.212 5.4.2 Rate matching for LDPC code
    The test coverage for this module includes BPSK, QPSK, 16QAM, 64QAM and 256QAM
*/

#ifndef _PHY_RATE_DEMATCHING_5GNR_H_
#define _PHY_RATE_DEMATCHING_5GNR_H_

#include <stdint.h>
#include "common_typedef_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LLR_VAL 127
#define MAX_LLR (LLR_VAL)
#define MIN_LLR (-LLR_VAL)

/*!
    \struct bblib_rate_dematching_5gnr_request
    \brief Request structure providing the inputs and configuration to the rate dematching.
*/
struct bblib_rate_dematching_5gnr_request {
    int8_t *p_in; /*!< the pointer of rate dematching input, non cache alignment requirement, the input symbol size is 8bits */

    int8_t *p_harq; /*!<
    The pointer of HARQ buffer for both input/output, assumed to be 64B cache aligned.
    This is also the input for the decoder with Filler bits not included
    As a consequence there is no pointer in the response structure */

    int32_t k0; /*!< k0 the start position in the circular buffer as defined in TS38212-5.4.2.1. */

    int32_t ncb; /*!<  Ncb the length of the circular buffer (including null bits) as defined in TS38212-5.4.2.1. */

    int32_t start_null_index; /*!< the start null bit position in Ncb */

    int32_t num_of_null; /*!< F The number of filler bits used in the encoding */

    int32_t e; /*!< E The number of post rate-matched output LLR values as defined in TS38212-5.4.2.1.*/

    int32_t rvid; /*!< redundancy version id as defined in TS38212-5.4.2.1. */

    int32_t zc;  /*!< Lifting factor Zc as defined in TS38212-5.2.1. */

    enum bblib_modulation_order modulation_order; /*!< modulation, the allowed values: 1, 2, 4, 6, or 8 */

    int32_t base_graph; /*!< LDPC Base graph, which can be 1 or 2  as defined in TS38212-5.2.1. */

    int32_t isretx; /*!<  flag of retransmission, 0: no retransmission, clear HARQ buffer, 1: retransmission */
};

/*!
    \struct bblib_rate_dematching_5gnr_response
    \brief Response structure which is empty - p_harq is also an output
*/
struct bblib_rate_dematching_5gnr_response {
};

//! @{
/*! \brief Implements rate dematching
    \param [in] request Structure containing the configuration, input data
    \param [out] response Structure containing the output data.
*/
void bblib_rate_dematching_5gnr(struct bblib_rate_dematching_5gnr_request *request,
    struct bblib_rate_dematching_5gnr_response *response);

void bblib_rate_dematching_5gnr_c(struct bblib_rate_dematching_5gnr_request *req,
struct bblib_rate_dematching_5gnr_response *resp);
void bblib_rate_dematching_5gnr_avx512(struct bblib_rate_dematching_5gnr_request *req,
struct bblib_rate_dematching_5gnr_response *resp);
//! @}

//! @{
/*! \brief Report the version number for the bblib_rate_dematching_5gnr library.
    \param [in] version Pointer to a char buffer where the version string should be copied.
    \param [in] buffer_size The length of the string buffer, must be at least
           BBLIB_SDK_VERSION_STRING_MAX_LEN characters.
    \return 0 when success, -1 when fail
*/
int16_t bblib_rate_dematching_5gnr_version(char *version, int buffer_size);
//! @}

//! @{
/*! \brief print out this rate dematching version for 5gnr
*/
void bblib_print_rate_dematching_5gnr_version(void);
//! @}

#ifdef __cplusplus
}
#endif

#endif
