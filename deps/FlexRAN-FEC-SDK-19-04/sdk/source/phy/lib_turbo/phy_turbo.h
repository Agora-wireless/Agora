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
    \file   phy_turbo.h
    \brief  External API for LTE turbo coder/decoder.
*/

#ifndef _PHY_TURBO_H_
#define _PHY_TURBO_H_

#include <stdint.h>
#include "common_typedef_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
    \struct bblib_turbo_decoder_request
    \brief Request structure for turbo decoder.
*/
struct bblib_turbo_decoder_request {

    int32_t c; /*!< C index value of codeblock for TB.

                    Its element has 1 to 1 mapping relationship with TBS_L1. */

    int32_t k; /*!< K index value of codeblock for each TB.

                    Its element has 1 to 1 mapping relationship with TBS_L1. */

    int32_t k_idx; /*!< Size index in TS.36.212, table 5.1.3-3 of codeblock for each TB.

                        Its element has 1 to 1 mapping relationship with TBS_L1. */

    int32_t max_iter_num; /*!< Maximum number of decoder iterations */

    int32_t early_term_disable; /*!< If set to 1, then max_iter_num is always used regardless of CRC check pass / fail. If 0, least number of iterations are used (1 <= iter <= max_iter_num) for decoding till crc check is pass */

    int8_t *input; /*!< Input buffer must be 64 bytes aligned*/
};

/*!
    \struct bblib_turbo_decoder_response
    \brief Response structure for turbo decoder.
*/
struct bblib_turbo_decoder_response {

    uint8_t *output; /*!< Output buffer must be 64 bytes aligned. */

    int8_t  *ag_buf; /*!< Alfa-gamma buffer to be used for internal calculations.

                          The expected buffer length is 6528*16 bytes. */

    uint16_t *cb_buf; /*!< Code block bits buffer used for internal calculations.

                           The expected buffer length is K/8. */
};


/*!
    \struct bblib_turbo_encoder_request
    \brief Request structure for turbo encoder.
*/
struct bblib_turbo_encoder_request {

    uint32_t length; /*!<Length of the input in bytes. */

    uint8_t case_id; /*!< Index of the internal interleaver parameters case index - TS 36.212,
                          Table 5.1.3-3, column 'i'. */

    uint8_t *input_win; /*!< Information and CRC bits buffer. */
};


/*!
    \struct bblib_turbo_encoder_response
    \brief Response structure for turbo encoder.
*/
struct bblib_turbo_encoder_response {
    uint8_t *output_win_0; /*!< Layer 0 bits buffer. */

    uint8_t *output_win_1; /*!< Layer 1 bits buffer. */

    uint8_t *output_win_2; /*!< Layer 2 bits buffer. */
};

/*! \brief Report the version number for the bblib_lte_turbo library
    \param [in] version Pointer to a char buffer where the version string should
            be copied.
    \param [in] buffer_size The length of the string buffer, has to be at least
            BBLIB_SDK_VERSION_STRING_MAX_LEN characters.
    \return 0 if the version string was populated, otherwise -1.
*/
int16_t
bblib_lte_turbo_version(char *version, int buffer_size);

//! @{
/*! \brief Turbo encoder implementation as defined in TS.36.212.
    \param request Input data container.
    \param response Output data container.
    \return 0 on success, non-zero on failure.
 */
int32_t
bblib_turbo_encoder(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response);

int32_t bblib_lte_turbo_encoder_sse(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response);
int32_t  bblib_lte_turbo_encoder_avx2(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response);
int32_t  bblib_lte_turbo_encoder_avx512(const struct bblib_turbo_encoder_request *request,
    struct bblib_turbo_encoder_response *response);
//! @}

//! @{
/*! \brief Turbo decoder implementation for different windows sizes as defined in TS.36.212.
    \param request Input data container.
    \param response Output data container.
    \return Number of half iterations on success, negative on failure
*/
int32_t
bblib_turbo_decoder(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response);

int32_t bblib_lte_turbo_decoder_64windows_avx512(const struct bblib_turbo_decoder_request *request,
                                                 struct bblib_turbo_decoder_response *response);
int32_t bblib_lte_turbo_decoder_32windows_avx2(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response);
int32_t bblib_lte_turbo_decoder_16windows_sse(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response);
int32_t bblib_lte_turbo_decoder_16windows_3iteration_sse(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response);
int32_t bblib_lte_turbo_decoder_8windows_sse(const struct bblib_turbo_decoder_request *request,
    struct bblib_turbo_decoder_response *response);
//! @}

#ifdef __cplusplus
}
#endif

#endif
