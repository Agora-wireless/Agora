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
    \file   phy_crc.h
    \brief  External API for lib_crc, which comprises of CRC generate and CRC validate functions
     for the following CRC algorithms as specified in 3GPP TS 38.212 v15.1.1:
     CRC24A, CRC24B, CRC24C, CRC16, CRC11 & CRC6, all initialised with zeros, and CRC24C
     initialised with ones as specified in 3GPP TS 38.212 section 7.3.2.

     The CRC generate function (bblib_lte_<algorithm>_gen) is used to calculate the CRC value
     based on the sequence of input data and data length, in bits passed to the function in the
     request structure. The CRC value is then appended to the end of the input data sequence and
     available in the response structure. Due to the nature of the algorithms being byte based,
     maximum performance is obtained when input data length is in multiples of 8 bits (ie. bytes),
     since padding of the data isn't required.

     The CRC validate function (bblib_lte_<algorithm>_check) is used to validate input data that
     already contains a CRC appended to the end. It calculates a CRC value and then compares that
     value to the one at the end of the data. The result (pass or fail) is indicated in the response
     structure.

     Testing:
     Each CRC algorithm's generate & validate function is tested over a range of test vectors from 1 to
     65536 bits, with both multiples and non-multiples of 8 bits.
     A series of performance tests have also been defined generally based on 2344 & 2340 bit test vectors.
*/

#ifndef _PHY_CRC_H_
#define _PHY_CRC_H_

#include <stdint.h>
#include <stdbool.h>

#include "common_typedef_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
    \struct bblib_crc_request
    \brief Request structure containing pointer to input data sequence and its length (in bits).
*/
struct bblib_crc_request {
    uint8_t *data; /*!< Pointer to input data sequence for the CRC generator or CRC validate functions.
                        Input data should be byte aligned in memory*/


    uint32_t len;  /*!< The length of input data, in bits.*/
};

/*!
    \struct bblib_crc_response
    \brief Response structure containing pointer to input data with appended CRC, new data length, CRC value
     and result of a CRC validate function.
*/
struct bblib_crc_response {
    uint8_t *data;      /*!< Pointer to output data comprising of the input data with appended CRC value.*/

    uint32_t len;       /*!< The length of output data including the CRC value, in bits.*/

    uint32_t crc_value; /*!< The calculated CRC value rounded to whole bytes,by appending zeros.
                             Example: CRC11 bit binary value of 10010110001 (0x962) would be rounded to
                                      the nearest byte therefore becomes 010010110001 (0x4b1)*/

    bool check_passed;  /*!< Result of CRC Check Function, where true = passed, false = failed. */

};



/*! \brief Report the version number for the bblib_lte_crc library
    \param [in] version Pointer to a char buffer where the version string should be copied.
    \param [in] buffer_size The length of the string buffer, must be at least
           BBLIB_SDK_VERSION_STRING_MAX_LEN characters.
    \return 0 if the version string was populated, otherwise -1.
*/
int16_t
bblib_lte_crc_version(char *version, int buffer_size);

//! @{
/*! \brief Performs CRC24A generate, calculating the CRC value and appending to the data.
    \param [in] request structure containing pointer to input data and data length.
    \param [out] response structure containing calculated CRC value, CRC appended data and new length.
    \return void
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note  CRC polynomial is "D24 + D23 + D18 + D17 + D14 + D11 + D10 + D7 + D6 + D5
           + D4 + D3 + D + 1", refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc24a_gen(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24a_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24a_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}

//! @{
/*!
    \brief Performs CRC24A validate, indicating if the input data sequence has a valid CRC value.
    \param [in] request structure containing pointer to input data and data length.
    \note  Length should be for the data part only, since the CRC algorithm determines the CRC length.
    \param [out] response structure with indication if CRC validation has passed.
    \return void
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note CRC polynomial is "D24 + D23 + D18 + D17 + D14 + D11 + D10 + D7 + D6 + D5 + D4 + D3
          + D + 1", refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc24a_check(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24a_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24a_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}

//! @{
/*!
    \brief Performs CRC24B generate, calculating the CRC value and appending to the data.
    \param [in] request structure containing pointer to input data and data length.
    \param [out] response structure containing calculated CRC value, CRC appended data and new length.
    \return void.
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note CRC polynomial is "D24 + D23 + D6 + D5 + D + 1",  refer to 3GPP TS 38.212, section 5.1.

*/
void bblib_lte_crc24b_gen(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24b_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24b_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}

//! @{
/*!
    \brief Performs CRC24B validate, indicating if the input data sequence has a valid CRC value.
    \param [in] request structure containing pointer to input data and data length.
    \note  Length should be for the data part only, since the CRC algorithm determines the CRC length.
    \param [out] response structure with indication if CRC validation has passed.
    \return void
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note CRC polynomial is "D24 + D23 + D6 + D5 + D + 1",  refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc24b_check(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24b_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24b_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}

//! @{
/*! \brief Performs CRC24C generate, calculating the CRC value and appending to the data.
    \param [in] request structure containing pointer to input data and data length.
    \param [out] response structure containing calculated CRC value, CRC appended data and new length.
    \return void.
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note  CRC polynomial is "D24 + D23 + D21 + D20 + D17 + D15 + D13 + D12 + D8 + D4
           + D2 + D + 1", refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc24c_gen(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24c_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

//! @}

//! @{
/*!
    \brief Performs CRC24C validate, indicating if the input data sequence has a valid CRC value.
    \param [in] request structure containing pointer to input data and data length.
    \note  Length should be for the data part only, since the CRC algorithm determines the CRC length.
    \param [out] response structure with indication if CRC validation has passed.
    \return void
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note  CRC polynomial is "D24 + D23 + D21 + D20 + D17 + D15 + D13 + D12 + D8 + D4
           + D2 + D + 1", refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc24c_check(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24c_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

//! @}

//! @{
/*! \brief Performs CRC24C initialised with 1s, generate, calculating the CRC value and appending to the data.
    \param [in] request structure containing pointer to input data and data length.
    \param [out] response structure containing calculated CRC value, CRC appended data and new length.
    \return void.
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note  CRC polynomial is "D24 + D23 + D21 + D20 + D17 + D15 + D13 + D12 + D8 + D4
           + D2 + D + 1", refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc24c_1_gen(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24c_1_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

//! @}

//! @{
/*!
    \brief Performs CRC24C initialised with 1s, validate, indicating if the input data sequence has a valid CRC value.
    \param [in] request structure containing pointer to input data and data length.
    \note  Length should be for the data part only, since the CRC algorithm determines the CRC length.
    \param [out] response structure with indication if CRC validation has passed.
    \return void
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note  CRC polynomial is "D24 + D23 + D21 + D20 + D17 + D15 + D13 + D12 + D8 + D4
           + D2 + D + 1", refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc24c_1_check(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc24c_1_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

//! @}

//! @{
/*!
    \brief Performs CRC16 generate, calculating the CRC value and appending to the data.
    \param [in] request structure containing pointer to input data and data length.
    \param [out] response structure containing calculated CRC value, CRC appended data and new length.
    \return void.
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note CRC polynomial is "D16 + D12 + D5 + 1",  refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc16_gen(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc16_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc16_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}

//! @{
/*!
    \brief Performs CRC16 validate, indicating if the input data sequence has a valid CRC value.
    \param [in] request structure containing pointer to input data and data length.
    \note  Length should be for the data part only, since the CRC algorithm determines the CRC length.
    \param [out] response structure with indication if CRC validation has passed.
    \return void.
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note CRC polynomial is "D16 + D12 + D5 + 1",  refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc16_check(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc16_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc16_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}


//! @{
/*!
    \brief Performs CRC11 generate, calculating the CRC value and appending to the data.
    \param [in] request structure containing pointer to input data and data length.
    \param [out] response structure containing calculated CRC value, CRC appended data and new length.
    \return void
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note CRC polynomial is "D11 + D10 + D9 + D5 + 1",  refer to 3GPP TS 38.212, section 5.1.
*/
void bblib_lte_crc11_gen(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc11_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc11_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}

//! @{
/*!
    \brief Performs CRC11 validate, indicating if the input data sequence has a valid CRC value.
    \param [in] request structure containing pointer to input data and data length.
    \note  Length should be for the data part only, since the CRC algorithm determines the CRC length.
    \param [out] response structure with indication if CRC validation has passed.
    \return void.
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note CRC polynomial is "D11 + D10 + D9 + D5 + 1",  refer to 3GPP TS 38.212, section 5.1.

*/
void bblib_lte_crc11_check(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc11_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc11_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}


//! @{
/*!
    \brief Performs CRC6 generate, calculating the CRC value and appending to the data.
    \param [in] request structure containing pointer to input data and data length.
    \param [out] response structure containing calculated CRC value, CRC appended data and new length.
    \return void
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note CRC polynomial is "D6 + D5 + 1",  refer to 3GPP TS 38.212.
*/
void bblib_lte_crc6_gen(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc6_gen_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc6_gen_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}

//! @{
/*!
    \brief Performs CRC6 validate, indicating if the input data sequence has a valid CRC value.
    \param [in] request structure containing pointer to input data and data length.
    \note  Length should be for the data part only, since the CRC algorithm determines the CRC length.
    \param [out] response structure with indication if CRC validation has passed.
    \return void.
    \note  Memory for both request.data & response.data structures should be allocated.
     Response.data can point to request.data if sufficient space is available at end of
     request.data structure for the appended CRC.
    \note CRC polynomial is "D6 + D5 + 1",  refer to 3GPP TS 38.212.
*/
void bblib_lte_crc6_check(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc6_check_avx512(struct bblib_crc_request *request, struct bblib_crc_response *response);

void bblib_lte_crc6_check_sse(struct bblib_crc_request *request, struct bblib_crc_response *response);
//! @}



#ifdef __cplusplus
}
#endif

#endif
