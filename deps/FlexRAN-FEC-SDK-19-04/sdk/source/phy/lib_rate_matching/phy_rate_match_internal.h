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
 * @file
 * @brief  Internal APIs
*/

#ifndef _PHY_RATE_MATCH_INTERNAL_H_
#define _PHY_RATE_MATCH_INTERNAL_H_

#include <cstdint>

#include "divide.h"

#ifndef MLIMIT
#define MLIMIT    (8)
#endif

#ifndef MIN
#define MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX_VALUE
#define MAX_VALUE (16)
#endif

#ifndef MIN_VALUE
#define MIN_VALUE (-16)
#endif

/* It is derived from TS 36.212 5.1.2 */
#define MAX_CODE_BLOCK_IN_ONE_TB (25)


extern int32_t g_nNum_NULL[188];
extern int32_t g_nIndex_NULL[188][84];
/* rate matching table */
extern int32_t g_ratetable[188][18444];

/* Rate matching bit to byte table */
__declspec (align(64)) extern uint8_t g_BitToByteTABLE[8192];

/**
 * @brief Initialize LTE rate matching with SSE instructions, read some files into global tables.
 * @return 0: init success, -1: init error.
 */
int32_t init_rate_matching_lte_sse();

/**
 * @brief Rate matching with large code block length, when CaseIndex<92 in TS 136.212 table 5.1.3-3, with SSE instructions
 * @param[in] r index of current code block in all code blocks
 * @param[in] C Total number of code blocks
 * @param[in] direction flag of DL or UL, 1 for DL and 0 for UL
 * @param[in] Nsoft Total number of soft bits according to UE categories
 * @param[in] KMIMO 2, which is related to MIMO type
 * @param[in] MDL_HARQ Maximum number of DL HARQ
 * @param[in] G length of bits before modulation for 1 UE in 1 subframe
 * @param[in] NL Number of layer
 * @param[in] Qm Modulation type, which can be 2/4/6, for QPSK/QAM16/QAM64
 * @param[in] rvidx Redundancy version, which can be 0/1/2/3
 * @param[in] bypass_rvidx If set ignore rvidx and set k0 to 0
 * @param[in] Kidx Postion in turbo code internal interleave table, TS 136.212 table 5.1.3-3
 * @param[in] nLen Length of input data in bits
 * @param[in] tin0 pointer to input stream 0 from turbo encoder
 * @param[in] tin1 pointer to input stream 1 from turbo encoder
 * @param[in] tin2 pointer to input stream 2 from turbo encoder
 * @param[out] output buffer for data stream after rate matching
 * @param[in] OutputLen Accumulated output length in bytes of rate matching before this code block
 * @param[in] pTable address for bit to byte table
 * @note tin0, tin1, tin2, and output need to be aligned with 128bits
 * @return Accumulated output length in bytes of rate matching after this code block in TB
 */
int32_t rate_matching_turbo_lte_short_sse(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO,
                    int32_t MDL_HARQ, int32_t G,int32_t NL, int32_t Qm, int32_t rvidx, int8_t bypass_rvidx, int32_t Kidx,
                    int32_t nLen, uint8_t * tin0,uint8_t * tin1,
                    uint8_t * tin2, uint8_t * output,
                    uint32_t OutputLen,uint8_t *pTable);

/**
 * @brief Rate matching with large code block length, when CaseIndex >92 in TS 136.212 table 5.1.3-3, with SSE instructions
 * @param[in] r index of current code block in all code blocks
 * @param[in] C Total number of code blocks
 * @param[in] direction flag of DL or UL, 1 for DL and 0 for UL
 * @param[in] Nsoft Total number of soft bits according to UE categories
 * @param[in] KMIMO 2, which is related to MIMO type
 * @param[in] MDL_HARQ Maximum number of DL HARQ
 * @param[in] G length of bits before modulation for 1 UE in 1 subframe
 * @param[in] NL Number of layer
 * @param[in] Qm Modulation type, which can be 2/4/6
 * @param[in] rvidx Redundancy version, which can be 0/1/2/3
 * @param[in] bypass_rvidx If set ignore rvidx and set k0 to 0
 * @param[in] Kidx Postion in turbo code internal interleave table, TS 136.212 table 5.1.3-3
 * @param[in] nLen Length of input data in bits, but the function reads 832 bytes for all the length
 * @param[in] tin0 pointer to input stream 0 from turbo encoder
 * @param[in] tin1 pointer to input stream 1 from turbo encoder
 * @param[in] tin2 pointer to input stream 2 from turbo encoder
 * @param[out] output buffer for data stream after rate matching
 * @param[in] OutputLen Accumulated output length in bytes of rate matching before this code block
 * @param[in] pTable address for bit to byte table
 * @note tin0, tin1, tin2, and output need to be aligned with 128bits
 * @return Accumulated output length in bytes of rate matching after this code block
 */
int32_t lte_rate_matching_lte_k6144_sse(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO,
    int32_t MDL_HARQ, int32_t G,int32_t NL, int32_t Qm, int32_t rvidx, int8_t bypass_rvidx, int32_t Kidx, int32_t nLen,
    uint8_t * pd0, uint8_t * pd1, uint8_t * pd2,
    uint8_t * pOutput, int32_t OutputLen, int32_t* plen_table );

/**
 * @brief Rate matching in TS 136.212 table 5.1.3-3, with SSE instructions
 * @param[in] r index of current code block in all code blocks
 * @param[in] C Total number of code blocks
 * @param[in] direction flag of DL or UL, 1 for DL and 0 for UL
 * @param[in] Nsoft Total number of soft bits according to UE categories
 * @param[in] KMIMO 2, which is related to MIMO type
 * @param[in] MDL_HARQ Maximum number of DL HARQ
 * @param[in] G length of bits before modulation
 * @param[in] NL Number of layer
 * @param[in] Qm Modulation type, which can be 2/4/6
 * @param[in] rvidx Redundancy version, which can be 0/1/2/3
 * @param[in] bypass_rvidx If set ignore rvidx and set k0 to 0
 * @param[in] Kidx Postion in turbo code internal interleave table, TS 136.212 table 5.1.3-3
 * @param[in] nLen Length of input data in bits
 * @param[in] tin0 pointer to input stream 0 from turbo encoder
 * @param[in] tin1 pointer to input stream 1 from turbo encoder
 * @param[in] tin2 pointer to input stream 2 from turbo encoder
 * @param[out] output buffer for data stream after rate matching
 * @param[in] OutputLen Accumulated output length of rate matching before this code block
 * @note tin0, tin1, tin2, and output need to be aligned with 128bits
 * @return Accumulated output length of rate matching after this code block
 */
int32_t rate_matching_turbo_lte_sse(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO,
                    int32_t MDL_HARQ, int32_t G, int32_t NL, int32_t Qm, int32_t rvidx, int8_t bypass_rvidx, int32_t Kidx,
                    int32_t nLen, uint8_t * tin0,uint8_t * tin1,
                    uint8_t * tin2, uint8_t * output,
                    uint32_t OutputLen );

int32_t rate_matching_turbo_lte_avx2(int32_t r, int32_t C, int8_t direction, int32_t Nsoft, int32_t KMIMO,
    int32_t MDL_HARQ, int32_t G, int NL, int32_t Qm, int32_t rvidx, int8_t bypass_rvidx, int32_t Kidx, const int32_t nLen,
    uint8_t * pd0, uint8_t * pd1, uint8_t * pd2, uint8_t * pOutput, int32_t outputLen);

/** Print to stdout the current version string */
void bblib_print_rate_match_version();

#endif
