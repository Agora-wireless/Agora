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

#ifndef _PHY_TURBO_INTERNAL_H_
#define _PHY_TURBO_INTERNAL_H_

#include <stdint.h>
#include "common_typedef_sdk.h"

#define MAX_DATA_LEN_INTERLEAVE (8192)

typedef struct {
    int8_t pattern[448][16];
    int32_t offset[188];
    int32_t inter_row_out_addr_for_interleaver[21693];
    int32_t inter_row_out_addr_for_deinterleaver[21693];
    int32_t intra_row_perm_pattern_for_interleaver[21693];
    int32_t intra_row_perm_pattern_for_deinterleaver[21693];
} _TurboInterleaver;

extern _TurboInterleaver g_TurboInterleaver;

/** @fn lte_turbo_interleaver_8windows_sse
 *  @brief  This function implements Turbo internal interleaver defined in TS36.212 section
       5.1.3.2.1. Used in transmitter.
 *  @param [in] caseId is i defined in TS36.212 Table 5.1.3-3
 *  @param [in] pInData buffer stores information bits and CRC bits. It should be 16-byte aligned so that SSE load is fesible.
 *  @param [in] pOutData buffer stores interleaved bit stream. It should be 16-byte aligned so that SSE load is fesible.
 *  @return int32_t, 1 is successful. otherwire is fail.
 */
int32_t
bblib_lte_turbo_interleaver_8windows_sse(uint8_t caseId, uint8_t *pInData, uint8_t* pOutData);

/** @fn lte_turbo_decoder_64windows_avx512
 *  @brief This function implements Turbo decoder when CW size is multiple of 64.
 *  @param[in] p is pointer of TurboDecoder_para
 *  @return 0 successful, -1 is fail.
*/
int32_t lte_turbo_decoder_64windows_avx512(void *p);

/** @fn lte_turbo_interleaver_initTable
 *  @brief  inintalize interleaver table function
 *  @param [in] pTabelPath the table path.
 *  @return void
 */
void
bblib_lte_turbo_interleaver_initTable(char* pTabelPath);

void
bblib_print_turbo_version();

void
init_turbo_decoder_interleaver_table(char *Table_Path, _TurboInterleaver *p);

void
get_turbo_buf_addr_table_new(uint16_t *p_table_TurboBufAddr,
                             int32_t *p_table_TurboBufAddr_Offset,
                             int32_t (*p_table_Kidx_K_Nmaxrep_shuf)[5]);

int32_t
init_common_tables(char *pTabelPath);

// for softbit mapping
#define _LOG_P1_P0

#endif
