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
 * @file phy_rate_dematching_5gnr_avx512.cpp
 * @brief  Implementation for rate dematching functions
 */

#include "phy_rate_dematching_5gnr_internal.h"
#include <ipp.h>
#include <ipps.h>
#include <string.h>


/* Bit interleaving as per 3GPP 38.212 5.4.2.2 */
void deInterleave(int8_t *pCbIn, int8_t *pDeInterleave, bblib_rate_dematching_5gnr_request *pRM) {
    int32_t byte, mod, intl_size = pRM->e / pRM->modulation_order;
    for (byte = 0; byte < intl_size; byte++)
        for (mod =0; mod < pRM->modulation_order; mod++)
            pDeInterleave[byte + mod * intl_size] =
                    pCbIn[pRM->modulation_order * byte + mod];
}

/* Adds and saturate to MAX_LLR the 2 LLR streams */
void combine(int8_t *pHarq, int8_t *p_in, int16_t length) {
    int16_t byte, temp;
    for (byte = 0; byte < length; byte++) {
        temp = ((int16_t) pHarq[byte]) + ((int16_t) p_in[byte]);
        temp = MIN(MAX_LLR, MAX(temp, MIN_LLR));
        pHarq[byte] = (int8_t) temp;
    }
}

void harq_combine(struct bblib_rate_dematching_5gnr_request *request,
    struct bblib_rate_dematching_5gnr_response *response, int8_t *pDeInterleave)
{
    if (request->isretx == 0)
        memset(request->p_harq,0x00,request->ncb);
    get_k0(request);
    int32_t ncb_, length, offset_e=0, offset_ncb=request->k0;
    ncb_ = request->ncb - request->num_of_null;
    if (offset_ncb > request->start_null_index)
        offset_ncb -= request->num_of_null;

    while (offset_e < request->e) {
        length = MIN(request->e - offset_e, ncb_ - offset_ncb);
        combine((int8_t *) request->p_harq + offset_ncb, (int8_t *) pDeInterleave + offset_e, length);
        offset_ncb = length + offset_ncb;
        if (offset_ncb == ncb_)
            offset_ncb = 0;
        offset_e += length;
    }
}

/**
 * @brief Implements rate dematching with AVX512
 * @param [in] request Structure containing the configuration, input data
 * @param [out] response Structure containing the output data.
**/
void bblib_rate_dematching_5gnr_c(struct bblib_rate_dematching_5gnr_request *req,
struct bblib_rate_dematching_5gnr_response *resp)
{
    __declspec (align(64)) int8_t internalBuffer[40 * 1024];
    deInterleave(req->p_in, internalBuffer, req);
    harq_combine(req,resp,internalBuffer);
}
