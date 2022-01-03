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
 * @file phy_rate_dematching_5gnr_internal.cpp
 * @brief  Implementation for rate dematching functions
 */

#include "phy_rate_dematching_5gnr_internal.h"


/**
 * @brief This function implements k0 calculation
 * @param[in] the pointer of rate dematching struct
 *
 */
void get_k0(struct bblib_rate_dematching_5gnr_request *pRM)
{
    int32_t k0;
    int32_t Ncb = pRM->ncb;
    int32_t Zc = pRM->zc;
    int32_t flagOfBG = pRM->base_graph;

    /* calculating k0 according to spec. */
    switch (pRM->rvid) {
        case 1:
            if(flagOfBG == 1)
                k0 = Zc * ((17*Ncb)/(66*Zc));
            else
                k0 = Zc * ((13*Ncb)/(50*Zc));
            break;
        case 2:
            if(flagOfBG == 1)
                k0 = Zc * ((33*Ncb)/(66*Zc));
            else
                k0 = Zc * ((25*Ncb)/(50*Zc));
            break;
        case 3:
            if(flagOfBG == 1)
                k0 = Zc * ((56*Ncb)/(66*Zc));
            else
                k0 = Zc * ((43*Ncb)/(50*Zc));
            break;
        default:
            k0 = 0;
    }
    pRM->k0 = k0;
}
