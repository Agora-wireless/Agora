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
 *  @file   phy_ldpc_dencoder_5gnr_avx512.cpp
 *  @brief  AVX512 code for 5GNR LDPC Encoder functions.
 */

#include <stdlib.h>
#include <string.h>
#include <ipp.h>
#include <ipps.h>
#include <immintrin.h>  /* AVX512 */
#include "phy_ldpc_decoder_5gnr.h"
#include "phy_ldpc_decoder_5gnr_internal.h"
#include "LdpcDecoder.hpp"

#include "common_typedef_sdk.h"



//-------------------------------------------------------------------------------------------
/**
 *  @brief Decoding for LDPC in 5GNR.
 *  @param [in] request Structure containing configuration information and input data.
 *  @param [out] response Structure containing kernel outputs.
 *  @return Success: return 0, else: return -1.
**/
int32_t bblib_ldpc_decoder_5gnr_avx512(struct bblib_ldpc_decoder_5gnr_request *request, struct bblib_ldpc_decoder_5gnr_response *response)
{
	SimdLdpc::Request local_request;
	SimdLdpc::Response local_response;

	local_request.basegraph = request->baseGraph == 1 ?
			SimdLdpc::BaseGraph::BG1 : SimdLdpc::BaseGraph::BG2;
	local_request.enableEarlyTermination = request->enableEarlyTermination;
	local_request.maxIterations = request->maxIterations;
	local_request.nRows = request->nRows;
	local_request.numChannelLlrs = request->numChannelLlrs;
	local_request.numFillerBits = request->numFillerBits;
	local_request.varNodes = request->varNodes;
	local_request.z = request->Zc;
	local_response.compactedMessageBytes = response->compactedMessageBytes;
	local_response.varNodes = response->varNodes;

	SimdLdpc::DecodeAvx512(&local_request, &local_response);

	response->iterationAtTermination = local_response.iterationAtTermination;
	response->numMsgBits = local_response.numMsgBits;
	response->parityPassedAtTermination = local_response.parityPassedAtTermination;
	//FIXME : Workaround for now
	//Mask the last byte
	int bitsInLastByte = local_response.numMsgBits % 8;
	if (bitsInLastByte > 0) {
		int lastbyte  = (local_response.numMsgBits +7)/8 -1;
		response->compactedMessageBytes[lastbyte] &=
				(1 << bitsInLastByte) - 1;
	}

	return 0;
}
