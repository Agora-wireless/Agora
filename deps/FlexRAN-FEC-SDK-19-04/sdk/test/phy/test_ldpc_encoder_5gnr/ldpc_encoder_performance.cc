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

#include "common.hpp"

#include "phy_ldpc_encoder_5gnr.h"

const std::string module_name = "ldpc_encoder_5gnr";

class LDPCEncoder5GNRPerf : public KernelTests {
protected:
    struct bblib_ldpc_encoder_5gnr_request ldpc_encoder_5gnr_request{};
    struct bblib_ldpc_encoder_5gnr_response ldpc_encoder_5gnr_response{};

    void SetUp() override {
        init_test("performance");

        const int buffer_len = 1024 * 1024;

 		ldpc_encoder_5gnr_request.Zc = get_input_parameter<uint16_t>("Zc");
        ldpc_encoder_5gnr_request.baseGraph = get_input_parameter<int32_t>("baseGraph");
        ldpc_encoder_5gnr_request.nRows = get_input_parameter<int32_t>("nRows");
        ldpc_encoder_5gnr_request.numberCodeblocks = get_input_parameter<int32_t>("numberCodeblocks");
		for (int i=0; i<ldpc_encoder_5gnr_request.numberCodeblocks; i++) {
	        ldpc_encoder_5gnr_request.input[i] = get_input_parameter<int8_t*>("input");
	        ldpc_encoder_5gnr_response.output[i] = aligned_malloc<int8_t>(buffer_len, 64);
		    //must set 0, because for some cases, output is not byte aligned
	        memset(ldpc_encoder_5gnr_response.output[i], 0, buffer_len);
        }  
    }

    void TearDown() override {
		for (int i=0; i<ldpc_encoder_5gnr_request.numberCodeblocks; i++) {
	        aligned_free(ldpc_encoder_5gnr_request.input[i]);
	        aligned_free(ldpc_encoder_5gnr_response.output[i]);
        }
    }
};

#ifdef _BBLIB_AVX512_
TEST_P(LDPCEncoder5GNRPerf, AVX512_Perf)
{
    performance("AVX512", module_name, bblib_ldpc_encoder_5gnr_avx512, &ldpc_encoder_5gnr_request, &ldpc_encoder_5gnr_response);
}
#endif

INSTANTIATE_TEST_CASE_P(UnitTest, LDPCEncoder5GNRPerf,
                        testing::ValuesIn(get_sequence(LDPCEncoder5GNRPerf::get_number_of_cases("performance"))));