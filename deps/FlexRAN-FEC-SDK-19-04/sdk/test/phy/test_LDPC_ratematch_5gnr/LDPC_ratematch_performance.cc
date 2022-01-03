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

#include "phy_LDPC_ratematch_5gnr.h"

const std::string module_name = "LDPC_ratematch_5gnr";

class LDPCRatematch5GNRPerf : public KernelTests {
protected:
    struct bblib_LDPC_ratematch_5gnr_request LDPC_ratematch_5gnr_request{};
    struct bblib_LDPC_ratematch_5gnr_response LDPC_ratematch_5gnr_response{};

    void SetUp() override {
        init_test("performance");

        const int buffer_len = 1024 * 1024;

        LDPC_ratematch_5gnr_request.Ncb = get_input_parameter<int32_t>("Ncb");
        LDPC_ratematch_5gnr_request.Zc = get_input_parameter<int32_t>("Zc");
        LDPC_ratematch_5gnr_request.E = get_input_parameter<int32_t>("E");
        LDPC_ratematch_5gnr_request.Qm = get_input_parameter<int32_t>("Qm");
        LDPC_ratematch_5gnr_request.rvidx = get_input_parameter<int32_t>("rvidx");
        LDPC_ratematch_5gnr_request.baseGraph = get_input_parameter<int32_t>("baseGraph");
        LDPC_ratematch_5gnr_request.nullIndex = get_input_parameter<int32_t>("nullIndex");
        LDPC_ratematch_5gnr_request.nLen = get_input_parameter<int32_t>("nLen");
        LDPC_ratematch_5gnr_request.input = aligned_malloc<uint8_t>(buffer_len, 64);

        LDPC_ratematch_5gnr_response.output = aligned_malloc<uint8_t>(buffer_len, 64);
    }

    void TearDown() override {
        aligned_free(LDPC_ratematch_5gnr_request.input);
        aligned_free(LDPC_ratematch_5gnr_response.output);
    }
};

#ifdef _BBLIB_AVX512_
TEST_P(LDPCRatematch5GNRPerf, AVX512_Perf)
{
    performance("AVX512", module_name, bblib_LDPC_ratematch_5gnr_avx512, &LDPC_ratematch_5gnr_request, &LDPC_ratematch_5gnr_response);
}
#endif

INSTANTIATE_TEST_CASE_P(UnitTest, LDPCRatematch5GNRPerf,
                        testing::ValuesIn(get_sequence(LDPCRatematch5GNRPerf::get_number_of_cases("performance"))));
