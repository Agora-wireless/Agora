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

#include "phy_rate_dematching_5gnr.h"

const std::string module_name = "rate_dematching_5gnr";

class RateDematching5GNRPerf : public KernelTests {
protected:
    struct bblib_rate_dematching_5gnr_request request{};
    struct bblib_rate_dematching_5gnr_response response{};

    void SetUp() override {
        init_test("performance");

        const unsigned harq_buffer = 26*1024;

        request.ncb = get_input_parameter<int32_t>("ncb");
        request.start_null_index = get_input_parameter<int32_t>("start_null");
        request.num_of_null = get_input_parameter<int32_t>("n_null");
        request.e = get_input_parameter<int32_t>("e");
        request.rvid = get_input_parameter<int32_t>("rv_id");
        request.zc = get_input_parameter<int32_t>("z_c");
        request.modulation_order = get_input_parameter<bblib_modulation_order>("mod_q");
        request.base_graph = get_input_parameter<int32_t>("flag_of_bg");
        request.isretx = get_input_parameter<int32_t>("is_retx");

        request.p_in = generate_random_data<int8_t>(request.e, 64);
        request.p_harq = aligned_malloc<int8_t>(harq_buffer, 64);
    }

    void TearDown() override {
        aligned_free(request.p_in);
        aligned_free(request.p_harq);
    }
};

#ifdef _BBLIB_AVX512_
TEST_P(RateDematching5GNRPerf, AVX512_Perf)
{
        performance("AVX512", module_name, bblib_rate_dematching_5gnr_avx512, &request, &response);
}
#endif

TEST_P(RateDematching5GNRPerf, C_Perf)
{
	performance("C", module_name, bblib_rate_dematching_5gnr_c, &request, &response);
}


INSTANTIATE_TEST_CASE_P(UnitTest, RateDematching5GNRPerf,
                        testing::ValuesIn(get_sequence(RateDematching5GNRPerf::get_number_of_cases("performance"))));
