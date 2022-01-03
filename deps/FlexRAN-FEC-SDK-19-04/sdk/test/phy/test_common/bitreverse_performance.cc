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

#include "bit_reverse.h"

const std::string module_name = "bit_reverse";

class BitReversePerf : public KernelTests {
protected:
    int8_t* pInOut;
    int32_t num_data;

    void SetUp() override {
        init_test("bitreverse_performance");
        num_data = get_input_parameter<int32_t>("num_data");
        pInOut = get_input_parameter<int8_t*>("input");
    }

    void TearDown() override {
        aligned_free(pInOut);
    }
};

#ifdef _BBLIB_AVX512_
TEST_P(BitReversePerf, AVX512_Perf)
{
    performance("AVX512", module_name, bblib_bit_reverse_avx512, pInOut, num_data);
}
#endif

#ifdef _BBLIB_AVX2_
TEST_P(BitReversePerf, AVX2_Perf)
{
    performance("AVX2", module_name, bblib_bit_reverse_avx2, pInOut, num_data);
}
#endif

TEST_P(BitReversePerf, C_Perf)
{
    performance("C", module_name, bblib_bit_reverse_c, pInOut, num_data);
}

INSTANTIATE_TEST_CASE_P(UnitTest, BitReversePerf,
                        testing::ValuesIn(get_sequence(BitReversePerf::get_number_of_cases("bitreverse_performance"))));