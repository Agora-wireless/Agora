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

#include "phy_turbo.h"

const std::string module_name = "turbo";

enum class TestType {
    DEC = 0,
    ENC = 1
};

class TurboPerf : public KernelTests {
protected:
    struct bblib_turbo_decoder_request dec_request{};
    struct bblib_turbo_decoder_response dec_response{};

    struct bblib_turbo_encoder_request enc_request{};
    struct bblib_turbo_encoder_response enc_response{};


    /* There are 2 functions to test, 2 types of tests are implemented:
       bblib_turbo_decoder (TestType::DEC),
       bblib_turbo_encoder (TestType::ENC) */
    TestType test_type;

    void SetUp() override {
        init_test("performance");

        const int dec_ag_buf_len = 6528 * 16;
        const int input_len = 1024 * 1024;
        int dec_output_len;
        int enc_output_len;

        test_type = TestType(get_input_parameter<int>("test_type"));

        switch (test_type) {
#if defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)
            case TestType::DEC :
                dec_request.c = get_input_parameter<int32_t>("c");
                dec_request.k = get_input_parameter<int32_t>("k");
                dec_request.k_idx = get_input_parameter<int32_t>("k_idx");
                dec_request.max_iter_num = get_input_parameter<int32_t>("max_iter_num");
                dec_request.early_term_disable = get_input_parameter<int32_t>("early_term_disable");
                dec_request.input = generate_random_data<int8_t>(input_len, 64);

                dec_output_len = get_input_parameter<int>("output_len");
                dec_response.output = aligned_malloc<uint8_t>(dec_output_len, 64);
                dec_response.ag_buf = aligned_malloc<int8_t>(dec_ag_buf_len, 64);
                dec_response.cb_buf = aligned_malloc<uint16_t>((dec_request.k / 8), 64);
                break;
#endif
            case TestType::ENC :
                enc_request.length = get_input_parameter<uint32_t>("turbo_len");
                enc_request.case_id = get_input_parameter<uint8_t>("case_id");
                enc_request.input_win = generate_random_data<uint8_t>(input_len, 64);

                enc_output_len = get_input_parameter<int>("output_len");
                enc_response.output_win_0 = aligned_malloc<uint8_t>(enc_output_len, 64);
                enc_response.output_win_1 = aligned_malloc<uint8_t>(enc_output_len, 64);
                enc_response.output_win_2 = aligned_malloc<uint8_t>(enc_output_len, 64);
                break;
        }
   }

    void TearDown() override {
        switch (test_type) {
#if defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)
            case TestType::DEC :
                aligned_free(dec_request.input);
                aligned_free(dec_response.output);
                aligned_free(dec_response.ag_buf);
                aligned_free(dec_response.cb_buf);
                break;
#endif
            case TestType::ENC :
                aligned_free(enc_request.input_win);
                aligned_free(enc_response.output_win_0);
                aligned_free(enc_response.output_win_1);
                aligned_free(enc_response.output_win_2);
                break;
        }
    }
};

/**
 * Test cases are not extracted as there are no test vectors to test windows smaller
 * than 32, hence ISA specific functions cannot be tested. This problem should be
 * addressed in the future (SCSY-1380).
 */
#ifdef _BBLIB_SSE4_2_
TEST_P(TurboPerf, SSE_Perf)
{
    switch (test_type) {
        case TestType::ENC :
            performance("SSE", module_name, bblib_lte_turbo_encoder_sse, &enc_request, &enc_response);
            break;
        default :
            std::cout << "[----------] No test case defined for windows 8 and 16." << std::endl;
            break;
    }
}
#endif

#ifdef _BBLIB_AVX2_
TEST_P(TurboPerf, AVX2_Perf)
{
    switch (test_type) {
        case TestType::DEC :
            performance("AVX2", module_name, bblib_lte_turbo_decoder_32windows_avx2, &dec_request, &dec_response);
            break;
        case TestType::ENC :
            performance("AVX2", module_name, bblib_lte_turbo_encoder_avx2, &enc_request, &enc_response);
            break;
    }
}
#endif

#ifdef _BBLIB_AVX512_
TEST_P(TurboPerf, AVX512_Perf)
{
    switch (test_type) {
        case TestType::DEC :
            performance("AVX512", module_name, bblib_lte_turbo_decoder_64windows_avx512, &dec_request, &dec_response);
            break;
        case TestType::ENC :
            performance("AVX512", module_name, bblib_lte_turbo_encoder_avx512, &enc_request, &enc_response);
            break;
    }
}
#endif

INSTANTIATE_TEST_CASE_P(UnitTest, TurboPerf,
                        testing::ValuesIn(get_sequence(TurboPerf::get_number_of_cases("performance"))));
