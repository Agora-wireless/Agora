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

#include "phy_rate_match.h"

const std::string module_name = "rate_matching";

enum class TestType {
    DL = 0,
    UL = 1,
    UL_DEINTERLEAVE = 2
};

class RateMatchingCheck : public KernelTests {
protected:
    struct bblib_rate_match_dl_request dl_request{};
    struct bblib_rate_match_dl_response dl_response{};
    struct bblib_rate_match_dl_response dl_reference{};

    struct bblib_rate_match_ul_request ul_request{};
    struct bblib_rate_match_ul_response ul_response{};
    struct bblib_rate_match_ul_response ul_reference{};

    struct bblib_deinterleave_ul_request ul_deinterleave_request{};
    struct bblib_deinterleave_ul_response ul_deinterleave_response{};
    struct bblib_deinterleave_ul_response ul_deinterleave_reference{};
    /* There are 2 functions to test, 2 types of tests are implemented:
       bblib_rate_match_dl (TestType::DL),
       bblib_rate_match_ul (TestType::UL) */
    TestType test_type;

    int ul_output_len;
    int ul_deinterleave_num_dummy; /* number of dummy bits per bit stream (Nd) */
    int ul_deinterleave_bitstream_len; /* bit stream length (Kpi) */

    void SetUp() override {
        init_test("functional");

        const int buffer_len = 1024 * 128 * 1200;

        test_type = TestType(get_input_parameter<int>("test_type"));

        switch (test_type) {
            case TestType::DL :
                dl_request.r = get_input_parameter<int32_t>("r");
                dl_request.C = get_input_parameter<int32_t>("C");
                dl_request.direction = get_input_parameter<int8_t>("direction");
                dl_request.Nsoft = get_input_parameter<int32_t>("Nsoft");
                dl_request.KMIMO = get_input_parameter<int32_t>("KMIMO");
                dl_request.MDL_HARQ = get_input_parameter<int32_t>("MDL_HARQ");
                dl_request.G = get_input_parameter<int32_t>("G");
                dl_request.NL = get_input_parameter<int32_t>("NL");
                dl_request.Qm = get_input_parameter<int32_t>("Qm");
                dl_request.rvidx = get_input_parameter<int32_t>("rvidx");
                dl_request.bypass_rvidx = get_input_parameter<int8_t>("bypass_rvidx");
                dl_request.Kidx = get_input_parameter<int32_t>("Kidx");
                dl_request.nLen = get_input_parameter<int32_t>("nLen");

                dl_request.tin0 = get_input_parameter<uint8_t*>("tin0");
                dl_request.tin1 = get_input_parameter<uint8_t*>("tin1");
                dl_request.tin2 = get_input_parameter<uint8_t*>("tin2");

                dl_response.output = aligned_malloc<uint8_t>(buffer_len, 64);

                dl_reference.output = get_reference_parameter<uint8_t*>("output");
                dl_reference.OutputLen = get_reference_parameter<uint32_t>("output_len");
                break;

            case TestType::UL :
                ul_request.k0withoutnull = get_input_parameter<int32_t>("k0withoutnull");
                ul_request.ncb = get_input_parameter<int32_t>("ncb");
                ul_request.e = get_input_parameter<int32_t>("e");
                ul_request.isretx = get_input_parameter<int32_t>("isretx");
                ul_request.isinverted = get_input_parameter<int32_t>("isinverted");

                ul_request.pdmout = get_input_parameter<uint8_t*>("dmout");

                ul_response.pharqbuffer = aligned_malloc<uint8_t>(buffer_len, 64);
                ul_response.pinteleavebuffer = aligned_malloc<uint8_t>(buffer_len, 64);
                ul_response.pharqout = aligned_malloc<uint8_t>(buffer_len, 64);
                std::memset(ul_response.pinteleavebuffer, 0, buffer_len);

                ul_reference.pharqout = get_reference_parameter<uint8_t*>("harqout");
                ul_output_len = get_reference_parameter<int>("output_len");
                break;
            case TestType::UL_DEINTERLEAVE :
                ul_deinterleave_request.ncb = get_input_parameter<int32_t>("ncb");
                ul_deinterleave_request.pharqbuffer = get_input_parameter<uint8_t*>("harqbuffer");
                ul_deinterleave_request.circ_buffer =
                    (get_input_parameter<int>("circ_buffer") == 0) ? BBLIB_CIRCULAR_BUFFER_WITHOUT_PADDING :
                                                                     BBLIB_FULL_CIRCULAR_BUFFER;

                ul_deinterleave_response.pinteleavebuffer = aligned_malloc<uint8_t>(buffer_len, 64);
                std::memset(ul_deinterleave_response.pinteleavebuffer, 0, buffer_len);
                ul_deinterleave_num_dummy = get_reference_parameter<int>("num_dummy");
                ul_deinterleave_bitstream_len = get_reference_parameter<int>("bitstream_len");

                ul_deinterleave_reference.pinteleavebuffer = get_reference_parameter<uint8_t*>("interleavebuffer");

        }
    }

    void TearDown() override {
        switch (test_type) {
            case TestType::DL :
                aligned_free(dl_request.tin0);
                aligned_free(dl_request.tin1);
                aligned_free(dl_request.tin2);
                aligned_free(dl_response.output);
                break;
            case TestType::UL :
                aligned_free(ul_request.pdmout);
                aligned_free(ul_response.pharqbuffer);
                aligned_free(ul_response.pinteleavebuffer);
                aligned_free(ul_response.pharqout);
                break;
            case TestType::UL_DEINTERLEAVE :
                aligned_free(ul_deinterleave_request.pharqbuffer);
                aligned_free(ul_deinterleave_response.pinteleavebuffer);
                aligned_free(ul_deinterleave_reference.pinteleavebuffer);
                break;
        }
    }

    template <typename F, typename ... Args>
    void functional(F function, const std::string isa, Args ... args)
    {
        constexpr int NUM_OF_BIT_STREAMS = 3;
        function(args ...);

        switch (test_type) {
            case TestType::DL :
                ASSERT_ARRAY_EQ(dl_reference.output,
                                dl_response.output,
                                dl_reference.OutputLen);
                break;
            case TestType::UL :
                ASSERT_ARRAY_EQ(ul_reference.pharqout,
                                ul_response.pharqout,
                                ul_output_len);
                break;
            case TestType::UL_DEINTERLEAVE :
                /* Structure of inteleavebuffer:
                   - begin of original data = pinteleavebuffer + num_dummy
                   - begin of parity 1 bits = pinteleavebuffer + bitstream_len + num_dummy + 64
                 * - begin of parity 1 bits = pinteleavebuffer + bitstream_len + num_dummy + 64
                 */
                for (int j = 0; j < ul_deinterleave_bitstream_len - ul_deinterleave_num_dummy; j++)
                    ASSERT_EQ(*(ul_deinterleave_reference.pinteleavebuffer + ul_deinterleave_num_dummy),
                              *(ul_deinterleave_response.pinteleavebuffer + ul_deinterleave_num_dummy))
                        << "Error in bit stream (1-3): " << 1 << " position " << j << std::endl;
                for (int j = 0; j < ul_deinterleave_bitstream_len - ul_deinterleave_num_dummy; j++)
                    ASSERT_EQ(*(ul_deinterleave_reference.pinteleavebuffer +
                                    ul_deinterleave_num_dummy + ul_deinterleave_bitstream_len + 64),
                              *(ul_deinterleave_response.pinteleavebuffer +
                                    ul_deinterleave_num_dummy + ul_deinterleave_bitstream_len + 64))
                        << "Error in bit stream (1-3): " << 2 << " position " << j << std::endl;
                for (int j = 0; j < ul_deinterleave_bitstream_len - ul_deinterleave_num_dummy; j++)
                    ASSERT_EQ(*(ul_deinterleave_reference.pinteleavebuffer +
                                    ul_deinterleave_num_dummy + 2 * ul_deinterleave_bitstream_len + 64 -1),
                              *(ul_deinterleave_response.pinteleavebuffer +
                                    ul_deinterleave_num_dummy + 2 * ul_deinterleave_bitstream_len + 64 - 1))
                        << "Error in bit stream (1-3): " << 3 << " position " << j << std::endl;
                break;
        }
        print_test_description(isa, module_name);
    }
};

#if defined(_BBLIB_SSE4_2_)
TEST_P(RateMatchingCheck, SSE4_2_Check)
{
    switch (test_type) {
        case TestType::DL :
            functional(bblib_rate_match_dl_sse, "SSE", &dl_request, &dl_response);
            break;
        case TestType::UL :
        case TestType::UL_DEINTERLEAVE :
            std::cout << "[----------] No function defined for UL SSE4_2." << std::endl;
            break;
    }
}
#endif

#ifdef _BBLIB_AVX2_
TEST_P(RateMatchingCheck, AVX2_Check)
{
    switch (test_type) {
        case TestType::DL :
            functional(bblib_rate_match_dl_avx2, "AVX2", &dl_request, &dl_response);
            break;
        case TestType::UL :
            functional(bblib_rate_match_ul_avx2, "AVX2", &ul_request, &ul_response);
            break;
        case TestType::UL_DEINTERLEAVE :
            functional(bblib_deinterleave_ul_avx2, "AVX2", &ul_deinterleave_request, &ul_deinterleave_response);
            break;
    }
}
#endif

#ifdef _BBLIB_AVX512_
TEST_P(RateMatchingCheck, AVX512_Check)
{
    switch (test_type) {
        case TestType::DL :
            std::cout << "[----------] No function defined for DL AVX512." << std::endl;
            break;
        case TestType::UL :
            functional(bblib_rate_match_ul_avx512, "AVX512", &ul_request, &ul_response);
            break;
        case TestType::UL_DEINTERLEAVE :
            functional(bblib_deinterleave_ul_avx512, "AVX512", &ul_deinterleave_request, &ul_deinterleave_response);
            break;
    }
}
#endif

INSTANTIATE_TEST_CASE_P(UnitTest, RateMatchingCheck,
                        testing::ValuesIn(get_sequence(RateMatchingCheck::get_number_of_cases("functional"))));
