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

#include "float_int16_convert_agc.h"

enum class TestType {
    I2I = 0,
    I2F = 1,
    F2I = 2,
    F2I_TH = 3,
    I2I_FXP = 4
};

struct Int16_to_int16_data {
    int16_t* output;
    int16_t* input;
    int16_t* ref_output;
};

struct Int16_to_float_data {
    float* output;
    int16_t* input;
    float* ref_output;
};

struct Float_to_int16_data {
    int16_t* output;
    float* input;
    int16_t* ref_output;
};

struct Float_to_int16_th_data {
    int16_t* output;
    float* input;
    int16_t* ref_output;
};

struct Int16_to_int16_fxp_data {
    int16_t* output;
    int16_t* input;
    int16_t* ref_output;
};

const std::string module_name = "float_fix_convert";

class FloatFixConvertCheck : public KernelTests {
protected:
    struct Int16_to_int16_data i2i{};
    struct Int16_to_float_data i2f{};
    struct Float_to_int16_data f2i{};
    struct Float_to_int16_th_data f2i_th{};
    struct Int16_to_int16_fxp_data i2i_fxp{};

    /* There are 4 functions to test, 4 types of tests are implemented:
       bblib_int16_to_int16_agc (TestType::I2I),
       bblib_int16_to_float_agc (TestType::I2F)
       bblib_float_to_int16_agc (TestType::F2I)
       bblib_float_to_int16_agc_threshold (TestType::F2I_TH) */
    TestType test_type;

    /* Common parameters */
    int32_t num_data;
    float gain;
    double precision;

    /* Parameter for bblib_float_to_int16_agc_threshold */
    int16_t threshold;

    void SetUp() override {
        init_test("float_fix_functional");

        /* Defined as the maximum size of all inputs/outputs in BYTE */
        const int buffer_len = 1024 * 1024 * 120;
        test_type = TestType(get_input_parameter<int>("test_type"));

        num_data = get_input_parameter<int32_t>("num_data");
        gain = get_input_parameter<float>("gain");
        precision = get_input_parameter<float>("precision");

        switch (test_type) {
            case TestType::I2I :
                i2i.input = generate_random_int_numbers<int16_t>((buffer_len / sizeof(int16_t)), 64, -32768, 32767);
                i2i.output = aligned_malloc<int16_t>((buffer_len / sizeof(int16_t)), 64);

                i2i.ref_output = aligned_malloc<int16_t>((buffer_len / sizeof(int16_t)), 64);
                for (int i = 0; i < num_data; i++)
                    i2i.ref_output[i] = (int16_t) std::max(std::min(float(i2i.input[i]) * gain, 32767.0f), -32768.0f);

                break;
            case TestType::I2F :
                i2f.input = generate_random_int_numbers<int16_t>((buffer_len / sizeof(int16_t)), 64, -32768, 32767);
                i2f.output = aligned_malloc<float>((buffer_len / sizeof(float)), 64);

                i2f.ref_output = aligned_malloc<float>((buffer_len / sizeof(float)), 64);
                for (int i = 0; i < num_data; i++)
                    i2f.ref_output[i] = i2f.input[i] * gain;

                break;
            case TestType::F2I :
                f2i.input = generate_random_real_numbers<float>((buffer_len / sizeof(float)), 64, -1048576, 1048575);
                f2i.output = aligned_malloc<int16_t>((buffer_len / sizeof(int16_t)), 64);

                f2i.ref_output = aligned_malloc<int16_t>((buffer_len / sizeof(int16_t)), 64);
                for (int i = 0; i < num_data; i++)
                    f2i.ref_output[i] = (int16_t) std::max(std::min(f2i.input[i] * gain, 32767.0f), -32768.0f);

                break;
            case TestType::F2I_TH :
                f2i_th.input = generate_random_real_numbers<float>((buffer_len / sizeof(float)), 64, -1048576, 1048575);
                f2i_th.output = aligned_malloc<int16_t>((buffer_len / sizeof(int16_t)), 64);

                threshold = get_input_parameter<int16_t>("threshold");

                f2i_th.ref_output = aligned_malloc<int16_t>((buffer_len / sizeof(int16_t)), 64);
                for (int i = 0; i < num_data; i++)
                    f2i_th.ref_output[i] = (int16_t) std::max(std::min(f2i_th.input[i] * gain, (float) threshold), (float) -threshold);

                break;
            case TestType::I2I_FXP :
                i2i_fxp.input = generate_random_int_numbers<int16_t>((buffer_len / sizeof(int16_t)), 64, -32768, 32767);
                i2i_fxp.output = aligned_malloc<int16_t>((buffer_len / sizeof(int16_t)), 64);

                i2i_fxp.ref_output = aligned_malloc<int16_t>((buffer_len / sizeof(int16_t)), 64);
                for (int i = 0; i < num_data; i++)
                    i2i_fxp.ref_output[i] = (int16_t) std::max(std::min( int(i2i_fxp.input[i] * gain)>>16, 32767), -32768);

                break;
        }
    }

    void TearDown() override {
        switch (test_type) {
            case TestType::I2I :
                aligned_free(i2i.input);
                aligned_free(i2i.output);
                aligned_free(i2i.ref_output);
                break;
            case TestType::I2F :
                aligned_free(i2f.input);
                aligned_free(i2f.output);
                aligned_free(i2f.ref_output);
                break;
            case TestType::F2I :
                aligned_free(f2i.input);
                aligned_free(f2i.output);
                aligned_free(f2i.ref_output);
                break;
            case TestType::F2I_TH :
                aligned_free(f2i_th.input);
                aligned_free(f2i_th.output);
                aligned_free(f2i_th.ref_output);
                break;
            case TestType::I2I_FXP :
                aligned_free(i2i_fxp.input);
                aligned_free(i2i_fxp.output);
                aligned_free(i2i_fxp.ref_output);
                break;
        }
    }

    template <typename F, typename ... Args>
    void functional(F function, const std::string isa, Args ... args)
    {
        function(args ...);

        switch (test_type) {
        case TestType::I2I :
            ASSERT_ARRAY_NEAR(i2i.ref_output, i2i.output, num_data, precision);
            break;
        case TestType::I2F :
            ASSERT_ARRAY_NEAR(i2f.ref_output, i2f.output, num_data, precision);
            break;
        case TestType::F2I :
            ASSERT_ARRAY_NEAR(f2i.ref_output, f2i.output, num_data, precision);
            break;
        case TestType::F2I_TH :
            ASSERT_ARRAY_NEAR(f2i_th.ref_output, f2i_th.output, num_data, precision);
            break;
        case TestType::I2I_FXP :
            ASSERT_ARRAY_NEAR(i2i_fxp.ref_output, i2i_fxp.output, num_data, precision);
            break;
        }
        print_test_description(isa, module_name);
    }
};

TEST_P(FloatFixConvertCheck, Default_Check)
{
    switch (test_type) {
        case TestType::I2I :
            functional(bblib_int16_to_int16_agc, "Default",
                       i2i.output, i2i.input, num_data, gain);
            break;
        case TestType::I2F :
            functional(bblib_int16_to_float_agc, "Default",
                       i2f.output, i2f.input, num_data, gain);
            break;
        case TestType::F2I :
            functional(bblib_float_to_int16_agc, "Default",
                       f2i.output, f2i.input, num_data, gain);
            break;
        case TestType::F2I_TH :
            functional(bblib_float_to_int16_agc_threshold, "Default",
                       f2i_th.output, f2i_th.input, num_data, gain, threshold);
            break;
        case TestType::I2I_FXP :
            functional(bblib_int16_to_int16_fxp_scale, "Default",
                       i2i_fxp.output, i2i_fxp.input, num_data, gain);
            break;
    }
}

TEST_P(FloatFixConvertCheck, C_Check)
{
    switch (test_type) {
        case TestType::I2I :
            functional(bblib_int16_to_int16_agc_c, "C",
                        i2i.output, i2i.input, num_data, gain);
            break;
        case TestType::I2F :
            functional(bblib_int16_to_float_agc_c, "C",
                        i2f.output, i2f.input, num_data, gain);
            break;
        case TestType::F2I :
            functional(bblib_float_to_int16_agc_c, "C",
                        f2i.output, f2i.input, num_data, gain);
            break;
        case TestType::F2I_TH :
            functional(bblib_float_to_int16_agc_threshold_c, "C",
                        f2i_th.output, f2i_th.input, num_data, gain, threshold);
            break;
        case TestType::I2I_FXP :
            functional(bblib_int16_to_int16_fxp_scale_c, "C",
                       i2i_fxp.output, i2i_fxp.input, num_data, gain);
            break;
        }
}

#ifdef _BBLIB_AVX2_
TEST_P(FloatFixConvertCheck, AVX2_Check)
{
    switch (test_type) {
        case TestType::I2I :
            functional(bblib_int16_to_int16_agc_avx2, "AVX2",
                        i2i.output, i2i.input, num_data, gain);
            break;
        case TestType::I2F :
            functional(bblib_int16_to_float_agc_avx2, "AVX2",
                        i2f.output, i2f.input, num_data, gain);
            break;
        case TestType::F2I :
            functional(bblib_float_to_int16_agc_avx2, "AVX2",
                        f2i.output, f2i.input, num_data, gain);
            break;
        case TestType::F2I_TH :
            functional(bblib_float_to_int16_agc_threshold_avx2, "AVX2",
                        f2i_th.output, f2i_th.input, num_data, gain, threshold);
            break;
        }
}
#endif

#ifdef _BBLIB_AVX512_
TEST_P(FloatFixConvertCheck, AVX512_Check)
{
    switch (test_type) {
        case TestType::I2I :
            functional(bblib_int16_to_int16_agc_avx512, "AVX512",
                        i2i.output, i2i.input, num_data, gain);
            break;
        case TestType::I2F :
            functional(bblib_int16_to_float_agc_avx512, "AVX512",
                        i2f.output, i2f.input, num_data, gain);
            break;
        case TestType::F2I :
            functional(bblib_float_to_int16_agc_avx512, "AVX512",
                        f2i.output, f2i.input, num_data, gain);
            break;
        case TestType::F2I_TH :
            functional(bblib_float_to_int16_agc_threshold_avx512, "AVX512",
                        f2i_th.output, f2i_th.input, num_data, gain, threshold);
            break;
        case TestType::I2I_FXP :
            functional(bblib_int16_to_int16_fxp_scale_avx512, "AVX512",
                       i2i_fxp.output, i2i_fxp.input, num_data, gain);
            break;
        }
}
#endif

INSTANTIATE_TEST_CASE_P(UnitTest, FloatFixConvertCheck,
                        testing::ValuesIn(get_sequence(FloatFixConvertCheck::get_number_of_cases("float_fix_functional"))));
