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
#include "phy_crc.h"

#include <stdint.h>


const std::string module_name = "crc";


/* This class of checks, tests both the CRC generation and CRC check
 * functions for each data stream scenario defined in the functional section
 * of the conf.json file.
 * The generation test (functional_gen) requires:
 * param [in]  request.data to point to the test data stream
 *             request.len to be the length of this data stream (bytes)
 *             also response.data should be configured to point to request.data
 * param [out] response.crc_value is the generated CRC value
 *             response.data is the CRC appended data stream
 *             response.len is the new length of the CRC appended data (bytes)
 * The check test (functional_chk) requires:
 * param [in]  request.data to point to CRC appended data stream. For these tests
 *             it uses the output (response.data) of the generation test.
 *             request.len to be the length of this data stream
 * param [out] response.check_passed is true if the CRC appended data is correct.
 */
class CrcByteLenGenerationCheck : public KernelTests
{
protected:
    struct bblib_crc_request request {};
    struct bblib_crc_response response {};
    struct bblib_crc_response reference {};

    void SetUp() override
    {
        /* Parameters stored in the functional section will be used. GTest will call
           TEST_P (including SetUp and TearDown) for each case in the section. */
        init_test("crc_byte_len_functional");

        /* buffer size defined as the maximum size of all inputs/outputs in BYTE */
        const int buffer_size = 1024*1024;

        /* Setup parameters for CRC generation & check test functions*/
        /* Load data stream and its length from gen_functional section of .jason file */
        request.len =  get_input_parameter<uint32_t>("data_length");
		
        request.data = aligned_malloc<uint8_t>(buffer_size, 64);

        uint8_t* data_in_temp;
        if (request.len > 0) {
            data_in_temp = get_input_parameter<uint8_t*>("data_in");
            std::copy(data_in_temp, data_in_temp + request.len, request.data);
            aligned_free(data_in_temp);
        }

        /* Set response data to point to request data */
        response.data = request.data;
        response.crc_value = 0;
        response.check_passed = false;
        /* set reference value for functional_chk test */
        reference.check_passed = get_reference_parameter<bool>("check_passed");

    }

    /* It's called after an execution of the each test case.*/
    void TearDown() override
    {
        aligned_free(request.data);
    }

    /* Call CRC generation kernel & confirm the correct crc_value is produced,
     * when compared with the relevant reference crc_value setup in the test
     * and defined in the conf.json file.
     */
    template <typename F>
    void functional_gen(F function, const std::string isa)
    {
        function(&request, &response);
        ASSERT_EQ(response.crc_value, reference.crc_value) << "FAIL: Generated CRC value does not compare with reference";
        print_test_description(isa, module_name);
    }

    /* Call the CRC check kernel, using the CRC appended data result from the
     * generation kernel and confirm it's validity
     */
    template <typename F>
    void functional_chk(F function, const std::string isa)
    {
        /* Note: request.data is configured to be the result of response.data by a
         * previous call to the generation function.
         * request.len is the length of this data minus the CRC value.
         */
        function(&request, &response);
        ASSERT_EQ(response.check_passed, reference.check_passed) << "FAIL: CRC check failed";
        print_test_description(isa, module_name);
    }
};



class CrcBitLenGenerationCheck : public KernelTests
{
protected:
    struct bblib_crc_request request {};
    struct bblib_crc_response response {};
    struct bblib_crc_response reference {};

    void SetUp() override
    {
        /* Parameters stored in the functional section will be used. GTest will call
           TEST_P (including SetUp and TearDown) for each case in the section. */
        init_test("crc_bit_len_functional");

        /* buffer size defined as the maximum size of all inputs/outputs in BYTE */
        const int buffer_size = 1024*1024;

        /* Setup parameters for CRC generation & check test functions*/
        /* Load data stream and its length from gen_functional section of .jason file */
        request.len =  get_input_parameter<uint32_t>("data_length");

        request.data = aligned_malloc<uint8_t>(buffer_size, 64);
        uint8_t* data_in_temp;
        if (request.len > 0) {
            data_in_temp = get_input_parameter<uint8_t*>("data_in");
            std::copy(data_in_temp, data_in_temp + request.len, request.data);
            aligned_free(data_in_temp);
        }

        /* setup parameters for CRC generation & check test functions*/
        /* Load data stream and its length from gen_functional section of .jason file */
        /* Set response data to point to request data */
        response.data = request.data;
        response.crc_value = 0;
        response.check_passed = false;
        /* set reference value for functional_chk test */
        reference.check_passed = get_reference_parameter<bool>("check_passed");
    }

    /* It's called after an execution of the each test case.*/
    void TearDown() override
    {
        aligned_free(request.data);
    }

    /* Call CRC generation kernel & confirm the correct crc_value is produced,
     * when compared with the relevant reference crc_value setup in the test
     * and defined in the conf.json file.
     */
    template <typename F>
    void functional_gen(F function, const std::string isa)
    {
        function(&request, &response);
        ASSERT_EQ(response.crc_value, reference.crc_value) << "FAIL: Generated CRC value does not compare with reference";
        print_test_description(isa, module_name);
    }

    /* Call the CRC check kernel, using the CRC appended data result from the
     * generation kernel and confirm it's validity
     */
    template <typename F>
    void functional_chk(F function, const std::string isa)
    {
        /* Note: request.data is configured to be the result of response.data by a
         * previous call to the generation function.
         * request.len is the length of this data minus the CRC value.
         */
        function(&request, &response);
        ASSERT_EQ(response.check_passed, reference.check_passed) << "FAIL: CRC check failed";
        print_test_description(isa, module_name);
    }
};



/* CRC Generation & Check Tests for SSE supported CRCs (CRC24A, 24B & 16)*/

#if defined(_BBLIB_SSE4_2_) || defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)

TEST_P(CrcByteLenGenerationCheck, CRC24A_SSE)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24a_value");
    functional_gen(bblib_lte_crc24a_gen_sse, "SSE");
    functional_chk(bblib_lte_crc24a_check_sse, "SSE");
}

TEST_P(CrcByteLenGenerationCheck, CRC24B_SSE)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24b_value");
    functional_gen(bblib_lte_crc24b_gen_sse, "SSE");
    functional_chk(bblib_lte_crc24b_check_sse, "SSE");
}

TEST_P(CrcByteLenGenerationCheck, CRC16_SSE)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc16_value");
    functional_gen(bblib_lte_crc16_gen_sse, "SSE");
    functional_chk(bblib_lte_crc16_check_sse, "SSE");
}

TEST_P(CrcByteLenGenerationCheck, CRC11_SSE)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc11_value");
    functional_gen(bblib_lte_crc11_gen_sse, "SSE");
    functional_chk(bblib_lte_crc11_check_sse, "SSE");
}

TEST_P(CrcByteLenGenerationCheck, CRC6_SSE)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc6_value");
    functional_gen(bblib_lte_crc6_gen_sse, "SSE");
    functional_chk(bblib_lte_crc6_check_sse, "SSE");
}

#endif



/* CRC byte length based, AVX512, Generation & Check Tests*/
#ifdef _BBLIB_AVX512_

TEST_P(CrcByteLenGenerationCheck, CRC6_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc6_value");
    functional_gen(bblib_lte_crc6_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc6_check_avx512, "AVX512");
}

TEST_P(CrcByteLenGenerationCheck, CRC11_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc11_value");
    functional_gen(bblib_lte_crc11_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc11_check_avx512, "AVX512");
}

TEST_P(CrcByteLenGenerationCheck, CRC16_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc16_value");
    functional_gen(bblib_lte_crc16_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc16_check_avx512, "AVX512");
}

TEST_P(CrcByteLenGenerationCheck, CRC24A_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24a_value");
    functional_gen(bblib_lte_crc24a_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc24a_check_avx512, "AVX512");
}

TEST_P(CrcByteLenGenerationCheck, CRC24B_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24b_value");
    functional_gen(bblib_lte_crc24b_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc24b_check_avx512, "AVX512");
}

TEST_P(CrcByteLenGenerationCheck, CRC24C_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24c_value");
    functional_gen(bblib_lte_crc24c_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc24c_check_avx512, "AVX512");
}

TEST_P(CrcByteLenGenerationCheck, CRC24C_1_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24c_1_value");
    functional_gen(bblib_lte_crc24c_1_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc24c_1_check_avx512, "AVX512");
}

#endif


/* CRC bit length based, AVX512, Generation & Check Tests */
#ifdef _BBLIB_AVX512_

TEST_P(CrcBitLenGenerationCheck, CRC6_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc6_value");
    functional_gen(bblib_lte_crc6_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc6_check_avx512, "AVX512");
}

TEST_P(CrcBitLenGenerationCheck, CRC11_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc11_value");
    functional_gen(bblib_lte_crc11_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc11_check_avx512, "AVX512");
}

TEST_P(CrcBitLenGenerationCheck, CRC16_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc16_value");
    functional_gen(bblib_lte_crc16_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc16_check_avx512, "AVX512");
}

TEST_P(CrcBitLenGenerationCheck, CRC24A_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24a_value");
    functional_gen(bblib_lte_crc24a_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc24a_check_avx512, "AVX512");
}

TEST_P(CrcBitLenGenerationCheck, CRC24B_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24b_value");
    functional_gen(bblib_lte_crc24b_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc24b_check_avx512, "AVX512");
}

TEST_P(CrcBitLenGenerationCheck, CRC24C_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24c_value");
    functional_gen(bblib_lte_crc24c_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc24c_check_avx512, "AVX512");
}

TEST_P(CrcBitLenGenerationCheck, CRC24C_1_AVX512)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24c_1_value");
    functional_gen(bblib_lte_crc24c_1_gen_avx512, "AVX512");
    functional_chk(bblib_lte_crc24c_1_check_avx512, "AVX512");
}

#endif


/* Default Tests - Applicable across multiple ISA devices */

/* CRC byte length based, Default ISA, Generation & Check Tests
   Note, for Default ISA, can only run byte length based tests, since
   bit length based tests are only supported in avx512 code */

TEST_P(CrcByteLenGenerationCheck, CRC6_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc6_value");
    functional_gen(bblib_lte_crc6_gen, "Default");
    functional_chk(bblib_lte_crc6_check, "Default");
}

TEST_P(CrcByteLenGenerationCheck, CRC11_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc11_value");
    functional_gen(bblib_lte_crc11_gen, "Default");
    functional_chk(bblib_lte_crc11_check, "Default");
}

TEST_P(CrcByteLenGenerationCheck, CRC16_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc16_value");
    functional_gen(bblib_lte_crc16_gen, "Default");
    functional_chk(bblib_lte_crc16_check, "Default");
}

TEST_P(CrcByteLenGenerationCheck, CRC24A_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24a_value");
    functional_gen(bblib_lte_crc24a_gen, "Default");
    functional_chk(bblib_lte_crc24a_check, "Default");
}

TEST_P(CrcByteLenGenerationCheck, CRC24B_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24b_value");
    functional_gen(bblib_lte_crc24b_gen, "Default");
    functional_chk(bblib_lte_crc24b_check, "Default");
}


/* Default Tests - Applicable only in avx512 ISA devices */
#ifdef _BBLIB_AVX512_

/* CRC byte length based, Default ISA, Generation & Check Tests*/

TEST_P(CrcByteLenGenerationCheck, CRC24C_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24c_value");
    functional_gen(bblib_lte_crc24c_gen, "Default");
    functional_chk(bblib_lte_crc24c_check, "Default");
}

TEST_P(CrcByteLenGenerationCheck, CRC24C_1_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24c_1_value");
    functional_gen(bblib_lte_crc24c_1_gen, "Default");
    functional_chk(bblib_lte_crc24c_1_check, "Default");
}


/* CRC bit length based, Default ISA, Generation & Check Tests
   Note, bit length based tests are only supported on avx512 architecture */

TEST_P(CrcBitLenGenerationCheck, CRC6_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc6_value");
    functional_gen(bblib_lte_crc6_gen, "Default");
    functional_chk(bblib_lte_crc6_check, "Default");
}

TEST_P(CrcBitLenGenerationCheck, CRC11_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc11_value");
    functional_gen(bblib_lte_crc11_gen, "Default");
    functional_chk(bblib_lte_crc11_check, "Default");
}

TEST_P(CrcBitLenGenerationCheck, CRC16_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc16_value");
    functional_gen(bblib_lte_crc16_gen, "Default");
    functional_chk(bblib_lte_crc16_check, "Default");
}

TEST_P(CrcBitLenGenerationCheck, CRC24A_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24a_value");
    functional_gen(bblib_lte_crc24a_gen, "Default");
    functional_chk(bblib_lte_crc24a_check, "Default");
}

TEST_P(CrcBitLenGenerationCheck, CRC24B_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24b_value");
    functional_gen(bblib_lte_crc24b_gen, "Default");
    functional_chk(bblib_lte_crc24b_check, "Default");
}

TEST_P(CrcBitLenGenerationCheck, CRC24C_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24c_value");
    functional_gen(bblib_lte_crc24c_gen, "Default");
    functional_chk(bblib_lte_crc24c_check, "Default");
}

TEST_P(CrcBitLenGenerationCheck, CRC24C_1_Default_ISA)
{
    reference.crc_value = get_reference_parameter<uint32_t>("crc24c_1_value");
    functional_gen(bblib_lte_crc24c_1_gen, "Default");
    functional_chk(bblib_lte_crc24c_1_check, "Default");
}
#endif


INSTANTIATE_TEST_CASE_P(UnitTest, CrcByteLenGenerationCheck,
                        testing::ValuesIn(get_sequence(CrcByteLenGenerationCheck::get_number_of_cases("crc_byte_len_functional"))));

INSTANTIATE_TEST_CASE_P(UnitTest, CrcBitLenGenerationCheck,
                        testing::ValuesIn(get_sequence(CrcBitLenGenerationCheck::get_number_of_cases("crc_bit_len_functional"))));

