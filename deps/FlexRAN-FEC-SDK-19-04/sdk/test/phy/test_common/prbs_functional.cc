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

#include "pseudo_random_seq_gen.h"

const std::string module_name = "prbs";

class PRBSCheck : public KernelTests {
protected:
    struct bblib_prbs_request request {};
    struct bblib_prbs_response response {};
    struct bblib_prbs_response reference {};

    int case_num;
    uint32_t *input_vars;
    uint32_t *ref_output_num;
    uint8_t *ref_output_data;

    void SetUp() override {
        init_test("prbs_functional");

        /* Test vecotrs conain data for number of cases = case_num */
        case_num = get_input_parameter<int>("case_num");
        input_vars = get_input_parameter<uint32_t*>("input_vars");
        ref_output_num = get_reference_parameter<uint32_t*>("output_num");
        ref_output_data = get_reference_parameter<uint8_t*>("output_data");
    }

    void SetUpCase(uint16_t c_init, uint16_t gold_code_advance, uint16_t num_bits,
                       uint16_t ref_num_bits, uint8_t *ref_bits) {
        request.c_init = c_init;
        request.gold_code_advance = gold_code_advance;
        request.num_bits = num_bits;

        response.bits = aligned_malloc<uint8_t>(request.num_bits, 64);

        reference.bits = ref_bits;
        reference.num_bits = ref_num_bits;
    }

    void TearDown() override {
        aligned_free(input_vars);
        aligned_free(ref_output_num);
        aligned_free(ref_output_data);
    }

    void TearDownCase() {
        aligned_free(response.bits);
    }

    template <typename F>
    void functional(F function)
    {
        uint8_t *ref_bits = ref_output_data;

        /* Instead of loops separate test cases should be defined in conf.json
           Loop lkept after refactoring due to common test vectors (for all test cases)
           and large number of cases.*/
        for (int test_case = 0; test_case < case_num; test_case++) {
            /* input_vars is a table of 3 parameters: c_init, gold_code_advance, num_bits */
            SetUpCase((uint16_t)input_vars[test_case * 3],
                      (uint16_t)input_vars[test_case * 3 + 1],
                      (uint16_t)input_vars[test_case * 3 + 2],
                      (uint16_t)ref_output_num[test_case],
                      ref_bits);
            function(&request, &response);

            ASSERT_EQ(response.num_bits, reference.num_bits);

            unsigned output_len_bytes = response.num_bits / 8;
            if (response.num_bits % 8) //ceiling operator
                output_len_bytes++;

            ASSERT_ARRAY_EQ(reference.bits, response.bits, output_len_bytes);

            ref_bits += output_len_bytes;
            TearDownCase();
        }
    }
};

TEST_P(PRBSCheck, C_Check)
{
    functional(bblib_prbs_basic);
}

INSTANTIATE_TEST_CASE_P(UnitTest, PRBSCheck,
                        testing::ValuesIn(get_sequence(PRBSCheck::get_number_of_cases("prbs_functional"))));