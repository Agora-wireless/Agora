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
#include <stdio.h>
#include <iostream>


const std::string module_name = "crc";

/* Parent class for CRC Generation Performance Tests */
class CrcPerf : public KernelTests
{
protected:
    struct bblib_crc_request request{};
    struct bblib_crc_response response{};

    void Generic_setup(const std::string& test_type)
    {
        /* Initialise tests based on test suite name in conf file */
        init_test(test_type);

        /* Division factor can be used to calculate results for sepcific unit. For example if we
           process n subcarriers we may set division factor to n, so we'll get results per
           subcarrier. In SDK it'll be useful in QR Decomposition where AVX512 process 2 times
           more data then AVX2, hence in overall AVX512 is worse, but it's much better when we
           look on processing time per matrix. Division factor is 1 by default so it doesn't have
           to be set. It's set here as an example.*/
        set_division_factor(1.0);

        /* buffer size defined as the maximum size of all inputs/outputs in BYTE */
        const int buffer_size = 1024*1024;

        /* Setup parameters for CRC performance tests*/
        /* Load data stream and its length from gen_functional section of .jason file */
        request.len =  get_input_parameter<uint32_t>("data_length");
        request.data = get_input_parameter<uint8_t*>("data_in");

        request.data = aligned_malloc<uint8_t>(buffer_size, 64);
        uint8_t* data_in_temp;
        if (request.len > 0) {
            data_in_temp = get_input_parameter<uint8_t*>("data_in");
            std::copy(data_in_temp, data_in_temp + request.len, request.data);
            aligned_free(data_in_temp);
        }

        response.data = request.data;
    }

    void TearDown() override
    {
        /* get_input_parameter<pointer_type>, generate_random_data<T> and
           aligned_malloc<T> allocate memory that has to be freed. */
        aligned_free(request.data);
    }
};


/* Child class for CRC24A Generation function performance, using parent class CrcPerf */
class Crc24AGeneratePerf : public CrcPerf
{
    void SetUp() override
    {
        /* Init crc24a generate performance tests */
        Generic_setup("performance_generate_crc24a");
    }
};


/* Child class for CRC24B Generation function performance, using parent class CrcPerf */
class Crc24BGeneratePerf : public CrcPerf
{
   void SetUp() override
    {
        /* Init crc24b generate performance tests */
        Generic_setup("performance_generate_crc24b");
    }
};


/* Child class for CRC24C Generation function performance, using parent class CrcPerf */
class Crc24CGeneratePerf : public CrcPerf
{
   void SetUp() override
    {
       /* Init crc24c generate performance tests */
        Generic_setup("performance_generate_crc24c");
    }
};


/* Child class for CRC24C init with 1's Generation function performance, using parent class CrcPerf */
class Crc24C_1_GeneratePerf : public CrcPerf
{
   void SetUp() override
    {
       /* Init crc24c_1 init with 1's generate performance tests */
        Generic_setup("performance_generate_crc24c_1");
    }
};


/* Child class for CRC16 Generation function performance, using parent class CrcPerf */
class Crc16GeneratePerf : public CrcPerf
{
   void SetUp() override
    {
       /* Init crc16 generate performance tests */
        Generic_setup("performance_generate_crc16");
    }
};


/* Child class for CRC11 Generation function performance, using parent class CrcPerf */
class Crc11GeneratePerf : public CrcPerf
{
   void SetUp() override
    {
       /* Init crc11 generate performance tests */
        Generic_setup("performance_generate_crc11");
    }
};


/* Child class for CRC6 Generation function performance, using parent class CrcPerf */
class Crc6GeneratePerf : public CrcPerf
{
   void SetUp() override
    {
       /* Init crc6 generate performance tests */
        Generic_setup("performance_generate_crc6");
    }
};


/* Child class for CRC24A Check function performance, using parent class CrcPerf */
class Crc24AValidatePerf : public CrcPerf
{
    void SetUp() override
    {
        /* Init crc24a check performance tests */
        Generic_setup("performance_validate_crc24a");
    }
};


/* Child class for CRC24B Check function performance, using parent class CrcPerf */
class Crc24BValidatePerf : public CrcPerf
{
    void SetUp() override
    {
        /* Init crc24b check performance tests */
        Generic_setup("performance_validate_crc24b");
    }
};


/* Child class for CRC24C Check function performance, using parent class CrcPerf */
class Crc24CValidatePerf : public CrcPerf
{
    void SetUp() override
    {
        /* Init crc24c check performance tests */
        Generic_setup("performance_validate_crc24c");
    }
};


/* Child class for CRC24C init with 1's Check function performance, using parent class CrcPerf */
class Crc24C_1_ValidatePerf : public CrcPerf
{
    void SetUp() override
    {
        /* Init crc24c init with 1's check performance tests */
        Generic_setup("performance_validate_crc24c_1");
    }
};


/* Child class for CRC16 Check function performance, using parent class CrcPerf */
class Crc16ValidatePerf : public CrcPerf
{
    void SetUp() override
    {
        /* Init crc16 check performance tests */
        Generic_setup("performance_validate_crc16");
    }
};


/* Child class for CRC11 Check function performance, using parent class CrcPerf */
class Crc11ValidatePerf : public CrcPerf
{
    void SetUp() override
    {
        /* Init crc11 check performance tests */
        Generic_setup("performance_validate_crc11");
    }
};


/* Child class for CRC6 Check function performance, using parent class CrcPerf */
class Crc6ValidatePerf : public CrcPerf
{
    void SetUp() override
    {
        /* Init crc6 check performance tests */
        Generic_setup("performance_validate_crc6");
    }
};


/* Child class for test function performance, using parent class CrcPerf */
class TestValidatePerf : public CrcPerf
{
    void SetUp() override
    {
        /* Init performance tests */
        Generic_setup("performance_validate_test");
    }
};


/* CRC SSE Generation Performance Tests*/
#if defined(_BBLIB_SSE4_2_) || defined(_BBLIB_AVX2_) || defined(_BBLIB_AVX512_)
TEST_P(Crc24AGeneratePerf, Gen_CRC24A_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc24a_gen_sse, &request, &response);
}

TEST_P(Crc24AValidatePerf, Val_CRC24A_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc24a_check_sse, &request, &response);
}

TEST_P(Crc24BGeneratePerf, Gen_CRC24B_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc24b_gen_sse, &request, &response);
}

TEST_P(Crc24BValidatePerf, Val_CRC24B_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc24b_check_sse, &request, &response);
}

TEST_P(Crc16GeneratePerf, Gen_CRC16_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc16_gen_sse, &request, &response);
}

TEST_P(Crc16ValidatePerf, Val_CRC16_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc16_check_sse, &request, &response);
}

TEST_P(Crc11GeneratePerf, Gen_CRC11_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc11_gen_sse, &request, &response);
}

TEST_P(Crc11ValidatePerf, Val_CRC11_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc11_check_sse, &request, &response);
}

TEST_P(Crc6GeneratePerf, Gen_CRC6_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc6_gen_sse, &request, &response);
}

TEST_P(Crc6ValidatePerf, Val_CRC6_SSE_Perf)
{
    performance("SSE", module_name, bblib_lte_crc6_check_sse, &request, &response);
}
#endif


/* CRC AVX512 Generation Performance Tests*/
#ifdef _BBLIB_AVX512_
TEST_P(Crc24AGeneratePerf, Gen_CRC24A_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc24a_gen_avx512, &request, &response);
}

TEST_P(Crc24AValidatePerf, Val_CRC24A_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc24a_check_avx512, &request, &response);
}

TEST_P(Crc24BGeneratePerf, Gen_CRC24B_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc24b_gen_avx512, &request, &response);
}

TEST_P(Crc24BValidatePerf, Val_CRC24B_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc24b_check_avx512, &request, &response);
}

TEST_P(Crc24CGeneratePerf, Gen_CRC24C_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc24c_gen_avx512, &request, &response);
}

TEST_P(Crc24CValidatePerf, Val_CRC24C_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc24c_check_avx512, &request, &response);
}

TEST_P(Crc24C_1_GeneratePerf, Gen_CRC24C_1_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc24c_1_gen_avx512, &request, &response);
}

TEST_P(Crc24C_1_ValidatePerf, Val_CRC24C_1_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc24c_1_check_avx512, &request, &response);
}

TEST_P(Crc16GeneratePerf, Gen_CRC16_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc16_gen_avx512, &request, &response);
}

TEST_P(Crc16ValidatePerf, Val_CRC16_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc16_check_avx512, &request, &response);
}

TEST_P(Crc11GeneratePerf, Gen_CRC11_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc11_gen_avx512, &request, &response);
}

TEST_P(Crc11ValidatePerf, Val_CRC11_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc11_check_avx512, &request, &response);
}

TEST_P(Crc6GeneratePerf, Gen_CRC6_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc6_gen_avx512, &request, &response);
}

TEST_P(Crc6ValidatePerf, Val_CRC6_AVX512_Perf)
{
    performance("AVX512", module_name, bblib_lte_crc6_check_avx512, &request, &response);
}

#endif

INSTANTIATE_TEST_CASE_P(UnitTest, Crc24AGeneratePerf,
                        testing::ValuesIn(get_sequence(Crc24AGeneratePerf::get_number_of_cases("performance_generate_crc24a"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc24BGeneratePerf,
                        testing::ValuesIn(get_sequence(Crc24BGeneratePerf::get_number_of_cases("performance_generate_crc24b"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc24CGeneratePerf,
                        testing::ValuesIn(get_sequence(Crc24CGeneratePerf::get_number_of_cases("performance_generate_crc24c"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc24C_1_GeneratePerf,
                        testing::ValuesIn(get_sequence(Crc24C_1_GeneratePerf::get_number_of_cases("performance_generate_crc24c_1"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc16GeneratePerf,
                        testing::ValuesIn(get_sequence(Crc16GeneratePerf::get_number_of_cases("performance_generate_crc16"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc11GeneratePerf,
                        testing::ValuesIn(get_sequence(Crc11GeneratePerf::get_number_of_cases("performance_generate_crc11"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc6GeneratePerf,
                        testing::ValuesIn(get_sequence(Crc6GeneratePerf::get_number_of_cases("performance_generate_crc6"))));

INSTANTIATE_TEST_CASE_P(UnitTest, Crc24AValidatePerf,
                        testing::ValuesIn(get_sequence(Crc24AValidatePerf::get_number_of_cases("performance_validate_crc24a"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc24BValidatePerf,
                        testing::ValuesIn(get_sequence(Crc24BValidatePerf::get_number_of_cases("performance_validate_crc24b"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc24CValidatePerf,
                        testing::ValuesIn(get_sequence(Crc24CValidatePerf::get_number_of_cases("performance_validate_crc24c"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc24C_1_ValidatePerf,
                        testing::ValuesIn(get_sequence(Crc24C_1_ValidatePerf::get_number_of_cases("performance_validate_crc24c_1"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc16ValidatePerf,
                        testing::ValuesIn(get_sequence(Crc16ValidatePerf::get_number_of_cases("performance_validate_crc16"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc11ValidatePerf,
                        testing::ValuesIn(get_sequence(Crc11ValidatePerf::get_number_of_cases("performance_validate_crc11"))));
INSTANTIATE_TEST_CASE_P(UnitTest, Crc6ValidatePerf,
                        testing::ValuesIn(get_sequence(Crc6ValidatePerf::get_number_of_cases("performance_validate_crc6"))));
