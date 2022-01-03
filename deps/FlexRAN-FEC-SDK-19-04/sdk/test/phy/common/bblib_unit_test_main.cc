/*******************************************************************************
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
 *******************************************************************************/
#ifdef _BBLIB_DPDK_
#include <rte_config.h>
#include <rte_eal.h>
#include <rte_debug.h>
#endif

#include "common.hpp"

static int parse_input_parameter(std::string executable, std::string option)
{
    std::size_t delim_pos = option.find("=");
    std::string param = option.substr(delim_pos + 1);

    try
    {
        return std::stoi(param);
    }
    catch(std::logic_error &e)
    {
        std::cout << executable << ": invalid argument '"<< param << "' for '" << option  << "'" << std::endl;
        std::cout << "Try '" << executable << " --usage' for more information." << std::endl;
        exit(-1);
    }
}

int main(int argc, char** argv) {
#ifdef _BBLIB_DPDK_
    /* Define DPDK init parameter "-n 4", n - set the number of memory channels to use */
    int rte_argc = 3;
    char* rte_argv[] = {"","-n","4", NULL};
    rte_argv[0] = argv[0];
    /* Init DPDK EAL needed to uses memory allocation from hugepages. */
    int ret_value = rte_eal_init(rte_argc, rte_argv);
    if (ret_value < 0)
    {
        rte_panic("Cannot init EAL\n");
        return -1;
    }
#endif


    /* Enable xml output by default */
    ::testing::GTEST_FLAG(output) = "xml:test_results.xml";

    ::testing::InitGoogleTest(&argc, argv);

    const std::string executable = argv[0];

    for(int index = 1; index < argc; index++) {

        const std::string option = argv[index];

        if (option.find("--nb_repetitions=") != std::string::npos)
        {
            BenchmarkParameters::repetition = parse_input_parameter(executable, option);
        }
        else if (option.find("--nb_loops=") != std::string::npos)
        {
            BenchmarkParameters::loop = parse_input_parameter(executable, option);
        }
        else if (option.find("--cpu_nb=") != std::string::npos)
        {
            BenchmarkParameters::cpu_id = (unsigned) parse_input_parameter(executable, option);

            if (BenchmarkParameters::cpu_id == 0)
                std::cout << std::endl << "Warning: Core #0 is running the VM's OS. "
                          << "Measurements won't be accurate. It shouldn't be used!"
                          << std::endl << std::endl;
        }
        /* --usage used instead of --help to avoid conflict with gtest --help */
        else if (!option.compare("--usage"))
        {
            std::cout << "Usage: " << executable << " [GTEST_OPTION]... [OPTION]..." << std::endl;
            std::cout << "Runs unittests with given gtest options." << std::endl;
            std::cout << std::endl;
            std::cout << "Available options: " << std::endl;
            std::cout << "--nb_repetitions=NUMBER Sets how many times results are measured" << std::endl;
            std::cout << "--nb_loops=NUMBER Sets how many times function is called per repetition"
                      << std::endl;
            std::cout << "--cpu_nb=NUMBER Sets core number to run tests on" << std::endl;
            std::cout << "--usage Prints this message" << std::endl;

            return 0;
        }
        else
        {
            std::cout << executable << ": inavlid option " << option << std::endl;
            std::cout << "Try '" << executable << " --usage' for more information." << std::endl;

            return -1;
        }
    }

    return RUN_ALL_TESTS();
}



