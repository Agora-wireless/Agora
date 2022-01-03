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
/*! \file bblib_common.hpp
    \brief  Common header file containing helper functions used throughout the
     BBLIB SDK
*/

#ifndef _BBLIB_COMMON_HPP_
#define _BBLIB_COMMON_HPP_


#include <cmath>
#include <fstream>
#include <numeric>
#include <vector>
#include <stdexcept>
#include <cstdlib>

#ifndef _WIN64
#include <unistd.h>
#include <sys/syscall.h>
#else
#include <Windows.h>
#endif


/* Common helper functions */

/*!
    \brief Reads binary input data from file within the SDK ROOT DIR

    \param [in] filename string containing the path and file name of the file
                to read. The path should start from the SDK ROOT DIR
    \param [in] output_buffer buffer with enough memory allocation to store the
                data read from filename.
    \return 0 on success
*/
int bblib_common_read_binary_data(const std::string filename, char * output_buffer);





struct reading_input_file_exception : public std::exception
{
    const char * what () const throw () override {
        return "Input file cannot be read!";
    }
};

int bblib_common_read_binary_data(const std::string filename, char  * output_buffer) {


    //std::string sdk_dir = getenv("DIR_WIRELESS_SDK");

    char *sdk_dir = getenv("DIR_WIRELESS_SDK");
    if(sdk_dir == NULL) {
        throw std::runtime_error("Failed to get environment variable DIR_WIRELESS_SDK!");
        return -1;
    }

    std::string inputfile = std::string(sdk_dir) + filename;

    std::ifstream input_stream(inputfile, std::ios::binary);
    std::vector<char> buffer((std::istreambuf_iterator<char>(input_stream)),
                                  std::istreambuf_iterator<char>());
    if(buffer.size() == 0)
        throw reading_input_file_exception();

    if(buffer.size() < sizeof(output_buffer))
        throw std::runtime_error("Input file error");

    std::copy(buffer.begin(), buffer.end(), output_buffer);


    return 0;
}


#endif /* #ifndef _BBLIB_COMMON_HPP_ */

