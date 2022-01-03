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
/*! \file   sdk_version.h
    \brief  This file stores the SDK version number reported by the libraries.
*/

#ifndef _SDK_VERSION_H_
#define _SDK_VERSION_H_

#include <stdint.h>

#include "common_typedef_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BBLIB_SDK_VERSION_STRING_MAX_LEN 150

/*! \brief Fill in the buffer_size long string array pointed by buff with the version string
           pointed by version.
    \param buffer Output buffer.
    \param version Version string.
    \param buffer_size Size of the buffer.
    \return 0 if success, else -1.
*/
int16_t
bblib_sdk_version(char **buffer, const char **version, int buffer_size);


/*! \brief Print the version string of the SDK common library */
void bblib_print_common_version();

#ifdef __cplusplus
}
#endif

#endif /* #ifndef _SDK_VERSION_H_ */

