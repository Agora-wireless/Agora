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

/*
 * @file   sdk_version.cpp
 * @brief  Report versions of all SDK modules
*/

#include <stdint.h>
#include <string.h>
#include "sdk_version.h"

int16_t
bblib_sdk_version(char **buffer, const char **version, int buffer_size)
{
    /* Check that the version string is set and that the buffer is
       sufficiently large */
    if (buffer_size < 1)
        return -1;

    if (!*version || (buffer_size <= strlen(*version))) {
        strncpy(*buffer, "", buffer_size-1);
        return -1;
    }

    strncpy(*buffer, *version, buffer_size-1);
    return 0;
}


struct bblib_common_init
{
    bblib_common_init()
    {
        bblib_print_common_version();
    }
};

bblib_common_init do_constructor_common;




int16_t
bblib_common_version(char *version, int buffer_size)
{
    /* The version string will be updated before the build process starts by the
     *       jobs building the library and/or preparing the release packages.
     *       Do not edit the version string manually */
    const char *msg = "FlexRAN SDK bblib_common version sdk-19.04-ea1-1-g3be2380";

    return(bblib_sdk_version(&version, &msg, buffer_size));
}

void
bblib_print_common_version()
{
    static bool was_executed = false;
    if(!was_executed) {
        was_executed = true;
        char version[BBLIB_SDK_VERSION_STRING_MAX_LEN] = { };
        bblib_common_version(version, sizeof(version));
        printf("%s\n", version);
    }
}
