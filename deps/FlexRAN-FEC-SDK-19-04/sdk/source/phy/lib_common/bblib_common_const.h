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

/**
 * @file bblib_common_const.h
 * @brief This header file defines common global constants uses throughout the
 * bblib libraries.
 */

#ifndef _BBLIB_COMMON_CONST_
#define _BBLIB_COMMON_CONST_


/*!
    \enum bblib_common_const_wireless_params
    \brief This enum contains the common wireless constants accross both LTE
    and 5G used throughout the bblib libraries.
*/
enum bblib_common_const_wireless_params {
    BBLIB_N_SC_PER_PRB = 12,    /*!< Number of subcarriers in a Physical Resource Block */
    BBLIB_N_SYMB_PER_SF = 14 /*!< Number of symbols in sub-frame */
};

#endif /* _BBLIB_COMMON_CONST_ */



