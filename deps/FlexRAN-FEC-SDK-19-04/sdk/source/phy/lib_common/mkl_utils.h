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
******************************************************************************/
/**
 * @brief This file consists of utilities using MKL (Math Kernel Libraries)
 * @file lte_bs_mkl_utils.h
 * @ingroup group_lte_source_phy_utils
 * @author Intel Corporation
 *
 **/


#ifndef _MKL_UTILS_H_
#define _MKL_UTILS_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <math.h>
#include <mkl.h>
#include <mkl_dfti.h>

#define MKL_UTILS_NUM_IDFT_SIZES            ( 34 )
#define MKL_UTILS_NUM_FFT_SIZES             ( 2 )
#define MKL_UTILS_NUM_IDX_1024              ( 0 )
#define MKL_UTILS_NUM_IDX_2048              ( 1 )

#define MKL_FFT                             ( 0 )
#define MKL_IFFT                            ( 1 )
#define MKL_IDFT                            ( 2 )
#define MKL_PRACH                           ( 3 )

#define MKL_FORWARD                         ( 0 )
#define MKL_BACKWARD                        ( 1 )

__int32 mkl_utils_init_fft_ifft(__int32 Nfft);
__int32 mkl_utils_destroy_fft_ifft(void);
MKL_LONG mkl_utils_run_fft_ifft(__int32 Nfft, __int32 type, __int32 direction, void *pIn, void *pOut);

__int32 mkl_utils_init_idft(void);
__int32 mkl_utils_destroy_idft(void);
MKL_LONG mkl_utils_run_idft(__int32 idx, __int32 type, __int32 direction, void *pIn, void *pOut);

#endif /* #ifndef _LTE_BS_MKL_UTILS_H_ */

