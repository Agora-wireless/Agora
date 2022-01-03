// INTEL CONFIDENTIAL
// Copyright 2009-2019 Intel Corporation All Rights Reserved.
// 
// The source code contained or described herein and all documents related to the
// source code ("Material") are owned by Intel Corporation or its suppliers or
// licensors. Title to the Material remains with Intel Corporation or its
// suppliers and licensors. The Material may contain trade secrets and proprietary
// and confidential information of Intel Corporation and its suppliers and
// licensors, and is protected by worldwide copyright and trade secret laws and
// treaty provisions. No part of the Material may be used, copied, reproduced,
// modified, published, uploaded, posted, transmitted, distributed, or disclosed
// in any way without Intel's prior express written permission.
// 
// No license under any patent, copyright, trade secret or other intellectual
// property right is granted to or conferred upon you by disclosure or delivery
// of the Materials, either expressly, by implication, inducement, estoppel or
// otherwise. Any license under such intellectual property rights must be
// express and approved by Intel in writing.
// 
// Unless otherwise agreed by Intel in writing, you may not remove or alter this
// notice or any other notice embedded in Materials by Intel or Intel's suppliers
// or licensors in any way.
// 
//  version: SDK-jenkins-FlexRAN-SDK-REL-448-g3be238

#include "pseudo_random_seq_gen.h"

/*
 * Pseudo-random sequence generation as defined in TS38.211 section 5.2
 */

void
bblib_prbs_basic (const bblib_prbs_request* request, bblib_prbs_response* response)
{
    uint32_t N = (uint32_t)request->num_bits;
    response->num_bits = request->num_bits;
    uint32_t cInit = (uint32_t)request->c_init;
    //-31 since there bits are already known from init seq, +27 to make it a ceiling calc
    uint32_t k_ScramSeqInitNumLoop = (request->gold_code_advance - 31 + 30) / 28;
    uint32_t X1 = 1581799488; // Will get modified if not 1600 advance
    uint32_t X2;
    uint32_t k_Lsb28BitMask = 0x0FFFFFFF;
    uint32_t nOutputs = 0;

   /*
    * initialize bit0 ~ bit30 of m sequence x2,bit0 is located
    * in the MSB bit, bit1 is located in the second MSB bit,...etc,bit30
    * is located the second LSB bit,the LSB bit is 0.
    */
    uint32_t  tempX2;
    uint32_t  x2 = cInit;
    uint32_t  x2_0 = x2;           /* x2(n)  :bit0~bit30,with zero at MSB bit */
    uint32_t  x2_1 = x2 >> 1;      /* x2(n+1):bit1~bit30,with zeros at 2 MSB bit */
    uint32_t  x2_2 = x2 >> 2;      /* x2(n+2):bit2~bit30,with zeros at 3 MSB bits */
    uint32_t  x2_3 = x2 >> 3;      /* x2(n+3):bit3~bit30,with zeros at 4 MSB bits */

   /*
    * For each loop, 28 bits of m sequence x2 are calculated, so there
    * are 57 = 1600/28 loops.
    */
    uint32_t  idxLoop;
    for (idxLoop = 0; idxLoop < k_ScramSeqInitNumLoop; idxLoop++)
    {
       /*
        * calculate the following 28 bits of m sequence x2 through x2(n+31)=
        * (x2(n+3)+x2(n+2)+x2(n+1)+x2(n))mod2(n=0,1,...).For example, for the
        * first iteration, bit31~bit58 are calculated and stored in tempX2
        */
        tempX2 = x2_0 ^ x2_1 ^ x2_2 ^ x2_3;

       /*
        * in order to calculate the next 28 bits of m sequence x2 in next loop,
        * combine the last 3 bits of 31 bits in x2  and new generated 28 bit data.
        * For example, for  the first iteration, bit28~bit30 and new generated
        * bit31~bit58 are combined to generate the following 28 bits in next loop
        */
        x2 = ((x2 << 1) >> 29) | (tempX2 << 3);
        x2_0 = x2;       /* x2(n),the LSB bit is null and not to use */
        x2_1 = x2 >> 1;  /* x2(n+1),the LSB bit is null and not to use */
        x2_2 = x2 >> 2;  /* x2(n+2),the LSB bit is null and not to use */
        x2_3 = x2 >> 3;  /* x2(n+3),the LSB bit is null and not to use */
    }

    /* so far have up to bit 1626 (31+57*28 bits), calculate bit1600 ~ bit1631 and output
     * Need to generalize to any request->goldCodeAdvance */
    tempX2 = x2_0 ^ x2_1 ^ x2_2 ^ x2_3;
    uint32_t shiftAmt = request->gold_code_advance + 31 - (31 + 28 * k_ScramSeqInitNumLoop) + 1;
    X2 = ((x2 << 1) >> shiftAmt) + (tempX2 << (32 - shiftAmt));

    if (request->gold_code_advance != 1600)
    {
        uint32_t  tempX1;
        uint32_t  x1 = 0x00000001; //1 in LSB
        uint32_t  x1_0 = x1;
        uint32_t  x1_3 = x1 >> 3;
        for (idxLoop = 0; idxLoop < k_ScramSeqInitNumLoop; idxLoop++)
        {
           /*
            * calculate the following 28 bits of m sequence x2 through x2(n+31)=
            * (x2(n+3)+x2(n+2)+x2(n+1)+x2(n))mod2(n=0,1,...).For example, for the
            *  first iteration, bit31~bit58 are calculated and stored in tempX2
            */
            tempX1 = x1_0 ^ x1_3;

           /*
            * in order to calculate the next 28 bits of m sequence x2 in next loop,
            * combine the last 3 bits of 31 bits in x2  and new generated 28 bit data.
            * For example, for  the first iteration, bit28~bit30 and new generated
            * bit31~bit58 are combined to generate the following 28 bits in next loop
            */
            x1 = ((x1 << 1) >> 29) | (tempX1 << 3);
            x1_0 = x1;     /* x1(n),the LSB bit is null and not to use */
            x1_3 = x1 >> 3;  /* x1(n+3),the LSB bit is null and not to use */
        }

       /* calculate bit1600 ~ bit1631 and output */
        tempX1 = x1_0 ^ x1_3;
        X1 = ((x1 << 1) >> shiftAmt) + (tempX1 << (32 - shiftAmt));
    }

    uint32_t numLoop = (N + 27) / 28;
    for (uint32_t idxLoop = 0; idxLoop < numLoop; idxLoop++)
    {

        uint32_t x1_0 = X1;       /* x1(n) */
        uint32_t x1_3 = X1 >> 3;  /* x1(n+3) */
        uint32_t x2_0 = X2;       /* x2(n) */
        uint32_t x2_1 = X2 >> 1;  /* x2(n+1) */
        uint32_t x2_2 = X2 >> 2;  /* x2(n+2) */
        uint32_t x2_3 = X2 >> 3;  /* x2(n+3) */
        /* 28-bits data of pseudo-random sequence c(n) is calculated: c(n)=(x1(n+Nc)+x2(n+Nc))mod2 */
        uint32_t scramSeq = (X1 ^ X2) & k_Lsb28BitMask;
        uint32_t tempX1 = x1_0 ^ x1_3;               /* x1(n+31)=(x1(n+3)+x1(n))mod2*/
        uint32_t tempX2 = x2_0 ^ x2_1 ^ x2_2 ^ x2_3; /* x2(n+31)=(x2(n+3)+x2(n+2)+x2(n+1)+x2(n))mod2*/
        /*
         * in each loop,28-bits m sequence x1 and x2 is used,the unused 3 bit and the next 28 bits x1
         * and x2 are combined to generate the next 31 bit(28+3) sequence for x1 and x2 sequence
         */
        X1 = ((X1 << 1) >> 29) | (tempX1 << 3);
        X2 = ((X2 << 1) >> 29) | (tempX2 << 3);

        if ((idxLoop & 1) == 0)
        {
            /* if idxLoop is even, fill 3 whole bytes and half of next */
            response->bits[nOutputs] = (uint8_t)(scramSeq & 0xFF);
            response->bits[nOutputs + 1] = (uint8_t)((scramSeq >> 8) & 0xFF);
            response->bits[nOutputs + 2] = (uint8_t)((scramSeq >> 16) & 0xFF);
            response->bits[nOutputs + 3] = (uint8_t)((scramSeq >> 24) & 0xF);
            nOutputs += 3;
        }
        else
        {
            /* if idxLoop is odd, complete partial byte and 3 next bytes */
            response->bits[nOutputs] |= (uint8_t)((scramSeq & 0xF) << 4);
            response->bits[nOutputs + 1] = (uint8_t)((scramSeq >> 4) & 0xFF);
            response->bits[nOutputs + 2] = (uint8_t)((scramSeq >> 12) & 0xFF);
            response->bits[nOutputs + 3] = (uint8_t)((scramSeq >> 20) & 0xFF);
            nOutputs += 4;
        }
    }

    uint32_t nBytesOutput = (N + 7) >> 3;
    uint32_t unused_bits_in_final_byte = nBytesOutput * 8 - N;
    uint32_t tmp = response->bits[nBytesOutput - 1];
    tmp &= (0xFF >> unused_bits_in_final_byte);
    response->bits[nBytesOutput - 1] = (uint8_t)tmp;
}

