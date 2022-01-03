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

/*******************************************************************************
*  @file phy_turboDecoder_MakeTable.cpp
*  @brief This file performs turbo decoder related table initilization
*  @author Zhang Senjie(senjie.zhang@intel.com)
*  @author Cao Jinyu(jinyu.cao@intel.com)
******************************************************************************/

/*******************************************************************************
 * Include private header files
 ******************************************************************************/
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "phy_turbo_internal.h"

/* Turbo table */
__align(64) uint16_t g_TurboBufAddr[1068000];
__align(64) int32_t g_TurboBufAddrOffset[188];
extern __align(64) uint16_t g_OutputWinTable8_sdk[256][8];


/* TS36.212 Table 5.1.3-3, Turbo code internal interleaver parameters with XXXX */
int32_t g_Kidx_K_Nmaxrep_shuf_sdk[188][5];

#define MAX_PATH_LEN 1024
#define C_TC_subblock 32

int8_t g_DataColumnPerm01_sdk[32] = {\
0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30, 1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31};
int8_t g_DataColumnPerm2_sdk[32] = {\
1, 17, 9, 25, 5, 21, 13, 29, 3, 19, 11, 27, 7, 23, 15, 31, 2, 18, 10, 26, 6, 22, 14, 30, 4, 20, 12, 28, 8, 24, 16, 0};

/*******************************************************************************
 * Function
 ******************************************************************************/
/** @fn InitTurboDecoderInterleaverTable
 *  @brief Turbo decoded used interleave table init
 *  @param [in] WORD8 * Table_Path
 *         [in out] _TurboInterleaver * p
 *  @return void
 */
void init_turbo_decoder_interleaver_table(char *Table_Path,
                                          _TurboInterleaver *p)
{
    if( (Table_Path == NULL) || (p == NULL) )
    {
        return;
    }

    int32_t i, j, re;
    FILE * fp;
    char interleavertable[] = "/source/phy/lib_turbo/TurboInterleaver.dat";
    char filename[MAX_PATH_LEN];

    if ((strlen(Table_Path) + strlen(interleavertable)) >= MAX_PATH_LEN)
    {
        printf("config invalid: tablePath = %s\n", Table_Path);
        return;
    }
    strncpy(filename, Table_Path, strlen(Table_Path) + 1);
    strncat(filename, interleavertable, strlen(interleavertable) + 1);

    fp = fopen(filename, "r");
    if (fp == NULL)
    {
        printf("Cannot open file\n");
        return;
    }
    for (i=0; i<188; i++)
    {
        fscanf(fp, "%d ", &re);
        p->offset[i] = re;
    }
    for (i=0; i<21693; i++)
    {
        fscanf(fp, "%d ", &re);
        p->inter_row_out_addr_for_interleaver[i] = re;
    }
    for (i=0; i<21693; i++)
    {
        fscanf(fp, "%d ", &re);
        p->inter_row_out_addr_for_deinterleaver[i] = re;
    }
    for (i=0; i<21693; i++)
    {
        fscanf(fp, "%d ", &re);
        p->intra_row_perm_pattern_for_interleaver[i] = re;
    }
    for (i=0; i<21693; i++)
    {
        fscanf(fp, "%d ", &re);
        p->intra_row_perm_pattern_for_deinterleaver[i] = re;
    }
    for (i=0; i<448; i++)
    {
        for (j=0; j<16; j++)
        {
            fscanf(fp, "%d ", &re);
            p->pattern[i][j] = (int8_t) re;
        }
    }
    fclose(fp);
    return;
}

static size_t read_table(char * table, void * ptr, size_t len)
{
    FILE * fp;
    size_t rlen;

    fp = fopen(table, "rb");
    if (fp == NULL)
    {
        printf("Invalid: failed to open %s\n", table);
        return(-1);
    }

    rlen = fread(ptr, 1, len, fp);
    fclose(fp);
    return rlen;
}

struct tableItem {
    void * tableptr;
    size_t len;
    char * tablenm;
};

int32_t init_common_tables(char *pTabelPath)
{
    #define TABLE_NR 2

    char filename[MAX_PATH_LEN];
    struct tableItem tables[TABLE_NR] = {
             {g_Kidx_K_Nmaxrep_shuf_sdk, sizeof(g_Kidx_K_Nmaxrep_shuf_sdk), "/source/phy/lib_turbo/kidx_k_nmaxrep_shuf.bin"},
             {g_OutputWinTable8_sdk, sizeof(g_OutputWinTable8_sdk), "/source/phy/lib_turbo/output_win_table8.bin"},
        };

    size_t lens[TABLE_NR], rlen;
    size_t maxlen = 0;
    int32_t i;

    for (i = 0; i < TABLE_NR; i++)
    {
        lens[i] = strlen(tables[i].tablenm);
        if (maxlen < lens[i])
        {
            maxlen = lens[i];
        }
    }

    if ((strlen(pTabelPath) + maxlen) >= MAX_PATH_LEN)

    {
        printf("config invalid: tablePath = %s\n", pTabelPath);
        return (-1);
    }

    for (i = 0; i < TABLE_NR; i++)
    {
        strncpy(filename, pTabelPath, strlen(pTabelPath) + 1);
        strncat(filename, tables[i].tablenm, lens[i] + 1);

        rlen = read_table(filename, tables[i].tableptr, tables[i].len);
        if (rlen != tables[i].len)
        {
            printf("table invalid: length of %s = %zd [%zd]\n", filename, rlen, tables[i].len);
            return(-1);
        }
    }
    return 0;
}

/** @fn GetTurboBufAddrTable_New
 *  @brief Turbo decoded related table init
 *  @param [in out] UWORD16 * p_table_TurboBufAddr
 *         [in out] WORD32 * p_table_TurboBufAddr_Offset
 *         [in out] WORD32 * p_table_Kidx_K_Nmaxrep_shuf
 *  @return void
 */
void get_turbo_buf_addr_table_new(uint16_t *p_table_TurboBufAddr,
                                  int32_t *p_table_TurboBufAddr_Offset,
                                  int32_t (*p_table_Kidx_K_Nmaxrep_shuf)[5])
{
    int32_t v0[6176], v1[6176], v2[6176];
    int32_t y0[6176], y1[6176], y2[6176];
    int32_t CircularBuf[18528];

    memset(y2, 0, sizeof(y2));

    int32_t i, j, k;
    int32_t K, D, Kblock, R_TC_subblock, Kpai, ND, Ncb, Kw, Kidx;

    int32_t bit, bit_RSC, bit_addr, row, col, Nw, addr_at_Turbo_input_struct=0;

    int32_t cnt = 0;
    int32_t cnt_good_bit;


    for (Kidx=1; Kidx<=188; Kidx++)
    {
        * (p_table_TurboBufAddr_Offset + Kidx-1) = cnt;
        /* --------------------------------------------------------------------------------------------------*/
        /* code block size related parameters */
        /*K = *(p_table_Kidx_K_Nmaxrep_shuf + ((((Kidx-1)*5 + 1)<188*5)?((Kidx-1)*5 + 1):0)); */
        K = p_table_Kidx_K_Nmaxrep_shuf[Kidx-1][1];
        D = K + 4;
        Kblock = 3 * D;
        if (D%32!=0)
        {
            R_TC_subblock = D / C_TC_subblock + 1;
        }
        else
        {
            R_TC_subblock = D / C_TC_subblock;
        }
        Kpai = R_TC_subblock * C_TC_subblock;
        ND = Kpai - D;
        Kw = 3 * Kpai;
        Ncb = Kw;

        /* -------------------------------------------------------------------------------------------------- */
        /* fulfill buffer for subblock permuted matrix */
        for (i=0; i<ND; i++)
        {
            v0[i] = (-1); /* <NULL> */
            v1[i] = (-1); /* <NULL> */
            v2[i] = (-1); /* <NULL> */
        }
        for (i=ND; i<K+ND; i++)
        {
            v0[i] = i-ND; /* system bits */
            v1[i] = 10000 + (i-ND); /* parity bits 0 */
            v2[i] = 20000 + (i-ND); /* parity bits 1 */
        }
        for (i=K+ND; i<Kpai; i++)
        {
            v0[i] = 30000 + (i-K-ND); /* system tail bits */
            v1[i] = 40000 + (i-K-ND); /* parity tail bits 0 */
            v2[i] = 50000 + (i-K-ND); /* parity tail bits 1 */
        }

        /* subblock matrix column permutation */
        for (i=0; i<R_TC_subblock; i++)
        {
            for (j=0; j<C_TC_subblock; j++)
            {
                y0[i*C_TC_subblock+j] = v0[i*C_TC_subblock+g_DataColumnPerm01_sdk[j]];
                y1[i*C_TC_subblock+j] = v1[i*C_TC_subblock+g_DataColumnPerm01_sdk[j]];
                y2[i*C_TC_subblock+j] = v2[i*C_TC_subblock+g_DataColumnPerm2_sdk[j]];
            }
        }
        j = C_TC_subblock - 1;
        i = 0;
        k = y2[i*C_TC_subblock+j];
        for (i=0; i<(R_TC_subblock-1); i++) y2[i*C_TC_subblock+j] = y2[(i+1)*C_TC_subblock+j];
        y2[i*C_TC_subblock+j] = k;

        /* -------------------------------------------------------------------------------------------------- */
        /* obtain virtual circular buffer */
        i = 0;
        for (col=0; col<C_TC_subblock; col++)
        {
            for (row=0; row<R_TC_subblock; row++)
            {
                CircularBuf[i++] = y0[row*C_TC_subblock+col];
            }
        }
        for (col=0; col<C_TC_subblock; col++)
        {
            for (row=0; row<R_TC_subblock; row++)
            {
                CircularBuf[i++] = y1[row*C_TC_subblock+col];
                CircularBuf[i++] = y2[row*C_TC_subblock+col];
            }
        }

        /* --------------------------------------------------------------------------------------------------
        /* get TurboBufAddr */
        if (K%16!=0) /* 8-window case */
        {
            Nw = K / 8;
            cnt_good_bit = 0;
            for (j=0; j<Ncb; j++)
            {
                if (CircularBuf[j]!=(-1))
                {
                    bit = CircularBuf[j];
                    bit_RSC = bit / 10000;
                    bit_addr = bit % 10000;
                    switch(bit_RSC)
                    {
                    case 0: /* system bits */
                        addr_at_Turbo_input_struct = 3*16 + bit_addr*2;
                        break;
                    case 1: /* parity bits 0 */
                        addr_at_Turbo_input_struct = 3*16 + bit_addr*2 + 1;
                        break;
                    case 2: /* parity bits 1 */
                        addr_at_Turbo_input_struct = (3 + 3*Nw)*16 + bit_addr*2 + 1;
                        break;
                    case 3: /* system tail bits */
                        addr_at_Turbo_input_struct = 0*3*16 + bit_addr;
                        break;
                    case 4: /* parity tail bits 0 */
                        addr_at_Turbo_input_struct = 0*3*16 + bit_addr + 4;
                        break;
                    case 5: /* parity tail bits 1 */
                        addr_at_Turbo_input_struct = 0*3*16 + bit_addr + 8;
                        break;
                    default:
                        printf("bit_RSC invalid\n");
                        break;
                } /* end of bit_RSC switch */
                *(p_table_TurboBufAddr + cnt++) =  (uint16_t) addr_at_Turbo_input_struct;
                cnt_good_bit++;
                } /* end of "if (CircularBuf[j]!=(-1))" */
            }
            if (cnt_good_bit!=Kblock)
            {
                printf("cnt_good_bit not match\n");
            }
        }
        else /* 16-window case */
        {
            Nw = K / 16;
            cnt_good_bit = 0;
            for (j=0; j<Ncb; j++)
            {
                if (CircularBuf[j]!=(-1))
                {
                    bit = CircularBuf[j];
                    bit_RSC = bit / 10000;
                    bit_addr = bit % 10000;
                    row = bit_addr % Nw;
                    col = bit_addr / Nw;
                    switch(bit_RSC)
                    {
                    case 0: /* system bits */
                        addr_at_Turbo_input_struct = (row+1)*3*16 + col + 16;
                        break;
                    case 1: /* parity bits 0 */
                        addr_at_Turbo_input_struct = (row+1)*3*16 + col + 32;
                        break;
                    case 2: /* parity bits 1 */
                        addr_at_Turbo_input_struct = (row+1+Nw)*3*16 + col + 32;
                        break;
                    case 3: /* system tail bits */
                        addr_at_Turbo_input_struct = 0*3*16 + bit_addr;
                        break;
                    case 4: /* parity tail bits 0 */
                        addr_at_Turbo_input_struct = 0*3*16 + bit_addr + 4;
                        break;
                    case 5: /* parity tail bits 0 */
                        addr_at_Turbo_input_struct = 0*3*16 + bit_addr + 8;
                        break;
                    default:
                        printf("bit_RSC invalid\n");
                        break;
                    } /* end of bit_RSC switch */
                    *(p_table_TurboBufAddr + cnt++) =  (uint16_t) addr_at_Turbo_input_struct;
                    cnt_good_bit++;
                } /* end of "if (CircularBuf[j]!=(-1))" */
            }
            if (cnt_good_bit!=Kblock)
            {
                printf("cnt_good_bit not match\n");
            }
        }

    } /* end of "for (Kidx=1; Kidx<=188; Kidx++)" */

    return;
}
