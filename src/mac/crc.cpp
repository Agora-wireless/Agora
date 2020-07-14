/*
 * crc.cpp
 * Cyclic Redundancy Check (CRC)
 *
 * This is an implementation of the CRC-24Q cyclic redundancy checksum
 * used by Qualcomm, RTCM104V3, and PGP 6.5.1.
 *
 * 1) It detects all single bit errors per 24-bit code word.
 * 2) It detects all double bit error combinations in a code word.
 * 3) It detects any odd number of errors.
 * 4) It detects any burst error for which the length of the burst is less than
 *    or equal to 24 bits.
 * 5) It detects most large error bursts with length greater than 24 bits;
 *    the odds of a false positive are at most 2^-23.
 *
 * This hash should not be considered cryptographically secure, but it
 * is extremely good at detecting noise errors.
 *
 * Code based on GPSD project implementation:
 * https://gitlab.com/gpsd/gpsd/-/blob/master/crc24q.c
 * 
 * Other implementations found at:
 * http://docs.ros.org/indigo/api/swiftnav/html/group__crc.html
 *
 * Copyright (c) 2008-2018 by the GPSD project
 * SPDX-License-Identifier: BSD-2-clause
 * https://spdx.org/licenses/BSD-2-Clause.html
 *
 *
   //CRC check
   bool is_good = crc_up->checkCRC24(pkt_curr);
   printf("\nCRC GOOD? %d \n", is_good);
   if (!is_good)
       exit(0); //continue;
 */


#include "crc.hpp"


#ifdef REBUILD_TABLE
static void DoCRC::initCRC24(uint32_t table[256])
{
    /*
     * Generate table of all posible remainders given all possible 8-bit
     * dividends.
     */
    unsigned i, j;
    unsigned h;

    table[0] = CRCSEED;
    table[1] = h = G_CRC_24A;

    for (i = 2; i < 256; i *= 2) {
	if ((h <<= 1) & 0x1000000)
	    h ^= G_CRC_24A;
	for (j = 0; j < i; j++)
	    table[i + j] = table[j] ^ h;
    }
}


int main()
{
    // Default: CRC24
    int i;
    uint32_t crcTable[256];
    crcInit(crcTable);

    for (i = 0; i < 256; i++) {
	    printf("0x%04X, ", crcTable[i]);
	    if ((i % 4) == 3) putchar('\n');
    }
    return 0;
}
#endif


void DoCRC::addCRC24(MacPacket* p)
{
    /* Init
     * TODO: Size of CRC should depend on Transport Block length and should 
     * consider both header and data, not just data
     * int tb_len = p->datalen_;    // Transport Block (TB) length in bits
     */ 

    uint32_t crc = calculateCRC24((unsigned char*)p->data, p->datalen);

    p->crc[0] = HI(crc);
    p->crc[1] = MID(crc);
    p->crc[2] = LO(crc);
}


uint32_t DoCRC::calculateCRC24(unsigned char *data, int len)
{
    /*
     *
     */
    int i;
    uint32_t crc = 0;

    for (i = 0; i < len; i++) {
	    crc = (crc << 8) ^ crc24_table[data[i] ^ (unsigned char)(crc >> 16)];
    }

    crc = (crc & 0x00ffffff);

    return crc;
}


bool DoCRC::checkCRC24(Packet* p)
{
    /*
     * Compute CRC for incoming packet and verify it matches the CRC entry.
     * Return TRUE if it matches, FALSE otherwise
     */

    bool rval;
    uint32_t crc = calculateCRC24(p->data_, p->datalen_);

    rval = (((p->crc_[0] == HI(crc)) &&
	         (p->crc_[1] == MID(crc)) && 
             (p->crc_[2] == LO(crc))));

    return rval;
}
