/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#ifndef OFFSET_H
#define OFFSET_H

#include <stdint.h>
#include <stdlib.h>

/* v1: 8 bits, v2: 8 bits, v3: 16bits */
inline int generateOffset3d(int v1, int v2, int v3)
{
    return ((v1 << 24) | ((v2 & 0xff) << 16) | (v3 & 0xffff));
};

inline void interpreteOffset3d(int offset, int* v1, int* v2, int* v3)
{
    *v1 = offset >> 24;
    *v2 = (offset >> 16) & 0xff;
    *v3 = offset & 0xffff;
};

#endif