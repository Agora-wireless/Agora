/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef OFFSET_H   
#define OFFSET_H


/* v1: 16 bits, v2: 16bits */
inline int generateOffset2d(int v1, int v2) 
{
    return ((v1 << 16) | (v2 & 0xffff));
};

inline void interpreteOffset2d(int offset, int *v1, int *v2)
{
    *v1 = offset >> 16;
    *v2 = offset & 0xffff;
};


/* v1: (32 - bits2) bits, v2: bits2 bits */
inline int generateOffset2d_setbits(int v1, int v2, int bits2) 
{
    return ((v1 << bits2) | (v2 & ((1 << bits2) - 1)));
};

inline void interpreteOffset2d_setbits(int offset, int *v1, int *v2, int bits2)
{
    *v1 = offset >> bits2;
    *v2 = offset & ((1 << bits2) - 1);
};


/* v1: 8 bits, v2: 8 bits, v3: 16bits */
inline int generateOffset3d(int v1, int v2, int v3)
{
    return ((v1 << 24) | ((v2 & 0xff) << 16) | (v3 & 0xffff));
};

inline void interpreteOffset3d(int offset, int *v1, int *v2, int *v3)
{
    *v1 = offset >> 24;
    *v2 = (offset >> 16) & 0xff;
    *v3 = offset & 0xffff;
};


#endif