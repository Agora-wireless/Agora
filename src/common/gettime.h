/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef GETTIME_H   
#define GETTIME_H

#include <time.h>
#include <stdint.h>
#include "Symbols.hpp"


/* assembly code to read the TSC */
static inline uint64_t RDTSC()
{
  unsigned int hi, lo;
  __asm__ volatile("rdtsc" : "=a" (lo), "=d" (hi));
  return ((uint64_t)hi << 32) | lo;
};


static inline double get_time(void)
{
#if USE_RDTSC
    return double(RDTSC())/2.3e3;
#else
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return tv.tv_sec * 1000000 + tv.tv_nsec / 1000.0;
#endif
};

#endif