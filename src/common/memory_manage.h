/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#ifndef MEMORY_MANAGE
#define MEMORY_MANAGE
#include <stdio.h>
#include <stdlib.h>
#include <cstring>

template <typename T, typename U>
static void alloc_buffer_2d(T ***buffer, int dim1, U dim2, int aligned_bytes, int init_zero) 
{
    aligned_bytes = (aligned_bytes) / 32 * 32;
    *buffer = (T **)malloc(dim1 * sizeof(T *));
    for (int i = 0; i < dim1; i++) {
        (*buffer)[i] = (T *)aligned_alloc(aligned_bytes, dim2 * sizeof(T));
        if (init_zero)
            memset((*buffer)[i], 0, dim2 * sizeof(T));
    }
};

template <typename T, typename U>
static void alloc_buffer_1d(T **buffer, U dim, int aligned_bytes, int init_zero) 
{
    aligned_bytes = (aligned_bytes) / 32 * 32;
    *buffer = (T *)aligned_alloc(aligned_bytes, dim * sizeof(T));
    if (init_zero)
        memset(*buffer, 0, dim * sizeof(T));
};

template <typename T>
static void free_buffer_2d(T ***buffer, int dim1) 
{
    for (int i = 0; i < dim1; i++) {
        free((*buffer)[i]);
    }
    free((*buffer));
};

template <typename T>
static void free_buffer_1d(T **buffer) {free(*buffer);};

#endif