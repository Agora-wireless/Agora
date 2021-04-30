#ifndef GETTIME_H
#define GETTIME_H

#include "Symbols.hpp"
#include <stdint.h>
#include <time.h>
#include <sys/time.h>

// Get current time in microseconds
static inline double get_time_us(void)
{
    struct timespec tv;
    clock_gettime(CLOCK_MONOTONIC, &tv);
    return tv.tv_sec * 1000000 + tv.tv_nsec / 1000.0;
}

// Get current time in microseconds. This can be deleted after we replace
// all occurences of get_time() with get_time_us()
static inline double get_time(void) { return get_time_us(); }

/// Return the TSC
static inline size_t rdtsc()
{
    uint64_t rax;
    uint64_t rdx;
    asm volatile("rdtsc" : "=a"(rax), "=d"(rdx));
    return static_cast<size_t>((rdx << 32) | rax);
}

/// Return the system timestamp
static inline size_t get_ns()
{
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    return current_time.tv_sec * 1000000000L + current_time.tv_usec;
}

/// Return the TSC or zero, depending on whether timing of workers is
/// enabled
static inline size_t worker_rdtsc()
{
    return kIsWorkerTimingEnabled ? rdtsc() : 0;
}

/// Sleep for some nanoseconds
static inline void nano_sleep(size_t ns, double freq_ghz)
{
    size_t start = rdtsc();
    size_t end = start;
    size_t upp = static_cast<size_t>(freq_ghz * ns);
    while (end - start < upp)
        end = rdtsc();
}

/// Measure the frequency of RDTSC based by comparing against
/// CLOCK_REALTIME. This is a pretty function that should be called only
/// during initialization.
static inline double measure_rdtsc_freq()
{
    struct timespec start, end;
    clock_gettime(CLOCK_REALTIME, &start);
    uint64_t rdtsc_start = rdtsc();

    // Do not change this loop! The hardcoded value below depends on this
    // loop and prevents it from being optimized out.
    uint64_t sum = 5;
    for (uint64_t i = 0; i < 1000000; i++) {
        sum += i + (sum + i) * (i % sum);
    }
    if (sum != 13580802877818827968ull) {
        printf("Error in RDTSC freq measurement");
        exit(-1);
    }

    clock_gettime(CLOCK_REALTIME, &end);
    uint64_t clock_ns
        = static_cast<uint64_t>(end.tv_sec - start.tv_sec) * 1000000000
        + static_cast<uint64_t>(end.tv_nsec - start.tv_nsec);
    uint64_t rdtsc_cycles = rdtsc() - rdtsc_start;

    double freq_ghz = rdtsc_cycles * 1.0 / clock_ns;

    // RDTSC frequencies outside these ranges are rare
    if (freq_ghz < 1.0 && freq_ghz > 4.0) {
        printf("Invalid RDTSC frequency %.2f\n", freq_ghz);
        exit(-1);
    }
    return freq_ghz;
}

/// Convert cycles measured by rdtsc with frequence \p freq_ghz to seconds
static inline double cycles_to_sec(size_t cycles, double freq_ghz)
{
    return (cycles / (freq_ghz * 1000000000));
}

/// Convert cycles measured by rdtsc with frequence \p freq_ghz to
/// milliseconds
static inline double cycles_to_ms(size_t cycles, double freq_ghz)
{
    return (cycles / (freq_ghz * 1000000));
}

/// Convert cycles measured by rdtsc with frequence \p freq_ghz to
/// microseconds
static inline double cycles_to_us(size_t cycles, double freq_ghz)
{
    return (cycles / (freq_ghz * 1000));
}

/// Convert cycles measured by rdtsc with frequence \p freq_ghz to
/// nanoseconds
static inline double cycles_to_ns(size_t cycles, double freq_ghz)
{
    return (cycles / freq_ghz);
}

static inline size_t ms_to_cycles(double ms, double freq_ghz)
{
    return static_cast<size_t>(ms * 1000 * 1000 * freq_ghz);
}

static inline size_t us_to_cycles(double us, double freq_ghz)
{
    return static_cast<size_t>(us * 1000 * freq_ghz);
}

static inline size_t ns_to_cycles(double ns, double freq_ghz)
{
    return static_cast<size_t>(ns * freq_ghz);
}

/// Return seconds elapsed since timestamp \p t0
static inline double sec_since(const struct timespec& t0)
{
    struct timespec t1;
    clock_gettime(CLOCK_REALTIME, &t1);
    return (t1.tv_sec - t0.tv_sec) + (t1.tv_nsec - t0.tv_nsec) / 1000000000.0;
}

/// Return nanoseconds elapsed since timestamp \p t0
static inline double ns_since(const struct timespec& t0)
{
    struct timespec t1;
    clock_gettime(CLOCK_REALTIME, &t1);
    return (t1.tv_sec - t0.tv_sec) * 1000000000.0 + (t1.tv_nsec - t0.tv_nsec);
}

#endif