#ifndef PROFILER_H
#define PROFILER_H

#include "Symbols.hpp"

class Profiler
{
public:
    enum State { All, Working, Stating };

    Profiler(size_t tid, State state, size_t slot);
    ~Profiler(); 

    static size_t GetTsc(size_t tid, State state);

private:
    const size_t tid_;
    const State state_;
    const size_t slot_;
    size_t start_tsc_;

    static const size_t slot_trigger_ = 200;
    static size_t total_tsc_[kMaxThreads];
    static size_t work_tsc_[kMaxThreads];
    static size_t state_tsc_[kMaxThreads];
};

#endif