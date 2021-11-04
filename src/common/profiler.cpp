#include "profiler.h"
#include "gettime.h"

size_t Profiler::total_tsc_[kMaxThreads] = {};
size_t Profiler::work_tsc_[kMaxThreads] = {};
size_t Profiler::state_tsc_[kMaxThreads] = {};

Profiler::Profiler(size_t tid, State state, size_t slot)
    : tid_(tid), state_(state), slot_(slot)
{
    start_tsc_ = rdtsc();
}

Profiler::~Profiler()
{
    if (slot_ >= slot_trigger_) {
        size_t duration_tsc = rdtsc() - start_tsc_;
        switch (state_)
        {
        case State::All:
            total_tsc_[tid_] += duration_tsc;
            break;
        case State::Working:
            work_tsc_[tid_] += duration_tsc;
            break;
        case State::Stating:
            state_tsc_[tid_] += duration_tsc;
            break;
        default:
            perror("Wrong type of state in Profiler!");
            break;
        }
    }
}

size_t Profiler::GetTsc(size_t tid, State state)
{
    switch (state)
    {
    case State::All:
        return total_tsc_[tid];
    case State::Working:
        return work_tsc_[tid];
    case State::Stating:
        return state_tsc_[tid];
    default:
        perror("Wrong type of state in Profiler!");
        break;
    }
    return 0;
}