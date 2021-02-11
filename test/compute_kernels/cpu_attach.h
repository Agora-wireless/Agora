#ifndef CPU_ATTACH
#define CPU_ATTACH
#include <pthread.h>
#include <unistd.h>

int stick_this_thread_to_core(int core_id);

#endif