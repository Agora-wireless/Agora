#ifndef CPU_ATTACH
#define CPU_ATTACH
#include <unistd.h>
#include <pthread.h>

int stick_this_thread_to_core(int core_id);


#endif