#ifndef CPU_ATTACH
#define CPU_ATTACH
#include <unistd.h>
#include <pthread.h>
#include <mach/thread_policy.h>
#include <mach/thread_act.h>
#include <stdio.h>  /* for fprintf */
#define SYSCTL_CORE_COUNT   "machdep.cpu.core_count"


kern_return_t	thread_policy_set(
					thread_t			thread,
					thread_policy_flavor_t		flavor,
					thread_policy_t			policy_info,
					mach_msg_type_number_t		count);

kern_return_t	thread_policy_get(
					thread_t			thread,
					thread_policy_flavor_t		flavor,
					thread_policy_t			policy_info,
					mach_msg_type_number_t		*count,
					boolean_t			*get_default);

typedef struct cpu_set {
  uint32_t    count;
} cpu_set_t;

static inline void
CPU_ZERO(cpu_set_t *cs) { cs->count = 0; }

static inline void
CPU_SET(int num, cpu_set_t *cs) { cs->count |= (1 << num); }

static inline int
CPU_ISSET(int num, cpu_set_t *cs) { return (cs->count & (1 << num)); }

int stick_this_thread_to_core(int core_id);

int pthread_setaffinity_np(pthread_t thread, size_t cpu_size,
                           cpu_set_t *cpu_set);


#endif