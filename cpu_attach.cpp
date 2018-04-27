#include "cpu_attach.hpp"

int stick_this_thread_to_core(int core_id) {
    int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
    // printf("core_id:%d,total_cores: %d\n", core_id,num_cores);
    if (core_id < 0 || core_id >= num_cores)
        return -1;

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(core_id, &cpuset);

    pthread_t current_thread = pthread_self();    

    return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
}


int pthread_setaffinity_np(pthread_t thread, size_t cpu_size,
                           cpu_set_t *cpu_set)
{
  thread_port_t mach_thread;
  int core = 0;

  for (core = 0; core < 8 * cpu_size; core++) {
    if (CPU_ISSET(core, cpu_set)) break;
  }
  // printf("binding to core %d\n", core);
  thread_affinity_policy_data_t policy = { core };
  mach_thread = pthread_mach_thread_np(thread);
  if (thread_policy_set(mach_thread, THREAD_AFFINITY_POLICY,
                    (thread_policy_t)&policy, 1) != KERN_SUCCESS) {
  	printf("Core: %d, thread policy set failed\n", core);
  	return -1;
  }
  else{
  	return 0;
  }
  // return 0;
}