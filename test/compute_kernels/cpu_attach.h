/**
 * @file cpu_attach.h
 * @brief Declaration file for cpu attachment helper functions
 */
#ifndef CPU_ATTACH_H_
#define CPU_ATTACH_H_

int stick_this_thread_to_core(int core_id);

#endif  // CPU_ATTACH_H_