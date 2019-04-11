#ifndef SYMBOLS
#define SYMBOLS
#define EXPORT __attribute__((visibility("default")))

#define ENABLE_CPU_ATTACH

#define BS_ANT_NUM 64
#define OFDM_CA_NUM 2048
#define FFT_LEN 2048
#define OFDM_DATA_NUM 1200
#define OFDM_DATA_START 424
#define OFDM_PREFIX_LEN 0
#define UE_NUM 8
#define CODED_LEN 32
#define ORIG_CODE_LEN 16
#define N_ITE 10
#define NUM_CODE_BLOCK 36
#define NUM_BITS 4
#define MAX_CODED_SC 1152


#define EVENT_PACKAGE_RECEIVED 0
#define EVENT_FFT 1
#define EVENT_ZF 2
#define EVENT_DEMUL 3

#define EVENT_PRED 4
#define EVENT_MODUL 5
#define EVENT_IFFT 6
#define EVENT_PRECODE 7
#define EVENT_PACKAGE_SENT 8

#define EVENT_DECODE 9
#define EVENT_ENCODE 10


#define TASK_FFT 0
#define TASK_ZF 1
#define TASK_DEMUL 2
#define TASK_PRED 3
#define TASK_PRECODE 4
#define TASK_IFFT 5
#define TASK_MODUL 6
#define TASK_SEND 7
#define TASK_DECODE 8
#define TASK_ENCODE 9


#define BIGSTATION 0
#define ENABLE_DOWNLINK 0
#define USE_IPV4 1
#define EXPORT_CONSTELLATION 0
#define ENABLE_DECODE 0
#define COMBINE_EQUAL_DECODE 1

#define DO_PREDICTION 0
#define INIT_FRAME_NUM 10


#define DEBUG_PRINT_PER_FRAME_DONE 1
#define DEBUG_PRINT_PER_SUBFRAME_DONE 0
#define DEBUG_PRINT_PER_TASK_DONE 0
#define DEBUG_PRINT_SUMMARY_100_FRAMES 0

#define DEBUG_PRINT_PER_FRAME_ENTER_QUEUE 0
#define DEBUG_PRINT_PER_SUBFRAME_ENTER_QUEUE 0
#define DEBUG_PRINT_PER_TASK_ENTER_QUEUE 0

#define DEBUG_PRINT_PER_FRAME_START 0

#define DEBUG_PRINT_STATS_PER_THREAD 1
#define DEBUG_UPDATE_STATS 1
#define DEBUG_UPDATE_STATS_DETAILED 1
#define DEBUG_PRINT_PILOT 0
#define MEASURE_TIME 1

#define DEBUG_PRINT_IN_TASK 0
#define DEBUG_SENDER 0
#define DEBUG_RECV 0
#define DEBUG_BS_SENDER 0
#define WRITE_DEMUL 0


static const int subframe_num_perframe = 70;
static const int pilot_subframe_num_perframe = UE_NUM;
static const int data_subframe_num_perframe = subframe_num_perframe - pilot_subframe_num_perframe;
static const int dl_data_subframe_start = 20;
static const int dl_data_subframe_num_perframe = data_subframe_num_perframe - dl_data_subframe_start;

static const int MAX_FRAME_ID = 1e4;

#endif
