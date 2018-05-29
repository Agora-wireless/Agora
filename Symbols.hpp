#ifndef SYMBOLS
#define SYMBOLS

#define ENABLE_CPU_ATTACH

#define BS_ANT_NUM 96
#define OFDM_CA_NUM 1024
#define OFDM_PREFIX_LEN 0
#define UE_NUM 4

#define EVENT_PACKAGE_RECEIVED 0
#define EVENT_CROPPED 1
#define EVENT_ZF 2
#define EVENT_DEMUL 3
#define EVENT_PRED 4
#define EVENT_MODUL 5
#define EVENT_IFFT 6
#define EVENT_PRECODE 7
#define EVENT_PACKAGE_SENT 8


#define TASK_CROP 0
#define TASK_ZF 1
#define TASK_DEMUL 2
#define TASK_PRED 3
#define TASK_PRECODE 4
#define TASK_IFFT 5
#define TASK_MODUL 6
#define TASK_SEND 7

#define ENABLE_DOWNLINK 1

#define DO_PREDICTION 0
#define INIT_FRAME_NUM 10

#define DEBUG_PRINT 0
#define DEBUG_PRINT_SUMMARY 0
#define DEBUG_PRINT_ENTER_QUEUE 0
#define DEBUG_PRINT_ENTER_QUEUE_DEMUL 0
#define DEBUG_PRINT_ENTER_QUEUE_FFT 0
#define DEBUG_PRINT_IN_TASK 0
#define DEBUG_PRINT_TASK_DONE 0
#define DEBUG_SENDER 0
#define DEBUG_RECV 0
#define DEBUG_BS_SENDER 0
#define WRITE_DEMUL 0


static const int subframe_num_perframe = 40;
static const int pilot_subframe_num_perframe = UE_NUM;
static const int data_subframe_num_perframe = subframe_num_perframe - pilot_subframe_num_perframe;

static const int MAX_FRAME_ID = 1e4;

#endif
