#ifndef SYMBOLS
#define SYMBOLS
#define EXPORT __attribute__((visibility("default")))

#define ENABLE_CPU_ATTACH
//#define GENERATE_PILOT
#define GENERATE_DATA
#define SEPARATE_TX_RX 0

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// #define BS_ANT_NUM 16
// #define OFDM_CA_NUM 2048
// #define FFT_LEN 2048
// #define OFDM_DATA_NUM 1200
// #define OFDM_DATA_START 424

// #ifdef USE_ARGOS
//   #define TX_PREFIX_LEN 128
//   #define CP_LEN 128
//   #define OFDM_PREFIX_LEN (152 + CP_LEN)
//   #define UE_NUM 2
// #else
//   #define CP_LEN 0
//   #define OFDM_PREFIX_LEN (0 + CP_LEN)
//   #define UE_NUM 8
// #endif

#define CODED_LEN 32
#define ORIG_CODE_LEN 16
#define N_ITE 10
#define NUM_CODE_BLOCK 36
#define NUM_BITS 4
#define MAX_CODED_SC 1152
#define TASK_BUFFER_FRAME_NUM 60
#define SOCKET_BUFFER_FRAME_NUM 100
#define DL_PILOT_SYMS 2
#define TX_FRAME_DELTA 8


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

#define PRINT_RX_PILOTS 0
#define PRINT_RX 1
#define PRINT_FFT_PILOTS 2
#define PRINT_FFT_DATA 3
#define PRINT_ZF 4
#define PRINT_DEMUL 5
#define PRINT_PRECODE 6
#define PRINT_IFFT 7
#define PRINT_TX_FIRST 8
#define PRINT_TX 9


#define BIGSTATION 0
#define ENABLE_DOWNLINK 0
#define USE_IPV4 1
//#define USE_DPDK 0
#define CONNECT_UDP 1
#define USE_RDTSC 1
#define EXPORT_CONSTELLATION 1
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

#define DEBUG_PRINT_STATS_PER_THREAD 0
#define DEBUG_UPDATE_STATS 1
#define DEBUG_UPDATE_STATS_DETAILED 1
#define DEBUG_PRINT_PILOT 0
#define DEBUG_PLOT 0
#define MEASURE_TIME 1

#define DEBUG_PRINT_IN_TASK 0
#define DEBUG_SENDER 0
#define DEBUG_RECV 0
#define DEBUG_BS_SENDER 0
#define WRITE_DEMUL 0
#define DEBUG_DOWNLINK 1

#define CORR_THRESHOLD    0x4
#define CORR_RST          0x0
#define CORR_SCNT         0x8
#define CORR_CONF         60
#define RF_RST_REG        48
#define TDD_CONF_REG      120
#define SCH_ADDR_REG      136
#define SCH_MODE_REG      140
#define TX_GAIN_CTRL      88


// #ifdef USE_ARGOS
// static const int subframe_num_perframe = 5;
// static const int pilot_subframe_num_perframe = UE_NUM;
// static const int data_subframe_num_perframe = 1; 
// static const int OFDM_FRAME_LEN = OFDM_CA_NUM + 2*TX_PREFIX_LEN;
// 	#if ENABLE_DOWNLINK
// 	static const int ul_data_subframe_num_perframe = 0;
// 	static const int dl_data_subframe_num_perframe = 3;
// 	static const int dl_data_subframe_start = data_subframe_num_perframe - 3;
// 	#else 
// 	static const int ul_data_subframe_num_perframe = data_subframe_num_perframe;
// 	static const int dl_data_subframe_num_perframe = 0;
// 	static const int dl_data_subframe_start = data_subframe_num_perframe;
// 	#endif

// #else
// static const int subframe_num_perframe = 70;
// static const int pilot_subframe_num_perframe = UE_NUM;
// static const int data_subframe_num_perframe = subframe_num_perframe - pilot_subframe_num_perframe;
// static const int OFDM_FRAME_LEN = OFDM_CA_NUM + OFDM_PREFIX_LEN;
// 	#if ENABLE_DOWNLINK
// 	static const int ul_data_subframe_num_perframe = 0;
// 	static const int dl_data_subframe_num_perframe = 10;
// 	static const int dl_data_subframe_start = data_subframe_num_perframe - 10;
// 	#else 
// 	static const int ul_data_subframe_num_perframe = data_subframe_num_perframe;
// 	static const int dl_data_subframe_num_perframe = 0;
// 	static const int dl_data_subframe_start = data_subframe_num_perframe;
// 	#endif
// #endif

// static const int dl_data_subframe_end = dl_data_subframe_start + dl_data_subframe_num_perframe;
// /* header 4 int for: frame_id, subframe_id, cell_id, ant_id, use short for I/Q samples */
// static const int package_header_offset = 64;
// static const int package_length = package_header_offset + sizeof(short) * OFDM_FRAME_LEN * 2;
static const int MAX_FRAME_ID = 1e4;

#endif
