#ifndef SYMBOLS
#define SYMBOLS
#define EXPORT __attribute__((visibility("default")))

#define ENABLE_CPU_ATTACH
//#define GENERATE_PILOT
#define GENERATE_DATA
#define SEPARATE_TX_RX 0

#define MOD_ORDER 4

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

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


#define EVENT_PACKET_RECEIVED 0
#define EVENT_FFT 1
#define EVENT_ZF 2
#define EVENT_DEMUL 3

#define EVENT_PRED 4
#define EVENT_MODUL 5
#define EVENT_IFFT 6
#define EVENT_PRECODE 7
#define EVENT_PACKET_SENT 8

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
#define ENABLE_DECODE 1
#define USE_IPV4 1
//#define USE_DPDK 0
#define CONNECT_UDP 1
#define USE_RDTSC 1
#define EXPORT_CONSTELLATION 0

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

#define DEBUG_PRINT_PER_FRAME_START 1

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

typedef enum {
	Master,
	Worker,
	Worker_FFT,
	Worker_ZF,
	Worker_Demul,
	Worker_RX,
	Worker_TX,
	Worker_TXRX,
	Master_RX,
	Master_TX,
} thread_type;

static const char *THREAD_TYPE_STRING[] = {
	"Master",
	"Worker",
	"Worker (FFT)",
	"Worker (ZF)",
	"Worker (Demul)",
	"RX",
	"TX",
	"TXRX",
	"Master (RX)",
	"Master (TX)"
};




struct LDPCconfig {
    uint16_t Bg;
    bool earlyTermination;
    int16_t decoderIter;
    uint16_t Zc;
    int nRows;
    uint32_t cbEncLen;
    uint32_t cbLen;
    uint32_t cbCodewLen;
};

typedef struct LDPCconfig LDPCconfig;


static const int MAX_FRAME_ID = 1e4;
static const int float_num_in_simd256 = 8;
static const int double_num_in_simd256 = 4;
#endif
