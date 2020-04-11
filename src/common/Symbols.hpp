#ifndef SYMBOLS
#define SYMBOLS

#include <stdint.h>
#include <string>

#define EXPORT __attribute__((visibility("default")))

#define ENABLE_CPU_ATTACH

#ifdef USE_ARGOS
#ifndef GENERATE_DATA
#define GENERATE_DATA
#endif
#define GENERATE_PILOT
#endif

#define ARMA_ALLOW_FAKE_GCC

#define SEPARATE_TX_RX_UE 0

#define MOD_ORDER 4

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

#define CODED_LEN 32
#define ORIG_CODE_LEN 16
#define N_ITE 10
#define NUM_CODE_BLOCK 36
#define NUM_BITS 4
#define MAX_CODED_SC 1152

// Number of frames received that we allocate space for in worker threads
#define TASK_BUFFER_FRAME_NUM 40

// Number of frames received that we allocate space for in TX/RX threads
#define SOCKET_BUFFER_FRAME_NUM 40

#define DL_PILOT_SYMS 2
#define TX_FRAME_DELTA 8
#define SETTLE_TIME_MS 1

enum class EventType : int {
    kPacketRX,
    kFFT,
    kZF,
    kDemul,
    kPred,
    kModul,
    kIFFT,
    kPrecode,
    kPacketTX,
    kDecode,
    kEncode,
    kRC,
    kRXSymbol,
    kInvalid
};

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
#define PRINT_DECODE 10
#define PRINT_ENCODE 11
#define PRINT_RC 12
#define PRINT_FFT_CAL 13

#define BIGSTATION 0
#define USE_IPV4 1
#define CONNECT_UDP 1
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
#define DEBUG_PRINT_PILOT 0
#define DEBUG_DL_PILOT 1
#define DEBUG_PLOT 0
#define MEASURE_TIME 1

#define DEBUG_PRINT_IN_TASK 0
#define DEBUG_SENDER 0
#define DEBUG_RECV 0
#define DEBUG_BS_SENDER 0
#define DEBUG_RADIO_TX 0
#define DEBUG_RADIO_RX 0
#define DEBUG_DOWNLINK 0
#define DEBUG_UPLINK 0
#define WRITE_RECV 0

#define CORR_THRESHOLD 0x4
#define CORR_RST 0x0
#define CORR_SCNT 0x8
#define CORR_CONF 60
#define RF_RST_REG 48
#define TDD_CONF_REG 120
#define SCH_ADDR_REG 136
#define SCH_MODE_REG 140
#define TX_GAIN_CTRL 88

enum class ThreadType {
    kMaster,
    kWorker,
    kWorkerFFT,
    kWorkerZF,
    kWorkerDemul,
    kWorkerRX,
    kWorkerTX,
    kWorkerTXRX,
    kMasterRX,
    kMasterTX,
};

static inline std::string thread_type_str(ThreadType thread_type)
{
    switch (thread_type) {
    case ThreadType::kMaster:
        return "Master";
    case ThreadType::kWorker:
        return "Worker";
    case ThreadType::kWorkerFFT:
        return "Worker (FFT)";
    case ThreadType::kWorkerZF:
        return "Worker (ZF)";
    case ThreadType::kWorkerDemul:
        return "Worker (Demul)";
    case ThreadType::kWorkerRX:
        return "RX";
    case ThreadType::kWorkerTX:
        return "TX";
    case ThreadType::kWorkerTXRX:
        return "TXRX";
    case ThreadType::kMasterRX:
        return "Master (RX)";
    case ThreadType::kMasterTX:
        return "Master (TX)";
    }
    return "Invalid thread type";
}

enum class SymbolType { kUL, kDL, kPilot, kCalDL, kCalUL, kUnknown };

struct LDPCconfig {
    uint16_t Bg;
    bool earlyTermination;
    int16_t decoderIter;
    uint16_t Zc;
    int nRows;
    uint32_t cbEncLen;
    uint32_t cbLen;
    uint32_t cbCodewLen;
    int nblocksInSymbol;
};

typedef struct LDPCconfig LDPCconfig;

// Number of cellular frames tracked by Millipede stats
static constexpr size_t kNumStatsFrames = 10000;

// If true, enable timing measurements in workers
static constexpr size_t kTimeWorkers = true;

// Maximum breakdown of a statistic (e.g., timing)
static constexpr size_t kMaxStatsBreakdown = 4;

// Maximum number of hardware threads on one machine
static constexpr size_t kMaxThreads = 128;

static const int MAX_FRAME_ID = 1e4;
static const int float_num_in_simd256 = 8;
static const int double_num_in_simd256 = 4;
#endif
