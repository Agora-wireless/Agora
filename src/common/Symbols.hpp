#ifndef SYMBOLS
#define SYMBOLS

#include <stdint.h>
#include <string>

#define EXPORT __attribute__((visibility("default")))

#define ARMA_ALLOW_FAKE_GCC

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Number of frames received that we allocate space for in worker threads. This
// is the frame window that we track in Agora.
#define TASK_BUFFER_FRAME_NUM 40
static constexpr size_t kFrameWnd = TASK_BUFFER_FRAME_NUM;

// Number of frames received that we allocate space for in TX/RX threads
#define SOCKET_BUFFER_FRAME_NUM 40

#define TX_FRAME_DELTA 8
#define SETTLE_TIME_MS 1

/// Return true at compile time iff a constant is a power of two
template <typename T> static constexpr inline bool is_power_of_two(T x)
{
    return x && ((x & T(x - 1)) == 0);
}

// TODO: Merge EventType and DoerType into WorkType
enum class EventType : int {
    kPacketRX,
    kFFT,
    kZF,
    kDemul,
    kIFFT,
    kPrecode,
    kPacketTX,
    kPacketPilotTX,
    kDecode,
    kDecodeLast,
    kEncode,
    kRC,
    kModul,
    kPacketFromMac,
    kPacketToMac,
    kSNRReport, // Signal new SNR measurement from PHY to MAC
    kRANUpdate, // Signal new RAN config to Agora
    kRBIndicator // Signal RB schedule to UEs
};
static constexpr size_t kNumEventTypes
    = static_cast<size_t>(EventType::kPacketToMac) + 1;

// Types of Agora Doers
enum class DoerType : size_t {
    kFFT,
    kCSI,
    kZF,
    kDemul,
    kDecode,
    kEncode,
    kIFFT,
    kPrecode,
    kRC
};
static constexpr size_t kNumDoerTypes = static_cast<size_t>(DoerType::kRC) + 1;

enum class PrintType : int {
    kPacketRXPilots,
    kPacketRX,
    kFFTPilots,
    kFFTData,
    kFFTCal,
    kZF,
    kDemul,
    kIFFT,
    kPrecode,
    kPacketTXFirst,
    kPacketTX,
    kDecode,
    kEncode,
    kRC,
    kPacketFromMac,
    kPacketToMac
};

// Enable thread pinning and exit if thread pinning fails. Thread pinning is
// crucial for good performance. For testing or developing Agora on machines
// with insufficient cores, disable this flag.
static constexpr size_t kEnableThreadPinning = true;

#define BIGSTATION 0
#ifdef USE_DPDK
static constexpr bool kUseDPDK = true;
#else
static constexpr bool kUseDPDK = false;
#endif

#ifdef ENABLE_MAC
static constexpr bool kEnableMac = true;
#else
static constexpr bool kEnableMac = false;
#endif

#ifdef USE_ARGOS
static constexpr bool kUseArgos = true;
#else
static constexpr bool kUseArgos = false;
#endif

static constexpr bool kExportConstellation = false;
static constexpr bool kPrintPhyStats = false;
static constexpr bool kCollectPhyStats = false;

static constexpr bool kDebugPrintPerFrameDone = true;
static constexpr bool kDebugPrintPerFrameStart = false;
static constexpr bool kDebugPrintPerSymbolDone = false;
static constexpr bool kDebugPrintPerTaskDone = false;
static constexpr bool kDebugPrintStatsPerThread = false;
static constexpr bool kDebugPrintInTask = false;

/// Print the I/Q samples in the pilots
static constexpr bool kDebugPrintPilot = false;

static constexpr bool kDebugBSSender = false;
static constexpr bool kDebugBSReceiver = true;

#define DEBUG_DL_PILOT 0
#define DEBUG_RECV 0
#define DEBUG_RADIO_TX 0
#define DEBUG_RADIO_RX 0

enum class ThreadType {
    kMaster,
    kWorker,
    kWorkerFFT,
    kWorkerZF,
    kWorkerDemul,
    kWorkerRX,
    kWorkerTX,
    kWorkerTXRX,
    kWorkerMacTXRX,
    kMasterRX,
    kMasterTX,
    kWorkerDemodTXRX,
    kWorkerSubcarrier,
    kWorkerDecode
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
    case ThreadType::kWorkerMacTXRX:
        return "MAC TXRX";
    case ThreadType::kMasterRX:
        return "Master (RX)";
    case ThreadType::kMasterTX:
        return "Master (TX)";
    case ThreadType::kWorkerDemodTXRX:
        return "TXRX (Demod)";
    case ThreadType::kWorkerSubcarrier:
        return "Worker (Subcarrier)";
    case ThreadType::kWorkerDecode:
        return "Worker (Decode)";
    }
    return "Invalid thread type";
}

enum class SymbolType { kBeacon, kUL, kDL, kPilot, kCalDL, kCalUL, kUnknown };

// Maximum number of symbols per frame allowed by Agora
static constexpr size_t kMaxSymbols = 70;

// Maximum number of OFDM data subcarriers in the 5G spec
static constexpr size_t kMaxDataSCs = 3300;

// Maximum number of antennas supported by Agora
static constexpr size_t kMaxAntennas = 128;

// Maximum number of UEs supported by Agora
static constexpr size_t kMaxUEs = 64;

// Maximum modulation (QAM256) supported by Agora. The implementation might
// support only lower modulation orders (e.g., up to QAM64), but using 8 here
// helps reduce false cache line sharing.
static constexpr size_t kMaxModType = 8;

// Number of cellular frames tracked by Agora stats
static constexpr size_t kNumStatsFrames = 10000;

// If true, enable timing measurements in workers
static constexpr bool kIsWorkerTimingEnabled = true;

// Maximum breakdown of a statistic (e.g., timing)
static constexpr size_t kMaxStatsBreakdown = 4;

// Maximum number of hardware threads on one machine
static constexpr size_t kMaxThreads = 128;

// Number of subcarriers in one cache line, when represented as complex floats
static constexpr size_t kSCsPerCacheline = 64 / (2 * sizeof(float));

// Number of subcarriers in a partial transpose block
static constexpr size_t kTransposeBlockSize = 8;
static_assert(is_power_of_two(kTransposeBlockSize), ""); // For cheap modulo
static_assert(kTransposeBlockSize % kSCsPerCacheline == 0, "");

#ifdef USE_AVX2_ENCODER
static constexpr bool kUseAVX2Encoder = true;
#else
static constexpr bool kUseAVX2Encoder = false;
#endif

// Enable debugging for sender and receiver applications
static constexpr bool kDebugSenderReceiver = false;
#endif
