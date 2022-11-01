#ifndef SYMBOLS_H_
#define SYMBOLS_H_

#include <mkl.h>

#include <array>
#include <cstddef>
#include <map>
#include <string>

#define EXPORT __attribute__((visibility("default")))

#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)

// Number of frames received that we allocate space for in worker threads. This
// is the frame window that we track in Agora.
static constexpr size_t kFrameWnd = 40;

#define TX_FRAME_DELTA (4)
#define SETTLE_TIME_MS (1)

// Just-in-time optimization for MKL cgemm is available only after MKL 2019
// update 3. Disable this on systems with an older MKL version.
#if __INTEL_MKL__ >= 2020 || (__INTEL_MKL__ == 2019 && __INTEL_MKL_UPDATE__ > 3)
#define USE_MKL_JIT (1)
#else
#undef USE_MKL_JIT
#endif

#define ENABLE_RB_IND (0)

/// Return true at compile time iff a constant is a power of two
template <typename T>
static constexpr inline bool IsPowerOfTwo(T x) {
  return x && ((x & T(x - 1)) == 0);
}

enum class Direction : int { kDownlink, kUplink };

/// \todo Merge EventType and DoerType into WorkType
enum class EventType : int {
  kPacketRX,
  kFFT,
  kBeam,
  kDemul,
  kIFFT,
  kPrecode,
  kPacketTX,
  kPacketPilotTX,
  kDecode,
  kEncode,
  kModul,
  kPacketFromMac,
  kPacketToMac,
  kFFTPilot,
  kSNRReport,    // Signal new SNR measurement from PHY to MAC
  kRANUpdate,    // Signal new RAN config to Agora
  kRBIndicator,  // Signal RB schedule to UEs
  kThreadTermination
};

static constexpr size_t kNumEventTypes =
    static_cast<size_t>(EventType::kThreadTermination) + 1;

// Types of Agora Doers
enum class DoerType : size_t {
  kFFT,
  kCSI,
  kBeam,
  kDemul,
  kDecode,
  kEncode,
  kIFFT,
  kPrecode,
  kRC
};

static constexpr std::array<DoerType, (static_cast<size_t>(DoerType::kRC) + 1)>
    kAllDoerTypes = {DoerType::kFFT,   DoerType::kCSI,     DoerType::kBeam,
                     DoerType::kDemul, DoerType::kDecode,  DoerType::kEncode,
                     DoerType::kIFFT,  DoerType::kPrecode, DoerType::kRC};
static constexpr size_t kNumDoerTypes = kAllDoerTypes.size();

static const std::map<DoerType, std::string> kDoerNames = {
    {DoerType::kFFT, std::string("FFT")},
    {DoerType::kCSI, std::string("CSI")},
    {DoerType::kBeam, std::string("Beamweights")},
    {DoerType::kDemul, std::string("Demul")},
    {DoerType::kDecode, std::string("Decode")},
    {DoerType::kEncode, std::string("Encode")},
    {DoerType::kIFFT, std::string("iFFT")},
    {DoerType::kPrecode, std::string("Precode")},
    {DoerType::kRC, std::string("RC")}};

enum class PrintType : int {
  kPacketRXPilots,
  kPacketRX,
  kFFTPilots,
  kFFTData,
  kFFTCal,
  kBeam,
  kDemul,
  kIFFT,
  kPrecode,
  kPacketTXFirst,
  kPacketTX,
  kDecode,
  kEncode,
  kRC,
  kPacketFromMac,
  kPacketToMac,
  kModul
};

enum ScheduleProcessingFlags : uint8_t {
  kNone = 0,
  kUplinkComplete = 0x1,
  kDownlinkComplete = 0x2,
  kProcessingComplete = (kUplinkComplete + kDownlinkComplete)
};

// Moved from Agora class
/// \todo need organization
static constexpr size_t kDefaultMessageQueueSize = 512;
static constexpr size_t kDefaultWorkerQueueSize = 256;
// Max number of worker threads allowed
//static constexpr size_t kMaxWorkerNum = 50;
static constexpr size_t kScheduleQueues = 2;
// Dequeue batch size, used to reduce the overhead of dequeue in main thread
static constexpr size_t kDequeueBulkSizeTXRX = 8;
static constexpr size_t kDequeueBulkSizeWorker = 4;

// Enable thread pinning and exit if thread pinning fails. Thread pinning is
// crucial for good performance. For testing or developing Agora on machines
// with insufficient cores, disable this flag.
static constexpr bool kEnableThreadPinning = true;
static constexpr bool kEnableCoreReuse = false;

#define BIGSTATION (0)
#if defined(USE_DPDK)
static constexpr bool kUseDPDK = true;
#else
static constexpr bool kUseDPDK = false;
#endif

#if defined(ENABLE_MAC)
static constexpr bool kEnableMac = true;
#else
static constexpr bool kEnableMac = false;
#endif

#if defined(USE_ARGOS)
static constexpr bool kUseArgos = true;
#else
static constexpr bool kUseArgos = false;
#endif

#if defined(USE_UHD)
static constexpr bool kUseUHD = true;
#else
static constexpr bool kUseUHD = false;
#endif

#if defined(ENABLE_CSV_LOG)
static constexpr bool kEnableCsvLog = true;
#else
static constexpr bool kEnableCsvLog = false;
#endif

#if defined(ENABLE_MAT_LOG)
static constexpr bool kEnableMatLog = true;
#else
static constexpr bool kEnableMatLog = false;
#endif

// Use 12-bit IQ sample to reduce network throughput
static constexpr bool kUse12BitIQ = false;
static constexpr bool kDebug12BitIQ = false;
static constexpr bool kDebugDownlink = false;
static constexpr bool kDebugUplink = false;

static constexpr bool kUsePartialTrans = true;

// Enable hard demodulation and disable LDPC decoding
// Useful for evaluating constellation quality
static constexpr bool kDownlinkHardDemod = false;
static constexpr bool kUplinkHardDemod = false;

static constexpr bool kExportConstellation = false;
static constexpr bool kPrintPhyStats = true;
static constexpr bool kCollectPhyStats = true;
static constexpr bool kPrintBeamStats = true;

static constexpr bool kStatsPrintFrameSummary = true;
static constexpr bool kDebugPrintPerFrameDone = true;
static constexpr bool kDebugPrintPerFrameStart = true;
static constexpr bool kDebugPrintPerSymbolDone = false;
static constexpr bool kDebugPrintPerTaskDone = false;
static constexpr bool kDebugPrintStatsPerThread = false;
static constexpr bool kDebugPrintInTask = false;
static constexpr bool kDebugMulticell = false;
static constexpr bool kRecordCalibrationMats = false;

/// Print the I/Q samples in the pilots
static constexpr bool kDebugPrintPilot = false;

static constexpr bool kDebugRadioTX = false;
static constexpr bool kDebugRadioRX = false;

static constexpr bool kLogMacPackets = false;

enum class ThreadType {
  kMaster,
  kWorker,
  kWorkerFFT,
  kWorkerBeam,
  kWorkerDemul,
  kWorkerDecode,
  kWorkerRX,
  kWorkerTX,
  kWorkerTXRX,
  kWorkerMacTXRX,
  kMasterRX,
  kMasterTX,
  kRecorderWorker
};

static inline std::string ThreadTypeStr(ThreadType thread_type) {
  switch (thread_type) {
    case ThreadType::kMaster:
      return "Master";
    case ThreadType::kWorker:
      return "Worker";
    case ThreadType::kWorkerFFT:
      return "Worker (FFT)";
    case ThreadType::kWorkerBeam:
      return "Worker (Beamweights)";
    case ThreadType::kWorkerDemul:
      return "Worker (Demul)";
    case ThreadType::kWorkerDecode:
      return "Worker (Decode)";
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
    case ThreadType::kRecorderWorker:
      return "Recorder Worker";
  }
  return "Invalid thread type";
}

enum class SymbolType {
  kBeacon,
  kUL,
  kDL,
  kPilot,
  kCalDL,
  kCalUL,
  kGuard,
  kUnknown
};
static const std::map<char, SymbolType> kSymbolMap = {
    {'B', SymbolType::kBeacon}, {'C', SymbolType::kCalDL},
    {'D', SymbolType::kDL},     {'G', SymbolType::kGuard},
    {'L', SymbolType::kCalUL},  {'P', SymbolType::kPilot},
    {'U', SymbolType::kUL}};

enum class SubcarrierType { kNull, kDMRS, kData };

// Maximum number of symbols per frame allowed by Agora
static constexpr size_t kMaxSymbols = 70;

// Maximum number of OFDM data subcarriers in the 5G spec
static constexpr size_t kMaxDataSCs = 3300;

// Maximum number of antennas supported by Agora
static constexpr size_t kMaxAntennas = 64;

// Maximum number of UEs supported by Agora
static constexpr size_t kMaxUEs = 64;

// Maximum number of transceiver channels per radio
static constexpr size_t kMaxChannels = 2;

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

// Agora Client sends UDP packets for UE #i (downlink packets at the client) to
// destination port kMacUserRemotePort + i
static constexpr size_t kMacUserRemotePort = 9070;

// Agora Client Mac Thread listens for UDP packets from applications (uplink
// packets at the client)
static constexpr size_t kMacUserLocalPort = 9170;

// After receiving decoded codeblocks from the PHY (uplink at the
// server, downlink at the client), we send UDP packets to kRemoteHostname
static constexpr char kMacRemoteHostname[] = "127.0.0.1";

// Agora sends UDP packets for UE #i (uplink packets at the server) to
// destination port kMacBaseRemotePort + i
static constexpr size_t kMacBaseRemotePort = 8080;

// Agora listens for UDP packets (downlink data packets at the server) at
// port kBaseLocalPort
static constexpr size_t kMacBaseLocalPort = 8180;

// Agora sends control information over an out-of-band control channel
// to each UE #i, at port kBaseClientPort + i
/// \todo need to generalize for hostname, port pairs for each client
static constexpr size_t kMacBaseClientPort = 7070;

// Number of subcarriers in a partial transpose block
static constexpr size_t kTransposeBlockSize = 8;
static_assert(IsPowerOfTwo(kTransposeBlockSize));  // For cheap modulo
static_assert(kTransposeBlockSize % kSCsPerCacheline == 0);

static constexpr size_t kCalibScGroupSize = 8;
static_assert(kCalibScGroupSize % kSCsPerCacheline == 0);

#ifdef USE_AVX2_ENCODER
static constexpr bool kUseAVX2Encoder = true;
#else
static constexpr bool kUseAVX2Encoder = false;
#endif

// Enable debugging for sender and receiver applications
static constexpr bool kDebugSenderReceiver = false;

#if defined(ENABLE_HDF5)
static constexpr bool kOutputUlScData = true;
#else
static constexpr bool kOutputUlScData = false;
#endif

static constexpr size_t kOfdmSymbolPerSlot = 1;
static constexpr size_t kOutputFrameNum = 1;

static constexpr bool kDebugTxData = false;
#endif  // SYMBOLS_H_
