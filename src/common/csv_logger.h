#ifndef CSV_LOGGER_H_
#define CSV_LOGGER_H_

#define ENABLE_CSV_LOGGER true

enum CsvLog {
  kCsvLogRFSNR,
  kCsvLogEVMSNR,
  kCsvLogBE,
  kCsvLogSE,
  kCsvLogN
};

extern FILE* fp_csvlog[kCsvLogN];

extern void CsvLogSetDev(int id);
extern void CsvLogInit(size_t log_id);
extern void CsvLogEnd();

#if ENABLE_CSV_LOGGER
#define CSV_LOG(LOG_ID, ...) \
  if (fp_csvlog[LOG_ID] == nullptr) { CsvLogInit(LOG_ID); } \
  fprintf(fp_csvlog[LOG_ID], __VA_ARGS__)
#else
#define CSV_LOG(LOG_ID, ...)
#endif

#endif //CSV_LOGGER_H_