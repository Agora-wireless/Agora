#include <iostream>
#include "csv_logger.h"

FILE* fp_csvlog[kCsvLogN] = {nullptr};

static int dev_id = -1;

static constexpr const char* kCsvLogName[kCsvLogN] = {
  "uedata-rfsnr",
  "uedata-evmsnr",
  "uedata-be",
  "uedata-se"
};

static constexpr const char* kCsvLogHeader[kCsvLogN] = {
  "Frame,Symbol,Ant,Pilot-Offset,RF-SNR",
  "Frame,Symbol,User,EVM,EVM-SNR",
  "",
  "Frame,Symbol,UE,Symbol-Errors"
};

void CsvLogSetDev(int id) {
  dev_id = id;
}

void CsvLogInit(size_t log_id) {
  char filename[64];
  sprintf(filename, "%s-%d.csv", kCsvLogName[log_id], dev_id);
  fp_csvlog[log_id] = fopen(filename, "w");
  fprintf(fp_csvlog[log_id], "%s\n", kCsvLogHeader[log_id]);
}

void CsvLogEnd() {
  for (size_t i = 0; i < kCsvLogN; i++) {
    fclose(fp_csvlog[i]);
  }
}