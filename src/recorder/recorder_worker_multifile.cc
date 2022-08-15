/**
 * @file recorder_worker_multifile.cc
 * @brief Recorder worker to write to multiple bin files per rx symbol
 */

#include "recorder_worker_multifile.h"

#include <string>

#include "logger.h"
#include "utils.h"

namespace Agora_recorder {

static constexpr bool kDebugPrint = false;

RecorderWorkerMultiFile::RecorderWorkerMultiFile(const Config* in_cfg,
                                                 size_t antenna_offset,
                                                 size_t num_antennas,
                                                 size_t record_interval)
    : RecorderWorker(in_cfg, antenna_offset, num_antennas, record_interval),
      cfg_(in_cfg),
      antenna_offset_(antenna_offset),
      num_antennas_(num_antennas),
      interval_(record_interval) {}

RecorderWorkerMultiFile::~RecorderWorkerMultiFile() = default;

void RecorderWorkerMultiFile::Init() {}
void RecorderWorkerMultiFile::Finalize() {}

int RecorderWorkerMultiFile::Record(const Packet* pkt) {
  const size_t end_antenna = (antenna_offset_ + num_antennas_) - 1;

  if ((pkt->ant_id_ < antenna_offset_) || (pkt->ant_id_ > end_antenna)) {
    AGORA_LOG_ERROR(
        "Antenna id is not within range of this recorder %d, %zu:%zu",
        pkt->ant_id_, antenna_offset_, end_antenna);
  }
  assert((pkt->ant_id_ >= antenna_offset_) && (pkt->ant_id_ <= end_antenna));
  int ret = 0;

  const size_t frame_id = pkt->frame_id_;
  const size_t ant_id = pkt->ant_id_;
  const size_t symbol_id = pkt->symbol_id_;
  const size_t dl_symbol_id = cfg_->Frame().GetDLSymbolIdx(pkt->symbol_id_);
  const size_t radio_id = ant_id / cfg_->NumUeChannels();

  if ((frame_id % interval_) == 0) {
    //Remove the beacon?
    if (dl_symbol_id != SIZE_MAX) {
      if (kDebugPrint) {
        std::printf(
            "RecorderWorkerMultiFile::record [frame %d, symbol %d, cell %d, "
            "ant %d] dl_id: %zu - samples: %d %d %d %d %d %d %d %d ....\n",
            pkt->frame_id_, pkt->symbol_id_, pkt->cell_id_, pkt->ant_id_,
            dl_symbol_id, pkt->data_[0u], pkt->data_[1u], pkt->data_[2u],
            pkt->data_[3u], pkt->data_[4u], pkt->data_[5u], pkt->data_[6u],
            pkt->data_[7u]);
      }

      bool is_data = dl_symbol_id >= cfg_->Frame().ClientDlPilotSymbols();
      const std::string pkt_id = "F" + std::to_string(frame_id) + "_S" +
                                 std::to_string(symbol_id) + "_A" +
                                 std::to_string(ant_id);

      const std::string short_serial = cfg_->UeRadioName().at(radio_id);

      if (is_data) {
        const std::string fname_rxdata =
            kOutputFilePath + "rxdata_" + pkt_id + "_" + short_serial + ".bin";
        FILE* fp_rxdata = std::fopen(fname_rxdata.c_str(), "wb");
        if (fp_rxdata == nullptr) {
          throw std::runtime_error(
              "RecorderWorkerMultiFile failed to open rxdata file for "
              "writing");
        }
        std::fwrite(pkt->data_, 2 * sizeof(int16_t), cfg_->SampsPerSymbol(),
                    fp_rxdata);
        std::fclose(fp_rxdata);
        ///Tx data
        const std::string fname_txdata =
            kOutputFilePath + "txdata_" + pkt_id + ".bin";
        FILE* fp_txdata = std::fopen(fname_txdata.c_str(), "wb");
        if (fp_txdata == nullptr) {
          throw std::runtime_error(
              "RecorderWorkerMultiFile failed to open txdata file for "
              "writing");
        }
        std::fwrite(const_cast<Config*>(cfg_)->DlIqF()[dl_symbol_id] +
                        ant_id * cfg_->OfdmCaNum(),
                    2 * sizeof(float), cfg_->OfdmCaNum(), fp_txdata);
        std::fclose(fp_txdata);
      } else {
        const std::string fname_rxpilot =
            kOutputFilePath + "rxpilot_" + pkt_id + "_" + short_serial + ".bin";
        FILE* fp_rxpilot = std::fopen(fname_rxpilot.c_str(), "wb");
        if (fp_rxpilot == nullptr) {
          throw std::runtime_error(
              "RecorderWorkerMultiFile failed to open rxpilot file for "
              "writing");
        }
        std::fwrite(pkt->data_, 2 * sizeof(int16_t), cfg_->SampsPerSymbol(),
                    fp_rxpilot);
        std::fclose(fp_rxpilot);
        ///Tx pilot
        const std::string fname_txpilot =
            kOutputFilePath + "txpilot_" + pkt_id + ".bin";
        FILE* fp_txpilot = std::fopen(fname_txpilot.c_str(), "wb");
        if (fp_txpilot == nullptr) {
          throw std::runtime_error(
              "RecorderWorkerMultiFile failed to open txpilot file for "
              "writing");
        }
        std::fwrite(const_cast<Config*>(cfg_)->UeSpecificPilot()[ant_id],
                    2 * sizeof(float), cfg_->OfdmDataNum(), fp_txpilot);
        std::fclose(fp_txpilot);
      }
    }
  }
  return ret;
}
};  // End namespace Agora_recorder
