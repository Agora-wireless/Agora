/**
 * @file recorder_worker_multifile.cc
 * @brief Recorder worker to write to multiple bin files per rx symbol
 */

#include "recorder_worker_multifile.h"

#include "logger.h"
#include "utils.h"

namespace Agora_recorder {

static constexpr bool kDebugPrint = false;
static constexpr size_t kRecordFrameInterval = 10;
static constexpr size_t kShortSerialLen = 3;

RecorderWorkerMultiFile::RecorderWorkerMultiFile(const Config* in_cfg,
                                                 size_t antenna_offset,
                                                 size_t num_antennas)
    : RecorderWorker(in_cfg, antenna_offset, num_antennas),
      cfg_(in_cfg),
      antenna_offset_(antenna_offset),
      num_antennas_(num_antennas) {}

RecorderWorkerMultiFile::~RecorderWorkerMultiFile() { Gc(); }

void RecorderWorkerMultiFile::Gc() {
  AGORA_LOG_INFO("RecorderWorkerMultiFile::Garbage collect\n");
}

void RecorderWorkerMultiFile::Init() {}
void RecorderWorkerMultiFile::Finalize() {}

int RecorderWorkerMultiFile::Record(const Packet* pkt) {
  /* TODO: remove TEMP check */
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

  if ((frame_id % kRecordFrameInterval) == 0) {
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

      const std::string short_serial =
          cfg_->UeRadioId().empty()
              ? "UE"
              : cfg_->UeRadioId().at(radio_id).substr(
                    cfg_->UeRadioId().at(radio_id).length() - kShortSerialLen);

      if (is_data) {
        std::string fname = "rxdataR_" + pkt_id + "_" + short_serial + ".bin";
        FILE* f = std::fopen(fname.c_str(), "wb");
        std::fwrite(pkt->data_, 2 * sizeof(int16_t), cfg_->SampsPerSymbol(), f);
        std::fclose(f);
        fname = "txdataR_" + pkt_id + ".bin";
        f = std::fopen(fname.c_str(), "wb");
        std::fwrite(const_cast<Config*>(cfg_)->DlIqF()[dl_symbol_id] +
                        ant_id * cfg_->OfdmCaNum(),
                    2 * sizeof(float), cfg_->OfdmCaNum(), f);
        std::fclose(f);
      } else {
        std::string fname = "rxpilotR_" + pkt_id + "_" + short_serial + ".bin";
        FILE* f = std::fopen(fname.c_str(), "wb");
        std::fwrite(pkt->data_, 2 * sizeof(int16_t), cfg_->SampsPerSymbol(), f);
        std::fclose(f);
        fname = "txpilotR_" + pkt_id + ".bin";
        f = std::fopen(fname.c_str(), "wb");
        std::fwrite(const_cast<Config*>(cfg_)->UeSpecificPilot()[ant_id],
                    2 * sizeof(float), cfg_->OfdmDataNum(), f);
        std::fclose(f);
      }
    }
  }
  return ret;
}
};  // End namespace Agora_recorder
