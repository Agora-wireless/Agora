#ifndef RECORDER_AGORA
#define RECORDER_AGORA

#include "config.h"
#include "buffer.h"
#include "H5Cpp.h"
#include "recorder/recorder_worker.h"
#include "symbols.h"

class RxWorker : public Recorder::RecorderWorker {
  public:
  RxWorker() = delete;
  RxWorker(EventType type, Config *in_cfg, H5::H5File *h5_file):
    Recorder::RecorderWorker(type, in_cfg, h5_file) {
    H5std_string dataset_name = Recorder::RecorderWorker::GetDataSetName();

    dataset_ = std::make_unique<H5::DataSet>(h5_file->openDataSet(dataset_name));
    data_len_ = in_cfg->PacketLength() - Packet::kOffsetOfData;
  }

  herr_t Record(void *in_pkt) override {
    return 0;
  }

  private:
  std::unique_ptr<H5::DataSet> dataset_;
  size_t data_len_;
};

class RxWorkerFactory : public Recorder::RecorderWorkerFactory {
  public:
  RxWorkerFactory():
    references_(0) {}

  std::unique_ptr<Recorder::RecorderWorker> GenWorker(Config *cfg, H5::H5File *h5_file) override {
    auto worker = std::make_unique<RxWorker>(EventType::kPacketRX, cfg, h5_file);
    if(references_.fetch_add(1) == 1) {
      // The file needs initialization
      hsize_t dims[] = {  cfg->FramesToTest(),
                          cfg->Frame().NumTotalSyms(),
                          cfg->NumAntennas(),
                          (cfg->PacketLength() - Packet::kOffsetOfData)
                        };
      auto data_space = std::make_unique<H5::DataSpace>(3, dims);
      
      h5_file->createDataSet(worker->GetDataSetName(), H5::PredType::NATIVE_SHORT, *data_space.get());
    }

    return worker;
  }

  private:
  std::atomic<size_t> references_;
};

#endif /* RECORDER_AGORA */