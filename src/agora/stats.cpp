#include "stats.hpp"
#include <typeinfo>

Stats::Stats(Config* cfg)
    : config_(cfg)
    , task_thread_num_(cfg->worker_thread_num())
    , fft_thread_num_(cfg->fft_thread_num())
    , zf_thread_num_(cfg->zf_thread_num())
    , demul_thread_num_(cfg->demul_thread_num())
    , decode_thread_num_(cfg->decode_thread_num())
    , freq_ghz_(cfg->freq_ghz())
    , creation_tsc_(rdtsc())
{
    frame_start_.calloc(config_->socket_thread_num(), kNumStatsFrames,
        Agora_memory::Alignment_t::k64Align);
}

Stats::~Stats() { frame_start_.free(); }

void Stats::PopulateSummary(
    FrameSummary* frame_summary, size_t thread_id, DoerType doer_type)
{
    DurationStat* ds = GetDurationStat(doer_type, thread_id);
    DurationStat* ds_old = GetDurationStatOld(doer_type, thread_id);

    frame_summary->count_this_thread = ds->task_count - ds_old->task_count;
    frame_summary->count_all_threads += frame_summary->count_this_thread;

    for (size_t j = 0; j < break_down_num_; j++) {
        frame_summary->us_this_thread[j] = cycles_to_us(
            ds->task_duration[j] - ds_old->task_duration[j], freq_ghz_);
        frame_summary->us_avg_threads[j] += frame_summary->us_this_thread[j];
    }
    *ds_old = *ds;
}

void Stats::ComputeAvgOverThreads(
    FrameSummary* frame_summary, size_t thread_num, size_t break_down_num_)
{
    for (size_t j = 0; j < break_down_num_; j++)
        frame_summary->us_avg_threads[j]
            = frame_summary->us_avg_threads[j] / thread_num;
}

void Stats::PrintPerThreadPerTask(FrameSummary s)
{
    std::printf("%zu tasks %.1f us (~ %.1f + %.1f + %.1f us), ",
        s.count_this_thread, s.us_this_thread[0] / s.count_this_thread,
        s.us_this_thread[1] / s.count_this_thread,
        s.us_this_thread[2] / s.count_this_thread,
        s.us_this_thread[3] / s.count_this_thread);
}

void Stats::PrintPerFrame(const char* doer_string, FrameSummary frame_summary)
{
    std::printf("%s (%zu tasks): %.3f ms (~ %.4f + %.3f + %.4f ms), ",
        doer_string, frame_summary.count_all_threads,
        frame_summary.us_avg_threads[0] / 1000.0,
        frame_summary.us_avg_threads[1] / 1000.0,
        frame_summary.us_avg_threads[2] / 1000.0,
        frame_summary.us_avg_threads[3] / 1000.0);
}

void Stats::UpdateStats(size_t frame_id)
{
    this->last_frame_id_ = frame_id;
    size_t frame_slot = (frame_id % kNumStatsFrames);

    if (kIsWorkerTimingEnabled == true) {
        //\TODO combine
        UpdateStatsInFunctionsDownlink(frame_slot);
        UpdateStatsInFunctionsUplink(frame_slot);
    }
}

void Stats::UpdateStatsInFunctionsUplink(size_t frame_id)
{
    this->last_frame_id_ = frame_id;
    size_t frame_slot = (frame_id % kNumStatsFrames);
    if (kIsWorkerTimingEnabled == true) {
        FrameSummary fft_frame_summary;
        FrameSummary csi_frame_summary;
        FrameSummary zf_frame_summary;
        FrameSummary demul_frame_summary;
        FrameSummary decode_frame_summary;

        if (config_->bigstation_mode() == true) {
            UpdateStatsInFunctionsUplinkBigstation(frame_slot,
                &fft_frame_summary, &csi_frame_summary, &zf_frame_summary,
                &demul_frame_summary, &decode_frame_summary);
        } else {
            UpdateStatsInFunctionsUplinkAgora(frame_slot,
                &fft_frame_summary, &csi_frame_summary, &zf_frame_summary,
                &demul_frame_summary, &decode_frame_summary);
        }

        this->fft_us_.at(frame_slot) = fft_frame_summary.us_avg_threads[0];
        this->csi_us_.at(frame_slot) = csi_frame_summary.us_avg_threads[0];
        this->zf_us_.at(frame_slot) = zf_frame_summary.us_avg_threads[0];
        this->demul_us_.at(frame_slot) = demul_frame_summary.us_avg_threads[0];
        this->decode_us_.at(frame_slot)
            = decode_frame_summary.us_avg_threads[0];

        const double sum_us_this_frame = this->fft_us_.at(frame_slot)
            + this->csi_us_.at(frame_slot) + this->zf_us_.at(frame_slot)
            + this->demul_us_.at(frame_slot) + this->decode_us_.at(frame_slot);

        for (size_t i = 1; i < this->break_down_num_; i++) {
            this->fft_breakdown_us_.at(i - 1).at(frame_slot)
                = fft_frame_summary.us_avg_threads[i];
            this->csi_breakdown_us_.at(i - 1).at(frame_slot)
                = csi_frame_summary.us_avg_threads[i];
            this->zf_breakdown_us_.at(i - 1).at(frame_slot)
                = zf_frame_summary.us_avg_threads[i];
            this->demul_breakdown_us_.at(i - 1).at(frame_slot)
                = demul_frame_summary.us_avg_threads[i];
            this->decode_breakdown_us_.at(i - 1).at(frame_slot)
                = decode_frame_summary.us_avg_threads[i];
        }

        if (kStatsPrintFrameSummary == true) {
            std::printf("Frame %zu summary: ", frame_id);
            PrintPerFrame("FFT", fft_frame_summary);
            PrintPerFrame("CSI", csi_frame_summary);
            PrintPerFrame("ZF", zf_frame_summary);
            PrintPerFrame("Demul", demul_frame_summary);
            PrintPerFrame("Decode", decode_frame_summary);
            std::printf("Total: %.2f ms\n", sum_us_this_frame / 1000);
        }
    }
}

void Stats::UpdateStatsInFunctionsDownlink(size_t frame_id)
{
    this->last_frame_id_ = frame_id;
    size_t frame_slot = frame_id % kNumStatsFrames;

    if (kIsWorkerTimingEnabled == true) {
        FrameSummary ifft_frame_summary;
        FrameSummary csi_frame_summary;
        FrameSummary zf_frame_summary;
        FrameSummary precode_frame_summary;
        FrameSummary encode_frame_summary;

        if (config_->bigstation_mode() == true) {
            UpdateStatsInFunctionsDownlinkBigstation(frame_slot,
                &ifft_frame_summary, &csi_frame_summary, &zf_frame_summary,
                &precode_frame_summary, &encode_frame_summary);
        } else {
            UpdateStatsInFunctionsDownlinkAgora(frame_slot,
                &ifft_frame_summary, &csi_frame_summary, &zf_frame_summary,
                &precode_frame_summary, &encode_frame_summary);
        }

        this->csi_us_.at(frame_slot) = csi_frame_summary.us_avg_threads[0];
        this->ifft_us_.at(frame_slot) = ifft_frame_summary.us_avg_threads[0];
        this->zf_us_.at(frame_slot) = zf_frame_summary.us_avg_threads[0];
        this->precode_us_.at(frame_slot)
            = precode_frame_summary.us_avg_threads[0];
        this->encode_us_.at(frame_slot)
            = encode_frame_summary.us_avg_threads[0];

        const double sum_us_this_frame = this->csi_us_.at(frame_slot)
            + this->ifft_us_.at(frame_slot) + this->zf_us_.at(frame_slot)
            + this->precode_us_.at(frame_slot)
            + this->encode_us_.at(frame_slot);

        if (kStatsPrintFrameSummary == true) {
            std::printf("Frame %zu summary: ", frame_id);
            PrintPerFrame("CSI", csi_frame_summary);
            PrintPerFrame("ZF", zf_frame_summary);
            PrintPerFrame("IFFT", ifft_frame_summary);
            PrintPerFrame("Precode", precode_frame_summary);
            PrintPerFrame("Encode", encode_frame_summary);
            std::printf("Total: %.2f ms\n", sum_us_this_frame / 1000.0);
        }
    }
}

void Stats::UpdateStatsInDofftBigstation(size_t frame_id, size_t thread_num,
    size_t thread_num_offset, FrameSummary* fft_frame_summary,
    FrameSummary* csi_frame_summary)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        PopulateSummary(fft_frame_summary, i, DoerType::kFFT);
        PopulateSummary(csi_frame_summary, i, DoerType::kCSI);

        if (kDebugPrintStatsPerThread) {
            double sum_us_this_frame_this_thread
                = fft_frame_summary->us_this_thread[0]
                + csi_frame_summary->us_this_thread[0];
            std::printf("In frame %zu, thread %zu, \t", frame_id, i);
            std::printf("csi: ");
            PrintPerThreadPerTask(*csi_frame_summary);
            std::printf("fft: ");
            PrintPerThreadPerTask(*fft_frame_summary);
            std::printf("sum: %.3f us\n", sum_us_this_frame_this_thread);
        }
    }
    ComputeAvgOverThreads(fft_frame_summary, thread_num, break_down_num_);
    ComputeAvgOverThreads(csi_frame_summary, thread_num, break_down_num_);
}

void Stats::UpdateStatsInDozfBigstation(size_t frame_id, size_t thread_num,
    size_t thread_num_offset, FrameSummary* zf_frame_summary)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        PopulateSummary(zf_frame_summary, i, DoerType::kZF);

        if (kDebugPrintStatsPerThread) {
            double sum_us_this_frame_this_thread
                = zf_frame_summary->us_this_thread[0];
            std::printf("In frame %zu, thread %zu, \t", frame_id, i);
            std::printf("zf: ");
            PrintPerThreadPerTask(*zf_frame_summary);
            std::printf("sum: %.3f us\n", sum_us_this_frame_this_thread);
        }
    }
    ComputeAvgOverThreads(zf_frame_summary, thread_num, break_down_num_);
}

void Stats::UpdateStatsInDodemulBigstation(size_t frame_id,
    size_t thread_num, size_t thread_num_offset,
    FrameSummary* demul_frame_summary)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        PopulateSummary(demul_frame_summary, i, DoerType::kDemul);

        if (kDebugPrintStatsPerThread) {
            double sum_us_this_frame_this_thread
                = demul_frame_summary->us_this_thread[0];
            std::printf("In frame %zu, thread %zu, \t", frame_id, i);
            std::printf("demul: ");
            PrintPerThreadPerTask(*demul_frame_summary);
            std::printf("sum: %.3f us\n", sum_us_this_frame_this_thread);
        }
    }
    ComputeAvgOverThreads(demul_frame_summary, thread_num, break_down_num_);
}

void Stats::UpdateStatsInDodecodeBigstation(size_t frame_id,
    size_t thread_num, size_t thread_num_offset,
    FrameSummary* decode_frame_summary)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        PopulateSummary(decode_frame_summary, i, DoerType::kDecode);

        if (kDebugPrintStatsPerThread) {
            double sum_us_this_frame_this_thread
                = decode_frame_summary->us_this_thread[0];
            std::printf("In frame %zu, thread %zu, \t", frame_id, i);
            std::printf("decode: ");
            PrintPerThreadPerTask(*decode_frame_summary);
            std::printf("sum: %.3f us\n", sum_us_this_frame_this_thread);
        }
    }
    ComputeAvgOverThreads(decode_frame_summary, thread_num, break_down_num_);
}

void Stats::UpdateStatsInDoifftBigstation(size_t frame_id,
    size_t thread_num, size_t thread_num_offset,
    FrameSummary* ifft_frame_summary, FrameSummary* csi_frame_summary)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        PopulateSummary(ifft_frame_summary, i, DoerType::kIFFT);
        PopulateSummary(csi_frame_summary, i, DoerType::kCSI);

        if (kDebugPrintStatsPerThread) {
            double sum_time_this_frame_this_thread
                = ifft_frame_summary->us_this_thread[0]
                + csi_frame_summary->us_this_thread[0];
            std::printf("In frame %zu, thread %zu, \t", frame_id, i);
            std::printf("csi: ");
            PrintPerThreadPerTask(*csi_frame_summary);
            std::printf("ifft: ");
            PrintPerThreadPerTask(*ifft_frame_summary);
            std::printf("sum: %.3f us\n", sum_time_this_frame_this_thread);
        }
    }
    ComputeAvgOverThreads(ifft_frame_summary, thread_num, break_down_num_);
    ComputeAvgOverThreads(csi_frame_summary, thread_num, break_down_num_);
}

void Stats::UpdateStatsInDoprecodeBigstation(size_t frame_id,
    size_t thread_num, size_t thread_num_offset,
    FrameSummary* precode_frame_summary)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        PopulateSummary(precode_frame_summary, i, DoerType::kPrecode);

        if (kDebugPrintStatsPerThread) {
            double sum_us_this_frame_this_thread
                = precode_frame_summary->us_this_thread[0];
            std::printf("In frame %zu, thread %zu, \t", frame_id, i);
            std::printf("precode: ");
            PrintPerThreadPerTask(*precode_frame_summary);
            std::printf("sum: %.3f\n", sum_us_this_frame_this_thread);
        }
    }
    ComputeAvgOverThreads(
        precode_frame_summary, thread_num, break_down_num_);
}

void Stats::UpdateStatsInDoencodeBigstation(size_t frame_id,
    size_t thread_num, size_t thread_num_offset,
    FrameSummary* encode_frame_summary)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        PopulateSummary(encode_frame_summary, i, DoerType::kEncode);

        if (kDebugPrintStatsPerThread) {
            double sum_us_this_frame_this_thread
                = encode_frame_summary->us_this_thread[0];
            std::printf("In frame %zu, thread %zu, \t", frame_id, i);
            std::printf("precode: ");
            PrintPerThreadPerTask(*encode_frame_summary);
            std::printf("sum: %.3f\n", sum_us_this_frame_this_thread);
        }
    }
    ComputeAvgOverThreads(encode_frame_summary, thread_num, break_down_num_);
}

void Stats::UpdateStatsInFunctionsUplinkBigstation(size_t frame_id,
    FrameSummary* fft_frame_summary, FrameSummary* csi_frame_summary,
    FrameSummary* zf_frame_summary, FrameSummary* demul_frame_summary,
    FrameSummary* decode_frame_summary)
{
    UpdateStatsInDofftBigstation(
        frame_id, fft_thread_num_, 0, fft_frame_summary, csi_frame_summary);
    UpdateStatsInDozfBigstation(
        frame_id, zf_thread_num_, fft_thread_num_, zf_frame_summary);
    UpdateStatsInDodemulBigstation(frame_id, demul_thread_num_,
        fft_thread_num_ + zf_thread_num_, demul_frame_summary);
    UpdateStatsInDodecodeBigstation(frame_id, decode_thread_num_,
        fft_thread_num_ + zf_thread_num_ + demul_thread_num_,
        decode_frame_summary);
}

void Stats::UpdateStatsInFunctionsDownlinkBigstation(size_t frame_id,
    FrameSummary* ifft_frame_summary, FrameSummary* csi_frame_summary,
    FrameSummary* zf_frame_summary, FrameSummary* precode_frame_summary,
    FrameSummary* encode_frame_summary)
{
    UpdateStatsInDoifftBigstation(
        frame_id, fft_thread_num_, 0, ifft_frame_summary, csi_frame_summary);
    UpdateStatsInDozfBigstation(
        frame_id, zf_thread_num_, fft_thread_num_, zf_frame_summary);
    UpdateStatsInDoprecodeBigstation(frame_id, demul_thread_num_,
        fft_thread_num_ + zf_thread_num_, precode_frame_summary);
    UpdateStatsInDoencodeBigstation(frame_id, decode_thread_num_,
        fft_thread_num_ + zf_thread_num_ + demul_thread_num_,
        encode_frame_summary);
}

void Stats::UpdateStatsInFunctionsUplinkAgora(size_t frame_id,
    FrameSummary* fft_frame_summary, FrameSummary* csi_frame_summary,
    FrameSummary* zf_frame_summary, FrameSummary* demul_frame_summary,
    FrameSummary* decode_frame_summary)
{
    for (size_t i = 0; i < task_thread_num_; i++) {
        PopulateSummary(fft_frame_summary, i, DoerType::kFFT);
        PopulateSummary(csi_frame_summary, i, DoerType::kCSI);
        PopulateSummary(zf_frame_summary, i, DoerType::kZF);
        PopulateSummary(demul_frame_summary, i, DoerType::kDemul);
        PopulateSummary(decode_frame_summary, i, DoerType::kDecode);

        if (kDebugPrintStatsPerThread == true) {
            double sum_us_this_frame_this_thread
                = fft_frame_summary->us_this_thread[0]
                + csi_frame_summary->us_this_thread[0]
                + zf_frame_summary->us_this_thread[0]
                + demul_frame_summary->us_this_thread[0]
                + decode_frame_summary->us_this_thread[0];
            std::printf("In frame %zu, thread %zu, \t", frame_id, i);
            std::printf("csi: ");
            PrintPerThreadPerTask(*csi_frame_summary);
            std::printf("fft: ");
            PrintPerThreadPerTask(*fft_frame_summary);
            std::printf("zf: ");
            PrintPerThreadPerTask(*zf_frame_summary);
            std::printf("demul: ");
            PrintPerThreadPerTask(*demul_frame_summary);
            std::printf("decode: ");
            PrintPerThreadPerTask(*decode_frame_summary);
            std::printf("sum: %.3f\n", sum_us_this_frame_this_thread);
        }
    }
    ComputeAvgOverThreads(
        fft_frame_summary, task_thread_num_, break_down_num_);
    ComputeAvgOverThreads(
        csi_frame_summary, task_thread_num_, break_down_num_);
    ComputeAvgOverThreads(
        zf_frame_summary, task_thread_num_, break_down_num_);
    ComputeAvgOverThreads(
        demul_frame_summary, task_thread_num_, break_down_num_);
    ComputeAvgOverThreads(
        decode_frame_summary, task_thread_num_, break_down_num_);
}

void Stats::UpdateStatsInFunctionsDownlinkAgora(size_t frame_id,
    FrameSummary* ifft_frame_summary, FrameSummary* csi_frame_summary,
    FrameSummary* zf_frame_summary, FrameSummary* precode_frame_summary,
    FrameSummary* encode_frame_summary)
{

    for (size_t i = 0; i < task_thread_num_; i++) {
        PopulateSummary(ifft_frame_summary, i, DoerType::kIFFT);
        PopulateSummary(csi_frame_summary, i, DoerType::kCSI);
        PopulateSummary(zf_frame_summary, i, DoerType::kZF);
        PopulateSummary(precode_frame_summary, i, DoerType::kPrecode);
        PopulateSummary(encode_frame_summary, i, DoerType::kEncode);

        if (kDebugPrintStatsPerThread == true) {
            double sum_us_this_frame_this_thread
                = ifft_frame_summary->us_this_thread[0]
                + csi_frame_summary->us_this_thread[0]
                + zf_frame_summary->us_this_thread[0]
                + precode_frame_summary->us_this_thread[0]
                + encode_frame_summary->us_this_thread[0];
            std::printf("In frame %zu, thread %zu, \t", frame_id, i);
            std::printf("csi: ");
            PrintPerThreadPerTask(*csi_frame_summary);
            std::printf("ifft: ");
            PrintPerThreadPerTask(*ifft_frame_summary);
            std::printf("zf: ");
            PrintPerThreadPerTask(*zf_frame_summary);
            std::printf("precode: ");
            PrintPerThreadPerTask(*precode_frame_summary);
            std::printf("encode: ");
            PrintPerThreadPerTask(*encode_frame_summary);
            std::printf("sum: %.3f us\n", sum_us_this_frame_this_thread);
        }
    }
    ComputeAvgOverThreads(
        ifft_frame_summary, task_thread_num_, break_down_num_);
    ComputeAvgOverThreads(
        csi_frame_summary, task_thread_num_, break_down_num_);
    ComputeAvgOverThreads(
        zf_frame_summary, task_thread_num_, break_down_num_);
    ComputeAvgOverThreads(
        precode_frame_summary, task_thread_num_, break_down_num_);
    ComputeAvgOverThreads(
        encode_frame_summary, task_thread_num_, break_down_num_);
}

void Stats::SaveToFile(void)
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/timeresult.txt";
    std::printf("Stats: Saving master timestamps to %s\n", filename.c_str());
    FILE* fp_debug = std::fopen(filename.c_str(), "w");
    rt_assert(fp_debug != nullptr,
        std::string("Open file failed ") + std::to_string(errno));

    //For backwards compatibility, it is easier to make a new file format for
    // the combined case
    if ((config_->frame().NumDLSyms() > 0)
        && (config_->frame().NumULSyms() > 0)) {
        std::fprintf(fp_debug,
            "Pilot RX by socket threads (= reference time), "
            "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
            "kZFDone, kPrecodeDone, kIFFTDone, kEncodeDone, kDemulDone, "
            "kDecodeDone, kRXDone, time in CSI, time in "
            "FFT, time in ZF, time in Demul, time in Decode\n");
        for (size_t i = 0; i < this->last_frame_id_; i++) {
            size_t ref_tsc = SIZE_MAX;
            for (size_t j = 0; j < config_->socket_thread_num(); j++) {
                ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
            }
            std::fprintf(fp_debug,
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
                "%.3f %.3f %.3f "
                "%.3f %.3f\n",
                cycles_to_us(ref_tsc - this->creation_tsc_, this->freq_ghz_),
                MasterGetUsFromRef(TsType::kPilotRX, i, ref_tsc),
                MasterGetUsFromRef(TsType::kProcessingStarted, i, ref_tsc),
                MasterGetUsFromRef(TsType::kPilotAllRX, i, ref_tsc),
                MasterGetUsFromRef(TsType::kFFTPilotsDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kZFDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kPrecodeDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kIFFTDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kEncodeDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kDemulDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kDecodeDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kRXDone, i, ref_tsc),
                this->csi_us_.at(i), this->fft_us_.at(i), this->zf_us_.at(i),
                this->demul_us_.at(i), this->decode_us_.at(i));
        }
    } else if (config_->frame().NumDLSyms() > 0) {
        std::fprintf(fp_debug,
            "Pilot RX by socket threads (= reference time), "
            "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
            "kZFDone, kPrecodeDone, kIFFTDone, kEncodeDone, kRXDone\n");
        for (size_t i = 0; i < this->last_frame_id_; i++) {
            size_t ref_tsc = SIZE_MAX;
            for (size_t j = 0; j < config_->socket_thread_num(); j++) {
                ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
            }
            std::fprintf(fp_debug,
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n",
                cycles_to_us(ref_tsc - this->creation_tsc_, this->freq_ghz_),
                MasterGetUsFromRef(TsType::kPilotRX, i, ref_tsc),
                MasterGetUsFromRef(TsType::kProcessingStarted, i, ref_tsc),
                MasterGetUsFromRef(TsType::kPilotAllRX, i, ref_tsc),
                MasterGetUsFromRef(TsType::kFFTPilotsDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kZFDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kPrecodeDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kIFFTDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kEncodeDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kRXDone, i, ref_tsc));
        }
    } else if (config_->frame().NumULSyms() > 0) {
        // Print the header
        std::fprintf(fp_debug,
            "Pilot RX by socket threads (= reference time), "
            "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTPilotsDone, "
            "kZFDone, kDemulDone, kDecodeDone, kRXDone, time in CSI, time in "
            "FFT, time in ZF, time in Demul, time in Decode\n");
        for (size_t i = 0; i < this->last_frame_id_; i++) {
            size_t ref_tsc = SIZE_MAX;
            for (size_t j = 0; j < config_->socket_thread_num(); j++) {
                ref_tsc = std::min(ref_tsc, this->frame_start_[j][i]);
            }
            std::fprintf(fp_debug,
                "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.3f %.3f %.3f "
                "%.3f %.3f\n",
                cycles_to_us(ref_tsc - this->creation_tsc_, this->freq_ghz_),
                MasterGetUsFromRef(TsType::kPilotRX, i, ref_tsc),
                MasterGetUsFromRef(TsType::kProcessingStarted, i, ref_tsc),
                MasterGetUsFromRef(TsType::kPilotAllRX, i, ref_tsc),
                MasterGetUsFromRef(TsType::kFFTPilotsDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kZFDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kDemulDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kDecodeDone, i, ref_tsc),
                MasterGetUsFromRef(TsType::kRXDone, i, ref_tsc),
                this->csi_us_.at(i), this->fft_us_.at(i), this->zf_us_.at(i),
                this->demul_us_.at(i), this->decode_us_.at(i));
        }
    } else {
        //Shouldn't happen
        rt_assert(
            false, std::string("No uplink or downlink symbols in the frame\n"));
    }

    std::fclose(fp_debug);

    if (kIsWorkerTimingEnabled == true) {
        std::string filename_detailed
            = cur_directory + "/data/timeresult_detail.txt";
        std::printf("Stats: Printing detailed results to %s\n",
            filename_detailed.c_str());

        FILE* fp_debug_detailed = std::fopen(filename_detailed.c_str(), "w");
        rt_assert(fp_debug_detailed != nullptr,
            std::string("Open file failed ") + std::to_string(errno));
        // Print the header
        std::fprintf(fp_debug_detailed,
            "fft_0, fft_1, fft_2, zf_0, zf_1, zf_2, demul_0, demul_1, demul_2, "
            "decode_0, decode_1, decode_2\n");

        for (size_t i = 0; i < this->last_frame_id_; i++) {
            std::fprintf(fp_debug_detailed,
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n",
                this->fft_breakdown_us_.at(0).at(i)
                    + this->csi_breakdown_us_.at(0).at(i),
                this->fft_breakdown_us_.at(1).at(i)
                    + this->csi_breakdown_us_.at(1).at(i),
                this->fft_breakdown_us_.at(2).at(i)
                    + this->csi_breakdown_us_.at(2).at(i),
                this->zf_breakdown_us_.at(0).at(i),
                this->zf_breakdown_us_.at(1).at(i),
                this->zf_breakdown_us_.at(2).at(i),
                this->demul_breakdown_us_.at(0).at(i),
                this->demul_breakdown_us_.at(1).at(i),
                this->demul_breakdown_us_.at(2).at(i),
                this->decode_breakdown_us_.at(0).at(i),
                this->decode_breakdown_us_.at(1).at(i),
                this->decode_breakdown_us_.at(2).at(i));
        }
        std::fclose(fp_debug_detailed);
    }
}

size_t Stats::GetTotalTaskCount(DoerType doer_type, size_t thread_num)
{
    size_t total_count = 0;
    for (size_t i = 0; i < thread_num; i++) {
        total_count = total_count + GetDurationStat(doer_type, i)->task_count;
    }
    return total_count;
}

void Stats::PrintSummary(void)
{
    std::printf(
        "Stats: total processed frames %zu\n", this->last_frame_id_ + 1);
    if (kIsWorkerTimingEnabled == false) {
        std::printf("Stats: Worker timing is disabled. Not printing summary\n");
    } else {
        size_t num_csi_tasks
            = GetTotalTaskCount(DoerType::kCSI, task_thread_num_);
        size_t num_fft_tasks
            = GetTotalTaskCount(DoerType::kFFT, task_thread_num_);
        size_t num_zf_tasks
            = GetTotalTaskCount(DoerType::kZF, task_thread_num_);
        size_t num_demul_tasks
            = GetTotalTaskCount(DoerType::kDemul, task_thread_num_);
        size_t num_decode_tasks
            = GetTotalTaskCount(DoerType::kDecode, task_thread_num_);
        size_t num_encode_tasks
            = GetTotalTaskCount(DoerType::kEncode, task_thread_num_);
        size_t num_ifft_tasks
            = GetTotalTaskCount(DoerType::kIFFT, task_thread_num_);
        size_t num_precode_tasks
            = GetTotalTaskCount(DoerType::kPrecode, task_thread_num_);
        double csi_frames = (double)num_csi_tasks / this->config_->bs_ant_num()
            / this->config_->frame().NumPilotSyms();
        double zf_frames
            = (double)num_zf_tasks / this->config_->zf_events_per_symbol();

        if (config_->frame().NumDLSyms() > 0) {
            double precode_frames = (double)num_precode_tasks
                / this->config_->ofdm_data_num()
                / this->config_->frame().NumDLSyms();
            double ifft_frames = (double)num_ifft_tasks
                / this->config_->bs_ant_num()
                / this->config_->frame().NumDLSyms();
            double encode_frames = (double)num_encode_tasks
                / this->config_->ldpc_config().num_blocks_in_symbol()
                / this->config_->ue_num() / this->config_->frame().NumDLSyms();
            std::printf("Downlink totals (tasks, frames): ");
            std::printf("CSI (%zu, %.2f), ", num_csi_tasks, csi_frames);
            std::printf("ZF (%zu, %.2f), ", num_zf_tasks, zf_frames);
            std::printf(
                "Encode (%zu, %.2f), ", num_encode_tasks, encode_frames);
            std::printf(
                "Precode (%zu, %.2f), ", num_precode_tasks, precode_frames);
            std::printf("IFFT (%zu, %.2f)", num_ifft_tasks, ifft_frames);
            std::printf("\n");
        } //config_->frame().NumDLSyms() > 0

        if (config_->frame().NumULSyms() > 0) {
            double fft_frames = (double)num_fft_tasks
                / this->config_->bs_ant_num()
                / this->config_->frame().NumULSyms();
            double demul_frames = (double)num_demul_tasks
                / this->config_->ofdm_data_num()
                / this->config_->frame().NumULSyms();
            double decode_frames = (double)num_decode_tasks
                / this->config_->ldpc_config().num_blocks_in_symbol()
                / this->config_->ue_num() / this->config_->frame().NumULSyms();
            std::printf("Uplink totals (tasks, frames): ");
            std::printf("CSI (%zu, %.2f), ", num_csi_tasks, csi_frames);
            std::printf("ZF (%zu, %.2f), ", num_zf_tasks, zf_frames);
            std::printf("FFT (%zu, %.2f), ", num_fft_tasks, fft_frames);
            std::printf("Demul (%zu, %.2f), ", num_demul_tasks, demul_frames);
            std::printf("Decode (%zu, %.2f)", num_decode_tasks, decode_frames);
            std::printf("\n");
        } //config_->frame().NumULSyms() > 0

        for (size_t i = 0; i < task_thread_num_; i++) {
            size_t num_csi_i = GetDurationStat(DoerType::kCSI, i)->task_count;
            size_t num_zf_i = GetDurationStat(DoerType::kZF, i)->task_count;
            size_t num_fft_i = GetDurationStat(DoerType::kFFT, i)->task_count;
            size_t num_precode_i
                = GetDurationStat(DoerType::kPrecode, i)->task_count;
            size_t num_ifft_i
                = GetDurationStat(DoerType::kIFFT, i)->task_count;
            size_t num_encode_i
                = GetDurationStat(DoerType::kEncode, i)->task_count;
            size_t num_demul_i
                = GetDurationStat(DoerType::kDemul, i)->task_count;
            size_t num_decode_i
                = GetDurationStat(DoerType::kDecode, i)->task_count;

            double percent_csi = (num_csi_i * 100.0f) / num_csi_tasks;
            double percent_zf = (num_zf_i * 100.0f) / num_zf_tasks;
            double percent_precode
                = (num_precode_i * 100.0f) / num_precode_tasks;
            double percent_ifft = (num_ifft_i * 100.0f) / num_ifft_tasks;
            double percent_encode = (num_encode_i * 100.0f) / num_encode_tasks;
            double percent_fft = (num_fft_i * 100.0f) / num_fft_tasks;
            double percent_demul = (num_demul_i * 100.0f) / num_demul_tasks;
            double percent_decode = (num_decode_i * 100.0f) / num_decode_tasks;
            std::printf("Thread %zu performed (tasks, fraction of tasks): ", i);
            std::printf("CSI (%zu, %.2f%%)", num_csi_i, percent_csi);
            std::printf(", ZF (%zu, %.2f%%)", num_zf_i, percent_zf);

            if (num_encode_i != 0) {
                std::printf(
                    ", Encode (%zu, %.2f%%)", num_encode_i, percent_encode);
            }
            if (num_precode_i != 0) {
                std::printf(
                    ", Precode (%zu, %.2f%%)", num_precode_i, percent_precode);
            }
            if (num_ifft_i != 0) {
                std::printf(", IFFT (%zu, %.2f%%)", num_ifft_i, percent_ifft);
            }
            if (num_fft_i != 0) {
                std::printf(", FFT (%zu, %.1f%%)", num_fft_i, percent_fft);
            }
            if (num_demul_i != 0) {
                std::printf(
                    ", Demul (%zu, %.1f%%)", num_demul_i, percent_demul);
            }
            if (num_decode_i != 0) {
                std::printf(
                    ", Decode (%zu, %.1f%%)", num_decode_i, percent_decode);
            }
            std::printf("\n");
        }
    } // kIsWorkerTimingEnabled == true
}
