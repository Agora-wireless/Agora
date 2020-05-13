/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "stats.hpp"

#include <typeinfo>

Stats::Stats(Config* cfg, size_t break_down_num, size_t task_thread_num,
    size_t fft_thread_num, size_t zf_thread_num, size_t demul_thread_num,
    double freq_ghz)
    : config_(cfg)
    , task_thread_num(task_thread_num)
    , fft_thread_num(fft_thread_num)
    , zf_thread_num(zf_thread_num)
    , demul_thread_num(demul_thread_num)
    , break_down_num(break_down_num)
    , freq_ghz(freq_ghz)
    , creation_tsc(rdtsc())
{
    rt_assert(
        break_down_num <= kMaxStatBreakdown, "Statistics breakdown too high");
    frame_start.calloc(config_->socket_thread_num, kNumStatsFrames, 64);
}

Stats::~Stats() {}

void Stats::update_stats_for_breakdowns(Stats_worker_per_frame* stats_per_frame,
    size_t thread_id, DoerType doer_type)
{
    DurationStat* ds = get_duration_stat(doer_type, thread_id);
    DurationStat* ds_old = get_duration_stat_old(doer_type, thread_id);

    stats_per_frame->count_this_thread = ds->task_count - ds_old->task_count;
    stats_per_frame->count_all_threads += stats_per_frame->count_this_thread;

    for (size_t j = 0; j < break_down_num; j++) {
        stats_per_frame->duration_this_thread[j] = cycles_to_us(
            ds->task_duration[j] - ds_old->task_duration[j], freq_ghz);
        stats_per_frame->duration_avg_threads[j]
            += stats_per_frame->duration_this_thread[j];
        if (kDebugPrintStatsPerThread) {
            stats_per_frame->duration_this_thread_per_task[j]
                = stats_per_frame->duration_this_thread[j]
                / stats_per_frame->count_this_thread;
        }
    }
    *ds_old = *ds;
}

void Stats::compute_avg_over_threads(Stats_worker_per_frame* stats_per_frame,
    size_t thread_num, size_t break_down_num)
{
    for (size_t j = 0; j < break_down_num; j++)
        stats_per_frame->duration_avg_threads[j]
            = stats_per_frame->duration_avg_threads[j] / thread_num;
}

void Stats::print_per_thread_per_task(Stats_worker_per_frame stats_per_frame)
{
    printf("%zu tasks %.1f (%.1f, %.1f, %.1f), ",
        stats_per_frame.count_this_thread,
        stats_per_frame.duration_this_thread_per_task[0],
        stats_per_frame.duration_this_thread_per_task[1],
        stats_per_frame.duration_this_thread_per_task[2],
        stats_per_frame.duration_this_thread_per_task[3]);
}

void Stats::print_per_frame(
    const char* doer_string, Stats_worker_per_frame stats_per_frame)
{
    printf("%s (%zu tasks): %.2f ms (~ %.2f + %.2f + %.2f ms), ", doer_string,
        stats_per_frame.count_all_threads,
        stats_per_frame.duration_avg_threads[0] / 1000.0,
        stats_per_frame.duration_avg_threads[1] / 1000.0,
        stats_per_frame.duration_avg_threads[2] / 1000.0,
        stats_per_frame.duration_avg_threads[3] / 1000.0);
}

void Stats::update_stats_in_functions_uplink(size_t frame_id)
{
    last_frame_id = (size_t)frame_id;
    if (!kIsWorkerTimingEnabled)
        return;

    Stats_worker_per_frame fft_stats_per_frame;
    Stats_worker_per_frame csi_stats_per_frame;
    Stats_worker_per_frame zf_stats_per_frame;
    Stats_worker_per_frame demul_stats_per_frame;
    Stats_worker_per_frame decode_stats_per_frame;

#if BIGSTATION
    update_stats_in_functions_uplink_bigstation(frame_id, &fft_stats_per_frame,
        &csi_stats_per_frame, &zf_stats_per_frame, &demul_stats_per_frame,
        &decode_stats_per_frame);
#else
    update_stats_in_functions_uplink_millipede(frame_id, &fft_stats_per_frame,
        &csi_stats_per_frame, &zf_stats_per_frame, &demul_stats_per_frame,
        &decode_stats_per_frame);
#endif

    fft_us[frame_id] = fft_stats_per_frame.duration_avg_threads[0];
    csi_us[frame_id] = csi_stats_per_frame.duration_avg_threads[0];
    zf_us[frame_id] = zf_stats_per_frame.duration_avg_threads[0];
    demul_us[frame_id] = demul_stats_per_frame.duration_avg_threads[0];
    decode_us[frame_id] = decode_stats_per_frame.duration_avg_threads[0];

    const double sum_us_this_frame = fft_us[frame_id] + csi_us[frame_id]
        + zf_us[frame_id] + demul_us[frame_id] + decode_us[frame_id];

    for (size_t i = 1; i < break_down_num; i++) {
        fft_breakdown_us[i - 1][frame_id]
            = fft_stats_per_frame.duration_avg_threads[i];
        csi_breakdown_us[i - 1][frame_id]
            = csi_stats_per_frame.duration_avg_threads[i];
        zf_breakdown_us[i - 1][frame_id]
            = zf_stats_per_frame.duration_avg_threads[i];
        demul_breakdown_us[i - 1][frame_id]
            = demul_stats_per_frame.duration_avg_threads[i];
    }

    if (kDebugPrintPerFrameDone) {
        printf("Frame %zu summary: ", frame_id);
        print_per_frame("FFT", fft_stats_per_frame);
        print_per_frame("CSI", csi_stats_per_frame);
        print_per_frame("ZF", zf_stats_per_frame);
        print_per_frame("Demul", demul_stats_per_frame);
        if (kUseLDPC) {
            print_per_frame("Decode", decode_stats_per_frame);
        }
        printf("Total: %.2f ms\n", sum_us_this_frame / 1000);
    }
}

void Stats::update_stats_in_functions_downlink(size_t frame_id)
{
    last_frame_id = (size_t)frame_id;
    if (!kIsWorkerTimingEnabled)
        return;

    Stats_worker_per_frame ifft_stats_per_frame;
    Stats_worker_per_frame csi_stats_per_frame;
    Stats_worker_per_frame zf_stats_per_frame;
    Stats_worker_per_frame precode_stats_per_frame;
    Stats_worker_per_frame encode_stats_per_frame;

#if BIGSTATION
    update_stats_in_functions_downlink_bigstation(frame_id,
        &ifft_stats_per_frame, &csi_stats_per_frame, &zf_stats_per_frame,
        &precode_stats_per_frame, &encode_stats_per_frame);
#else
    update_stats_in_functions_downlink_millipede(frame_id,
        &ifft_stats_per_frame, &csi_stats_per_frame, &zf_stats_per_frame,
        &precode_stats_per_frame, &encode_stats_per_frame);
#endif
    csi_us[frame_id] = csi_stats_per_frame.duration_avg_threads[0];
    ifft_us[frame_id] = ifft_stats_per_frame.duration_avg_threads[0];
    zf_us[frame_id] = zf_stats_per_frame.duration_avg_threads[0];
    precode_us[frame_id] = precode_stats_per_frame.duration_avg_threads[0];
    encode_us[frame_id] = encode_stats_per_frame.duration_avg_threads[0];

    const double sum_us_this_frame = csi_us[frame_id] + ifft_us[frame_id]
        + zf_us[frame_id] + precode_us[frame_id] + encode_us[frame_id];

    if (kDebugPrintPerFrameDone) {
        printf("Frame %zu summary: ", frame_id);
        print_per_frame("CSI", csi_stats_per_frame);
        print_per_frame("IFFT", ifft_stats_per_frame);
        print_per_frame("ZF", zf_stats_per_frame);
        print_per_frame("Precode", precode_stats_per_frame);
        if (kUseLDPC) {
            print_per_frame("Encode", encode_stats_per_frame);
        }
        printf("Total: %.2f ms\n", sum_us_this_frame / 1000.0);
    }
}

void Stats::update_stats_in_dofft_bigstation(size_t frame_id, size_t thread_num,
    size_t thread_num_offset, Stats_worker_per_frame* fft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        update_stats_for_breakdowns(fft_stats_per_frame, i, DoerType::kFFT);
        update_stats_for_breakdowns(csi_stats_per_frame, i, DoerType::kCSI);

        if (kDebugPrintStatsPerThread) {
            double sum_time_this_frame_this_thread
                = fft_stats_per_frame->duration_this_thread[0]
                + csi_stats_per_frame->duration_this_thread[0];
            printf("In frame %zu, thread %zu, \t", frame_id, i);
            printf("csi: ");
            print_per_thread_per_task(*csi_stats_per_frame);
            printf("fft: ");
            print_per_thread_per_task(*fft_stats_per_frame);
            printf("sum: %.3f\n", sum_time_this_frame_this_thread);
        }
    }
    compute_avg_over_threads(fft_stats_per_frame, thread_num, break_down_num);
    compute_avg_over_threads(csi_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_dozf_bigstation(size_t frame_id, size_t thread_num,
    size_t thread_num_offset, Stats_worker_per_frame* zf_stats_per_frame)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        update_stats_for_breakdowns(zf_stats_per_frame, i, DoerType::kZF);

        if (kDebugPrintStatsPerThread) {
            double sum_time_this_frame_this_thread
                = zf_stats_per_frame->duration_this_thread[0];
            printf("In frame %zu, thread %zu, \t", frame_id, i);
            printf("zf: ");
            print_per_thread_per_task(*zf_stats_per_frame);
            printf("sum: %.3f\n", sum_time_this_frame_this_thread);
        }
    }
    compute_avg_over_threads(zf_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_dodemul_bigstation(size_t frame_id,
    size_t thread_num, size_t thread_num_offset,
    Stats_worker_per_frame* demul_stats_per_frame)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        update_stats_for_breakdowns(demul_stats_per_frame, i, DoerType::kDemul);

        if (kDebugPrintStatsPerThread) {
            double sum_time_this_frame_this_thread
                = demul_stats_per_frame->duration_this_thread[0];
            printf("In frame %zu, thread %zu, \t", frame_id, i);
            printf("demul: ");
            print_per_thread_per_task(*demul_stats_per_frame);
            printf("sum: %.3f\n", sum_time_this_frame_this_thread);
        }
    }
    compute_avg_over_threads(demul_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_doifft_bigstation(size_t frame_id,
    size_t thread_num, size_t thread_num_offset,
    Stats_worker_per_frame* ifft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        update_stats_for_breakdowns(ifft_stats_per_frame, i, DoerType::kIFFT);
        update_stats_for_breakdowns(csi_stats_per_frame, i, DoerType::kCSI);

        if (kDebugPrintStatsPerThread) {
            double sum_time_this_frame_this_thread
                = ifft_stats_per_frame->duration_this_thread[0]
                + csi_stats_per_frame->duration_this_thread[0];
            printf("In frame %zu, thread %zu, \t", frame_id, i);
            printf("csi: ");
            print_per_thread_per_task(*csi_stats_per_frame);
            printf("ifft: ");
            print_per_thread_per_task(*ifft_stats_per_frame);
            printf("sum: %.3f\n", sum_time_this_frame_this_thread);
        }
    }
    compute_avg_over_threads(ifft_stats_per_frame, thread_num, break_down_num);
    compute_avg_over_threads(csi_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_doprecode_bigstation(size_t frame_id,
    size_t thread_num, size_t thread_num_offset,
    Stats_worker_per_frame* precode_stats_per_frame)
{
    for (size_t i = thread_num_offset; i < thread_num_offset + thread_num;
         i++) {
        update_stats_for_breakdowns(
            precode_stats_per_frame, i, DoerType::kPrecode);

        if (kDebugPrintStatsPerThread) {
            double sum_time_this_frame_this_thread
                = precode_stats_per_frame->duration_this_thread[0];
            printf("In frame %zu, thread %zu, \t", frame_id, i);
            printf("precode: ");
            print_per_thread_per_task(*precode_stats_per_frame);
            printf("sum: %.3f\n", sum_time_this_frame_this_thread);
        }
    }
    compute_avg_over_threads(
        precode_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_functions_uplink_bigstation(size_t frame_id,
    Stats_worker_per_frame* fft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame,
    Stats_worker_per_frame* zf_stats_per_frame,
    Stats_worker_per_frame* demul_stats_per_frame,
    UNUSED Stats_worker_per_frame* decode_stats_per_frame)
{
    update_stats_in_dofft_bigstation(
        frame_id, fft_thread_num, 0, fft_stats_per_frame, csi_stats_per_frame);
    update_stats_in_dozf_bigstation(
        frame_id, zf_thread_num, fft_thread_num, zf_stats_per_frame);
    update_stats_in_dodemul_bigstation(frame_id, demul_thread_num,
        fft_thread_num + zf_thread_num, demul_stats_per_frame);
}

void Stats::update_stats_in_functions_downlink_bigstation(size_t frame_id,
    Stats_worker_per_frame* ifft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame,
    Stats_worker_per_frame* zf_stats_per_frame,
    Stats_worker_per_frame* precode_stats_per_frame,
    UNUSED Stats_worker_per_frame* encode_stats_per_frame)
{
    update_stats_in_doifft_bigstation(
        frame_id, fft_thread_num, 0, ifft_stats_per_frame, csi_stats_per_frame);
    update_stats_in_dozf_bigstation(
        frame_id, zf_thread_num, fft_thread_num, zf_stats_per_frame);
    update_stats_in_doprecode_bigstation(frame_id, demul_thread_num,
        fft_thread_num + zf_thread_num, precode_stats_per_frame);
}

void Stats::update_stats_in_functions_uplink_millipede(size_t frame_id,
    Stats_worker_per_frame* fft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame,
    Stats_worker_per_frame* zf_stats_per_frame,
    Stats_worker_per_frame* demul_stats_per_frame,
    Stats_worker_per_frame* decode_stats_per_frame)
{
    for (size_t i = 0; i < task_thread_num; i++) {
        update_stats_for_breakdowns(fft_stats_per_frame, i, DoerType::kFFT);
        update_stats_for_breakdowns(csi_stats_per_frame, i, DoerType::kCSI);
        update_stats_for_breakdowns(zf_stats_per_frame, i, DoerType::kZF);
        update_stats_for_breakdowns(demul_stats_per_frame, i, DoerType::kDemul);
        update_stats_for_breakdowns(
            decode_stats_per_frame, i, DoerType::kDecode);

        if (kDebugPrintStatsPerThread) {
            double sum_time_this_frame_this_thread
                = fft_stats_per_frame->duration_this_thread[0]
                + csi_stats_per_frame->duration_this_thread[0]
                + zf_stats_per_frame->duration_this_thread[0]
                + demul_stats_per_frame->duration_this_thread[0]
                + decode_stats_per_frame->duration_this_thread[0];
            printf("In frame %zu, thread %zu, \t", frame_id, i);
            printf("csi: ");
            print_per_thread_per_task(*csi_stats_per_frame);
            printf("fft: ");
            print_per_thread_per_task(*fft_stats_per_frame);
            printf("zf: ");
            print_per_thread_per_task(*zf_stats_per_frame);
            printf("demul: ");
            print_per_thread_per_task(*demul_stats_per_frame);
            if (kUseLDPC) {
                printf("decode: ");
                print_per_thread_per_task(*decode_stats_per_frame);
            }
            printf("sum: %.3f\n", sum_time_this_frame_this_thread);
        }
    }
    compute_avg_over_threads(
        fft_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        csi_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        zf_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        demul_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        decode_stats_per_frame, task_thread_num, break_down_num);
}

void Stats::update_stats_in_functions_downlink_millipede(size_t frame_id,
    Stats_worker_per_frame* ifft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame,
    Stats_worker_per_frame* zf_stats_per_frame,
    Stats_worker_per_frame* precode_stats_per_frame,
    Stats_worker_per_frame* encode_stats_per_frame)
{

    for (size_t i = 0; i < task_thread_num; i++) {
        update_stats_for_breakdowns(ifft_stats_per_frame, i, DoerType::kIFFT);
        update_stats_for_breakdowns(csi_stats_per_frame, i, DoerType::kCSI);
        update_stats_for_breakdowns(zf_stats_per_frame, i, DoerType::kZF);
        update_stats_for_breakdowns(
            precode_stats_per_frame, i, DoerType::kPrecode);
        update_stats_for_breakdowns(
            encode_stats_per_frame, i, DoerType::kEncode);

        if (kDebugPrintStatsPerThread) {
            double sum_time_this_frame_this_thread
                = ifft_stats_per_frame->duration_this_thread[0]
                + csi_stats_per_frame->duration_this_thread[0]
                + zf_stats_per_frame->duration_this_thread[0]
                + precode_stats_per_frame->duration_this_thread[0]
                + encode_stats_per_frame->duration_this_thread[0];
            printf("In frame %zu, thread %zu, \t", frame_id, i);
            printf("csi: ");
            print_per_thread_per_task(*csi_stats_per_frame);
            printf("ifft: ");
            print_per_thread_per_task(*ifft_stats_per_frame);
            printf("zf: ");
            print_per_thread_per_task(*zf_stats_per_frame);
            printf("precode: ");
            if (kUseLDPC) {
                print_per_thread_per_task(*precode_stats_per_frame);
                printf("encode: ");
            }
            print_per_thread_per_task(*encode_stats_per_frame);
            printf("sum: %.3f\n", sum_time_this_frame_this_thread);
        }
    }
    compute_avg_over_threads(
        ifft_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        csi_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        zf_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        precode_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        encode_stats_per_frame, task_thread_num, break_down_num);
}

void Stats::save_to_file()
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/timeresult.txt";
    FILE* fp_debug = fopen(filename.c_str(), "w");
    rt_assert(fp_debug != nullptr,
        std::string("Open file failed ") + std::to_string(errno));
    printf("Stats: Saving master timestamps to %s\n", filename.c_str());

    if (config_->downlink_mode) {
        for (size_t ii = 0; ii < last_frame_id; ii++) {
            fprintf(fp_debug,
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
                "%.3f %.3f %.3f",
                cycles_to_us(master_get_tsc(TsType::kPilotRX, ii), freq_ghz),
                cycles_to_us(master_get_tsc(TsType::kRXDone, ii), freq_ghz),
                cycles_to_us(master_get_tsc(TsType::kFFTDone, ii), freq_ghz),
                cycles_to_us(master_get_tsc(TsType::kZFDone, ii), freq_ghz),
                cycles_to_us(
                    master_get_tsc(TsType::kPrecodeDone, ii), freq_ghz),
                cycles_to_us(master_get_tsc(TsType::kIFFTDone, ii), freq_ghz),
                cycles_to_us(master_get_tsc(TsType::kTXDone, ii), freq_ghz),
                cycles_to_us(
                    master_get_tsc(TsType::kTXProcessedFirst, ii), freq_ghz),
                csi_us[ii], zf_us[ii], precode_us[ii], ifft_us[ii],
                cycles_to_us(
                    master_get_tsc(TsType::kProcessingStarted, ii), freq_ghz),
                cycles_to_us(frame_start[0][ii], freq_ghz));

            if (config_->socket_thread_num > 1)
                fprintf(fp_debug, " %.3f",
                    cycles_to_us(frame_start[1][ii], freq_ghz));
            fprintf(fp_debug, "\n");
        }
    } else {
        // Print the header
        fprintf(fp_debug,
            "Pilot RX by socket threads (= reference time), "
            "kPilotRX, kProcessingStarted, kPilotAllRX, kFFTDone, kZFDone, "
            "kDemulDone, kRXDone, time in CSI, time in FFT, time in ZF, "
            "time in Demul\n");
        for (size_t i = 0; i < last_frame_id; i++) {
            size_t ref_tsc = SIZE_MAX;
            for (size_t j = 0; j < config_->socket_thread_num; j++) {
                ref_tsc = std::min(ref_tsc, frame_start[j][i]);
            }

            fprintf(fp_debug,
                "%.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f\n",
                cycles_to_us(ref_tsc - creation_tsc, freq_ghz),
                master_get_us_from_ref(TsType::kPilotRX, i, ref_tsc),
                master_get_us_from_ref(TsType::kProcessingStarted, i, ref_tsc),
                master_get_us_from_ref(TsType::kPilotAllRX, i, ref_tsc),
                master_get_us_from_ref(TsType::kFFTDone, i, ref_tsc),
                master_get_us_from_ref(TsType::kZFDone, i, ref_tsc),
                master_get_us_from_ref(TsType::kDemulDone, i, ref_tsc),
                master_get_us_from_ref(TsType::kRXDone, i, ref_tsc), csi_us[i],
                fft_us[i], zf_us[i], demul_us[i]);
        }
    }

    fclose(fp_debug);

    if (kIsWorkerTimingEnabled) {
        std::string filename_detailed
            = cur_directory + "/data/timeresult_detail.txt";
        printf("Stats: Printing detailed results to %s\n",
            filename_detailed.c_str());

        FILE* fp_debug_detailed = fopen(filename_detailed.c_str(), "w");
        rt_assert(fp_debug_detailed != nullptr,
            std::string("Open file failed ") + std::to_string(errno));

        for (size_t i = 0; i < last_frame_id; i++) {
            fprintf(fp_debug_detailed,
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n",
                fft_breakdown_us[0][i], fft_breakdown_us[1][i],
                fft_breakdown_us[2][i], zf_breakdown_us[0][i],
                zf_breakdown_us[1][i], zf_breakdown_us[2][i],
                demul_breakdown_us[0][i], demul_breakdown_us[1][i],
                demul_breakdown_us[2][i]);
        }
        fclose(fp_debug_detailed);
    }
}

size_t Stats::get_total_task_count(DoerType doer_type, size_t thread_num)
{
    size_t total_count = 0;
    for (size_t i = 0; i < thread_num; i++) {
        total_count = total_count + get_duration_stat(doer_type, i)->task_count;
    }
    return total_count;
}

void Stats::print_summary()
{
    printf("Stats: total processed frames %zu\n", last_frame_id + 1);
    if (!kIsWorkerTimingEnabled) {
        printf("Stats: Worker timing is disabled. Not printing summary\n");
        return;
    }

    auto& c = config_;
    size_t num_csi_tasks
        = get_total_task_count(DoerType::kCSI, task_thread_num);
    size_t num_fft_tasks
        = get_total_task_count(DoerType::kFFT, task_thread_num);
    size_t num_zf_tasks = get_total_task_count(DoerType::kZF, task_thread_num);
    size_t num_demul_tasks
        = get_total_task_count(DoerType::kDemul, task_thread_num);
    size_t num_decode_tasks = kUseLDPC
        ? get_total_task_count(DoerType::kDecode, task_thread_num)
        : -1;
    size_t num_encode_tasks = kUseLDPC
        ? get_total_task_count(DoerType::kEncode, task_thread_num)
        : -1;
    size_t num_ifft_tasks
        = get_total_task_count(DoerType::kIFFT, task_thread_num);
    size_t num_precode_tasks
        = get_total_task_count(DoerType::kPrecode, task_thread_num);
    double csi_frames
        = (double)num_csi_tasks / c->BS_ANT_NUM / c->pilot_symbol_num_perframe;
    double zf_frames = (double)num_zf_tasks / c->zf_events_per_symbol;

    if (c->downlink_mode) {
        double precode_frames = (double)num_precode_tasks / c->OFDM_DATA_NUM
            / c->dl_data_symbol_num_perframe;
        double ifft_frames = (double)num_ifft_tasks / c->BS_ANT_NUM
            / c->dl_data_symbol_num_perframe;
        printf("Downlink totals (tasks, frames): ");
        printf("CSI (%zu, %.2f), ", num_csi_tasks, csi_frames);
        printf("ZF (%zu, %.2f), ", num_zf_tasks, zf_frames);
        if (kUseLDPC) {
            double encode_frames = (double)num_encode_tasks
                / c->LDPC_config.nblocksInSymbol / c->UE_NUM
                / c->dl_data_symbol_num_perframe;
            printf("Encode (%zu, %.2f), ", num_encode_tasks, encode_frames);
        }
        printf("Precode (%zu, %.2f), ", num_precode_tasks, precode_frames);
        printf("IFFT (%zu, %.2f)", num_ifft_tasks, ifft_frames);
        printf("\n");
        for (size_t i = 0; i < task_thread_num; i++) {
            size_t num_csi_i = get_duration_stat(DoerType::kCSI, i)->task_count;
            size_t num_zf_i = get_duration_stat(DoerType::kZF, i)->task_count;
            size_t num_precode_i
                = get_duration_stat(DoerType::kPrecode, i)->task_count;
            size_t num_ifft_i
                = get_duration_stat(DoerType::kIFFT, i)->task_count;

            double percent_csi = num_csi_i * 100.0 / num_csi_tasks;
            double percent_zf = num_zf_i * 100.0 / num_zf_tasks;
            double percent_precode = num_precode_i * 100.0 / num_precode_tasks;
            double percent_ifft = num_ifft_i * 100.0 / num_ifft_tasks;
            printf("Thread %zu performed (tasks, fraction of tasks): ", i);
            printf("CSI (%zu, %.2f%%), ", num_csi_i, percent_csi);
            printf("ZF (%zu, %.2f%%), ", num_zf_i, percent_zf);
            if (kUseLDPC) {
                size_t num_encode_i
                    = get_duration_stat(DoerType::kEncode, i)->task_count;
                double percent_encode = num_encode_i * 100.0 / num_encode_tasks;
                printf("Encode (%zu, %.2f%%), ", num_encode_i, percent_encode);
            }
            printf("Precode (%zu, %.2f%%), ", num_precode_i, percent_precode);
            printf("IFFT (%zu, %.2f%%)", num_ifft_i, percent_ifft);
            printf("\n");
        }
    } else {
        double fft_frames = (double)num_fft_tasks / c->BS_ANT_NUM
            / c->ul_data_symbol_num_perframe;
        double demul_frames = (double)num_demul_tasks / c->OFDM_DATA_NUM
            / c->ul_data_symbol_num_perframe;
        printf("Uplink totals (tasks, frames): ");
        printf("CSI (%zu, %.2f), ", num_csi_tasks, csi_frames);
        printf("ZF (%zu, %.2f), ", num_zf_tasks, zf_frames);
        printf("FFT (%zu, %.2f), ", num_fft_tasks, fft_frames);
        printf("Demul (%zu, %.2f), ", num_demul_tasks, demul_frames);
        if (kUseLDPC) {
            double decode_frames = (double)num_decode_tasks
                / c->LDPC_config.nblocksInSymbol / c->UE_NUM
                / c->ul_data_symbol_num_perframe;
            printf("Decode (%zu, %.2f)", num_decode_tasks, decode_frames);
        }
        printf("\n");
        for (size_t i = 0; i < task_thread_num; i++) {
            size_t num_csi_i = get_duration_stat(DoerType::kCSI, i)->task_count;
            size_t num_fft_i = get_duration_stat(DoerType::kFFT, i)->task_count;
            size_t num_zf_i = get_duration_stat(DoerType::kZF, i)->task_count;
            size_t num_demul_i
                = get_duration_stat(DoerType::kDemul, i)->task_count;

            double percent_csi = num_csi_i * 100.0 / num_csi_tasks;
            double percent_fft = num_fft_i * 100.0 / num_fft_tasks;
            double percent_zf = num_zf_i * 100.0 / num_zf_tasks;
            double percent_demul = num_demul_i * 100.0 / num_demul_tasks;

            printf("Thread %zu performed (tasks, fraction of tasks): ", i);
            printf("CSI (%zu, %.1f%%), ", num_csi_i, percent_csi);
            printf("ZF (%zu, %.1f%%), ", num_zf_i, percent_zf);
            printf("FFT (%zu, %.1f%%), ", num_fft_i, percent_fft);
            printf("Demul (%zu, %.1f%%), ", num_demul_i, percent_demul);
            if (kUseLDPC) {
                size_t num_decode_i
                    = get_duration_stat(DoerType::kDecode, i)->task_count;
                double percent_decode = num_decode_i * 100.0 / num_decode_tasks;
                printf("Decode (%zu, %.1f%%) ", num_decode_i, percent_decode);
            }
            printf("\n");
        }
    }
}
