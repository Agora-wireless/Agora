/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "stats.hpp"

#include <typeinfo>

Stats::Stats(Config* cfg, int break_down_num, int task_thread_num,
    int fft_thread_num, int zf_thread_num, int demul_thread_num,
    double freq_ghz)
    : config_(cfg)
    , task_thread_num(task_thread_num)
    , fft_thread_num(fft_thread_num)
    , zf_thread_num(zf_thread_num)
    , demul_thread_num(demul_thread_num)
    , break_down_num(break_down_num)
    , freq_ghz(freq_ghz)
{
    rt_assert(break_down_num <= (int)kMaxStatBreakdown,
        "Statistics breakdown too high");

    csi_time_in_function_details.calloc(
        break_down_num - 1, kNumStatsFrames, 4096);
    fft_time_in_function_details.calloc(
        break_down_num - 1, kNumStatsFrames, 4096);
    zf_time_in_function_details.calloc(
        break_down_num - 1, kNumStatsFrames, 4096);
    demul_time_in_function_details.calloc(
        break_down_num - 1, kNumStatsFrames, 4096);

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

    for (int j = 0; j < break_down_num; j++) {
        stats_per_frame->duration_this_thread[j] = cycles_to_us(
            ds->task_duration[j] - ds_old->task_duration[j], freq_ghz);
        stats_per_frame->duration_avg_threads[j]
            += stats_per_frame->duration_this_thread[j];
#if DEBUG_PRINT_STATS_PER_THREAD
        stats_per_frame->duration_this_thread_per_task[j]
            = stats_per_frame->duration_this_thread[j]
            / stats_per_frame->count_this_thread;
#endif
    }
    *ds_old = *ds;
}

void Stats::compute_avg_over_threads(
    Stats_worker_per_frame* stats_per_frame, int thread_num, int break_down_num)
{
    for (int j = 0; j < break_down_num; j++)
        stats_per_frame->duration_avg_threads[j]
            = stats_per_frame->duration_avg_threads[j] / thread_num;
}

void Stats::print_per_thread_per_task(Stats_worker_per_frame stats_per_frame)
{
    printf(" %d tasks %.1f (%.1f, %.1f, %.1f), ",
        stats_per_frame.count_this_thread,
        stats_per_frame.duration_this_thread_per_task[0],
        stats_per_frame.duration_this_thread_per_task[1],
        stats_per_frame.duration_this_thread_per_task[2],
        stats_per_frame.duration_this_thread_per_task[3]);
}

void Stats::print_per_frame(Stats_worker_per_frame stats_per_frame)
{
    printf("%d tasks %.1f (%.1f, %.1f, %.1f), ",
        stats_per_frame.count_all_threads,
        stats_per_frame.duration_avg_threads[0],
        stats_per_frame.duration_avg_threads[1],
        stats_per_frame.duration_avg_threads[2],
        stats_per_frame.duration_avg_threads[3]);
}

void Stats::update_stats_in_functions_uplink(int frame_id)
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

    double sum_time_this_frame = csi_stats_per_frame.duration_avg_threads[0]
        + fft_stats_per_frame.duration_avg_threads[0]
        + zf_stats_per_frame.duration_avg_threads[0]
        + demul_stats_per_frame.duration_avg_threads[0]
        + decode_stats_per_frame.duration_avg_threads[0];

    csi_time_in_function[frame_id]
        = csi_stats_per_frame.duration_avg_threads[0];
    fft_time_in_function[frame_id]
        = fft_stats_per_frame.duration_avg_threads[0];
    zf_time_in_function[frame_id] = zf_stats_per_frame.duration_avg_threads[0];
    demul_time_in_function[frame_id]
        = demul_stats_per_frame.duration_avg_threads[0];
    decode_time_in_function[frame_id]
        = decode_stats_per_frame.duration_avg_threads[0];

    for (int i = 1; i < break_down_num; i++) {
        csi_time_in_function_details[i - 1][frame_id]
            = csi_stats_per_frame.duration_avg_threads[i];
        fft_time_in_function_details[i - 1][frame_id]
            = fft_stats_per_frame.duration_avg_threads[i];
        zf_time_in_function_details[i - 1][frame_id]
            = zf_stats_per_frame.duration_avg_threads[i];
        demul_time_in_function_details[i - 1][frame_id]
            = demul_stats_per_frame.duration_avg_threads[i];
    }

#if DEBUG_PRINT_PER_FRAME_DONE
    printf("In frame %d (us): ", frame_id);
    printf("CSI: ");
    print_per_frame(csi_stats_per_frame);
    printf("FFT: ");
    print_per_frame(fft_stats_per_frame);
    printf("ZF: ");
    print_per_frame(zf_stats_per_frame);
    printf("Demul: ");
    print_per_frame(demul_stats_per_frame);
#if USE_LDPC
    printf("Decode: ");
    print_per_frame(decode_stats_per_frame);
#endif
    printf("Total: %.1f\n", sum_time_this_frame);
#endif
}

void Stats::update_stats_in_functions_downlink(int frame_id)
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
    double sum_time_this_frame = csi_stats_per_frame.duration_avg_threads[0]
        + ifft_stats_per_frame.duration_avg_threads[0]
        + zf_stats_per_frame.duration_avg_threads[0]
        + precode_stats_per_frame.duration_avg_threads[0]
        + encode_stats_per_frame.duration_avg_threads[0];

    csi_time_in_function[frame_id]
        = csi_stats_per_frame.duration_avg_threads[0];
    ifft_time_in_function[frame_id]
        = ifft_stats_per_frame.duration_avg_threads[0];
    zf_time_in_function[frame_id] = zf_stats_per_frame.duration_avg_threads[0];
    precode_time_in_function[frame_id]
        = precode_stats_per_frame.duration_avg_threads[0];
    encode_time_in_function[frame_id]
        = encode_stats_per_frame.duration_avg_threads[0];

#if DEBUG_PRINT_PER_FRAME_DONE
    printf("In frame %d (us)", frame_id);
    printf("CSI: ");
    print_per_frame(csi_stats_per_frame);
    printf("IFFT: ");
    print_per_frame(ifft_stats_per_frame);
    printf("ZF: ");
    print_per_frame(zf_stats_per_frame);
    printf("Precode: ");
    print_per_frame(precode_stats_per_frame);
#if USE_LDPC
    printf("Encode: ");
    print_per_frame(encode_stats_per_frame);
#endif
    printf("Total: %.1f\n", sum_time_this_frame);
#endif
}

void Stats::update_stats_in_dofft_bigstation(UNUSED int frame_id,
    int thread_num, int thread_num_offset,
    Stats_worker_per_frame* fft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame)
{
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        update_stats_for_breakdowns(fft_stats_per_frame, i, DoerType::kFFT);
        update_stats_for_breakdowns(csi_stats_per_frame, i, DoerType::kCSI);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = fft_stats_per_frame->duration_this_thread[0]
            + csi_stats_per_frame->duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("csi: ");
        print_per_thread_per_task(*csi_stats_per_frame);
        printf("fft: ");
        print_per_thread_per_task(*fft_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(fft_stats_per_frame, thread_num, break_down_num);
    compute_avg_over_threads(csi_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_dozf_bigstation(UNUSED int frame_id, int thread_num,
    int thread_num_offset, Stats_worker_per_frame* zf_stats_per_frame)
{
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        update_stats_for_breakdowns(zf_stats_per_frame, i, DoerType::kZF);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = zf_stats_per_frame->duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("zf: ");
        print_per_thread_per_task(*zf_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(zf_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_dodemul_bigstation(UNUSED int frame_id,
    int thread_num, int thread_num_offset,
    Stats_worker_per_frame* demul_stats_per_frame)
{
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        update_stats_for_breakdowns(demul_stats_per_frame, i, DoerType::kDemul);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = demul_stats_per_frame->duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("demul: ");
        print_per_thread_per_task(*demul_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(demul_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_doifft_bigstation(UNUSED int frame_id,
    int thread_num, int thread_num_offset,
    Stats_worker_per_frame* ifft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame)
{
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        update_stats_for_breakdowns(ifft_stats_per_frame, i, DoerType::kIFFT);
        update_stats_for_breakdowns(csi_stats_per_frame, i, DoerType::kCSI);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = ifft_stats_per_frame->duration_this_thread[0]
            + csi_stats_per_frame->duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("csi: ");
        print_per_thread_per_task(*csi_stats_per_frame);
        printf("ifft: ");
        print_per_thread_per_task(*ifft_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(ifft_stats_per_frame, thread_num, break_down_num);
    compute_avg_over_threads(csi_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_doprecode_bigstation(UNUSED int frame_id,
    int thread_num, int thread_num_offset,
    Stats_worker_per_frame* precode_stats_per_frame)
{
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        update_stats_for_breakdowns(
            precode_stats_per_frame, i, DoerType::kPrecode);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = precode_stats_per_frame->duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("precode: ");
        print_per_thread_per_task(*precode_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(
        precode_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_functions_uplink_bigstation(int frame_id,
    Stats_worker_per_frame* fft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame,
    Stats_worker_per_frame* zf_stats_per_frame,
    Stats_worker_per_frame* demul_stats_per_frame,
    Stats_worker_per_frame* decode_stats_per_frame)
{
    update_stats_in_dofft_bigstation(
        frame_id, fft_thread_num, 0, fft_stats_per_frame, csi_stats_per_frame);
    update_stats_in_dozf_bigstation(
        frame_id, zf_thread_num, fft_thread_num, zf_stats_per_frame);
    update_stats_in_dodemul_bigstation(frame_id, demul_thread_num,
        fft_thread_num + zf_thread_num, demul_stats_per_frame);
}

void Stats::update_stats_in_functions_downlink_bigstation(int frame_id,
    Stats_worker_per_frame* ifft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame,
    Stats_worker_per_frame* zf_stats_per_frame,
    Stats_worker_per_frame* precode_stats_per_frame,
    Stats_worker_per_frame* encode_stats_per_frame)
{
    update_stats_in_doifft_bigstation(
        frame_id, fft_thread_num, 0, ifft_stats_per_frame, csi_stats_per_frame);
    update_stats_in_dozf_bigstation(
        frame_id, zf_thread_num, fft_thread_num, zf_stats_per_frame);
    update_stats_in_doprecode_bigstation(frame_id, demul_thread_num,
        fft_thread_num + zf_thread_num, precode_stats_per_frame);
}

void Stats::update_stats_in_functions_uplink_millipede(UNUSED int frame_id,
    Stats_worker_per_frame* fft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame,
    Stats_worker_per_frame* zf_stats_per_frame,
    Stats_worker_per_frame* demul_stats_per_frame,
    Stats_worker_per_frame* decode_stats_per_frame)
{
    for (int i = 0; i < task_thread_num; i++) {
        update_stats_for_breakdowns(fft_stats_per_frame, i, DoerType::kFFT);
        update_stats_for_breakdowns(csi_stats_per_frame, i, DoerType::kCSI);
        update_stats_for_breakdowns(zf_stats_per_frame, i, DoerType::kZF);
        update_stats_for_breakdowns(demul_stats_per_frame, i, DoerType::kDemul);
        update_stats_for_breakdowns(
            decode_stats_per_frame, i, DoerType::kDecode);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = fft_stats_per_frame->duration_this_thread[0]
            + csi_stats_per_frame->duration_this_thread[0]
            + zf_stats_per_frame->duration_this_thread[0]
            + demul_stats_per_frame->duration_this_thread[0]
            + decode_stats_per_frame->duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("csi: ");
        print_per_thread_per_task(*csi_stats_per_frame);
        printf("fft: ");
        print_per_thread_per_task(*fft_stats_per_frame);
        printf("zf: ");
        print_per_thread_per_task(*zf_stats_per_frame);
        printf("demul: ");
        print_per_thread_per_task(*demul_stats_per_frame);
#if USE_LDPC
        printf("decode: ");
        print_per_thread_per_task(*decode_stats_per_frame);
#endif
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
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

void Stats::update_stats_in_functions_downlink_millipede(UNUSED int frame_id,
    Stats_worker_per_frame* ifft_stats_per_frame,
    Stats_worker_per_frame* csi_stats_per_frame,
    Stats_worker_per_frame* zf_stats_per_frame,
    Stats_worker_per_frame* precode_stats_per_frame,
    Stats_worker_per_frame* encode_stats_per_frame)
{

    for (int i = 0; i < task_thread_num; i++) {
        update_stats_for_breakdowns(ifft_stats_per_frame, i, DoerType::kIFFT);
        update_stats_for_breakdowns(csi_stats_per_frame, i, DoerType::kCSI);
        update_stats_for_breakdowns(zf_stats_per_frame, i, DoerType::kZF);
        update_stats_for_breakdowns(
            precode_stats_per_frame, i, DoerType::kPrecode);
        update_stats_for_breakdowns(
            encode_stats_per_frame, i, DoerType::kEncode);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = ifft_stats_per_frame->duration_this_thread[0]
            + csi_stats_per_frame->duration_this_thread[0]
            + zf_stats_per_frame->duration_this_thread[0]
            + precode_stats_per_frame->duration_this_thread[0]
            + encode_stats_per_frame->duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("csi: ");
        print_per_thread_per_task(*csi_stats_per_frame);
        printf("ifft: ");
        print_per_thread_per_task(*ifft_stats_per_frame);
        printf("zf: ");
        print_per_thread_per_task(*zf_stats_per_frame);
        printf("precode: ");
#if USE_LDPC
        print_per_thread_per_task(*precode_stats_per_frame);
        printf("encode: ");
#endif
        print_per_thread_per_task(*encode_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
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
                csi_time_in_function[ii], zf_time_in_function[ii],
                precode_time_in_function[ii], ifft_time_in_function[ii],
                cycles_to_us(
                    master_get_tsc(TsType::kProcessingStarted, ii), freq_ghz),
                cycles_to_us(frame_start[0][ii], freq_ghz));

            if (config_->socket_thread_num > 1)
                fprintf(fp_debug, " %.3f",
                    cycles_to_us(frame_start[1][ii], freq_ghz));
            fprintf(fp_debug, "\n");
        }
    } else {
        for (size_t ii = 0; ii < last_frame_id; ii++) {
            fprintf(fp_debug,
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                cycles_to_us(master_get_tsc(TsType::kPilotRX, ii), freq_ghz),
                cycles_to_us(master_get_tsc(TsType::kRXDone, ii), freq_ghz),
                cycles_to_us(master_get_tsc(TsType::kFFTDone, ii), freq_ghz),
                cycles_to_us(master_get_tsc(TsType::kZFDone, ii), freq_ghz),
                cycles_to_us(master_get_tsc(TsType::kDemulDone, ii), freq_ghz),
                csi_time_in_function[ii], fft_time_in_function[ii],
                zf_time_in_function[ii], demul_time_in_function[ii],
                cycles_to_us(
                    master_get_tsc(TsType::kProcessingStarted, ii), freq_ghz),
                cycles_to_us(frame_start[0][ii], freq_ghz));
            if (config_->socket_thread_num > 1)
                fprintf(fp_debug, " %.3f",
                    cycles_to_us(frame_start[1][ii], freq_ghz));
            fprintf(fp_debug, " %.3f\n",
                cycles_to_us(
                    master_get_tsc(TsType::kPilotAllRX, ii), freq_ghz));
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

        for (size_t ii = 0; ii < last_frame_id; ii++) {
            fprintf(fp_debug_detailed,
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n",
                fft_time_in_function_details[0][ii],
                fft_time_in_function_details[1][ii],
                fft_time_in_function_details[2][ii],
                zf_time_in_function_details[0][ii],
                zf_time_in_function_details[1][ii],
                zf_time_in_function_details[2][ii],
                demul_time_in_function_details[0][ii],
                demul_time_in_function_details[1][ii],
                demul_time_in_function_details[2][ii]);
        }
        fclose(fp_debug_detailed);
    }
}

int Stats::get_total_task_count(DoerType doer_type, int thread_num)
{
    int total_count = 0;
    for (int i = 0; i < thread_num; i++) {
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

    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int PILOT_NUM = config_->pilot_symbol_num_perframe;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    int dl_data_subframe_num_perframe = config_->dl_data_symbol_num_perframe;
    int csi_total_count = get_total_task_count(DoerType::kCSI, task_thread_num);
    int fft_total_count = get_total_task_count(DoerType::kFFT, task_thread_num);
    int zf_total_count = get_total_task_count(DoerType::kZF, task_thread_num);
    int demul_total_count
        = get_total_task_count(DoerType::kDemul, task_thread_num);
#if USE_LDPC
    int UE_NUM = config_->UE_NUM;
    int decode_total_count
        = get_total_task_count(DoerType::kDecode, task_thread_num);
    int encode_total_count
        = get_total_task_count(DoerType::kEncode, task_thread_num);
    int nblocksInSymbol = config_->LDPC_config.nblocksInSymbol;
#endif
    int ifft_total_count
        = get_total_task_count(DoerType::kIFFT, task_thread_num);
    int precode_total_count
        = get_total_task_count(DoerType::kPrecode, task_thread_num);
    double csi_frames = (double)csi_total_count / BS_ANT_NUM / PILOT_NUM;
    double zf_frames = (double)zf_total_count / config_->zf_block_num;

    if (config_->downlink_mode) {
        double precode_frames = (double)precode_total_count / OFDM_DATA_NUM
            / dl_data_subframe_num_perframe;
        double ifft_frames = (double)ifft_total_count / BS_ANT_NUM
            / dl_data_subframe_num_perframe;
        printf("Downlink totals: ");
        printf("CSI %d (%.2f frames), ", csi_total_count, csi_frames);
        printf("ZF: %d (%.2f frames), ", zf_total_count, zf_frames);
#if USE_LDPC
        double encode_frames = (double)encode_total_count / nblocksInSymbol
            / UE_NUM / dl_data_subframe_num_perframe;
        printf("Encode: %d (%.2f frames), ", encode_total_count, encode_frames);
#endif
        printf(
            "Precode: %d (%.2f frames), ", precode_total_count, precode_frames);
        printf("IFFT: %d (%.2f frames)", ifft_total_count, ifft_frames);
        printf("\n");
        for (int i = 0; i < task_thread_num; i++) {
            int num_csi_i = get_duration_stat(DoerType::kCSI, i)->task_count;
            int num_zf_i = get_duration_stat(DoerType::kZF, i)->task_count;
            int num_precode_i
                = get_duration_stat(DoerType::kPrecode, i)->task_count;
            int num_ifft_i = get_duration_stat(DoerType::kIFFT, i)->task_count;

            double percent_csi = num_csi_i * 100.0 / csi_total_count;
            double percent_zf = num_zf_i * 100.0 / zf_total_count;
            double percent_precode
                = num_precode_i * 100.0 / precode_total_count;
            double percent_ifft = num_ifft_i * 100.0 / ifft_total_count;
            printf("thread %d performed ", i);
            printf("CSI: %d (%.2f%%), ", num_csi_i, percent_csi);
            printf("ZF: %d (%.2f%%), ", num_zf_i, percent_zf);
#if USE_LDPC
            int num_encode_i
                = get_duration_stat(DoerType::kEncode, i)->task_count;
            double percent_encode = num_encode_i * 100.0 / encode_total_count;
            printf("Encode: %d (%.2f%%), ", num_encode_i, percent_encode);
#endif
            printf("Precode: %d (%.2f%%), ", num_precode_i, percent_precode);
            printf("IFFT: %d (%.2f%%)", num_ifft_i, percent_ifft);
            printf("\n");
        }
    } else {
        double fft_frames = (double)fft_total_count / BS_ANT_NUM
            / ul_data_subframe_num_perframe;
        double demul_frames = (double)demul_total_count / OFDM_DATA_NUM
            / ul_data_subframe_num_perframe;
        printf("Uplink totals: ");
        printf("CSI %d (%.2f frames), ", csi_total_count, csi_frames);
        printf("ZF: %d (%.2f frames), ", zf_total_count, zf_frames);
        printf("FFT: %d (%.2f frames), ", fft_total_count, fft_frames);
        printf("Demul: %d (%.2f frames) ", demul_total_count, demul_frames);
#if USE_LDPC
        double decode_frames = (double)decode_total_count / nblocksInSymbol
            / UE_NUM / ul_data_subframe_num_perframe;
        printf("Decode: %d (%.2f frames)", decode_total_count, decode_frames);
#endif
        printf("\n");
        for (int i = 0; i < task_thread_num; i++) {
            int num_csi_i = get_duration_stat(DoerType::kCSI, i)->task_count;
            int num_fft_i = get_duration_stat(DoerType::kIFFT, i)->task_count;
            int num_zf_i = get_duration_stat(DoerType::kZF, i)->task_count;
            int num_demul_i
                = get_duration_stat(DoerType::kDemul, i)->task_count;

            double percent_csi = num_csi_i * 100.0 / csi_total_count;
            double percent_fft = num_fft_i * 100.0 / fft_total_count;
            double percent_zf = num_zf_i * 100.0 / zf_total_count;
            double percent_demul = num_demul_i * 100.0 / demul_total_count;

            printf("Thread %d performed: ", i);
            printf("CSI: %d (%.1f%%), ", num_csi_i, percent_csi);
            printf("ZF: %d (%.1f%%), ", num_zf_i, percent_zf);
            printf("FFT: %d (%.1f%%), ", num_fft_i, percent_fft);
            printf("Demul: %d (%.1f%%) ", num_demul_i, percent_demul);
#if USE_LDPC
            int num_decode_i
                = get_duration_stat(DoerType::kDecode, i)->task_count;
            double percent_decode = num_decode_i * 100.0 / decode_total_count;
            printf("Decode: %d (%.1f%%) ", num_decode_i, percent_decode);
#endif
            printf("\n");
        }
    }
}
