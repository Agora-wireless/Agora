/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 *
 */
#include "stats.hpp"

#include <typeinfo>

Stats::Stats(Config* cfg, int break_down_num, int task_thread_num,
    int fft_thread_num, int zf_thread_num, int demul_thread_num)
    : config_(cfg)
    , task_thread_num(task_thread_num)
    , fft_thread_num(fft_thread_num)
    , zf_thread_num(zf_thread_num)
    , demul_thread_num(demul_thread_num)
    , break_down_num(break_down_num)
{
    rt_assert(break_down_num <= (int)kMaxStatBreakdown,
        "Statistics breakdown too high");

    init_stats_worker(&csi_stats_worker, task_thread_num, break_down_num);
    init_stats_worker(&fft_stats_worker, task_thread_num, break_down_num);
    init_stats_worker(&zf_stats_worker, task_thread_num, break_down_num);
    init_stats_worker(&demul_stats_worker, task_thread_num, break_down_num);
    init_stats_worker(&decode_stats_worker, task_thread_num, break_down_num);
    init_stats_worker(&encode_stats_worker, task_thread_num, break_down_num);
    init_stats_worker(&ifft_stats_worker, task_thread_num, break_down_num);
    init_stats_worker(&precode_stats_worker, task_thread_num, break_down_num);
    init_stats_worker(&rc_stats_worker, task_thread_num, break_down_num);

    init_stats_worker(&csi_stats_worker_old, task_thread_num, break_down_num);
    init_stats_worker(&fft_stats_worker_old, task_thread_num, break_down_num);
    init_stats_worker(&zf_stats_worker_old, task_thread_num, break_down_num);
    init_stats_worker(&demul_stats_worker_old, task_thread_num, break_down_num);
    init_stats_worker(
        &decode_stats_worker_old, task_thread_num, break_down_num);
    init_stats_worker(
        &encode_stats_worker_old, task_thread_num, break_down_num);
    init_stats_worker(&ifft_stats_worker_old, task_thread_num, break_down_num);
    init_stats_worker(
        &precode_stats_worker_old, task_thread_num, break_down_num);
    init_stats_worker(&rc_stats_worker_old, task_thread_num, break_down_num);

#if DEBUG_UPDATE_STATS_DETAILED
    csi_time_in_function_details.calloc(
        break_down_num - 1, kNumStatsFrames, 4096);
    fft_time_in_function_details.calloc(
        break_down_num - 1, kNumStatsFrames, 4096);
    zf_time_in_function_details.calloc(
        break_down_num - 1, kNumStatsFrames, 4096);
    demul_time_in_function_details.calloc(
        break_down_num - 1, kNumStatsFrames, 4096);
#endif

    frame_start.calloc(config_->socket_thread_num, kNumStatsFrames, 64);
}

Stats::~Stats()
{
    free_stats_worker(&csi_stats_worker, task_thread_num);
    free_stats_worker(&fft_stats_worker, task_thread_num);
    free_stats_worker(&zf_stats_worker, task_thread_num);
    free_stats_worker(&demul_stats_worker, task_thread_num);
    free_stats_worker(&decode_stats_worker, task_thread_num);
    free_stats_worker(&encode_stats_worker, task_thread_num);
    free_stats_worker(&ifft_stats_worker, task_thread_num);
    free_stats_worker(&precode_stats_worker, task_thread_num);
    free_stats_worker(&rc_stats_worker, task_thread_num);

    free_stats_worker(&csi_stats_worker_old, task_thread_num);
    free_stats_worker(&fft_stats_worker_old, task_thread_num);
    free_stats_worker(&zf_stats_worker_old, task_thread_num);
    free_stats_worker(&demul_stats_worker_old, task_thread_num);
    free_stats_worker(&decode_stats_worker_old, task_thread_num);
    free_stats_worker(&encode_stats_worker_old, task_thread_num);
    free_stats_worker(&ifft_stats_worker_old, task_thread_num);
    free_stats_worker(&precode_stats_worker_old, task_thread_num);
    free_stats_worker(&rc_stats_worker_old, task_thread_num);
}

void Stats::init_stats_worker(
    Stats_worker* stats_in_worker, int thread_num, int break_down_num)
{
    stats_in_worker->task_duration.calloc(thread_num * 8, break_down_num, 32);
    alloc_buffer_1d(&stats_in_worker->task_count, thread_num * 16, 32, 1);
}

void Stats::free_stats_worker(
    Stats_worker* stats_in_worker, UNUSED int thread_num)
{
    stats_in_worker->task_duration.free();
    free_buffer_1d(&stats_in_worker->task_count);
}

void Stats::update_stats_for_breakdowns(Stats_worker_per_frame* stats_per_frame,
    Stats_worker stats_in_worker, Stats_worker* stats_in_worker_old,
    int thread_id, int break_down_num)
{
    stats_per_frame->count_this_thread
        = stats_in_worker.task_count[thread_id * 16]
        - stats_in_worker_old->task_count[thread_id * 16];
    stats_per_frame->count_all_threads += stats_per_frame->count_this_thread;
    stats_in_worker_old->task_count[thread_id * 16]
        = stats_in_worker.task_count[thread_id * 16];
    for (int j = 0; j < break_down_num; j++) {
        stats_per_frame->duration_this_thread[j]
            = stats_in_worker.task_duration[thread_id * 8][j]
            - stats_in_worker_old->task_duration[thread_id * 8][j];
        stats_per_frame->duration_avg_threads[j]
            += stats_per_frame->duration_this_thread[j];
#if DEBUG_PRINT_STATS_PER_THREAD
        stats_per_frame->duration_this_thread_per_task[j]
            = stats_per_frame->duration_this_thread[j]
            / stats_per_frame->count_this_thread;
#endif
        stats_in_worker_old->task_duration[thread_id * 8][j]
            = stats_in_worker.task_duration[thread_id * 8][j];
    }
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
#if DEBUG_UPDATE_STATS
#if BIGSTATION
    update_stats_in_functions_uplink_bigstation(frame_id);
#else
    update_stats_in_functions_uplink_millipede(frame_id);
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

#if DEBUG_UPDATE_STATS_DETAILED
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
#endif

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
#endif

    last_frame_id = (size_t)frame_id;
}

void Stats::update_stats_in_functions_downlink(int frame_id)
{
#if DEBUG_UPDATE_STATS
#if BIGSTATION
    update_stats_in_functions_downlink_bigstation(frame_id);
#else
    update_stats_in_functions_downlink_millipede(frame_id);
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
#endif

    last_frame_id = (size_t)frame_id;
}

void Stats::update_stats_in_dofft(
    UNUSED int frame_id, int thread_num, int thread_num_offset)
{
    fft_stats_per_frame.reset();
    csi_stats_per_frame.reset();

    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        /* compute stats for FFT */
        update_stats_for_breakdowns(&fft_stats_per_frame, fft_stats_worker,
            &fft_stats_worker_old, i, break_down_num);
        /* compute stats for CSI */
        update_stats_for_breakdowns(&csi_stats_per_frame, csi_stats_worker,
            &csi_stats_worker_old, i, break_down_num);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = fft_stats_per_frame.duration_this_thread[0]
            + csi_stats_per_frame.duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("csi: ");
        print_per_thread_per_task(csi_stats_per_frame);
        printf("fft: ");
        print_per_thread_per_task(fft_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(&fft_stats_per_frame, thread_num, break_down_num);
    compute_avg_over_threads(&csi_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_dozf(
    UNUSED int frame_id, int thread_num, int thread_num_offset)
{
    zf_stats_per_frame.reset();
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        /* compute stats for ZF */
        update_stats_for_breakdowns(&zf_stats_per_frame, zf_stats_worker,
            &zf_stats_worker_old, i, break_down_num);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = zf_stats_per_frame.duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("zf: ");
        print_per_thread_per_task(zf_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(&zf_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_dodemul(
    UNUSED int frame_id, int thread_num, int thread_num_offset)
{
    demul_stats_per_frame.reset();
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        /* compute stats for Demul */
        update_stats_for_breakdowns(&demul_stats_per_frame, demul_stats_worker,
            &demul_stats_worker_old, i, break_down_num);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = demul_stats_per_frame.duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("demul: ");
        print_per_thread_per_task(demul_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(
        &demul_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_doifft(
    UNUSED int frame_id, int thread_num, int thread_num_offset)
{
    ifft_stats_per_frame.reset();
    csi_stats_per_frame.reset();
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        /* compute stats for IFFT */
        update_stats_for_breakdowns(&fft_stats_per_frame, fft_stats_worker,
            &fft_stats_worker_old, i, break_down_num);
        /* compute stats for CSI */
        update_stats_for_breakdowns(&csi_stats_per_frame, csi_stats_worker,
            &csi_stats_worker_old, i, break_down_num);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = ifft_stats_per_frame.duration_this_thread[0]
            + csi_stats_per_frame.duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("csi: ");
        print_per_thread_per_task(csi_stats_per_frame);
        printf("ifft: ");
        print_per_thread_per_task(ifft_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(&ifft_stats_per_frame, thread_num, break_down_num);
    compute_avg_over_threads(&csi_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_doprecode(
    UNUSED int frame_id, int thread_num, int thread_num_offset)
{
    precode_stats_per_frame.reset();
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
        /* compute stats for Precode */
        update_stats_for_breakdowns(&precode_stats_per_frame,
            precode_stats_worker, &precode_stats_worker_old, i, break_down_num);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = precode_stats_per_frame.duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("precode: ");
        print_per_thread_per_task(precode_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(
        &precode_stats_per_frame, thread_num, break_down_num);
}

void Stats::update_stats_in_functions_uplink_bigstation(int frame_id)
{
    update_stats_in_dofft(frame_id, fft_thread_num, 0);
    update_stats_in_dozf(frame_id, zf_thread_num, fft_thread_num);
    update_stats_in_dodemul(
        frame_id, demul_thread_num, fft_thread_num + zf_thread_num);
}

void Stats::update_stats_in_functions_downlink_bigstation(int frame_id)
{
    update_stats_in_doifft(frame_id, fft_thread_num, 0);
    update_stats_in_dozf(frame_id, zf_thread_num, fft_thread_num);
    update_stats_in_doprecode(
        frame_id, demul_thread_num, fft_thread_num + zf_thread_num);
}

void Stats::update_stats_in_functions_uplink_millipede(UNUSED int frame_id)
{
    fft_stats_per_frame.reset();
    csi_stats_per_frame.reset();
    zf_stats_per_frame.reset();
    demul_stats_per_frame.reset();
    decode_stats_per_frame.reset();

    for (int i = 0; i < task_thread_num; i++) {
        /* compute stats for FFT */
        update_stats_for_breakdowns(&fft_stats_per_frame, fft_stats_worker,
            &fft_stats_worker_old, i, break_down_num);
        /* compute stats for CSI */
        update_stats_for_breakdowns(&csi_stats_per_frame, csi_stats_worker,
            &csi_stats_worker_old, i, break_down_num);
        /* compute stats for ZF */
        update_stats_for_breakdowns(&zf_stats_per_frame, zf_stats_worker,
            &zf_stats_worker_old, i, break_down_num);
        /* compute stats for Demul */
        update_stats_for_breakdowns(&demul_stats_per_frame, demul_stats_worker,
            &demul_stats_worker_old, i, break_down_num);
        /* compute stats for Decode */
        update_stats_for_breakdowns(&decode_stats_per_frame,
            decode_stats_worker, &decode_stats_worker_old, i, break_down_num);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = fft_stats_per_frame.duration_this_thread[0]
            + csi_stats_per_frame.duration_this_thread[0]
            + zf_stats_per_frame.duration_this_thread[0]
            + demul_stats_per_frame.duration_this_thread[0]
            + decode_stats_per_frame.duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("csi: ");
        print_per_thread_per_task(csi_stats_per_frame);
        printf("fft: ");
        print_per_thread_per_task(fft_stats_per_frame);
        printf("zf: ");
        print_per_thread_per_task(zf_stats_per_frame);
        printf("demul: ");
        print_per_thread_per_task(demul_stats_per_frame);
#if USE_LDPC
        printf("decode: ");
        print_per_thread_per_task(decode_stats_per_frame);
#endif
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(
        &fft_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        &csi_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        &zf_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        &demul_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        &decode_stats_per_frame, task_thread_num, break_down_num);
}

void Stats::update_stats_in_functions_downlink_millipede(UNUSED int frame_id)
{
    ifft_stats_per_frame.reset();
    csi_stats_per_frame.reset();
    zf_stats_per_frame.reset();
    precode_stats_per_frame.reset();
    encode_stats_per_frame.reset();

    for (int i = 0; i < task_thread_num; i++) {
        /* compute stats for IFFT */
        update_stats_for_breakdowns(&ifft_stats_per_frame, ifft_stats_worker,
            &ifft_stats_worker_old, i, break_down_num);
        /* compute stats for CSI */
        update_stats_for_breakdowns(&csi_stats_per_frame, csi_stats_worker,
            &csi_stats_worker_old, i, break_down_num);
        /* compute stats for ZF */
        update_stats_for_breakdowns(&zf_stats_per_frame, zf_stats_worker,
            &zf_stats_worker_old, i, break_down_num);
        /* compute stats for Precode */
        update_stats_for_breakdowns(&precode_stats_per_frame,
            precode_stats_worker, &precode_stats_worker_old, i, break_down_num);
        /* compute stats for Encode */
        update_stats_for_breakdowns(&encode_stats_per_frame,
            encode_stats_worker, &encode_stats_worker_old, i, break_down_num);

#if DEBUG_PRINT_STATS_PER_THREAD
        double sum_time_this_frame_this_thread
            = ifft_stats_per_frame.duration_this_thread[0]
            + csi_stats_per_frame.duration_this_thread[0]
            + zf_stats_per_frame.duration_this_thread[0]
            + precode_stats_per_frame.duration_this_thread[0]
            + encode_stats_per_frame.duration_this_thread[0];
        printf("In frame %d, thread %d, \t", frame_id, i);
        printf("csi: ");
        print_per_thread_per_task(csi_stats_per_frame);
        printf("ifft: ");
        print_per_thread_per_task(ifft_stats_per_frame);
        printf("zf: ");
        print_per_thread_per_task(zf_stats_per_frame);
        printf("precode: ");
#if USE_LDPC
        print_per_thread_per_task(precode_stats_per_frame);
        printf("encode: ");
#endif
        print_per_thread_per_task(encode_stats_per_frame);
        printf("sum: %.3f\n", sum_time_this_frame_this_thread);
#endif
    }
    compute_avg_over_threads(
        &ifft_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        &csi_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        &zf_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        &precode_stats_per_frame, task_thread_num, break_down_num);
    compute_avg_over_threads(
        &encode_stats_per_frame, task_thread_num, break_down_num);
}

void Stats::save_to_file()
{
    std::string cur_directory = TOSTRING(PROJECT_DIRECTORY);
    std::string filename = cur_directory + "/data/timeresult.txt";
    FILE* fp_debug = fopen(filename.c_str(), "w");
    if (fp_debug == NULL) {
        printf("open file faild\n");
        std::cerr << "Error: " << strerror(errno) << std::endl;
        exit(0);
    }

    printf("Saving timestamps to data/timeresult.txt\n");

    if (config_->downlink_mode) {
        for (size_t ii = 0; ii < last_frame_id; ii++) {
            fprintf(fp_debug,
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
                "%.3f %.3f %.3f",
                master_get_timestamp(TsType::kPilotRX, ii),
                master_get_timestamp(TsType::kRXDone, ii),
                master_get_timestamp(TsType::kFFTDone, ii),
                master_get_timestamp(TsType::kZFDone, ii),
                master_get_timestamp(TsType::kPrecodeDone, ii),
                master_get_timestamp(TsType::kIFFTDone, ii),
                master_get_timestamp(TsType::kTXDone, ii),
                master_get_timestamp(TsType::kTXProcessedFirst, ii),
                csi_time_in_function[ii], zf_time_in_function[ii],
                precode_time_in_function[ii], ifft_time_in_function[ii],
                master_get_timestamp(TsType::kProcessingStarted, ii),
                frame_start[0][ii]);

            if (config_->socket_thread_num > 1)
                fprintf(fp_debug, " %.3f", frame_start[1][ii]);
            fprintf(fp_debug, "\n");
        }
    } else {
        for (size_t ii = 0; ii < last_frame_id; ii++) {
            fprintf(fp_debug,
                "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f",
                master_get_timestamp(TsType::kPilotRX, ii),
                master_get_timestamp(TsType::kRXDone, ii),
                master_get_timestamp(TsType::kFFTDone, ii),
                master_get_timestamp(TsType::kZFDone, ii),
                master_get_timestamp(TsType::kDemulDone, ii),
                csi_time_in_function[ii], fft_time_in_function[ii],
                zf_time_in_function[ii], demul_time_in_function[ii],
                master_get_timestamp(TsType::kProcessingStarted, ii),
                frame_start[0][ii]);
            if (config_->socket_thread_num > 1)
                fprintf(fp_debug, " %.3f", frame_start[1][ii]);
            fprintf(fp_debug, " %.3f\n",
                master_get_timestamp(TsType::kPilotAllRX, ii));
        }

#if DEBUG_UPDATE_STATS_DETAILED
        printf("Printing detailed results to data/timeresult_detail.txt\n");

        std::string filename_detailed
            = cur_directory + "/data/timeresult_detail.txt";
        FILE* fp_debug_detailed = fopen(filename_detailed.c_str(), "w");
        if (fp_debug_detailed == NULL) {
            printf("open file faild\n");
            std::cerr << "Error: " << strerror(errno) << std::endl;
            exit(0);
        }

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
#endif
    }
    fclose(fp_debug);
}

int Stats::compute_total_count(Stats_worker stats_in_worker, int thread_num)
{
    int total_count = 0;
    for (int i = 0; i < thread_num; i++)
        total_count = total_count + stats_in_worker.task_count[i * 16];
    return total_count;
}

double Stats::compute_count_percentage(
    Stats_worker stats_in_worker, int total_count, int thread_id)
{
    double percentage = 100
        * ((double)stats_in_worker.task_count[thread_id * 16] / total_count);
    return percentage;
}

void Stats::print_summary()
{
    printf("Stats: total processed frames %zu\n", last_frame_id + 1);
    int BS_ANT_NUM = config_->BS_ANT_NUM;
    int PILOT_NUM = config_->pilot_symbol_num_perframe;
    int OFDM_DATA_NUM = config_->OFDM_DATA_NUM;
    int ul_data_subframe_num_perframe = config_->ul_data_symbol_num_perframe;
    int dl_data_subframe_num_perframe = config_->dl_data_symbol_num_perframe;
    int CSI_total_count
        = compute_total_count(csi_stats_worker, task_thread_num);
    int FFT_total_count
        = compute_total_count(fft_stats_worker, task_thread_num);
    int ZF_total_count = compute_total_count(zf_stats_worker, task_thread_num);
    int Demul_total_count
        = compute_total_count(demul_stats_worker, task_thread_num);
#if USE_LDPC
    int UE_NUM = config_->UE_NUM;
    int Decode_total_count
        = compute_total_count(decode_stats_worker, task_thread_num);
    int Encode_total_count
        = compute_total_count(encode_stats_worker, task_thread_num);
    int nblocksInSymbol = config_->LDPC_config.nblocksInSymbol;
#endif
    int IFFT_total_count
        = compute_total_count(ifft_stats_worker, task_thread_num);
    int Precode_total_count
        = compute_total_count(precode_stats_worker, task_thread_num);
    double csi_frames = (double)CSI_total_count / BS_ANT_NUM / PILOT_NUM;
    double zf_frames = (double)ZF_total_count / config_->zf_block_num;
    if (config_->downlink_mode) {
        double precode_frames = (double)Precode_total_count / OFDM_DATA_NUM
            / dl_data_subframe_num_perframe;
        double ifft_frames = (double)IFFT_total_count / BS_ANT_NUM
            / dl_data_subframe_num_perframe;
        printf("Downlink totals: ");
        printf("CSI %d (%.2f frames), ", CSI_total_count, csi_frames);
        printf("ZF: %d (%.2f frames), ", ZF_total_count, zf_frames);
#if USE_LDPC
        double encode_frames = (double)Encode_total_count / nblocksInSymbol
            / UE_NUM / dl_data_subframe_num_perframe;
        printf("Encode: %d (%.2f frames), ", Encode_total_count, encode_frames);
#endif
        printf(
            "Precode: %d (%.2f frames), ", Precode_total_count, precode_frames);
        printf("IFFT: %d (%.2f frames)", IFFT_total_count, ifft_frames);
        printf("\n");
        for (int i = 0; i < task_thread_num; i++) {
            double percent_CSI = compute_count_percentage(
                csi_stats_worker, CSI_total_count, i);
            double percent_ZF
                = compute_count_percentage(zf_stats_worker, ZF_total_count, i);
            double percent_Precode = compute_count_percentage(
                precode_stats_worker, Precode_total_count, i);
            double percent_IFFT = compute_count_percentage(
                ifft_stats_worker, IFFT_total_count, i);
            printf("thread %d performed ", i);
            printf("CSI: %d (%.2f%%), ", csi_stats_worker.task_count[i * 16],
                percent_CSI);
            printf("ZF: %d (%.2f%%), ", zf_stats_worker.task_count[i * 16],
                percent_ZF);
#if USE_LDPC
            double percent_Encode = compute_count_percentage(
                encode_stats_worker, Encode_total_count, i);
            printf("Encode: %d (%.2f%%), ",
                encode_stats_worker.task_count[i * 16], percent_Encode);
#endif
            printf("Precode: %d (%.2f%%), ",
                precode_stats_worker.task_count[i * 16], percent_Precode);
            printf("IFFT: %d (%.2f%%)", ifft_stats_worker.task_count[i * 16],
                percent_IFFT);
            printf("\n");
        }
    } else {
        double fft_frames = (double)FFT_total_count / BS_ANT_NUM
            / ul_data_subframe_num_perframe;
        double demul_frames = (double)Demul_total_count / OFDM_DATA_NUM
            / ul_data_subframe_num_perframe;
        printf("Uplink totals: ");
        printf("CSI %d (%.2f frames), ", CSI_total_count, csi_frames);
        printf("ZF: %d (%.2f frames), ", ZF_total_count, zf_frames);
        printf("FFT: %d (%.2f frames), ", FFT_total_count, fft_frames);
        printf("Demul: %d (%.2f frames) ", Demul_total_count, demul_frames);
#if USE_LDPC
        double decode_frames = (double)Decode_total_count / nblocksInSymbol
            / UE_NUM / ul_data_subframe_num_perframe;
        printf("Decode: %d (%.2f frames)", Decode_total_count, decode_frames);
#endif
        printf("\n");
        for (int i = 0; i < task_thread_num; i++) {
            double percent_CSI = compute_count_percentage(
                csi_stats_worker, CSI_total_count, i);
            double percent_FFT = compute_count_percentage(
                fft_stats_worker, FFT_total_count, i);
            double percent_ZF
                = compute_count_percentage(zf_stats_worker, ZF_total_count, i);
            double percent_Demul = compute_count_percentage(
                demul_stats_worker, Demul_total_count, i);

            printf("Thread %d performed: ", i);
            printf("CSI: %d (%.1f%%), ", csi_stats_worker.task_count[i * 16],
                percent_CSI);
            printf("ZF: %d (%.1f%%), ", zf_stats_worker.task_count[i * 16],
                percent_ZF);
            printf("FFT: %d (%.1f%%), ", fft_stats_worker.task_count[i * 16],
                percent_FFT);
            printf("Demul: %d (%.1f%%) ", demul_stats_worker.task_count[i * 16],
                percent_Demul);
#if USE_LDPC
            double percent_Decode = compute_count_percentage(
                decode_stats_worker, Decode_total_count, i);
            printf("Decode: %d (%.1f%%) ",
                decode_stats_worker.task_count[i * 16], percent_Decode);
#endif
            printf("\n");
        }
    }
}
