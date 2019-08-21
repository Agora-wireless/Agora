/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "stats.hpp"

Stats::Stats(double **in_CSI_task_duration, int *in_CSI_task_count, double **in_FFT_task_duration, int *in_FFT_task_count, 
          double **in_ZF_task_duration, int *in_ZF_task_count, double **in_Demul_task_duration, int *in_Demul_task_count,
          double **in_IFFT_task_duration, int *in_IFFT_task_count, double **in_Precode_task_duration, int *in_Precode_task_count,
          double **in_frame_start, 
          int in_task_duration_dim1, int in_task_duration_dim2, int in_task_count_dim,
          int in_task_thread_num, int in_fft_thread_num, int in_zf_thread_num, int in_demul_thread_num)
{
    printf("Initialize stats manager\n");

    task_thread_num = in_task_thread_num;
    fft_thread_num = in_fft_thread_num;
    zf_thread_num = in_zf_thread_num;
    demul_thread_num = in_demul_thread_num;

    task_duration_dim1 = in_task_duration_dim1;
    task_duration_dim2 = in_task_duration_dim2;
    task_count_dim = in_task_count_dim;

    CSI_task_duration = in_CSI_task_duration;
    FFT_task_duration = in_FFT_task_duration;
    ZF_task_duration = in_ZF_task_duration;
    Demul_task_duration = in_Demul_task_duration;
    IFFT_task_duration = in_IFFT_task_duration;
    Precode_task_duration = in_Precode_task_duration;

    CSI_task_count = in_CSI_task_count;
    FFT_task_count = in_FFT_task_count;
    ZF_task_count = in_ZF_task_count;
    Demul_task_count = in_Demul_task_count;
    IFFT_task_count = in_IFFT_task_count;
    Precode_task_count = in_Precode_task_count;

    frame_start = in_frame_start;
#if DEBUG_UPDATE_STATS_DETAILED
    alloc_buffer_2d(&csi_time_in_function_details, task_duration_dim2-1, 10000, 4096, 1);
    alloc_buffer_2d(&fft_time_in_function_details, task_duration_dim2-1, 10000, 4096, 1);
    alloc_buffer_2d(&zf_time_in_function_details, task_duration_dim2-1, 10000, 4096, 1);
    alloc_buffer_2d(&demul_time_in_function_details, task_duration_dim2-1, 10000, 4096, 1);
#endif
    alloc_buffer_1d(&csi_time_this_frame_this_thread, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&fft_time_this_frame_this_thread, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&zf_time_this_frame_this_thread, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&demul_time_this_frame_this_thread, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&ifft_time_this_frame_this_thread, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&precode_time_this_frame_this_thread, task_duration_dim2, 32, 1);

    alloc_buffer_1d(&csi_time_this_frame_this_thread_per_task, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&fft_time_this_frame_this_thread_per_task, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&zf_time_this_frame_this_thread_per_task, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&demul_time_this_frame_this_thread_per_task, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&ifft_time_this_frame_this_thread_per_task, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&precode_time_this_frame_this_thread_per_task, task_duration_dim2, 32, 1);

    alloc_buffer_1d(&csi_time_this_frame, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&fft_time_this_frame, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&zf_time_this_frame, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&demul_time_this_frame, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&ifft_time_this_frame, task_duration_dim2, 32, 1);
    alloc_buffer_1d(&precode_time_this_frame, task_duration_dim2, 32, 1);

    alloc_buffer_2d(&CSI_task_duration_prev_frame_each_thread, task_duration_dim2, task_thread_num, 32, 1);
    alloc_buffer_2d(&FFT_task_duration_prev_frame_each_thread, task_duration_dim2, task_thread_num, 32, 1);
    alloc_buffer_2d(&ZF_task_duration_prev_frame_each_thread, task_duration_dim2, task_thread_num, 32, 1);
    alloc_buffer_2d(&Demul_task_duration_prev_frame_each_thread, task_duration_dim2, task_thread_num, 32, 1);
    alloc_buffer_2d(&IFFT_task_duration_prev_frame_each_thread, task_duration_dim2, task_thread_num, 32, 1);
    alloc_buffer_2d(&Precode_task_duration_prev_frame_each_thread, task_duration_dim2, task_thread_num, 32, 1);

    alloc_buffer_1d(&CSI_task_count_prev_frame_each_thread, task_thread_num, 32, 1);
    alloc_buffer_1d(&FFT_task_count_prev_frame_each_thread, task_thread_num, 32, 1);
    alloc_buffer_1d(&ZF_task_count_prev_frame_each_thread, task_thread_num, 32, 1);
    alloc_buffer_1d(&Demul_task_count_prev_frame_each_thread, task_thread_num, 32, 1);
    alloc_buffer_1d(&IFFT_task_count_prev_frame_each_thread, task_thread_num, 32, 1);
    alloc_buffer_1d(&Precode_task_count_prev_frame_each_thread, task_thread_num, 32, 1);


    alloc_buffer_1d(&fft_time_this_frame_this_thread, task_duration_dim2, 32, 1);
}


void Stats::update_stats_in_functions_uplink(int frame_id) 
{
#if DEBUG_UPDATE_STATS 
	#if BIGSTATION
	    update_stats_in_functions_uplink_bigstation(frame_id);
	#else  
		update_stats_in_functions_uplink_millipede(frame_id); 
	#endif    
		double sum_time_this_frame = csi_time_this_frame[0] + fft_time_this_frame[0] + zf_time_this_frame[0] + demul_time_this_frame[0];
	    csi_time_in_function[frame_id] = csi_time_this_frame[0];
	    fft_time_in_function[frame_id] = fft_time_this_frame[0];
	    zf_time_in_function[frame_id] = zf_time_this_frame[0];
	    demul_time_in_function[frame_id] = demul_time_this_frame[0];
	#if DEBUG_UPDATE_STATS_DETAILED
	    for (int i = 0; i < task_duration_dim2 - 1; i++) {
	        csi_time_in_function_details[i][frame_id] = csi_time_this_frame[i+1];
	        fft_time_in_function_details[i][frame_id] = fft_time_this_frame[i+1];
	        zf_time_in_function_details[i][frame_id] = zf_time_this_frame[i+1];
	        demul_time_in_function_details[i][frame_id] = demul_time_this_frame[i+1];
	    }
	#endif
	#if DEBUG_PRINT_PER_FRAME_DONE
	    printf("In frame %d, \t\t\t\t\t csi: %d tasks %.3f (%.3f, %.3f, %.3f), fft: %d tasks %.3f (%.3f, %.3f, %.3f), " 
	    		"zf: %d tasks %.3f (%.3f, %.3f, %.3f), demul: %d tasks %.3f (%.3f, %.3f, %.3f), sum: %.3f\n", 
	            frame_id, csi_count_this_frame, csi_time_this_frame[0], csi_time_this_frame[1], csi_time_this_frame[2], csi_time_this_frame[3], 
	            fft_count_this_frame, fft_time_this_frame[0], fft_time_this_frame[1], fft_time_this_frame[2], fft_time_this_frame[3], 
	            zf_count_this_frame, zf_time_this_frame[0], zf_time_this_frame[1], zf_time_this_frame[2], zf_time_this_frame[3], 
	            demul_count_this_frame, demul_time_this_frame[0], demul_time_this_frame[1], demul_time_this_frame[2], demul_time_this_frame[3],
	            sum_time_this_frame);
	#endif     
#endif                  
}


void Stats::update_stats_in_functions_downlink(int frame_id) 
{
#if DEBUG_UPDATE_STATS 
	#if BIGSTATION
	    update_stats_in_functions_downlink_bigstation(frame_id);
	#else  
		update_stats_in_functions_downlink_millipede(frame_id); 
	#endif   
	    double sum_time_this_frame = csi_time_this_frame[0] + zf_time_this_frame[0] + precode_time_this_frame[0] + ifft_time_this_frame[0];
	    csi_time_in_function[frame_id] = csi_time_this_frame[0];
	    zf_time_in_function[frame_id] = zf_time_this_frame[0];
	    ifft_time_in_function[frame_id] = ifft_time_this_frame[0];
	    precode_time_in_function[frame_id] = precode_time_this_frame[0];
	#if DEBUG_PRINT_PER_FRAME_DONE
	    printf("In frame %d, \t\t\t\t\t csi: %d tasks %.3f (%.3f, %.3f, %.3f), ifft: %d tasks %.3f (%.3f, %.3f, %.3f), "
	    		"zf: %d tasks %.3f (%.3f, %.3f, %.3f), precode: %d tasks %.3f (%.3f, %.3f, %.3f), sum: %.3f\n", 
	            frame_id, csi_count_this_frame, csi_time_this_frame[0], csi_time_this_frame[1], csi_time_this_frame[2], csi_time_this_frame[3], 
	            ifft_count_this_frame, ifft_time_this_frame[0], ifft_time_this_frame[1], ifft_time_this_frame[2], ifft_time_this_frame[3], 
	            zf_count_this_frame, zf_time_this_frame[0], zf_time_this_frame[1], zf_time_this_frame[2], zf_time_this_frame[3], 
	            precode_count_this_frame, precode_time_this_frame[0], precode_time_this_frame[1], precode_time_this_frame[2], precode_time_this_frame[3],
	            sum_time_this_frame);
	#endif
#endif	    
}




void Stats::update_stats_in_dofft(int frame_id, int thread_num, int thread_num_offset)
{
	fft_count_this_frame = 0;
	csi_count_this_frame = 0;
	memset(fft_time_this_frame, 0, sizeof(double) * task_duration_dim2);
	memset(csi_time_this_frame, 0, sizeof(double) * task_duration_dim2);
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
    	/* compute stats for FFT */
        for (int j = 0; j < task_duration_dim2; j++) {
            fft_time_this_frame_this_thread[j] = FFT_task_duration[i * 8][j] - FFT_task_duration_prev_frame_each_thread[j][i];
            fft_time_this_frame[j] += fft_time_this_frame_this_thread[j];
            FFT_task_duration_prev_frame_each_thread[j][i] = FFT_task_duration[i * 8][j];
        }
        fft_count_this_frame_this_thread = FFT_task_count[i * 16] - FFT_task_count_prev_frame_each_thread[i];
        fft_count_this_frame += fft_count_this_frame_this_thread;
        FFT_task_count_prev_frame_each_thread[i] = FFT_task_count[i * 16];
        
        /* compute stats for CSI */
        double csi_time_this_frame_this_thread[4];
        for (int j = 0; j < task_duration_dim2; j++) {                    
            csi_time_this_frame_this_thread[j] = CSI_task_duration[i * 8][j] - CSI_task_duration_prev_frame_each_thread[j][i];
            csi_time_this_frame[j] += csi_time_this_frame_this_thread[j];
            CSI_task_duration_prev_frame_each_thread[j][i] = CSI_task_duration[i * 8][j];
        }
        csi_count_this_frame_this_thread = CSI_task_count[i * 16] - CSI_task_count_prev_frame_each_thread[i];
        csi_count_this_frame += csi_count_this_frame_this_thread;
        CSI_task_count_prev_frame_each_thread[i] = CSI_task_count[i * 16];

#if DEBUG_PRINT_STATS_PER_THREAD
        for (int j = 0; j < task_duration_dim2; j++) {
        	fft_time_this_frame_this_thread_per_task[j] = fft_time_this_frame_this_thread[j] / fft_count_this_frame_this_thread;
        	csi_time_this_frame_this_thread_per_task[j] = csi_time_this_frame_this_thread[j] / csi_count_this_frame_this_thread;
        }

        double sum_time_this_frame_this_thread = fft_time_this_frame_this_thread[0] + csi_time_this_frame_this_thread[0];
        printf("In frame %d, thread %d, \t\t\t csi: %d tasks %.3f (%.3f, %.3f, %.3f), fft: %d tasks %.3f (%.3f, %.3f, %.3f), sum: %.3f\n",
                frame_id, i, csi_count_this_frame_this_thread, 
                csi_time_this_frame_this_thread_per_task[0], csi_time_this_frame_this_thread_per_task[1], 
                csi_time_this_frame_this_thread_per_task[2], csi_time_this_frame_this_thread_per_task[3],
                fft_count_this_frame_this_thread, 
                fft_time_this_frame_this_thread_per_task[0], fft_time_this_frame_this_thread_per_task[1], 
                fft_time_this_frame_this_thread_per_task[2], fft_time_this_frame_this_thread_per_task[3],
                sum_time_this_frame_this_thread);
#endif
    }  
    for (int j = 0; j < task_duration_dim2; j++) {
    	fft_time_this_frame[j] = fft_time_this_frame[j] / thread_num;
    	csi_time_this_frame[j] = csi_time_this_frame[j] / thread_num;
    }
}

void Stats::update_stats_in_dozf(int frame_id, int thread_num, int thread_num_offset)
{
	zf_count_this_frame = 0;
	memset(zf_time_this_frame, 0, sizeof(double) * task_duration_dim2);
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
    	/* compute stats for ZF */
        for (int j = 0; j < task_duration_dim2; j++) {
            zf_time_this_frame_this_thread[j] = ZF_task_duration[i * 8][j] - ZF_task_duration_prev_frame_each_thread[j][i];
            zf_time_this_frame[j] += zf_time_this_frame_this_thread[j];
            ZF_task_duration_prev_frame_each_thread[j][i] = ZF_task_duration[i * 8][j];
        }
        zf_count_this_frame_this_thread = ZF_task_count[i * 16] - ZF_task_count_prev_frame_each_thread[i];
        zf_count_this_frame += zf_count_this_frame_this_thread;
        ZF_task_count_prev_frame_each_thread[i] = ZF_task_count[i * 16];

#if DEBUG_PRINT_STATS_PER_THREAD
        for (int j = 0; j < task_duration_dim2; j++) 
        	zf_time_this_frame_this_thread_per_task[j] = zf_time_this_frame_this_thread[j] / zf_count_this_frame_this_thread;
        double sum_time_this_frame_this_thread = zf_time_this_frame_this_thread[0];
        printf("In frame %d, thread %d, \t\t\t zf: %d tasks %.3f (%.3f, %.3f, %.3f), sum: %.3f\n",
                frame_id, i, zf_count_this_frame_this_thread, 
                zf_time_this_frame_this_thread_per_task[0], zf_time_this_frame_this_thread_per_task[1], 
                zf_time_this_frame_this_thread_per_task[2], zf_time_this_frame_this_thread_per_task[3],
                sum_time_this_frame_this_thread);
#endif
   }
    for (int j = 0; j < task_duration_dim2; j++) 
    	zf_time_this_frame[j] = zf_time_this_frame[j] / thread_num;
}


void Stats::update_stats_in_dodemul(int frame_id, int thread_num, int thread_num_offset)
{
	demul_count_this_frame = 0;
	memset(demul_time_this_frame, 0, sizeof(double) * task_duration_dim2);
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
    	/* compute stats for Demul */
        for (int j = 0; j < task_duration_dim2; j++) {
            demul_time_this_frame_this_thread[j] = Demul_task_duration[i * 8][j] - Demul_task_duration_prev_frame_each_thread[j][i];
            demul_time_this_frame[j] += demul_time_this_frame_this_thread[j];
            Demul_task_duration_prev_frame_each_thread[j][i] = Demul_task_duration[i * 8][j];
        }
        demul_count_this_frame_this_thread = Demul_task_count[i * 16] - Demul_task_count_prev_frame_each_thread[i];
        demul_count_this_frame += demul_count_this_frame_this_thread;
        Demul_task_count_prev_frame_each_thread[i] = Demul_task_count[i * 16];

#if DEBUG_PRINT_STATS_PER_THREAD
        for (int j = 0; j < task_duration_dim2; j++) 
        	demul_time_this_frame_this_thread_per_task[j] = demul_time_this_frame_this_thread[j] / demul_count_this_frame_this_thread;
        double sum_time_this_frame_this_thread = demul_time_this_frame_this_thread[0];
        printf("In frame %d, thread %d, \t\t\t demul: %d tasks %.3f (%.3f, %.3f, %.3f), sum: %.3f\n",
                frame_id, i, demul_count_this_frame_this_thread, 
                demul_time_this_frame_this_thread_per_task[0], demul_time_this_frame_this_thread_per_task[1], 
                demul_time_this_frame_this_thread_per_task[2], demul_time_this_frame_this_thread_per_task[3],
                sum_time_this_frame_this_thread);
#endif
   }
   for (int j = 0; j < task_duration_dim2; j++) 
    	demul_time_this_frame[j] = demul_time_this_frame[j] / thread_num;
}


void Stats::update_stats_in_doifft(int frame_id, int thread_num, int thread_num_offset)
{
	ifft_count_this_frame = 0;
	memset(ifft_time_this_frame, 0, sizeof(double) * task_duration_dim2);
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
    	/* compute stats for IFFT */
        for (int j = 0; j < task_duration_dim2; j++) {
            ifft_time_this_frame_this_thread[j] = IFFT_task_duration[i * 8][j] - IFFT_task_duration_prev_frame_each_thread[j][i];
            ifft_time_this_frame[j] += ifft_time_this_frame_this_thread[j];
            IFFT_task_duration_prev_frame_each_thread[j][i] = IFFT_task_duration[i * 8][j];
        }
        ifft_count_this_frame_this_thread = IFFT_task_count[i * 16] - IFFT_task_count_prev_frame_each_thread[i];
        ifft_count_this_frame += ifft_count_this_frame_this_thread;
        IFFT_task_count_prev_frame_each_thread[i] = IFFT_task_count[i * 16];
        
        /* compute stats for CSI */
        double csi_time_this_frame_this_thread[4];
        for (int j = 0; j < task_duration_dim2; j++) {                    
            csi_time_this_frame_this_thread[j] = CSI_task_duration[i * 8][j] - CSI_task_duration_prev_frame_each_thread[j][i];
            csi_time_this_frame[j] += csi_time_this_frame_this_thread[j];
            CSI_task_duration_prev_frame_each_thread[j][i] = CSI_task_duration[i * 8][j];
        }
        csi_count_this_frame_this_thread = CSI_task_count[i * 16] - CSI_task_count_prev_frame_each_thread[i];
        csi_count_this_frame += csi_count_this_frame_this_thread;
        CSI_task_count_prev_frame_each_thread[i] = CSI_task_count[i * 16];

#if DEBUG_PRINT_STATS_PER_THREAD
        for (int j = 0; j < task_duration_dim2; j++) {
        	ifft_time_this_frame_this_thread_per_task[j] = ifft_time_this_frame_this_thread[j] / ifft_count_this_frame_this_thread;
        	csi_time_this_frame_this_thread_per_task[j] = csi_time_this_frame_this_thread[j] / csi_count_this_frame_this_thread;
        }
        double sum_time_this_frame_this_thread = ifft_time_this_frame_this_thread[0] + csi_time_this_frame_this_thread[0];
        printf("In frame %d, thread %d, \t\t\t csi: %d tasks %.3f (%.3f, %.3f, %.3f), fft: %d tasks %.3f (%.3f, %.3f, %.3f), sum: %.3f\n",
                frame_id, i, csi_count_this_frame_this_thread, 
                csi_time_this_frame_this_thread_per_task[0], csi_time_this_frame_this_thread_per_task[1], 
                csi_time_this_frame_this_thread_per_task[2], csi_time_this_frame_this_thread_per_task[3],
                ifft_count_this_frame_this_thread, 
                ifft_time_this_frame_this_thread_per_task[0], ifft_time_this_frame_this_thread_per_task[1], 
                ifft_time_this_frame_this_thread_per_task[2], ifft_time_this_frame_this_thread_per_task[3],
                sum_time_this_frame_this_thread);
#endif
   }
    for (int j = 0; j < task_duration_dim2; j++) {
    	ifft_time_this_frame[j] = ifft_time_this_frame[j] / thread_num;
    	csi_time_this_frame[j] = csi_time_this_frame[j] / thread_num;
    }
}


void Stats::update_stats_in_doprecode(int frame_id, int thread_num, int thread_num_offset)
{
	precode_count_this_frame = 0;
	memset(precode_time_this_frame, 0, sizeof(double) * task_duration_dim2);
    for (int i = thread_num_offset; i < thread_num_offset + thread_num; i++) {
    	/* compute stats for Precode */
        for (int j = 0; j < task_duration_dim2; j++) {
            precode_time_this_frame_this_thread[j] = Precode_task_duration[i * 8][j] - Precode_task_duration_prev_frame_each_thread[j][i];
            precode_time_this_frame[j] += precode_time_this_frame_this_thread[j];
            Precode_task_duration_prev_frame_each_thread[j][i] = Precode_task_duration[i * 8][j];
        }
        precode_count_this_frame_this_thread = Precode_task_count[i * 16] - Precode_task_count_prev_frame_each_thread[i];
        precode_count_this_frame += precode_count_this_frame_this_thread;
        Precode_task_count_prev_frame_each_thread[i] = Precode_task_count[i * 16];

#if DEBUG_PRINT_STATS_PER_THREAD
        for (int j = 0; j < task_duration_dim2; j++) 
        	precode_time_this_frame_this_thread_per_task[j] = precode_time_this_frame_this_thread[j] / precode_count_this_frame_this_thread;
        double sum_time_this_frame_this_thread = precode_time_this_frame_this_thread[0];
        printf("In frame %d, thread %d, \t\t\t demul: %d tasks %.3f (%.3f, %.3f, %.3f), sum: %.3f\n",
                frame_id, i, precode_count_this_frame_this_thread, 
                precode_time_this_frame_this_thread_per_task[0], precode_time_this_frame_this_thread_per_task[1], 
                precode_time_this_frame_this_thread_per_task[2], precode_time_this_frame_this_thread_per_task[3],
                sum_time_this_frame_this_thread);
#endif
   }
    for (int j = 0; j < task_duration_dim2; j++) 
    	precode_time_this_frame[j] = precode_time_this_frame[j] / thread_num;
}


void Stats::update_stats_in_functions_uplink_bigstation(int frame_id) 
{
	update_stats_in_dofft(frame_id, fft_thread_num, 0);
    update_stats_in_dozf(frame_id, zf_thread_num, fft_thread_num);
    update_stats_in_dodemul(frame_id, demul_thread_num, fft_thread_num + zf_thread_num);
}


void Stats::update_stats_in_functions_downlink_bigstation(int frame_id)
{
	update_stats_in_doifft(frame_id, fft_thread_num, 0);
    update_stats_in_dozf(frame_id, zf_thread_num, fft_thread_num);
    update_stats_in_doprecode(frame_id, demul_thread_num, fft_thread_num + zf_thread_num);
}

void Stats::update_stats_in_functions_uplink_millipede(int frame_id) 
{
	fft_count_this_frame = 0;
	csi_count_this_frame = 0;
	zf_count_this_frame = 0;
	demul_count_this_frame = 0;
	memset(fft_time_this_frame, 0, sizeof(double) * task_duration_dim2);
	memset(csi_time_this_frame, 0, sizeof(double) * task_duration_dim2);
	memset(zf_time_this_frame, 0, sizeof(double) * task_duration_dim2);
	memset(demul_time_this_frame, 0, sizeof(double) * task_duration_dim2);

    for (int i = 0; i < task_thread_num; i++) {
    	/* compute stats for FFT */
        for (int j = 0; j < task_duration_dim2; j++) {
            fft_time_this_frame_this_thread[j] = FFT_task_duration[i * 8][j] - FFT_task_duration_prev_frame_each_thread[j][i];
            fft_time_this_frame[j] += fft_time_this_frame_this_thread[j];
            FFT_task_duration_prev_frame_each_thread[j][i] = FFT_task_duration[i * 8][j];
        }
        fft_count_this_frame_this_thread = FFT_task_count[i * 16] - FFT_task_count_prev_frame_each_thread[i];
        fft_count_this_frame += fft_count_this_frame_this_thread;
        FFT_task_count_prev_frame_each_thread[i] = FFT_task_count[i * 16];
        
        /* compute stats for CSI */
        double csi_time_this_frame_this_thread[4];
        for (int j = 0; j < task_duration_dim2; j++) {                    
            csi_time_this_frame_this_thread[j] = CSI_task_duration[i * 8][j] - CSI_task_duration_prev_frame_each_thread[j][i];
            csi_time_this_frame[j] += csi_time_this_frame_this_thread[j];
            CSI_task_duration_prev_frame_each_thread[j][i] = CSI_task_duration[i * 8][j];
        }
        csi_count_this_frame_this_thread = CSI_task_count[i * 16] - CSI_task_count_prev_frame_each_thread[i];
        csi_count_this_frame += csi_count_this_frame_this_thread;
        CSI_task_count_prev_frame_each_thread[i] = CSI_task_count[i * 16];

       	/* compute stats for ZF */
        for (int j = 0; j < task_duration_dim2; j++) {
            zf_time_this_frame_this_thread[j] = ZF_task_duration[i * 8][j] - ZF_task_duration_prev_frame_each_thread[j][i];
            zf_time_this_frame[j] += zf_time_this_frame_this_thread[j];
            ZF_task_duration_prev_frame_each_thread[j][i] = ZF_task_duration[i * 8][j];
        }
        zf_count_this_frame_this_thread = ZF_task_count[i * 16] - ZF_task_count_prev_frame_each_thread[i];
        zf_count_this_frame += zf_count_this_frame_this_thread;
        ZF_task_count_prev_frame_each_thread[i] = ZF_task_count[i * 16];

        /* compute stats for Demul */
        for (int j = 0; j < task_duration_dim2; j++) {
            demul_time_this_frame_this_thread[j] = Demul_task_duration[i * 8][j] - Demul_task_duration_prev_frame_each_thread[j][i];
            demul_time_this_frame[j] += demul_time_this_frame_this_thread[j];
            Demul_task_duration_prev_frame_each_thread[j][i] = Demul_task_duration[i * 8][j];
        }
        demul_count_this_frame_this_thread = Demul_task_count[i * 16] - Demul_task_count_prev_frame_each_thread[i];
        demul_count_this_frame += demul_count_this_frame_this_thread;
        Demul_task_count_prev_frame_each_thread[i] = Demul_task_count[i * 16];

#if DEBUG_PRINT_STATS_PER_THREAD
        for (int j = 0; j < task_duration_dim2; j++) {
        	fft_time_this_frame_this_thread_per_task[j] = fft_time_this_frame_this_thread[j] / fft_count_this_frame_this_thread;
        	csi_time_this_frame_this_thread_per_task[j] = csi_time_this_frame_this_thread[j] / csi_count_this_frame_this_thread;
        	zf_time_this_frame_this_thread_per_task[j] = zf_time_this_frame_this_thread[j] / zf_count_this_frame_this_thread;
        	demul_time_this_frame_this_thread_per_task[j] = demul_time_this_frame_this_thread[j] / demul_count_this_frame_this_thread;
        }
        double sum_time_this_frame_this_thread = fft_time_this_frame_this_thread[0] + csi_time_this_frame_this_thread[0]
        										+ zf_time_this_frame_this_thread[0] + demul_time_this_frame_this_thread[0];
       printf("In frame %d, thread %d, \t csi: %d tasks %.3f (%.3f, %.3f, %.3f), fft: %d tasks %.3f (%.3f, %.3f, %.3f), "
       			"zf: %d tasks %.3f (%.3f, %.3f, %.3f), demul: %d tasks %.3f (%.3f, %.3f, %.3f), sum: %.3f\n",
                frame_id, i, csi_count_this_frame_this_thread, 
                csi_time_this_frame_this_thread_per_task[0], csi_time_this_frame_this_thread_per_task[1], 
                csi_time_this_frame_this_thread_per_task[2], csi_time_this_frame_this_thread_per_task[3],
                fft_count_this_frame_this_thread, 
                fft_time_this_frame_this_thread_per_task[0], fft_time_this_frame_this_thread_per_task[1], 
                fft_time_this_frame_this_thread_per_task[2], fft_time_this_frame_this_thread_per_task[3],
                zf_count_this_frame_this_thread, 
                zf_time_this_frame_this_thread_per_task[0], zf_time_this_frame_this_thread_per_task[1], 
                zf_time_this_frame_this_thread_per_task[2], zf_time_this_frame_this_thread_per_task[3],
                demul_count_this_frame_this_thread, 
                demul_time_this_frame_this_thread_per_task[0], demul_time_this_frame_this_thread_per_task[1], 
                demul_time_this_frame_this_thread_per_task[2], demul_time_this_frame_this_thread_per_task[3],
                sum_time_this_frame_this_thread);
#endif
    }
    for (int j = 0; j < task_duration_dim2; j++) {
    	fft_time_this_frame[j] = fft_time_this_frame[j] / task_thread_num;
    	csi_time_this_frame[j] = csi_time_this_frame[j] / task_thread_num;
    	zf_time_this_frame[j] = zf_time_this_frame[j] / task_thread_num;
    	demul_time_this_frame[j] = demul_time_this_frame[j] / task_thread_num;
    }
}




void Stats::update_stats_in_functions_downlink_millipede(int frame_id)
{
	ifft_count_this_frame = 0;
	csi_count_this_frame = 0;
	zf_count_this_frame = 0;
	precode_count_this_frame = 0;
	memset(ifft_time_this_frame, 0, sizeof(double) * task_duration_dim2);
	memset(csi_time_this_frame, 0, sizeof(double) * task_duration_dim2);
	memset(zf_time_this_frame, 0, sizeof(double) * task_duration_dim2);
	memset(precode_time_this_frame, 0, sizeof(double) * task_duration_dim2);

    for (int i = 0; i < task_thread_num; i++) {
    	/* compute stats for IFFT */
        for (int j = 0; j < task_duration_dim2; j++) {
            ifft_time_this_frame_this_thread[j] = IFFT_task_duration[i * 8][j] - IFFT_task_duration_prev_frame_each_thread[j][i];
            ifft_time_this_frame[j] += ifft_time_this_frame_this_thread[j];
            IFFT_task_duration_prev_frame_each_thread[j][i] = IFFT_task_duration[i * 8][j];
        }
        ifft_count_this_frame_this_thread = IFFT_task_count[i * 16] - IFFT_task_count_prev_frame_each_thread[i];
        ifft_count_this_frame += ifft_count_this_frame_this_thread;
        IFFT_task_count_prev_frame_each_thread[i] = IFFT_task_count[i * 16];
        
        /* compute stats for CSI */
        double csi_time_this_frame_this_thread[4];
        for (int j = 0; j < task_duration_dim2; j++) {                    
            csi_time_this_frame_this_thread[j] = CSI_task_duration[i * 8][j] - CSI_task_duration_prev_frame_each_thread[j][i];
            csi_time_this_frame[j] += csi_time_this_frame_this_thread[j];
            CSI_task_duration_prev_frame_each_thread[j][i] = CSI_task_duration[i * 8][j];
        }
        csi_count_this_frame_this_thread = CSI_task_count[i * 16] - CSI_task_count_prev_frame_each_thread[i];
        csi_count_this_frame += csi_count_this_frame_this_thread;
        CSI_task_count_prev_frame_each_thread[i] = CSI_task_count[i * 16];

       	/* compute stats for ZF */
        for (int j = 0; j < task_duration_dim2; j++) {
            zf_time_this_frame_this_thread[j] = ZF_task_duration[i * 8][j] - ZF_task_duration_prev_frame_each_thread[j][i];
            zf_time_this_frame[j] += zf_time_this_frame_this_thread[j];
            ZF_task_duration_prev_frame_each_thread[j][i] = ZF_task_duration[i * 8][j];
        }
        zf_count_this_frame_this_thread = ZF_task_count[i * 16] - ZF_task_count_prev_frame_each_thread[i];
        zf_count_this_frame += zf_count_this_frame_this_thread;
        ZF_task_count_prev_frame_each_thread[i] = ZF_task_count[i * 16];

        /* compute stats for Precode */
        for (int j = 0; j < task_duration_dim2; j++) {
            precode_time_this_frame_this_thread[j] = Precode_task_duration[i * 8][j] - Precode_task_duration_prev_frame_each_thread[j][i];
            precode_time_this_frame[j] += precode_time_this_frame_this_thread[j];
            Precode_task_duration_prev_frame_each_thread[j][i] = Precode_task_duration[i * 8][j];
        }
        precode_count_this_frame_this_thread = Precode_task_count[i * 16] - Precode_task_count_prev_frame_each_thread[i];
        precode_count_this_frame += precode_count_this_frame_this_thread;
        Precode_task_count_prev_frame_each_thread[i] = Precode_task_count[i * 16];

#if DEBUG_PRINT_STATS_PER_THREAD
        for (int j = 0; j < task_duration_dim2; j++) {
        	ifft_time_this_frame_this_thread_per_task[j] = ifft_time_this_frame_this_thread[j] / ifft_count_this_frame_this_thread;
        	csi_time_this_frame_this_thread_per_task[j] = csi_time_this_frame_this_thread[j] / csi_count_this_frame_this_thread;
        	zf_time_this_frame_this_thread_per_task[j] = zf_time_this_frame_this_thread[j] / zf_count_this_frame_this_thread;
        	precode_time_this_frame_this_thread_per_task[j] = precode_time_this_frame_this_thread[j] / precode_count_this_frame_this_thread;
        }
        double sum_time_this_frame_this_thread = ifft_time_this_frame_this_thread[0] + csi_time_this_frame_this_thread[0]
        										+ zf_time_this_frame_this_thread[0] + precode_time_this_frame_this_thread[0];
       printf("In frame %d, thread %d, \t csi: %d tasks %.3f (%.3f, %.3f, %.3f), ifft: %d tasks %.3f (%.3f, %.3f, %.3f), "
       			"zf: %d tasks %.3f (%.3f, %.3f, %.3f), precode: %d tasks %.3f (%.3f, %.3f, %.3f), sum: %.3f\n",
                frame_id, i, csi_count_this_frame_this_thread, 
                csi_time_this_frame_this_thread_per_task[0], csi_time_this_frame_this_thread_per_task[1], 
                csi_time_this_frame_this_thread_per_task[2], csi_time_this_frame_this_thread_per_task[3],
                ifft_count_this_frame_this_thread, 
                ifft_time_this_frame_this_thread_per_task[0], ifft_time_this_frame_this_thread_per_task[1], 
                ifft_time_this_frame_this_thread_per_task[2], ifft_time_this_frame_this_thread_per_task[3],
                zf_count_this_frame_this_thread, 
                zf_time_this_frame_this_thread_per_task[0], zf_time_this_frame_this_thread_per_task[1], 
                zf_time_this_frame_this_thread_per_task[2], zf_time_this_frame_this_thread_per_task[3],
                precode_count_this_frame_this_thread, 
                precode_time_this_frame_this_thread_per_task[0], precode_time_this_frame_this_thread_per_task[1], 
                precode_time_this_frame_this_thread_per_task[2], precode_time_this_frame_this_thread_per_task[3],
                sum_time_this_frame_this_thread);
#endif
    }
    for (int j = 0; j < task_duration_dim2; j++) {
    	ifft_time_this_frame[j] = ifft_time_this_frame[j] / task_thread_num;
    	csi_time_this_frame[j] = csi_time_this_frame[j] / task_thread_num;
    	zf_time_this_frame[j] = zf_time_this_frame[j] / task_thread_num;
    	precode_time_this_frame[j] = precode_time_this_frame[j] / task_thread_num;
    }
}

void Stats::save_to_file(int last_frame_id, int socket_rx_thread_num)
{
	printf("saving timestamps to file.........\n");
	printf("Total processed frames %d ", last_frame_id);
    FILE* fp_debug = fopen("../matlab/timeresult.txt", "w");
    if (fp_debug==NULL) {
        printf("open file faild");
        std::cerr << "Error: " << strerror(errno) << std::endl;
        exit(0);
    }
#if ENABLE_DOWNLINK
    for(int ii = 0; ii < last_frame_id; ii++) { 
        if (socket_rx_thread_num == 1) {
            fprintf(fp_debug, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], 
                precode_processed[ii], ifft_processed[ii], tx_processed[ii],tx_processed_first[ii],
                csi_time_in_function[ii], zf_time_in_function[ii], precode_time_in_function[ii], ifft_time_in_function[ii], processing_started[ii], frame_start[0][ii]);
        } 
        else {
            fprintf(fp_debug, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], 
                precode_processed[ii], ifft_processed[ii], tx_processed[ii],tx_processed_first[ii],
                csi_time_in_function[ii], zf_time_in_function[ii], precode_time_in_function[ii], ifft_time_in_function[ii], processing_started[ii], frame_start[0][ii], frame_start[1][ii]);
        }
    }
#else
    for(int ii = 0; ii < last_frame_id; ii++) {  
        if (socket_rx_thread_num == 1) {    
                fprintf(fp_debug, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
                        csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii], pilot_all_received[ii]);
        }
        else {                 
                fprintf(fp_debug, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n", pilot_received[ii], rx_processed[ii], fft_processed[ii], zf_processed[ii], demul_processed[ii],
                    csi_time_in_function[ii], fft_time_in_function[ii], zf_time_in_function[ii], demul_time_in_function[ii], processing_started[ii], frame_start[0][ii], frame_start[1][ii], pilot_all_received[ii]);
        }
    }
	#if DEBUG_UPDATE_STATS_DETAILED
        printf("Print results detailed\n");
        FILE* fp_debug_detailed = fopen("../timeresult_detail.txt", "w");
        if (fp_debug_detailed==NULL) {
            printf("open file faild");
            std::cerr << "Error: " << strerror(errno) << std::endl;
            exit(0);
        }

        for(int ii = 0; ii < last_frame_id; ii++) {    
            fprintf(fp_debug_detailed, "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f \n", fft_time_in_function_details[0][ii], fft_time_in_function_details[1][ii],
                fft_time_in_function_details[2][ii], zf_time_in_function_details[0][ii], zf_time_in_function_details[1][ii], zf_time_in_function_details[2][ii],
                demul_time_in_function_details[0][ii], demul_time_in_function_details[1][ii], demul_time_in_function_details[2][ii] );
        }
        fclose(fp_debug_detailed);
	#endif
#endif
    fclose(fp_debug);
}



void Stats::print_summary(int last_frame_id)
{
    int CSI_total_count = 0;
    int FFT_total_count = 0;
    int ZF_total_count = 0;
    int Demul_total_count = 0;
    int IFFT_total_count = 0;
    int Precode_total_count = 0;

    for (int i = 0; i < task_thread_num; i++) {
        CSI_total_count = CSI_total_count + CSI_task_count[i * 16];
        FFT_total_count = FFT_total_count + FFT_task_count[i * 16];
        ZF_total_count = ZF_total_count + ZF_task_count[i * 16];
        Demul_total_count = Demul_total_count + Demul_task_count[i * 16];
        IFFT_total_count = IFFT_total_count + IFFT_task_count[i * 16];
        Precode_total_count = Precode_total_count + Precode_task_count[i * 16];
    }
    
#if ENABLE_DOWNLINK
    double csi_frames = (double)CSI_total_count / BS_ANT_NUM / UE_NUM;
    double precode_frames = (double)Precode_total_count / OFDM_DATA_NUM / dl_data_subframe_num_perframe;
    double ifft_frames = (double)IFFT_total_count / BS_ANT_NUM / dl_data_subframe_num_perframe;
    double zf_frames = (double)ZF_total_count / OFDM_DATA_NUM;
    printf("Downlink: total performed FFT: %d (%.2f frames), ZF: %d (%.2f frames), precode: %d (%.2f frames), IFFT: %d (%.2f frames)\n", 
        	CSI_total_count, csi_frames, ZF_total_count, zf_frames, Precode_total_count, precode_frames, IFFT_total_count, ifft_frames);
    for (int i = 0; i < task_thread_num; i++) {
        double percent_CSI = 100 * double(CSI_task_count[i * 16])/CSI_total_count;
        double percent_ZF = 100 * double(ZF_task_count[i * 16]) / ZF_total_count;
        double percent_Precode = 100 * double(Precode_task_count[i * 16])/Precode_total_count;
        double percent_IFFT = 100 * double(IFFT_task_count[i * 16]) / IFFT_total_count;
        printf("thread %d performed FFT: %d (%.2f%%), ZF: %d (%.2f%%), precode: %d (%.2f%%), IFFT: %d (%.2f%%)\n", 
            	i, CSI_task_count[i * 16], percent_CSI, ZF_task_count[i * 16], percent_ZF, Precode_task_count[i * 16], percent_Precode, 
            	IFFT_task_count[i * 16], percent_IFFT);
    }
#else
    double csi_frames = (double)CSI_total_count / BS_ANT_NUM / UE_NUM;
    double fft_frames = (double)FFT_total_count / BS_ANT_NUM / data_subframe_num_perframe;
    double demul_frames = (double)Demul_total_count / OFDM_DATA_NUM / data_subframe_num_perframe;
    double zf_frames = (double)ZF_total_count / OFDM_DATA_NUM;
    printf("Uplink: total performed CSI %d (%.2f frames), FFT: %d (%.2f frames), ZF: %d (%.2f frames), Demulation: %d (%.2f frames)\n", 
        CSI_total_count, csi_frames, FFT_total_count, fft_frames, ZF_total_count, zf_frames, Demul_total_count, demul_frames);
    for (int i = 0; i < task_thread_num; i++) {
        double percent_CSI = 100 * double(CSI_task_count[i * 16]) / CSI_total_count;
        double percent_FFT = 100 * double(FFT_task_count[i * 16]) / FFT_total_count;
        double percent_ZF = 100 * double(ZF_task_count[i * 16]) / ZF_total_count;
        double percent_Demul = 100 * double(Demul_task_count[i * 16]) / Demul_total_count;
        printf("thread %d performed CSI: %d (%.2f%%), FFT: %d (%.2f%%), ZF: %d (%.2f%%), Demulation: %d (%.2f%%)\n", 
            i, CSI_task_count[i * 16], percent_CSI, FFT_task_count[i * 16], percent_FFT, ZF_task_count[i * 16], percent_ZF, 
            Demul_task_count[i * 16], percent_Demul);
    }
#endif 
} 

















