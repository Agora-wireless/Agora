/**
 * Author: Jian Ding
 * Email: jianding17@gmail.com
 * 
 */
#include "reciprocity.hpp"

using namespace arma;
Reciprocity::Reciprocity(Config *cfg, int in_tid,
    moodycamel::ConcurrentQueue<Event_data> *in_complete_task_queue, moodycamel::ProducerToken *in_task_ptok,
    complex_float **in_calib_buffer, complex_float **in_recip_buffer,
    double **in_recip_task_duration, int *in_recip_task_count) 
{
    config_ = cfg;
    BS_ANT_NUM = cfg->BS_ANT_NUM;
    UE_NUM = cfg->UE_NUM;
    OFDM_CA_NUM = cfg->OFDM_CA_NUM;
    OFDM_DATA_NUM = cfg->OFDM_DATA_NUM;
    ref_ant = cfg->ref_ant;

    tid = in_tid;
    complete_task_queue_ = in_complete_task_queue;
    task_ptok = in_task_ptok;

    calib_buffer_ = in_calib_buffer;
    recip_buffer_ = in_recip_buffer;

    RC_task_duration = in_recip_task_duration;
    RC_task_count = in_recip_task_count;

    calib_gather_buffer = (complex_float *)aligned_alloc(64, BS_ANT_NUM * OFDM_DATA_NUM * sizeof(complex_float));

}


Reciprocity::~Reciprocity()
{
    free(calib_gather_buffer);   
}


void Reciprocity::computeReciprocityCalib(int offset)
{
    int frame_id = offset;

#if DEBUG_PRINT_IN_TASK
        printf("In doRecip thread %d: frame: %d, subcarrier: %d\n", tid, frame_id, sc_id);
#endif

#if DEBUG_UPDATE_STATS
        double start_time1 = get_time();
#endif

    cx_float *ptr_in = (cx_float *)calib_buffer_[offset];
    cx_fmat mat_input(ptr_in, OFDM_DATA_NUM, BS_ANT_NUM, false);
    cx_fvec vec_calib_ref = mat_input.col(ref_ant);
    cx_float *ptr_out = (cx_float *)calib_gather_buffer;
    cx_fmat mat_output(ptr_out, BS_ANT_NUM, OFDM_DATA_NUM, false);

    for (int ant_id = 0; ant_id < BS_ANT_NUM; ant_id++) {
        cx_fvec vec_calib = mat_input.col(ant_id);
        cx_fvec recipFactor = vec_calib_ref / vec_calib;
        mat_output.row(ant_id) = recipFactor;
        for (int sc_id = ant_id; sc_id < OFDM_DATA_NUM; sc_id += BS_ANT_NUM) {
            for (int i = 0; i < BS_ANT_NUM; i++) {
                recip_buffer_[sc_id+i][ant_id].re = mat_output.at(ant_id, sc_id).real();
                recip_buffer_[sc_id+i][ant_id].im = mat_output.at(ant_id, sc_id).imag();
	    }
        }
    }
   
    #if DEBUG_UPDATE_STATS_DETAILED
        double duration1 = get_time() - start_time1;
        RC_task_duration[tid * 8][1] += duration1;
        // double start_time2 = get_time();
    #endif

    #if DEBUG_UPDATE_STATS_DETAILED
        double start_time2 = get_time();
        double duration2 = start_time2 - start_time1;
        RC_task_duration[tid * 8][2] += duration2;
        // double start_time2 = get_time();
    #endif

        // cout<<"Precoder:" <<mat_output<<endl;
    #if DEBUG_UPDATE_STATS_DETAILED    
        double duration3 = get_time() - start_time2;
        RC_task_duration[tid * 8][3] += duration3;
        // double start_time3 = get_time();
    #endif

    #if DEBUG_UPDATE_STATS   
        RC_task_count[tid * 16] = RC_task_count[tid * 16]+1; 
        double duration = get_time() - start_time1;
        RC_task_duration[tid * 8][0] += duration;
        if (duration > 500) {
            printf("Thread %d RC takes %.2f\n", tid, duration);
        }
    #endif


    // inform main thread
    Event_data RC_finish_event;
    RC_finish_event.event_type = EVENT_RC;
    RC_finish_event.data = offset;
    

    if ( !complete_task_queue_->enqueue(*task_ptok, RC_finish_event ) ) {
        printf("RC message enqueue failed\n");
        exit(0);
    }


}
