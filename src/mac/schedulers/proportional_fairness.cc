/**
 * @file proportional_fairness.cc
 * @brief Implementation file for the Proportional Fairness scheduling algorithm 
 */
#include "proportional_fairness.h"
#include "logger.h"

static constexpr bool kPrintSchedulingBuffers = false;

void ProportionalFairness::Combination( int k, int offset = 0) {
  if (k == 0) {
    combination_vector.push_back( combination );
    return;
  }
  for (size_t i = offset; i <= ues_vector.size() - k; ++i) {
    combination.push_back(ues_vector[i]);
    Combination(k-1, i+1);
    combination.pop_back();
  }
}

ProportionalFairness::ProportionalFairness( const size_t spatial_streams, const size_t bss_num, const size_t ues_num , size_t ofdm_data_num_ ) : 
    spatial_streams_(spatial_streams),
    bss_num_(bss_num),
    ues_num_(ues_num),
    lamda_(0.5F),
    last_SE_(arma::zeros(ues_num))
{

    std::stringstream ss;
    ss << "\n\n ============== Proportional Fairness Algorithm ============== \n\n";

    selected_action_ = 0;

    //Define UE Index vector
    for( size_t ue_idx = 0; ue_idx < ues_num_; ue_idx++){
        ues_vector.push_back(ue_idx);
        ues_flags_.push_back(false);
        pf_ues_history.push_back(0.01F);
    }

    //Possible actions
    Combination(spatial_streams_);
    actions_num_ = combination_vector.size();
    
    schedule_buffer_.Calloc(actions_num_, ues_num_ * ofdm_data_num_,
                            Agora_memory::Alignment_t::kAlign64);
    schedule_buffer_index_.Calloc(actions_num_, spatial_streams_ * ofdm_data_num_,
                            Agora_memory::Alignment_t::kAlign64);

    //Schedule Buffer Process
    for( size_t gp = 0; gp < actions_num_; gp++ )
    {
        for (size_t sc = 0; sc < ofdm_data_num_; sc++) 
        {
            std::vector<size_t> UEs_idx = combination_vector[gp];

            for( size_t ue_idx = 0; ue_idx < spatial_streams_; ue_idx++ )
            {
                schedule_buffer_[gp][UEs_idx[ue_idx] + ues_num_ * sc] = 1;
            }

            for( size_t ue = gp; ue < gp + spatial_streams_; ue++ )
            {
                schedule_buffer_index_[gp][(ue - gp) + spatial_streams_ * sc] = UEs_idx[ue - gp];
            }
            
        }

    }
    
    if( kPrintSchedulingBuffers )
    {

        for (size_t row = 0; row < actions_num_; row++) 
        {

            ss << "schedule_PF_index_buffer_" << row << " \n\n";

            for (size_t col = 0; col < spatial_streams_ * ofdm_data_num_; col++) {
                ss << schedule_buffer_index_[row][col] << " ";
            }

            ss << "\n\n";

            ss << "schedule_PF_SC_buffer_" << row << " \n\n";

            for (size_t col = 0; col < spatial_streams_ * ofdm_data_num_; col++) {
                ss << schedule_buffer_[row][col] << " ";
            }
            ss <<  "\n\n";

        }

    }

    ss << "PF Scheduling Groups \n";

    for( size_t gp = 0; gp < actions_num_; gp++)
    {
        
        ss << "Group " << gp << " = [ ";
        
        for( size_t ue = 0; ue < ues_num_; ue++ )
        {
            ss << schedule_buffer_[gp][ue] << " ";
        }    

        ss << "]\n\n";

    }

    ss << "============== Proportional Fairness Algorithm FINISHED ============== \n\n";

    AGORA_LOG_INFO( ss.str() );

}

size_t ProportionalFairness::UpdateScheduler( size_t frame, std::vector<float> ues_capacity )
{

    std::vector<float> ues_capacity = { 0.68226f,0.678098f,0.671843f,0.68403f };

    if( current_frame != frame )
    {

        current_frame = frame;
        Schedule( frame , ues_capacity);
        UpdatePF( frame , ues_capacity);
        
        std::vector<size_t> option = combination_vector[selected_action_];

        std::stringstream s_;
        s_ << "Scheduled PF Frame " << frame  << " [" << selected_action_ << "] = [ ";

        for( size_t i = 0; i < ues_num_; i++)
        {
            std::string a = ( ues_flags_[i] ) ? "1 " : "0 ";
            s_ << a;
        }
        
        s_ << "] \n ";

        AGORA_LOG_INFO( s_.str() );
        std::cout << s_.str() << "] \n ";

    }
    
    selected_action_ = ( frame % 2 == 0 )? 1: 4;
    return selected_action_;

}

void ProportionalFairness::Schedule( size_t frame , std::vector<float> ues_capacity )
{
    arma::vec pf_;
    pf_.zeros(actions_num_);

    arma::fmat ues_capacity_;
    ues_capacity_.zeros( actions_num_, spatial_streams_ ); 

    arma::vec selected_ues_idx;

    //From all actions, select the maximum capacity 
    float max_pf = 0;
    for( size_t action = 0; action < actions_num_; action++ )
    {
        
        std::vector<size_t> selected_ues = combination_vector[action];

        if( frame > 0 )
        {
        
            for( size_t i = 0; i < selected_ues.size(); i++)
            {

                float tp_history = 0;
                size_t ue_idx = selected_ues[i];

                if( ues_flags_[ ue_idx ] )
                {

                    tp_history = lamda_*pf_ues_history[ue_idx]/frame + (1-lamda_)*last_SE_[ue_idx];

                }
                else
                {
                    
                    tp_history = lamda_*pf_ues_history[ue_idx]/frame;

                }

                pf_[action] += ues_capacity[ue_idx] / tp_history;

                if( pf_[action] >= max_pf )
                {
                    max_pf = pf_[action];
                    selected_action_ = action;
                }

            }
        
        } 
    }
}

void ProportionalFairness::UpdatePF( size_t frame, std::vector<float> ues_capacity )
{
    
    for( size_t i = 0; i < combination_vector[selected_action_].size(); i++)
    {
        size_t idx_ = combination_vector[selected_action_][i];
        pf_ues_history[idx_] += ues_capacity[idx_];
    }

    for( size_t ue = 0; ue < ues_num_; ue++)
    {
        if(std::find(combination_vector[selected_action_].begin(), combination_vector[selected_action_].end(), ue) != combination_vector[selected_action_].end()) {
            ues_flags_[ue] = true;
            last_SE_[ue] = ues_capacity[ue];
        } else {
            ues_flags_[ue] = false;
            last_SE_[ue] = 0.0F;
        }
    }
}