#include "round_robbin.h"

#include "logger.h"

RoundRobbin::RoundRobbin(const size_t spatial_streams,
                        const size_t bss_num,
                        const size_t ues_num,
                        size_t ofdm_data_num_)     
        : selected_group_(0),
        spatial_streams_(spatial_streams),
        bss_num_(bss_num),
        ues_num_(ues_num) {
    
    num_groups_ =
        (spatial_streams_ == ues_num_) ? 1 : ues_num_;
    schedule_buffer_.Calloc(num_groups_, ues_num * ofdm_data_num_,
                            Agora_memory::Alignment_t::kAlign64);
    schedule_buffer_index_.Calloc(num_groups_,
                                spatial_streams_ * ofdm_data_num_,
                                Agora_memory::Alignment_t::kAlign64);
    
    for (size_t gp = 0u; gp < num_groups_; gp++) {
    for (size_t sc = 0; sc < ofdm_data_num_; sc++) {
        for (size_t ue = gp; ue < gp + spatial_streams_; ue++) {
        size_t cur_ue = ue % ues_num_;
        // for now all SCs are allocated to scheduled UEs
        schedule_buffer_[gp][cur_ue + ues_num_ * sc] = 1;
        schedule_buffer_index_[gp][(ue - gp) + spatial_streams_ * sc] =
            cur_ue;
        }
    }
    }

}