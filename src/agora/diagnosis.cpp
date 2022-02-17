#include "diagnosis.hpp"

Diagnosis::Diagnosis(Config* cfg, SharedState* state,
    std::vector<BottleneckSubcarrier>& subcarrier_bottleneck,
    std::vector<BottleneckDecode>& decode_bottleneck) {
    size_t base_frame = state->cur_frame_;
    if (cfg->downlink_mode) {
        printf("Sorry! The diagnosis is not supported in downlink mode.\n");
    } else {
        printf("\n********************Diagnosis start********************\n");
        printf("Checking packet receiving:\n");

        size_t cur_frame;
        size_t symbol_id_ul;
        size_t ue_id;
        for (cur_frame = base_frame; cur_frame < base_frame + kFrameWnd; cur_frame ++) {
            for (symbol_id_ul = 0; symbol_id_ul < cfg->ul_data_symbol_num_perframe; symbol_id_ul ++) {
                for (ue_id = cfg->ue_start; ue_id < cfg->ue_end; ue_id ++) {
                    if (!state->received_all_demod_pkts(ue_id, cur_frame, symbol_id_ul)) {
                        goto demod_pkt_check_done;            
                    }
                }
            }
        }
    demod_pkt_check_done:
        size_t demod_last_frame = cur_frame;
        printf("[Demod packet recv] Last frame: %zu. Symbol %zu, ue %zu, received %zu, required %zu\n", 
            demod_last_frame, symbol_id_ul, ue_id, state->num_demod_pkts_[ue_id][cur_frame%kFrameWnd][symbol_id_ul].load(),
            state->num_demod_pkts_per_symbol_per_ue_);
        
        cur_frame = base_frame;
        for (cur_frame = base_frame; cur_frame < base_frame + kFrameWnd; cur_frame ++) {
            for (symbol_id_ul = 0; symbol_id_ul < cfg->ul_data_symbol_num_perframe; symbol_id_ul ++) {
                if (!state->received_all_data_pkts(cur_frame, symbol_id_ul)) {
                    goto data_pkt_check_done;            
                }
            }
        }
    data_pkt_check_done:
        size_t data_last_frame = cur_frame;
        printf("[Data packet recv] Last frame: %zu. Symbol %zu, received %zu, required %zu\n", 
            data_last_frame, symbol_id_ul, state->num_data_pkts_[cur_frame%kFrameWnd][symbol_id_ul].load(),
            state->num_pkts_per_symbol_);
        
        cur_frame = base_frame;
        for (cur_frame = base_frame; cur_frame < base_frame + kFrameWnd; cur_frame ++) {
            if (!state->received_all_pilots(cur_frame)) {
                goto pilot_check_done;            
            }
        }
    pilot_check_done:
        size_t pilot_last_frame = cur_frame;
        printf("[Pilot recv] Last frame: %zu. Received %zu, required %zu\n", 
            pilot_last_frame, state->num_pilot_pkts_[cur_frame%kFrameWnd].load(), state->num_pilot_pkts_per_frame_);

        if (base_frame + kFrameWnd - demod_last_frame < 5) {
            printf("Packet receiving is not a problem. Decoding might be the bottleneck. See details further\n");
        } else if (base_frame + kFrameWnd - data_last_frame < 5 && base_frame + kFrameWnd - pilot_last_frame < 5) {
            if (demod_last_frame - base_frame < 2) {
                printf("Did not receive all demod packets. It could be packet loss or failures/bottlenecks from some Hydra server. See details further\n");
            } else {
                printf("It could be a combination of problems. First, it did not receive all demod packets. Second, decoding might be the bottleneck. See details further\n");
            }
        } else if (base_frame + kFrameWnd - pilot_last_frame < 5) {
            if (data_last_frame - base_frame < 2) {
                printf("Packet loss for data packets. See details further\n");
            } else {
                printf("I really don't know what happened. See details further\n");
            }
        } else {
            if (pilot_last_frame - base_frame < 2) {
                printf("Packet loss for pilot packets. See details further\n");
            } else {
                printf("I really don't know what happened. See details further\n");
            }
        }

        if (base_frame <= 200) {
            printf("Hydra did not cross the slow start duration. Bottleneck analysis did not start.");
        } else {
            printf("\nChecking bottleneck:\n");
            for (size_t i = 0; i < subcarrier_bottleneck.size(); i ++) {
                if (subcarrier_bottleneck[i].idle < 1) {
                    printf("Subcarrier %zu is bottlenecked.\n", i);
                    printf("Subcarrier %zu bottleneck data: (csi %lf, zf %lf, demul %lf)\n", i, 
                        subcarrier_bottleneck[i].csi, subcarrier_bottleneck[i].zf, 
                        subcarrier_bottleneck[i].demul);
                    return;
                }
            }
            for (size_t i = 0; i < decode_bottleneck.size(); i ++) {
                if (decode_bottleneck[i].idle < 1) {
                    printf("Decode %zu is bottlenecked.\n", i);
                    printf("Decode %zu bottleneck data: (decode %lf)\n", i, 
                        decode_bottleneck[i].decode);
                    return;
                }
            }
            printf("No bottleneck found.\n");
        }
    }
}

Diagnosis::~Diagnosis() {}