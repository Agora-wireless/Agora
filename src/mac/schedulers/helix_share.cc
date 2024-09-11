#include "helix_share.h"
#include "logger.h"
#include "H5Cpp.h"


RB_Share::RB_Share(Config* const cfg) // Initialization
    : SchedulerModel(cfg),
      total_slice_tp(arma::zeros(Num_slice)),
      est_slice_tp(arma::zeros(Num_slice))
    {

    avg_rb = 0.0;
    total_rb_allocated = 0.0;

    //Possible scheduling options
    num_groups_ = 5; // # of slots information we want to save
    schedule_buffer_.Calloc(num_groups_, cfg_->UeAntNum() * num_prbs_,
                            Agora_memory::Alignment_t::kAlign64);
    schedule_buffer_index_.Calloc(num_groups_,
                                cfg_->SpatialStreamsNum() * num_prbs_,
                                Agora_memory::Alignment_t::kAlign64);
    ul_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
    dl_mcs_buffer_.Calloc(num_groups_, cfg_->UeAntNum(),
                        Agora_memory::Alignment_t::kAlign64);
    selected_group_vec_.resize(num_prbs_, 0);

    //Schedule Buffer Process
    for (size_t gp = 0; gp < num_groups_; gp++) {
    // for (size_t prb = 0; prb < num_prbs_; prb++) {
    // //   std::vector<size_t> u_es_idx = groups_vector_[gp];
    //   for (size_t ue_idx = 0; ue_idx < cfg_->UeAntNum(); ue_idx++) {
    //     schedule_buffer_[gp][ue_idx + cfg_->UeAntNum() * prb] = 0;
    //     schedule_buffer_index_[gp][ue_idx + cfg_->SpatialStreamsNum() * prb] =
    //         u_es_idx[ue_idx];
    //   }
    // }
        for (size_t ue = 0; ue < cfg_->UeAntNum(); ue++) {
            ul_mcs_buffer_[gp][ue] = cfg->MacParams().McsIndex(Direction::kUplink);
            dl_mcs_buffer_[gp][ue] = cfg->MacParams().McsIndex(Direction::kDownlink);
            AGORA_LOG_TRACE("UL MCS Init: gp %zu, ue %zu, mcs %zu\n", gp, ue,
                            ul_mcs_buffer_[gp][ue]);
        }
    }

    if (kPrintSchedulingBuffers) {
    std::stringstream ss;
    for (size_t row = 0; row < num_groups_; row++) {
        ss << "schedule_PF_index_buffer_" << row << " \n\n";
        for (size_t prb = 0; prb < num_prbs_; prb++) {
        for (size_t col = 0; col < cfg_->SpatialStreamsNum(); col++) {
            ss << schedule_buffer_index_[row]
                                        [col + prb * cfg_->SpatialStreamsNum()]
                << " ";
        }
        ss << "\n";
        }
        ss << "\n\n";
        ss << "schedule_PF_SC_buffer_" << row << " \n\n";
        for (size_t prb = 0; prb < num_prbs_; prb++) {
        for (size_t col = 0; col < cfg_->UeAntNum(); col++) {
            ss << schedule_buffer_[row][col + prb * cfg_->UeAntNum()] << " ";
        }
        ss << "\n";
        }
        ss << "\n\n";
    }
    AGORA_LOG_INFO(ss.str());
    }

    //************************************************** Load Channel Gain and USER Groups *******************************************************************
    H5File file_2("Path to agora_cg_ug.hdf5", H5F_ACC_RDONLY);

    // Try to open the dataset
    DataSet dataset = file_2.openDataSet("var_length_lists");
    DataType datatype = dataset.getDataType();

    // Get dataspace of the dataset.
    DataSpace dataspace = dataset.getSpace();

    // Get the dimension size of each dimension in the dataspace and display them.
    hsize_t num_elements;
    dataspace.getSimpleExtentDims(&num_elements);


    std::vector<hvl_t> read_data(num_elements); // For variable-length data
    dataset.read(read_data.data(), datatype); // Read directly into hvl_t array

    // Store group in a new vector
    std::vector<std::vector<int>> group_vector;

    for (const auto& elem : read_data) {
        // Cast the void pointer to an int pointer
        int* int_array = static_cast<int*>(elem.p);

        // Create a vector from the int array
        std::vector<int> temp(int_array, int_array + elem.len);  // Using the vector range constructor

        // Add the newly created vector to the destination vector
        group_vector.push_back(temp);
    }

    for (size_t i = 0; i < num_elements; ++i) {
        free(read_data[i].p);
    }
    
    DataSet dataset_len = file_2.openDataSet("len");
    DataSet dataset_cg = file_2.openDataSet("cg");

    // Get the dataspace of the datasets
    DataSpace dataspace_len = dataset_len.getSpace();
    DataSpace dataspace_cg = dataset_cg.getSpace();

    // Get the dimensions of the datasets
    hsize_t dims_len[1];
    hsize_t dims_cg[3];
    dataspace_len.getSimpleExtentDims(dims_len);
    dataspace_cg.getSimpleExtentDims(dims_cg);

    // std::cout << "(" << num_elements << ", " << dims_len[0] << ", " << dims_cg[0] << ", "<< dims_cg[1] << ", " << dims_cg[2] << ")" << std::endl;

    arma::vec group_len(dims_len[0], arma::fill::zeros);
    arma::vec cg_tti_rb_ue(dims_cg[0] * dims_cg[1] * dims_cg[2], arma::fill::zeros);;

    
    // Read data from the datasets
    dataset_len.read(group_len.memptr(), PredType::NATIVE_DOUBLE);
    dataset_cg.read(cg_tti_rb_ue.memptr(), PredType::NATIVE_DOUBLE);

    

    // Close the datasets and file
    // dataset.close();
    dataset_len.close();
    dataset_cg.close();
    file_2.close();

    std::cout << "READING DATA GOOD FOR NOW" << std::endl;

}
    
void RB_Share::do_alloc(const std::vector<arma::cx_fmat>& csi, size_t frame, const std::vector<float>& last_throughput){
// csi shape: 1-D arma::cx_vec (Num_UE * Num_BS * Num_RBG); 
// total_slice_tp: accumulated data rate of slices
// avg_rb only for Multi-RB to compute parallel
// total_rb_allocated: count total # of allocated RBs over TTIs

    for (size_t ue = 0; ue < cfg->UeAntNum(); ue++){
        if (ue < 9)
        {
            total_slice_tp[0] += last_throughput[i];
        }
        else if (ue < 21)
        {
            total_slice_tp[1] += last_throughput[i];
        }
        else if (ue < 39)
        {
            total_slice_tp[2] += last_throughput[i];
        }
        else if (ue < 59)
        {
            total_slice_tp[3] += last_throughput[i];
        }
        else if (ue < 84)
        {
            total_slice_tp[4] += last_throughput[i];
        }
        else if (ue < 117)
        {
            total_slice_tp[5] += last_throughput[i];
        }
        else if (ue < 162)
        {
            total_slice_tp[6] += last_throughput[i];
        }
        else
        {
            total_slice_tp[7] += last_throughput[i];
        }
    }
    
    // auto start = std::chrono::high_resolution_clock::now();
    std::vector<double> delta_slice(Num_slice);
    for (size_t sl = 0; sl < Num_slice; sl++)
    {
        delta_slice[sl] = SLAs[sl] * (frame+1) - total_slice_tp[sl];
    }

    // last_slice_tp = total_slice_tp;
    est_slice_tp = total_slice_tp;
    std::vector<int> RBG_remain;
    for (int num = 0; num < num_prbs_; ++num) {
        RBG_remain.push_back(num);
    }

    arma::mat cg_rb_ue(num_prbs_, cfg->UeAntNum(), arma::fill::zeros);
    size_t est_frame = frame / 10;
    for (size_t i = 0; i < num_prbs_; i++)
    {
        for (size_t j = 0; j < cfg->UeAntNum(); j++)
        {
            cg_rb_ue(i,j) = cg_tti_rb_ue[est_frame*num_prbs_*cfg->UeAntNum() + i*cfg->UeAntNum() + j];
        }
        
    }

    // auto cg_time = std::chrono::high_resolution_clock::now();

    double rb_alloc_time = 0;
    while (any_positive(delta_slice) && !RBG_remain.empty()) {
        
        // auto loop_time = std::chrono::high_resolution_clock::now();
        std::vector<int> UE_list;
        for (int num = 0; num < cfg->UeAntNum(); ++num) {
            UE_list.push_back(num);
        }

        std::vector<double> non_zero;
        double mean_delta = 0;
        
        int parallel = 1;
        // Find Non-zero Elements in Delta_slice
        for (double x : delta_slice) {
            if (x > 0) {
                non_zero.push_back(x);
            }
        }
        // Define Mean_delta to classify Large Group and Small Group
        if (!non_zero.empty()) {
            mean_delta = std::accumulate(non_zero.begin(), non_zero.end(), 0.0) / non_zero.size();
        } else {
            // Calculate mean of delta_slice if non_zero is empty
            if (!delta_slice.empty()) {
                mean_delta = std::accumulate(delta_slice.begin(), delta_slice.end(), 0.0) / delta_slice.size();
            }
        }
        // Define Parallel
        if (new_rb_para)
        {
            if (frame>1)
            {
                double max_non_zero = *std::max_element(non_zero.begin(), non_zero.end());
                parallel = static_cast<int>((max_non_zero * non_zero.size() / avg_rb)) + 1;

            }else{
                parallel = 1;
            }
            
        }
        // std::cout << "Current Parallel:" << parallel << std::endl;
        std::vector<int> large_ue_list;
        std::vector<int> small_ue_list;
        std::vector<int> prefix_sum(Num_UE_ps.size() + 1, 0);
        std::partial_sum(Num_UE_ps.begin(), Num_UE_ps.end(), prefix_sum.begin() + 1);

        for (int sl = 0; sl < Num_slice; ++sl) {
            if (delta_slice[sl] >= mean_delta) {
                large_ue_list.insert(large_ue_list.end(), 
                                    UE_list.begin() + prefix_sum[sl], 
                                    UE_list.begin() + prefix_sum[sl + 1]);
            } else if (delta_slice[sl] < mean_delta && delta_slice[sl] > 0) {
                small_ue_list.insert(small_ue_list.end(), 
                                    UE_list.begin() + prefix_sum[sl], 
                                    UE_list.begin() + prefix_sum[sl + 1]);
            }
        }

        arma::mat cg_large(RBG_remain.size(), large_ue_list.size(), arma::fill::zeros);

        for (size_t i = 0; i < RBG_remain.size(); i++)
        {
            for (size_t j = 0; j < large_ue_list.size(); j++)
            {
                cg_large(i,j) = cg_rb_ue(i,large_ue_list[j]);
            }
        }
        
        if (parallel == 1)
        {
            std::cout << "One RB" << std::endl;
            double max_value = 0;
            std::pair<int, int> max_index(-1, -1);
            // Search for the maximum value
            for (size_t i = 0; i < cg_large.n_rows; i++) {
                for (size_t j = 0; j < cg_large.n_cols; j++) {
                    if (cg_large(i,j) > max_value) {
                        max_value = cg_large(i,j);
                        max_index = {i, j};  // Store the new indices
                    }
                }
            }
            int rbg_index = max_index.first;
            int sel_rbg = RBG_remain[rbg_index];
            int ue_index = large_ue_list[max_index.second];

            arma::cx_fmat H_t(cfg->UeAntNum(), cfg->BsAntNum());


            for (size_t ue_idx = 0; ue_idx < cfg->UeAntNum(); ue_idx++) {
                arma::cx_fmat csi_mat = csi.at(ue_idx);  // CSI matrix for the UE

                // Extract the column corresponding to the target PRB
                arma::cx_fvec prb_column = csi_mat.col(sel_rbg);

                // Place the PRB CSI (column) into the appropriate row in the prb_csi matrix
                H_t.row(ue_idx) = prb_column.t();  // Transpose to make it a row
            }

            arma::vec cg_ue_vec(cfg->UeAntNum(), arma::fill::zeros);
            for (size_t i = 0; i < cfg->UeAntNum(); i++)
            {
                cg_ue_vec[i] = cg_rb_ue(rbg_index,i);
            }

            int sum_lower; 
            if (est_frame==0 && sel_rbg==0)
            {
                sum_lower = 0;
            }else{
                sum_lower = arma::sum(group_len.subvec(0, est_frame*num_prbs_+sel_rbg-1));
            }
            
            int sum_upper = arma::sum(group_len.subvec(0, est_frame*num_prbs_+sel_rbg));

            std::vector<std::vector<int>> group_target(group_vector.begin()+ sum_lower, group_vector.begin()+ sum_upper);
            
            // std::vector<std::vector<int>> group_target = group_vector;
            
            auto[sel_UE_list, cap_per_ue] = alloc_rb(group_target, cfg_->SpatialStreamsNum(), cg_ue_vec, H_t, ue_index, large_ue_list, small_ue_list);

            // sel_rb.push_back(sel_rbg);
            // sel_UE_list_rb.push_back(sel_UE_list);

            // *******************  Allocation Results Recording  **********************
            size_t frame_next = frame +1;
            size_t gp = frame_next % num_groups_;

            for (size_t i = 0; i < sel_UE_list.size(); i++) {
                schedule_buffer_index_[gp][i + cfg_->SpatialStreamsNum() * sel_rbg] = sel_UE_list[i];
                schedule_buffer_[gp][sel_UE_list[i] + cfg_->UeAntNum() * sel_rbg] = 1;
            }
            // *******************  Allocation Results Recording  **********************


            for (size_t i = 0; i < sel_UE_list.size(); i++) {
                int ue = sel_UE_list[i];
                if (ue < 9)
                {
                    est_slice_tp[0] += cap_per_ue[i];
                }
                else if (ue < 21)
                {
                    est_slice_tp[1] += cap_per_ue[i];
                }
                else if (ue < 39)
                {
                    est_slice_tp[2] += cap_per_ue[i];
                }
                else if (ue < 59)
                {
                    est_slice_tp[3] += cap_per_ue[i];
                }
                else if (ue < 84)
                {
                    est_slice_tp[4] += cap_per_ue[i];
                }
                else if (ue < 117)
                {
                    est_slice_tp[5] += cap_per_ue[i];
                }
                else if (ue < 162)
                {
                    est_slice_tp[6] += cap_per_ue[i];
                }
                else
                {
                    est_slice_tp[7] += cap_per_ue[i];
                }
            }

            cg_rb_ue.shed_row(rbg_index);
            RBG_remain.erase(RBG_remain.begin() + rbg_index);
            avg_rb = std::accumulate(cap_per_ue.begin(), cap_per_ue.end(), 0.0) / parallel;
            // auto end_in = std::chrono::high_resolution_clock::now();
            // auto loop_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_in - loop_time);
            // rb_alloc_time += loop_duration.count();

        }else{
            std::cout << "Multiple RBs and parallel:"<< parallel << std::endl;
            // std::vector<int> large_ue_list;
            // std::vector<int> small_ue_list;
            double total_rate = 0;
            // auto iter_time = std::chrono::high_resolution_clock::now();
            for (size_t count= 0; count < parallel; count++)
            {
                
                double max_value = 0;
                std::pair<int, int> max_index(-1, -1);
                // std::cout << "Now Parallel:" << count << std::endl;
                // Search for the maximum value
                for (size_t i = 0; i < cg_large.n_rows; i++) {
                    for (size_t j = 0; j < cg_large.n_cols; j++) {
                        if (cg_large(i,j) > max_value) {
                            max_value = cg_large(i,j);
                            max_index = {i, j};  // Store the new indices
                        }
                    }
                }
                
                int rbg_index = max_index.first;
                int sel_rbg = RBG_remain[rbg_index];
                int ue_index = large_ue_list[max_index.second];
                
                // std::cout << "RBG:"<< sel_rbg << std::endl;

                std::vector<int> large_ue = large_ue_list;
                std::vector<int> small_ue = small_ue_list;
                

                arma::cx_mat H_t(cfg->UeAntNum(), cfg->BsAntNum(), arma::fill::zeros);
                for (size_t ue_idx = 0; ue_idx < cfg->UeAntNum(); ue_idx++) {
                    arma::cx_fmat csi_mat = csi.at(ue_idx);  // CSI matrix for the UE

                    // Extract the column corresponding to the target PRB
                    arma::cx_fvec prb_column = csi_mat.col(sel_rbg);

                    // Place the PRB CSI (column) into the appropriate row in the prb_csi matrix
                    H_t.row(ue_idx) = prb_column.t();  // Transpose to make it a row
                }

                arma::vec cg_ue_vec(cfg->UeAntNum(), arma::fill::zeros);

                for (size_t i = 0; i < cfg->UeAntNum(); i++)
                {
                    cg_ue_vec[i] = cg_rb_ue(rbg_index,i);
                    // cg_ue_vec[i] = 1;
                }

                int sum_lower; 
                if (est_frame==0 && sel_rbg==0)
                {
                    sum_lower = 0;
                }else{
                    sum_lower = arma::sum(group_len.subvec(0, est_frame*num_prbs_+sel_rbg-1));
                }
                
                int sum_upper = arma::sum(group_len.subvec(0, est_frame*num_prbs_+sel_rbg));

                std::vector<std::vector<int>> group_target(group_vector.begin()+ sum_lower, group_vector.begin()+ sum_upper);

                // std::vector<std::vector<int>> group_target = group_vector;

                // std::cout << "Before alloc_rb Good:" << count << std::endl;
                auto[sel_UE_list, cap_per_ue] = alloc_rb(group_target, cfg_->SpatialStreamsNum(), cg_ue_vec, H_t, ue_index, large_ue, small_ue);

                // *******************  Allocation Results Recording  **********************

                size_t frame_next = frame +1;
                size_t gp = frame_next % num_groups_;

                for (size_t i = 0; i < sel_UE_list.size(); i++) {
                    schedule_buffer_index_[gp][i + cfg_->SpatialStreamsNum() * sel_rbg] = sel_UE_list[i];
                    schedule_buffer_[gp][sel_UE_list[i] + cfg_->UeAntNum() * sel_rbg] = 1;
                }

                // *******************  Allocation Results Recording  **********************
                
                for (size_t i = 0; i < sel_UE_list.size(); i++) {
                    int ue = sel_UE_list[i];
                    if (ue < 9)
                    {
                        est_slice_tp[0] += cap_per_ue[i];
                    }
                    else if (ue < 21)
                    {
                        est_slice_tp[1] += cap_per_ue[i];
                    }
                    else if (ue < 39)
                    {
                        est_slice_tp[2] += cap_per_ue[i];
                    }
                    else if (ue < 59)
                    {
                        est_slice_tp[3] += cap_per_ue[i];
                    }
                    else if (ue < 84)
                    {
                        est_slice_tp[4] += cap_per_ue[i];
                    }
                    else if (ue < 117)
                    {
                        est_slice_tp[5] += cap_per_ue[i];
                    }
                    else if (ue < 162)
                    {
                        est_slice_tp[6] += cap_per_ue[i];
                    }
                    else
                    {
                        est_slice_tp[7] += cap_per_ue[i];
                    }
                }

                cg_rb_ue.shed_row(rbg_index);
                // std::cout << "After cg_rb Good:" << count << std::endl;
                cg_large.shed_row(rbg_index);
                // std::cout << "After cg_large Good:" << count << std::endl;
                RBG_remain.erase(RBG_remain.begin() + rbg_index);
                total_rate += std::accumulate(cap_per_ue.begin(), cap_per_ue.end(), 0.0);
                // std::cout << "After RB_remain Good:" << count << std::endl;
            
            }

            avg_rb = total_rate / parallel;
            // auto iter_over = std::chrono::high_resolution_clock::now();
            // auto duration_1 = std::chrono::duration_cast<std::chrono::microseconds>(iter_time - loop_time);
            // auto duration_2 = std::chrono::duration_cast<std::chrono::microseconds>(iter_over - iter_time);
            // // std::cout << "rb_alloc_time:" << rb_alloc_time << std::endl;
            // rb_alloc_time += duration_1.count() + duration_2.count()/parallel;
            // // std::cout << "Iter_pre and iter_over:" << rb_alloc_time << "=" << duration_1.count() << "," << duration_2.count()/parallel << std::endl;

        }

        for (size_t sl = 0; sl < Num_slice; sl++)
        {
            delta_slice[sl] = SLAs[sl] * (frame+1) - est_slice_tp[sl];
            // std::cout << "Delta of slice" << sl << ":" << delta_slice[sl] << std::endl;
        }
    }
    // auto rb_finish = std::chrono::high_resolution_clock::now();

// ************************* For Multi-RB *************************************
    std::vector<double> slice_tp (Num_slice);

    for (size_t i = 0; i < Num_slice; i++)
    {
        slice_tp[i] = est_slice_tp[i] - total_slice_tp[i];
    }
    double tp_tti = std::accumulate(slice_tp.begin(), slice_tp.end(), 0.0);
    double current_alloc_rb = num_prbs_ - RBG_remain.size();
    avg_rb = tp_tti / current_alloc_rb; // Update avg_rb to compute parallel next TTI
// ************************* For Multi-RB *************************************
    total_rb_allocated += current_alloc_rb; // Update # of allocated RBs

    // auto end = std::chrono::high_resolution_clock::now();
    // if (frame>1)
    // {
    //     auto mid_time = std::chrono::duration_cast<std::chrono::microseconds>(end - rb_finish  + cg_time - start);
    //     double tti_time = mid_time.count() + rb_alloc_time;
    //     std::cout << "One TTI time: " << tti_time << "," << mid_time.count() << "," << rb_alloc_time << " us" << std::endl;
    // }

}

bool RB_Share::any_positive(const std::vector<double>& data) {
    return std::any_of(data.begin(), data.end(), [](double x) { return x > 1; });
}

std::pair<std::vector<int>, arma::vec> RB_Share::alloc_rb(std::vector<std::vector<int>>& group_list, const arma::vec& cg_ue_vec, const arma::cx_mat& H, int ue_index, std::vector<int>& large_ue_list, std::vector<int>& small_ue_list) {
    int len_remain = cfg_->SpatialStreamsNum();
    std::vector<int> sel_UE_list;

    while (len_remain > 0) {
        // std::cout << "ue_index:" << ue_index << std::endl;
        for (auto it = group_list.begin(); it != group_list.end(); ++it) {
            auto& group = *it;
            // size_t index = std::distance(group_list.begin(), it);
            if (std::find(group.begin(), group.end(), ue_index) != group.end()) {
                if (group.size() <= len_remain) {
                    for (int ele : group) {
                        if (std::find(sel_UE_list.begin(), sel_UE_list.end(), ele) == sel_UE_list.end()) {
                            auto idx = std::find(large_ue_list.begin(), large_ue_list.end(), ele);
                            auto idx_1 = std::find(small_ue_list.begin(), small_ue_list.end(), ele);
                            if (idx != large_ue_list.end()) {
                                sel_UE_list.push_back(ele);
                                large_ue_list.erase(idx);
                                --len_remain;
                            } else if (idx_1 != small_ue_list.end()) {
                                sel_UE_list.push_back(ele);
                                small_ue_list.erase(idx_1);
                                --len_remain;
                            }
                        }
                    }
                    group_list.erase(it);
                    break;
                } else {
                    for (int ele : group) {
                        if (std::find(sel_UE_list.begin(), sel_UE_list.end(), ele) == sel_UE_list.end()) {
                            auto idx_3 = std::find(large_ue_list.begin(), large_ue_list.end(), ele);
                            if (idx_3 != large_ue_list.end()) {
                                sel_UE_list.push_back(ele);
                                large_ue_list.erase(idx_3);
                                --len_remain;
                            }
                        }
                        if (len_remain<=0) break;
                        
                    }

                    if (len_remain > 0) {
                        for (int ele : group) {
                            if (std::find(sel_UE_list.begin(), sel_UE_list.end(), ele) == sel_UE_list.end()) {
                                auto idx_4 = std::find(small_ue_list.begin(), small_ue_list.end(), ele);
                                if (idx_4 != small_ue_list.end()) {
                                    sel_UE_list.push_back(ele);
                                    small_ue_list.erase(idx_4);
                                    --len_remain;
                                }
                            }
                            if (len_remain<=0) break;
                        }
                    }
                    break;
                }
            }
        }

        if (len_remain>0) {
            if (!large_ue_list.empty())
            {
                double max_value = 0;
                int max_index = -1;
                for (int index : large_ue_list) {
                    if (cg_ue_vec(index) > max_value) {
                        max_value = cg_ue_vec(index);
                        max_index = index;  // Store the actual index within the original vector
                    
                    }
                    // std::cout << "Max Index:" << max_index << std::endl;
                }
                ue_index = max_index;

            }else if (!small_ue_list.empty())
            {
                double max_value = 0;
                int max_index = -1;
                for (int index : small_ue_list) {
                    if (cg_ue_vec(index) > max_value) {
                        max_value = cg_ue_vec(index);
                        max_index = index;  // Store the actual index within the original vector
                    }
                }
                ue_index = max_index;
            }else{
                break;
            }
        }
    }

    arma::cx_mat H_s(sel_UE_list.size(), H.n_cols);

    for (size_t i = 0; i < sel_UE_list.size(); ++i) {
        H_s.row(i) = H.row(sel_UE_list[i]);
    }

    arma::vec sig_user = arma::real(arma::diagvec(arma::inv(H_s * H_s.t())));

 
    arma::vec data_rate = arma::log2(1 + 10 / sig_user);

    return std::make_pair(sel_UE_list, data_rate);
     
}


bool RB_Share::IsUeScheduled(size_t frame_id, size_t prb_id,
                                         size_t ue_id) {
    size_t gp = frame_id % num_groups_;
    return (schedule_buffer_[gp][ue_id + cfg_->UeAntNum() * prb_id] != 0);
}

arma::uvec RB_Share::ScheduledUeMap(size_t frame_id,
                                                size_t prb_id) {
    size_t gp = frame_id % num_groups_;
    return arma::uvec(reinterpret_cast<unsigned long long*>(
                            &schedule_buffer_[gp][cfg_->UeAntNum() * prb_id]),
                        cfg_->UeAntNum(), false);
}

arma::uvec RB_Share::ScheduledUeList(size_t frame_id,
                                                 size_t prb_id) {
    size_t gp = frame_id % num_groups_;
    return sort(arma::uvec(
        reinterpret_cast<unsigned long long*>(
            &schedule_buffer_index_[gp][cfg_->SpatialStreamsNum() * prb_id]),
        cfg_->SpatialStreamsNum(), false));
}

void RB_Share::Update(size_t frame_id) {
    selected_group_ = frame_id % num_groups_;
}

size_t RB_Share::GetGroup(size_t frame_id) {
    return frame_id % num_groups_;
}

size_t RB_Share::SelectedUlMcs(size_t frame_id, size_t ue_id) {
    return ul_mcs_buffer_[frame_id % num_groups_][ue_id];
}

size_t RB_Share::SelectedDlMcs(size_t frame_id, size_t ue_id) {
    return dl_mcs_buffer_[frame_id % num_groups_][ue_id];
}