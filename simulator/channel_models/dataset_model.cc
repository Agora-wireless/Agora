#include <H5Cpp.h>
#include <iostream>

#include "logger.h"
#include "dataset_model.h"

void DatasetModel::InstantiateDataset(std::string dataset_path) 
{

    hsize_t frames_num = 0;
    hsize_t bss_num = 0;
    hsize_t ues_num = 0;

    current_frame_num = 0;

    try
    {

        H5::H5File file(dataset_path, H5F_ACC_RDONLY);
        H5::DataSet h_r_dataset =
            file.openDataSet("H_r");  //H Matrix Real part dataset
        H5::DataSet h_i_dataset =
            file.openDataSet("H_i");  //H Matrix Imaginary part dataset

        H5::DataSpace real_dataspace = h_r_dataset.getSpace();
        H5::DataSpace im_dataspace = h_i_dataset.getSpace();

        hsize_t real_dimensions[3];
        real_dataspace.getSimpleExtentDims(real_dimensions, NULL);

        hsize_t im_dimensions[3];
        im_dataspace.getSimpleExtentDims(im_dimensions, NULL);

        //Check the Matrix types dimensions
        if (im_dimensions[0] != real_dimensions[0] || im_dimensions[1] != real_dimensions[1] || im_dimensions[2] != real_dimensions[2]) 
        {
            AGORA_LOG_ERROR("Invalid Dataset size\n");
        }

        frames_num = real_dimensions[0];
        bss_num = real_dimensions[1];
        ues_num = real_dimensions[2];

        //Instantiate the matrices data
        float* real_temp_data = new float[bss_num * ues_num * frames_num];
        float* im_temp_data = new float[bss_num * ues_num * frames_num];

        h_r_dataset.read(real_temp_data, H5::PredType::NATIVE_FLOAT);
        h_i_dataset.read(im_temp_data, H5::PredType::NATIVE_FLOAT);

        arma::Cube<float> real_data(real_temp_data, bss_num, ues_num, frames_num, false, true);
        arma::Cube<float> im_data(im_temp_data, bss_num, ues_num, frames_num, false, true);

        real_matrices_data = real_data;
        im_matrices_data = im_data;

    }
    catch (H5::FileIException& error) 
    {
    
        AGORA_LOG_ERROR("Read Dataset %s, FileIException - %s\n",
                        dataset_path.c_str(), error.getCDetailMsg());
        H5::FileIException::printErrorStack();
        throw error;
    
    }
    catch (H5::DataSetIException& error) 
    {

        AGORA_LOG_ERROR("Read Dataset %s, DataSetIException - %s\n",
                        dataset_path.c_str(), error.getCDetailMsg());
    
        H5::DataSetIException::printErrorStack();
        throw error;
    
    }
    catch (H5::DataSpaceIException& error) 
    {
    
        AGORA_LOG_ERROR("CreateDataset %s, DataSpaceIException - %s\n",
                        dataset_path.c_str(), error.getCDetailMsg());
        
        H5::DataSpaceIException::printErrorStack();
        throw error;
    
    }

    AGORA_LOG_INFO(
        "Dataset Succesfully loaded\n"
        "Path: %s \n"
        "Frames: %lld \n"
        "BSs: %lld \n"
        "UEs: %lld \n",
        dataset_path.c_str(), frames_num, bss_num, ues_num);

}

arma::cx_fmat DatasetModel::GetMatrixByFrame(hsize_t frame) 
{

    arma::Mat<float> real_matrix = real_matrices_data.slice(frame);
    arma::Mat<float> im_matrix = im_matrices_data.slice(frame);

    return arma::cx_fmat(real_matrix, im_matrix);

}

arma::cx_fmat DatasetModel::GetAndUpdateMatrix()
{

    return GetMatrixByFrame( current_frame_num++ );

}