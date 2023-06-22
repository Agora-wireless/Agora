#include "dataset_model.h"

DatasetChannelModel* DatasetChannelModel::instance{ nullptr };

void DatasetChannelModel::InstantiateDataset( std::string dataset_path )
{

    instance = new DatasetChannelModel( dataset_path );
    
}

DatasetChannelModel::DatasetChannelModel( std::string dataset_path )
{

    current_frame_num = 0;

    H5::H5File file( dataset_path , H5F_ACC_RDONLY );
    H5::DataSet h_r_dataset = file.openDataSet("H_r"); //H Matrix Real part dataset 
    H5::DataSet h_i_dataset = file.openDataSet("H_i"); //H Matrix Imaginary part dataset 

    H5::DataSpace real_dataspace = h_r_dataset.getSpace();
    H5::DataSpace im_dataspace = h_i_dataset.getSpace();

    hsize_t real_dimensions[3];
    real_dataspace.getSimpleExtentDims(real_dimensions, NULL);

    hsize_t im_dimensions[3];
    im_dataspace.getSimpleExtentDims(im_dimensions, NULL);

    //Check the Matrix types dimensions
    if( im_dimensions[0] != real_dimensions[0] || im_dimensions[1] != real_dimensions[1] || im_dimensions[2] != real_dimensions[2] )
    {
        throw std::runtime_error("Invalid Dataset size");
    }

    hsize_t frames_num = real_dimensions[0];
    hsize_t bss_num = real_dimensions[1];
    hsize_t ues_num = real_dimensions[2];

    //Instantiate the matrices data
    float* real_temp_data = new float[bss_num * ues_num * frames_num];
    float* im_temp_data = new float[bss_num * ues_num * frames_num];

    h_r_dataset.read( real_temp_data , H5::PredType::NATIVE_FLOAT );
    h_i_dataset.read( im_temp_data , H5::PredType::NATIVE_FLOAT );

    arma::Cube<float> real_data( real_temp_data, bss_num, ues_num, frames_num, false, true);
    arma::Cube<float> im_data( im_temp_data, bss_num, ues_num, frames_num, false, true);

    real_matrices_data = real_data;
    im_matrices_data = im_data;
    
    //Log Dataset information
	std::printf("Dataset Loaded at: %s \n", dataset_path.c_str());    
	std::printf("Dataset frames: %lld \n", frames_num );
	std::printf("BSs: %lld \n", bss_num );
	std::printf("UEs: %lld \n", bss_num );

}

DatasetChannelModel::~DatasetChannelModel() = default;

arma::cx_fmat DatasetChannelModel::GetMatrixByFrame( hsize_t frame )
{
    
    arma::Mat<float> real_matrix = real_matrices_data.slice( frame );
    arma::Mat<float> im_matrix = im_matrices_data.slice( frame );

    return arma::cx_fmat(real_matrix, im_matrix);

}

arma::cx_fmat DatasetChannelModel::GetAndUpdateMatrix()
{

    return instance->GetMatrixByFrame( instance->current_frame_num++ );

}