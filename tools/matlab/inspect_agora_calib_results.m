function rx_calib_cxdouble = inspect_agora_calib_results(dataset_filename, verbose)
    %%Load data from the input file and inspect evm and snr
    %dataset_filename = "BsRxData.h5";
    %inspect_frame = 100;
    %verbose = "false";
    % -------- Fixed Values --------
    group_id = '/Data';
    %Display file / group attributes and datasets
    if verbose == "true"
        h5disp(dataset_filename, group_id);
    end

    total_frames = double(h5readatt(dataset_filename, group_id, 'MAX_FRAME'));
    samples_per_slot = double(h5readatt(dataset_filename, group_id, 'SLOT_SAMP_LEN'));
    tx_zero_prefix_len = double(h5readatt(dataset_filename, group_id, 'TX_ZERO_PREFIX_LEN'));
    data_size = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_NUM'));
    data_start = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_START'));
    data_stop = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_STOP'));
    fft_size = double(h5readatt(dataset_filename, group_id, 'OFDM_CA_NUM'));
    cp_len = double(h5readatt(dataset_filename, group_id, 'CP_LEN'));
    total_ul_symbols = double(h5readatt(dataset_filename, group_id, 'UL_SLOTS'));
    ul_pilot_symbols = double(h5readatt(dataset_filename, group_id, 'UL_PILOT_SLOTS'));
    %cl_sdr_id = h5readatt(dataset_filename, group_id, 'CL_SDR_ID');

    %Choose the Pilot data
    dataset_id = '/Pilot_Samples';

    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    %Generate a int16 array
    rx_pilots_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id));
    % Dimensions  [Samples, Ant, Symbol, Cells, Frame]
    total_users = size(rx_pilots_hdf5, 2);
    %Convert to double and scale
    rx_pilots_scaled_double = double(rx_pilots_hdf5) ./ double(intmax('int16'));
    clear rx_pilots_hdf5;
    %Convert to complex double
    % Samples x User x Symbol
    rx_pilots_cxdouble = complex(rx_pilots_scaled_double(1:2:end, :, :, 1:total_frames), rx_pilots_scaled_double(2:2:end,:, :, 1:total_frames));
    clear rx_pilots_scaled_double;

    %Choose the Pilot data
    dataset_id = '/Calib_Samples';

    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    %Generate a int16 array
    rx_calib_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id));
    % Dimensions  [Samples, Ant, Symbol (1), Cells, Frame]
    total_antennas = size(rx_calib_hdf5, 2);
    %Convert to double and scale
    rx_calib_scaled_double = double(rx_calib_hdf5) ./ double(intmax('int16'));
    clear rx_calib_hdf5;
    %Convert to complex double
    % Samples x User x Symbol
    rx_calib_cxdouble = complex(rx_calib_scaled_double(1:2:end, :, :, 1:total_frames), rx_calib_scaled_double(2:2:end,:, :, 1:total_frames));
end
