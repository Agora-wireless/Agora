function inspect_single_ul_frame(dataset_filename, inspect_frame, verbose)
    %%Load data from the input file and inspect evm and snr
    %dataset_filename = "trace-uplink-2022-9-27-15-10-12_1_8x1.hdf5";
    %inspect_frame = 5;
    %verbose = "true";
    % -------- Fixed Values --------
    group_id = '/Data';
    %Display file / group attributes and datasets
    if verbose == "true"
        h5disp(dataset_filename, group_id);
    end

    samples_per_slot = double(h5readatt(dataset_filename, group_id, 'SLOT_SAMP_LEN'));
    tx_zero_prefix_len = double(h5readatt(dataset_filename, group_id, 'TX_ZERO_PREFIX_LEN'));
    data_size = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_NUM'));
    data_start = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_START'));
    data_stop = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_STOP'));
    fft_size = double(h5readatt(dataset_filename, group_id, 'OFDM_CA_NUM'));
    cp_len = double(h5readatt(dataset_filename, group_id, 'CP_LEN'));
    total_ul_symbols = double(h5readatt(dataset_filename, group_id, 'UL_SLOTS'));
    ul_pilot_symbols = double(h5readatt(dataset_filename, group_id, 'UL_PILOT_SLOTS'));
    total_users = double(h5readatt(dataset_filename, group_id, 'CL_NUM'));
    bs_antennas = double(h5readatt(dataset_filename, group_id, 'ANT_NUM'));
    pilot_symbols = double(h5readatt(dataset_filename, group_id, 'PILOT_SLOTS'));

    %Choose the downlink data
    dataset_id = '/UplinkData';

    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id, dataset_id));
    end
    % Dimensions  [Samples, Ant, Symbol, Cells, Frame]
    start = [1 1 1 1 inspect_frame];
    count = [(samples_per_slot * 2) bs_antennas total_ul_symbols 1 1];
    %Generate a int16 array
    rx_syms_hdf5 = h5read(dataset_filename, strcat(group_id, dataset_id), start, count);
    %Convert to double and scale
    rx_syms_scaled_double = double(rx_syms_hdf5) ./ double(intmax('int16'));
    clear rx_syms_hdf5;
    %Convert to complex double
    % Samples x User x Symbol
    rx_syms_cxdouble = complex(rx_syms_scaled_double(1:2:end, :, :), rx_syms_scaled_double(2:2:end, :, :));
    clear rx_syms_scaled_double;
    % Split off pilots and data
    rx_pilot_cxdouble = rx_syms_cxdouble(:,:,1:ul_pilot_symbols);
    rx_data_cxdouble = rx_syms_cxdouble(:,:,1+ul_pilot_symbols:end);
    clear start count;

    %Choose the pilot data
    dataset_id = '/Pilot_Samples';

    % Dimensions  [Samples, Ant, Symbol, Cells, Frame]
    start = [1 1 1 1 inspect_frame];
    count = [(samples_per_slot * 2) bs_antennas pilot_symbols 1 1];
    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    %Generate a int16 array
    rx_pilot_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id), start, count);
    %Convert to double and scale
    rx_pilot_scaled_double = double(rx_pilot_hdf5) ./ double(intmax('int16'));
    clear rx_pilot_hdf5;
    %Convert to complex double
    % Samples x User x Symbol
    rx_pilot_cxdouble = complex(rx_pilot_scaled_double(1:2:end,:,:), rx_pilot_scaled_double(2:2:end,:, :));
    clear rx_pilot_scaled_double;
    clear start count;
    
    %Plot Rx waveform
    for u = 1:bs_antennas
        combined_rx = reshape(squeeze(rx_syms_cxdouble(:, u, :)), [], 1);
        figure('Name', ['Ant ' num2str(u) ', Frame ' num2str(inspect_frame) ' Receive WaveForm' ]);
        tiledlayout(2,1)
        %Top (Real)
        nexttile;
        plot(real(combined_rx));
        %axis([0 inf -1 1]);
        title('Rx Real (I)');
        %Bottom (Q)
        nexttile;
        plot(imag(combined_rx));
    end
    clear u;
end
