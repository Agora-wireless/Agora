function inspect_agora_results(dataset_filename, inspect_frame, verbose)
    %%Load data from the input file and inspect evm and snr
    % -------- Fixed Values --------
    group_id = '/Data';
    %Display file / group attributes and datasets
    if verbose == "true"
        h5disp(dataset_filename,group_id);
    end

    samples_per_slot = double(h5readatt(dataset_filename, group_id, 'SLOT_SAMP_LEN'));
    tx_zero_prefix_len = double(h5readatt(dataset_filename, group_id, 'TX_ZERO_PREFIX_LEN'));
    data_size = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_NUM'));
    data_start = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_START'));
    data_stop = double(h5readatt(dataset_filename, group_id, 'OFDM_DATA_STOP'));
    fft_size = double(h5readatt(dataset_filename, group_id, 'OFDM_CA_NUM'));
    cp_len = double(h5readatt(dataset_filename, group_id, 'CP_LEN'));
    total_dl_symbols = double(h5readatt(dataset_filename, group_id, 'DL_SLOTS'));
    dl_pilot_symbols = double(h5readatt(dataset_filename, group_id, 'DL_PILOT_SLOTS'));
    dl_data_symbols = total_dl_symbols - dl_pilot_symbols;
    total_users = 1;

    %Choose the downlink data
    dataset_id = '/DownlinkData';

    % Dimensions  [Samples, Ant, Symbol, Cells, Frame]
    start = [1 1 1 1 inspect_frame];
    count = [(samples_per_slot * 2) total_users total_dl_symbols 1 1];
    %Display Info
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    %Generate a int16 array
    rx_syms_hdf5 = h5read(dataset_filename, strcat(group_id,dataset_id), start, count);
    %Convert to double and scale
    rx_syms_scaled_double = double(rx_syms_hdf5) ./ double(intmax('int16'));
    clear rx_syms_hdf5;
    %Convert to complex double
    % Samples x User x Symbol
    rx_syms_cxdouble = complex(rx_syms_scaled_double(1:2:end,:,:), rx_syms_scaled_double(2:2:end,:, :));
    clear rx_syms_scaled_double;
    % Split off pilots and data
    rx_pilot_cxdouble = rx_syms_cxdouble(:,:,1:dl_pilot_symbols);
    rx_data_cxdouble = rx_syms_cxdouble(:,:,1+dl_pilot_symbols:end);
    clear start count rx_syms_cxdouble;

    dataset_id = '/TxPilot';
    %*2 for complex type (native float)
    total_samples = data_size * 2;
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end 
    start = [1 1 1 1 1];
    count = [total_samples total_users 1 1 1];
    tx_pilot_hdf5 = double(h5read(dataset_filename, strcat(group_id,dataset_id), start, count));
    %Convert to complex
    tx_pilot_cxdouble = complex(tx_pilot_hdf5(1:2:end,:), tx_pilot_hdf5(2:2:end,:));
    clear tx_pilot_hdf5 dataset_id start count;

    dataset_id = '/TxData';
    %Compare the pilot data
    if verbose == "true"
        h5disp(dataset_filename,strcat(group_id,dataset_id));
    end
    start = [1 1 1 1 1];
    count = [total_samples total_users total_dl_symbols 1 1];
    tx_data_hdf5 = double(h5read(dataset_filename, strcat(group_id,dataset_id), start, count));
    %Convert to complex type
    tx_data_cxdouble = complex(tx_data_hdf5(1:2:end,:,:), tx_data_hdf5(2:2:end,:,:));
    % Samples (complex) x User Ant x Downlink Symbol Id
    % removing the pilot for now because it doesn't check out?
    %tx_pilot_bad = tx_data_cxdouble(:,:,1);
    tx_data_cxdouble = tx_data_cxdouble(:,:,2:end);
    clear start count total_samples tx_data_hdf5 dataset_id;

    clear dataset_filename group_id;


    %% Process Loaded Files (Channel Estimation and Equalization)
    % Plot Constellations and print EVMs and SNRs
    start_id = cp_len + tx_zero_prefix_len;
    snr = zeros(1, total_users);
    evm = zeros(1, total_users);
    cl = 0;
    nz_sc_idx = data_start+1:data_stop;
    plt_trk_sp = 16;
    data_sc_idx = setdiff(1:data_size, 1:plt_trk_sp:data_size);
    for u=1:total_users
        % Process pilots
        ch_est = zeros(data_size, dl_pilot_symbols);
        for p=1:dl_pilot_symbols
          rx_pilot_f_tmp = fftshift(fft(rx_pilot_cxdouble(start_id + 1:start_id + fft_size, u, p)));
          ch_est(:, p) = rx_pilot_f_tmp(nz_sc_idx) ./ tx_pilot_cxdouble(:, u, p);
        end
        clear p
        if dl_pilot_symbols == 1
            ch_est_mean = ch_est;
        else
            ch_est_mean = mean(ch_est, 2);
        end

        %Process data symbols
        data_phase_corr = zeros(data_size, dl_data_symbols);
        aevms = zeros(u, dl_data_symbols);
        for d=1:dl_data_symbols
          rx_data_f_tmp = fftshift(fft(rx_data_cxdouble(start_id + 1:start_id + fft_size, u, d)));
          data_eq = rx_data_f_tmp(nz_sc_idx) ./ ch_est_mean;

          % pilot tracking
          phase_err = angle(mean((data_eq(1:plt_trk_sp:end) .* conj(tx_pilot_cxdouble(1:plt_trk_sp:end, u)))));
          data_phase_corr(data_sc_idx, d) = data_eq(data_sc_idx) .* exp(-1j*phase_err);

          evm_mat = abs(data_phase_corr(data_sc_idx, d) - tx_data_cxdouble(data_sc_idx, u, d)).^2;
          aevms(u, d) = mean(evm_mat(:)); % needs to be a scalar

          cl = cl + 1;
          figure(cl);
          scatter(real(data_phase_corr(data_sc_idx, d)), imag(data_phase_corr(data_sc_idx, d)),'r')
          hold on
          scatter(real(tx_data_cxdouble(data_sc_idx)), imag(tx_data_cxdouble(data_sc_idx)),'b')
          title(['Constellation [User ', num2str(u), ', Symbol ', num2str(d), ']'])
        end
        clear d

        snr(u) = 10*log10(1./mean(aevms(u, :))); % calculate in dB scale.
        evm(u) = mean(aevms(u, :)) * 100;
    end

    clear u cl start_id rx_pilot_f_tmp rx_data_f_tmp plt_trk_sp nz_sc_idx;

    
    disp(['Frame Inspect: ', num2str(inspect_frame)]);
    disp(['SNR: ', num2str(snr)]);
    disp(['EVM: ', num2str(evm)]);
end
