clear;
close all;

symbol_size = 896;
fft_size = 512;
data_size = 304;
offset = 160;
cp = 32;
n_user = 1;
pig = 0;
dig = [1:2];

nz_start_idx = (fft_size - data_size)/2;
nz_sc_idx = nz_start_idx+1:nz_start_idx+data_size;

plt_trk_sp = 16;
data_sc_idx = setdiff(1:data_size, 1:plt_trk_sp:data_size);

rx_pilot = zeros(symbol_size, n_user, length(pig));
tx_pilot = zeros(data_size, n_user, length(pig));
rx_data = zeros(symbol_size, n_user, length(dig));
tx_data=zeros(data_size, n_user, length(dig));

%% Load Data
% read rx and tx pilot files
for u=1:n_user
  for p=1:length(pig)
    filename = 'rxpilot' + string(pig(p)) + '_' + string(u-1) + '.bin';
    fileID = fopen(filename, 'r');
    rx = fread(fileID, 'short');
    fclose(fileID);
    rx_pilot(:, u, p) = rx(1:2:end)/32768 + 1j*rx(2:2:end)/32768;
    
    filename = 'txpilot_f_' + string(pig(p)) + '_' + string(u-1) + '.bin';
    fileID = fopen(filename, 'r');
    tx = fread(fileID, 'float');
    fclose(fileID);
    tx_pilot(:, u, p) = tx(1:2:end) + 1j*tx(2:2:end);
  end  
end

% read rx and tx data files
for u=1:n_user
  for d=1:length(dig)
    filename = 'rxdata' + string(dig(d)) + '_' + string(u-1) + '.bin';
    fileID = fopen(filename, 'r');
    data = fread(fileID, 'short');
    fclose(fileID);
    rx_data(:, u, d) = data(1:2:end)/32768 + 1j*data(2:2:end)/32768;
    
    filename = 'txdata' + string(dig(d)) + '_' + string(u-1) + '.bin';
    fileID = fopen(filename, 'r');
    data = fread(fileID, 'float');
    fclose(fileID);
    txdata_tmp = data(1:2:end) + 1j*data(2:2:end);
    tx_data(:, u, d) = txdata_tmp(nz_sc_idx);
  end  
end


%% Process Loaded Files (Channel Estimation and Equalization)
% Plot Constellations and print EVMs and SNRs
start_id = cp + offset;
snr = zeros(1, n_user);
evm = zeros(1, n_user);
cl = 0;
for u=1:n_user
    ch_est = zeros(data_size, length(pig));
    for p=1:length(pig)
      rx_pilot_f_tmp = fft(rx_pilot(start_id + 1:start_id + fft_size, u, p));
      ch_est(:, p) = rx_pilot_f_tmp(nz_sc_idx) ./ tx_pilot(:, u, p);
    end
    if length(pig) == 1
        ch_est_mean = ch_est;
    else
        ch_est_mean = mean(ch_est, 2);
    end
    data_phase_corr = zeros(data_size, length(dig));
    aevms = zeros(u, length(dig));
    for d=1:length(dig)
      rx_data_f_tmp = fft(rx_data(start_id + 1:start_id + fft_size, u, d));
      data_eq = rx_data_f_tmp(nz_sc_idx) ./ ch_est_mean;
      
      % pilot tracking
      phase_err = angle(mean((data_eq(1:plt_trk_sp:end) .* conj(tx_pilot(1:plt_trk_sp:end, u)))));
      data_phase_corr(data_sc_idx, d) = data_eq(data_sc_idx) .* exp(-1j*phase_err);
      
      evm_mat = abs(data_phase_corr(data_sc_idx, d) - tx_data(data_sc_idx, u, d)).^2;
      aevms(u, d) = mean(evm_mat(:)); % needs to be a scalar
      
      cl = cl + 1;
      figure(cl);
      scatter(real(data_phase_corr(data_sc_idx, d)), imag(data_phase_corr(data_sc_idx, d)),'ro')
      hold on
      scatter(real(tx_data(data_sc_idx)), imag(tx_data(data_sc_idx)),'bo')
      title(['Constellation [User ', num2str(u), ', Symbol ', num2str(d), ']'])
    end
   



    snr(u) = 10*log10(1./mean(aevms(u, :))); % calculate in dB scale.
    evm(u) = mean(aevms(u, :)) * 100;

end

snr
evm
