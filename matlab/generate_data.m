%% Parameters
MOD_ORDER = 16;
N_SC = 1024;
SC_IND_DATA = 1:1024;
NUM_BS_ANT = 96;
NUM_UE = 4;
N_OFDM_SYMS = 36*NUM_UE; 
NUM_SUBFRAME = 10;
N_DATA_SYMS = N_OFDM_SYMS * length(SC_IND_DATA);
GENERATE_PILOT = 0;
GENERATE_DATA = 0;
CP_LEN = 0;
frmLen = 100;       % frame length

%% Generate pilot
if GENERATE_PILOT
    pilot_f = randi(3,length(SC_IND_DATA),1)-2;
    pilot_f(pilot_f==0) = 1;
    pilot_t = ifft(pilot_f, 1024);
    fileID = fopen('pilot_f.bin','w');
    fwrite(fileID,pilot_f,'float');
    fclose(fileID);
    fileID = fopen('pilot_t.bin','w');
    fwrite(fileID,pilot_t,'float');
    fclose(fileID);
else
    fileID = fopen('pilot_f.bin');
    pilot_f = fread(fileID,[1024,1],'float');
    pilot_t = ifft(pilot_f,1024);
    fclose(fileID);
%     fileID = fopen('pilot_t.bin');
%     pilot_t = fread(fileID,[1024,1],'float');
end


%% Generate data
if GENERATE_DATA
    tx_data = randi(MOD_ORDER, 1, N_DATA_SYMS) - 1;
    fileID_data = fopen('orig_data.bin','w');
    fwrite(fileID_data,tx_data,'int');
    fclose(fileID_data);
else
    fileID_data = fopen('orig_data.bin');
    tx_data = fread(fileID_data,[N_DATA_SYMS,1],'int');
    fclose(fileID);
end

%% Modulate data

modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
modvec_16qam  = (1/sqrt(10))  .* [-3 -1 +3 +1];

mod_fcn_bpsk  = @(x) complex(modvec_bpsk(1+x),0);
mod_fcn_qpsk  = @(x) complex(modvec_bpsk(1+bitshift(x, -1)), modvec_bpsk(1+mod(x, 2)));
mod_fcn_16qam = @(x) complex(modvec_16qam(1+bitshift(x, -2)), modvec_16qam(1+mod(x,4)));

switch MOD_ORDER
    case 2         % BPSK
        tx_syms = arrayfun(mod_fcn_bpsk, tx_data);
    case 4         % QPSK
        tx_syms = arrayfun(mod_fcn_qpsk, tx_data);
    case 16        % 16-QAM
        tx_syms = arrayfun(mod_fcn_16qam, tx_data);
    otherwise
        fprintf('Invalid MOD_ORDER (%d)!  Must be in [2, 4, 16]\n', MOD_ORDER);
        return;
end

%% IFFT

% % Reshape the symbol vector into two different spatial streams
% % size: N_SC*N_OFDM_SYMS \times NUM_UE
% tx_syms = reshape(tx_syms, length(tx_syms)/NUM_UE, NUM_UE);

% Reshape the symbol vector to a matrix with one column per OFDM symbol
% size: N_SC \times N_OFDM_SYMS/NUM_UE \times NUM_UE
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_OFDM_SYMS/NUM_UE, NUM_UE);
tx_data_orig = reshape(tx_data, length(SC_IND_DATA), N_OFDM_SYMS/NUM_UE, NUM_UE);
% Construct the IFFT input matrix
% ifft_in_mat = zeros(NUM_BS_ANT, N_SC, N_OFDM_SYMS/NUM_BS_ANT);


% Insert the data and pilot values; other subcarriers will remain at 0
ifft_in_mat = tx_syms_mat;
% ifft_in_mat_A(SC_IND_PILOTS, :) = pilots_mat_A;


%Perform the IFFT 
tx_payload_mat = ifft(ifft_in_mat, N_SC, 1);

%% Add CP
% Insert the cyclic prefix
if(CP_LEN > 0)
    tx_cp = tx_payload_mat(:,(end-CP_LEN+1 : end), :);
    tx_payload_mat = cat(2,tx_cp,tx_payload_mat);
end

%% Generate CSI

% Create the Rayleigh distributed channel response matrix
%   for two transmit and two receive antennas
% size: NUM_UE \times NUN_BS_ANT
H = (randn(NUM_UE, NUM_BS_ANT) + 1i*randn(NUM_UE, NUM_BS_ANT))/sqrt(2);

%% Tx data + Pliots
pilot_all_ue = zeros(N_SC,NUM_UE,NUM_UE);
for i = 1:NUM_UE
    pilot_all_ue(:,i,i)=pilot_t;
end
% pilot_all_ue = reshape(pilot_all_ue, N_SC,1,NUM_UE);
tx_mat_all = cat(2,pilot_all_ue,tx_payload_mat);
tx_mat = reshape(tx_mat_all,size(tx_mat_all,1)*size(tx_mat_all,2),size(tx_mat_all,3));

%% Rx data
rx_mat = tx_mat*H;
rx_mat_all = reshape(rx_mat,size(tx_mat_all,1),size(tx_mat_all,2),NUM_BS_ANT);
rx_mat_all = permute(rx_mat_all,[1,3,2]);
rx_vec = reshape(rx_mat_all, 1, numel(rx_mat_all));
% if GENERATE_DATA
rx_vec_float = zeros(1,size(rx_vec,2)*2);
rx_vec_float(1:2:end) = real(rx_vec);
rx_vec_float(2:2:end) = imag(rx_vec);
%     fileID = fopen('rx_data.bin','w');
%     fwrite(fileID,rx_vec_float,'float');
%     fclose(fileID);
% end

    
%% CSI estimation
CSI_est = zeros(N_SC,NUM_BS_ANT,NUM_UE);
for i = 1:NUM_UE
    CSI_est(:,:,i) = fft(rx_mat_all(:,:,i),N_SC,1).*repmat(pilot_f,1,NUM_BS_ANT);
end
fprintf("CSI_estimation error: %d/%d\n",sum(sum(abs(squeeze(CSI_est(2,:,:)).'-H)>1e-3)),length(H(:)));

H_est = squeeze(CSI_est(1,:,:));


%% Equalization + demodulation

% size: NUM_UE \times NUM_BS_ANT
precoder = pinv(H_est);
rx_mat_data = rx_mat_all(:,:,NUM_UE+1:end);
% size: NUM_BS_ANT \times N_SC \times N_OFDM_SYMS/NUM_UE
rx_mat_data = permute(rx_mat_data,[2,1,3]);
rx_data_fft = fft(rx_mat_data,N_SC,2);
rx_data_fft = reshape(rx_data_fft, NUM_BS_ANT, N_SC*N_OFDM_SYMS/NUM_UE);
rx_syms = precoder * rx_data_fft;
rx_syms_mat = reshape(rx_syms, NUM_UE,N_SC,N_OFDM_SYMS/NUM_UE);


demod_fcn_bpsk = @(x) double(real(x)>0);
demod_fcn_qpsk = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));

switch(MOD_ORDER)
    case 2         % BPSK
        rx_data = arrayfun(demod_fcn_bpsk, rx_syms);
    case 4         % QPSK
        rx_data = arrayfun(demod_fcn_qpsk, rx_syms);
    case 16        % 16-QAM
        rx_data = arrayfun(demod_fcn_16qam, rx_syms);
end

rx_data = reshape(rx_data,NUM_UE,N_SC,N_OFDM_SYMS/NUM_UE);

rx_data = permute(rx_data,[2,3,1]);

% % Reshape to a vector
% % size: 1 \times N_SC*N_OFDM_SYMS
% tx_payload_vec = reshape(tx_payload_mat, 1, numel(tx_payload_mat));
% 
% 
% % Construct the full time-domain OFDM waveform
% tx_vec = [repmat(pilot_t.',1,NUM_UE) tx_payload_vec];
% 
% 
% 
% rx_mat = tx_vec.reshape(tx_vec,N_SC,NUM_SUBFRAME);
