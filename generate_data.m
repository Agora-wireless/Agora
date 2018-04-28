%% Parameters
MOD_ORDER = 16;
N_SC = 1024;
SC_IND_DATA = 1:1024;
NUM_BS_ANT = 96;
NUM_UE = 4;
N_OFDM_SYMS = 9*NUM_UE; 
NUM_SUBFRAME = 10;
N_DATA_SYMS = N_OFDM_SYMS * length(SC_IND_DATA);
GENERATE_PILOT = 0;
CP_LEN = 20;
frmLen = 100;       % frame length

%% Generate pilot
if GENERATE_PILOT
    pilot_f = randi(3,length(SC_IND_DATA),1)-2;
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
    fileID = fopen('pilot_t.bin');
    pilot_t = fread(fileID,[1024,1],'float');
end


%% Generate data


tx_data = randi(MOD_ORDER, 1, N_DATA_SYMS) - 1;

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

% Reshape the symbol vector into two different spatial streams
tx_syms = reshape(tx_syms, length(tx_syms)/NUM_UE, NUM_UE);

% Reshape the symbol vector to a matrix with one column per OFDM symbol
tx_syms_mat = reshape(tx_syms, length(SC_IND_DATA), N_OFDM_SYMS/NUM_UE);

% Construct the IFFT input matrix
% ifft_in_mat = zeros(NUM_BS_ANT, N_SC, N_OFDM_SYMS/NUM_BS_ANT);


% Insert the data and pilot values; other subcarriers will remain at 0
ifft_in_mat = tx_syms_mat;
% ifft_in_mat_A(SC_IND_PILOTS, :) = pilots_mat_A;


%Perform the IFFT
tx_payload_mat = ifft(ifft_in_mat, N_SC, 2);


% Insert the cyclic prefix
if(CP_LEN > 0)
    tx_cp = tx_payload_mat(:,(end-CP_LEN+1 : end), :);
    tx_payload_mat = cap(2,tx_cp,tx_payload_mat);
end

% Reshape to a vector
tx_payload_vec = reshape(tx_payload_mat, 1, numel(tx_payload_mat));


% Construct the full time-domain OFDM waveform
tx_vec = [repmat(pilot_t.',1,NUM_UE) tx_payload_vec];


% Create the Rayleigh distributed channel response matrix
%   for two transmit and two receive antennas
H = (randn(NUM_BS_ANT,NUM_UE) + 1i*randn( NUM_BS_ANT,NUM_UE))/sqrt(2);
rx_mat = tx_vec.reshape(tx_vec,N_SC,NUM_SUBFRAME);
H = repmat(H,)