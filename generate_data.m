%% Parameters
MOD_ORDER = 16;
SC_IND_DATA = 1:1024;
N_OFDM_SYMS = 36; 
N_DATA_SYMS = N_OFDM_SYMS * length(SC_IND_DATA);
GENERATE_PILOT = 0;

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

preamble_mimo_A = [lts_t(33:64), lts_t, zeros(1,96)];
preamble_mimo_B = [zeros(1,96), lts_t(33:64), lts_t];

tx_data = randi(MOD_ORDER, 1, N_DATA_SYMS) - 1;

modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
modvec_16qam  = (1/sqrt(10))  .* [-3 -1 +3 +1];

mod_fcn_bpsk  = @(x) complex(modvec_bpsk(1+x),0);
mod_fcn_qpsk  = @(x) complex(modvec_bpsk(1+bitshift(x, -1)), modvec_bpsk(1+mod(x, 2)));
mod_fcn_16qam = @(x) complex(modvec_16qam(1+bitshift(x, -2)), modvec_16qam(1+mod(x,4)));