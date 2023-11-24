function tx_syms = mod_sym(data, MOD_ORDER)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%	Author(s): C. Nicolas Barati nicobarati@rice.edu 
%		Rahman Doost-Mohamamdy: doost@rice.edu
%
%---------------------------------------------------------------------
% Original code copyright Mango Communications, Inc.
% Distributed under the WARP License http://warpproject.org/license
% Copyright (c) 2018-2019, Rice University
% RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
% ---------------------------------------------------------------------
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


    % Functions for data -> complex symbol mapping
    % These anonymous functions implement the modulation mapping from IEEE 802.11-2012 Section 18.3.5.8
    modvec_bpsk   =  (1/sqrt(2))  .* [-1 1];
    modvec_16qam  =  (1/sqrt(10)) .* [-3 -1 +3 +1];
    modvec_64qam  =  (1/sqrt(42)) .* [-7 -5 -1 -3 +7 +5 +1 +3];

    mod_fcn_bpsk  = @(x) complex(modvec_bpsk(1+x),0);
    mod_fcn_qpsk  = @(x) complex(modvec_bpsk(1+bitshift(x, -1)), modvec_bpsk(1+mod(x, 2)));
    mod_fcn_16qam = @(x) complex(modvec_16qam(1+bitshift(x, -2)), modvec_16qam(1+mod(x,4)));    
    mod_fcn_64qam = @(x) complex(modvec_64qam(1+bitshift(x, -3)), modvec_64qam(1+mod(x,8)));

    % Map the data values on to complex symbols
    switch MOD_ORDER
        case 2         % BPSK
            tx_syms = arrayfun(mod_fcn_bpsk, data);
        case 4         % QPSK
            tx_syms = arrayfun(mod_fcn_qpsk, data);
        case 16        % 16-QAM
            tx_syms = arrayfun(mod_fcn_16qam, data);
        case 64        % 64-QAM
            tx_syms = arrayfun(mod_fcn_64qam, data);
        otherwise
            error('Invalid MOD_ORDER (%d)!  Must be in [2, 4, 16, 64]\n', MOD_ORDER);
    end

end