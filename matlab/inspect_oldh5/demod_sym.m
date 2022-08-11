function data = demod_sym(syms, MOD_ORDER)
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

% Demodulate symbols
    demod_fcn_bpsk = @(x) double(real(x)>0);
    demod_fcn_qpsk = @(x) double(2*(real(x)>0) + 1*(imag(x)>0));
    demod_fcn_16qam = @(x) (8*(real(x)>0)) + (4*(abs(real(x))<0.6325)) + (2*(imag(x)>0)) + (1*(abs(imag(x))<0.6325));
    demod_fcn_64qam = @(x) (32*(real(x)>0)) + (16*(abs(real(x))<0.6172))...
        + (8*((abs(real(x))<(0.9258))&&((abs(real(x))>(0.3086)))))...
        + (4*(imag(x)>0)) + (2*(abs(imag(x))<0.6172))...
        + (1*((abs(imag(x))<(0.9258))&&((abs(imag(x))>(0.3086)))));

    switch(MOD_ORDER)
        case 2         % BPSK
            data = arrayfun(demod_fcn_bpsk, syms);
        case 4         % QPSK
            data = arrayfun(demod_fcn_qpsk, syms);
        case 16        % 16-QAM
            data = arrayfun(demod_fcn_16qam, syms);
        case 64        % 64-QAM
            data = arrayfun(demod_fcn_64qam, syms);
    end

end