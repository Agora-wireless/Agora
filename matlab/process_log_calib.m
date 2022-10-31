clear all;
filename = 'log-calib-BS-1ch.csv';
logcalibBS1 = readmatrix(filename);
logcalib = logcalibBS1(1:end-1, :);
num_ant = (size(logcalib, 2) / 2) - 1;
num_sc = max(logcalib(:, 2)) + 1;
total_samps = size(logcalib, 1);
num_frame = total_samps / num_sc;

permute_mat = reshape(logcalib(:, 2), num_sc, num_frame);
calib_bs_mat = reshape(logcalib(:, 3:num_ant * 2 + 2)', num_ant * 2, num_sc, num_frame);
calib_bs_mat_ord = zeros(num_ant, num_sc, num_frame);
for i = 1:num_frame
    new_ord = 1 + permute_mat(:, i).';
    calib_bs_mat_ord(:, :, i) = calib_bs_mat(1:2:end, new_ord, i) + calib_bs_mat(2:2:end, new_ord, i)*1i;
end

figure(1);
title('Magnitude');
for i = 1:num_ant
   subplot(4, 4, i)
   imagesc(squeeze(abs(calib_bs_mat_ord(i, :, :))))
   title('Ant' , i);
end

figure(2);
title('Phase');
for i = 1:num_ant
   subplot(4, 4, i)
   imagesc(squeeze(angle(calib_bs_mat_ord(i, :, :))))
   title('Ant' , i);
end

