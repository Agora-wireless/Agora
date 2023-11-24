
clc
clear
close all

offset=20;
range = 900;  % number of frames to display
frame_num = offset+range-1; % last frame numbber
order=16;
cons_idx = [15, 2, 10 ,0, 8];
EVM_considx = zeros(length(cons_idx), range);
verbose = "false";
dataset_filename = "UeRxData-loc5.h5";
for f=offset: frame_num
    EVM_considx(:,f-offset+1) = inspect_single_frame_showEVM(dataset_filename, f, verbose, cons_idx, order);
end

figure; hold on;
for i=1:length(cons_idx)
    plot(offset:frame_num, 100.*EVM_considx(i,:), 'DisplayName', num2str(cons_idx(i)), LineWidth=2)
end
xlabel('Frame index'); ylabel('EVM Percentage');

grid on
legend show
