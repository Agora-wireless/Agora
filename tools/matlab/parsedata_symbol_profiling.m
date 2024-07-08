clearvars; clc; close all;

rewards = [-35, -30, -25, -20, -15, -10, -5, 40, -30, -35, -40];
rewards_mapped = rewards;
rewards_mapped(find(rewards > 0)) = 0;
latencies = 0:0.1:1;
figure;
rectangle("Position", [0.7 rewards_mapped(end) 0.1 2*abs(rewards(end))], 'EdgeColor', 'g' , 'LineWidth', 2);
for i = 1:length(rewards)-1
    rectangle("Position", [latencies(i) rewards_mapped(i) 0.1 abs(rewards(i))], 'EdgeColor', 'k', 'FaceColor', [0 0.4470 0.7410]);
end
rectangle("Position", [latencies(end) rewards_mapped(end) 0.2 abs(rewards(end))], 'EdgeColor', 'k', 'FaceColor', [0 0.4470 0.7410]);
rectangle("Position", [0.7 rewards_mapped(end) 0.1 2*abs(rewards(end))], 'EdgeColor', 'g' , 'LineWidth', 2);
% xticklabels([0 0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1 1.1]);
grid on;
xlabel('TTI Duration = 1ms');
ylabel('Reward');
x_text = [0.4 0.58];
y_text = [0.7 0.7];
annotation('textarrow', x_text, y_text, 'String', 'Optimal Latency Range');
xa = [.84 .9];
ya = [.3 .3];
annotation('arrow', xa, ya);
% legend('Optimal TTI Limit', 'Location', 'best');
axis([0 1.1 -40 40]);

num_ul_symbols = 12;
ro_thresh_low = 0;
ro_thresh_high = 100;
moving_avg_frames = 1;
coeff_ma = ones(1, moving_avg_frames)/moving_avg_frames;

col_arr = ['m', 'k', 'b', 'g', 'c', 'y', 'r'];
marker_arr = ['*', 's', '+', 'o', '^', 'v', '>'];

% 1. Pilot RX by socket threads (= reference time), 
% 2. kPilotRX, 3. kProcessingStarted, 4. kPilotAllRX, 5. kFFTDone, 6. kZFDone,
% 7. kPrecodeDone, 8. kIFFTDone, 9. kEncodeDone, 10. kRXDone

% fid_4 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers4.txt');
% temp_4 = textscan(fid_4, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_4 = temp_4{1};   
% symbol_workers_4 = temp_4{2};
% ref_tsc_4 = temp_4{3};
% ref_us_4 = {4};
% first_rx_packet_tsc_4 = temp_4{5};
% last_cb_decode_tsc_4 = temp_4{6};
% diff_tsc_4 = temp_4{7};
% first_rx_packet_us_4 = temp_4{8};
% last_cb_decode_us_4 = temp_4{9};
% diff_us_4 = temp_4{10};
% processing_us_4 = temp_4{11};
% 
% diff_frame_us_4 = diff_us_4(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_4 = rmoutliers(diff_frame_us_4, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_4 = filter(coeff_ma, 1, ro_diff_frame_us_4);
% processing_frame_us_4 = processing_us_4(num_ul_symbols:num_ul_symbols:end);
% 
% fid_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers6.txt');
% temp_6 = textscan(fid_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_6 = temp_6{1};
% symbol_workers_6 = temp_6{2};
% ref_tsc_6 = temp_6{3};
% ref_us_6 = {4};
% first_rx_packet_tsc_6 = temp_6{5};
% last_cb_decode_tsc_6 = temp_6{6};
% diff_tsc_6 = temp_6{7};
% first_rx_packet_us_6 = temp_6{8};
% last_cb_decode_us_6 = temp_6{9};
% diff_us_6 = temp_6{10};
% processing_us_6 = temp_6{11};
% 
% diff_frame_us_6 = diff_us_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_6 = rmoutliers(diff_frame_us_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_6 = filter(coeff_ma, 1, ro_diff_frame_us_6);
% processing_frame_us_6 = processing_us_6(num_ul_symbols:num_ul_symbols:end);
% 
% fid_8 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers8.txt');
% temp_8 = textscan(fid_8, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_8 = temp_8{1};   
% symbol_workers_8 = temp_8{2};
% ref_tsc_8 = temp_8{3};
% ref_us_8 = {4};
% first_rx_packet_tsc_8 = temp_8{5};
% last_cb_decode_tsc_8 = temp_8{6};
% diff_tsc_8 = temp_8{7};
% first_rx_packet_us_8 = temp_8{8};
% last_cb_decode_us_8 = temp_8{9};
% diff_us_8 = temp_8{10};
% processing_us_8 = temp_8{11};
% 
% diff_frame_us_8 = diff_us_8(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_8 = rmoutliers(diff_frame_us_8, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_8 = filter(coeff_ma, 1, ro_diff_frame_us_8);
% processing_frame_us_8 = processing_us_8(num_ul_symbols:num_ul_symbols:end);
% 
% fid_10 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers10.txt');
% temp_10 = textscan(fid_10, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_10 = temp_10{1};   
% symbol_workers_10 = temp_10{2};
% ref_tsc_10 = temp_10{3};
% ref_us_10 = {4};
% first_rx_packet_tsc_10 = temp_10{5};
% last_cb_decode_tsc_10 = temp_10{6};
% diff_tsc_10 = temp_10{7};
% first_rx_packet_us_10 = temp_10{8};
% last_cb_decode_us_10 = temp_10{9};
% diff_us_10 = temp_10{10};
% processing_us_10 = temp_10{11};
% 
% diff_frame_us_10 = diff_us_10(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_10 = rmoutliers(diff_frame_us_10, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_10 = filter(coeff_ma, 1, ro_diff_frame_us_10);
% processing_frame_us_10 = processing_us_10(num_ul_symbols:num_ul_symbols:end);
% 
% fid_12 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers12.txt');
% temp_12 = textscan(fid_12, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_12 = temp_12{1};   
% symbol_workers_12 = temp_12{2};
% ref_tsc_12 = temp_12{3};
% ref_us_12 = {4};
% first_rx_packet_tsc_12 = temp_12{5};
% last_cb_decode_tsc_12 = temp_12{6};
% diff_tsc_12 = temp_12{7};
% first_rx_packet_us_12 = temp_12{8};
% last_cb_decode_us_12 = temp_12{9};
% diff_us_12 = temp_12{10};
% processing_us_12 = temp_12{11};
% 
% diff_frame_us_12 = diff_us_12(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_12 = rmoutliers(diff_frame_us_12, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_12 = filter(coeff_ma, 1, ro_diff_frame_us_12);
% processing_frame_us_12 = processing_us_12(num_ul_symbols:num_ul_symbols:end);
% 
% fid_14 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers14.txt');
% temp_14 = textscan(fid_14, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_14 = temp_14{1};   
% symbol_workers_14 = temp_14{2};
% ref_tsc_14 = temp_14{3};
% ref_us_14 = {4};
% first_rx_packet_tsc_14 = temp_14{5};
% last_cb_decode_tsc_14 = temp_14{6};
% diff_tsc_14 = temp_14{7};
% first_rx_packet_us_14 = temp_14{8};
% last_cb_decode_us_14 = temp_14{9};
% diff_us_14 = temp_14{10};
% processing_us_14 = temp_14{11};
% 
% diff_frame_us_14 = diff_us_14(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_14 = rmoutliers(diff_frame_us_14, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_14 = filter(coeff_ma, 1, ro_diff_frame_us_14);
% processing_frame_us_14 = processing_us_14(num_ul_symbols:num_ul_symbols:end);
% 
% fid_16 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers16.txt');
% temp_16 = textscan(fid_16, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_16 = temp_16{1};   
% symbol_workers_16 = temp_16{2};
% ref_tsc_16 = temp_16{3};
% ref_us_16 = {4};
% first_rx_packet_tsc_16 = temp_16{5};
% last_cb_decode_tsc_16 = temp_16{6};
% diff_tsc_16 = temp_16{7};
% first_rx_packet_us_16 = temp_16{8};
% last_cb_decode_us_16 = temp_16{9};
% diff_us_16 = temp_16{10};
% processing_us_16 = temp_16{11};
% 
% diff_frame_us_16 = diff_us_16(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_16 = rmoutliers(diff_frame_us_16, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_16 = filter(coeff_ma, 1, ro_diff_frame_us_16);
% processing_frame_us_16 = processing_us_16(num_ul_symbols:num_ul_symbols:end);
% 
% fid_18 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers18.txt');
% temp_18 = textscan(fid_18, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_18 = temp_18{1};   
% symbol_workers_18 = temp_18{2};
% ref_tsc_18 = temp_18{3};
% ref_us_18 = {4};
% first_rx_packet_tsc_18 = temp_18{5};
% last_cb_decode_tsc_18 = temp_18{6};
% diff_tsc_18 = temp_18{7};
% first_rx_packet_us_18 = temp_18{8};
% last_cb_decode_us_18 = temp_18{9};
% diff_us_18 = temp_18{10};
% processing_us_18 = temp_18{11};
% 
% diff_frame_us_18 = diff_us_18(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_18 = rmoutliers(diff_frame_us_18, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_18 = filter(coeff_ma, 1, ro_diff_frame_us_18);
% processing_frame_us_18 = processing_us_18(num_ul_symbols:num_ul_symbols:end);
% 
% figure;
% plot(ro_diff_frame_us_4);
% hold on;
% plot(ro_diff_frame_us_6);
% hold on;
% plot(ro_diff_frame_us_8);
% hold on;
% plot(ro_diff_frame_us_10);
% hold on;
% plot(ro_diff_frame_us_12);
% hold on;
% plot(ro_diff_frame_us_14);
% hold on;
% plot(ro_diff_frame_us_16);
% hold on;
% plot(ro_diff_frame_us_18);
% grid on;
% title("Symbol Latency (us)");
% xlabel("Frames");
% ylabel("Symbol Latency (us)");
% legend('64 BS Ants, 8 Users, 4 Cores', '64 BS Ants, 8 Users, 6 Cores', '64 BS Ants, 8 Users, 8 Cores', '64 BS Ants, 8 Users, 10 Cores', '64 BS Ants, 8 Users, 12 Cores', '64 BS Ants, 8 Users, 14 Cores', '64 BS Ants, 8 Users, 16 Cores', '64 BS Ants, 8 Users, 18 Cores', 'Location', 'best');
% 
% figure;
% plot(ma_ro_diff_frame_us_4);
% hold on;
% plot(ma_ro_diff_frame_us_6);
% hold on;
% plot(ma_ro_diff_frame_us_8);
% hold on;
% plot(ma_ro_diff_frame_us_10);
% hold on;
% plot(ma_ro_diff_frame_us_12);
% hold on;
% plot(ma_ro_diff_frame_us_14);
% hold on;
% plot(ma_ro_diff_frame_us_16);
% hold on;
% plot(ma_ro_diff_frame_us_18);
% grid on;
% title(['Moving Average of Symbol Latency for ', num2str(moving_avg_frames), ' Frames Moving Window']);
% xlabel("Frames");
% ylabel("Symbol Latency (us)");
% legend('64 BS Ants, 8 Users, 4 Cores', '64 BS Ants, 8 Users, 6 Cores', '64 BS Ants, 8 Users, 8 Cores', '64 BS Ants, 8 Users, 10 Cores', '64 BS Ants, 8 Users, 12 Cores', '64 BS Ants, 8 Users, 14 Cores', '64 BS Ants, 8 Users, 16 Cores', '64 BS Ants, 8 Users, 18 Cores', 'Location', 'best');
% 
% figure;
% [f_4, x_4] = ecdf(ro_diff_frame_us_4);
% plot(x_4, f_4, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(1));
% hold on;
% [f_6, x_6] = ecdf(ro_diff_frame_us_6);
% plot(x_6, f_6, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(2));
% hold on;
% [f_8, x_8] = ecdf(ro_diff_frame_us_8);
% plot(x_8, f_8, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(3));
% hold on;
% [f_10, x_10] = ecdf(ro_diff_frame_us_10);
% plot(x_10, f_10, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(4));
% hold on;
% [f_12, x_12] = ecdf(ro_diff_frame_us_12);
% plot(x_12, f_12, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(5));
% hold on;
% [f_14, x_14] = ecdf(ro_diff_frame_us_14);
% plot(x_14, f_14, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(6));
% hold on;
% [f_16, x_16] = ecdf(ro_diff_frame_us_16);
% plot(x_16, f_16, 'linewidth', 1, 'LineStyle', ':', 'color', col_arr(1));
% hold on;
% [f_16, x_16] = ecdf(ro_diff_frame_us_18);
% plot(x_16, f_16, 'linewidth', 1, 'LineStyle', ':', 'color', col_arr(2));
% hold on;
% line([1000, 1000], [0, 1], 'linewidth', 2, 'color', 'r');
% grid on;
% title("Empirical CDF of Frame Latency (one TTI = 1000us)");
% legend('64 BS Ants, 8 Users, 4 Cores', '64 BS Ants, 8 Users, 6 Cores', '64 BS Ants, 8 Users, 8 Cores', '64 BS Ants, 8 Users, 10 Cores', '64 BS Ants, 8 Users, 12 Cores', '64 BS Ants, 8 Users, 14 Cores', '64 BS Ants, 8 Users, 16 Cores', '64 BS Ants, 8 Users, 18 Cores', 'TTI Mark', 'Location', 'best');


% figure;
% plot(processing_frame_us_4);
% hold on;
% plot(processing_frame_us_6);
% hold on;
% plot(processing_frame_us_8);
% hold on;
% plot(processing_frame_us_10);
% hold on;
% plot(processing_frame_us_12);
% hold on;
% plot(processing_frame_us_14);
% hold on;
% plot(processing_frame_us_16);
% hold on;
% plot(processing_frame_us_18);
% grid on;
% xlabel("Frames");
% ylabel("Sum of Frame Processing Time of all Doers (us)");
% legend('4 Cores', '6 Cores', '8 Cores', '10 Cores', '12 Cores', '14 Cores', '16 Cores', '18 Cores', 'Location', 'best');
% 
% figure;
% cdfplot(processing_frame_us_4);
% hold on;
% cdfplot(processing_frame_us_6);
% hold on;
% cdfplot(processing_frame_us_8);
% hold on;
% cdfplot(processing_frame_us_10);
% hold on;
% cdfplot(processing_frame_us_12);
% hold on;
% cdfplot(processing_frame_us_14);
% hold on;
% cdfplot(processing_frame_us_16);
% hold on;
% cdfplot(processing_frame_us_18);
% grid on;
% title("Empirical CDF of Sum of Frame Processing Time of all Doers (us)");
% legend('4 Cores', '6 Cores', '8 Cores', '10 Cores', '12 Cores', '14 Cores', '16 Cores', '18 Cores', 'Location', 'best');






% fid_64_16_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant16_workers6.txt');
% temp_64_16_6 = textscan(fid_64_16_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_6 = temp_64_16_6{1};   
% symbol_workers_64_16_6 = temp_64_16_6{2};
% ref_tsc_64_16_6 = temp_64_16_6{3};
% ref_us_64_16_6 = {4};
% first_rx_packet_tsc_64_16_6 = temp_64_16_6{5};
% last_cb_decode_tsc_64_16_6 = temp_64_16_6{6};
% diff_tsc_64_16_6 = temp_64_16_6{7};
% first_rx_packet_us_64_16_6 = temp_64_16_6{8};
% last_cb_decode_us_64_16_6 = temp_64_16_6{9};
% diff_us_64_16_6 = temp_64_16_6{10};
% processing_us_64_16_6 = temp_64_16_6{11};
% 
% diff_frame_us_64_16_6 = diff_us_64_16_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_6 = rmoutliers(diff_frame_us_64_16_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_6);
% processing_frame_us_64_16_6 = processing_us_64_16_6(num_ul_symbols:num_ul_symbols:end);

% fid_64_16_7 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant16_workers7.txt');
% temp_64_16_7 = textscan(fid_64_16_7, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_7 = temp_64_16_7{1};   
% symbol_workers_64_16_7 = temp_64_16_7{2};
% ref_tsc_64_16_7 = temp_64_16_7{3};
% ref_us_64_16_7 = {4};
% first_rx_packet_tsc_64_16_7 = temp_64_16_7{5};
% last_cb_decode_tsc_64_16_7 = temp_64_16_7{6};
% diff_tsc_64_16_7 = temp_64_16_7{7};
% first_rx_packet_us_64_16_7 = temp_64_16_7{8};
% last_cb_decode_us_64_16_7 = temp_64_16_7{9};
% diff_us_64_16_7 = temp_64_16_7{10};
% processing_us_64_16_7 = temp_64_16_7{11};
% 
% diff_frame_us_64_16_7 = diff_us_64_16_7(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_7 = rmoutliers(diff_frame_us_64_16_7, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_7 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_7);
% processing_frame_us_64_16_7 = processing_us_64_16_7(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_16_8 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant16_workers8.txt');
% temp_64_16_8 = textscan(fid_64_16_8, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_8 = temp_64_16_8{1};   
% symbol_workers_64_16_8 = temp_64_16_8{2};
% ref_tsc_64_16_8 = temp_64_16_8{3};
% ref_us_64_16_8 = {4};
% first_rx_packet_tsc_64_16_8 = temp_64_16_8{5};
% last_cb_decode_tsc_64_16_8 = temp_64_16_8{6};
% diff_tsc_64_16_8 = temp_64_16_8{7};
% first_rx_packet_us_64_16_8 = temp_64_16_8{8};
% last_cb_decode_us_64_16_8 = temp_64_16_8{9};
% diff_us_64_16_8 = temp_64_16_8{10};
% processing_us_64_16_8 = temp_64_16_8{11};
% 
% diff_frame_us_64_16_8 = diff_us_64_16_8(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_8 = rmoutliers(diff_frame_us_64_16_8, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_8 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_8);
% processing_frame_us_64_16_8 = processing_us_64_16_8(num_ul_symbols:num_ul_symbols:end);
% 
% fid_64_16_9 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant16_workers9.txt');
% temp_64_16_9 = textscan(fid_64_16_9, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_9 = temp_64_16_9{1};   
% symbol_workers_64_16_9 = temp_64_16_9{2};
% ref_tsc_64_16_9 = temp_64_16_9{3};
% ref_us_64_16_9 = {4};
% first_rx_packet_tsc_64_16_9 = temp_64_16_9{5};
% last_cb_decode_tsc_64_16_9 = temp_64_16_9{6};
% diff_tsc_64_16_9 = temp_64_16_9{7};
% first_rx_packet_us_64_16_9 = temp_64_16_9{8};
% last_cb_decode_us_64_16_9 = temp_64_16_9{9};
% diff_us_64_16_9 = temp_64_16_9{10};
% processing_us_64_16_9 = temp_64_16_9{11};
% 
% diff_frame_us_64_16_9 = diff_us_64_16_9(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_9 = rmoutliers(diff_frame_us_64_16_9, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_9 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_9);
% processing_frame_us_64_16_9 = processing_us_64_16_9(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_16_10 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant16_workers10.txt');
% temp_64_16_10 = textscan(fid_64_16_10, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_10 = temp_64_16_10{1};   
% symbol_workers_64_16_10 = temp_64_16_10{2};
% ref_tsc_64_16_10 = temp_64_16_10{3};
% ref_us_64_16_10 = {4};
% first_rx_packet_tsc_64_16_10 = temp_64_16_10{5};
% last_cb_decode_tsc_64_16_10 = temp_64_16_10{6};
% diff_tsc_64_16_10 = temp_64_16_10{7};
% first_rx_packet_us_64_16_10 = temp_64_16_10{8};
% last_cb_decode_us_64_16_10 = temp_64_16_10{9};
% diff_us_64_16_10 = temp_64_16_10{10};
% processing_us_64_16_10 = temp_64_16_10{11};
% 
% diff_frame_us_64_16_10 = diff_us_64_16_10(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_10 = rmoutliers(diff_frame_us_64_16_10, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_10 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_10);
% processing_frame_us_64_16_10 = processing_us_64_16_10(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_16_11 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant16_workers11.txt');
% temp_64_16_11 = textscan(fid_64_16_11, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_11 = temp_64_16_11{1};   
% symbol_workers_64_16_11 = temp_64_16_11{2};
% ref_tsc_64_16_11 = temp_64_16_11{3};
% ref_us_64_16_11 = {4};
% first_rx_packet_tsc_64_16_11 = temp_64_16_11{5};
% last_cb_decode_tsc_64_16_11 = temp_64_16_11{6};
% diff_tsc_64_16_11 = temp_64_16_11{7};
% first_rx_packet_us_64_16_11 = temp_64_16_11{8};
% last_cb_decode_us_64_16_11 = temp_64_16_11{9};
% diff_us_64_16_11 = temp_64_16_11{10};
% processing_us_64_16_11 = temp_64_16_11{11};
% 
% diff_frame_us_64_16_11 = diff_us_64_16_11(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_11 = rmoutliers(diff_frame_us_64_16_11, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_11 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_11);
% processing_frame_us_64_16_11 = processing_us_64_16_11(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_16_12 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant16_workers12.txt');
% temp_64_16_12 = textscan(fid_64_16_12, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_12 = temp_64_16_12{1};   
% symbol_workers_64_16_12 = temp_64_16_12{2};
% ref_tsc_64_16_12 = temp_64_16_12{3};
% ref_us_64_16_12 = {4};
% first_rx_packet_tsc_64_16_12 = temp_64_16_12{5};
% last_cb_decode_tsc_64_16_12 = temp_64_16_12{6};
% diff_tsc_64_16_12 = temp_64_16_12{7};
% first_rx_packet_us_64_16_12 = temp_64_16_12{8};
% last_cb_decode_us_64_16_12 = temp_64_16_12{9};
% diff_us_64_16_12 = temp_64_16_12{10};
% processing_us_64_16_12 = temp_64_16_12{11};
% 
% diff_frame_us_64_16_12 = diff_us_64_16_12(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_12 = rmoutliers(diff_frame_us_64_16_12, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_12 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_12);
% processing_frame_us_64_16_12 = processing_us_64_16_12(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_16_13 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant16_workers13.txt');
% temp_64_16_13 = textscan(fid_64_16_13, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_13 = temp_64_16_13{1};   
% symbol_workers_64_16_13 = temp_64_16_13{2};
% ref_tsc_64_16_13 = temp_64_16_13{3};
% ref_us_64_16_13 = {4};
% first_rx_packet_tsc_64_16_13 = temp_64_16_13{5};
% last_cb_decode_tsc_64_16_13 = temp_64_16_13{6};
% diff_tsc_64_16_13 = temp_64_16_13{7};
% first_rx_packet_us_64_16_13 = temp_64_16_13{8};
% last_cb_decode_us_64_16_13 = temp_64_16_13{9};
% diff_us_64_16_13 = temp_64_16_13{10};
% processing_us_64_16_13 = temp_64_16_13{11};
% 
% diff_frame_us_64_16_13 = diff_us_64_16_13(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_13 = rmoutliers(diff_frame_us_64_16_13, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_13 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_13);
% processing_frame_us_64_16_13 = processing_us_64_16_13(num_ul_symbols:num_ul_symbols:end);
% 
% figure;
% [f_64_16_7, x_64_16_7] = ecdf(ro_diff_frame_us_64_16_7);
% plot(x_64_16_7, f_64_16_7, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(1));
% hold on;
% [f_64_16_8, x_64_16_8] = ecdf(ro_diff_frame_us_64_16_8);
% plot(x_64_16_8, f_64_16_8, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(2));
% hold on;
% [f_64_16_9, x_64_16_9] = ecdf(ro_diff_frame_us_64_16_9);
% plot(x_64_16_9, f_64_16_9, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(3));
% hold on;
% [f_64_16_10, x_64_16_10] = ecdf(ro_diff_frame_us_64_16_10);
% plot(x_64_16_10, f_64_16_10, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(4));
% hold on;
% [f_64_16_11, x_64_16_11] = ecdf(ro_diff_frame_us_64_16_11);
% plot(x_64_16_11, f_64_16_11, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(5));
% hold on;
% [f_64_16_12, x_64_16_12] = ecdf(ro_diff_frame_us_64_16_12);
% plot(x_64_16_12, f_64_16_12, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(6));
% hold on;
% [f_64_16_13, x_64_16_13] = ecdf(ro_diff_frame_us_64_16_13);
% plot(x_64_16_13, f_64_16_13, 'linewidth', 1, 'LineStyle', ':', 'color', col_arr(1));
% hold on;
% line([1000, 1000], [0, 1], 'linewidth', 2, 'color', 'r');
% grid on;
% title("Empirical CDF of Frame Latency (one TTI = 1000us)");
% legend('64 BS Ants, 16 Users, 7 Cores', '64 BS Ants, 16 Users, 8 Cores', '64 BS Ants, 16 Users, 9 Cores', '64 BS Ants, 16 Users, 10 Cores', '64 BS Ants, 16 Users, 11 Cores', '64 BS Ants, 16 Users, 12 Cores', '64 BS Ants, 16 Users, 13 Cores', 'TTI Mark', 'Location', 'best');
% 
% 
% fid_64_2_4 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant2_workers4.txt');
% temp_64_2_4 = textscan(fid_64_2_4, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_2_4 = temp_64_2_4{1};   
% symbol_workers_64_2_4 = temp_64_2_4{2};
% ref_tsc_64_2_4 = temp_64_2_4{3};
% ref_us_64_2_4 = {4};
% first_rx_packet_tsc_64_2_4 = temp_64_2_4{5};
% last_cb_decode_tsc_64_2_4 = temp_64_2_4{6};
% diff_tsc_64_2_4 = temp_64_2_4{7};
% first_rx_packet_us_64_2_4 = temp_64_2_4{8};
% last_cb_decode_us_64_2_4 = temp_64_2_4{9};
% diff_us_64_2_4 = temp_64_2_4{10};
% processing_us_64_2_4 = temp_64_2_4{11};
% 
% diff_frame_us_64_2_4 = diff_us_64_2_4(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_2_4 = rmoutliers(diff_frame_us_64_2_4, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_2_4 = filter(coeff_ma, 1, ro_diff_frame_us_64_2_4);
% processing_frame_us_64_2_4 = processing_us_64_2_4(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_4_4 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_workers4.txt');
% temp_64_4_4 = textscan(fid_64_4_4, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_4 = temp_64_4_4{1};   
% symbol_workers_64_4_4 = temp_64_4_4{2};
% ref_tsc_64_4_4 = temp_64_4_4{3};
% ref_us_64_4_4 = {4};
% first_rx_packet_tsc_64_4_4 = temp_64_4_4{5};
% last_cb_decode_tsc_64_4_4 = temp_64_4_4{6};
% diff_tsc_64_4_4 = temp_64_4_4{7};
% first_rx_packet_us_64_4_4 = temp_64_4_4{8};
% last_cb_decode_us_64_4_4 = temp_64_4_4{9};
% diff_us_64_4_4 = temp_64_4_4{10};
% processing_us_64_4_4 = temp_64_4_4{11};
% 
% diff_frame_us_64_4_4 = diff_us_64_4_4(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_4 = rmoutliers(diff_frame_us_64_4_4, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_4 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_4);
% processing_frame_us_64_4_4 = processing_us_64_4_4(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_6_4 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant6_workers4.txt');
% temp_64_6_4 = textscan(fid_64_6_4, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_6_4 = temp_64_6_4{1};   
% symbol_workers_64_6_4 = temp_64_6_4{2};
% ref_tsc_64_6_4 = temp_64_6_4{3};
% ref_us_64_6_4 = {4};
% first_rx_packet_tsc_64_6_4 = temp_64_6_4{5};
% last_cb_decode_tsc_64_6_4 = temp_64_6_4{6};
% diff_tsc_64_6_4 = temp_64_6_4{7};
% first_rx_packet_us_64_6_4 = temp_64_6_4{8};
% last_cb_decode_us_64_6_4 = temp_64_6_4{9};
% diff_us_64_6_4 = temp_64_6_4{10};
% processing_us_64_6_4 = temp_64_6_4{11};
% 
% diff_frame_us_64_6_4 = diff_us_64_6_4(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_6_4 = rmoutliers(diff_frame_us_64_6_4, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_6_4 = filter(coeff_ma, 1, ro_diff_frame_us_64_6_4);
% processing_frame_us_64_6_4 = processing_us_64_6_4(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_8_4 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers4.txt');
% temp_64_8_4 = textscan(fid_64_8_4, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_8_4 = temp_64_8_4{1};   
% symbol_workers_64_8_4 = temp_64_8_4{2};
% ref_tsc_64_8_4 = temp_64_8_4{3};
% ref_us_64_8_4 = {4};
% first_rx_packet_tsc_64_8_4 = temp_64_8_4{5};
% last_cb_decode_tsc_64_8_4 = temp_64_8_4{6};
% diff_tsc_64_8_4 = temp_64_8_4{7};
% first_rx_packet_us_64_8_4 = temp_64_8_4{8};
% last_cb_decode_us_64_8_4 = temp_64_8_4{9};
% diff_us_64_8_4 = temp_64_8_4{10};
% processing_us_64_8_4 = temp_64_8_4{11};
% 
% diff_frame_us_64_8_4 = diff_us_64_8_4(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_8_4 = rmoutliers(diff_frame_us_64_8_4, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_8_4 = filter(coeff_ma, 1, ro_diff_frame_us_64_8_4);
% processing_frame_us_64_8_4 = processing_us_64_8_4(num_ul_symbols:num_ul_symbols:end);
% 
% figure;
% [f_64_2_4, x_64_2_4] = ecdf(ro_diff_frame_us_64_2_4);
% plot(x_64_2_4, f_64_2_4, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(1));
% hold on;
% [f_64_4_4, x_64_4_4] = ecdf(ro_diff_frame_us_64_4_4);
% plot(x_64_4_4, f_64_4_4, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(2));
% hold on;
% [f_64_6_4, x_64_6_4] = ecdf(ro_diff_frame_us_64_6_4);
% plot(x_64_6_4, f_64_6_4, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(3));
% hold on;
% [f_64_8_4, x_64_8_4] = ecdf(ro_diff_frame_us_64_8_4);
% plot(x_64_8_4, f_64_8_4, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(4));
% hold on;
% line([1000, 1000], [0, 1], 'linewidth', 2, 'color', 'r');
% grid on;
% title("Empirical CDF of Frame Latency (one TTI = 1000us)");
% legend('64 BS Ants, 2 Users, 4 Cores', '64 BS Ants, 4 Users, 4 Cores', '64 BS Ants, 6 Users, 4 Cores', '64 BS Ants, 8 Users, 4 Cores', 'TTI Mark', 'Location', 'best');
% 
% 
% 
% fid_64_2_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant2_workers6.txt');
% temp_64_2_6 = textscan(fid_64_2_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_2_6 = temp_64_2_6{1};   
% symbol_workers_64_2_6 = temp_64_2_6{2};
% ref_tsc_64_2_6 = temp_64_2_6{3};
% ref_us_64_2_6 = {4};
% first_rx_packet_tsc_64_2_6 = temp_64_2_6{5};
% last_cb_decode_tsc_64_2_6 = temp_64_2_6{6};
% diff_tsc_64_2_6 = temp_64_2_6{7};
% first_rx_packet_us_64_2_6 = temp_64_2_6{8};
% last_cb_decode_us_64_2_6 = temp_64_2_6{9};
% diff_us_64_2_6 = temp_64_2_6{10};
% processing_us_64_2_6 = temp_64_2_6{11};
% 
% diff_frame_us_64_2_6 = diff_us_64_2_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_2_6 = rmoutliers(diff_frame_us_64_2_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_2_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_2_6);
% processing_frame_us_64_2_6 = processing_us_64_2_6(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_4_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_workers6.txt');
% temp_64_4_6 = textscan(fid_64_4_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_6 = temp_64_4_6{1};   
% symbol_workers_64_4_6 = temp_64_4_6{2};
% ref_tsc_64_4_6 = temp_64_4_6{3};
% ref_us_64_4_6 = {4};
% first_rx_packet_tsc_64_4_6 = temp_64_4_6{5};
% last_cb_decode_tsc_64_4_6 = temp_64_4_6{6};
% diff_tsc_64_4_6 = temp_64_4_6{7};
% first_rx_packet_us_64_4_6 = temp_64_4_6{8};
% last_cb_decode_us_64_4_6 = temp_64_4_6{9};
% diff_us_64_4_6 = temp_64_4_6{10};
% processing_us_64_4_6 = temp_64_4_6{11};
% 
% diff_frame_us_64_4_6 = diff_us_64_4_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_6 = rmoutliers(diff_frame_us_64_4_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_6);
% processing_frame_us_64_4_6 = processing_us_64_4_6(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_6_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant6_workers6.txt');
% temp_64_6_6 = textscan(fid_64_6_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_6_6 = temp_64_6_6{1};   
% symbol_workers_64_6_6 = temp_64_6_6{2};
% ref_tsc_64_6_6 = temp_64_6_6{3};
% ref_us_64_6_6 = {4};
% first_rx_packet_tsc_64_6_6 = temp_64_6_6{5};
% last_cb_decode_tsc_64_6_6 = temp_64_6_6{6};
% diff_tsc_64_6_6 = temp_64_6_6{7};
% first_rx_packet_us_64_6_6 = temp_64_6_6{8};
% last_cb_decode_us_64_6_6 = temp_64_6_6{9};
% diff_us_64_6_6 = temp_64_6_6{10};
% processing_us_64_6_6 = temp_64_6_6{11};
% 
% diff_frame_us_64_6_6 = diff_us_64_6_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_6_6 = rmoutliers(diff_frame_us_64_6_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_6_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_6_6);
% processing_frame_us_64_6_6 = processing_us_64_6_6(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_8_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers6.txt');
% temp_64_8_6 = textscan(fid_64_8_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_8_6 = temp_64_8_6{1};   
% symbol_workers_64_8_6 = temp_64_8_6{2};
% ref_tsc_64_8_6 = temp_64_8_6{3};
% ref_us_64_8_6 = {4};
% first_rx_packet_tsc_64_8_6 = temp_64_8_6{5};
% last_cb_decode_tsc_64_8_6 = temp_64_8_6{6};
% diff_tsc_64_8_6 = temp_64_8_6{7};
% first_rx_packet_us_64_8_6 = temp_64_8_6{8};
% last_cb_decode_us_64_8_6 = temp_64_8_6{9};
% diff_us_64_8_6 = temp_64_8_6{10};
% processing_us_64_8_6 = temp_64_8_6{11};
% 
% diff_frame_us_64_8_6 = diff_us_64_8_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_8_6 = rmoutliers(diff_frame_us_64_8_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_8_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_8_6);
% processing_frame_us_64_8_6 = processing_us_64_8_6(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_10_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant10_workers6.txt');
% temp_64_10_6 = textscan(fid_64_10_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_10_6 = temp_64_10_6{1};   
% symbol_workers_64_10_6 = temp_64_10_6{2};
% ref_tsc_64_10_6 = temp_64_10_6{3};
% ref_us_64_10_6 = {4};
% first_rx_packet_tsc_64_10_6 = temp_64_10_6{5};
% last_cb_decode_tsc_64_10_6 = temp_64_10_6{6};
% diff_tsc_64_10_6 = temp_64_10_6{7};
% first_rx_packet_us_64_10_6 = temp_64_10_6{8};
% last_cb_decode_us_64_10_6 = temp_64_10_6{9};
% diff_us_64_10_6 = temp_64_10_6{10};
% processing_us_64_10_6 = temp_64_10_6{11};
% 
% diff_frame_us_64_10_6 = diff_us_64_10_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_10_6 = rmoutliers(diff_frame_us_64_10_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_10_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_10_6);
% processing_frame_us_64_10_6 = processing_us_64_10_6(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_12_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant12_workers6.txt');
% temp_64_12_6 = textscan(fid_64_12_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_12_6 = temp_64_12_6{1};   
% symbol_workers_64_12_6 = temp_64_12_6{2};
% ref_tsc_64_12_6 = temp_64_12_6{3};
% ref_us_64_12_6 = {4};
% first_rx_packet_tsc_64_12_6 = temp_64_12_6{5};
% last_cb_decode_tsc_64_12_6 = temp_64_12_6{6};
% diff_tsc_64_12_6 = temp_64_12_6{7};
% first_rx_packet_us_64_12_6 = temp_64_12_6{8};
% last_cb_decode_us_64_12_6 = temp_64_12_6{9};
% diff_us_64_12_6 = temp_64_12_6{10};
% processing_us_64_12_6 = temp_64_12_6{11};
% 
% diff_frame_us_64_12_6 = diff_us_64_12_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_12_6 = rmoutliers(diff_frame_us_64_12_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_12_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_12_6);
% processing_frame_us_64_12_6 = processing_us_64_12_6(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_14_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant14_workers6.txt');
% temp_64_14_6 = textscan(fid_64_14_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_14_6 = temp_64_14_6{1};   
% symbol_workers_64_14_6 = temp_64_14_6{2};
% ref_tsc_64_14_6 = temp_64_14_6{3};
% ref_us_64_14_6 = {4};
% first_rx_packet_tsc_64_14_6 = temp_64_14_6{5};
% last_cb_decode_tsc_64_14_6 = temp_64_14_6{6};
% diff_tsc_64_14_6 = temp_64_14_6{7};
% first_rx_packet_us_64_14_6 = temp_64_14_6{8};
% last_cb_decode_us_64_14_6 = temp_64_14_6{9};
% diff_us_64_14_6 = temp_64_14_6{10};
% processing_us_64_14_6 = temp_64_14_6{11};
% 
% diff_frame_us_64_14_6 = diff_us_64_14_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_14_6 = rmoutliers(diff_frame_us_64_14_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_14_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_14_6);
% processing_frame_us_64_14_6 = processing_us_64_14_6(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_16_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant16_workers6.txt');
% temp_64_16_6 = textscan(fid_64_16_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_6 = temp_64_16_6{1};   
% symbol_workers_64_16_6 = temp_64_16_6{2};
% ref_tsc_64_16_6 = temp_64_16_6{3};
% ref_us_64_16_6 = {4};
% first_rx_packet_tsc_64_16_6 = temp_64_16_6{5};
% last_cb_decode_tsc_64_16_6 = temp_64_16_6{6};
% diff_tsc_64_16_6 = temp_64_16_6{7};
% first_rx_packet_us_64_16_6 = temp_64_16_6{8};
% last_cb_decode_us_64_16_6 = temp_64_16_6{9};
% diff_us_64_16_6 = temp_64_16_6{10};
% processing_us_64_16_6 = temp_64_16_6{11};
% 
% diff_frame_us_64_16_6 = diff_us_64_16_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_6 = rmoutliers(diff_frame_us_64_16_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_6);
% processing_frame_us_64_16_6 = processing_us_64_16_6(num_ul_symbols:num_ul_symbols:end);
% 
% 
% figure;
% [f_64_2_6, x_64_2_6] = ecdf(ro_diff_frame_us_64_2_6);
% plot(x_64_2_6, f_64_2_6, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(1));
% hold on;
% [f_64_4_6, x_64_4_6] = ecdf(ro_diff_frame_us_64_4_6);
% plot(x_64_4_6, f_64_4_6, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(2));
% hold on;
% [f_64_6_6, x_64_6_6] = ecdf(ro_diff_frame_us_64_6_6);
% plot(x_64_6_6, f_64_6_6, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(3));
% hold on;
% [f_64_8_6, x_64_8_6] = ecdf(ro_diff_frame_us_64_8_6);
% plot(x_64_8_6, f_64_8_6, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(4));
% % hold on;
% % [f_64_10_6, x_64_10_6] = ecdf(ro_diff_frame_us_64_10_6);
% % plot(x_64_10_6, f_64_10_6, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(5));
% % hold on;
% % [f_64_12_6, x_64_12_6] = ecdf(ro_diff_frame_us_64_12_6);
% % plot(x_64_12_6, f_64_12_6, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(6));
% hold on;
% line([1000, 1000], [0, 1], 'linewidth', 2, 'color', 'r');
% grid on;
% title("Empirical CDF of Frame Latency (one TTI = 1000us)");
% legend('64 BS Ants, 2 Users, 6 Cores', '64 BS Ants, 4 Users, 6 Cores', '64 BS Ants, 6 Users, 6 Cores', '64 BS Ants, 8 Users, 6 Cores', 'TTI Mark', 'Location', 'best');
% 
% 
% 
% fid_64_2_8 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant2_workers8.txt');
% temp_64_2_8 = textscan(fid_64_2_8, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_2_8 = temp_64_2_8{1};   
% symbol_workers_64_2_8 = temp_64_2_8{2};
% ref_tsc_64_2_8 = temp_64_2_8{3};
% ref_us_64_2_8 = {4};
% first_rx_packet_tsc_64_2_8 = temp_64_2_8{5};
% last_cb_decode_tsc_64_2_8 = temp_64_2_8{6};
% diff_tsc_64_2_8 = temp_64_2_8{7};
% first_rx_packet_us_64_2_8 = temp_64_2_8{8};
% last_cb_decode_us_64_2_8 = temp_64_2_8{9};
% diff_us_64_2_8 = temp_64_2_8{10};
% processing_us_64_2_8 = temp_64_2_8{11};
% 
% diff_frame_us_64_2_8 = diff_us_64_2_8(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_2_8 = rmoutliers(diff_frame_us_64_2_8, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_2_8 = filter(coeff_ma, 1, ro_diff_frame_us_64_2_8);
% processing_frame_us_64_2_8 = processing_us_64_2_8(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_4_8 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_workers8.txt');
% temp_64_4_8 = textscan(fid_64_4_8, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_8 = temp_64_4_8{1};   
% symbol_workers_64_4_8 = temp_64_4_8{2};
% ref_tsc_64_4_8 = temp_64_4_8{3};
% ref_us_64_4_8 = {4};
% first_rx_packet_tsc_64_4_8 = temp_64_4_8{5};
% last_cb_decode_tsc_64_4_8 = temp_64_4_8{6};
% diff_tsc_64_4_8 = temp_64_4_8{7};
% first_rx_packet_us_64_4_8 = temp_64_4_8{8};
% last_cb_decode_us_64_4_8 = temp_64_4_8{9};
% diff_us_64_4_8 = temp_64_4_8{10};
% processing_us_64_4_8 = temp_64_4_8{11};
% 
% diff_frame_us_64_4_8 = diff_us_64_4_8(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_8 = rmoutliers(diff_frame_us_64_4_8, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_8 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_8);
% processing_frame_us_64_4_8 = processing_us_64_4_8(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_6_8 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant6_workers8.txt');
% temp_64_6_8 = textscan(fid_64_6_8, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_6_8 = temp_64_6_8{1};   
% symbol_workers_64_6_8 = temp_64_6_8{2};
% ref_tsc_64_6_8 = temp_64_6_8{3};
% ref_us_64_6_8 = {4};
% first_rx_packet_tsc_64_6_8 = temp_64_6_8{5};
% last_cb_decode_tsc_64_6_8 = temp_64_6_8{6};
% diff_tsc_64_6_8 = temp_64_6_8{7};
% first_rx_packet_us_64_6_8 = temp_64_6_8{8};
% last_cb_decode_us_64_6_8 = temp_64_6_8{9};
% diff_us_64_6_8 = temp_64_6_8{10};
% processing_us_64_6_8 = temp_64_6_8{11};
% 
% diff_frame_us_64_6_8 = diff_us_64_6_8(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_6_8 = rmoutliers(diff_frame_us_64_6_8, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_6_8 = filter(coeff_ma, 1, ro_diff_frame_us_64_6_8);
% processing_frame_us_64_6_8 = processing_us_64_6_8(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_8_8 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant8_workers8.txt');
% temp_64_8_8 = textscan(fid_64_8_8, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_8_8 = temp_64_8_8{1};   
% symbol_workers_64_8_8 = temp_64_8_8{2};
% ref_tsc_64_8_8 = temp_64_8_8{3};
% ref_us_64_8_8 = {4};
% first_rx_packet_tsc_64_8_8 = temp_64_8_8{5};
% last_cb_decode_tsc_64_8_8 = temp_64_8_8{6};
% diff_tsc_64_8_8 = temp_64_8_8{7};
% first_rx_packet_us_64_8_8 = temp_64_8_8{8};
% last_cb_decode_us_64_8_8 = temp_64_8_8{9};
% diff_us_64_8_8 = temp_64_8_8{10};
% processing_us_64_8_8 = temp_64_8_8{11};
% 
% diff_frame_us_64_8_8 = diff_us_64_8_8(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_8_8 = rmoutliers(diff_frame_us_64_8_8, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_8_8 = filter(coeff_ma, 1, ro_diff_frame_us_64_8_8);
% processing_frame_us_64_8_8 = processing_us_64_8_8(num_ul_symbols:num_ul_symbols:end);
% 
% figure;
% [f_64_2_8, x_64_2_8] = ecdf(ro_diff_frame_us_64_2_8);
% plot(x_64_2_8, f_64_2_8, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(1));
% hold on;
% [f_64_4_8, x_64_4_8] = ecdf(ro_diff_frame_us_64_4_8);
% plot(x_64_4_8, f_64_4_8, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(2));
% hold on;
% [f_64_6_8, x_64_6_8] = ecdf(ro_diff_frame_us_64_6_8);
% plot(x_64_6_8, f_64_6_8, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(3));
% hold on;
% [f_64_8_8, x_64_8_8] = ecdf(ro_diff_frame_us_64_8_8);
% plot(x_64_8_8, f_64_8_8, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(4));
% hold on;
% line([1000, 1000], [0, 1], 'linewidth', 2, 'color', 'r');
% grid on;
% title("Empirical CDF of Frame Latency (one TTI = 1000us)");
% legend('64 BS Ants, 2 Users, 8 Cores', '64 BS Ants, 4 Users, 8 Cores', '64 BS Ants, 6 Users, 8 Cores', '64 BS Ants, 8 Users, 8 Cores', 'TTI Mark', 'Location', 'best');
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% 
% fid_64_4_10_3 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul10_workers3.txt');
% temp_64_4_10_3 = textscan(fid_64_4_10_3, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_10_3 = temp_64_4_10_3{1};   
% symbol_workers_64_4_10_3 = temp_64_4_10_3{2};
% ref_tsc_64_4_10_3 = temp_64_4_10_3{3};
% ref_us_64_4_10_3 = {4};
% first_rx_packet_tsc_64_4_10_3 = temp_64_4_10_3{5};
% last_cb_decode_tsc_64_4_10_3 = temp_64_4_10_3{6};
% diff_tsc_64_4_10_3 = temp_64_4_10_3{7};
% first_rx_packet_us_64_4_10_3 = temp_64_4_10_3{8};
% last_cb_decode_us_64_4_10_3 = temp_64_4_10_3{9};
% diff_us_64_4_10_3 = temp_64_4_10_3{10};
% processing_us_64_4_10_3 = temp_64_4_10_3{11};
% 
% diff_frame_us_64_4_10_3 = diff_us_64_4_10_3(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_10_3 = rmoutliers(diff_frame_us_64_4_10_3, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_10_3 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_10_3);
% processing_frame_us_64_4_10_3 = processing_us_64_4_10_3(num_ul_symbols:num_ul_symbols:end);
% 
% fid_64_4_10_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul10_workers6.txt');
% temp_64_4_10_6 = textscan(fid_64_4_10_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_10_6 = temp_64_4_10_6{1};   
% symbol_workers_64_4_10_6 = temp_64_4_10_6{2};
% ref_tsc_64_4_10_6 = temp_64_4_10_6{3};
% ref_us_64_4_10_6 = {4};
% first_rx_packet_tsc_64_4_10_6 = temp_64_4_10_6{5};
% last_cb_decode_tsc_64_4_10_6 = temp_64_4_10_6{6};
% diff_tsc_64_4_10_6 = temp_64_4_10_6{7};
% first_rx_packet_us_64_4_10_6 = temp_64_4_10_6{8};
% last_cb_decode_us_64_4_10_6 = temp_64_4_10_6{9};
% diff_us_64_4_10_6 = temp_64_4_10_6{10};
% processing_us_64_4_10_6 = temp_64_4_10_6{11};
% 
% diff_frame_us_64_4_10_6 = diff_us_64_4_10_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_10_6 = rmoutliers(diff_frame_us_64_4_10_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_10_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_10_6);
% processing_frame_us_64_4_10_6 = processing_us_64_4_10_6(num_ul_symbols:num_ul_symbols:end);
% 
% fid_64_4_10_9 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul10_workers9.txt');
% temp_64_4_10_9 = textscan(fid_64_4_10_9, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_10_9 = temp_64_4_10_9{1};   
% symbol_workers_64_4_10_9 = temp_64_4_10_9{2};
% ref_tsc_64_4_10_9 = temp_64_4_10_9{3};
% ref_us_64_4_10_9 = {4};
% first_rx_packet_tsc_64_4_10_9 = temp_64_4_10_9{5};
% last_cb_decode_tsc_64_4_10_9 = temp_64_4_10_9{6};
% diff_tsc_64_4_10_9 = temp_64_4_10_9{7};
% first_rx_packet_us_64_4_10_9 = temp_64_4_10_9{8};
% last_cb_decode_us_64_4_10_9 = temp_64_4_10_9{9};
% diff_us_64_4_10_9 = temp_64_4_10_9{10};
% processing_us_64_4_10_9 = temp_64_4_10_9{11};
% 
% diff_frame_us_64_4_10_9 = diff_us_64_4_10_9(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_10_9 = rmoutliers(diff_frame_us_64_4_10_9, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_10_9 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_10_9);
% processing_frame_us_64_4_10_9 = processing_us_64_4_10_9(num_ul_symbols:num_ul_symbols:end);
% 
% fid_64_4_10_12 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul10_workers12.txt');
% temp_64_4_10_12 = textscan(fid_64_4_10_12, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_10_12 = temp_64_4_10_12{1};   
% symbol_workers_64_4_10_12 = temp_64_4_10_12{2};
% ref_tsc_64_4_10_12 = temp_64_4_10_12{3};
% ref_us_64_4_10_12 = {4};
% first_rx_packet_tsc_64_4_10_12 = temp_64_4_10_12{5};
% last_cb_decode_tsc_64_4_10_12 = temp_64_4_10_12{6};
% diff_tsc_64_4_10_12 = temp_64_4_10_12{7};
% first_rx_packet_us_64_4_10_12 = temp_64_4_10_12{8};
% last_cb_decode_us_64_4_10_12 = temp_64_4_10_12{9};
% diff_us_64_4_10_12 = temp_64_4_10_12{10};
% processing_us_64_4_10_12 = temp_64_4_10_12{11};
% 
% diff_frame_us_64_4_10_12 = diff_us_64_4_10_12(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_10_12 = rmoutliers(diff_frame_us_64_4_10_12, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_10_12 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_10_12);
% processing_frame_us_64_4_10_12 = processing_us_64_4_10_12(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_4_10_15 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul10_workers15.txt');
% temp_64_4_10_15 = textscan(fid_64_4_10_15, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_10_15 = temp_64_4_10_15{1};   
% symbol_workers_64_4_10_15 = temp_64_4_10_15{2};
% ref_tsc_64_4_10_15 = temp_64_4_10_15{3};
% ref_us_64_4_10_15 = {4};
% first_rx_packet_tsc_64_4_10_15 = temp_64_4_10_15{5};
% last_cb_decode_tsc_64_4_10_15 = temp_64_4_10_15{6};
% diff_tsc_64_4_10_15 = temp_64_4_10_15{7};
% first_rx_packet_us_64_4_10_15 = temp_64_4_10_15{8};
% last_cb_decode_us_64_4_10_15 = temp_64_4_10_15{9};
% diff_us_64_4_10_15 = temp_64_4_10_15{10};
% processing_us_64_4_10_15 = temp_64_4_10_15{11};
% 
% diff_frame_us_64_4_10_15 = diff_us_64_4_10_15(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_10_15 = rmoutliers(diff_frame_us_64_4_10_15, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_10_15 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_10_15);
% processing_frame_us_64_4_10_15 = processing_us_64_4_10_15(num_ul_symbols:num_ul_symbols:end);
% 
% figure;
% [f_64_4_10_3, x_64_4_10_3] = ecdf(ro_diff_frame_us_64_4_10_3);
% plot(x_64_4_10_3, f_64_4_10_3, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(1));
% hold on;
% [f_64_4_10_6, x_64_4_10_6] = ecdf(ro_diff_frame_us_64_4_10_6);
% plot(x_64_4_10_6, f_64_4_10_6, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(2));
% hold on;
% [f_64_4_10_9, x_64_4_10_9] = ecdf(ro_diff_frame_us_64_4_10_9);
% plot(x_64_4_10_9, f_64_4_10_9, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(3));
% hold on;
% [f_64_4_10_12, x_64_4_10_12] = ecdf(ro_diff_frame_us_64_4_10_12);
% plot(x_64_4_10_12, f_64_4_10_12, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(4));
% hold on;
% [f_64_4_10_15, x_64_4_10_15] = ecdf(ro_diff_frame_us_64_4_10_15);
% plot(x_64_4_10_15, f_64_4_10_15, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(5));
% hold on;
% line([1000, 1000], [0, 1], 'linewidth', 2, 'color', 'r');
% grid on;
% title("Empirical CDF of a TTI Latency (one TTI = 1000us)");
% legend('64 BS Ants, 4 Spatial Streams, 10 MCS Index UL, 3 Cores', '64 BS Ants, 4 Spatial Streams, 10 MCS Index UL, 6 Cores', '64 BS Ants, 4 Spatial Streams, 10 MCS Index UL, 9 Cores', '64 BS Ants, 4 Spatial Streams, 10 MCS Index UL, 12 Cores', '64 BS Ants, 4 Spatial Streams, 10 MCS Index UL, 15 Cores', 'TTI Mark', 'Location', 'best');
% 
% 
% 
% fid_64_4_16_3 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul16_workers3.txt');
% temp_64_4_16_3 = textscan(fid_64_4_16_3, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_16_3 = temp_64_4_16_3{1};   
% symbol_workers_64_4_16_3 = temp_64_4_16_3{2};
% ref_tsc_64_4_16_3 = temp_64_4_16_3{3};
% ref_us_64_4_16_3 = {4};
% first_rx_packet_tsc_64_4_16_3 = temp_64_4_16_3{5};
% last_cb_decode_tsc_64_4_16_3 = temp_64_4_16_3{6};
% diff_tsc_64_4_16_3 = temp_64_4_16_3{7};
% first_rx_packet_us_64_4_16_3 = temp_64_4_16_3{8};
% last_cb_decode_us_64_4_16_3 = temp_64_4_16_3{9};
% diff_us_64_4_16_3 = temp_64_4_16_3{10};
% processing_us_64_4_16_3 = temp_64_4_16_3{11};
% 
% diff_frame_us_64_4_16_3 = diff_us_64_4_16_3(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_16_3 = rmoutliers(diff_frame_us_64_4_16_3, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_16_3 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_16_3);
% processing_frame_us_64_4_16_3 = processing_us_64_4_16_3(num_ul_symbols:num_ul_symbols:end);
% 
% fid_64_4_16_6 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul16_workers6.txt');
% temp_64_4_16_6 = textscan(fid_64_4_16_6, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_16_6 = temp_64_4_16_6{1};   
% symbol_workers_64_4_16_6 = temp_64_4_16_6{2};
% ref_tsc_64_4_16_6 = temp_64_4_16_6{3};
% ref_us_64_4_16_6 = {4};
% first_rx_packet_tsc_64_4_16_6 = temp_64_4_16_6{5};
% last_cb_decode_tsc_64_4_16_6 = temp_64_4_16_6{6};
% diff_tsc_64_4_16_6 = temp_64_4_16_6{7};
% first_rx_packet_us_64_4_16_6 = temp_64_4_16_6{8};
% last_cb_decode_us_64_4_16_6 = temp_64_4_16_6{9};
% diff_us_64_4_16_6 = temp_64_4_16_6{10};
% processing_us_64_4_16_6 = temp_64_4_16_6{11};
% 
% diff_frame_us_64_4_16_6 = diff_us_64_4_16_6(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_16_6 = rmoutliers(diff_frame_us_64_4_16_6, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_16_6 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_16_6);
% processing_frame_us_64_4_16_6 = processing_us_64_4_16_6(num_ul_symbols:num_ul_symbols:end);
% 
% fid_64_4_16_9 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul16_workers9.txt');
% temp_64_4_16_9 = textscan(fid_64_4_16_9, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_16_9 = temp_64_4_16_9{1};   
% symbol_workers_64_4_16_9 = temp_64_4_16_9{2};
% ref_tsc_64_4_16_9 = temp_64_4_16_9{3};
% ref_us_64_4_16_9 = {4};
% first_rx_packet_tsc_64_4_16_9 = temp_64_4_16_9{5};
% last_cb_decode_tsc_64_4_16_9 = temp_64_4_16_9{6};
% diff_tsc_64_4_16_9 = temp_64_4_16_9{7};
% first_rx_packet_us_64_4_16_9 = temp_64_4_16_9{8};
% last_cb_decode_us_64_4_16_9 = temp_64_4_16_9{9};
% diff_us_64_4_16_9 = temp_64_4_16_9{10};
% processing_us_64_4_16_9 = temp_64_4_16_9{11};
% 
% diff_frame_us_64_4_16_9 = diff_us_64_4_16_9(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_16_9 = rmoutliers(diff_frame_us_64_4_16_9, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_16_9 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_16_9);
% processing_frame_us_64_4_16_9 = processing_us_64_4_16_9(num_ul_symbols:num_ul_symbols:end);
% 
% fid_64_4_16_12 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul16_workers12.txt');
% temp_64_4_16_12 = textscan(fid_64_4_16_12, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_16_12 = temp_64_4_16_12{1};   
% symbol_workers_64_4_16_12 = temp_64_4_16_12{2};
% ref_tsc_64_4_16_12 = temp_64_4_16_12{3};
% ref_us_64_4_16_12 = {4};
% first_rx_packet_tsc_64_4_16_12 = temp_64_4_16_12{5};
% last_cb_decode_tsc_64_4_16_12 = temp_64_4_16_12{6};
% diff_tsc_64_4_16_12 = temp_64_4_16_12{7};
% first_rx_packet_us_64_4_16_12 = temp_64_4_16_12{8};
% last_cb_decode_us_64_4_16_12 = temp_64_4_16_12{9};
% diff_us_64_4_16_12 = temp_64_4_16_12{10};
% processing_us_64_4_16_12 = temp_64_4_16_12{11};
% 
% diff_frame_us_64_4_16_12 = diff_us_64_4_16_12(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_16_12 = rmoutliers(diff_frame_us_64_4_16_12, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_16_12 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_16_12);
% processing_frame_us_64_4_16_12 = processing_us_64_4_16_12(num_ul_symbols:num_ul_symbols:end);
% 
% 
% fid_64_4_16_15 = fopen('../../files/config/experiment/timeresult_symbol_bsant64_ueant4_mcsul16_workers15.txt');
% temp_64_4_16_15 = textscan(fid_64_4_16_15, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_16_15 = temp_64_4_16_15{1};
% symbol_workers_64_4_16_15 = temp_64_4_16_15{2};
% ref_tsc_64_4_16_15 = temp_64_4_16_15{3};
% ref_us_64_4_16_15 = {4};
% first_rx_packet_tsc_64_4_16_15 = temp_64_4_16_15{5};
% last_cb_decode_tsc_64_4_16_15 = temp_64_4_16_15{6};
% diff_tsc_64_4_16_15 = temp_64_4_16_15{7};
% first_rx_packet_us_64_4_16_15 = temp_64_4_16_15{8};
% last_cb_decode_us_64_4_16_15 = temp_64_4_16_15{9};
% diff_us_64_4_16_15 = temp_64_4_16_15{10};
% processing_us_64_4_16_15 = temp_64_4_16_15{11};
% 
% diff_frame_us_64_4_16_15 = diff_us_64_4_16_15(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_16_15 = rmoutliers(diff_frame_us_64_4_16_15, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_16_15 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_16_15);
% processing_frame_us_64_4_16_15 = processing_us_64_4_16_15(num_ul_symbols:num_ul_symbols:end);
% 
% figure;
% [f_64_4_16_3, x_64_4_16_3] = ecdf(ro_diff_frame_us_64_4_16_3);
% plot(x_64_4_16_3, f_64_4_16_3, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(1));
% hold on;
% [f_64_4_16_6, x_64_4_16_6] = ecdf(ro_diff_frame_us_64_4_16_6);
% plot(x_64_4_16_6, f_64_4_16_6, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(2));
% hold on;
% [f_64_4_16_9, x_64_4_16_9] = ecdf(ro_diff_frame_us_64_4_16_9);
% plot(x_64_4_16_9, f_64_4_16_9, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(3));
% hold on;
% [f_64_4_16_12, x_64_4_16_12] = ecdf(ro_diff_frame_us_64_4_16_12);
% plot(x_64_4_16_12, f_64_4_16_12, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(4));
% hold on;
% [f_64_4_16_15, x_64_4_16_15] = ecdf(ro_diff_frame_us_64_4_16_15);
% plot(x_64_4_16_15, f_64_4_16_15, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(5));
% hold on;
% line([1000, 1000], [0, 1], 'linewidth', 2, 'color', 'r');
% grid on;
% title("Empirical CDF of a TTI Latency (one TTI = 1000us)");
% legend('64 BS Ants, 4 Spatial Streams, 16 MCS Index UL, 3 Cores', '64 BS Ants, 4 Spatial Streams, 16 MCS Index UL, 6 Cores', '64 BS Ants, 4 Spatial Streams, 16 MCS Index UL, 9 Cores', '64 BS Ants, 4 Spatial Streams, 16 MCS Index UL, 12 Cores', '64 BS Ants, 4 Spatial Streams, 16 MCS Index UL, 15 Cores', 'TTI Mark', 'Location', 'best');


% num_bs_ants = 64;
% num_ss = [4, 8, 12, 16];
% mcs_index = [10, 16, 17, 25];
% num_cores = [3, 6, 9, 12, 15];
% 
% for ii = 1:length(num_ss)
%     for jj = 1:length(mcs_index)
%         figure;
%         legend_details = cell(length(num_cores),1);
%         for kk = 1:length(num_cores)
%             fid = fopen(['../../files/config/experiment/timeresult_symbol_bsant' num2str(num_bs_ants) '_ueant' num2str(num_ss(ii)) '_mcsul' num2str(mcs_index(jj)) '_workers' num2str(num_cores(kk)) '.txt']);
%             temp = textscan(fid, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
%             frame = temp{1};
%             symbol_workers = temp{2};
%             ref_tsc = temp{3};
%             ref_us = {4};
%             first_rx_packet_tsc = temp{5};
%             last_cb_decode_tsc = temp{6};
%             diff_tsc = temp{7};
%             first_rx_packet_us = temp{8};
%             last_cb_decode_us = temp{9};
%             diff_us = temp{10};
%             processing_us = temp{11};
% 
%             diff_frame_us = diff_us(num_ul_symbols:num_ul_symbols:end);
%             ro_diff_frame_us = rmoutliers(diff_frame_us, "percentiles", [ro_thresh_low ro_thresh_high]);
%             ma_ro_diff_frame_us = filter(coeff_ma, 1, ro_diff_frame_us);
%             processing_frame_us = processing_us(num_ul_symbols:num_ul_symbols:end);
% 
%             percentile_50(jj, kk) = prctile(ma_ro_diff_frame_us, 90);
%             [f, x] = ecdf(ma_ro_diff_frame_us);
%             plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(kk));
%             hold on;
%             legend_details{kk} = [num2str(num_bs_ants) ' BS Ants, ' num2str(num_ss(ii)) ' Spatial Streams, ' num2str(mcs_index(jj)) ' MCS Index UL, ' num2str(num_cores(kk)) ' Cores'];
%             fclose(fid);
%         end
%         hold on;
%         line([700, 700], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'b');
%         legend_details{kk+1} = 'Optimal Lower Latency Limit (70% of TTI)';
%         hold on;
%         line([800, 800], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g');        
%         legend_details{kk+2} = 'Optimal Higher Latency Limit (80% of TTI)';
%         hold on;
%         line([1000, 1000], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'r');        
%         legend_details{kk+3} = 'TTI Duration (1000 us)';
%         grid on;
%         legend(legend_details, 'Location', 'best');
%     end
% 
%     figure;
%     surf(num_cores, mcs_index, percentile_50);
%     title([num2str(num_ss(ii)), ' Spatial Streams']);
%     xlabel('Number of CPU Cores');
%     ylabel('MCS Index Index');
%     zlabel('PHY Processing Time (us)');
%     colorbar;
% end
% 
% 
% config_comb(1,:,:) = [4, 10, 3;
%                       4, 10, 6;
%                       4, 10, 9;
%                       4, 10, 12;
%                       4, 10, 15];
% 
% config_comb(2,:,:) = [12, 25, 3;
%                       12, 25, 6;
%                       12, 25, 9;
%                       12, 25, 12;
%                       12, 25, 15];
% 
% config_comb(3,:,:) = [ 4, 17, 3;
%                        4, 17, 6;
%                        4, 17, 9;
%                        4, 17, 12;
%                        4, 17, 15];
% 
% config_comb(4,:,:) = [16, 10, 3;
%                       16, 10, 6;
%                       16, 10, 9;
%                       16, 10, 12;
%                       16, 10, 15];
% 
% config_comb(5,:,:) = [8, 17, 3;
%                       8, 17, 6;
%                       8, 17, 9;
%                       8, 17, 12;
%                       8, 17, 15];
% 
% [num_profiles, num_configs, num_params] = size(config_comb);
% 
% for ii = 1:num_profiles
%     figure;
%     legend_details = cell(num_configs, 1);
%     for jj = 1:num_configs
%         fid = fopen(['../../files/config/experiment/timeresult_symbol_bsant' num2str(num_bs_ants) '_ueant' num2str(config_comb(ii, jj, 1)) '_mcsul' num2str(config_comb(ii, jj, 2)) '_workers' num2str(config_comb(ii, jj, 3)) '.txt']);
%         temp = textscan(fid, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
%         frame = temp{1};
%         symbol_workers = temp{2};
%         ref_tsc = temp{3};
%         ref_us = {4};
%         first_rx_packet_tsc = temp{5};
%         last_cb_decode_tsc = temp{6};
%         diff_tsc = temp{7};
%         first_rx_packet_us = temp{8};
%         last_cb_decode_us = temp{9};
%         diff_us = temp{10};
%         processing_us = temp{11};
% 
%         diff_frame_us = diff_us(num_ul_symbols:num_ul_symbols:end);
%         ro_diff_frame_us = rmoutliers(diff_frame_us, "percentiles", [ro_thresh_low ro_thresh_high]);
%         ma_ro_diff_frame_us = filter(coeff_ma, 1, ro_diff_frame_us);
%         processing_frame_us = processing_us(num_ul_symbols:num_ul_symbols:end);
% 
%         percentile_50(ii, jj) = prctile(ma_ro_diff_frame_us, 50);
%         [f, x] = ecdf(ma_ro_diff_frame_us);
%         plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(jj));
%         hold on;
%         legend_details{jj} = [num2str(num_bs_ants) ' BS Ants, ' num2str(config_comb(ii, jj, 1)) ' Spatial Streams, ' num2str(config_comb(ii, jj, 2)) ' MCS Index UL, ' num2str(config_comb(ii, jj, 3)) ' Cores'];
%         fclose(fid);
%     end
%     grid on;
%     legend(legend_details, 'Location', 'best');
% end
% 
% figure;
% for ii = 1:num_profiles
%     cpu_cores = config_comb(ii, :, 3);
%     plot(cpu_cores, percentile_50(ii, :), 'LineWidth', 1, 'LineStyle', '-', 'Marker', marker_arr(ii), 'Color', col_arr(ii));
%     hold on;
% end
% xlabel('Number of CPU Cores');
% ylabel('PHY Processing Time (us) at 50th Percentile of CDF');
% grid on;
% legend('64 BS Ants, 4 Spatial Streams, 10 MCS Index UL', '64 BS Ants, 12 Spatial Streams, 25 MCS Index UL', '64 BS Ants, 4 Spatial Streams, 17 MCS Index UL', '64 BS Ants, 16 Spatial Streams, 10 MCS Index UL', '64 BS Ants, 8 Spatial Streams, 17 MCS Index UL');

% fid_64_4_10_3 = fopen(['../../files/config/experiment/timeresult_symbol_bsant' num2str(num_bs_ants) '_ueant' num2str(num_ss(1)) '_mcsul' num2str(mcs_index(1)) '_workers' num2str(num_cores(1)) '.txt']);
% temp_64_4_10_3 = textscan(fid_64_4_10_3, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_4_10_3 = temp_64_4_10_3{1};
% symbol_workers_64_4_10_3 = temp_64_4_10_3{2};
% ref_tsc_64_4_10_3 = temp_64_4_10_3{3};
% ref_us_64_4_10_3 = {4};
% first_rx_packet_tsc_64_4_10_3 = temp_64_4_10_3{5};
% last_cb_decode_tsc_64_4_10_3 = temp_64_4_10_3{6};
% diff_tsc_64_4_10_3 = temp_64_4_10_3{7};
% first_rx_packet_us_64_4_10_3 = temp_64_4_10_3{8};
% last_cb_decode_us_64_4_10_3 = temp_64_4_10_3{9};
% diff_us_64_4_10_3 = temp_64_4_10_3{10};
% processing_us_64_4_10_3 = temp_64_4_10_3{11};
% 
% diff_frame_us_64_4_10_3 = diff_us_64_4_10_3(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_4_10_3 = rmoutliers(diff_frame_us_64_4_10_3, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_4_10_3 = filter(coeff_ma, 1, ro_diff_frame_us_64_4_10_3);
% processing_frame_us_64_4_10_3 = processing_us_64_4_10_3(num_ul_symbols:num_ul_symbols:end);
% 
% percentile_64_4_10_3 = prctile(ro_diff_frame_us_64_4_10_3, 90);
% [f, x] = ecdf(ro_diff_frame_us_64_4_10_3);
% figure;
% plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(1));
% hold on;
% legend_details{1} = [num2str(num_bs_ants) ' BS Ants, ' num2str(num_ss(1)) ' Spatial Streams, ' num2str(mcs_index(1)) ' MCS Index UL, ' num2str(num_cores(1)) ' Cores'];
% fclose(fid_64_4_10_3);
% 
% fid_64_16_25_12 = fopen(['../../files/config/experiment/timeresult_symbol_bsant' num2str(num_bs_ants) '_ueant' num2str(num_ss(4)) '_mcsul' num2str(mcs_index(4)) '_workers' num2str(num_cores(4)) '.txt']);
% temp_64_16_25_12 = textscan(fid_64_16_25_12, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame_64_16_25_12 = temp_64_16_25_12{1};
% symbol_workers_64_16_25_12 = temp_64_16_25_12{2};
% ref_tsc_64_16_25_12 = temp_64_16_25_12{3};
% ref_us_64_16_25_12 = {4};
% first_rx_packet_tsc_64_16_25_12 = temp_64_16_25_12{5};
% last_cb_decode_tsc_64_16_25_12 = temp_64_16_25_12{6};
% diff_tsc_64_16_25_12 = temp_64_16_25_12{7};
% first_rx_packet_us_64_16_25_12 = temp_64_16_25_12{8};
% last_cb_decode_us_64_16_25_12 = temp_64_16_25_12{9};
% diff_us_64_16_25_12 = temp_64_16_25_12{10};
% processing_us_64_16_25_12 = temp_64_16_25_12{11};
% 
% diff_frame_us_64_16_25_12 = diff_us_64_16_25_12(num_ul_symbols:num_ul_symbols:end);
% ro_diff_frame_us_64_16_25_12 = rmoutliers(diff_frame_us_64_16_25_12, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us_64_16_25_12 = filter(coeff_ma, 1, ro_diff_frame_us_64_16_25_12);
% processing_frame_us_64_16_25_12 = processing_us_64_16_25_12(num_ul_symbols:num_ul_symbols:end);
% 
% percentile_64_16_25_12 = prctile(ro_diff_frame_us_64_16_25_12, 90);
% [f, x] = ecdf(ro_diff_frame_us_64_16_25_12);
% hold on;
% plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(2));
% hold on;
% legend_details{2} = [num2str(num_bs_ants) ' BS Ants, ' num2str(num_ss(4)) ' Spatial Streams, ' num2str(mcs_index(4)) ' MCS Index UL, ' num2str(num_cores(4)) ' Cores'];
% fclose(fid_64_16_25_12);
% 
% 
% grid on;
% legend(legend_details, 'Location', 'best');





% kk = 1;
% fid = fopen(['../../files/config/experiment/timeresult_symbol_bsant64_ueant16_mcsul25_snrdb26_ldpciter5_workers16.txt']);
% temp = textscan(fid, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame = temp{1};
% symbol_workers = temp{2};
% ref_tsc = temp{3};
% ref_us = {4};
% first_rx_packet_tsc = temp{5};
% last_cb_decode_tsc = temp{6};
% diff_tsc = temp{7};
% first_rx_packet_us = temp{8};
% last_cb_decode_us = temp{9};
% diff_us = temp{10};
% processing_us = temp{11};
% 
% diff_frame_us = diff_us(num_ul_symbols-11:num_ul_symbols:end);
% % diff_frame_us = diff_us;
% ro_diff_frame_us = rmoutliers(diff_frame_us, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us = filter(coeff_ma, 1, ro_diff_frame_us);
% processing_frame_us = processing_us(num_ul_symbols:num_ul_symbols:end);
% 
% [f, x] = ecdf(ma_ro_diff_frame_us);
% figure;
% plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(kk));
% hold on;
% legend_details{kk} = ['64 BS Ants ', '16 Spatial Streams ', '25 MCS Index UL ', '26 SNR dB ', '5 Iterations ', '16 Cores'];
% fclose(fid);
% 
% kk = 2;
% fid = fopen(['../../files/config/experiment/timeresult_symbol_bsant64_ueant16_mcsul25_snrdb26_ldpciter30_workers16.txt']);
% temp = textscan(fid, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame = temp{1};
% symbol_workers = temp{2};
% ref_tsc = temp{3};
% ref_us = {4};
% first_rx_packet_tsc = temp{5};
% last_cb_decode_tsc = temp{6};
% diff_tsc = temp{7};
% first_rx_packet_us = temp{8};
% last_cb_decode_us = temp{9};
% diff_us = temp{10};
% processing_us = temp{11};
% 
% diff_frame_us = diff_us(num_ul_symbols-11:num_ul_symbols:end);
% % diff_frame_us = diff_us;
% ro_diff_frame_us = rmoutliers(diff_frame_us, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us = filter(coeff_ma, 1, ro_diff_frame_us);
% processing_frame_us = processing_us(num_ul_symbols:num_ul_symbols:end);
% 
% [f, x] = ecdf(ma_ro_diff_frame_us);
% hold on;
% plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(kk));
% hold on;
% legend_details{kk} = ['64 BS Ants ', '16 Spatial Streams ', '25 MCS Index UL ', '26 SNR dB ', '30 Iterations ', '16 Cores'];
% fclose(fid);
% 
% % hold on;
% % line([700, 700], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'b');
% % legend_details{kk+1} = 'Optimal Lower Latency Limit (70% of TTI)';
% % hold on;
% % line([800, 800], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g');
% % legend_details{kk+2} = 'Optimal Higher Latency Limit (80% of TTI)';
% hold on;
% line([1000, 1000], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'r');
% legend_details{kk+1} = 'TTI Duration (1000 us)';
% grid on;
% legend(legend_details, 'Location', 'best');




% figure;
% kk = 1;
% fid = fopen(['../../files/config/experiment/timeresult_symbol_bsant64_ueant16_mcsul25_snrdb26_ldpciter30_workers16.txt']);
% temp = textscan(fid, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame = temp{1};
% symbol_workers = temp{2};
% ref_tsc = temp{3};
% ref_us = {4};
% first_rx_packet_tsc = temp{5};
% last_cb_decode_tsc = temp{6};
% diff_tsc = temp{7};
% first_rx_packet_us = temp{8};
% last_cb_decode_us = temp{9};
% diff_us = temp{10};
% processing_us = temp{11};
% fclose(fid);
% 
% diff_frame_us = diff_us(num_ul_symbols-11:num_ul_symbols:end);
% % diff_frame_us = diff_us;
% ro_diff_frame_us = rmoutliers(diff_frame_us, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us = filter(coeff_ma, 1, ro_diff_frame_us);
% processing_frame_us = processing_us(num_ul_symbols:num_ul_symbols:end);
% 
% [f, x] = ecdf(ma_ro_diff_frame_us);
% subplot(1,2,1);
% plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(kk));
% hold on;
% line([850, 850], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'b');
% hold on;
% line([900, 900], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g');
% hold on;
% line([1000, 1000], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'r');
% x_text = [0.23 0.33];
% y_text = [0.2 0.2];
% annotation('arrow', x_text, y_text);
% x_text = [0.4 0.32];
% y_text = [0.9 0.9];
% annotation('arrow', x_text, y_text);grid on;
% legend('Actual Processing Time', 'Location', 'best');
% xlabel('TTI Processing Time (us)');
% ylabel('CDF');
% axis([700 1000 0 1]);
% 
% 
% kk = 1;
% fid = fopen(['../../files/config/experiment/timeresult_symbol_bsant64_ueant16_mcsul25_snrdb26_ldpciter5_workers16.txt']);
% temp = textscan(fid, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
% frame = temp{1};
% symbol_workers = temp{2};
% ref_tsc = temp{3};
% ref_us = {4};
% first_rx_packet_tsc = temp{5};
% last_cb_decode_tsc = temp{6};
% diff_tsc = temp{7};
% first_rx_packet_us = temp{8};
% last_cb_decode_us = temp{9};
% diff_us = temp{10};
% processing_us = temp{11};
% fclose(fid);
% 
% diff_frame_us = diff_us(num_ul_symbols:num_ul_symbols:end) + 320;
% % diff_frame_us = diff_us;
% ro_diff_frame_us = rmoutliers(diff_frame_us, "percentiles", [ro_thresh_low ro_thresh_high]);
% ma_ro_diff_frame_us = filter(coeff_ma, 1, ro_diff_frame_us);
% processing_frame_us = processing_us(num_ul_symbols:num_ul_symbols:end);
% 
% [f, x] = ecdf(ma_ro_diff_frame_us);
% subplot(1,2,2);
% plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(kk));
% grid on;
% hold on;
% line([850, 850], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'b');
% hold on;
% line([900, 900], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'g');
% hold on;
% line([1000, 1000], [0, 1], 'LineWidth', 2, 'LineStyle', '--', 'Color', 'r');
% legend('Target Processing Time', 'Location', 'best');
% xlabel('TTI Processing Time (us)');
% ylabel('CDF');
% axis([700 1000 0 1]);



num_bs_ants = 64;
users_array = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16];
min_users_index = 1;
max_users_index = numel(users_array);
mcsul_array = [25];
snrdb_array = [40];
ldpciter_aray = [5];
cores_array = [3, 4, 5, 6, 7, 8, 9, 10, 11];
min_cores_index = 1;
max_cores_index = numel(cores_array);
cb_size = 5632;
symbol_to_check = num_ul_symbols-11;
lower_latency_limit = 0.7 * 1000;
higher_latency_limit = 0.8 * 1000;

kk = 1;
for ii = 1:numel(users_array)
    for jj = 1:numel(cores_array)
        fid = fopen(['../../files/config/experiment/timeresult_symbol_bsant' num2str(num_bs_ants) '_ueant' num2str(users_array(ii)) '_mcsul' num2str(mcsul_array(1)) '_snrdb' num2str(snrdb_array(1)) '_ldpciter' num2str(ldpciter_aray(1)) '_workers' num2str(cores_array(jj)) '.txt']);
        temp = textscan(fid, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
        frame = temp{1};
        symbol_workers = temp{2};
        ref_tsc = temp{3};
        ref_us = {4};
        first_rx_packet_tsc = temp{5};
        last_cb_decode_tsc = temp{6};
        diff_tsc = temp{7};
        first_rx_packet_us = temp{8};
        last_cb_decode_us = temp{9};
        diff_us = temp{10};
        processing_us = temp{11};
        fclose(fid);

        diff_frame_us = diff_us(symbol_to_check:num_ul_symbols:end);
        % diff_frame_us = diff_us;
        ro_diff_frame_us = rmoutliers(diff_frame_us, "percentiles", [ro_thresh_low ro_thresh_high]);
        ma_ro_diff_frame_us = filter(coeff_ma, 1, ro_diff_frame_us);
        percentile_50(ii, jj) = prctile(ma_ro_diff_frame_us, 50);
        percentile_95(ii, jj) = prctile(ma_ro_diff_frame_us, 95);

        actual_tput(ii, jj) = cb_size * users_array(ii) / 1e-3;
    end
end

updated_users_tput = actual_tput;
updated_cores_tput = actual_tput;
for ii = 1:numel(users_array)
    for jj = 1:numel(cores_array)
        updated_users_latency(ii, jj) = percentile_50(ii, jj);
        updated_users(ii, jj) = ii;
        users_index = ii;
        check_flag = true;
        if percentile_50(users_index, jj) >= higher_latency_limit
            while check_flag == true
                users_index = users_index - 1;
                if users_index < min_users_index
                    users_index = min_users_index;
                    check_flag = false;
                end
                if percentile_50(users_index, jj) < higher_latency_limit
                    check_flag = false;
                end
                updated_users_latency(ii, jj) = percentile_50(users_index, jj);
                updated_users(ii, jj) = users_array(users_index);
                updated_users_tput(ii, jj) = cb_size * users_array(users_index) / 1e-3;
            end
        else
            while check_flag == true
                users_index = users_index + 1;
                if users_index > max_users_index
                    users_index = max_users_index;
                    check_flag = false;
                end
                if percentile_50(users_index, jj) >= lower_latency_limit
                    check_flag = false;
                end
                updated_users_latency(ii, jj) = percentile_50(users_index, jj);
                updated_users(ii, jj) = users_array(users_index);
                updated_users_tput(ii, jj) = cb_size * users_array(users_index) / 1e-3;
            end
        end

        updated_cores_latency(ii, jj) = percentile_50(ii, jj);
        updated_cores(ii, jj) = cores_array(jj);
        cores_index = jj;
        check_flag = true;
        if percentile_50(ii, cores_index) >= higher_latency_limit
            while check_flag == true
                cores_index = cores_index + 1;
                if cores_index > max_cores_index
                    cores_index = max_cores_index;
                    check_flag = false;
                end
                if percentile_50(ii, cores_index) < higher_latency_limit
                    check_flag = false;
                end
                updated_cores_latency(ii, jj) = percentile_50(ii, cores_index);
                updated_cores(ii, jj) = cores_array(cores_index);
            end
        else
            while check_flag == true
                cores_index = cores_index - 1;
                if cores_index < min_cores_index
                    cores_index = min_cores_index;
                    check_flag = false;
                end
                if percentile_50(ii, cores_index) >= lower_latency_limit
                    check_flag = false;
                end
                updated_cores_latency(ii, jj) = percentile_50(ii, cores_index);
                updated_cores(ii, jj) = cores_array(cores_index);
            end
        end
    end
end

figure;
surf(cores_array, users_array, percentile_50);
xlabel('Cores');
ylabel('Users');
zlabel('Actual Latency(us)');

% figure;
% surf(cores_array, users_array, updated_users_latency);
% xlabel('Cores');
% ylabel('Users');
% zlabel('Updated Users Latency(us)');

% figure;
% surf(cores_array, users_array, updated_cores_latency);
% xlabel('Cores');
% ylabel('Users');
% zlabel('Updated Cores Latency(us)');

% figure;
% surf(cores_array, users_array, actual_tput);
% xlabel('Cores');
% ylabel('Users');
% zlabel('Actual Throughput');

% figure;
% surf(cores_array, users_array, updated_users);
% xlabel('Cores');
% ylabel('Users');
% zlabel('Users Updated');

% figure;
% surf(cores_array, users_array, updated_cores);
% xlabel('Cores');
% ylabel('Users');
% zlabel('Cores Updated');

% figure;
% surf(cores_array, users_array, updated_users_tput);
% xlabel('Cores');
% ylabel('Users');
% zlabel('Users Updated Throughput (bps)');



% [f, x] = ecdf(ma_ro_diff_frame_us);
% figure;
% plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(kk));
% hold on;
% legend_details{1} = [num2str(num_bs_ants) ' BS Ants, ' num2str(num_ss(1)) ' Spatial Streams, ' num2str(mcs_index(1)) ' MCS Index UL, ' num2str(num_cores(1)) ' Cores'];




kk = 1;
fid = fopen(['../../files/config/experiment/timeresult_symbol_bsant64_ueant16_mcsul25_snrdb40_ldpciter5_workers10.txt']);
temp = textscan(fid, "%d %d %d %f %d %d %d %f %f %f %f", 'HeaderLines', 1);
frame = temp{1};
symbol_workers = temp{2};
ref_tsc = temp{3};
ref_us = {4};
first_rx_packet_tsc = temp{5};
last_cb_decode_tsc = temp{6};
diff_tsc = temp{7};
first_rx_packet_us = temp{8};
last_cb_decode_us = temp{9};
diff_us = temp{10};
processing_us = temp{11};

diff_frame_us = diff_us(num_ul_symbols-0:num_ul_symbols:end);
% diff_frame_us = diff_us;
ro_diff_frame_us = rmoutliers(diff_frame_us, "percentiles", [ro_thresh_low ro_thresh_high]);
ma_ro_diff_frame_us = filter(coeff_ma, 1, ro_diff_frame_us);
processing_frame_us = processing_us(num_ul_symbols:num_ul_symbols:end);

[f, x] = ecdf(ma_ro_diff_frame_us);
figure;
plot(x, f, 'linewidth', 1, 'LineStyle', '-', 'color', col_arr(kk));
grid on;
xlabel('TTI Processing Time (us)');
ylabel('CDF');
axis([0 1000 0 1]);
legend_details{kk} = ['64 BS Ants ', '16 Spatial Streams ', '25 MCS Index UL ', '40 SNR dB ', '5 Iterations ', '10 Cores'];
legend(legend_details, 'Location', 'best');
fclose(fid);

mean_latency = mean(ma_ro_diff_frame_us)
std_latency = std(ma_ro_diff_frame_us)
samp_size = ceil(((100*1.96*std_latency)/(5*mean_latency))^2)
q3 = prctile(ma_ro_diff_frame_us, 75);
q1 = prctile(ma_ro_diff_frame_us, 25);
iqr = q3-q1;
lb = q1-1.5*iqr
hb = q3+1.5*iqr