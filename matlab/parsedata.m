fid = fopen('timeresult.txt');
% fid = fopen('result/timeresult_bigstation_newsender_smp.txt');
% fid = fopen('result/timeresult_bigstation3.txt');
%  fid = fopen('result_newsender/timeresult_25core.txt');
% fid = fopen('result/timeresult_25core_cmp_bigstation_newsender_smp.txt');
% fid = fopen('result_bench_overhead/timeresult_30core_30task_2RX.txt');
% fid = fopen('result_compare/timeresult_23core_bigstation.txt');
% fid = fopen('result_scale_ant/timeresult_ant16_21core.txt');
temp = textscan(fid,"%f%f%f%f%f%f%f%f%f%f%f%f");
pilot_recv=temp{1};
rx_processed = temp{2};
fft_processed = temp{3};
zf_processed = temp{4};
demul_processed = temp{5};
frame_starts = [];
frame_starts(1,:) = temp{9};
frame_starts(2,:) = temp{10};
frame_start = min(frame_starts);
frame_start = frame_start.';

demul_duration = demul_processed-zf_processed;
zf_duration = zf_processed-fft_processed;
fft_duration = fft_processed-frame_start;
frame_duration_rx = rx_processed-frame_start;%diff(pilot_recv);
frame_duration_fft = fft_processed-frame_start;
frame_duration_demul = demul_processed-frame_start;
frame_duration_zf = zf_processed-frame_start;
frame_count = length(demul_processed);
frame_count = frame_count - 1;

% demul_duration = demul_processed-zf_processed;
% zf_duration = zf_processed-fft_processed;
% fft_duration = fft_processed-pilot_recv;
% frame_duration_rx = rx_processed-pilot_recv;%diff(pilot_recv);
% frame_duration_fft = fft_processed-pilot_recv;
% frame_duration_demul = demul_processed-pilot_recv;
% frame_duration_zf = zf_processed-pilot_recv;
% frame_count = length(demul_processed);
% frame_count = frame_count - 1;

fft_time_in_function = temp{6};
zf_time_in_function = temp{7};
demul_time_in_function = temp{8};

index = [1:frame_count];
index(1:512:frame_count) = [];
subset = index(1200:end);


fprintf("Average delay between rx thread and main thread: %.2f\n", ...
    mean(pilot_recv(subset)-frame_start(subset)));

fprintf("In functions: FFT: %.2f, ZF: %.2f, Demul: %.2f\n",...
    mean(fft_time_in_function(subset)),mean(zf_time_in_function(subset)),...
    mean(demul_time_in_function(1200:frame_count)));

avg_rx_duration = mean(frame_duration_rx(subset));
avg_proc_duration = mean(frame_duration_demul(subset));
avg_zf_duration = mean(zf_duration(subset));
fprintf("In main thread: FFT of pilots done: %.2f, ZF done: %.2f, Demodulation done: %.2f, total: %.2f\n",...
    mean(fft_duration(subset)),avg_zf_duration,...
    mean(demul_duration(subset)),avg_proc_duration);
fprintf("RX duration: %.5f, Processing delay: %.5f\n", avg_rx_duration, avg_proc_duration-avg_rx_duration);

process_delay = demul_processed-rx_processed;
avg_proc_delay = mean(process_delay(subset));

[f_proc_delay,x_proc_delay] = ecdf(process_delay(subset));
x_set = x_proc_delay(f_proc_delay>0.99);
proc_delay_99th = x_set(1);

[f_zf_delay,x_zf_delay] = ecdf(zf_duration(subset));
x_set = x_zf_delay(f_zf_delay>0.99);
zf_delay_99th = x_set(1);

fprintf("Process delay 99th: %.2f, ZF delay 99th: %.3f\n", proc_delay_99th, zf_delay_99th);
fprintf("Process delay mean: %.2f, ZF delay mean: %.2f\n", avg_proc_delay, avg_zf_duration);

figure(1);clf;
% plot(fft_duration(1:frame_count),'LineWidth',2);
plot(fft_time_in_function(1:frame_count), 'LineWidth',2);
hold on;
plot(zf_time_in_function(1:frame_count), 'LineWidth',2);
hold on;
plot(demul_time_in_function(1:frame_count), 'LineWidth',2);
hold on;
plot(zf_duration(1:frame_count),'LineWidth',2);
set(gca,'FontSize',18);
grid on;
xlabel('Frame id');
ylabel('Duration (us)');
legend({'FFT in function','ZF in function','Demodulation in function','ZF'});

figure(2);clf;
plot(frame_duration_fft(1:frame_count),'LineWidth',2);
hold on;
plot(frame_duration_zf(1:frame_count),'LineWidth',2);
hold on;
plot(frame_duration_demul(1:frame_count),'LineWidth',2);
hold on;
plot(frame_duration_rx(1:frame_count),'LineWidth',2);
set(gca,'FontSize',18);
grid on;
xlabel('Frame id');
ylabel('Duration (us)');
legend({'FFT of pilots','ZF','Demodulation','RX'});



figure(4);clf;
plot(process_delay(1:frame_count),'LineWidth',2);
set(gca,'FontSize',18);
grid on;
xlabel('Frame id');
ylabel('Delay (us)');
title('Processing delay')

figure(5);clf;
h1 = cdfplot(process_delay(subset));
set(h1,'LineWidth',2);
set(gca,'FontSize',18);
grid on;
% xlabel('Frame id');
xlabel('Delay (us)');
title('Processing delay')

figure(7);clf;
plot(pilot_recv(1:end)-frame_start(1:end),'LineWidth',2);
set(gca,'FontSize',18);
grid on;
xlabel('Frame id');
ylabel('Time (us)');
title('Delay between main thread and rx thread')

figure(8);clf;
h1 = cdfplot(pilot_recv(subset)-frame_start(subset));
set(h1,'LineWidth',2);
set(gca,'FontSize',18);
grid on;
% xlabel('Frame id');
xlabel('Time (us)');
title('Delay between main thread and rx thread')

%%
fid = fopen('tx_result.txt');
temp = textscan(fid,"%f");
frame_ends=temp{1};
frame_duration_tx = diff(frame_ends);
avg_tx_duration = mean(frame_duration_tx(subset));
fprintf("In emulator: TX duration %.5f\n", avg_tx_duration);
figure(6);clf;
plot(frame_duration_tx(1:end),'LineWidth',2);
set(gca,'FontSize',18);
grid on;
xlabel('Frame id');
ylabel('Time (us)');
title('TX duration');

frame_duration_rx1 = diff(frame_start);
figure(3);clf;
h1=cdfplot(frame_duration_rx(subset));
hold on;
h2=cdfplot(frame_duration_demul(subset));
hold on;
h3=cdfplot(frame_duration_tx(subset));
set(h1,'LineWidth',2);
set(h2,'LineWidth',2);
set(h3,'LineWidth',2,'LineStyle', '--', 'Color', 'k');
grid on;
set(gca,'FontSize',18);
xlabel('Duration (us)');
legend({sprintf('RX duration (avg: %.2f us)',avg_rx_duration),...
    sprintf('Data processing duration (avg: %.2f us)', avg_proc_duration),...
    sprintf('TX duration in emulator (avg: %.2f us)', avg_tx_duration)},'Location','southeast');
title('CDF of frame duration and processing duration')

